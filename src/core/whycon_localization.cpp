
#include "whycode/core/whycon_localization.hpp"

#include <iostream>

#include "whycode/core/marker_detector.hpp"
#include "whycode/utils/whycon_config.h"

namespace xs = xsimd;

whycon::LocalizationSystem::LocalizationSystem(
    int _targets, int _width, int _height, const cv::Mat& _K, const cv::Mat& _dist_coeff,
    const whycon::DetectorParameters& parameters)
  : detector(_targets, _width, _height, parameters)
  , targets(_targets)
  , width(_width)
  , height(_height)
  , circle_diameter(parameters.outer_diameter)
  , identify_enabled_(parameters.identify)
  , id_bits_(parameters.id_bits)
  , id_samples_(parameters.id_samples)
  , hamming_distance_(parameters.hamming_distance)
  , image_gamma_(parameters.image_gamma)
  , ambiguity_noise_px_(parameters.ambiguity_noise_px)
  , min_marker_pixels_(parameters.min_marker_pixels)
  , diameter_ratio_correction_(parameters.diameter_ratio_correction)
{
    _K.copyTo(K);
    _dist_coeff.copyTo(dist_coeff);

    // Extract focal lengths and principal point from camera matrix
    camera_params.focal_length_x    = K.at<double>(0, 0);
    camera_params.focal_length_y    = K.at<double>(1, 1);
    camera_params.principal_point_x = K.at<double>(0, 2);
    camera_params.principal_point_y = K.at<double>(1, 2);

    camera_params.distortion_coeffs[0] = 1;
    for (int i = 0; i < 5; i++)
        camera_params.distortion_coeffs[i + 1] = dist_coeff.at<double>(i);

    // Only undistorPoints reads the map, and only for a camera with distortion
    has_distortion_ = cv::countNonZero(dist_coeff) > 0;
    if (has_distortion_)
    {
        precomputeUndistorMap();
    }

    std::cout.precision(30);

    // CNecklace Initialization
    if (identify_enabled_)
    {
        decoder_ = std::make_unique<CNecklace>(id_bits_, id_samples_, hamming_distance_);
    }

    // Initialize analysis data with correct code size
    for (auto& data : analysis_data_)
    {
        data.code.resize(id_bits_ * 4 + 1, '\0');
        if (data.signal.size() != static_cast<size_t>(id_samples_))
            data.signal.resize(id_samples_);
        if (data.smooth.size() != static_cast<size_t>(id_samples_))
            data.smooth.resize(id_samples_);
    }

    segment_width_ = id_samples_ / id_bits_ / 2;

    // Initialize the trig lookup table
    calcTrigLUT(trig_lut_samples_, id_samples_, id_samples_);
    calcTrigLUT(trig_lut_seg_width_, segment_width_, id_samples_);
}

whycon::LocalizationSystem::~LocalizationSystem() {}

bool whycon::LocalizationSystem::localizeMarkers(
    ImageHandler& image_handler, bool reset, DebugImageManager* debug_manager)
{
    return detector.detectMarkers(image_handler, reset, debug_manager);
}

void whycon::LocalizationSystem::estimateMarkerPose(
    const ImageHandler& image_handler, const whycon::MarkerDetector::Marker& outer_marker,
    const whycon::MarkerDetector::Marker& inner_marker, MarkerPose& result,
    DebugImageManager* debug_manager)
{
    result.tracking_id = outer_marker.tracking_id;  // Copy track id to the pose result

    // Refit the outline to sub-pixel edges, then both pose solutions of the OUTER ellipse
    refined_outer_ = outer_marker;
    refineOutline(image_handler, inner_marker, refined_outer_);
    calcEllipseCenters(refined_outer_, ellipse_centers_buffer_);
    const EllipseCenters& centers = ellipse_centers_buffer_;

    float log_odds = 0.0f;  // Of solution 0 over solution 1
    int   chosen   = 0;
    if (identify_enabled_)
    {
        log_odds =
            identifyAndWeighSolutions(image_handler, centers, refined_outer_, result, debug_manager);
        chosen = log_odds >= 0.0f ? 0 : 1;
        result.ambiguity_signal_px =
            std::hypot(centers.u[0] - centers.u[1], centers.v[0] - centers.v[1]);
    }
    else
    {
        // A WhyCon marker carries no code to weigh the solutions: its inner circle only orders them
        chosen                     = nearestSolution(centers, inner_marker);
        marker_angle_              = inner_marker.angle;
        result.ambiguity_signal_px = 0.0f;
        result.id_valid            = true;  // No ID to validate, so pose is considered valid
    }

    // Report this frame's favourite; AmbiguityResolver may swap in the other across frames
    const int other = 1 - chosen;
    result.pos      = cv::Vec3f(centers.t[chosen][0], centers.t[chosen][1], centers.t[chosen][2]);
    result.alt_pos  = cv::Vec3f(centers.t[other][0], centers.t[other][1], centers.t[other][2]);
    result.orientation            = orientationFromNormal(centers.n[chosen]);
    result.alt_orientation        = orientationFromNormal(centers.n[other]);
    result.ambiguity_llr          = std::abs(log_odds);
    result.orientation_confidence = 1.0f / (1.0f + std::exp(-result.ambiguity_llr));

    // Convert quaternion to Euler angles for convenience
    updateEulerAngles(result);
}

float whycon::LocalizationSystem::identifyAndWeighSolutions(
    const ImageHandler& image_handler, const EllipseCenters& ellipse_centers,
    const MarkerDetector::Marker& outer_marker, MarkerPose& pose, DebugImageManager* debug_manager)
{
    pose.ID       = -1;
    pose.id_valid = false;

    // The solutions' centres are under a pixel apart, so rings around the ellipse centre serve both
    SegmentParameters ring{ outer_marker.x,
                            outer_marker.y,
                            diameter_ratio_correction_ * outer_marker.m0,
                            diameter_ratio_correction_ * outer_marker.m1,
                            outer_marker.v0,
                            outer_marker.v1 };

    if (debug_manager && debug_manager->isEnabled())
    {
        for (int i = 0; i < 2; i++)
        {
            ellipse_processing_.solutions[i].segment   = ring;
            ellipse_processing_.solutions[i].segment.x = ellipse_centers.u[i];
            ellipse_processing_.solutions[i].segment.y = ellipse_centers.v[i];
            ellipse_processing_.solutions[i].u         = ellipse_centers.u[i];
            ellipse_processing_.solutions[i].v         = ellipse_centers.v[i];
        }
        drawSolutionDebugImage(image_handler, debug_manager);
    }

    if (!sampleRing(ring, image_handler, analysis_data_[0]))
    {
        return 0.0f;
    }

    // Collect the code's radial edges on the ID ring and on rings inside it
    setPlaneFrames(ellipse_centers);
    accumulateEdges(analysis_data_[0], 0);
    for (size_t r = 0; r < CUE_RING_RATIOS.size(); r++)
    {
        ring.m0 = CUE_RING_RATIOS[r] * outer_marker.m0;
        ring.m1 = CUE_RING_RATIOS[r] * outer_marker.m1;
        if (sampleRing(ring, image_handler, analysis_data_[1]))
        {
            accumulateEdges(analysis_data_[1], static_cast<int>(r) + 1);
        }
    }

    // Under the right solution the edges' plane angles fall on the segment grid. The residual is
    // their weighted circular variance about it, one grid offset per ring and polarity, which
    // absorbs blur widening the black or the white segments.
    std::array<double, 2> residual{};
    for (int k = 0; k < 2; k++)
    {
        for (int g = 0; g < CUE_GROUPS; g++)
        {
            residual[k] += 2.0 * (grid_weight_[k][g] - std::abs(grid_sum_[k][g]));
        }
    }
    // The edge noise in plane metres, scaled by the grid's phase gain (segments per turn)
    const double noise =
        2.0 * id_bits_ * ambiguity_noise_px_ * circle_diameter / (4.0 * outer_marker.m0);
    const float log_odds = static_cast<float>((residual[1] - residual[0]) / (2.0 * noise * noise));

    // Read the code around the favoured solution's projected centre, where the radial edges meet:
    // off it, the edges' phase on the ring, and with it the yaw, shift with the code's pattern
    const int favourite = log_odds >= 0.0f ? 0 : 1;

    // How far the code's edges spread about the segment grid (circular variance, 0 = on it)
    double grid_weight = 0.0;
    for (int g = 0; g < CUE_GROUPS; g++)
    {
        grid_weight += grid_weight_[favourite][g];
    }
    const double spread = grid_weight > 0.0 ? residual[favourite] / grid_weight : 2.0;
    ring.x              = ellipse_centers.u[favourite];
    ring.y              = ellipse_centers.v[favourite];
    ring.m0             = diameter_ratio_correction_ * outer_marker.m0;
    ring.m1             = diameter_ratio_correction_ * outer_marker.m1;
    SignalAnalysisData& code_ring =
        sampleRing(ring, image_handler, analysis_data_[1]) ? analysis_data_[1] : analysis_data_[0];

    readCode(code_ring);
    char     realCode[id_bits_ + 1];
    SDecoded decoded = decoder_->decode(
        code_ring.code.data(),
        realCode,
        code_ring.max_idx,
        outer_marker.v0,
        outer_marker.v1);
    marker_angle_ = decoded.angle;  // Store the marker angle for orientation calculation

    // Manchester coding gives each bit two opposite segments: a read that breaks it is garbled
    bool manchester = true;
    for (int b = 0; b < id_bits_; b++)
    {
        const int first = decoded.edgeIndex + 2 * b;
        manchester &= code_ring.code[first] != code_ring.code[(first + 1) % (2 * id_bits_)];
    }
    // A real code's edges sit on the grid; on a large outline, a spread this wide is clutter
    const bool regular = outer_marker.size < REGULAR_CODE_PIXELS || spread < MAX_CODE_SPREAD;
    pose.id_valid =
        decoded.id >= 0 && manchester && regular && outer_marker.size >= min_marker_pixels_;
    pose.ID = pose.id_valid ? decoded.id + 1 : -1;

    WHYCON_DEBUG("Decoded marker ID: " << pose.ID << " angle: " << marker_angle_);
    return log_odds;
}

// Performance improved upto 20x than scalar version
bool whycon::LocalizationSystem::computeEllipseCoordinates(
    int width, int height, const SegmentParameters& s, aligned_vec<float>& x_coords,
    aligned_vec<float>& y_coords)
{
    // SIMD calculation of coordinates
    using bf     = xs::batch<float>;
    const int VS = bf::size;  // SIMD Vector Size (8 for AVX2)

    // Broadcast constants to SIMD registers. computeSignal reads the 2x2 neighbourhood of each
    // sample, so the last column and row are out of bounds as a sample's origin.
    bf bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f);
    bf zeros(0.0f), width_b(static_cast<float>(width - 1)),
        height_b(static_cast<float>(height - 1));

    int a = 0;
    bf  co, si, dx, dy, x, y;

    // Process in SIMD Chunks - Performance boost 20x
    for (; a + VS <= id_samples_; a += VS)
    {
        // Load pre-computed cos/sin values from LUT
        co = xs::load_aligned(&trig_lut_samples_.cosv[a]);
        si = xs::load_aligned(&trig_lut_samples_.sinv[a]);

        // Calculate dx,dy offsets
        dx = two * (bm0 * co * bv0 + bm1 * si * bv1);
        dy = two * (bm0 * co * bv1 - bm1 * si * bv0);

        // Calculate final coordinates
        x = bx + dx;
        y = by + dy;

        // Store results in aligned arrays
        xs::store_aligned(&x_coords[a], x);
        xs::store_aligned(&y_coords[a], y);

        // Check bounds using vectorized comparison
        auto mask_x_lo = x < zeros;
        auto mask_x_hi = x >= width_b;
        auto mask_y_lo = y < zeros;
        auto mask_y_hi = y >= height_b;

        // If any point is out of bounds, set flag
        if (xs::any(mask_x_lo | mask_x_hi | mask_y_lo | mask_y_hi))
        {
            return false;
        }
    }

    // Handle remaining elements (tail) with scalar code
    for (; a < id_samples_; ++a)
    {
        float co = trig_lut_samples_.cosv[a];
        float si = trig_lut_samples_.sinv[a];

        float x = s.x + 2.0f * (s.m0 * co * s.v0 + s.m1 * si * s.v1);
        float y = s.y + 2.0f * (s.m0 * co * s.v1 - s.m1 * si * s.v0);

        x_coords[a] = x;
        y_coords[a] = y;

        if (x < 0 || x >= width - 1 || y < 0 || y >= height - 1)
        {
            return false;
        }
    }
    return true;
}

namespace
{
// Bilinear samples of the summed colour channels: the first three, or the one grey channel
template <int CHANNELS>
void sampleSignal(
    const unsigned char* image_data, int width, int step, int id_samples, const float* x_coords,
    const float* y_coords, float* signal)
{
    const auto sum = [](const unsigned char* pixel) {
        float s = pixel[0];
        for (int c = 1; c < CHANNELS; c++)
        {
            s += pixel[c];
        }
        return s;
    };
    for (int a = 0; a < id_samples; ++a)
    {
        const float xf = x_coords[a];
        const float yf = y_coords[a];
        const int   px = static_cast<int>(xf);
        const int   py = static_cast<int>(yf);
        const float gx = xf - px;
        const float gy = yf - py;

        // Base rows
        const unsigned char* row0 = image_data + py * width * step;
        const unsigned char* row1 = row0 + width * step;
        const int            p0   = px * step;

        // Sum the channels at the 4 bilinear corners
        const float S00 = sum(row0 + p0);
        const float S10 = sum(row0 + p0 + step);
        const float S01 = sum(row1 + p0);
        const float S11 = sum(row1 + p0 + step);

        // Two horizontal lerps + one vertical (FMA)
        const float row0_interp = std::fmaf(S10 - S00, gx, S00);
        const float row1_interp = std::fmaf(S11 - S01, gx, S01);
        signal[a]               = std::fmaf(row1_interp - row0_interp, gy, row0_interp);
    }
}
}  // namespace

void whycon::LocalizationSystem::computeSignal(
    const unsigned char* image_data, int width, int id_samples,
    whycon::LocalizationSystem::SignalAnalysisData& analysis_data,
    const whycon::ImageHandler&                     image_handler)
{
    const int step = image_handler.getBPP();
    if (step == 1)
    {
        sampleSignal<1>(
            image_data,
            width,
            step,
            id_samples,
            analysis_data.x_coords.data(),
            analysis_data.y_coords.data(),
            analysis_data.signal.data());
    }
    else
    {
        sampleSignal<3>(
            image_data,
            width,
            step,
            id_samples,
            analysis_data.x_coords.data(),
            analysis_data.y_coords.data(),
            analysis_data.signal.data());
    }
}

void whycon::LocalizationSystem::binarizeSignal(
    whycon::LocalizationSystem::SignalAnalysisData& data, int n)
{
    using bf     = xs::batch<float>;
    const int VS = bf::size;

    int i = 0;
    // Accumulators for partial sums (mean calculation)
    bf acc0(0.f), acc1(0.f), acc2(0.f), acc3(0.f);

    // Unrolled SIMD loop: accumulate signal values in batches of 4*VS
    for (; i + 4 * VS <= n; i += 4 * VS)
    {
        acc0 += xs::load_aligned(&data.signal[i + 0 * VS]);
        acc1 += xs::load_aligned(&data.signal[i + 1 * VS]);
        acc2 += xs::load_aligned(&data.signal[i + 2 * VS]);
        acc3 += xs::load_aligned(&data.signal[i + 3 * VS]);
    }
    bf acc = acc0 + acc1 + acc2 + acc3;

    // Handle remaining full SIMD batches
    for (; i + VS <= n; i += VS)
        acc += xs::load_aligned(&data.signal[i]);
    float sum = xs::reduce_add(acc);

    // Handle any leftover elements (tail) with scalar code
    for (; i < n; ++i)
        sum += data.signal[i];

    // Compute average value of the signal
    float avg = sum / float(n);
    data.mean = avg;
    const bf avgv(avg);  // Broadcast average to SIMD batch

    i = 0;
    // SIMD binarization: process in batches of 4*VS
    for (; i + 4 * VS <= n; i += 4 * VS)
    {
        bf v0 = xs::load_aligned(&data.signal[i + 0 * VS]);
        bf v1 = xs::load_aligned(&data.signal[i + 1 * VS]);
        bf v2 = xs::load_aligned(&data.signal[i + 2 * VS]);
        bf v3 = xs::load_aligned(&data.signal[i + 3 * VS]);
        // Store 1.0f if signal > avg, else 0.0f
        xs::store_aligned(&data.smooth[i + 0 * VS], xs::select(v0 > avgv, bf(1.f), bf(0.f)));
        xs::store_aligned(&data.smooth[i + 1 * VS], xs::select(v1 > avgv, bf(1.f), bf(0.f)));
        xs::store_aligned(&data.smooth[i + 2 * VS], xs::select(v2 > avgv, bf(1.f), bf(0.f)));
        xs::store_aligned(&data.smooth[i + 3 * VS], xs::select(v3 > avgv, bf(1.f), bf(0.f)));
    }
    // Handle remaining full SIMD batches
    for (; i + VS <= n; i += VS)
    {
        bf v = xs::load_aligned(&data.signal[i]);
        xs::store_aligned(&data.smooth[i], xs::select(v > avgv, bf(1.f), bf(0.f)));
    }
    // Handle any leftover elements (tail) with scalar code
    for (; i < n; ++i)
        data.smooth[i] = (data.signal[i] > avg) ? 1.f : 0.f;
}

void whycon::LocalizationSystem::refineOutline(
    const ImageHandler& image_handler, const MarkerDetector::Marker& inner,
    MarkerDetector::Marker& outer) const
{
    const cv::Mat& grey = image_handler.getGray();
    const float    threshold =
        static_cast<float>(MarkerDetector::edgeThreshold(outer.mean, inner.mean, image_gamma_));
    const float major = 2.0f * outer.m0, minor = 2.0f * outer.m1;  // Semi-axes (px)
    if (minor <= 0.0f)
    {
        return;
    }
    const float scale = 1.0f / major;  // Fit in units of the major semi-axis, for conditioning

    // Search from inside the black ring, never past its 0.4 R width, to outside the outline
    const float inside = std::min(OUTLINE_INSIDE_PX, 0.3f * minor);
    const int   steps  = static_cast<int>((inside + OUTLINE_OUTSIDE_PX) / OUTLINE_STEP_PX) + 1;

    cv::Matx<double, 5, 5> normal_matrix = cv::Matx<double, 5, 5>::zeros();
    cv::Vec<double, 5>     rhs;
    int                    points = 0;
    for (int k = 0; k < OUTLINE_RAYS; k++)
    {
        const float alpha = 2.0f * static_cast<float>(M_PI) * k / OUTLINE_RAYS;
        const float c = std::cos(alpha), s = std::sin(alpha);
        // A point of the ellipse and its outward normal
        const float px = outer.x + major * c * outer.v0 + minor * s * outer.v1;
        const float py = outer.y + major * c * outer.v1 - minor * s * outer.v0;
        float       nx = c / major * outer.v0 + s / minor * outer.v1;
        float       ny = c / major * outer.v1 - s / minor * outer.v0;
        const float n  = std::hypot(nx, ny);
        nx /= n;
        ny /= n;

        // The first dark-to-light crossing going outwards
        bool  below = false, found = false;
        float prev_t = 0.0f, prev_v = 0.0f, edge_t = 0.0f;
        for (int j = 0; j < steps; j++)
        {
            const float t = -inside + j * OUTLINE_STEP_PX;
            const float x = px + t * nx, y = py + t * ny;
            if (x < 0.0f || y < 0.0f || x >= grey.cols - 1 || y >= grey.rows - 1)
            {
                break;
            }
            const int    ix = static_cast<int>(x), iy = static_cast<int>(y);
            const float  gx = x - ix, gy = y - iy;
            const uchar* r0 = grey.ptr<uchar>(iy) + ix;
            const uchar* r1 = r0 + grey.step;
            const float  v =
                (1 - gy) * ((1 - gx) * r0[0] + gx * r0[1]) + gy * ((1 - gx) * r1[0] + gx * r1[1]);
            if (v < threshold)
            {
                below = true;
            }
            else if (below)
            {
                edge_t = prev_t + (threshold - prev_v) / (v - prev_v) * OUTLINE_STEP_PX;
                found  = true;
                break;
            }
            prev_t = t;
            prev_v = v;
        }
        if (!found)
        {
            continue;
        }

        // Conic through the edge points, relative to the moment centre: ax2+bxy+cy2+dx+ey = 1
        const double             ex = (px + edge_t * nx - outer.x) * scale;
        const double             ey = (py + edge_t * ny - outer.y) * scale;
        const cv::Vec<double, 5> row(ex * ex, ex * ey, ey * ey, ex, ey);
        normal_matrix += row * row.t();
        rhs += row;
        points++;
    }
    if (points < MIN_OUTLINE_POINTS)
    {
        return;
    }

    // Cholesky on stack copies; a failure returns zeros, which fail the ellipse test below
    const cv::Vec<double, 5> conic = normal_matrix.solve(rhs, cv::DECOMP_CHOLESKY);
    // Centre, then the quadratic form's eigenvectors give the axes
    const double a = conic[0], b = conic[1] / 2.0, c = conic[2];
    const double det = a * c - b * b;
    if (det <= 0.0)
    {
        return;  // Not an ellipse
    }
    const double cx    = (b * conic[4] / 2.0 - c * conic[3] / 2.0) / det;
    const double cy    = (b * conic[3] / 2.0 - a * conic[4] / 2.0) / det;
    const double level = 1.0 + a * cx * cx + 2.0 * b * cx * cy + c * cy * cy;
    const double half = 0.5 * (a + c), root = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
    const double small = half - root, large = half + root;  // Eigenvalues: major, minor axis
    if (small <= 0.0 || level <= 0.0)
    {
        return;
    }
    // The refit sets the centre and the shape; the size stays the moments', whose area is
    // calibrated against range (the crossing sits ~0.1 px outside the binarised outline)
    const double size = std::sqrt(major * minor * scale * scale * std::sqrt(small * large) / level);
    const double new_major = size * std::sqrt(level / small) / scale;
    const double new_minor = size * std::sqrt(level / large) / scale;
    // Major axis direction: eigenvector of the smaller eigenvalue
    double vx = b, vy = small - a;
    if (std::abs(vx) + std::abs(vy) < 1e-12)
    {
        vx = 1.0;
        vy = 0.0;
    }
    const double vn = std::hypot(vx, vy);
    vx /= vn;
    vy /= vn;
    if (vx * outer.v0 + vy * outer.v1 < 0.0)
    {
        vx = -vx;
        vy = -vy;
    }

    // A refit far from the moments means stray edges: keep the moment ellipse
    const double shift = std::hypot(cx, cy) / scale;
    if (shift > 1.0 || std::abs(new_major / major - 1.0) > 0.15 ||
        std::abs(new_minor / minor - 1.0) > 0.15)
    {
        return;
    }
    outer.x += static_cast<float>(cx / scale);
    outer.y += static_cast<float>(cy / scale);
    outer.m0 = static_cast<float>(new_major / 2.0);
    outer.m1 = static_cast<float>(new_minor / 2.0);
    outer.v0 = static_cast<float>(vx);
    outer.v1 = static_cast<float>(vy);
}

bool whycon::LocalizationSystem::sampleRing(
    const SegmentParameters& ring, const ImageHandler& image_handler, SignalAnalysisData& data)
{
    if (!computeEllipseCoordinates(width, height, ring, data.x_coords, data.y_coords))
    {
        return false;
    }
    computeSignal(image_handler.getData(), width, id_samples_, data, image_handler);
    binarizeSignal(data, id_samples_);
    return true;
}

void whycon::LocalizationSystem::readCode(SignalAnalysisData& data) const
{
    // The edges' mean phase on the segment grid places the segment boundaries
    float sx = 0.0f, sy = 0.0f;
    if (data.smooth[id_samples_ - 1] != data.smooth[0])
    {
        sx = 1.0f;
    }
    for (int a = 1; a < id_samples_; a++)
    {
        if (data.smooth[a] != data.smooth[a - 1])
        {
            sx += trig_lut_seg_width_.cosv[a];
            sy += trig_lut_seg_width_.sinv[a];
        }
    }
    data.max_idx = atan2(sy, sx) / 2 / M_PI * segment_width_ + segment_width_ / 2;

    // Sample each segment at its centre
    for (int a = 0; a < id_bits_ * 2; a++)
    {
        data.code[a] = data.smooth[(data.max_idx + a * segment_width_) % id_samples_] + '0';
    }
    data.code[id_bits_ * 2] = 0;
}

void whycon::LocalizationSystem::setPlaneFrames(const EllipseCenters& centers) const
{
    for (int k = 0; k < 2; k++)
    {
        PlaneFrame& f = plane_frames_[k];
        f.n           = cv::Vec3d(centers.n[k][0], centers.n[k][1], centers.n[k][2]);
        // Positions are kept in the body frame (x forward, y left, z up): back to optical
        f.t = cv::Vec3d(-centers.t[k][1], -centers.t[k][2], centers.t[k][0]);
        // Any in-plane axes do: the grid fit leaves the code's rotation free
        const cv::Vec3d helper = std::abs(f.n[0]) < 0.9 ? cv::Vec3d(1, 0, 0) : cv::Vec3d(0, 1, 0);
        f.e1                   = cv::normalize(f.n.cross(helper));
        f.e2                   = f.n.cross(f.e1);
        f.nt                   = f.n.dot(f.t);
        grid_sum_[k].fill({});
        grid_weight_[k].fill(0.0);
    }
}

void whycon::LocalizationSystem::accumulateEdges(const SignalAnalysisData& data, int ring_index) const
{
    const int segments = 2 * id_bits_;
    for (int a = 0; a < id_samples_; a++)
    {
        const int b = a + 1 == id_samples_ ? 0 : a + 1;
        if (data.smooth[a] == data.smooth[b])
        {
            continue;
        }

        // The ray through the sub-sample crossing of the threshold
        const float frac = (data.mean - data.signal[a]) / (data.signal[b] - data.signal[a]);
        double      x, y;
        undistorPoints(
            data.x_coords[a] + frac * (data.x_coords[b] - data.x_coords[a]),
            data.y_coords[a] + frac * (data.y_coords[b] - data.y_coords[a]),
            x,
            y);
        const cv::Vec3d ray(x, y, 1.0);
        const int       group = 2 * ring_index + (data.smooth[b] > data.smooth[a] ? 1 : 0);

        for (int k = 0; k < 2; k++)
        {
            const PlaneFrame& f     = plane_frames_[k];
            const double      denom = f.n.dot(ray);
            if (std::abs(denom) < 1e-9)
            {
                continue;
            }
            // The edge's point on this solution's plane, relative to the marker centre
            const cv::Vec3d            d = (f.nt / denom) * ray - f.t;
            const std::complex<double> p(f.e1.dot(d), f.e2.dot(d));
            const double               w = std::norm(p);
            if (w <= 0.0)
            {
                continue;
            }
            // w * exp(i * segments * angle), the power by squaring
            std::complex<double> z = p / std::sqrt(w), zk(1.0, 0.0);
            for (int e = segments; e > 0; e >>= 1)
            {
                if (e & 1)
                {
                    zk *= z;
                }
                z *= z;
            }
            grid_sum_[k][group] += w * zk;
            grid_weight_[k][group] += w;
        }
    }
}

void whycon::LocalizationSystem::calcEllipseCenters(
    const whycon::MarkerDetector::Marker& circle, EllipseCenters& result) const
{
    // check to ensure ellipse parameters are valid
    if (fabs(circle.m0) < 0.000001 || fabs(circle.m1) < 0.000001)
    {
        WHYCON_ERROR(
            "Invalid ellipse parameters: m0=" << circle.m0 << ", m1=" << circle.m1
                                              << " - cannot calculate pose");
        result = EllipseCenters();  // Reset result
        return;
    }

    // Use the pre-allocated cache
    auto& cache = pose_calc_cache_;

    // Transform the center to cononical camera coordinates
    undistorPoints(circle.x, circle.y, cache.x, cache.y);

    // Calculate the major axis
    // Endpoints in image coordinates
    cache.sx1 = circle.x + circle.v0 * circle.m0 * 2;
    cache.sx2 = circle.x - circle.v0 * circle.m0 * 2;
    cache.sy1 = circle.y + circle.v1 * circle.m0 * 2;
    cache.sy2 = circle.y - circle.v1 * circle.m0 * 2;

    // Endpoints in canonical camera coordinates
    undistorPoints(cache.sx1, cache.sy1, cache.x1, cache.y1);
    undistorPoints(cache.sx2, cache.sy2, cache.x2, cache.y2);

    // Compute the length of the major axis in normalized coordinates
    cache.major = sqrt(
                      (cache.x1 - cache.x2) * (cache.x1 - cache.x2) +
                      (cache.y1 - cache.y2) * (cache.y1 - cache.y2)) /
                  2.0;

    // Compute direction vector for the major axis
    cache.v0 = (cache.x2 - cache.x1) / cache.major / 2.0;
    cache.v1 = (cache.y2 - cache.y1) / cache.major / 2.0;

    // Calculate the minor axis
    // Endpoints in image coordinates
    cache.sx1 = circle.x + circle.v1 * circle.m1 * 2;
    cache.sx2 = circle.x - circle.v1 * circle.m1 * 2;
    cache.sy1 = circle.y - circle.v0 * circle.m1 * 2;
    cache.sy2 = circle.y + circle.v0 * circle.m1 * 2;

    // Endpoints in canonical camera coordinates
    undistorPoints(cache.sx1, cache.sy1, cache.x1, cache.y1);
    undistorPoints(cache.sx2, cache.sy2, cache.x2, cache.y2);

    // Compute the length of the minor axis in normalized coordinates
    cache.minor = sqrt(
                      (cache.x1 - cache.x2) * (cache.x1 - cache.x2) +
                      (cache.y1 - cache.y2) * (cache.y1 - cache.y2)) /
                  2.0;

    // Construct the conic (ellipse) equation coefficients in normalized coordinates
    cache.a = cache.v0 * cache.v0 / (cache.major * cache.major) +
              cache.v1 * cache.v1 / (cache.minor * cache.minor);
    cache.b = cache.v0 * cache.v1 *
              (1.0 / (cache.major * cache.major) - 1.0 / (cache.minor * cache.minor));
    cache.c = cache.v0 * cache.v0 / (cache.minor * cache.minor) +
              cache.v1 * cache.v1 / (cache.major * cache.major);
    cache.d = (-cache.x * cache.a - cache.b * cache.y);
    cache.e = (-cache.y * cache.c - cache.b * cache.x);
    cache.f =
        (cache.a * cache.x * cache.x + cache.c * cache.y * cache.y +
         2.0 * cache.b * cache.x * cache.y - 1.0);

    // Matrix conic coefficients
    cache.conic_matrix_data[0] = cache.a;
    cache.conic_matrix_data[1] = cache.b;
    cache.conic_matrix_data[2] = cache.d;
    cache.conic_matrix_data[3] = cache.b;
    cache.conic_matrix_data[4] = cache.c;
    cache.conic_matrix_data[5] = cache.e;
    cache.conic_matrix_data[6] = cache.d;
    cache.conic_matrix_data[7] = cache.e;
    cache.conic_matrix_data[8] = cache.f;

    // Calculate eigenvalues and vectors to get the 3D solutions
    return calcEigen(result);
}

void whycon::LocalizationSystem::calcEigen(EllipseCenters& result) const
{
    // Use the pre-allocated cache instead of local variables
    auto& cache = pose_calc_cache_;

    cv::Matx33d mat_data(cache.conic_matrix_data);

    // Compute eigenvalues and eigenvectors of the conic matrix
    cv::eigen(mat_data, cache.eigenvalues, cache.eigenvectors);

    // Extract eigenvalues
    cache.L0 = cache.eigenvalues.at<double>(0);
    cache.L1 = cache.eigenvalues.at<double>(1);
    cache.L2 = cache.eigenvalues.at<double>(2);

    // Eigenvectors - select appropriate indices
    static constexpr int V0 = 0;  // First eigenvector
    static constexpr int V2 = 2;  // Second eigenvector

    // Compute partial results for possible solutions
    cache.c0  = sqrt((cache.L0 - cache.L1) / (cache.L0 - cache.L2));
    cache.c0x = cache.c0 * cache.eigenvectors.at<double>(V0, 0);
    cache.c0y = cache.c0 * cache.eigenvectors.at<double>(V0, 1);
    cache.c0z = cache.c0 * cache.eigenvectors.at<double>(V0, 2);

    cache.c1  = sqrt((cache.L1 - cache.L2) / (cache.L0 - cache.L2));
    cache.c1x = cache.c1 * cache.eigenvectors.at<double>(V2, 0);
    cache.c1y = cache.c1 * cache.eigenvectors.at<double>(V2, 1);
    cache.c1z = cache.c1 * cache.eigenvectors.at<double>(V2, 2);

    cache.c2 = circle_diameter / sqrt(-cache.L0 * cache.L2) / 2.0;

    // Arrays of sign combinations for generating up to 8 possible solutions
    static constexpr float s0[8] = { 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0 };
    static constexpr float s1[8] = { 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0 };
    static constexpr float s2[8] = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };

    int valid_solutions = 0;

    // Iterate over all sign combinations to find physically valid solutions
    for (int i = 0; i < 8; i++)
    {
        cache.n2 = s0[i] * cache.c0z + s1[i] * cache.c1z;
        cache.t2 = s2[i] * cache.c2 * (s0[i] * cache.L2 * cache.c0z + s1[i] * cache.L0 * cache.c1z);

        // Only solutions with marker in front of camera (n2 > 0, t2 > 0)
        if (cache.n2 > 0 && cache.t2 > 0)
        {
            cache.n0 = s0[i] * cache.c0x + s1[i] * cache.c1x;
            cache.n1 = s0[i] * cache.c0y + s1[i] * cache.c1y;

            cache.t0 =
                s2[i] * cache.c2 * (s0[i] * cache.L2 * cache.c0x + s1[i] * cache.L0 * cache.c1x);
            cache.t1 =
                s2[i] * cache.c2 * (s0[i] * cache.L2 * cache.c0y + s1[i] * cache.L0 * cache.c1y);

            // Store normal vector
            result.n[valid_solutions][0] = cache.n0;
            result.n[valid_solutions][1] = cache.n1;
            result.n[valid_solutions][2] = cache.n2;

            // Store position vector (camera coordinate system)
            // t2 ~ z -> x, -t0 ~ -x -> y, -t1 ~ -y -> z
            result.t[valid_solutions][0] = cache.t2;
            result.t[valid_solutions][1] = -cache.t0;
            result.t[valid_solutions][2] = -cache.t1;

            // Store reprojected image coordinates; without distortion the pinhole model is exact
            // and, unlike cv::projectPoints, allocates nothing
            if (has_distortion_)
            {
                cache.pt3d[0] = cv::Point3f(cache.t0, cache.t1, cache.t2);
                cv::projectPoints(
                    cache.pt3d,
                    cv::Mat::zeros(3, 1, CV_64F),
                    cv::Mat::zeros(3, 1, CV_64F),
                    K,
                    dist_coeff,
                    cache.pt2d);
                result.u[valid_solutions] = cache.pt2d[0].x;
                result.v[valid_solutions] = cache.pt2d[0].y;
            }
            else
            {
                result.u[valid_solutions] = camera_params.focal_length_x * cache.t0 / cache.t2 +
                                            camera_params.principal_point_x;
                result.v[valid_solutions] = camera_params.focal_length_y * cache.t1 / cache.t2 +
                                            camera_params.principal_point_y;
            }
            WHYCON_INFO(
                "Ellipse center: " << result.u[valid_solutions] << ", "
                                   << result.v[valid_solutions]);
            valid_solutions++;

            // We only need two solutions
            if (valid_solutions >= 2)
                break;
        }
    }
}

int whycon::LocalizationSystem::nearestSolution(
    const EllipseCenters& centers, const whycon::MarkerDetector::Marker& marker) const
{
    // The solution closer to the inner circle center is more likely correct
    const float dist0 = std::hypot(marker.x - centers.u[0], marker.y - centers.v[0]);
    const float dist1 = std::hypot(marker.x - centers.u[1], marker.y - centers.v[1]);
    return dist0 < dist1 ? 0 : 1;
}

void whycon::LocalizationSystem::calcTrigLUT(TrigLUTAligned& lut, int value, int size)
{
    if (lut.n == size && !lut.cosv.empty())
        return;
    lut.n = size;
    lut.cosv.resize(size);
    lut.sinv.resize(size);
    const float k = 2.0f * float(M_PI) / value;
#pragma GCC unroll 4
    for (int a = 0; a < size; ++a)
    {
        float ang   = a * k;
        lut.cosv[a] = cosf(ang);
        lut.sinv[a] = sinf(ang);
    }
}

whycon::Quaternion
whycon::LocalizationSystem::orientationFromNormal(const std::array<float, 3>& normal) const
{
    // Compute rotation from initial marker normal to detected normal
    cv::Vec3f initial_norm(1.0, 0.0, 0.0);
    cv::Vec3f final_norm(normal[2], -normal[0], -normal[1]);  // Adapt coordinate system
    cv::normalize(final_norm, final_norm);

    // Calculate rotation axis as cross product
    cv::Vec3f axis_vec = final_norm.cross(initial_norm);
    cv::normalize(axis_vec, axis_vec);

    // Calculate rotation angle between normals
    float dot_pro   = final_norm.dot(initial_norm);
    float rot_angle = -std::acos(dot_pro);

    // Create first quaternion from axis-angle
    Quaternion q1 = Quaternion::fromAxisAngle(axis_vec, rot_angle);
    q1.normalize();

    // Create quaternion for rotation around the marker's normal using its angle
    float new_angle = marker_angle_;
    if (new_angle > M_PI)
        new_angle = new_angle - 2 * M_PI;

    Quaternion q2 = Quaternion::fromAxisAngle(final_norm, new_angle);
    q2.normalize();

    // Combine the two quaternions
    Quaternion q3 = q2 * q1;
    q3.normalize();

    // Apply an additional z-axis rotation to match the original implementation
    Quaternion z_rot;
    z_rot.x = 0.0f;
    z_rot.y = 0.0f;
    z_rot.z = 1.0f;
    z_rot.w = 0.0f;

    Quaternion orientation = q3 * z_rot;
    orientation.normalize();
    return orientation;
}

void whycon::LocalizationSystem::updateEulerAngles(MarkerPose& pose) const
{
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    cv::Vec3f euler = pose.orientation.toEulerAngles();

    // Store in both formats for convenience
    pose.rot   = euler;
    pose.roll  = euler[0];
    pose.pitch = euler[1];
    pose.yaw   = euler[2];
}

const whycon::MarkerDetector::Marker& whycon::LocalizationSystem::getMarkerByID(int id)
{
    return detector.circles[id];
}

const whycon::MarkerDetector::Marker& whycon::LocalizationSystem::getOuterMarkerByID(int id)
{
    return detector.outer_circles[id];
}

/* normalize coordinates: move from image to canonical and remove distortion */
void whycon::LocalizationSystem::undistorPoints(
    double x_in, double y_in, double& x_out, double& y_out) const
{
    // Without lens distortion the pinhole normalisation is exact
    if (!has_distortion_)
    {
        x_out = (x_in - camera_params.principal_point_x) / camera_params.focal_length_x;
        y_out = (y_in - camera_params.principal_point_y) / camera_params.focal_length_y;
        return;
    }

    // Inside the image, interpolate the map precomputed at start-up
    if (x_in >= 0.0 && y_in >= 0.0 && x_in < width - 1 && y_in < height - 1)
    {
        const int        ix  = static_cast<int>(x_in);
        const int        iy  = static_cast<int>(y_in);
        const double     gx  = x_in - ix;
        const double     gy  = y_in - iy;
        const cv::Vec2f& p00 = undistort_map.at<cv::Vec2f>(iy, ix);
        const cv::Vec2f& p10 = undistort_map.at<cv::Vec2f>(iy, ix + 1);
        const cv::Vec2f& p01 = undistort_map.at<cv::Vec2f>(iy + 1, ix);
        const cv::Vec2f& p11 = undistort_map.at<cv::Vec2f>(iy + 1, ix + 1);
        x_out =
            (1 - gy) * ((1 - gx) * p00[0] + gx * p10[0]) + gy * ((1 - gx) * p01[0] + gx * p11[0]);
        y_out =
            (1 - gy) * ((1 - gx) * p00[1] + gx * p10[1]) + gy * ((1 - gx) * p01[1] + gx * p11[1]);
        return;
    }

    // Off the image (an axis end of a marker at the border): exact, but allocates
    std::vector<cv::Vec2d> src(1, cv::Vec2d(x_in, y_in));
    std::vector<cv::Vec2d> dst(1);
    cv::undistortPoints(src, dst, K, dist_coeff);
    x_out = dst[0](0);
    y_out = dst[0](1);
}

void whycon::LocalizationSystem::precomputeUndistorMap(void)
{
    // Precompute a map from distorted image coordinates to undistorted normalized camera coordinates.
    // This speeds up repeated undistortion operations by caching the mapping for each pixel.
    undistort_map.create(height, width, CV_32FC2);
    for (int i = 0; i < height; i++)
    {
        std::vector<cv::Vec2f> coords_in(width);
        for (int j = 0; j < width; j++)
            coords_in[j] = cv::Vec2f(j, i);  // TODO: reverse y? add 0.5?

        undistortPoints(coords_in, undistort_map.row(i), K, dist_coeff);
    }
}

void whycon::LocalizationSystem::drawSolutionDebugImage(
    const ImageHandler& image_handler, DebugImageManager* debug_manager) const
{
    // Draw these two solutions of ellipses on the image for debugging
    // Create debug visualization for marker solutions
    cv::Mat debug_image;
    image_handler.copyColour(debug_image);
    cv::circle(
        debug_image,
        cv::Point2f(
            ellipse_processing_.solutions[0].segment.x,
            ellipse_processing_.solutions[0].segment.y),
        1,
        cv::Scalar(255, 0, 0),
        2);
    cv::circle(
        debug_image,
        cv::Point2f(
            ellipse_processing_.solutions[1].segment.x,
            ellipse_processing_.solutions[1].segment.y),
        1,
        cv::Scalar(0, 255, 0),
        2);

    // Draw ellipse for the first solution using the given function
    for (float e = 0; e < 2 * M_PI; e += 0.01)
    {
        float fx = ellipse_processing_.solutions[0].segment.x +
                   cos(e) * ellipse_processing_.solutions[0].segment.v0 *
                       ellipse_processing_.solutions[0].segment.m0 * 2 +
                   ellipse_processing_.solutions[0].segment.v1 *
                       ellipse_processing_.solutions[0].segment.m1 * 2 * sin(e);
        float fy = ellipse_processing_.solutions[0].segment.y +
                   cos(e) * ellipse_processing_.solutions[0].segment.v1 *
                       ellipse_processing_.solutions[0].segment.m0 * 2 -
                   ellipse_processing_.solutions[0].segment.v0 *
                       ellipse_processing_.solutions[0].segment.m1 * 2 * sin(e);
        int fxi = static_cast<int>(fx + 0.5);
        int fyi = static_cast<int>(fy + 0.5);
        if (fxi >= 0 && fxi < debug_image.cols && fyi >= 0 && fyi < debug_image.rows)
            debug_image.at<cv::Vec3b>(fyi, fxi) = cv::Vec3b(255, 0, 0);
    }
    // Draw ellipse for the second solution using the given function
    for (float e = 0; e < 2 * M_PI; e += 0.01)
    {
        float fx = ellipse_processing_.solutions[1].segment.x +
                   cos(e) * ellipse_processing_.solutions[1].segment.v0 *
                       ellipse_processing_.solutions[1].segment.m0 * 2 +
                   ellipse_processing_.solutions[1].segment.v1 *
                       ellipse_processing_.solutions[1].segment.m1 * 2 * sin(e);
        float fy = ellipse_processing_.solutions[1].segment.y +
                   cos(e) * ellipse_processing_.solutions[1].segment.v1 *
                       ellipse_processing_.solutions[1].segment.m0 * 2 -
                   ellipse_processing_.solutions[1].segment.v0 *
                       ellipse_processing_.solutions[1].segment.m1 * 2 * sin(e);
        int fxi = static_cast<int>(fx + 0.5);
        int fyi = static_cast<int>(fy + 0.5);
        if (fxi >= 0 && fxi < debug_image.cols && fyi >= 0 && fyi < debug_image.rows)
            debug_image.at<cv::Vec3b>(fyi, fxi) = cv::Vec3b(0, 255, 0);
    }

    // Put text on the image
    cv::putText(
        debug_image,
        "Solution 1",
        cv::Point2f(
            ellipse_processing_.solutions[0].segment.x,
            ellipse_processing_.solutions[0].segment.y - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 0, 0),
        1);
    cv::putText(
        debug_image,
        "Solution 2",
        cv::Point2f(
            ellipse_processing_.solutions[1].segment.x,
            ellipse_processing_.solutions[1].segment.y - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 0),
        1);

    debug_manager->addDebugImage(debug_image, "Marker Solutions");
}