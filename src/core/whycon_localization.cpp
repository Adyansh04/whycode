
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
  , variance_threshold_(parameters.variance_threshold)
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

    precomputeUndistorMap();

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

    // Calculate ellipse centers with both possible solutions using OUTER marker
    calcEllipseCenters(outer_marker, ellipse_centers_buffer_);

    // Resolve ambiguity using inner marker for comparison
    if (identify_enabled_)
    {
        result.id_valid = processMarkerAmbiguityAndIdentify(
            image_handler,
            result,
            ellipse_centers_buffer_,
            outer_marker,
            debug_manager);
    }
    else
    {
        resolveAmbiguity(result, ellipse_centers_buffer_, inner_marker);
        result.id_valid = true;  // No ID to validate, so pose is considered valid
    }

    // Calculate full orientation using outer marker's angle
    calculateOrientation(result);

    // Convert quaternion to Euler angles for convenience
    updateEulerAngles(result);
}

bool whycon::LocalizationSystem::processMarkerAmbiguityAndIdentify(
    const ImageHandler& image_handler, MarkerPose& pose, const EllipseCenters& ellipse_centers,
    const MarkerDetector::Marker& outer_marker, DebugImageManager* debug_manager)
{
    unsigned char* image_data = image_handler.getData();
    int            width      = image_handler.getWidth();
    int            height     = image_handler.getHeight();

    // Set up ellipse processing data for both solutions
    for (int i = 0; i < 2; i++)
    {
        ellipse_processing_.solutions[i].segment.x  = ellipse_centers.u[i];
        ellipse_processing_.solutions[i].segment.y  = ellipse_centers.v[i];
        ellipse_processing_.solutions[i].segment.m0 = diameter_ratio_correction_ * outer_marker.m0;
        ellipse_processing_.solutions[i].segment.m1 = diameter_ratio_correction_ * outer_marker.m1;
        ellipse_processing_.solutions[i].segment.v0 = outer_marker.v0;
        ellipse_processing_.solutions[i].segment.v1 = outer_marker.v1;

        ellipse_processing_.solutions[i].u = ellipse_centers.u[i];
        ellipse_processing_.solutions[i].v = ellipse_centers.v[i];
    }
    // Draw these two solutions of ellipses on the image for debugging
    if (debug_manager && debug_manager->isEnabled())
        drawSolutionDebugImage(image_handler, debug_manager);

    // Process both solutions using the new function
    for (int i = 0; i < 2; i++)
    {
        if (!processSingleSolution(i, image_handler, width, height, image_data))
        {
            return false;
        }
    }
    // Select best solution and decode ID using the new function
    bool id_decode_success = selectSolutionAndDecodeID(pose, ellipse_centers, outer_marker);

    WHYCON_DEBUG("Decoded marker ID: " << pose.ID << " angle: " << pose.angle);
    return id_decode_success;
}

// Performance improved upto 20x than scalar version
bool whycon::LocalizationSystem::computeEllipseCoordinates(
    int width, int height, const SegmentParameters& s, aligned_vec<float>& x_coords,
    aligned_vec<float>& y_coords)
{
    // SIMD calculation of coordinates
    using bf     = xs::batch<float>;
    const int VS = bf::size;  // SIMD Vector Size (8 for AVX2)

    // Broadcast constants to SIMD registers
    bf bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f);
    bf zeros(0.0f), width_b(static_cast<float>(width)), height_b(static_cast<float>(height));

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

        if (x < 0 || x >= width || y < 0 || y >= height)
        {
            return false;
        }
    }
    return true;
}

void whycon::LocalizationSystem::computeSignal(
    const unsigned char* image_data, int width, int id_samples,
    whycon::LocalizationSystem::SignalAnalysisData& analysis_data,
    const whycon::ImageHandler&                     image_handler)
{
    const int step = image_handler.getBPP();

    for (int a = 0; a < id_samples; ++a)
    {
        const float xf = analysis_data.x_coords[a];
        const float yf = analysis_data.y_coords[a];
        const int   px = static_cast<int>(xf);
        const int   py = static_cast<int>(yf);
        const float gx = xf - px;
        const float gy = yf - py;

        // Base rows
        const unsigned char* row0 = image_data + py * width * step;
        const unsigned char* row1 = row0 + width * step;
        const int            p0   = px * step;

        // Sum RGB at 4 bilinear corners
        const float S00 = float(row0[p0 + 0]) + float(row0[p0 + 1]) + float(row0[p0 + 2]);
        const float S10 =
            float(row0[p0 + step + 0]) + float(row0[p0 + step + 1]) + float(row0[p0 + step + 2]);
        const float S01 = float(row1[p0 + 0]) + float(row1[p0 + 1]) + float(row1[p0 + 2]);
        const float S11 =
            float(row1[p0 + step + 0]) + float(row1[p0 + step + 1]) + float(row1[p0 + step + 2]);

        // Two horizontal lerps + one vertical (FMA)
        const float row0_interp = std::fmaf(S10 - S00, gx, S00);
        const float row1_interp = std::fmaf(S11 - S01, gx, S01);
        analysis_data.signal[a] = std::fmaf(row1_interp - row0_interp, gy, row0_interp);
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
    float    avg = sum / float(n);
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

bool whycon::LocalizationSystem::processSingleSolution(
    int solution_idx, const ImageHandler& image_handler, int width, int height,
    unsigned char* image_data)
{
    int pos = 0;

    // Reset analysis data for this solution
    analysis_data_[solution_idx].sum      = 0.0f;
    analysis_data_[solution_idx].variance = 0.0f;

    // Get segment parameters for current solution
    if (!computeEllipseCoordinates(
            width,
            height,
            ellipse_processing_.solutions[solution_idx].segment,
            analysis_data_[solution_idx].x_coords,
            analysis_data_[solution_idx].y_coords))
    {
        return false;
    }

    computeSignal(image_data, width, id_samples_, analysis_data_[solution_idx], image_handler);

    binarizeSignal(analysis_data_[solution_idx], id_samples_);

    // Find the edge's locations
    float sx = 0.0f, sy = 0.0f;
    analysis_data_[solution_idx].num_points = 0;
    if (analysis_data_[solution_idx].smooth[id_samples_ - 1] !=
        analysis_data_[solution_idx].smooth[0])
    {
        sx                                      = 1.0f;
        analysis_data_[solution_idx].num_points = 1;
    }
    for (int a = 1; a < id_samples_; a++)
    {
        if (analysis_data_[solution_idx].smooth[a] != analysis_data_[solution_idx].smooth[a - 1])
        {
            sx += trig_lut_seg_width_.cosv[a];
            sy += trig_lut_seg_width_.sinv[a];
            analysis_data_[solution_idx].num_points++;
        }
    }

    analysis_data_[solution_idx].max_idx =
        atan2(sy, sx) / 2 / M_PI * segment_width_ + segment_width_ / 2;

    // Compute Variance
    float meanX = sx / analysis_data_[solution_idx].num_points;
    float meanY = sy / analysis_data_[solution_idx].num_points;
    float errX, errY;
    for (int a = 1; a < id_samples_; a++)
    {
        if (analysis_data_[solution_idx].smooth[a] != analysis_data_[solution_idx].smooth[a - 1])
        {
            errX = trig_lut_seg_width_.cosv[a] - meanX;
            errY = trig_lut_seg_width_.sinv[a] - meanY;
            analysis_data_[solution_idx].sum += errX * errX + errY * errY;
        }
    }
    analysis_data_[solution_idx].variance =
        analysis_data_[solution_idx].sum / analysis_data_[solution_idx].num_points;

    // determine raw code
    for (int a = 0; a < id_bits_ * 2; a++)
        analysis_data_[solution_idx].code[a] =
            analysis_data_[solution_idx]
                .smooth[(analysis_data_[solution_idx].max_idx + a * segment_width_) % id_samples_] +
            '0';

    analysis_data_[solution_idx].code[id_bits_ * 2] = 0;

    const int step = image_handler.getBPP();
    if (image_data != nullptr)
    {
        for (int a = 0; a < id_samples_; a++)
        {
            pos =
                ((int)analysis_data_[ellipse_processing_.selected_idx].x_coords[a] +
                 ((int)analysis_data_[ellipse_processing_.selected_idx].y_coords[a]) * width);
            if (pos > 0 && pos < width * height)
            {
                image_data[step * pos + 0] = 0;
                image_data[step * pos + 1] = (unsigned char)(255.0 * a / id_samples_);
                image_data[step * pos + 2] = 0;
            }
        }
    }

    return true;
}

bool whycon::LocalizationSystem::selectSolutionAndDecodeID(
    MarkerPose& pose, const EllipseCenters& ellipse_centers,
    const MarkerDetector::Marker& outer_marker)
{
    if (!decoder_)
    {
        WHYCON_ERROR("Error: decoder_ is null. ID decoding cannot proceed." << std::endl);
        return false;
    }
    // Choose solution with lower variance
    ellipse_processing_.selected_idx =
        (analysis_data_[0].variance < analysis_data_[1].variance) ? 0 : 1;

    // Set position from the selected solution using ellipse_centers directly
    pose.pos = cv::Vec3f(
        ellipse_centers.t[ellipse_processing_.selected_idx][0],
        ellipse_centers.t[ellipse_processing_.selected_idx][1],
        ellipse_centers.t[ellipse_processing_.selected_idx][2]);

    // Set normal for orientation calculation using ellipse_centers directly
    pose.rot = cv::Vec3f(
        ellipse_centers.n[ellipse_processing_.selected_idx][0],
        ellipse_centers.n[ellipse_processing_.selected_idx][1],
        ellipse_centers.n[ellipse_processing_.selected_idx][2]);

    // Decode the ID using structured data
    int  maxIndex = analysis_data_[ellipse_processing_.selected_idx].max_idx;
    char realCode[id_bits_ + 1];

    SDecoded segment_decode = decoder_->decode(
        analysis_data_[ellipse_processing_.selected_idx].code.data(),
        realCode,
        maxIndex,
        outer_marker.v0,
        outer_marker.v1);
    marker_angle_ = segment_decode.angle;  // Store the marker angle for orientation calculation
    // Variance different check for ID Validity
    ellipse_processing_.variance_difference =
        fabs(analysis_data_[0].variance - analysis_data_[1].variance);

    bool reliable_detection = (ellipse_processing_.variance_difference >
                               variance_threshold_) &&     // Good variance separation
                              (segment_decode.id >= 0) &&  // Valid decoded ID
                              (outer_marker.size >= min_marker_pixels_);  // Sufficient marker size

    if (reliable_detection)
    {
        // Sufficient variance difference - reliable ID detection
        pose.ID       = segment_decode.id + 1;
        pose.id_valid = true;
    }
    else
    {
        // Insufficient variance difference - ambiguous solution
        pose.ID       = -1;
        pose.id_valid = false;
    }

    return pose.id_valid;
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

            // Store reprojected image coordinates
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

void whycon::LocalizationSystem::resolveAmbiguity(
    MarkerPose& pose, const EllipseCenters& centers, const whycon::MarkerDetector::Marker& marker)
{
    // Use inner circle to disambiguate between the two solutions
    // The solution closer to the inner circle center is more likely correct
    float dist0 = std::sqrt(
        (marker.x - centers.u[0]) * (marker.x - centers.u[0]) +
        (marker.y - centers.v[0]) * (marker.y - centers.v[0]));
    float dist1 = std::sqrt(
        (marker.x - centers.u[1]) * (marker.x - centers.u[1]) +
        (marker.y - centers.v[1]) * (marker.y - centers.v[1]));

    // Select the solution with minimal distance
    int idx = (dist0 < dist1) ? 0 : 1;

    // Set position from the selected solution
    pose.pos = cv::Vec3f(centers.t[idx][0], centers.t[idx][1], centers.t[idx][2]);

    // Set normal for orientation calculation
    pose.rot = cv::Vec3f(centers.n[idx][0], centers.n[idx][1], centers.n[idx][2]);

    marker_angle_ = marker.angle;  // Store the marker angle for orientation calculation
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

void whycon::LocalizationSystem::calculateOrientation(MarkerPose& pose) const
{
    // Compute rotation from initial marker normal to detected normal
    cv::Vec3f initial_norm(1.0, 0.0, 0.0);
    cv::Vec3f final_norm(pose.rot[2], -pose.rot[0], -pose.rot[1]);  // Adapt coordinate system
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

    pose.orientation = q3 * z_rot;
    pose.orientation.normalize();
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
    // This function transforms a point from distorted image coordinates to normalized (undistorted)
    // camera coordinates. ENABLE_FULL_UNDISTORT: Use a simple pinhole model (no distortion
    // correction, just normalization). Otherwise: Use OpenCV's undistortPoints for full distortion
    // correction using camera parameters.
#if defined(ENABLE_FULL_UNDISTORT)
    x_out = (x_in - cc[0]) / fc[0];
    y_out = (y_in - cc[1]) / fc[1];
#else
    std::vector<cv::Vec2d> src(1, cv::Vec2d(x_in, y_in));
    std::vector<cv::Vec2d> dst(1);
    cv::undistortPoints(src, dst, K, dist_coeff);
    x_out = dst[0](0);
    y_out = dst[0](1);
#endif
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
    int            width      = image_handler.getWidth();
    int            height     = image_handler.getHeight();
    int            bpp        = image_handler.getBPP();
    unsigned char* image_data = image_handler.getData();

    // Create debug visualization for marker solutions
    cv::Mat debug_image = cv::Mat::zeros(height, width, CV_8UC3);

    // Convert raw image data to cv::Mat for debug visualization
    cv::Mat temp_input_image(height, width, (bpp == 3) ? CV_8UC3 : CV_8UC1, image_data);
    if (bpp == 3)
    {
        temp_input_image.copyTo(debug_image);
    }
    else
    {
        cv::cvtColor(temp_input_image, debug_image, cv::COLOR_GRAY2BGR);
    }
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