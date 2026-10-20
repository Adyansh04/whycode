
#include "whycode/core/marker_detector.hpp"

#include <algorithm>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "whycode/image/free_pixels.hpp"
#include "whycode/utils/whycon_config.h"

#undef ENABLE_RANDOMIZED_THRESHOLD
// #define ENABLE_RANDOMIZED_THRESHOLD

namespace
{
// Mean grey level of a segment's runs, rounded down as the per-pixel sum was
int meanGrey(
    const uchar* gray, int width, const whycon::MarkerDetector::DetectionContext::Span* runs,
    size_t begin, size_t end, int size) noexcept
{
    int sum = 0;
    for (size_t i = begin; i < end; i++)
    {
        const uchar* row = gray + runs[i].y * width;
        for (int x = runs[i].x0; x <= runs[i].x1; x++)
        {
            sum += row[x];
        }
    }
    return sum / size;
}

// The unvisited pixels of one colour in the current binary image
whycon::FreePixels freePixels(
    const whycon::ImageHandler&                     image_handler,
    const whycon::MarkerDetector::DetectionContext& context, bool black) noexcept
{
    return { image_handler.getBinaryPacked().data(),
             context.visited.data(),
             context.width,
             static_cast<size_t>(context.stride),
             black };
}
}  // namespace

whycon::MarkerDetector::MarkerDetector(
    DetectionContext* _context, const DetectorParameters& _parameters)
  : parameters(_parameters)
  , context(_context)
  , diameter_ratio(parameters.inner_diameter / parameters.outer_diameter)
  , outer_area_ratio(M_PI * (1.0 - (diameter_ratio * diameter_ratio)) / 4)
  , inner_area_ratio(M_PI / 4.0)
  , areas_ratio((1.0 - (diameter_ratio * diameter_ratio)) / (diameter_ratio * diameter_ratio))
  , inv_areas_ratio(1.0f / areas_ratio)
  , threshold(256 / 2)
  , threshold_counter(0)
{
    // Initialize the marker buffers
    inner_marker_ = std::make_unique<Marker>();
    outer_marker_ = std::make_unique<Marker>();

    WHYCON_INFO(
        "MarkerDetector initialized with ID detection: "
        << (parameters.identify ? "enabled" : "disabled"));
}

whycon::MarkerDetector::~MarkerDetector() { WHYCON_INFO("MarkerDetector destroyed"); }

int whycon::MarkerDetector::getCurrentThreshold(void) const { return threshold; }

void whycon::MarkerDetector::adjustThreshold(void)
{
// int old_threshold = threshold;
#if !defined(ENABLE_RANDOMIZED_THRESHOLD)
    threshold_counter++;
    int d   = threshold_counter;
    int div = 1;
    while (d > 1)
    {
        d /= 2;
        div *= 2;
    }
    int step  = 256 / div;
    threshold = (step * (threshold_counter - div) + step / 2);
    if (step <= 16)
        threshold_counter = 0;
#else
    unsigned int seed = static_cast<unsigned int>(time(nullptr));
    threshold         = (rand_r(&seed) % 48) * 16;
#endif
    WHYCON_DEBUG("threshold changed to " << threshold);
}

inline float whycon::MarkerDetector::normalizeAngle(float a)
{
    while (a > +M_PI)
        a += -2 * M_PI;
    while (a < -M_PI)
        a += +2 * M_PI;
    return a;
}

void whycon::MarkerDetector::computeEllipseParameters(size_t begin, size_t end, Marker& m) const
{
    // Exact sums over the runs, about the segment's first pixel; a run's sums of x and x^2 have
    // closed forms
    const auto* spans    = context->spans.get();
    const int   origin_x = spans[begin].x0;
    const int   origin_y = spans[begin].y;
    int64_t     n = 0, sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0, sum_yy = 0;
    for (size_t i = begin; i < end; i++)
    {
        const int64_t a = spans[i].x0 - origin_x, b = spans[i].x1 - origin_x;
        const int64_t y = spans[i].y - origin_y, k = b - a + 1;
        const int64_t run_x = (a + b) * k / 2;
        n += k;
        sum_x += run_x;
        sum_y += k * y;
        sum_xx += (b * (b + 1) * (2 * b + 1) - (a - 1) * a * (2 * a - 1)) / 6;
        sum_xy += run_x * y;
        sum_yy += k * y * y;
    }

    // Centroid and central moments, each from an exact integer numerator: the result does not
    // depend on the origin or the pixel order. Exact while n * width stays below 3e9.
    const double count  = static_cast<double>(n);
    const double mean_x = static_cast<double>(origin_x * n + sum_x) / count;
    const double mean_y = static_cast<double>(origin_y * n + sum_y) / count;
    const double cov_xx = static_cast<double>(n * sum_xx - sum_x * sum_x) / (count * count);
    const double cov_xy = static_cast<double>(n * sum_xy - sum_x * sum_y) / (count * count);
    const double cov_yy = static_cast<double>(n * sum_yy - sum_y * sum_y) / (count * count);

    // Compute eigenvalues (squared axes lengths) from the trace and determinant
    const double trace     = cov_xx + cov_yy;
    const double det       = cov_xx * cov_yy - cov_xy * cov_xy;
    const double temp      = trace * trace - 4 * det;
    const double sqrt_term = (temp > 0) ? std::sqrt(temp) : 0.0;
    const double lambda1   = (trace + sqrt_term) / 2.0;  // major axis squared
    const double lambda2   = (trace - sqrt_term) / 2.0;  // minor axis squared

    // Assign outputs: center
    m.x = static_cast<float>(mean_x);
    m.y = static_cast<float>(mean_y);

    // Assign outputs: axes lengths (take sqrt to get actual axis length). Rounding can leave a
    // line-like segment's eigenvalue just below zero, and a NaN axis gets past the -ffast-math
    // circularity check; clamped, the zero axis fails it.
    m.m0 = static_cast<float>(std::sqrt(std::max(lambda1, 0.0)));  // major axis
    m.m1 = static_cast<float>(std::sqrt(std::max(lambda2, 0.0)));  // minor axis

    // Compute orientation vector (eigenvector for major axis)
    if (cov_xy != 0.0)
    {
        const double norm = std::sqrt(cov_xy * cov_xy + (cov_xx - lambda1) * (cov_xx - lambda1));
        m.v0              = static_cast<float>(-cov_xy / norm);
        m.v1              = static_cast<float>((cov_xx - lambda1) / norm);
    }
    else
    {
        // If the ellipse is axis-aligned, set orientation accordingly
        m.v0 = m.v1 = 0.0f;
        if (cov_xx > cov_yy)
        {
            m.v0 = 1.0f;
        }
        else
        {
            m.v1 = 1.0f;
        }
    }
}

bool whycon::MarkerDetector::analyzeMarkerCandidate(
    const ImageHandler& image_handler, whycon::MarkerDetector::Marker& marker, int seed_pixel_index,
    float expected_area_ratio, bool is_outer)
{
    const int  width  = image_handler.getWidth();
    const int  height = image_handler.getHeight();
    const int  seed_x = seed_pixel_index % width;
    const int  seed_y = seed_pixel_index / width;
    const bool type   = image_handler.getBinaryPacked().getPixel(seed_x, seed_y);

    context->total_segments++;
    marker.x     = static_cast<float>(seed_x);
    marker.y     = static_cast<float>(seed_pixel_index) / static_cast<float>(width);
    marker.valid = false;
    marker.round = false;

    // Scanline flood fill: each run of free pixels is taken whole, and the rows above and below it
    // are searched 64 pixels at a time. 4-connected, so the segment is the per-pixel fill's.
    const FreePixels free   = freePixels(image_handler, *context, type);
    auto*            spans  = context->spans.get();
    const size_t     stride = context->stride;
    int              size = 0, minx = seed_x, maxx = seed_x, miny = seed_y, maxy = seed_y;
    const auto       take = [&](int y, int x0, int x1) {
        bits::set(context->visited.data() + y * stride, x0, x1, true);
        spans[span_end_++] = { y, x0, x1 };
        size += x1 - x0 + 1;
        minx = std::min(minx, x0);
        maxx = std::max(maxx, x1);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    };
    span_start_ = span_end_;
    take(seed_y, free.runStart(seed_y, seed_x), free.runEnd(seed_y, seed_x));
    for (size_t read = span_start_; read < span_end_; read++)
    {
        const auto run = spans[read];
        for (const int y : { run.y - 1, run.y + 1 })
        {
            if (y < 0 || y >= height)
            {
                continue;
            }
            free.forEachRun(y, run.x0, run.x1, [&](int first, int last) { take(y, first, last); });
        }
    }

    // once the segment is complete, we have its size
    marker.size = size;

    // Check if segment is within valid size range
    if (marker.size > parameters.min_size && marker.size < parameters.max_size)
    {
        // Store segment properties
        marker.maxx = maxx;
        marker.maxy = maxy;
        marker.minx = minx;
        marker.miny = miny;
        marker.type = -static_cast<int>(type);

        const int width_pixels  = (marker.maxx - marker.minx + 1);
        const int height_pixels = (marker.maxy - marker.miny + 1);

        marker.x = (marker.maxx + marker.minx) / 2;
        marker.y = (marker.maxy + marker.miny) / 2;

        marker.roundness = width_pixels * height_pixels * expected_area_ratio / marker.size;

        WHYCON_DEBUG(
            "width_pixels,height_pixels " << width_pixels << "," << height_pixels << " roundness "
                                          << marker.roundness);
        // Check if segment is round enough
        if (fabsf(marker.roundness - 1.0f) < parameters.roundness_tolerance || !is_outer)
        {
            // The mean grey level waits for a whole marker: most candidates are not one
            marker.round = true;

            WHYCON_INFO(
                "valid segment found: " << marker.size << " pixels, with size " << width_pixels
                                        << " x " << height_pixels);
        }
        else
        {
            WHYCON_ERROR(
                "Segment not round enough (" << marker.roundness << ") width_pixels/height_pixels "
                                             << width_pixels << " x " << height_pixels << " ctr "
                                             << marker.x << " " << marker.y << " " << marker.size
                                             << " " << expected_area_ratio);
        }
    }
    return marker.round;
}

inline bool
whycon::MarkerDetector::validateMarkerPair(const Marker& inner, const Marker& outer) const
{
    // Check area ratio
    const float measured_ratio_over_expected =
        (static_cast<float>(outer.size) / static_cast<float>(inner.size)) * inv_areas_ratio;
    if (fabsf(measured_ratio_over_expected - 1.0f) > parameters.ratio_tolerance)
    {
        WHYCON_DEBUG(
            "Validation failed: Area ratio out of tolerance. Measured/Expected: "
            << measured_ratio_over_expected);
        return false;
    }

    // Check center distance
    const float center_dx = fabsf(inner.x - outer.x);
    const float center_dy = fabsf(inner.y - outer.y);
    const float tolerance_x =
        parameters.center_distance_tolerance_abs +
        parameters.center_distance_tolerance_ratio * static_cast<float>(outer.maxx - outer.minx);
    const float tolerance_y =
        parameters.center_distance_tolerance_abs +
        parameters.center_distance_tolerance_ratio * static_cast<float>(outer.maxy - outer.miny);

    if (center_dx > tolerance_x || center_dy > tolerance_y)
    {
        WHYCON_DEBUG(
            "Validation failed: Center distance out of tolerance. dx: "
            << center_dx << " > " << tolerance_x << " or dy: " << center_dy << " > "
            << tolerance_y);
        return false;
    }

    return true;
}

bool whycon::MarkerDetector::examineSeed(
    const ImageHandler& image_handler, int seed_pixel_index, DebugImageManager* debug_manager)
{
    const int  width = image_handler.getWidth();
    const bool debug = debug_manager && debug_manager->isEnabled();

    // The outer ring: a round black segment
    span_end_ = 0;
    if (!analyzeMarkerCandidate(
            image_handler,
            *outer_marker_,
            seed_pixel_index,
            outer_area_ratio,
            true))
    {
        return false;
    }

    // The inner disc: the white segment at the ring's centre, if no segment has taken it yet
    const int centre_x = static_cast<int>(outer_marker_->x);
    const int centre_y = static_cast<int>(outer_marker_->y);
    WHYCON_DEBUG(
        "found valid outer, looking for white at (" << centre_x << "," << centre_y
                                                    << ") id: " << context->total_segments - 1);
    if (!freePixels(image_handler, *context, PIXEL_WHITE).isFree(centre_y, centre_x) ||
        !analyzeMarkerCandidate(
            image_handler,
            *inner_marker_,
            centre_y * width + centre_x,
            inner_area_ratio,
            false) ||
        !validateMarkerPair(*inner_marker_, *outer_marker_))
    {
        return false;
    }
    if (debug)
    {
        generateSegmentDebugImage(image_handler, debug_manager);
    }

    computeEllipseParameters(span_start_, span_end_, *inner_marker_);
    // The outline is the ring and its centre together, as in the original WhyCon: the ring's
    // moments alone overstate the radius by 11-17% and pull the centre towards the code's teeth.
    computeEllipseParameters(0, span_end_, *outer_marker_);
    if (debug)
    {
        generateEllipseDebugImage(image_handler, debug_manager);
    }
    WHYCON_DEBUG(
        "Outer ellipse params: center=("
        << outer_marker_->x << "," << outer_marker_->y << ")"
        << " m0=" << outer_marker_->m0 << " m1=" << outer_marker_->m1 << " v0=" << outer_marker_->v0
        << " v1=" << outer_marker_->v1);

    inner_marker_->bwRatio = static_cast<float>(outer_marker_->size) / inner_marker_->size;

    // Calculate ellipse quality metrics
    const float circularity = static_cast<float>(
        M_PI * 4 * (outer_marker_->m0) * (outer_marker_->m1) /
        (outer_marker_->size + inner_marker_->size));
    const float eccentricity = sqrtf(
        1.0f - (outer_marker_->m1 * outer_marker_->m1) / (outer_marker_->m0 * outer_marker_->m0));
    if (debug)
    {
        generateMParamDebugImage(image_handler, debug_manager, eccentricity, circularity);
    }
    if (!(fabsf(circularity - 1.0f) < parameters.circularity_tolerance &&
          eccentricity < parameters.max_eccentricity))
    {
        return false;
    }

    outer_marker_->valid = inner_marker_->valid = true;
    const uchar* gray                           = image_handler.getGray().data;
    const auto*  spans                          = context->spans.get();
    outer_marker_->mean = meanGrey(gray, width, spans, 0, span_start_, outer_marker_->size);
    inner_marker_->mean = meanGrey(gray, width, spans, span_start_, span_end_, inner_marker_->size);

    // use a new threshold estimate based on current detection
    threshold = edgeThreshold(outer_marker_->mean, inner_marker_->mean, parameters.image_gamma);
    WHYCON_DEBUG("found inner segment " << context->total_segments - 1);
    return true;
}

void whycon::MarkerDetector::detectMarkerPair(
    const ImageHandler& image_handler, MarkerPair& result, const Marker& previous_circle,
    DebugImageManager* debug_manager, const SearchWindow* window)
{
    // Reset the member buffers to ensure a clean state for this detection run
    *inner_marker_ = Marker{};
    *outer_marker_ = Marker{};

    const int width = image_handler.getWidth();

    // this allows to differentiate segments found by this detector from others, and know how many
    // segments where found in this call
    initial_segment_id = context->total_segments;

    WHYCON_DEBUG("detector id " << detector_id << " B/W " << PIXEL_BLACK << "/" << PIXEL_WHITE);
    WHYCON_DEBUG("threshold " << threshold);
    WHYCON_DEBUG("initial segment id " << initial_segment_id);

    int seed_pixel_index = 0;

    if (previous_circle.valid)
    {
        WHYCON_DEBUG(
            "starting with previously valid circle at " << previous_circle.x << ","
                                                        << previous_circle.y);
        seed_pixel_index =
            (static_cast<int>(previous_circle.y)) * width + static_cast<int>(previous_circle.x);
    }

    // Seeds are the unvisited black pixels, in row-major order; true once the search is over
    const FreePixels seeds      = freePixels(image_handler, *context, PIXEL_BLACK);
    const auto       search_row = [&](int y, int x_first, int x_last) {
        for (int x = seeds.next(y, x_first, x_last); x >= 0;
             x     = x < x_last ? seeds.next(y, x + 1, x_last) : -1)
        {
            if ((context->total_segments - initial_segment_id) > MAX_SEGMENTS)
            {
                WHYCON_DEBUG("reached maximum number of segments");
                return true;
            }
            if (examineSeed(image_handler, y * width + x, debug_manager))
            {
                return true;
            }
        }
        return false;
    };

    // From the previous centre, row by row with wrap-around, over the window or the whole image
    int x0 = 0, y0 = 0, x1 = width, y1 = image_handler.getHeight();
    if (window != nullptr)
    {
        x0 = window->x0;
        y0 = window->y0;
        x1 = window->x1;
        y1 = window->y1;
    }
    const int cols = x1 - x0, rows = y1 - y0;
    if (cols > 0 && rows > 0)
    {
        const int first = std::clamp(
            (seed_pixel_index / width - y0) * cols + seed_pixel_index % width - x0,
            0,
            cols * rows - 1);
        const int first_y = y0 + first / cols, first_x = x0 + first % cols;
        bool      done = search_row(first_y, first_x, x1 - 1);
        for (int k = 1; k < rows && !done; k++)
        {
            done = search_row(y0 + (first_y - y0 + k) % rows, x0, x1 - 1);
        }
        if (!done && first_x > x0)
        {
            search_row(first_y, x0, first_x - 1);
        }
    }

    // Pose-processing: angle and threshold logic
    if (inner_marker_->valid)
    {
        // Calculate angle between inner and outer markers
        float orient =
            atan2(outer_marker_->y - inner_marker_->y, outer_marker_->x - inner_marker_->x);

        // Calculate orientation from ellipse parameters
        outer_marker_->angle = atan2(outer_marker_->v1, outer_marker_->v0);

        // Normalize the angle to be consistent with the marker orientation
        if (fabs(normalizeAngle(outer_marker_->angle - orient)) > M_PI / 2)
            outer_marker_->angle = normalizeAngle(outer_marker_->angle + M_PI);

        inner_marker_->angle = outer_marker_->angle;  // copy angle to inner marker too
        WHYCON_DEBUG("angle " << outer_marker_->angle << " orient " << orient);

        // Reset the threshold counter on successful detection
        threshold_counter = 0;
    }
    else
    {
        // If detection failed, adjust threshold for next attempt
        WHYCON_DEBUG("Adjusting threshold for next detection attempt.");
        adjustThreshold();
    }

    WHYCON_DEBUG("processed segments " << (context->total_segments - initial_segment_id));

    // Return both inner and outer markers as a MarkerPair
    result.inner = *inner_marker_;
    result.outer = *outer_marker_;
    result.valid = (inner_marker_->valid && outer_marker_->valid);
}

whycon::MarkerDetector::Marker::Marker() {}

void whycon::MarkerDetector::Marker::draw(
    cv::Mat& image, std::string_view text, cv::Vec3b color, float thickness) const
{
    for (float e = 0; e < 2 * M_PI; e += 0.01)
    {
        float fx  = x + cos(e) * v0 * m0 * 2 + v1 * m1 * 2 * sin(e);
        float fy  = y + cos(e) * v1 * m0 * 2 - v0 * m1 * 2 * sin(e);
        int   fxi = static_cast<int>(fx + 0.5);
        int   fyi = static_cast<int>(fy + 0.5);
        if (fxi >= 0 && fxi < image.cols && fyi >= 0 && fyi < image.rows)
            image.at<cv::Vec3b>(fyi, fxi) = color;
    }

    if (!text.empty())
    {
        // putText ignores '\n', so each line is its own call, outlined to read on any background
        const float scale      = image.size().width / 2560.0f;
        const int   line_count = static_cast<int>(std::count(text.begin(), text.end(), '\n')) + 1;
        int         baseline   = 0;
        const int   line_height =
            cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline).height +
            baseline + 2;
        const float radius = 2 * std::max(m0, m1);
        int         text_x = static_cast<int>(x - radius);
        int         text_y = static_cast<int>(y + radius) + line_height;
        if (text_y + (line_count - 1) * line_height >= image.rows)
        {
            text_y = static_cast<int>(y - radius) - (line_count - 1) * line_height - baseline;
        }

        size_t start = 0;
        while (start <= text.size())
        {
            const size_t      end = std::min(text.find('\n', start), text.size());
            const std::string line(text.substr(start, end - start));
            const int         width =
                cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline).width;
            const cv::Point origin(std::clamp(text_x, 0, std::max(image.cols - width, 0)), text_y);
            cv::putText(
                image,
                line,
                origin,
                cv::FONT_HERSHEY_SIMPLEX,
                scale,
                cv::Scalar(0, 0, 0),
                thickness + 2,
                cv::LINE_AA);
            cv::putText(
                image,
                line,
                origin,
                cv::FONT_HERSHEY_SIMPLEX,
                scale,
                cv::Scalar(color),
                thickness,
                cv::LINE_AA);
            text_y += line_height;
            start = end + 1;
        }
    }
    cv::line(
        image,
        cv::Point(x + v0 * m0 * 2, y + v1 * m0 * 2),
        cv::Point(x - v0 * m0 * 2, y - v1 * m0 * 2),
        cv::Scalar(color),
        1,
        8);
    cv::line(
        image,
        cv::Point(x + v1 * m1 * 2, y - v0 * m1 * 2),
        cv::Point(x - v1 * m1 * 2, y + v0 * m1 * 2),
        cv::Scalar(color),
        1,
        8);
}

whycon::MarkerDetector::DetectionContext::DetectionContext(int _width, int _height)
{
    width  = _width;
    height = _height;
    stride = static_cast<int>(PackedBinaryImage::strideFor(width));
    visited.resize(static_cast<size_t>(stride) * height);
    // A run takes at least one pixel
    spans = std::make_unique_for_overwrite<Span[]>(static_cast<size_t>(width) * height);

    cleanupBuffer();
    reset();
}

void whycon::MarkerDetector::DetectionContext::reset(void)
{
    next_detector_id = 0;
    total_segments   = 0;
}

void whycon::MarkerDetector::DetectionContext::cleanupBuffer(void)
{
    WHYCON_INFO("Cleaning the entire buffer.");
    std::fill(visited.begin(), visited.end(), 0);
    // The bits past the image stay visited: no segment runs into them
    for (int y = 0; y < height && width < stride * 8; y++)
    {
        bits::set(visited.data() + static_cast<size_t>(y) * stride, width, stride * 8 - 1, true);
    }
}

void whycon::MarkerDetector::generateSegmentDebugImage(
    const ImageHandler& image_handler, DebugImageManager* debug_manager) const
{
    // Make a copy of the input image for visualization
    cv::Mat segment_debug;
    image_handler.copyColour(segment_debug);

    // Draw the segments' runs: yellow for outer, magenta for inner
    for (size_t i = 0; i < span_end_; i++)
    {
        const auto& run = context->spans[i];
        cv::line(
            segment_debug,
            cv::Point(run.x0, run.y),
            cv::Point(run.x1, run.y),
            i < span_start_ ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 255));
    }
    // Draw/ Plot all points of outer and inner
    cv::circle(
        segment_debug,
        cv::Point((outer_marker_->maxx), outer_marker_->maxy),
        2,
        cv::Scalar(0, 255, 0),
        10);
    cv::circle(
        segment_debug,
        cv::Point((outer_marker_->minx), outer_marker_->miny),
        2,
        cv::Scalar(0, 255, 0),
        10);
    cv::circle(
        segment_debug,
        cv::Point((inner_marker_->maxx), inner_marker_->maxy),
        2,
        cv::Scalar(255, 0, 0),
        10);
    cv::circle(
        segment_debug,
        cv::Point((inner_marker_->minx), inner_marker_->miny),
        2,
        cv::Scalar(255, 0, 0),
        10);

    debug_manager->addDebugImage(segment_debug, "Marker Pair Detection");
}

void whycon::MarkerDetector::generateEllipseDebugImage(
    const ImageHandler& image_handler, DebugImageManager* debug_manager) const
{
    // Make a copy of the input image for visualization
    cv::Mat ellipse_debug;
    image_handler.copyColour(ellipse_debug);

    // Draw outer and ellipse in a new opencv imshow window
    cv::ellipse(
        ellipse_debug,
        cv::Point(outer_marker_->x, outer_marker_->y),
        cv::Size(outer_marker_->m0, outer_marker_->m1),
        atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI,
        0,
        360,
        cv::Scalar(0, 255, 255),
        2);
    cv::ellipse(
        ellipse_debug,
        cv::Point(inner_marker_->x, inner_marker_->y),
        cv::Size(inner_marker_->m0, inner_marker_->m1),
        atan2(inner_marker_->v1, inner_marker_->v0) * 180 / M_PI,
        0,
        360,
        cv::Scalar(255, 0, 255),
        2);
    // Draw center points of inner and outer ellipses
    cv::circle(
        ellipse_debug,
        cv::Point(outer_marker_->x, outer_marker_->y),
        2,
        cv::Scalar(0, 255, 0),
        5);
    cv::circle(
        ellipse_debug,
        cv::Point(inner_marker_->x, inner_marker_->y),
        2,
        cv::Scalar(255, 0, 0),
        5);

    cv::putText(
        ellipse_debug,
        "Outer",
        cv::Point(outer_marker_->x + 10, outer_marker_->y - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 255),
        1);
    cv::putText(
        ellipse_debug,
        "Inner",
        cv::Point(inner_marker_->x + 10, inner_marker_->y - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 0, 255),
        1);

    debug_manager->addDebugImage(ellipse_debug, "ellipse_debug");
}

void whycon::MarkerDetector::generateMParamDebugImage(
    const ImageHandler& image_handler, DebugImageManager* debug_manager, float eccentricity,
    float circularity) const
{
    // Make a copy of the input image for visualization
    cv::Mat annotate_debug;
    image_handler.copyColour(annotate_debug);

    // Draw ellipse and center
    cv::ellipse(
        annotate_debug,
        cv::Point(outer_marker_->x, outer_marker_->y),
        cv::Size(outer_marker_->m0, outer_marker_->m1),
        atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI,
        0,
        360,
        cv::Scalar(0, 255, 255),
        2);
    cv::circle(
        annotate_debug,
        cv::Point(outer_marker_->x, outer_marker_->y),
        2,
        cv::Scalar(0, 255, 0),
        5);

    // Calculate center distance
    float center_distance = std::sqrt(
        (inner_marker_->x - outer_marker_->x) * (inner_marker_->x - outer_marker_->x) +
        (inner_marker_->y - outer_marker_->y) * (inner_marker_->y - outer_marker_->y));

    // Calculate center tolerance values
    float toleranceX = parameters.center_distance_tolerance_abs +
                       parameters.center_distance_tolerance_ratio *
                           static_cast<float>(outer_marker_->maxx - outer_marker_->minx);
    float toleranceY = parameters.center_distance_tolerance_abs +
                       parameters.center_distance_tolerance_ratio *
                           static_cast<float>(outer_marker_->maxy - outer_marker_->miny);

    float center_x_diff = std::abs(inner_marker_->x - outer_marker_->x);
    float center_y_diff = std::abs(inner_marker_->y - outer_marker_->y);

    // Prepare annotation texts (one per line)
    char text1[128], text2[128], text3[128], text4[128];
    snprintf(text1, sizeof(text1), "Ecc: %.3f  Round: %.3f", eccentricity, outer_marker_->roundness);
    snprintf(
        text2,
        sizeof(text2),
        "CtrDist: %.1f  CtrThrX: %.1f  CtrThrY: %.1f",
        center_distance,
        toleranceX,
        toleranceY);
    snprintf(text4, sizeof(text4), "  CtrX: %.1f  CtrY: %.1f", center_x_diff, center_y_diff);
    snprintf(text3, sizeof(text3), "Circ: %.3f  Size: %d", circularity, outer_marker_->size);

    // Draw text near the marker with vertical offsets
    int base_x     = std::max(0, static_cast<int>(outer_marker_->x) + 10 - 400);
    int base_y     = std::max(0, static_cast<int>(outer_marker_->y) - 10);
    int lineHeight = 15;  // vertical space between lines

    cv::putText(
        annotate_debug,
        text1,
        cv::Point(base_x, base_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 255),
        1);
    cv::putText(
        annotate_debug,
        text2,
        cv::Point(base_x, base_y + lineHeight),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 255),
        1);
    cv::putText(
        annotate_debug,
        text4,
        cv::Point(base_x, base_y + 2 * lineHeight),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 255),
        1);
    cv::putText(
        annotate_debug,
        text3,
        cv::Point(base_x, base_y + 3 * lineHeight),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 255),
        1);

    // Optionally, color code the ellipse if it failed a check
    cv::Scalar ellipse_color = (fabsf(circularity - 1) < parameters.circularity_tolerance &&
                                eccentricity < parameters.max_eccentricity) ?
                                   cv::Scalar(0, 255, 0)  // Green for valid
                                   :
                                   cv::Scalar(0, 0, 255);  // Red for invalid
    cv::ellipse(
        annotate_debug,
        cv::Point(outer_marker_->x, outer_marker_->y),
        cv::Size(outer_marker_->m0, outer_marker_->m1),
        atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI,
        0,
        360,
        ellipse_color,
        2);

    debug_manager->addDebugImage(annotate_debug, "Marker Checks");
}