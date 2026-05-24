
#include "whycode/core/marker_detector.hpp"

#include <algorithm>
#include <ctime>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <xsimd/types/xsimd_api.hpp>

#include "whycode/utils/coord_lut.hpp"
#include "whycode/utils/whycon_config.h"

#undef ENABLE_RANDOMIZED_THRESHOLD
// #define ENABLE_RANDOMIZED_THRESHOLD

whycon::MarkerDetector::MarkerDetector(int _width, int _height, DetectionContext* _context,
                                       const DetectorParameters& _parameters)
    : parameters(_parameters)
    , context(_context)
    , width_(_width)
    , height_(_height)
    , len(_width * _height)
    , siz(_width * _height * 3)
    , diameter_ratio(parameters.inner_diameter / parameters.outer_diameter)
    , outer_area_ratio(M_PI * (1.0 - (diameter_ratio * diameter_ratio)) / 4)
    , inner_area_ratio(M_PI / 4.0)
    , areas_ratio((1.0 - (diameter_ratio * diameter_ratio)) / (diameter_ratio * diameter_ratio))
    , inv_areas_ratio(1.0f / areas_ratio)
    , threshold(256 / 2)
    , threshold_counter(0)
    , width_vec_(batch_int(_width)) {
    // Initialize the marker buffers
    inner_marker_ = std::make_unique<Marker>();
    outer_marker_ = std::make_unique<Marker>();

    WHYCON_INFO("MarkerDetector initialized with ID detection: " << (parameters.identify ? "enabled" : "disabled"));
}

whycon::MarkerDetector::~MarkerDetector() {
    WHYCON_INFO("MarkerDetector destroyed");
}

int whycon::MarkerDetector::getCurrentThreshold(void) const {
    return threshold;
}

void whycon::MarkerDetector::adjustThreshold(void) {
// int old_threshold = threshold;
#if !defined(ENABLE_RANDOMIZED_THRESHOLD)
    threshold_counter++;
    int d   = threshold_counter;
    int div = 1;
    while (d > 1) {
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

inline float whycon::MarkerDetector::normalizeAngle(float a) {
    while (a > +M_PI)
        a += -2 * M_PI;
    while (a < -M_PI)
        a += +2 * M_PI;
    return a;
}

void whycon::MarkerDetector::computeEllipseStatsScalar(const int* queue, int start, int end,
                                                       EllipseComputationCache& cache) const {
    int        idx;
    std::div_t div_result;
    float      px, py;
    for (int p = start; p < end; p++) {
        idx        = queue[p];
        div_result = std::div(idx, width_);
        px         = static_cast<float>(div_result.rem);   // x-coordinate
        py         = static_cast<float>(div_result.quot);  // y-coordinate

        // Update cache with pixel coordinates
        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

void whycon::MarkerDetector::computeEllipseStatsSIMD(const int* queue, int start, int end,
                                                     EllipseComputationCache& cache) const {
    // Reset pre-allocated SIMD vectors (much faster than construction)
    sum_x_vec_  = batch_float(0.0f);
    sum_y_vec_  = batch_float(0.0f);
    sum_xx_vec_ = batch_float(0.0f);
    sum_xy_vec_ = batch_float(0.0f);
    sum_yy_vec_ = batch_float(0.0f);

    // Width Vector
    const batch_int width_vec(width_);

    // Process SIMD-sized chunks
    int       p        = start;
    const int simd_end = start + ((end - start) / simd_size_) * simd_size_;

    for (; p < simd_end; p += simd_size_) {
        // Load indices directly as integers
        auto idx_vec = xsimd::load_unaligned(&queue[p]);

        // Compute coordinates using division and modulo
        auto py_vec = idx_vec / width_vec;  // y-coordinates
        auto px_vec = idx_vec % width_vec;  // x-coordinates

        // Convert to float for accumulation
        auto px_f = xsimd::batch_cast<float>(px_vec);
        auto py_f = xsimd::batch_cast<float>(py_vec);

        // Accumulate statistics
        sum_x_vec_ += px_f;
        sum_y_vec_ += py_f;
        sum_xx_vec_ += px_f * px_f;
        sum_xy_vec_ += px_f * py_f;
        sum_yy_vec_ += py_f * py_f;
    }

    // Reduce SIMD accumulators to scalars
    cache.sum_x  = xsimd::reduce_add(sum_x_vec_);
    cache.sum_y  = xsimd::reduce_add(sum_y_vec_);
    cache.sum_xx = xsimd::reduce_add(sum_xx_vec_);
    cache.sum_xy = xsimd::reduce_add(sum_xy_vec_);
    cache.sum_yy = xsimd::reduce_add(sum_yy_vec_);

    //_ Process remaining elements
    int        idx;
    std::div_t div_result;
    float      px, py;
    for (; p < end; p++) {
        idx        = queue[p];
        div_result = std::div(idx, width_);
        px         = static_cast<float>(div_result.rem);   // x-coordinate
        py         = static_cast<float>(div_result.quot);  // y-coordinate

        // Update cache with pixel coordinates
        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

void whycon::MarkerDetector::computeEllipseParameters(const int* queue, int start, int end, Marker& m) const {
    // Use the pre-allocated cache and reset it for the new computation
    auto& cache = ellipse_cache_;
    cache.reset();

    cache.num_points = end - start;

    // Calculate sums and second moments for all points in the segment
    if (cache.num_points >= 4 * simd_size_) {
        computeEllipseStatsSIMD(queue, start, end, cache);
    } else {
        computeEllipseStatsScalar(queue, start, end, cache);
    }

    // Compute centroid (mean position)
    cache.mean_x = cache.sum_x / cache.num_points;
    cache.mean_y = cache.sum_y / cache.num_points;

    // Compute central moments (covariance matrix elements)
    cache.cov_xx = (cache.sum_xx - cache.mean_x * cache.mean_x * cache.num_points) / cache.num_points;
    cache.cov_xy = (cache.sum_xy - cache.mean_x * cache.mean_y * cache.num_points) / cache.num_points;
    cache.cov_yy = (cache.sum_yy - cache.mean_y * cache.mean_y * cache.num_points) / cache.num_points;

    // The covariance matrix is:
    // | cov_xx  cov_xy |
    // | cov_xy  cov_yy |

    // Compute trace and determinant for eigenvalue calculation
    cache.trace = cache.cov_xx + cache.cov_yy;
    cache.det   = cache.cov_xx * cache.cov_yy - cache.cov_xy * cache.cov_xy;

    // Compute eigenvalues (squared axes lengths)
    cache.temp      = cache.trace * cache.trace - 4 * cache.det;
    cache.sqrt_term = (cache.temp > 0) ? sqrtf(cache.temp) : 0.0f;
    cache.lambda1   = (cache.trace + cache.sqrt_term) / 2.0f;  // major axis squared
    cache.lambda2   = (cache.trace - cache.sqrt_term) / 2.0f;  // minor axis squared

    // Assign outputs: center
    m.x = cache.mean_x;
    m.y = cache.mean_y;

    // Assign outputs: axes lengths (take sqrt to get actual axis length)
    m.m0 = sqrtf(cache.lambda1);  // major axis
    m.m1 = sqrtf(cache.lambda2);  // minor axis

    // Compute orientation vector (eigenvector for major axis)
    if (cache.cov_xy != 0.0f) {
        cache.norm =
                sqrtf(cache.cov_xy * cache.cov_xy + (cache.cov_xx - cache.lambda1) * (cache.cov_xx - cache.lambda1));
        m.v0 = -cache.cov_xy / cache.norm;
        m.v1 = (cache.cov_xx - cache.lambda1) / cache.norm;
    } else {
        // If the ellipse is axis-aligned, set orientation accordingly
        m.v0 = m.v1 = 0.0f;
        if (cache.cov_xx > cache.cov_yy) {
            m.v0 = 1.0f;
        } else {
            m.v1 = 1.0f;
        }
    }
}

bool whycon::MarkerDetector::analyzeMarkerCandidate(const ImageHandler&             image_handler,
                                                    whycon::MarkerDetector::Marker& marker, int seed_pixel_index,
                                                    float expected_area_ratio, bool is_outer,
                                                    DebugImageManager* debug_manager) {
    // Use the pre-allocated cache and reset it for the new computation
    auto& cache = analysis_cache_;
    cache.reset();

    // Cache frequently accessed pointers
    cache.buffer_ptr = context->buffer.get();
    cache.queue_ptr  = context->queue.get();

    const int    width     = image_handler.getWidth();
    const int    height    = image_handler.getHeight();
    const uchar* gray_data = image_handler.getGray().data;
    // const uchar* binary_data = image_handler.getBinary().data;
    const auto& binary_packed = image_handler.getBinaryPacked();

    queue_old_start = queue_start;
    cache.type      = binary_packed.getPixelLinear(seed_pixel_index);

    // WHYCON_DEBUG("examine (type " << cache.type << ") at " << seed_pixel_index / width << ","
    //                               << seed_pixel_index % width << " (numseg " << context->total_segments << ")");

    cache.segment_id                   = context->total_segments++;
    cache.buffer_ptr[seed_pixel_index] = cache.segment_id;
    marker.x                           = static_cast<float>(seed_pixel_index % width);
    marker.y                           = static_cast<float>(seed_pixel_index) / static_cast<float>(width);

    cache.minx = cache.maxx = marker.x;
    cache.miny = cache.maxy = marker.y;
    marker.valid            = false;
    marker.round            = false;

    // push segment coords to the queue
    cache.queue_ptr[queue_end++] = seed_pixel_index;

    // Pre-compute width inverse for faster division
    cache.width_inv = 1.0f / static_cast<float>(width);

    // Flood fill algorithm
    while (queue_end > queue_start) {
        // pull the coord from the queue
        cache.position = cache.queue_ptr[queue_start++];

        cache.position_x = cache.position % width;
        cache.position_y = static_cast<int>(cache.position * cache.width_inv);

        // Check right neighbor
        if (cache.position_x + 1 < width) {
            cache.pos = cache.position + 1;
            if (cache.buffer_ptr[cache.pos] == UNVISITED && binary_packed.getPixelLinear(cache.pos) == cache.type) {
                cache.queue_ptr[queue_end++] = cache.pos;
                cache.buffer_ptr[cache.pos]  = cache.segment_id;
                cache.maxx                   = wMax(cache.maxx, cache.position_x + 1);
            }
        }

        // Check left neighbor
        if (cache.position_x - 1 >= 0) {
            cache.pos = cache.position - 1;
            if (cache.buffer_ptr[cache.pos] == UNVISITED && binary_packed.getPixelLinear(cache.pos) == cache.type) {
                cache.queue_ptr[queue_end++] = cache.pos;
                cache.buffer_ptr[cache.pos]  = cache.segment_id;
                cache.minx                   = wMin(cache.minx, cache.position_x - 1);
            }
        }

        // Check bottom neighbor
        if (cache.position_y + 1 < height) {
            cache.pos = cache.position + width;
            if (cache.buffer_ptr[cache.pos] == UNVISITED && binary_packed.getPixelLinear(cache.pos) == cache.type) {
                cache.queue_ptr[queue_end++] = cache.pos;
                cache.buffer_ptr[cache.pos]  = cache.segment_id;
                cache.maxy                   = wMax(cache.maxy, cache.position_y + 1);
            }
        }

        // Check top neighbor
        if (cache.position_y - 1 >= 0) {
            cache.pos = cache.position - width;
            if (cache.buffer_ptr[cache.pos] == UNVISITED && binary_packed.getPixelLinear(cache.pos) == cache.type) {
                cache.queue_ptr[queue_end++] = cache.pos;
                cache.buffer_ptr[cache.pos]  = cache.segment_id;
                cache.miny                   = wMin(cache.miny, cache.position_y - 1);
            }
        }
    }

    // once the queue is empty, i.e. segment is complete, we compute its size
    marker.size = queue_end - queue_old_start;
    // WHYCON_DEBUG("segment size " << marker.size << " (queue_end " << queue_end << ", queue_old_start "
    //                              << queue_old_start << ")");

    // WHYCON_DEBUG("size " << marker.size << " (minx,maxx,miny,maxy) " << cache.minx << "," << cache.maxx << ","
    //                      << cache.miny << "," << cache.maxy);

    // Check if segment is within valid size range
    if (marker.size > parameters.min_size && marker.size < parameters.max_size) {
        // Store segment properties
        marker.maxx = cache.maxx;
        marker.maxy = cache.maxy;
        marker.minx = cache.minx;
        marker.miny = cache.miny;
        marker.type = -cache.type;

        cache.width_pixels  = (marker.maxx - marker.minx + 1);
        cache.height_pixels = (marker.maxy - marker.miny + 1);

        marker.x = (marker.maxx + marker.minx) / 2;
        marker.y = (marker.maxy + marker.miny) / 2;

        marker.roundness = cache.width_pixels * cache.height_pixels * expected_area_ratio / marker.size;

        WHYCON_DEBUG("width_pixels,height_pixels " << cache.width_pixels << "," << cache.height_pixels << " roundness "
                                                   << marker.roundness);
        // Check if segment is round enough
        if (fabsf(marker.roundness - 1.0f) < parameters.roundness_tolerance || !is_outer) {
            marker.round = true;

            // Calculate segment mean intensity
            marker.mean = 0;
            for (int p = queue_old_start; p < queue_end; p++) {
                cache.pos = cache.queue_ptr[p];
                marker.mean += gray_data[cache.pos];
            }
            marker.mean  = marker.mean / marker.size;
            cache.result = true;

            WHYCON_INFO("valid segment found: " << marker.size << " pixels, with size " << cache.width_pixels << " x "
                                                << cache.height_pixels << " with mean " << marker.mean);
        } else {
            WHYCON_ERROR("Segment not round enough (" << marker.roundness << ") width_pixels/height_pixels "
                                                      << cache.width_pixels << " x " << cache.height_pixels << " ctr "
                                                      << marker.x << " " << marker.y << " " << marker.size << " "
                                                      << expected_area_ratio);
        }
    } else {
        // WHYCON_ERROR("Segment too small (" << marker.size << "/" << parameters.min_size << ") at seed_pixel_index "
        //                                    << seed_pixel_index << " with type " << cache.type);
    }
#if FLOODFILL_DEBUG_IMAGE
    // Flood Fill Debug image
    //    Add debug visualization at the end of the function before return:
    if (debug_manager && debug_manager->isEnabled() && cache.result) {
        cv::Mat flood_debug = cv::Mat::zeros(height, width, CV_8UC3);

        // Visualize flood-filled region
        for (int i = queue_old_start; i < queue_end; i++) {
            int pos = context->queue[i];
            int y   = pos / width;
            int x   = pos % width;
            if (x >= 0 && x < width && y >= 0 && y < height) {
                flood_debug.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);  // Yellow for filled pixels
            }
        }

        // Draw bounding boxs
        cv::rectangle(flood_debug, cv::Point(marker.minx, marker.miny), cv::Point(marker.maxx, marker.maxy),
                      cv::Scalar(255, 0, 0), 1);
        cv::circle(flood_debug, cv::Point(marker.x, marker.y), 2, cv::Scalar(0, 0, 255), -1);

        debug_manager->addDebugImage(flood_debug, "Flood Fill Analysis");
    }
#endif
    return cache.result;
}

inline bool whycon::MarkerDetector::validateMarkerPair(const Marker& inner, const Marker& outer) const {
    // Check area ratio
    const float measured_ratio_over_expected =
            (static_cast<float>(outer.size) / static_cast<float>(inner.size)) * inv_areas_ratio;
    if (fabsf(measured_ratio_over_expected - 1.0f) > parameters.ratio_tolerance) {
        WHYCON_DEBUG(
                "Validation failed: Area ratio out of tolerance. Measured/Expected: " << measured_ratio_over_expected);
        return false;
    }

    // Check center distance
    const float center_dx   = fabsf(inner.x - outer.x);
    const float center_dy   = fabsf(inner.y - outer.y);
    const float tolerance_x = parameters.center_distance_tolerance_abs +
                              parameters.center_distance_tolerance_ratio * static_cast<float>(outer.maxx - outer.minx);
    const float tolerance_y = parameters.center_distance_tolerance_abs +
                              parameters.center_distance_tolerance_ratio * static_cast<float>(outer.maxy - outer.miny);

    if (center_dx > tolerance_x || center_dy > tolerance_y) {
        WHYCON_DEBUG("Validation failed: Center distance out of tolerance. dx: "
                     << center_dx << " > " << tolerance_x << " or dy: " << center_dy << " > " << tolerance_y);
        return false;
    }

    return true;
}

void whycon::MarkerDetector::detectMarkerPair(const ImageHandler& image_handler, bool& fast_cleanup_possible,
                                              MarkerPair& result, const Marker& previous_circle,
                                              DebugImageManager* debug_manager) {
    // Reset the member buffers to ensure a clean state for this detection run
    *inner_marker_ = Marker{};
    *outer_marker_ = Marker{};

    const int   width         = image_handler.getWidth();
    const int   height        = image_handler.getHeight();
    const auto& binary_packed = image_handler.getBinaryPacked();

    const int* buffer_ptr = context->buffer.get();
    const int* queue_ptr  = context->queue.get();

    // this allows to differentiate segments found by this detector from others, and know how many segments where
    // found in this call
    initial_segment_id = context->total_segments;

    WHYCON_DEBUG("detector id " << detector_id << " B/W/U " << PIXEL_BLACK << "/" << PIXEL_WHITE << "/" << UNVISITED);
    WHYCON_DEBUG("threshold " << threshold);
    WHYCON_DEBUG("initial segment id " << initial_segment_id);

    int pos              = 0;
    int seed_pixel_index = 0;
    int start            = 0;

    if (previous_circle.valid) {
        WHYCON_DEBUG("starting with previously valid circle at " << previous_circle.x << "," << previous_circle.y);
        seed_pixel_index = (static_cast<int>(previous_circle.y)) * width + static_cast<int>(previous_circle.x);
        start            = seed_pixel_index;
    }

    // Main Detection Loop
    do {
        if ((context->total_segments - initial_segment_id) > MAX_SEGMENTS) {
            WHYCON_DEBUG("reached maximum number of segments");
            break;
        }

        // Check if the pixel has been visited. If not, check its type from the pre-binarized image.
        if (buffer_ptr[seed_pixel_index] == UNVISITED && binary_packed.getPixelLinear(seed_pixel_index) == PIXEL_BLACK) {
            queue_end   = 0;
            queue_start = 0;

            // check if looks like the outer portion of the ring
            if (analyzeMarkerCandidate(image_handler, *outer_marker_, seed_pixel_index, outer_area_ratio, true,
                                       debug_manager)) {
                pos = outer_marker_->y * width + outer_marker_->x;  // jump to the middle of the ring

                WHYCON_DEBUG("found valid outer, looking for white at " << pos
                                                                        << " id: " << context->total_segments - 1);

                // Check the center of the ring from the pre-binarized image
                if (buffer_ptr[pos] == UNVISITED && binary_packed.getPixelLinear(pos) == PIXEL_WHITE) {
                    // check if it looks like the inner portion
                    if (analyzeMarkerCandidate(image_handler, *inner_marker_, pos, inner_area_ratio, false,
                                               debug_manager)) {
                        // it does, now actually check specific properties to see if it is a valid target
                        if (validateMarkerPair(*inner_marker_, *outer_marker_)) {
                            // DEBUG: View Segments
                            if (debug_manager && debug_manager->isEnabled()) {
                                generateSegmentDebugImage(image_handler, debug_manager);
                            }
                            WHYCON_DEBUG("This is a valid marker candidate, computing ellipse parameters...");
                            WHYCON_DEBUG("queue_old_start: " << queue_old_start << ", queue_end: " << queue_end);

                            //* --- Compute  ellipse parameters ---
                            computeEllipseParameters(queue_ptr, queue_old_start, queue_end, *inner_marker_);
                            computeEllipseParameters(queue_ptr, 0, queue_old_start, *outer_marker_);

                            // DEBUG: Draw Ellipse parameters
                            if (debug_manager && debug_manager->isEnabled()) {
                                generateEllipseDebugImage(image_handler, debug_manager);
                            }

                            // Log final ellipse parameters
                            WHYCON_DEBUG("Outer ellipse params: center=("
                                         << outer_marker_->x << "," << outer_marker_->y << ")"
                                         << " m0=" << outer_marker_->m0 << " m1=" << outer_marker_->m1
                                         << " v0=" << outer_marker_->v0 << " v1=" << outer_marker_->v1);
                            // outer.size = size_outer;

                            inner_marker_->bwRatio = static_cast<float>(outer_marker_->size) / inner_marker_->size;
                            WHYCON_DEBUG("inner size " << inner_marker_->size << " outer size " << outer_marker_->size
                                                       << " ratio " << inner_marker_->bwRatio);

                            // Calculate ellipse quality metrics
                            const float circularity  = static_cast<float>(M_PI * 4 * (outer_marker_->m0) *
                                                                         (outer_marker_->m1) / queue_end);
                            const float eccentricity = sqrtf(1.0f - (outer_marker_->m1 * outer_marker_->m1) /
                                                                            (outer_marker_->m0 * outer_marker_->m0));

                            // Generate debug visualization for marker parameters
                            if (debug_manager && debug_manager->isEnabled()) {
                                generateMParamDebugImage(image_handler, debug_manager, eccentricity, circularity);
                            }

                            // Final validation for circularity and eccentricity
                            if (fabsf(circularity - 1.0f) < parameters.circularity_tolerance &&
                                eccentricity < parameters.max_eccentricity) {
                                outer_marker_->valid = inner_marker_->valid = true;

                                // use a new threshold estimate based on current detection
                                threshold = (outer_marker_->mean + inner_marker_->mean) / 2;

                                WHYCON_DEBUG("found inner segment " << context->total_segments - 1);
                                break;
                            }
                        }
                    } else {
                        WHYCON_DEBUG("inner segment not valid");
                        inner_marker_->valid = false;
                    }
                } else {
                    WHYCON_DEBUG("outer segment not valid");
                    outer_marker_->valid = false;
                }
            }
        }

        // Update search position
        seed_pixel_index++;
        if (seed_pixel_index >= len)
            seed_pixel_index = 0;

    } while (seed_pixel_index != start);

    // Pose-processing: angle and threshold logic
    if (inner_marker_->valid) {
        // Calculate angle between inner and outer markers
        float orient = atan2(outer_marker_->y - inner_marker_->y, outer_marker_->x - inner_marker_->x);

        // Calculate orientation from ellipse parameters
        outer_marker_->angle = atan2(outer_marker_->v1, outer_marker_->v0);

        // Normalize the angle to be consistent with the marker orientation
        if (fabs(normalizeAngle(outer_marker_->angle - orient)) > M_PI / 2)
            outer_marker_->angle = normalizeAngle(outer_marker_->angle + M_PI);

        inner_marker_->angle = outer_marker_->angle;  // copy angle to inner marker too
        WHYCON_DEBUG("angle " << outer_marker_->angle << " orient " << orient);

        // Reset the threshold counter on successful detection
        threshold_counter = 0;
    } else {
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

void whycon::MarkerDetector::coverLastDetected(cv::Mat& image) {
    int* queue = context->queue.get();
    for (int i = queue_old_start; i < queue_end; i++) {
        int    pos = queue[i];
        uchar* ptr = image.data + 3 * pos;
        *ptr       = 255;
        ptr++;
        *ptr = 255;
        ptr++;
        *ptr = 255;
    }
}

whycon::MarkerDetector::Marker::Marker() {}

void whycon::MarkerDetector::Marker::draw(cv::Mat& image, const std::string& text, cv::Vec3b color,
                                          float thickness) const {
    for (float e = 0; e < 2 * M_PI; e += 0.01) {
        float fx  = x + cos(e) * v0 * m0 * 2 + v1 * m1 * 2 * sin(e);
        float fy  = y + cos(e) * v1 * m0 * 2 - v0 * m1 * 2 * sin(e);
        int   fxi = static_cast<int>(fx + 0.5);
        int   fyi = static_cast<int>(fy + 0.5);
        if (fxi >= 0 && fxi < image.cols && fyi >= 0 && fyi < image.rows)
            image.at<cv::Vec3b>(fyi, fxi) = color;
    }

    float scale = image.size().width / 1800.0f;
    // float thickness = scale * 3.0;
    // if (thickness < 1) thickness = 1;
    cv::putText(image, text.c_str(), cv::Point(x + 2 * m0 - 100, y + 2 * m1 + 5), cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(color), thickness, cv::LINE_AA);
    cv::line(image, cv::Point(x + v0 * m0 * 2, y + v1 * m0 * 2), cv::Point(x - v0 * m0 * 2, y - v1 * m0 * 2),
             cv::Scalar(color), 1, 8);
    cv::line(image, cv::Point(x + v1 * m1 * 2, y - v0 * m1 * 2), cv::Point(x - v1 * m1 * 2, y + v0 * m1 * 2),
             cv::Scalar(color), 1, 8);
}

whycon::MarkerDetector::DetectionContext::DetectionContext(int _width, int _height) {
    width   = _width;
    height  = _height;
    int len = width * height;
    buffer  = std::unique_ptr<int[]>(new int[len]);
    queue   = std::unique_ptr<int[]>(new int[len]);

    cleanupBuffer();
    reset();
}

void whycon::MarkerDetector::DetectionContext::reset(void) {
    next_detector_id = 0;
    valid_segment_ids.clear();
    total_segments = 0;
}

void whycon::MarkerDetector::DetectionContext::cleanupBuffer(void) {
    WHYCON_INFO("Cleaning the entire buffer.");
    int len = width * height;
    std::memset(buffer.get(), -1, sizeof(int) * len);
}

void whycon::MarkerDetector::DetectionContext::cleanupBuffer(const Marker& c) {
    if (c.valid) {
        WHYCON_INFO("Cleaning buffer region for marker at (" << c.x << "," << c.y << ")");

        // zero only parts modified when detecting 'c'
        int ix = std::max(c.minx - 2, 1);
        int ax = std::min(c.maxx + 2, width - 2);
        int iy = std::max(c.miny - 2, 1);
        int ay = std::min(c.maxy + 2, height - 2);
        for (int y = iy; y < ay; y++) {
            int pos = y * width;
            for (int x = ix; x < ax; x++)
                buffer[pos + x] = UNVISITED;
        }
    }
}

// Function to generate debug images, not used in runtime but useful for debugging
void whycon::MarkerDetector::DetectionContext::debugBuffer(const cv::Mat& image, cv::Mat& out) {
    std::map<int, cv::Vec3b> colors;
    for (int i = 0; i < total_segments; i++)
        colors[i] =
                cv::Vec3b(rand() / static_cast<float>(RAND_MAX) * 255.0, rand() / static_cast<float>(RAND_MAX) * 255.0,
                          rand() / static_cast<float>(RAND_MAX) * 255.0);

    out.create(height, width, CV_8UC3);
    cv::Vec3b*       out_ptr = out.ptr<cv::Vec3b>(0);
    const cv::Vec3b* im_ptr  = image.ptr<cv::Vec3b>(0);
    out                      = cv::Scalar(128, 128, 128);
    for (uint i = 0; i < out.total(); i++, ++out_ptr, ++im_ptr) {
        if (buffer[i] >= 0) {
            *out_ptr = colors[buffer[i]];
        } else {
            int pixel_class = (-(buffer[i] + 1) % 3);
            if (pixel_class == 0)
                *out_ptr = cv::Vec3b(0, 255, 0);  // UNKNOWN
            else if (pixel_class == 1)
                *out_ptr = cv::Vec3b(255, 0, 0);  // WHITE
            else
                *out_ptr = cv::Vec3b(0, 0, 255);  // BLACK
        }
    }
}

void whycon::MarkerDetector::generateSegmentDebugImage(const ImageHandler& image_handler,
                                                       DebugImageManager*  debug_manager) const {
    int width  = image_handler.getWidth();
    int height = image_handler.getHeight();
    int bpp    = image_handler.getBPP();

    // Make a copy of the input image for visualization
    cv::Mat segment_debug = cv::Mat::zeros(height, width, CV_8UC3);
    // Convert raw image for visualization
    cv::Mat temp_image(height, width, (bpp == 3) ? CV_8UC3 : CV_8UC1, image_handler.getData());
    if (bpp == 3) {
        temp_image.copyTo(segment_debug);
    } else {
        cv::cvtColor(temp_image, segment_debug, cv::COLOR_GRAY2BGR);
    }

    // Draw outer segment pixels (queue[0] to queue_old_start)
    for (int p = 0; p < queue_old_start; p++) {
        int pos = context->queue.get()[p];
        int x   = pos % width;
        int y   = pos / width;
        if (x >= 0 && x < segment_debug.cols && y >= 0 && y < segment_debug.rows)
            segment_debug.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);  // Yellow for outer
    }

    // Draw inner segment pixels (queue_old_start to queue_end)
    for (int p = queue_old_start; p < queue_end; p++) {
        int pos = context->queue.get()[p];
        int x   = pos % width;
        int y   = pos / width;
        if (x >= 0 && x < segment_debug.cols && y >= 0 && y < segment_debug.rows)
            segment_debug.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 255);  // Magenta for inner
    }
    // Draw/ Plot all points of outer and inner
    cv::circle(segment_debug, cv::Point((outer_marker_->maxx), outer_marker_->maxy), 2, cv::Scalar(0, 255, 0), 10);
    cv::circle(segment_debug, cv::Point((outer_marker_->minx), outer_marker_->miny), 2, cv::Scalar(0, 255, 0), 10);
    cv::circle(segment_debug, cv::Point((inner_marker_->maxx), inner_marker_->maxy), 2, cv::Scalar(255, 0, 0), 10);
    cv::circle(segment_debug, cv::Point((inner_marker_->minx), inner_marker_->miny), 2, cv::Scalar(255, 0, 0), 10);

    debug_manager->addDebugImage(segment_debug, "Marker Pair Detection");
}

void whycon::MarkerDetector::generateEllipseDebugImage(const ImageHandler& image_handler,
                                                       DebugImageManager*  debug_manager) const {
    int width  = image_handler.getWidth();
    int height = image_handler.getHeight();
    int bpp    = image_handler.getBPP();

    // Make a copy of the input image for visualization
    cv::Mat ellipse_debug = cv::Mat::zeros(height, width, CV_8UC3);
    // Convert raw image for visualization
    cv::Mat temp_image(height, width, (bpp == 3) ? CV_8UC3 : CV_8UC1, image_handler.getData());
    if (bpp == 3) {
        temp_image.copyTo(ellipse_debug);
    } else {
        cv::cvtColor(temp_image, ellipse_debug, cv::COLOR_GRAY2BGR);
    }

    // Draw outer and ellipse in a new opencv imshow window
    cv::ellipse(ellipse_debug, cv::Point(outer_marker_->x, outer_marker_->y),
                cv::Size(outer_marker_->m0, outer_marker_->m1),
                atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI, 0, 360, cv::Scalar(0, 255, 255), 2);
    cv::ellipse(ellipse_debug, cv::Point(inner_marker_->x, inner_marker_->y),
                cv::Size(inner_marker_->m0, inner_marker_->m1),
                atan2(inner_marker_->v1, inner_marker_->v0) * 180 / M_PI, 0, 360, cv::Scalar(255, 0, 255), 2);
    // Draw center points of inner and outer ellipses
    cv::circle(ellipse_debug, cv::Point(outer_marker_->x, outer_marker_->y), 2, cv::Scalar(0, 255, 0), 5);
    cv::circle(ellipse_debug, cv::Point(inner_marker_->x, inner_marker_->y), 2, cv::Scalar(255, 0, 0), 5);

    cv::putText(ellipse_debug, "Outer", cv::Point(outer_marker_->x + 10, outer_marker_->y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    cv::putText(ellipse_debug, "Inner", cv::Point(inner_marker_->x + 10, inner_marker_->y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

    debug_manager->addDebugImage(ellipse_debug, "ellipse_debug");
}

void whycon::MarkerDetector::generateMParamDebugImage(const ImageHandler& image_handler,
                                                      DebugImageManager* debug_manager, float eccentricity,
                                                      float circularity) const {
    // Make a copy of the input image for visualization
    int     bpp            = image_handler.getBPP();
    cv::Mat annotate_debug = cv::Mat::zeros(height_, width_, CV_8UC3);
    cv::Mat temp_image(height_, width_, (bpp == 3) ? CV_8UC3 : CV_8UC1, image_handler.getData());
    if (bpp == 3) {
        temp_image.copyTo(annotate_debug);
    } else {
        cv::cvtColor(temp_image, annotate_debug, cv::COLOR_GRAY2BGR);
    }

    // Draw ellipse and center
    cv::ellipse(annotate_debug, cv::Point(outer_marker_->x, outer_marker_->y),
                cv::Size(outer_marker_->m0, outer_marker_->m1),
                atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI, 0, 360, cv::Scalar(0, 255, 255), 2);
    cv::circle(annotate_debug, cv::Point(outer_marker_->x, outer_marker_->y), 2, cv::Scalar(0, 255, 0), 5);

    // Calculate center distance
    float center_distance = std::sqrt((inner_marker_->x - outer_marker_->x) * (inner_marker_->x - outer_marker_->x) +
                                      (inner_marker_->y - outer_marker_->y) * (inner_marker_->y - outer_marker_->y));

    // Calculate center tolerance values
    float toleranceX =
            parameters.center_distance_tolerance_abs +
            parameters.center_distance_tolerance_ratio * static_cast<float>(outer_marker_->maxx - outer_marker_->minx);
    float toleranceY =
            parameters.center_distance_tolerance_abs +
            parameters.center_distance_tolerance_ratio * static_cast<float>(outer_marker_->maxy - outer_marker_->miny);

    float center_x_diff = std::abs(inner_marker_->x - outer_marker_->x);
    float center_y_diff = std::abs(inner_marker_->y - outer_marker_->y);

    // Prepare annotation texts (one per line)
    char text1[128], text2[128], text3[128], text4[128];
    snprintf(text1, sizeof(text1), "Ecc: %.3f  Round: %.3f", eccentricity, outer_marker_->roundness);
    snprintf(text2, sizeof(text2), "CtrDist: %.1f  CtrThrX: %.1f  CtrThrY: %.1f", center_distance, toleranceX,
             toleranceY);
    snprintf(text4 + strlen(text4), sizeof(text4) - strlen(text4), "  CtrX: %.1f  CtrY: %.1f", center_x_diff,
             center_y_diff);
    snprintf(text3, sizeof(text3), "Circ: %.3f  Size: %d", circularity, outer_marker_->size);

    // Draw text near the marker with vertical offsets
    int base_x     = std::max(0, static_cast<int>(outer_marker_->x) + 10 - 400);
    int base_y     = std::max(0, static_cast<int>(outer_marker_->y) - 10);
    int lineHeight = 15;  // vertical space between lines

    cv::putText(annotate_debug, text1, cv::Point(base_x, base_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 255), 1);
    cv::putText(annotate_debug, text2, cv::Point(base_x, base_y + lineHeight), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 255), 1);
    cv::putText(annotate_debug, text4, cv::Point(base_x, base_y + 2 * lineHeight), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 255), 1);
    cv::putText(annotate_debug, text3, cv::Point(base_x, base_y + 3 * lineHeight), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 255), 1);

    // Optionally, color code the ellipse if it failed a check
    cv::Scalar ellipse_color =
            (fabsf(circularity - 1) < parameters.circularity_tolerance && eccentricity < parameters.max_eccentricity) ?
                    cv::Scalar(0, 255, 0)  // Green for valid
                    :
                    cv::Scalar(0, 0, 255);  // Red for invalid
    cv::ellipse(annotate_debug, cv::Point(outer_marker_->x, outer_marker_->y),
                cv::Size(outer_marker_->m0, outer_marker_->m1),
                atan2(outer_marker_->v1, outer_marker_->v0) * 180 / M_PI, 0, 360, ellipse_color, 2);

    debug_manager->addDebugImage(annotate_debug, "Marker Checks");
}