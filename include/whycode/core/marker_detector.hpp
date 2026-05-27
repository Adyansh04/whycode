#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include <unordered_set>
#include <xsimd/xsimd.hpp>

#include "whycode/image/debug_image_manager.hpp"
#include "whycode/image/image_handler.hpp"

template <typename T>
inline constexpr T wMax(T a, T b)
{
    return (a > b ? a : b);
}

template <typename T>
inline constexpr T wMin(T a, T b)
{
    return (a < b ? a : b);
}
namespace whycon
{
/**
 * @brief Parameters for marker detection and validation
 *
 * These parameters control the minimum/maximum size, roundness, ratio, and other
 * geometric and photometric constraints for valid marker detection.
 */
struct DetectorParameters
{
    double center_distance_tolerance_ratio = 0.1;
    double center_distance_tolerance_abs   = 5;
    double roundness_tolerance             = 0.3;
    double circularity_tolerance           = 0.01;
    double ratio_tolerance                 = 1.0;
    double max_eccentricity                = 1.0;

    double inner_diameter = 0.050;
    double outer_diameter = 0.122;

    float diameter_ratio_correction = 0.45;
    float variance_threshold        = 0.35;

    int min_size = 10;
    int max_size = 100 * 100;

    int id_bits          = 6;
    int id_samples       = 720;
    int hamming_distance = 1;

    int  min_marker_pixels = 77;
    bool identify          = true;
};

/**
 * @brief Detects circular markers in an image and computes their properties.
 *
 * This class implements a marker detection algorithm that identifies circular markers
 * based on their size, shape, and color. It uses a queue-based approach to analyze connected
 * components in the image and determine if they meet the criteria for valid markers.
 */
class MarkerDetector
{
public:
    // static constexpr int PIXEL_BLACK = 0;
    // static constexpr int PIXEL_WHITE = 255;
    static constexpr bool PIXEL_BLACK  = 1;
    static constexpr bool PIXEL_WHITE  = 0;
    static constexpr int  UNVISITED    = -1;
    static constexpr int  MAX_SEGMENTS = 10000;

    class Marker;
    class DetectionContext;

    /**
     * @brief Constructs a marker detector for a given image size and context.
     * @param width Image width in pixels.
     * @param height Image height in pixels.
     * @param context Shared detection context (buffer, queue, etc).
     * @param parameters Detection parameters (see DetectorParameters).
     */
    MarkerDetector(
        int width, int height, DetectionContext* context,
        const DetectorParameters& parameters = DetectorParameters());
    ~MarkerDetector();

    // Rule of Five: Explicitly define move semantics and delete copy semantics
    // because this class manages unique_ptr members.
    MarkerDetector(const MarkerDetector&)            = delete;
    MarkerDetector& operator=(const MarkerDetector&) = delete;
    MarkerDetector(MarkerDetector&&)                 = default;
    MarkerDetector& operator=(MarkerDetector&&)      = default;

    /**
     * @brief Analyzes a connected component to determine if it's a valid marker.
     * @param image Input image (BGR).
     * @param marker Output: marker properties for the segment.
     * @param seed_pixel_index Index of the seed pixel for the segment.
     * @param areaRatio Expected area ratio for the segment.
     * @return True if the segment is a valid marker.
     */
    bool analyzeMarkerCandidate(
        const ImageHandler& image_handler, whycon::MarkerDetector::Marker& marker,
        int seed_pixel_index, float expected_area_ratio, bool is_outer,
        DebugImageManager* debug_manager = nullptr);

    /**
     * @brief Covers the last detected marker in the image (for visualization).
     * @param image Input/output image (BGR).
     */
    void coverLastDetected(cv::Mat& image);

    // Debug image generation helpers
    void generateSegmentDebugImage(
        const ImageHandler& image_handler, DebugImageManager* debug_manager) const;
    void generateEllipseDebugImage(
        const ImageHandler& image_handler, DebugImageManager* debug_manager) const;
    void generateMParamDebugImage(
        const ImageHandler& image_handler, DebugImageManager* debug_manager, float eccentricity,
        float circularity) const;

    /**
     * @brief Returns the current intensity threshold value.
     */
    int getCurrentThreshold(void) const;

    /**
     * @brief Stores the properties of a detected marker (ellipse).
     */
    class Marker
    {
    public:
        /**
         * @brief Default constructor. Initializes marker as invalid.
         */
        Marker(void);

        float x = 0.0f, y = 0.0f;    // Center coordinates of the marker (pixels)
        float roundness = 0.0f;      // Roundness metric
        float bwRatio   = 0.0f;      // Black/white area ratio
        float m0 = 0.0f, m1 = 0.0f;  // Ellipse axis dimensions
        float v0 = 0.0f, v1 = 0.0f;  // Ellipse axis orientation
        float angle = 0.0f;          // Rotation angle around marker's normal vector

        int ID          = -1;    // Whycode ID
        int tracking_id = -1;    // Unique tracking identifier for the marker
        int detector_id = -1;    // Index of the detector that found this marker
        int size        = 0;     // Number of pixels in the segment
        int maxx = 0, maxy = 0;  // Maximum x,y coordinates
        int minx = 0, miny = 0;  // Minimum x,y coordinates
        int mean = 0;            // Mean intensity of the segment
        int type = 0;            // Segment type (BLACK/WHITE)

        bool round = false;  // True if the segment is round
        bool valid = false;  // True if the marker is valid

        /**
         * @brief Draws the marker ellipse and optional text on an image.
         * @param image Image to draw on.
         * @param text Optional text to display.
         * @param color Color for drawing.
         * @param thickness Line thickness.
         */
        void draw(
            cv::Mat& image, const std::string& text = std::string(),
            cv::Vec3b color = cv::Vec3b(0, 255, 0), float thickness = 1) const;
    };

    /**
     * @brief Stores a pair of detected markers (inner and outer ellipses).
     */
    struct MarkerPair
    {
        Marker inner;          // Inner ellipse marker
        Marker outer;          // Outer ellipse marker
        bool   valid = false;  // True if detection was successful
    };

    /**
     * @brief Detects a marker in the given image.
     * @param image_handler The image handler containing the frame to process.
     * @param fast_cleanup_possible Output: true if buffer cleanup can be optimized.
     * @param result Output: MarkerPair containing both inner and outer markers.
     * @param previous_circle Optional previous detection for windowed search.
     * @param debug_manager Optional manager for collecting debug images.
     */
    void detectMarkerPair(
        const ImageHandler& image_handler, bool& fast_cleanup_possible, MarkerPair& result,
        const Marker&      previous_circle = whycon::MarkerDetector::Marker(),
        DebugImageManager* debug_manager   = nullptr);

    /**
     * @brief Returns the last detected outer marker.
     * @return Reference to the last detected outer marker.
     */

private:
    /**
     * @brief A cache for temporary variables used during ellipse parameter calculation.
     *
     * This struct holds all intermediate variables for the computeEllipseParameters function.
     * By making it a class member, we avoid reallocating these variables on the stack
     * for every detected segment, improving performance and cache locality.
     */
    struct EllipseComputationCache
    {
        // Accumulators for sums and moments
        float sum_x = 0.0f, sum_y = 0.0f;
        float sum_xx = 0.0f, sum_xy = 0.0f, sum_yy = 0.0f;
        int   num_points = 0;

        float mean_x = 0.0f, mean_y = 0.0f;

        float cov_xx = 0.0f, cov_xy = 0.0f, cov_yy = 0.0f;
        float trace = 0.0f, det = 0.0f;
        float temp = 0.0f, sqrt_term = 0.0f;
        float lambda1 = 0.0f, lambda2 = 0.0f;
        float norm = 0.0f;

        /**
         * @brief Resets all cached values to zero before a new computation.
         */
        void reset()
        {
            sum_x = sum_y = 0.0f;
            sum_xx = sum_xy = sum_yy = 0.0f;
            num_points               = 0;
            mean_x = mean_y = 0.0f;
            cov_xx = cov_xy = cov_yy = 0.0f;
            trace = det = 0.0f;
            temp = sqrt_term = 0.0f;
            lambda1 = lambda2 = 0.0f;
            norm              = 0.0f;
        }
    };

    /**
     * @brief A cache for temporary variables used during flood-fill analysis.
     *
     * This struct holds all intermediate variables for the analyzeMarkerCandidate function.
     * By making it a class member, we avoid reallocating these variables on the stack
     * for every candidate segment, improving performance and cache locality.
     */
    struct CandidateAnalysisCache
    {
        const uchar* gray_data  = nullptr;
        int*         buffer_ptr = nullptr;
        int*         queue_ptr  = nullptr;

        float width_inv    = 0.0f;
        int   width_pixels = 0, height_pixels = 0;
        int   position = 0;
        int   pos      = 0;
        bool  type     = 0;
        int   maxx = 0, maxy = 0, minx = 0, miny = 0;
        int   pixel_class = 0;
        int   segment_id  = 0;
        int   position_x  = 0;
        int   position_y  = 0;
        bool  result      = false;

        /**
         * @brief Resets all cached values to zero before a new computation.
         */
        void reset() noexcept
        {
            width_pixels = height_pixels = 0;
            position                     = 0;
            pos                          = 0;
            result                       = false;
            type                         = 0;
            maxx = maxy = minx = miny = 0;
            pixel_class               = 0;
            segment_id                = 0;
            width_inv                 = 0.0f;
            gray_data                 = nullptr;
            buffer_ptr                = nullptr;
            queue_ptr                 = nullptr;
            position_x = position_y = 0;
        }
    };

    DetectorParameters              parameters;
    mutable EllipseComputationCache ellipse_cache_;
    mutable CandidateAnalysisCache  analysis_cache_;

    // SIMD batch types for ellipse parameter computation
    using batch_int   = xsimd::batch<int>;
    using batch_float = xsimd::batch<float>;
    static constexpr std::size_t simd_size_ =
        batch_int::size;  // Number of elements in a SIMD batch

    // Pre-allocated SIMD vectors (mutable for const functions)
    mutable batch_float sum_x_vec_;
    mutable batch_float sum_y_vec_;
    mutable batch_float sum_xx_vec_;
    mutable batch_float sum_xy_vec_;
    mutable batch_float sum_yy_vec_;
    mutable batch_int   width_vec_;

    int width_ = 0, height_ = 0;  // Stores the image width and height
    int len = 0;                  // The total number of pixels in the image (width * height)
    int siz = 0;                  // The total data size for the image, len*3 (for 3-channel image).

    float diameter_ratio =
        0.0f;  // The ratio of the inner marker's diameter to the outer marker's diameter.
    float outer_area_ratio = 0.0f;  // Expected area ratio of outer marker ring
    float inner_area_ratio = 0.0f;  // Expected area ratio of inner marker disk
    float areas_ratio      = 0.0f;  // Ratio between outer and inner marker areas
    float inv_areas_ratio  = 0.0f;  //  Inverse of areas_ratio for faster computations

    int threshold_step    = 0;
    int threshold         = 128;  // Initial intensity threshold for detection
    int threshold_counter = 0;    // Counter for tracking adjustments to the threshold

    int queue_start     = 0;  // Current start position in the queue
    int queue_end       = 0;  // Current end position in the queue
    int queue_old_start = 0;  // Previous start position in the queue
    int num_segments    = 0;  // Number of segments processed

    DetectionContext* context            = nullptr;
    int               detector_id        = 0;
    int               initial_segment_id = 0;

    std::unique_ptr<Marker> inner_marker_;
    std::unique_ptr<Marker> outer_marker_;

    /**
     * @brief Modifies threshold to improve detection on next attempt.
     */
    void adjustThreshold(void);

    /**
     * @brief Validates if a pair of inner and outer markers form a valid Whycon/WhyCode target.
     * @param inner The detected inner circle candidate.
     * @param outer The detected outer ring candidate.
     * @return True if the marker pair is valid based on geometric and area constraints.
     */
    inline bool validateMarkerPair(const Marker& inner, const Marker& outer) const;

    /**
     * @brief Computes ellipse parameters (center, axes, orientation) from a set of pixel indices.
     *
     * @param queue Array of pixel indices in the segment.
     * @param start Start index in the queue.
     * @param end End index in the queue.
     * @param width Width of the image (for coordinate conversion).
     * @param[out] marker Output marker to store computed ellipse parameters.
     */
    void computeEllipseParameters(const int* queue, int start, int end, Marker& marker) const;

    // Compute ellipse statistics using SIMD acceleration
    void computeEllipseStatsSIMD(
        const int* queue, int start, int end, EllipseComputationCache& cache) const;

    // Fall back to scalar computation for small segments or when SIMD is unavailable
    void computeEllipseStatsScalar(
        const int* queue, int start, int end, EllipseComputationCache& cache) const;

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * @param a Input angle (radians).
     * @return Normalized angle.
     */
    inline float normalizeAngle(float a);

public:
    /**
     * @brief Context for marker detection (shared buffer, queue, etc).
     */
    class DetectionContext
    {
    public:
        /**
         * @brief Constructs a detection context for a given image size.
         * @param _width Image width.
         * @param _height Image height.
         */
        DetectionContext(int _width, int _height);

        /**
         * @brief Visualizes the buffer state for debugging.
         * @param image Input image.
         * @param img Output debug image.
         */
        void debugBuffer(const cv::Mat& image, cv::Mat& img);

        /**
         * @brief Resets the buffer to initial state.
         */
        void cleanupBuffer(void);

        /**
         * @brief Resets the buffer in the region of a given marker.
         * @param c Marker whose region to reset.
         */
        void cleanupBuffer(const Marker& c);

        /**
         * @brief Resets context state (segment IDs, etc).
         */
        void reset(void);

        std::unique_ptr<int[]> buffer;  // Pointer to the buffer data
        std::unique_ptr<int[]> queue;   // Pointer to the queue data
        int                    width = 0, height = 0;

        int                     next_detector_id = 0;
        std::unordered_set<int> valid_segment_ids;
        int                     total_segments = 0;
    };
};
}  // namespace whycon
#endif  // MARKER_DETECTOR_HPP