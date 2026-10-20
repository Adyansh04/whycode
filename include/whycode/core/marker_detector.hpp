#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <cmath>
#include <cstdint>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>
#include <string_view>
#include <vector>

#include "whycode/image/debug_image_manager.hpp"
#include "whycode/image/image_handler.hpp"

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
    double image_gamma    = 1.0;  // Encoding gamma of the grey levels; 1.0 for linear images

    float diameter_ratio_correction = 0.45;
    float ambiguity_noise_px        = 0.3f;  // Edge noise of the code, weighs the pose solutions

    int min_size = 10;
    int max_size = 100 * 100;

    int id_bits          = 6;
    int id_samples       = 720;
    int hamming_distance = 1;

    int  min_marker_pixels = 77;
    bool identify          = true;

    int full_scan_interval = 1;  // Frames between whole-image searches; between them only known
                                 // markers are looked for, near where they were
};

/**
 * @brief Detects circular markers in an image and computes their properties.
 *
 * This class implements a marker detection algorithm that identifies circular markers
 * based on their size, shape, and color. A scanline flood fill on the bit-packed binary image
 * finds connected components, which are checked against the criteria for valid markers.
 */
class MarkerDetector
{
public:
    // static constexpr int PIXEL_BLACK = 0;
    // static constexpr int PIXEL_WHITE = 255;
    static constexpr bool PIXEL_BLACK  = 1;
    static constexpr bool PIXEL_WHITE  = 0;
    static constexpr int  MAX_SEGMENTS = 10000;

    /**
     * @brief Grey level of an edge between a dark and a light region.
     *
     * An edge lies halfway between black and white in linear light. In a gamma-encoded image that
     * grey level is above the mean of the two, and thresholding at the mean shrank outlines ~1%.
     *
     * @param dark Grey level of the dark side (0-255).
     * @param light Grey level of the light side (0-255).
     * @param gamma Encoding gamma of the grey levels; 1.0 for linear images.
     * @return The edge's grey level.
     */
    static int edgeThreshold(int dark, int light, double gamma) noexcept
    {
        if (gamma == 1.0)
        {
            return (dark + light) / 2;
        }
        const double mid = 0.5 * (std::pow(dark / 255.0, gamma) + std::pow(light / 255.0, gamma));
        return static_cast<int>(std::lround(255.0 * std::pow(mid, 1.0 / gamma)));
    }

    class Marker;
    class DetectionContext;

    /**
     * @brief How far a seed's segment got through the checks: the one it failed, or MARKER.
     */
    enum class Check : uint8_t
    {
        RING,    // The black segment's size or roundness
        CENTRE,  // No unvisited white pixel at the ring's centre
        DISC,    // The white centre segment's size
        PAIR,    // Area ratio or concentricity of ring and disc
        SHAPE,   // Circularity or eccentricity of the fitted outline
        MARKER   // Passed every check
    };

    /**
     * @brief Constructs a marker detector for a shared context.
     * @param context Shared detection context (visited pixels, segment runs), which sets the size.
     * @param parameters Detection parameters (see DetectorParameters).
     */
    MarkerDetector(
        DetectionContext* context, const DetectorParameters& parameters = DetectorParameters());
    ~MarkerDetector();

    // Rule of Five: Explicitly define move semantics and delete copy semantics
    // because this class manages unique_ptr members.
    MarkerDetector(const MarkerDetector&)            = delete;
    MarkerDetector& operator=(const MarkerDetector&) = delete;
    MarkerDetector(MarkerDetector&&)                 = default;
    MarkerDetector& operator=(MarkerDetector&&)      = default;

    /**
     * @brief Tests one seed pixel: flood-fills its segment and checks for a whole marker.
     * @param image_handler The image handler containing the frame to process.
     * @param seed_pixel_index Linear index of an unvisited black pixel.
     * @return The check the segment failed, or Check::MARKER if it is a valid marker.
     */
    Check examineSeed(const ImageHandler& image_handler, int seed_pixel_index);

    /**
     * @brief Flood-fills the segment of a seed pixel and checks its size and roundness.
     * @param image_handler The image handler containing the frame to process.
     * @param marker Output: marker properties for the segment.
     * @param seed_pixel_index Linear index of an unvisited pixel of the segment.
     * @param expected_area_ratio Expected segment area over its bounding box area.
     * @param is_outer True for the outer ring; the inner disc skips the roundness check.
     * @return True if the segment is a candidate for its part of a marker.
     */
    bool analyzeMarkerCandidate(
        const ImageHandler& image_handler, whycon::MarkerDetector::Marker& marker,
        int seed_pixel_index, float expected_area_ratio, bool is_outer);

    /**
     * @brief Draws the key to the candidates panel's colours.
     * @param image The candidates panel.
     */
    static void drawCandidateKey(cv::Mat& image);

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
        int mean = 0;            // Mean grey level, set once the pair is a valid marker
        int type = 0;            // Segment type (BLACK/WHITE)

        bool round = false;  // True if the segment is round
        bool valid = false;  // True if the marker is valid

        /**
         * @brief Draws the marker ellipse and optional text on an image.
         * @param image Image to draw on.
         * @param text Optional text, one image line per '\n'-separated line, placed below the
         * ellipse and kept inside the image.
         * @param color Color for drawing.
         * @param thickness Line thickness.
         */
        void draw(
            cv::Mat& image, std::string_view text = {}, cv::Vec3b color = cv::Vec3b(0, 255, 0),
            float thickness = 1) const;
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
     * @brief Image region a search is limited to, half-open: [x0, x1) x [y0, y1).
     */
    struct SearchWindow
    {
        int x0, y0, x1, y1;
    };

    /**
     * @brief Detects a marker in the given image.
     * @param image_handler The image handler containing the frame to process.
     * @param result Output: MarkerPair containing both inner and outer markers.
     * @param previous_circle Optional previous detection, where the search starts.
     * @param debug_manager Optional debug panels; every examined segment is drawn on the
     * candidates panel while it is wanted.
     * @param window Optional region to search instead of the whole image.
     */
    void detectMarkerPair(
        const ImageHandler& image_handler, MarkerPair& result,
        const Marker&      previous_circle = whycon::MarkerDetector::Marker(),
        DebugImageManager* debug_manager = nullptr, const SearchWindow* window = nullptr);

private:
    DetectorParameters parameters;

    // The last segment's runs are [span_start_, span_end_) of the context's list
    size_t span_start_ = 0;
    size_t span_end_   = 0;

    float diameter_ratio =
        0.0f;  // The ratio of the inner marker's diameter to the outer marker's diameter.
    float outer_area_ratio = 0.0f;  // Expected area ratio of outer marker ring
    float inner_area_ratio = 0.0f;  // Expected area ratio of inner marker disk
    float areas_ratio      = 0.0f;  // Ratio between outer and inner marker areas
    float inv_areas_ratio  = 0.0f;  //  Inverse of areas_ratio for faster computations

    int threshold         = 128;  // Initial intensity threshold for detection
    int threshold_counter = 0;    // Counter for tracking adjustments to the threshold

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
     * @brief Computes ellipse parameters (center, axes, orientation) from a segment's runs.
     *
     * @param begin First of the segment's runs in the context's list.
     * @param end One past the segment's last run.
     * @param[out] marker Output marker to store computed ellipse parameters.
     */
    void computeEllipseParameters(size_t begin, size_t end, Marker& marker) const;

    // Searches the seeds from the previous centre, over the window or the whole image. DRAW
    // paints every examined segment on candidates; it is a template parameter so the plain search
    // tests nothing per seed. Out of line, so each instantiation is optimised on its own: inlined
    // together into detectMarkerPair, the row search stopped being inlined (+0.8% instructions).
    template <bool DRAW>
    [[gnu::noinline]] void searchSeeds(
        const ImageHandler& image_handler, int seed_pixel_index, const SearchWindow* window,
        cv::Mat* candidates);

    // Paints the last examined segment on the candidates panel, with its values against the limits
    void drawCandidate(cv::Mat& image, Check check) const;

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * @param a Input angle (radians).
     * @return Normalized angle.
     */
    inline float normalizeAngle(float a);

public:
    /**
     * @brief Context for marker detection (visited pixels, segment runs).
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
         * @brief Marks every pixel unvisited, and the padding bits past each row visited.
         */
        void cleanupBuffer(void);

        /**
         * @brief Resets context state (segment IDs, etc).
         */
        void reset(void);

        // A run of one segment's pixels on one row, x0 to x1 inclusive
        struct Span
        {
            int y, x0, x1;
        };

        // One bit per pixel in PackedBinaryImage's layout, set once a segment has taken the pixel.
        // The padding bits past each row stay set, so no segment runs off the image (FreePixels).
        std::vector<uint8_t>    visited;
        std::unique_ptr<Span[]> spans;  // Runs of the segments examined from the current seed
        int                     width = 0, height = 0, stride = 0;

        int next_detector_id = 0;
        int total_segments   = 0;
    };
};
}  // namespace whycon
#endif  // MARKER_DETECTOR_HPP