/*
 * whycon_localization.hpp
 * ---------------------------------------------
 * This header defines the LocalizationSystem class and supporting structures for marker detection
 * and 3D pose estimation in the WhyCon/WhyCode system. It provides the interface for detecting
 * circular markers in an image, estimating their 3D pose using camera calibration parameters, and
 * retrieving marker information.
 * ---------------------------------------------
 */

#ifndef WHYCON_LOCALIZATION_H
#define WHYCON_LOCALIZATION_H

#include <array>
#include <complex>
#include <opencv2/opencv.hpp>
#include <string_view>

#include "whycode/core/CNecklace.hpp"
#include "whycode/core/multi_marker_detector.hpp"
#include "whycode/image/debug_image_manager.hpp"
#include "whycode/image/image_handler.hpp"
#include "xsimd/xsimd.hpp"

namespace whycon
{

// Helper types for aligned storage
template <typename T>
using aligned_vec = std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

/**
 * @brief Utility class for quaternion operations.
 * @brief Utility class for quaternion operations.
 *
 * Handles quaternion mathematics including creation, normalization,
 * multiplication, and conversion to Euler angles.
 */
class Quaternion
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    Quaternion() = default;
    Quaternion(float x_, float y_, float z_, float w_)
      : x(x_)
      , y(y_)
      , z(z_)
      , w(w_)
    {}

    // Create quaternion from axis-angle representation
    static Quaternion fromAxisAngle(const cv::Vec3f& axis, float angle)
    {
        float s = std::sin(angle / 2.0f);
        float c = std::cos(angle / 2.0f);
        return Quaternion(axis[0] * s, axis[1] * s, axis[2] * s, c);
    }

    // Hamilton product of two quaternions
    Quaternion operator*(const Quaternion& q) const
    {
        // clang-format off
        return Quaternion(  
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        );
        // clang-format on
    }

    // Normalize the quaternion to unit length
    void normalize()
    {
        float norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (std::abs(norm - 1.0f) > 1e-8f)
        {
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    }

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    cv::Vec3f toEulerAngles() const
    {
        cv::Vec3f euler;

        // Roll (x-axis rotation)
        euler[0] = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp);
        else
            euler[1] = std::asin(sinp);

        // Yaw (z-axis rotation)
        euler[2] = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

        return euler;
    }
};

/**
 * @brief Stores the possible ellipse centers and normal/position solutions for a detected marker.
 *
 * The ellipse detected in the image can correspond to two possible 3D orientations (due to
 * ambiguity in projection). This struct stores both possible solutions for the center and normal
 * vector of the marker in camera coordinates.
 */
struct EllipseCenters
{
    std::array<float, 2>
        u;  // x-coordinates of the ellipse center in the undistorted image (2 solutions)
    std::array<float, 2>
        v;  // y-coordinates of the ellipse center in the undistorted image (2 solutions)
    std::array<std::array<float, 3>, 2>
        n;  // Normal vectors of the marker's plane in camera coordinates (2 solutions)
    std::array<std::array<float, 3>, 2>
        t;  // Position vectors of the marker's center in camera coordinates (2 solutions)
};

/**
 * @brief Parameters for segment detection in the WhyCon system.
 *
 * This struct holds parameters for detecting segments in the WhyCon markers, including their
 * position, major and minor axis lengths, and variances.
 */
struct SegmentParameters
{
    float x, y;    // Center coordinates
    float m0, m1;  // Axes Lengths
    float v0, v1;  // Orientation Vector
};

/**
 * @brief Main class for marker localization and 3D pose estimation.
 *
 * This class manages the detection of multiple circular markers in an image, applies camera
 * calibration parameters, and estimates the 3D pose (position and orientation) of each marker. It
 * uses MultiMarkerDetector for image processing and provides methods to retrieve marker information
 * and estimated poses.
 */
class LocalizationSystem
{
public:
    /**
     * @brief Stores the estimated 3D pose of a marker.
     *
     * Contains both Euler angles and quaternion representation for orientation.
     */
    struct MarkerPose
    {
        cv::Vec3f  pos;  // 3D position of the marker in camera coordinates (x, y, z)
        cv::Vec3f  rot;  // Orientation as Euler angles: pitch, roll, yaw (in radians)
        Quaternion orientation;

        // The mirror solution the same outline allows, and this frame's evidence between the two
        cv::Vec3f  alt_pos;
        Quaternion alt_orientation;
        float      ambiguity_llr          = 0.0f;  // Log-odds of this solution over the mirror one
        float      ambiguity_signal_px    = 0.0f;  // Image distance between the two marker centres
        float      orientation_confidence = 0.5f;  // Probability that this is the right solution

        // Euler Angles
        float roll = 0, pitch = 0, yaw = 0;

        // ID Fields
        float angle       = 0.0;    // Marker Surface angle
        int   ID          = -1;     // Decoder marker ID
        int   tracking_id = -1;     // Unique tracking identifier for the marker
        bool  id_valid    = false;  // Whether ID decoding was successful.

        /** @brief Reports the mirror solution instead of the current one. */
        void swapSolutions() noexcept
        {
            std::swap(pos, alt_pos);
            std::swap(orientation, alt_orientation);
            rot   = orientation.toEulerAngles();
            roll  = rot[0];
            pitch = rot[1];
            yaw   = rot[2];
        }
    };

    // Struct to hold all signal analysis data for a single solution
    struct SignalAnalysisData
    {
        aligned_vec<float> x_coords;  // X coordinates along ellipse (aligned)
        aligned_vec<float> y_coords;  // Y coordinates along ellipse (aligned)
        aligned_vec<float> signal;    // Raw signal values from image (aligned)
        aligned_vec<float> smooth;    // Binarized signal values (aligned)
        std::vector<char>  code;      // Raw bit code with null terminator

        // Analysis results
        float mean    = 0.0f;  // Binarisation threshold: the ring's mean signal
        int   max_idx = 0;     // Sample at the centre of the first code segment

        // Constructor to initialize the code vector with proper size
        SignalAnalysisData(int id_bits)
          : x_coords(MAX_ID_SAMPLES)
          , y_coords(MAX_ID_SAMPLES)
          , signal(MAX_ID_SAMPLES)
          , smooth(MAX_ID_SAMPLES)
          , code(id_bits * 4 + 1, '\0')
        {}

        // Default constructor for when id_bits is not yet known
        SignalAnalysisData()
          : x_coords(MAX_ID_SAMPLES)
          , y_coords(MAX_ID_SAMPLES)
          , signal(MAX_ID_SAMPLES)
          , smooth(MAX_ID_SAMPLES)
          , code(24 + 1, '\0')
        {}
    };

    /**
     * @brief Constructs the localization system with camera parameters and detection settings.
     * @param targets Number of markers to detect.
     * @param width Image width in pixels.
     * @param height Image height in pixels.
     * @param K Camera intrinsic matrix (3x3).
     * @param dist_coeff Camera distortion coefficients (1x5 or 1x8).
     * @param parameters Marker detection parameters (see DetectorParameters).
     *
     * Initializes the detector and stores camera calibration for later use in pose estimation.
     */
    LocalizationSystem(
        int targets, int width, int height, const cv::Mat& K, const cv::Mat& dist_coeff,
        const whycon::DetectorParameters& parameters = DetectorParameters());

    ~LocalizationSystem();
    /**
     * @brief Detects and localizes all markers in the given image.
     * @param image_handler The image handler containing the frame to process.
     * @param reset If true, resets detection state (searches whole image).
     * @param debug_manager Optional manager for collecting debug images.
     * @return True if all markers were detected and localized.
     */
    bool localizeMarkers(
        ImageHandler& image_handler, bool reset, DebugImageManager* debug_manager = nullptr);

    /**
     * @brief Estimates the 3D pose of a marker by index.
     * @param id Index of the marker (0-based).
     * @return Estimated pose (position and orientation) of the marker.
     *
     * Uses the detected marker's ellipse parameters and camera calibration to compute the pose.
     */
    void estimateMarkerPose(int id, MarkerPose& result) const;

    /**
     * @brief Estimates the 3D pose of a marker given its outer and inner ellipse detections.
     * @param image_handler The image handler for the current frame.
     * @param outer_marker Outer ellipse marker parameters.
     * @param inner_marker Inner ellipse marker parameters.
     * @param debug_manager Optional manager for collecting debug images.
     * @param result Output: Estimated pose (position and orientation) of the marker.
     *
     * Resolves ambiguity in orientation using both ellipses.
     */
    void estimateMarkerPose(
        const ImageHandler& image_handler, const MarkerDetector::Marker& outer_circle,
        const MarkerDetector::Marker& inner_circle, MarkerPose& result,
        DebugImageManager* debug_manager = nullptr);

    /**
     * @brief Retrieves the detected marker by index.
     * @param id Index of the marker (0-based).
     * @return Reference to the detected marker (inner ellipse).
     */
    const MarkerDetector::Marker& getMarkerByID(int id);

    /**
     * @brief Retrieves the detected outer marker by index.
     * @param id Index of the marker (0-based).
     * @return Reference to the detected outer marker (outer ellipse).
     */
    const MarkerDetector::Marker& getOuterMarkerByID(int id);

    /**
     * @brief Gets the number of successfully detected markers.
     * @return Number of valid markers detected in the last frame.
     */
    inline int getDetectedMarkerCount() const noexcept
    {
        int count = 0;
        for (int i = 0; i < targets; i++)
        {
            if (detector.circles[i].valid)
            {
                count++;
            }
        }
        return count;
    }

    /**
     * @brief Checks if a specific marker was detected.
     * @param id Index of the marker to check.
     * @return True if the marker at index id was successfully detected.
     */
    inline bool isMarkerDetected(int id) const noexcept
    {
        if (id < 0 || id >= targets)
        {
            return false;  // Invalid id, marker cannot be detected.
        }

        return detector.circles[id].valid;
    }

    MultiMarkerDetector detector;  // Detector for multiple circular markers

private:
    // Constants
    static constexpr int MAX_ID_SAMPLES = 720;

    // Pose ambiguity cue: rings nearer the centre than the ID ring, all inside the code's teeth
    // (0.342-0.600 of the outer radius)
    static constexpr std::array<float, 2> CUE_RING_RATIOS = { 0.40f, 0.47f };
    static constexpr int                  CUE_GROUPS =
        2 * (static_cast<int>(CUE_RING_RATIOS.size()) + 1);  // ring x polarity

    // Outlines from this many pixels read codes whose edges spread under 0.3 about the segment
    // grid (p99.9); a wider spread there is clutter, not a code
    static constexpr int    REGULAR_CODE_PIXELS = 300;
    static constexpr double MAX_CODE_SPREAD     = 0.5;

    // Debug tile: each code segment's bit is drawn this many code-ring radii out, in the black ring
    static constexpr float CODE_LABEL_RADIUS = 1.55f;

    // Sub-pixel outline: edge search along normals of the moment ellipse
    static constexpr int   OUTLINE_RAYS       = 64;
    static constexpr int   MIN_OUTLINE_POINTS = 24;    // Fewer, and the moment ellipse is kept
    static constexpr float OUTLINE_STEP_PX    = 0.5f;  // Profile sampling step
    static constexpr float OUTLINE_INSIDE_PX  = 1.5f;  // Search starts this far inside the outline
    static constexpr float OUTLINE_OUTSIDE_PX = 2.5f;  // and ends this far outside

    // LUT structure for fast trig calculations
    struct TrigLUTAligned
    {
        aligned_vec<float> cosv;
        aligned_vec<float> sinv;
        int                n = 0;
    };

    /**
     * @brief One pose solution's marker plane in the optical camera frame.
     *
     * Back-projects image points onto the plane, in polar coordinates about the marker centre.
     */
    struct PlaneFrame
    {
        cv::Vec3d n, t;    // Plane normal and marker centre
        cv::Vec3d e1, e2;  // In-plane axes
        double    nt;      // n . t
    };

    /**
     * @brief A cache for temporary variables used during pose calculation.
     *
     * This struct holds all intermediate variables for the calcEllipseCenters and calcEigen
     * functions. By making it a class member, we avoid reallocating these variables on the stack
     * for every marker, improving performance and cache locality.
     */
    struct PoseCalculationCache
    {
        // From calcEllipseCenters
        double x, y, x1, x2, y1, y2, major, minor, v0, v1;
        double sx1, sx2, sy1, sy2;
        double a, b, c, d, e, f;
        double conic_matrix_data[9];

        // From calcEigen
        cv::Mat eigenvalues;
        cv::Mat eigenvectors;
        double  L0, L1, L2;
        float   c0, c0x, c0y, c0z;
        float   c1, c1x, c1y, c1z;
        float   c2;
        float   n0, n1, n2, t0, t1, t2;

        // For reprojection in calcEigen
        std::vector<cv::Point3f> pt3d;
        std::vector<cv::Point2f> pt2d;

        // Constructor to pre-size vectors and avoid reallocation
        PoseCalculationCache()
          : pt3d(1)
          , pt2d(1)
        {}
    };

    // Camera parameters
    struct CameraParameters
    {
        double                focal_length_x;     // fx
        double                focal_length_y;     // fy
        double                principal_point_x;  // cx
        double                principal_point_y;  // cy
        std::array<double, 6> distortion_coeffs;  // k1, k2, p1, p2, k3, ...
    };

    // System configuration
    int   targets;          // Number of markers to detect
    int   width;            // Image width in pixels
    int   height;           // Image height in pixels
    float circle_diameter;  // Physical diameter of the marker (used for pose estimation)

    // ID detection parameters
    bool identify_enabled_;  // Whether to identify markers
    int  id_bits_;           // Number of bits in the marker ID
    int  id_samples_;        // Number of samples to take for ID decoding
    int  segment_width_;     // Width of the segments for ID decoding
    int  hamming_distance_;  // Hamming distance for error correction

    // Threshold values
    double image_gamma_;
    float  ambiguity_noise_px_;
    int    min_marker_pixels_;
    float  diameter_ratio_correction_;

    float marker_angle_;  //

    // Camera calibration
    cv::Mat          K;              // Camera intrinsic matrix (3x3)
    cv::Mat          dist_coeff;     // Camera distortion coefficients
    CameraParameters camera_params;  // Structured camera parameters
    bool             has_distortion_ = false;

    std::unique_ptr<CNecklace> decoder_;  // ID Decoder for WhyCode markers

    // Cached state variables for computation
    mutable std::array<SignalAnalysisData, 2> analysis_data_;
    mutable EllipseCenters                    ellipse_centers_buffer_;
    mutable PoseCalculationCache              pose_calc_cache_;

    // Per solution: the marker plane, and per ring and polarity the sums of w * exp(i k angle)
    // and of w over the edges, with w the squared plane radius and k the number of code segments
    mutable std::array<PlaneFrame, 2>                                   plane_frames_;
    mutable std::array<std::array<std::complex<double>, CUE_GROUPS>, 2> grid_sum_;
    mutable std::array<std::array<double, CUE_GROUPS>, 2>               grid_weight_;

    MarkerDetector::Marker refined_outer_;  // The outer ellipse refit to sub-pixel edges

    TrigLUTAligned trig_lut_samples_;    // Lookup table for sine/cosine values of 2Pi/id_samples
    TrigLUTAligned trig_lut_seg_width_;  // Lookup table for sine/cosine values of 2Pi/segment_width

    /**
     * @brief Calculate the trigonometric lookup table is populated.
     */
    void calcTrigLUT(TrigLUTAligned& lut, int value, int size);

    /**
     * @brief Computes the ellipse coordinates for a given segment.
     *
     * @param width      The width of the image.
     * @param height     The height of the image.
     * @param id_samples The number of ID samples.
     * @param s          The segment parameters.
     * @param lut        The trigonometric lookup table.
     * @param[out] x_coords The output x coordinates.
     * @param[out] y_coords The output y coordinates.
     *
     * @remark
     * Scalar Version (Performance increase ~20x):
     * ```
     * const float two_pi_over_samples = 2.0f * M_PI / id_samples_;
     * for (int a = 0; a < id_samples_; a++) {
     *     const float cos_angle = cosf(a * two_pi_over_samples);
     *     const float sin_angle = sinf(a * two_pi_over_samples);
     *     x_coords[a] = s.x + (s.m0 * cos_angle * s.v0 + s.m1 * sin_angle * s.v1) * 2.0f;
     *     y_coords[a] = s.y + (s.m0 * cos_angle * s.v1 - s.m1 * sin_angle * s.v0) * 2.0f;
     *     if (x_coords[a] < 0 || width <= x_coords[a] || y_coords[a] < 0 ||
     *         height <= y_coords[a]) {
     *         return false;
     *     }
     * }
     * ```
     */
    bool computeEllipseCoordinates(
        int width, int height, const SegmentParameters& s, aligned_vec<float>& x_coords,
        aligned_vec<float>& y_coords);

    /**
     * @brief Computes signal values for sample points in an image using bilinear interpolation.
     *
     * Processes the provided image data and calculates the signal at each sample location
     * specified in the analysis_data structure. Uses bilinear interpolation of RGB values
     * at each sample coordinate and stores the result in analysis_data.signal.
     * Employs data prefetching to optimize memory access for future samples.
     *
     * @param image_data     Pointer to raw image data (RGB format).
     * @param width          Width of the image in pixels.
     * @param id_samples     Number of sample points to process.
     * @param analysis_data  Structure containing sample coordinates and output signal array.
     * @param image_handler  ImageHandler providing image properties (e.g., bytes per pixel).
     *
     * @remark
     * Scalar Version (Performance increase ~2x):
     * ```
     * float gx, gy;
     * int px, py;
     * const unsigned char* ptr = image_data;
     * int step = image_handler.getBPP();  // RGB(3)
     * for (int a = 0; a < id_samples_; a++) {
     *     px = static_cast<int>(analysis_data_[solution_idx].x_coords[a]);
     *     py = static_cast<int>(analysis_data_[solution_idx].y_coords[a]);
     *     gx = analysis_data_[solution_idx].x_coords[a] - px;
     *     gy = analysis_data_[solution_idx].y_coords[a] - py;
     *     pos = (px + py * width);
     *     // Detection from the image
     *     analysis_data_[solution_idx].signal[a] =
     *         ptr[(pos + 0) * step + 0] * (1 - gx) * (1 - gy) + ptr[(pos + 1) * step + 0] * gx * (1
     * - gy) + ptr[(pos + width) * step + 0] * (1 - gx) * gy + ptr[step * (pos + width + 1) + 0] *
     * gx * gy; analysis_data_[solution_idx].signal[a] += ptr[(pos + 0) * step + 1] * (1 - gx) * (1
     * - gy) + ptr[(pos + 1) * step + 1] * gx * (1 - gy) + ptr[(pos + width) * step + 1] * (1 - gx)
     * * gy + ptr[step * (pos + width + 1) + 1] * gx * gy; analysis_data_[solution_idx].signal[a] +=
     *         ptr[(pos + 0) * step + 2] * (1 - gx) * (1 - gy) + ptr[(pos + 1) * step + 2] * gx * (1
     * - gy) + ptr[(pos + width) * step + 2] * (1 - gx) * gy + ptr[step * (pos + width + 1) + 2] *
     * gx * gy;
     * }
     * ```
     */
    void computeSignal(
        const unsigned char* image_data, int width, int id_samples,
        whycon::LocalizationSystem::SignalAnalysisData& analysis_data,
        const ImageHandler&                             image_handler);

    /**
     * @brief Binarizes the input signal by thresholding it against its average value.
     *
     * This function processes the input signal stored in the SignalAnalysisData structure,
     * computes the average value of the signal, and then sets each element in the output
     * 'smooth' array to 1.0 if the corresponding signal value is greater than the average,
     * or 0.0 otherwise. The operation is optimized using SIMD batch processing for improved
     * performance on large signals.
     *
     * @param data SignalAnalysisData containing the input signal and output array.
     * @param n Number of elements in the signal to process.
     *
     * @remark
     * Scalar Version (Perf increase ~5x):
     * ```
     * float avg = 0.0f;
     * for (int a = 0; a < id_samples_; a++)
     *     avg += analysis_data_[solution_idx].signal[a];
     *
     * avg = avg / id_samples_;
     *
     * for (int a = 0; a < id_samples_; a++) {
     *     analysis_data_[solution_idx].smooth[a] = (analysis_data_[solution_idx].signal[a] > avg)
     * ? 1.0f : 0.0f;
     * }
     * ```
     */
    void binarizeSignal(SignalAnalysisData& data, int n);

    /**
     * @brief Decodes the ID and weighs the two pose solutions against each other.
     *
     * Samples the ID ring and two rings inside it around the ellipse centre, then back-projects
     * the code's radial edges onto each solution's plane: only the right one puts them on a
     * regular angular grid.
     *
     * @param image_handler The image handler containing the frame data.
     * @param ellipse_centers Both pose solutions of the outer ellipse.
     * @param outer_marker The outer ellipse.
     * @param pose Output: decoded ID and its validity.
     * @param debug_manager Optional manager for collecting debug images.
     * @return Log-odds of solution 0 over solution 1; 0 when the ID ring leaves the image.
     */
    float identifyAndWeighSolutions(
        const ImageHandler& image_handler, const EllipseCenters& ellipse_centers,
        const MarkerDetector::Marker& outer_marker, MarkerPose& pose,
        DebugImageManager* debug_manager = nullptr);

    /**
     * @brief Samples one ring of the marker and binarises it at its mean.
     * @param ring Ring centre, axes and orientation.
     * @param image_handler The image handler containing the frame data.
     * @param data Output: sample coordinates, signal, binarised signal and threshold.
     * @return False if the ring leaves the image.
     */
    bool sampleRing(
        const SegmentParameters& ring, const ImageHandler& image_handler, SignalAnalysisData& data);

    /**
     * @brief Refits the outer ellipse to sub-pixel points of the marker's outline.
     *
     * The binarised outline's moments place the centre to ~0.07 px, as much as the perspective
     * offset that tells the two pose solutions apart on markers under 20 px. Points found along
     * normals of that ellipse, at the detector's own edge threshold, fit it far closer.
     *
     * @param image_handler The image handler containing the frame data.
     * @param inner The inner ellipse: its mean grey is the outline's light side.
     * @param outer The outer ellipse, refined in place unless the fit is implausible.
     */
    void refineOutline(
        const ImageHandler& image_handler, const MarkerDetector::Marker& inner,
        MarkerDetector::Marker& outer) const;

    void readCode(SignalAnalysisData& data) const;
    void setPlaneFrames(const EllipseCenters& centers) const;
    void accumulateEdges(const SignalAnalysisData& data, int ring_index) const;

    /**
     * @brief Draws a marker's tile on the markers debug panel.
     *
     * A zoomed crop of the grey frame around the outline, with the outline, the cue rings, the
     * code ring's samples coloured by the bit each reads, each code segment's bit and both pose
     * solutions' centres.
     *
     * @param debug_manager The debug panels; nothing is drawn once every tile is used.
     * @param image_handler The image handler containing the frame data.
     * @param centers Both pose solutions of the outline.
     * @param outline The outer ellipse the pose was computed from.
     * @param code_ring The ring the code was read from, or nullptr if none was.
     * @param favourite The solution this frame favours.
     * @param top Two lines above the marker.
     * @param problems The ID checks that failed, in red under the top lines.
     * @param bottom Two lines below the marker.
     */
    void drawMarkerTile(
        DebugImageManager& debug_manager, const ImageHandler& image_handler,
        const EllipseCenters& centers, const MarkerDetector::Marker& outline,
        const SignalAnalysisData* code_ring, int favourite, std::string_view top,
        std::string_view problems, std::string_view bottom) const;

    /**
     * @brief Applies camera undistortion to an image point.
     * @param x_in Input x coordinate (distorted image).
     * @param y_in Input y coordinate (distorted image).
     * @param x_out Output x coordinate (undistorted image).
     * @param y_out Output y coordinate (undistorted image).
     *
     * Uses the camera matrix and distortion coefficients to correct for lens distortion.
     */
    void undistorPoints(double x_in, double y_in, double& x_out, double& y_out) const;

    /**
     * @brief Precomputes undistortion map for optimization.
     *
     * This speeds up repeated undistortion operations by caching results.
     */
    void    precomputeUndistorMap(void);
    cv::Mat undistort_map;  // Cached undistortion map for fast lookup

    /**
     * @brief Calculates possible ellipse centers and normals from detected marker.
     * @param marker Detected marker ellipse parameters.
     * @return EllipseCenters struct with two possible solutions.
     *
     * Handles the geometric ambiguity of projecting a circle as an ellipse.
     */
    void calcEllipseCenters(const MarkerDetector::Marker& marker, EllipseCenters& result) const;

    /**
     * @brief Calculates eigenvalues/eigenvectors for ellipse fitting.
     * @return EllipseCenters struct with computed values.
     */
    void calcEigen(EllipseCenters& result) const;

    /**
     * @brief Picks the solution whose projected centre is nearer the inner ellipse's centre.
     * @param centers EllipseCenters with both possible solutions.
     * @param marker Inner marker ellipse parameters.
     * @return Index of the nearer solution.
     */
    int nearestSolution(const EllipseCenters& centers, const MarkerDetector::Marker& marker) const;

    // Orientation calculation utilities
    Quaternion orientationFromNormal(const std::array<float, 3>& normal) const;
    void       updateEulerAngles(MarkerPose& pose) const;
};
}  // namespace whycon

#endif  // WHYCON_LOCALIZATION_H
