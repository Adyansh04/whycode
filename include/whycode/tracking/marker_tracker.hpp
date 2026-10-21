#ifndef MARKER_TRACKER_HPP
#define MARKER_TRACKER_HPP

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <vector>

#include "whycode/core/marker_detector.hpp"
#include "whycode/image/image_handler.hpp"

namespace whycon
{

/**
 * @brief Represents a single tracked marker: a constant-velocity Kalman filter on its centre.
 */
struct ActiveTrack
{
    int         tracking_id;    // The persistent ID for this track.
    cv::Point2f last_position;  // The last known 2D position in image coordinates.
    int   frames_unseen;  // Counter for how many consecutive frames this track has not been seen.
    int   marker_age;     // How many frames this track has been alive.
    float size;           // Outer ellipse's m0 at the last match (px)
    cv::Vec4f   state;    // Kalman state [x, y, vx, vy] (px, px/frame)
    cv::Matx44f covariance;  // Kalman state covariance

    ActiveTrack(int id, const cv::Point2f& pos, float marker_size, float initial_variance)
      : tracking_id(id)
      , last_position(pos)
      , frames_unseen(0)
      , marker_age(1)
      , size(marker_size)
      , state(pos.x, pos.y, 0.0f, 0.0f)
      , covariance(cv::Matx44f::eye() * initial_variance)
    {}
};

/**
 * @brief A helper struct to store a potential match between a track and a detection.
 */
struct PotentialMatch
{
    double dist_sq;        // The squared distance between the track and detection.
    int    track_idx;      // The index of the track in the tracks_ vector.
    int    detection_idx;  // The index of the detection in the current_detections vector.

    // Add a comparison operator for sorting
    bool operator<(const PotentialMatch& other) const { return dist_sq < other.dist_sq; }
};

/**
 * @brief Keeps a persistent tracking ID per marker across frames.
 *
 * A detection joins the track whose prediction is nearest, within the distance gate, the Kalman
 * filter's own gate and a size check, so a track whose marker is missed for a frame does not
 * take over a neighbouring marker.
 */
class MarkerTracker
{
public:
    /**
     * @param max_frames_unseen The number of frames a track can be unseen before it is removed.
     * @param max_association_dist The maximum distance (in pixels) to associate a detection with a track.
     * @param min_track_age The minimum number of frames a track must exist to be considered "confirmed".
     * @param max_tracks Maximum expected number of simultaneous tracks (for memory pre-allocation).
     */
    MarkerTracker(
        int max_frames_unseen, double max_association_dist, int min_track_age, int max_tracks = 25);

    /**
     * @brief Updates tracks with a new set of detections.
     *
     * This function performs data association. It modifies the 'tracking_id'
     * field of the Marker objects in the input vector in-place by using pointers.
     *
     * @param current_detections A vector of pointers to markers detected in the current frame.
     * @param image_handler A constant reference to the image handler containing the current frame.
     * @param removed_ids Output vector that will be cleared and filled with the tracking_ids that
     * were removed.
     */
    void update(
        std::vector<whycon::MarkerDetector::Marker*>& current_detections,
        const ImageHandler& image_handler, std::vector<int>& removed_ids);

    /**
     * @brief Gets the marker_age of a specific track.
     * @param tracking_id The ID of the track to query.
     * @return The marker_age of the track in frames, or nullopt if the track does not exist.
     */
    std::optional<int> getTrackAge(int tracking_id) const;

private:
    std::vector<ActiveTrack> tracks_;
    double                   max_dist_sq_;
    int                      max_unseen_;
    int                      min_track_age_;
    int                      next_id_;
    int                      max_tracks_;
    bool                     initialized_;

    // Vectors for computations (pre-allocated)
    std::vector<cv::Point2f>    predicted_positions_;
    std::vector<cv::Matx22f>    innovation_inverse_;  // Inverse covariance of each prediction
    std::vector<bool>           detection_matched_;
    std::vector<bool>           track_assigned_;
    std::vector<PotentialMatch> potential_matches_;

    // Kalman filter parameters
    static constexpr float ACCELERATION_NOISE = 2.0f;   // px/frame^2; fast turns reach 5.5
    static constexpr float MEASUREMENT_NOISE  = 4.0f;   // pixels ^ 2
    static constexpr float INITIAL_VARIANCE = 1000.0f;  // Unknown velocity: the distance gate rules
    static constexpr float GATE_CHI2        = 13.8f;  // 99.9% of innovations, 2 degrees of freedom
    static constexpr float MAX_LOG_SIZE_CHANGE = 0.3f;  // Apparent size changes under 3% per frame

    void        addTrack(whycon::MarkerDetector::Marker& detection);
    static void predict(ActiveTrack& track) noexcept;
    static void correct(ActiveTrack& track, const cv::Point2f& measurement) noexcept;
};

}  // namespace whycon

#endif  // MARKER_TRACKER_HPP
