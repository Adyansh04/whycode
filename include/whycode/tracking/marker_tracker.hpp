#ifndef MARKER_TRACKER_HPP
#define MARKER_TRACKER_HPP

#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>
#include <optional>
#include <unordered_map>
#include <vector>

#include "whycode/core/marker_detector.hpp"
#include "whycode/image/image_handler.hpp"

namespace whycon {

/**
 * @brief Represents a single tracked marker over time using its last known position.
 */
struct ActiveTrack {
    int              tracking_id;    // The persistent ID for this track.
    cv::Point2f      last_position;  // The last known 2D position in image coordinates.
    int              frames_unseen;  // Counter for how many consecutive frames this track has not been seen.
    int              marker_age;     // How many frames this track has been alive.
    cv::KalmanFilter kf;             // Kalman filter for this track (state: [x, y, vx, vy])

    ActiveTrack(int id, const cv::Point2f& pos)
        : tracking_id(id), last_position(pos), frames_unseen(0), marker_age(1) {}
};

/**
 * @brief A helper struct to store a potential match between a track and a detection.
 */
struct PotentialMatch {
    double dist_sq;        // The squared distance between the track and detection.
    int    track_idx;      // The index of the track in the tracks_ vector.
    int    detection_idx;  // The index of the detection in the current_detections vector.

    // Add a comparison operator for sorting
    bool operator<(const PotentialMatch& other) const { return dist_sq < other.dist_sq; }
};

/**
 * @brief Manages marker tracking across video frames using Lucas-Kanade Optical Flow.
 */
class MarkerTracker {
  public:
    /**
     * @param max_frames_unseen The number of frames a track can be unseen before it is removed.
     * @param max_association_dist The maximum distance (in pixels) to associate a detection with a track.
     * @param min_track_age The minimum number of frames a track must exist to be considered "confirmed".
     * @param max_tracks Maximum expected number of simultaneous tracks (for memory pre-allocation).
     */
    MarkerTracker(int max_frames_unseen, double max_association_dist, int min_track_age, int max_tracks = 25);

    /**
     * @brief Updates tracks with a new set of detections using Optical Flow.
     *
     * This function performs data association. It modifies the 'tracking_id'
     * field of the Marker objects in the input vector in-place by using pointers.
     *
     * @param current_detections A vector of pointers to markers detected in the current frame.
     * @param image_handler A constant reference to the image handler containing the current frame.
     * @param removed_ids Output vector that will be cleared and filled with the tracking_ids that were removed.
     */
    void update(std::vector<whycon::MarkerDetector::Marker*>& current_detections, const ImageHandler& image_handler,
                std::vector<int>& removed_ids);

    /**
     * @brief Gets the marker_age of a specific track.
     * @param tracking_id The ID of the track to query.
     * @return The marker_age of the track in frames, or nullopt if the track does not exist.
     */
    std::optional<int> getTrackAge(int tracking_id) const;

  private:
    std::vector<ActiveTrack>        tracks_;
    std::unordered_map<int, size_t> track_id_to_index_;  // Maps track ID to index in tracks_ vector
    double                          max_dist_sq_;
    int                             max_unseen_;
    int                             min_track_age_;
    int                             next_id_;
    int                             max_tracks_;
    bool                            initialized_;

    // Vectors for computations (pre-allocated)
    std::vector<cv::Point2f>    predicted_positions_;
    std::vector<bool>           detection_matched_;
    std::vector<bool>           track_assigned_;
    std::vector<PotentialMatch> potential_matches_;

    // Kalman filter parameters
    static constexpr double PROCESS_NOISE_POS = 1e-2;
    static constexpr double PROCESS_NOISE_VEL = 1e-3;
    static constexpr double MEASUREMENT_NOISE = 4.0;  // pixels ^ 2

    /**
     * @brief Initialize a Kalman filter for a new track
     *
     * @param kf The Kalman filter to initialize
     * @param initial_pos The initial position of the tracked object
     */
    void initializeKalmanFilter(cv::KalmanFilter& kf, const cv::Point2f& initial_pos);
};

}  // namespace whycon

#endif  // MARKER_TRACKER_HPP
