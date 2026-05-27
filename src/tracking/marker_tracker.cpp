#include "whycode/tracking/marker_tracker.hpp"

#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <optional>
#include <vector>

#include "whycode/utils/whycon_config.h"

namespace whycon
{

MarkerTracker::MarkerTracker(
    int max_frames_unseen, double max_association_dist, int min_track_age, int max_tracks)
  : max_unseen_(max_frames_unseen)
  , min_track_age_(min_track_age)
  , max_dist_sq_(max_association_dist * max_association_dist)
  , next_id_(0)
  , initialized_(false)
  , max_tracks_(max_tracks)
{
    // Pre-allocate memory for all vectors to avoid reallocations during tracking
    tracks_.reserve(max_tracks_);
    track_id_to_index_.reserve(max_tracks_);
    predicted_positions_.reserve(max_tracks_);
    potential_matches_.reserve(max_tracks_ * 5);  // Assuming max 5 candidates per track
    detection_matched_.reserve(max_tracks_);
    track_assigned_.reserve(max_tracks_);
}

void MarkerTracker::initializeKalmanFilter(cv::KalmanFilter& kf, const cv::Point2f& initial_pos)
{
    // clang-format off
    // State: [x,y,vx,vy] - position and velocity
    // Measurement: [x,y] - only position is observed
    kf.init(4, 2, 0, CV_32F);

    // Transition matrix (constant velocity model)
    // [1 0 dt  0]
    // [0 1  0 dt]
    // [0 0  1  0]
    // [0 0  0  1]
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Measurement matrix - we only observe position
    // [1 0 0 0]
    // [0 1 0 0]
    kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);
    
    // Process noise covariance
    kf.processNoiseCov = (cv::Mat_<float>(4, 4) <<
        PROCESS_NOISE_POS, 0, 0, 0,
        0, PROCESS_NOISE_POS, 0, 0,
        0, 0, PROCESS_NOISE_VEL, 0,
        0, 0, 0, PROCESS_NOISE_VEL);
    
    // Measurement noise covariance
    kf.measurementNoiseCov = (cv::Mat_<float>(2, 2) <<
        MEASUREMENT_NOISE, 0,
        0, MEASUREMENT_NOISE);
    
    // Initial state covariance
    kf.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * 1000;
    
    // Set initial state [x, y, vx=0, vy=0]
    kf.statePost = (cv::Mat_<float>(4, 1) << initial_pos.x, initial_pos.y, 0, 0);
    // clang-format on
}

void MarkerTracker::update(
    std::vector<whycon::MarkerDetector::Marker*>& current_detections,
    const ImageHandler& image_handler, std::vector<int>& removed_ids)
{
    // Validate inputs to avoid crashes
    for (const auto* detection : current_detections)
    {
        if (detection == nullptr)
        {
            WHYCON_ERROR("[Tracker] Null detection pointer received");
            removed_ids.clear();  // Clear removed_ids to avoid confusion
            return;
        }
    }

    // Handle potential integer overflow in track IDs
    if (next_id_ >= std::numeric_limits<int>::max() - 10)
    {
        WHYCON_INFO("[Tracker] Track ID counter approaching maximum, resetting");
        next_id_ = 0;
    }

    // Increment age and unseen counter for all existing tracks.
    // The unseen counter will be reset to 0 for tracks that are successfully matched below.
    for (auto& track : tracks_)
    {
        track.marker_age++;
        track.frames_unseen++;
    }

    // --- First Frame Initialization ---
    if (!initialized_)
    {
        WHYCON_INFO("[Tracker] Initializing tracker with first set of detections.");
        for (auto* detection : current_detections)
        {
            tracks_.emplace_back(next_id_++, cv::Point2f(detection->x, detection->y));
            ActiveTrack& new_track = tracks_.back();

            // Initialize Kalman filter for new track
            initializeKalmanFilter(new_track.kf, new_track.last_position);

            const int new_track_id           = new_track.tracking_id;
            track_id_to_index_[new_track_id] = tracks_.size() - 1;
            detection->tracking_id           = new_track_id;
        }
        initialized_ = true;
        return;  // No tracks to remove on first frame
    }

    // --- Stage 1: Predict new positions using Kalman Filter ---
    predicted_positions_.clear();

    if (tracks_.empty())
    {
        WHYCON_DEBUG("[Tracker] No active tracks to predict.");
    }
    else
    {
        // Collect points from active tracks
        predicted_positions_.reserve(tracks_.size());

        for (auto& track : tracks_)
        {
            // Predict next state
            cv::Mat prediction = track.kf.predict();

            // Extract predicted position
            cv::Point2f predicted_pos(prediction.at<float>(0), prediction.at<float>(1));
            predicted_positions_.push_back(predicted_pos);

            WHYCON_DEBUG(
                "[Tracker] Track " << track.tracking_id << " predicted at (" << predicted_pos.x
                                   << ", " << predicted_pos.y << ")");
        }
    }

    // --- Stage 2: Data Association (Greedy Assignment) ---
    detection_matched_.assign(current_detections.size(), false);
    potential_matches_.clear();

    for (size_t track_idx = 0; track_idx < tracks_.size(); ++track_idx)
    {
        const cv::Point2f& predicted_pos = predicted_positions_[track_idx];

        for (size_t detection_idx = 0; detection_idx < current_detections.size(); ++detection_idx)
        {
            const auto& detection = current_detections[detection_idx];

            // Calculate squared distance efficiently
            const double dx               = predicted_pos.x - detection->x;
            const double dy               = predicted_pos.y - detection->y;
            const double squared_distance = dx * dx + dy * dy;

            // Only consider matches within threshold distance
            if (squared_distance < max_dist_sq_)
            {
                potential_matches_.push_back({ squared_distance,
                                               static_cast<int>(track_idx),
                                               static_cast<int>(detection_idx) });
            }
        }
    }

    std::sort(potential_matches_.begin(), potential_matches_.end());

    // Perform greedy assignment
    track_assigned_.assign(tracks_.size(), false);
    for (const auto& [dist_sq, track_idx, detection_idx] : potential_matches_)
    {
        if (!track_assigned_[track_idx] && !detection_matched_[detection_idx])
        {
            WHYCON_DEBUG(
                "[Tracker] Matched detection to track ID " << tracks_[track_idx].tracking_id);

            // Update the track with kalman filter correction
            cv::Mat measurement =
                (cv::Mat_<float>(2, 1) << current_detections[detection_idx]->x,
                 current_detections[detection_idx]->y);

            tracks_[track_idx].kf.correct(measurement);

            // Update last known position and reset unseen counter
            tracks_[track_idx].last_position               = { current_detections[detection_idx]->x,
                                                               current_detections[detection_idx]->y };
            tracks_[track_idx].frames_unseen               = 0;
            current_detections[detection_idx]->tracking_id = tracks_[track_idx].tracking_id;

            track_assigned_[track_idx]        = true;
            detection_matched_[detection_idx] = true;
        }
    }

    // --- Stage 3: Create new tracks for unmatched detections ---
    for (size_t i = 0; i < current_detections.size(); ++i)
    {
        if (!detection_matched_[i])
        {
            // Add new track and update mapping
            tracks_.emplace_back(
                next_id_++,
                cv::Point2f(current_detections[i]->x, current_detections[i]->y));
            ActiveTrack& new_track = tracks_.back();

            // Initialize Kalman filter for new track
            initializeKalmanFilter(new_track.kf, new_track.last_position);

            const int new_track_id             = new_track.tracking_id;
            track_id_to_index_[new_track_id]   = tracks_.size() - 1;
            current_detections[i]->tracking_id = new_track_id;
            WHYCON_INFO("[Tracker] Created new track with ID " << new_track_id);
        }
    }

    // --- Stage 4: Remove stale tracks ---
    removed_ids.clear();

    auto   it    = tracks_.begin();
    size_t index = 0;

    while (it != tracks_.end())
    {
        if (it->frames_unseen > max_unseen_)
        {
            WHYCON_INFO("[Tracker] Removing stale track with ID " << it->tracking_id);
            removed_ids.push_back(it->tracking_id);

            // Remove from id-to-index map
            track_id_to_index_.erase(it->tracking_id);

            // Remove the track from vector
            it = tracks_.erase(it);

            // Update indices for all subsequent tracks in the map
            for (auto& [id, idx] : track_id_to_index_)
            {
                if (idx > index)
                {
                    idx--;  // Decrement indices of tracks that were after the removed one
                }
            }
        }
        else
        {
            ++it;
            ++index;
        }
    }

    WHYCON_DEBUG("[Tracker] Update complete. Active tracks: " << tracks_.size());
}

std::optional<int> MarkerTracker::getTrackAge(int tracking_id) const
{
    auto it = track_id_to_index_.find(tracking_id);
    if (it != track_id_to_index_.end())
    {
        return tracks_[it->second].marker_age;
    }
    return std::nullopt;  // Track not found
}

}  // namespace whycon
