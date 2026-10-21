#include "whycode/tracking/marker_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "whycode/utils/whycon_config.h"

namespace whycon
{

MarkerTracker::MarkerTracker(
    int max_frames_unseen, double max_association_dist, int min_track_age, float motion_scale,
    float acceleration_scale, int max_tracks)
  : max_unseen_(max_frames_unseen)
  , min_track_age_(min_track_age)
  , max_dist_sq_(max_association_dist * max_association_dist)
  , next_id_(0)
  , initialized_(false)
  , max_tracks_(max_tracks)
  , initial_variance_(INITIAL_VARIANCE * motion_scale * motion_scale)
{
    // clang-format off
    const float acceleration = ACCELERATION_NOISE * acceleration_scale;
    process_noise_ = acceleration * acceleration * cv::Matx44f(
        0.25f, 0,     0.5f, 0,
        0,     0.25f, 0,    0.5f,
        0.5f,  0,     1,    0,
        0,     0.5f,  0,    1);
    // clang-format on

    // Pre-allocate memory for all vectors to avoid reallocations during tracking
    tracks_.reserve(max_tracks_);
    predicted_positions_.reserve(max_tracks_);
    innovation_inverse_.reserve(max_tracks_);
    potential_matches_.reserve(max_tracks_ * 5);  // Assuming max 5 candidates per track
    detection_matched_.reserve(max_tracks_);
    track_assigned_.reserve(max_tracks_);
}

void MarkerTracker::addTrack(whycon::MarkerDetector::Marker& detection)
{
    tracks_.emplace_back(
        next_id_++,
        cv::Point2f(detection.x, detection.y),
        detection.m0,
        initial_variance_);
    detection.tracking_id = tracks_.back().tracking_id;
    WHYCON_INFO("[Tracker] Created new track with ID " << detection.tracking_id);
}

void MarkerTracker::predict(ActiveTrack& track) const noexcept
{
    // clang-format off
    // Constant velocity over one frame; the process noise is a random acceleration
    static const cv::Matx44f transition(
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    // clang-format on
    track.state      = transition * track.state;
    track.covariance = transition * track.covariance * transition.t() + process_noise_;
}

void MarkerTracker::correct(ActiveTrack& track, const cv::Point2f& measurement) noexcept
{
    // Only the position is measured, so P H^T is P's first two columns
    const cv::Matx<float, 4, 2> cross = track.covariance.get_minor<4, 2>(0, 0);
    const cv::Matx22f           innovation =
        track.covariance.get_minor<2, 2>(0, 0) + cv::Matx22f::eye() * MEASUREMENT_NOISE;
    const cv::Matx<float, 4, 2> gain = cross * innovation.inv();
    const cv::Vec2f residual(measurement.x - track.state[0], measurement.y - track.state[1]);

    track.state += cv::Vec4f(gain * residual);
    track.covariance -= gain * cross.t();
    track.covariance =
        0.5f * (track.covariance + track.covariance.t());  // Rounding breaks symmetry
}

void MarkerTracker::update(
    std::vector<whycon::MarkerDetector::Marker*>& current_detections,
    const ImageHandler& image_handler, std::vector<int>& removed_ids)
{
    removed_ids.clear();

    // Validate inputs to avoid crashes
    for (const auto* detection : current_detections)
    {
        if (detection == nullptr)
        {
            WHYCON_ERROR("[Tracker] Null detection pointer received");
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
            addTrack(*detection);
        }
        initialized_ = true;
        return;  // No tracks to remove on first frame
    }

    // --- Stage 1: Predict new positions using Kalman Filter ---
    predicted_positions_.clear();
    innovation_inverse_.clear();
    for (auto& track : tracks_)
    {
        predict(track);
        predicted_positions_.emplace_back(track.state[0], track.state[1]);
        innovation_inverse_.push_back(
            (track.covariance.get_minor<2, 2>(0, 0) + cv::Matx22f::eye() * MEASUREMENT_NOISE).inv());
    }

    // --- Stage 2: Data Association (Greedy Assignment) ---
    detection_matched_.assign(current_detections.size(), false);
    potential_matches_.clear();

    for (size_t track_idx = 0; track_idx < tracks_.size(); ++track_idx)
    {
        const cv::Point2f& predicted_pos = predicted_positions_[track_idx];
        const cv::Matx22f& inverse       = innovation_inverse_[track_idx];

        for (size_t detection_idx = 0; detection_idx < current_detections.size(); ++detection_idx)
        {
            const auto& detection = current_detections[detection_idx];

            const float  dx               = detection->x - predicted_pos.x;
            const float  dy               = detection->y - predicted_pos.y;
            const double squared_distance = dx * dx + dy * dy;
            const float  mahalanobis_sq =
                inverse(0, 0) * dx * dx + 2.0f * inverse(0, 1) * dx * dy + inverse(1, 1) * dy * dy;

            // Within the distance gate, the filter's own gate, and of a similar apparent size
            if (squared_distance < max_dist_sq_ && mahalanobis_sq < GATE_CHI2 &&
                std::abs(std::log(detection->m0 / tracks_[track_idx].size)) < MAX_LOG_SIZE_CHANGE)
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
            ActiveTrack&                    track     = tracks_[track_idx];
            whycon::MarkerDetector::Marker& detection = *current_detections[detection_idx];
            WHYCON_DEBUG("[Tracker] Matched detection to track ID " << track.tracking_id);

            track.last_position = { detection.x, detection.y };
            correct(track, track.last_position);
            track.size            = detection.m0;
            track.frames_unseen   = 0;
            detection.tracking_id = track.tracking_id;

            track_assigned_[track_idx]        = true;
            detection_matched_[detection_idx] = true;
        }
    }

    // --- Stage 3: Create new tracks for unmatched detections ---
    for (size_t i = 0; i < current_detections.size(); ++i)
    {
        if (!detection_matched_[i])
        {
            addTrack(*current_detections[i]);
        }
    }

    // --- Stage 4: Remove stale tracks ---
    std::erase_if(tracks_, [this, &removed_ids](const ActiveTrack& track) {
        if (track.frames_unseen <= max_unseen_)
        {
            return false;
        }
        WHYCON_INFO("[Tracker] Removing stale track with ID " << track.tracking_id);
        removed_ids.push_back(track.tracking_id);
        return true;
    });

    WHYCON_DEBUG("[Tracker] Update complete. Active tracks: " << tracks_.size());
}

std::optional<int> MarkerTracker::getTrackAge(int tracking_id) const
{
    // A handful of tracks: a scan is cheaper than keeping an index in step
    for (const auto& track : tracks_)
    {
        if (track.tracking_id == tracking_id)
        {
            return track.marker_age;
        }
    }
    return std::nullopt;  // Track not found
}

}  // namespace whycon
