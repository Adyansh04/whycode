#include "whycode/tracking/pose_stabilizer.hpp"

#include "whycode/utils/whycon_config.h"

namespace whycon {

PoseStabilizer::PoseStabilizer(float alpha)
    : alpha_(std::clamp(alpha, MIN_ALPHA, MAX_ALPHA)), one_minus_alpha_(1.0f - alpha_) {
    filtered_poses_.reserve(INITIAL_CAPACITY);

    // Log if alpha was clamped for debugging
    if (alpha != alpha_) {
        WHYCON_DEBUG("[PoseStabilizer] Alpha value " << alpha << " clamped to " << alpha_);
    }
}

cv::Vec3f PoseStabilizer::stabilize(int tracking_id, const cv::Vec3f& current_pose) {
    // Attempt to find the existing filtered pose for this tracking ID
    auto [it, inserted] = filtered_poses_.try_emplace(tracking_id, current_pose);

    // If this is the first time we see this track, return the current pose
    if (inserted) {
        WHYCON_DEBUG("[PoseStabilizer] Initializing filter for new track ID: " << tracking_id);
        return current_pose;
    }

    // Apply EMA filter and update stored pose in-place
    cv::Vec3f& filtered_pose = it->second;
    applyEmaFilter(filtered_pose, current_pose);

    return filtered_pose;
}

inline void PoseStabilizer::applyEmaFilter(cv::Vec3f& filtered_pose, const cv::Vec3f& current_pose) const {
    // Apply Exponential Moving Average filter to each component using pre-computed factors
    filtered_pose = alpha_ * current_pose + one_minus_alpha_ * filtered_pose;
}

void PoseStabilizer::removeTrack(int tracking_id) {
    if (filtered_poses_.erase(tracking_id) > 0) {
        WHYCON_INFO("[PoseStabilizer] Removing history for lost track ID: " << tracking_id);
    }
}

void PoseStabilizer::removeTracks(const std::vector<int>& tracking_ids) {
    for (int id : tracking_ids) {
        removeTrack(id);
    }
}

}  // namespace whycon