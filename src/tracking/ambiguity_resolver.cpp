#include "whycode/tracking/ambiguity_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace whycon
{

AmbiguityResolver::AmbiguityResolver() { tracks_.reserve(INITIAL_CAPACITY); }

cv::Vec3f AmbiguityResolver::faceNormal(const Quaternion& q) noexcept
{
    // The marker frame's x axis
    return { 1.0f - 2.0f * (q.y * q.y + q.z * q.z),
             2.0f * (q.x * q.y + q.z * q.w),
             2.0f * (q.x * q.z - q.y * q.w) };
}

void AmbiguityResolver::resolve(LocalizationSystem::MarkerPose& pose)
{
    // The ellipse centre's error persists along a track and flips the cue whenever it exceeds half
    // the solutions' separation: frames with this separation, however many, earn no more evidence
    // than log((1 - p) / p), p the chance of such an error
    const float p = 0.5f * std::erfc(
                               pose.ambiguity_signal_px /
                               (2.0f * std::numbers::sqrt2_v<float> * CENTRE_ERROR_PX));
    const float cap = std::min(MAX_EVIDENCE, std::log((1.0f - p) / std::max(p, MIN_PROBABILITY)));

    // This frame's evidence for the solution the pose reports
    const float frame =
        std::min(FRAME_WEIGHT * std::min(pose.ambiguity_llr, MAX_FRAME_EVIDENCE), cap);
    if (pose.tracking_id < 0)
    {
        pose.orientation_confidence = 1.0f / (1.0f + std::exp(-frame));
        return;
    }

    const cv::Vec3f normal     = faceNormal(pose.orientation);
    const cv::Vec3f alt_normal = faceNormal(pose.alt_orientation);
    auto            it = std::find_if(tracks_.begin(), tracks_.end(), [&pose](const Branches& b) {
        return b.tracking_id == pose.tracking_id;
    });
    const bool      inserted = it == tracks_.end();
    if (inserted)
    {
        it = tracks_.insert(tracks_.end(), Branches{ pose.tracking_id, {}, 0.0f });
    }
    Branches& track = *it;

    // Each solution continues the branch whose normal it is nearer
    bool on_branch0 = true;
    if (!inserted)
    {
        const float same =
            std::abs(normal.dot(track.normal[0])) + std::abs(alt_normal.dot(track.normal[1]));
        const float cross =
            std::abs(normal.dot(track.normal[1])) + std::abs(alt_normal.dot(track.normal[0]));
        on_branch0 = same >= cross;
    }
    track.normal[on_branch0 ? 0 : 1] = normal;
    track.normal[on_branch0 ? 1 : 0] = alt_normal;

    // Below MIN_SIGNAL_PX a frame's cue is mostly that persistent error: it adds nothing
    const float frame0 = on_branch0 ? frame : -frame;
    if (pose.ambiguity_signal_px >= MIN_SIGNAL_PX && std::abs(track.evidence) <= cap)
    {
        track.evidence = std::clamp(track.evidence + frame0, -cap, cap);
    }

    // Until a frame is informative, this frame's own cue decides; with no cue at all (no code to
    // read, as on plain WhyCon markers) the frame's own choice stands
    const float log_odds0 = track.evidence != 0.0f ? track.evidence : frame0;
    if (log_odds0 != 0.0f && (log_odds0 > 0.0f) != on_branch0)
    {
        pose.swapSolutions();
    }
    pose.orientation_confidence = 1.0f / (1.0f + std::exp(-std::abs(log_odds0)));
}

void AmbiguityResolver::removeTracks(const std::vector<int>& tracking_ids)
{
    std::erase_if(tracks_, [&tracking_ids](const Branches& b) {
        return std::find(tracking_ids.begin(), tracking_ids.end(), b.tracking_id) !=
               tracking_ids.end();
    });
}

}  // namespace whycon
