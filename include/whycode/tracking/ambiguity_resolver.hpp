#ifndef AMBIGUITY_RESOLVER_HPP
#define AMBIGUITY_RESOLVER_HPP

#include <opencv2/core/types.hpp>
#include <vector>

#include "whycode/core/whycon_localization.hpp"

namespace whycon
{

/**
 * @brief Chooses between a marker's two pose solutions using the evidence of its whole track.
 *
 * A circle's outline fits two poses whose normals mirror about the line of sight. Each frame
 * weighs them; this class follows both solutions along a track and sums that evidence, so a
 * choice made while the marker is large holds once it is small, and one frame cannot flip it.
 */
class AmbiguityResolver
{
public:
    AmbiguityResolver();

    /**
     * @brief Reports the solution the track's evidence favours, with its confidence.
     * @param pose Detection with both solutions and this frame's evidence; its solutions are
     *        swapped in place when the track favours the mirror one.
     */
    void resolve(LocalizationSystem::MarkerPose& pose);

    /**
     * @brief Removes the history of tracks that ended.
     * @param tracking_ids Track ids to forget.
     */
    void removeTracks(const std::vector<int>& tracking_ids);

private:
    static constexpr float  FRAME_WEIGHT       = 0.5f;  // Frames of one track share their errors
    static constexpr float  MAX_FRAME_EVIDENCE = 4.0f;  // Log-odds one frame can add
    static constexpr float  MAX_EVIDENCE       = 8.0f;  // Lets a track recover from a wrong start
    static constexpr float  MIN_SIGNAL_PX      = 0.1f;  // Separation of the solutions that counts
    static constexpr float  CENTRE_ERROR_PX = 0.07f;  // Ellipse centre error, calibrated on the sim
    static constexpr float  MIN_PROBABILITY = 1e-6f;
    static constexpr size_t INITIAL_CAPACITY = 30;  // Pre-allocate for marker counts

    struct Branches
    {
        int       tracking_id;
        cv::Vec3f normal[2];        // Each solution's face normal, followed frame to frame
        float     evidence = 0.0f;  // Log-odds that branch 0 is the right solution
    };

    static cv::Vec3f faceNormal(const Quaternion& q) noexcept;

    std::vector<Branches> tracks_;  // A handful of tracks: scanned, not hashed
};

}  // namespace whycon

#endif  // AMBIGUITY_RESOLVER_HPP
