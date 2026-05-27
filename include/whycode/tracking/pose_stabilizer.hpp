#ifndef POSE_STABILIZER_HPP
#define POSE_STABILIZER_HPP

#include <opencv2/core/types.hpp>
#include <unordered_map>
#include <vector>

namespace whycon
{

/**
 * @brief Stabilizes noisy 3D pose data using an Exponential Moving Average (EMA) filter.
 *
 * This class maintains a filtered state for the (x, y, z) position of each
 * tracked marker. It smooths out high-frequency noise (jitter) from the raw
 * pose estimations.
 */
class PoseStabilizer
{
public:
    /**
     * @brief Constructs the pose stabilizer.
     * @param alpha The smoothing factor for the EMA filter (0.0 to 1.0).
     *              A smaller alpha provides more smoothing but increases latency.
     *              A larger alpha provides less smoothing but is more responsive.
     */
    explicit PoseStabilizer(float alpha);

    /**
     * @brief Processes a new pose and returns a stabilized version.
     * @param tracking_id The persistent tracking ID from the MarkerTracker.
     * @param current_pose The raw (x, y, z) pose from the current frame.
     * @return The stabilized (x, y, z) pose.
     */
    cv::Vec3f stabilize(int tracking_id, const cv::Vec3f& current_pose);

    /**
     * @brief Removes history for multiple tracks that are no longer seen.
     * @param tracking_ids A vector of track IDs to remove.
     */
    void removeTracks(const std::vector<int>& tracking_ids);

private:
    static constexpr float  MIN_ALPHA        = 0.0f;
    static constexpr float  MAX_ALPHA        = 1.0f;
    static constexpr size_t POSE_DIMENSIONS  = 3;
    static constexpr size_t INITIAL_CAPACITY = 30;  // Pre-allocate for marker counts

    void        removeTrack(int tracking_id);
    inline void applyEmaFilter(cv::Vec3f& filtered_pose, const cv::Vec3f& current_pose) const;

    std::unordered_map<int, cv::Vec3f> filtered_poses_;
    float                              alpha_;
    float                              one_minus_alpha_;
};

}  // namespace whycon

#endif  // POSE_STABILIZER_HPP