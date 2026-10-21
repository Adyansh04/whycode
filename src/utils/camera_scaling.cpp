#include "whycode/utils/camera_scaling.hpp"

#include <algorithm>
#include <cmath>

namespace whycon
{

CameraScaling::CameraScaling(
    double focal_length_px, double frame_rate_hz, int width, int height) noexcept
  : motion_scale_((focal_length_px / REFERENCE_FOCAL_PX) * (REFERENCE_RATE_HZ / frame_rate_hz))
  , frame_rate_scale_(frame_rate_hz / REFERENCE_RATE_HZ)
  , width_(width)
  , height_(height)
{}

double CameraScaling::associationDistance() const noexcept
{
    return REFERENCE_ASSOCIATION * motion_scale_;
}

int CameraScaling::maxSegmentSize() const noexcept { return width_ * height_; }

int CameraScaling::maxUnseenFrames() const noexcept
{
    return std::max(1, static_cast<int>(std::lround(REFERENCE_UNSEEN_FRAMES * frame_rate_scale_)));
}

}  // namespace whycon
