#ifndef WHYCON_CAMERA_SCALING_HPP
#define WHYCON_CAMERA_SCALING_HPP

namespace whycon
{

/**
 * @brief The detector's camera-dependent limits, derived once at startup.
 *
 * A marker's image motion per frame grows with the focal length in pixels and shrinks with the
 * frame rate, so the motion limits, tuned on the Isaac Sim camera (762.7 px focal length, 30 Hz),
 * scale by (focal length / 762.7) * (30 / frame rate). A marker's own pixel limits (noise blobs,
 * decoding, edge noise) and the shape tolerances do not depend on the camera: from 0.5x to 1.5x
 * the Isaac resolution, detection holds with them unchanged.
 */
class CameraScaling
{
public:
    /**
     * @brief Derives the limits for one camera.
     * @param focal_length_px Horizontal focal length (px), from the camera matrix.
     * @param frame_rate_hz Camera frame rate (Hz).
     * @param width Image width (px).
     * @param height Image height (px).
     */
    CameraScaling(double focal_length_px, double frame_rate_hz, int width, int height) noexcept;

    /**
     * @brief Image motion per frame relative to the reference camera.
     * @return 1 on the reference camera.
     */
    double motionScale() const noexcept { return motion_scale_; }

    /**
     * @brief Change of image motion over one frame relative to the reference camera.
     * @return 1 on the reference camera; acceleration per frame grows with the frame time squared.
     */
    double accelerationScale() const noexcept { return motion_scale_ / frame_rate_scale_; }

    /**
     * @brief Distance gate for a new track's first match.
     * @return Gate (px): markers move up to 37 px per frame in turns on the reference camera.
     */
    double associationDistance() const noexcept;

    /**
     * @brief Largest segment worth examining.
     * @return The image area (px).
     */
    int maxSegmentSize() const noexcept;

    /**
     * @brief Frames a track may go unseen before it is dropped.
     * @return A third of a second's frames, at least one.
     */
    int maxUnseenFrames() const noexcept;

private:
    static constexpr double REFERENCE_FOCAL_PX      = 762.72224426269531;  // Isaac Sim camera
    static constexpr double REFERENCE_RATE_HZ       = 30.0;
    static constexpr double REFERENCE_ASSOCIATION   = 40.0;  // px, on the reference camera
    static constexpr int    REFERENCE_UNSEEN_FRAMES = 10;    // At the reference rate

    double motion_scale_;
    double frame_rate_scale_;  // Frame rate over the reference rate
    int    width_, height_;
};

}  // namespace whycon

#endif  // WHYCON_CAMERA_SCALING_HPP
