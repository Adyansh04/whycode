#ifndef WHYCON_DEBUG_IMAGE_MANAGER_HPP
#define WHYCON_DEBUG_IMAGE_MANAGER_HPP

#include <array>
#include <opencv2/core.hpp>
#include <string_view>

namespace whycon
{
class ImageHandler;

/**
 * @brief The debug panels of a frame, each drawn only while it is wanted.
 *
 * Panels are RGB images built from the frame's grey image, which is what the detector sees, so
 * their colours do not depend on the camera's encoding. Nothing is drawn while no panel is wanted.
 */
class DebugImageManager
{
public:
    enum Panel
    {
        BINARY,      // The frame binarised at the first searching detector's threshold
        CANDIDATES,  // Every segment the search examined, painted by the check that dropped it
        MARKERS,     // A zoomed tile per marker: outline, code ring, both pose solutions
        PANEL_COUNT
    };

    static constexpr std::array<const char*, PANEL_COUNT> PANEL_NAMES = { "binary",
                                                                          "candidates",
                                                                          "markers" };
    static constexpr int                                  TILE_PX = 360;  // Edge of a marker tile

    /**
     * @brief Constructs the manager with no panel wanted.
     * @param max_markers Tiles the markers panel holds, one per marker the system looks for.
     */
    explicit DebugImageManager(int max_markers);

    /**
     * @brief Sets whether a panel is drawn from the next frame on.
     * @param panel The panel.
     * @param wanted True while something shows the panel.
     */
    void setWanted(Panel panel, bool wanted) noexcept { wanted_[panel] = wanted; }

    /**
     * @brief Whether a panel is drawn this frame.
     * @param panel The panel.
     * @return True if it is wanted.
     */
    bool wants(Panel panel) const noexcept { return wanted_[panel]; }

    /**
     * @brief Whether any panel is drawn this frame.
     * @return True if at least one panel is wanted.
     */
    bool any() const noexcept { return wanted_[BINARY] || wanted_[CANDIDATES] || wanted_[MARKERS]; }

    /**
     * @brief Starts the wanted panels of a frame.
     * @param image_handler The frame; its grey image is the candidates panel's background.
     */
    void beginFrame(const ImageHandler& image_handler);

    /**
     * @brief The image of a panel for this frame.
     * @param panel The panel.
     * @return The panel's CV_8UC3 RGB image, to draw on.
     */
    cv::Mat& image(Panel panel) noexcept { return images_[panel]; }

    /**
     * @brief The next unused tile of the markers panel.
     * @return A TILE_PX square view into the markers panel; empty once every tile is used.
     */
    cv::Mat nextTile();

    /**
     * @brief Draws text, one image line per '\n'-separated line, outlined to read on any background.
     * @param image Image to draw on.
     * @param text The text.
     * @param origin Baseline start of the first line.
     * @param colour Text colour.
     * @param scale Font scale.
     */
    static void drawText(
        cv::Mat& image, std::string_view text, cv::Point origin, const cv::Scalar& colour,
        double scale = 0.5);

    /**
     * @brief Draws an ellipse to sub-pixel precision.
     * @param image Image to draw on.
     * @param centre Centre, with pixel centres at integer coordinates.
     * @param semi_axes Semi-axis along the angle, then across it.
     * @param angle Angle of the first semi-axis (radians, from image x towards image y).
     * @param colour Line colour.
     * @param thickness Line thickness.
     */
    static void drawEllipse(
        cv::Mat& image, cv::Point2f centre, cv::Size2f semi_axes, float angle,
        const cv::Scalar& colour, int thickness = 1);

private:
    static constexpr double BACKGROUND_GAIN = 0.7;  // Dims the candidates' background

    std::array<bool, PANEL_COUNT>    wanted_{};
    std::array<cv::Mat, PANEL_COUNT> images_;
    int                              tile_cols_  = 1;
    int                              tile_rows_  = 1;
    int                              tiles_used_ = 0;
};

}  // namespace whycon

#endif  // WHYCON_DEBUG_IMAGE_MANAGER_HPP
