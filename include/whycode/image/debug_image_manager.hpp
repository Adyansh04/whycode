#ifndef _WHYCON_DEBUG_IMAGE_MANAGER_HPP_
#define _WHYCON_DEBUG_IMAGE_MANAGER_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace whycon
{

struct GridLayout
{
    int rows;
    int cols;

    GridLayout(int r = 1, int c = 1)
      : rows(r)
      , cols(c)
    {}
};

/**
 * @brief Manages collection and publishing of debug images
 *
 * Efficiently collects debug images during processing and creates
 * a consolidated view for debugging purposes when enabled.
 */
class DebugImageManager
{
public:
    /**
     * @brief Constructor
     * @param enabled Whether debug image collection is enabled
     */
    DebugImageManager(bool enabled = false);

    /**
     * @brief Add a debug image to the collection
     * @param image Debug image to add
     * @param title Title/label for the image
     */
    void addDebugImage(const cv::Mat& image, const std::string& title);

    /**
     * @brief Clear all collected debug images
     */
    void clearDebugImages();

    /**
     * @brief Create consolidated debug image from all collected images
     * @return Consolidated debug image, empty if not enabled or no images
     */
    cv::Mat createConsolidatedImage();

    /**
     * @brief Check if debug image collection is enabled
     */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Enable/disable debug image collection
     */
    void setEnabled(bool enabled)
    {
        enabled_ = enabled;
        if (!enabled_)
            clearDebugImages();
    }

private:
    bool                     enabled_;
    std::vector<cv::Mat>     debug_images_;
    std::vector<std::string> image_titles_;

    /**
     * @brief Calculate optimal grid layout for images
     * @param num_images Number of images to arrange
     * @return GridLayout struct containing rows and cols for grid layout
     */
    GridLayout calculateGridLayout(int num_images);

    /**
     * @brief Resize image to fit in grid cell
     * @param image Input image
     * @param target_width Target width
     * @param target_height Target height
     * @return Resized image
     */
    void resizeToFit(const cv::Mat& image, cv::Mat& output, int target_width, int target_height);
};

}  // namespace whycon

#endif  // _WHYCON_DEBUG_IMAGE_MANAGER_HPP_
