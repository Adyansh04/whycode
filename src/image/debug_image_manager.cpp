#include "whycode/image/debug_image_manager.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <string>

#include "whycode/image/image_handler.hpp"

namespace whycon
{

DebugImageManager::DebugImageManager(int max_markers)
{
    // Tiles in a near-square grid: 6 markers take 3 x 2
    const int tiles = std::max(max_markers, 1);
    tile_cols_      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(tiles))));
    tile_rows_      = (tiles + tile_cols_ - 1) / tile_cols_;
}

void DebugImageManager::beginFrame(const ImageHandler& image_handler)
{
    const cv::Mat& grey = image_handler.getGray();
    if (wanted_[BINARY])
    {
        // Stays black if no detector searches this frame
        images_[BINARY].create(grey.size(), CV_8UC3);
        images_[BINARY].setTo(cv::Scalar::all(0));
    }
    if (wanted_[CANDIDATES])
    {
        cv::cvtColor(grey, images_[CANDIDATES], cv::COLOR_GRAY2RGB);
        images_[CANDIDATES].convertTo(images_[CANDIDATES], -1, BACKGROUND_GAIN);
    }
    if (wanted_[MARKERS])
    {
        images_[MARKERS].create(tile_rows_ * TILE_PX, tile_cols_ * TILE_PX, CV_8UC3);
        images_[MARKERS].setTo(cv::Scalar::all(0));
        tiles_used_ = 0;
    }
}

cv::Mat DebugImageManager::nextTile()
{
    if (!wanted_[MARKERS] || tiles_used_ >= tile_cols_ * tile_rows_)
    {
        return {};
    }
    const int tile = tiles_used_++;
    return images_[MARKERS](
        cv::Rect(tile % tile_cols_ * TILE_PX, tile / tile_cols_ * TILE_PX, TILE_PX, TILE_PX));
}

void DebugImageManager::drawText(
    cv::Mat& image, std::string_view text, cv::Point origin, const cv::Scalar& colour, double scale)
{
    int       baseline = 0;
    const int line_height =
        cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline).height + baseline + 4;
    size_t start = 0;
    while (start <= text.size())
    {
        const size_t      end = std::min(text.find('\n', start), text.size());
        const std::string line(text.substr(start, end - start));
        cv::putText(
            image,
            line,
            origin,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            cv::Scalar::all(0),
            3,
            cv::LINE_AA);
        cv::putText(image, line, origin, cv::FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv::LINE_AA);
        origin.y += line_height;
        start = end + 1;
    }
}

void DebugImageManager::drawEllipse(
    cv::Mat& image, cv::Point2f centre, cv::Size2f semi_axes, float angle, const cv::Scalar& colour,
    int thickness)
{
    // Coordinates in 1/16 pixel. Clamped as integers: cv::ellipse throws on a negative axis, and a
    // NaN would get past a float comparison under -ffast-math
    constexpr int   SHIFT = 4;
    constexpr float ONE   = 1 << SHIFT;
    const int       limit = (image.cols + image.rows) << SHIFT;
    const cv::Size  axes(
        std::clamp(cvRound(semi_axes.width * ONE), 0, limit),
        std::clamp(cvRound(semi_axes.height * ONE), 0, limit));
    const cv::Point at(
        std::clamp(cvRound(centre.x * ONE), -limit, limit),
        std::clamp(cvRound(centre.y * ONE), -limit, limit));
    cv::ellipse(image, at, axes, angle * 180.0 / CV_PI, 0, 360, colour, thickness, cv::LINE_AA, SHIFT);
}

}  // namespace whycon
