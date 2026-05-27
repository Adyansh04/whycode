#include "whycode/image/image_handler.hpp"

#include <sys/types.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <xsimd/types/xsimd_api.hpp>

#include "Simd/SimdLib.hpp"
#include "whycode/utils/whycon_config.h"

#define ENABLE_BINARY_IMSHOW 0

namespace whycon
{
namespace xs = xsimd;

ImageHandler::ImageHandler(int initial_width, int initial_height, const std::string& encoding)
  : width_(initial_width)
  , height_(initial_height)
  , data_(nullptr)
{
    // Calculate bytes per pixel from encoding string once at construction
    if (encoding == "rgb8" || encoding == "bgr8")
    {
        bpp_ = 3;
    }
    else if (encoding == "rgba8" || encoding == "bgra8")
    {
        bpp_ = 4;
    }
    else if (encoding == "mono8")
    {
        bpp_ = 1;
    }
    else
    {
        // Fallback for unknown encodings
        WHYCON_ERROR(
            "Unsupported encoding for optimized ImageHandler: " << encoding
                                                                << ". Defaulting to bpp=3.");
        bpp_ = 3;
    }

    size_ = width_ * height_ * bpp_;

    // Allocate aligned memory
    aligned_gray_.resize(height_ * width_);

    switch (bpp_)
    {
        case 1:
            cv_type_ = CV_8UC1;
            break;
        case 3:
            cv_type_ = CV_8UC3;
            break;
        case 4:
            cv_type_ = CV_8UC4;
            break;
        default:
            cv_type_ = CV_8UC3;
            break;
    }

    // Allocate initial buffer
    data_ = (unsigned char*)std::malloc(sizeof(unsigned char) * size_);
    if (!data_)
    {
        throw std::runtime_error("Failed to allocate image buffer");
    }

    // Initialize grayscale mats
    gray_   = cv::Mat(height_, width_, CV_8UC1, aligned_gray_.data());
    binary_ = cv::Mat(height_, width_, CV_8UC1);

    // Initialize PackedBinaryImage
    binary_packed_ = std::make_unique<PackedBinaryImage>(width_, height_);
}

ImageHandler::~ImageHandler()
{
    if (data_)
    {
        std::free(data_);
        data_ = nullptr;
    }
}

void ImageHandler::binarize(uchar threshold)
{
    if (gray_.empty())
        return;
    SimdBinarization(
        gray_.data,
        gray_.step,
        width_,
        height_,
        threshold,
        0,
        255,
        binary_.data,
        binary_.step,
        SimdCompareLesser);
}

void ImageHandler::binarizeSIMD(uint8_t threshold)
{
    if (gray_.empty())
        return;

    // Use the SIMD aligned binarization
    binary_packed_->binarizeBitPacked(aligned_gray_, threshold);
}

bool ImageHandler::updateFromROS(const sensor_msgs::msg::Image::ConstSharedPtr& ros_image)
{
    std::memcpy(data_, &ros_image->data[0], size_);

    // Store timestamp
    last_ros_timestamp_ = ros_image->header.stamp;

    // Convert to grayscale
    // cv::cvtColor(raw_mat, gray_, cv::COLOR_BGR2GRAY);
    SimdRgbToGray(data_, width_, height_, width_ * bpp_, gray_.data, gray_.step);

#if ENABLE_BINARY_IMSHOW
    cv::imshow("Gray Image", gray_);
    cv::waitKey(1);

    // Display SIMD binarized image if available
    if (binary_packed_)
    {
        cv::Mat binary_vis = getBinaryVisualization();
        cv::imshow("SIMD Binary Image", binary_vis);
        cv::waitKey(1);
    }
#endif

    return true;
}

cv::Mat ImageHandler::getBinaryVisualization() const { return binary_packed_->convertToMat(); }

void ImageHandler::reallocateIfNeeded(int new_width, int new_height, int new_bpp)
{
    int new_size = new_width * new_height * new_bpp;

    if (width_ != new_width || height_ != new_height || bpp_ != new_bpp)
    {
        WHYCON_INFO(
            "[ImageHandler] Readjusting image format from "
            << width_ << "x" << height_ << "x" << bpp_ << " to " << new_width << "x" << new_height
            << "x" << new_bpp);

        width_  = new_width;
        height_ = new_height;
        bpp_    = new_bpp;
        size_   = new_size;

        data_ = (unsigned char*)std::realloc(data_, sizeof(unsigned char) * size_);
        if (!data_)
        {
            throw std::runtime_error("Failed to reallocate image buffer");
        }
        // Also resize the grayscale mats
        gray_   = cv::Mat(height_, width_, CV_8UC1);
        binary_ = cv::Mat(height_, width_, CV_8UC1);
    }
}

}  // namespace whycon
