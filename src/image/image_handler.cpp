#include "whycode/image/image_handler.hpp"

#include <sys/types.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "Simd/SimdLib.hpp"
#include "whycode/utils/whycon_config.h"

#define ENABLE_BINARY_IMSHOW 0

namespace whycon
{
ImageHandler::ImageHandler(int initial_width, int initial_height, const std::string& encoding)
  : width_(initial_width)
  , height_(initial_height)
  , encoding_name_(encoding)
{
    // Calculate bytes per pixel from encoding string once at construction
    if (encoding == "rgb8" || encoding == "bgr8")
    {
        bpp_      = 3;
        encoding_ = encoding == "rgb8" ? Encoding::RGB8 : Encoding::BGR8;
    }
    else if (encoding == "rgba8" || encoding == "bgra8")
    {
        bpp_      = 4;
        encoding_ = encoding == "rgba8" ? Encoding::RGBA8 : Encoding::BGRA8;
    }
    else if (encoding == "mono8")
    {
        bpp_      = 1;
        encoding_ = Encoding::MONO8;
    }
    else
    {
        // Fallback for unknown encodings
        WHYCON_ERROR(
            "Unsupported encoding for optimized ImageHandler: " << encoding
                                                                << ". Defaulting to rgb8.");
        bpp_      = 3;
        encoding_ = Encoding::RGB8;
    }

    size_    = width_ * height_ * bpp_;
    cv_type_ = CV_MAKETYPE(CV_8U, bpp_);

    // Allocate aligned memory
    aligned_gray_.resize(height_ * width_);
    gray_ = cv::Mat(height_, width_, CV_8UC1, aligned_gray_.data());

    // Initialize PackedBinaryImage
    binary_packed_ = std::make_unique<PackedBinaryImage>(width_, height_);
}

void ImageHandler::binarizeSIMD(uint8_t threshold)
{
    if (gray_.empty())
        return;

    // Use the SIMD aligned binarization
    binary_packed_->binarizeBitPacked(aligned_gray_, threshold);
}

bool ImageHandler::updateFromROS(const sensor_msgs::msg::Image& ros_image)
{
    // Every buffer downstream is sized once from the configured camera; a different frame would
    // be read past its end, and another encoding would be read with the wrong layout
    if (static_cast<int>(ros_image.width) != width_ ||
        static_cast<int>(ros_image.height) != height_ ||
        static_cast<int>(ros_image.step) != width_ * bpp_ ||
        ros_image.data.size() < static_cast<size_t>(size_) || ros_image.encoding != encoding_name_)
    {
        return false;
    }
    data_ = ros_image.data.data();

    // Store timestamp
    last_ros_timestamp_ = ros_image.header.stamp;

    // Convert to grayscale
    const size_t stride = static_cast<size_t>(width_) * bpp_;
    switch (encoding_)
    {
        case Encoding::BGR8:
            SimdBgrToGray(data_, width_, height_, stride, gray_.data, gray_.step);
            break;
        case Encoding::RGBA8:
            SimdRgbaToGray(data_, width_, height_, stride, gray_.data, gray_.step);
            break;
        case Encoding::BGRA8:
            SimdBgraToGray(data_, width_, height_, stride, gray_.data, gray_.step);
            break;
        case Encoding::MONO8:
            std::memcpy(gray_.data, data_, size_);  // gray_ rows have no padding
            break;
        default:
            SimdRgbToGray(data_, width_, height_, stride, gray_.data, gray_.step);
            break;
    }

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

void ImageHandler::copyColour(cv::Mat& out) const
{
    // The pixels are the received message's: drawing on them would change what the detector
    // samples and what other subscribers see, so this is the one place that views them as a Mat
    const cv::Mat frame(height_, width_, cv_type_, const_cast<unsigned char*>(data_));
    switch (encoding_)
    {
        case Encoding::MONO8:
            cv::cvtColor(frame, out, cv::COLOR_GRAY2RGB);
            break;
        case Encoding::RGBA8:
            cv::cvtColor(frame, out, cv::COLOR_RGBA2RGB);
            break;
        case Encoding::BGRA8:
            cv::cvtColor(frame, out, cv::COLOR_BGRA2BGR);
            break;
        default:
            frame.copyTo(out);
            break;
    }
}

const char* ImageHandler::colourEncoding() const
{
    return (encoding_ == Encoding::BGR8 || encoding_ == Encoding::BGRA8) ? "bgr8" : "rgb8";
}

cv::Mat ImageHandler::getBinaryVisualization() const { return binary_packed_->convertToMat(); }
}  // namespace whycon
