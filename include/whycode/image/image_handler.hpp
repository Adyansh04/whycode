#ifndef WHYCON_IMAGE_HANDLER_HPP
#define WHYCON_IMAGE_HANDLER_HPP

#include <sys/types.h>

#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "whycode/image/packed_binary_image.hpp"
#include "xsimd/xsimd.hpp"

namespace whycon
{

// Aligned allocator for image data
template <typename T>
using aligned_vector =
    std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

// Forward declaration
class PackedBinaryImage;

/**
 * @brief image handler for WhyCon processing
 *
 * Reads each frame in place, converts it to grey and binarises it, bit-packed.
 */
class ImageHandler
{
public:
    /**
     * @brief Construct ImageHandler with initial dimensions
     *
     * @param initial_width Width of the image
     * @param initial_height Height of the image
     * @param encoding Image encoding: rgb8, bgr8, rgba8, bgra8 or mono8 (others are read as rgb8)
     */
    ImageHandler(int initial_width, int initial_height, const std::string& encoding);

    /**
     * @brief Takes a frame and converts it to grey.
     *
     * The frame is read in place, not copied: the message must stay alive while the frame is
     * processed.
     *
     * @param ros_image ROS image message
     * @return true if successful, false if the frame does not match the configured camera
     */
    bool updateFromROS(const sensor_msgs::msg::Image& ros_image);

    // Get latest timesetamp for publishing
    const builtin_interfaces::msg::Time& getLastRosTimestamp() const { return last_ros_timestamp_; }

    /**
     * @brief SIMD-optimized binarization with bit-packing (1 bit per pixel)
     *
     * @param threshold Binarization threshold
     */
    void binarizeSIMD(const uint8_t threshold);

    /**
     * @brief The current binary image for display: black pixels 0, the others 255.
     * @return A new CV_8UC1 image; it allocates, so debug output only.
     */
    cv::Mat getBinaryVisualization() const;

    /**
     * @brief Copy the frame as 3-channel colour, to draw on
     *
     * @param out Output: CV_8UC3 in the channel order colourEncoding() names; its buffer is
     * reused when it already has the frame's size
     */
    void copyColour(cv::Mat& out) const;

    /**
     * @brief Encoding of copyColour's output
     * @return "rgb8" or "bgr8"
     */
    const char* colourEncoding() const;

    /**
     * @brief Get raw image data pointer, valid while the frame is processed
     */
    inline const unsigned char* getData() const { return data_; }

    /**
     * @brief Get the current grayscale image
     */
    inline const cv::Mat& getGray() const { return gray_; }

    /**
     * @brief Get the Binary Packed object data
     */
    const PackedBinaryImage& getBinaryPacked() const { return *binary_packed_; }
    PackedBinaryImage&       getBinaryPacked() { return *binary_packed_; }

    // Getters
    inline int                getWidth() const { return width_; }
    inline int                getHeight() const { return height_; }
    inline int                getBPP() const { return bpp_; }
    inline int                getSize() const { return size_; }
    inline const std::string& getEncoding() const { return encoding_name_; }

private:
    enum class Encoding
    {
        RGB8,
        BGR8,
        RGBA8,
        BGRA8,
        MONO8
    };

    int         width_;     // Image width in pixels
    int         height_;    // Image height in pixels
    int         bpp_;       // Bytes per pixel
    int         size_;      // Total image size in bytes
    int         cv_type_;   // OpenCV type based on bpp
    Encoding    encoding_;  // How the pixels are laid out
    std::string encoding_name_;

    const unsigned char* data_ = nullptr;  // The current frame, inside its message

    aligned_vector<uint8_t>            aligned_gray_;
    cv::Mat                            gray_;           // Grayscale image over aligned_gray_
    std::unique_ptr<PackedBinaryImage> binary_packed_;  // Packed binary image

    // Timestamp tracking
    builtin_interfaces::msg::Time last_ros_timestamp_;
};

}  // namespace whycon

#endif  // WHYCON_IMAGE_HANDLER_HPP
