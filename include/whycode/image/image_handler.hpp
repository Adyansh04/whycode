#ifndef WHYCON_IMAGE_HANDLER_HPP
#define WHYCON_IMAGE_HANDLER_HPP

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sys/types.h>

#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "whycode/image/packed_binary_image.hpp"
#include "xsimd/xsimd.hpp"

namespace whycon {

// Aligned allocator for image data
template <typename T>
using aligned_vector = std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

// Forward declaration
class PackedBinaryImage;

/**
 * @brief image handler for WhyCon processing
 *
 * Provides direct memory management and format conversion for marker detection.
 */
class ImageHandler {
  public:
    /**
     * @brief Enumeration for image processing modes
     *
     * Defines the modes in which the ImageHandler can operate.
     */
    enum class Mode { ROS, ICEORYX };

    // Callback function type for Iceoryx mode
    using IceoryxCallback = std::function<void()>;

    /**
     * @brief Construct ImageHandler with initial dimensions
     *
     * @param initial_width Width of the image
     * @param initial_height Height of the image
     * @param encoding Image encoding type
     * @param mode Operating mode (ROS or ICEORYX)
     */
    ImageHandler(int initial_width, int initial_height, const std::string& encoding, Mode mode = Mode::ROS,
                 const std::string& runtime_name = "WhyconImageHandler", const std::string& service_instance = "Camera",
                 const std::string& service_method = "Image", const std::string& service_event = "RGB");

    /**
     * @brief Destructor - cleanup allocated memory
     */
    ~ImageHandler();

    /**
     * @brief Update image data from ROS sensor_msgs::Image
     *
     * @param ros_image ROS image message
     * @return true if successful, false on error
     */
    bool updateFromROS(const sensor_msgs::msg::Image::ConstSharedPtr& ros_image);

    /**
     * @brief Set callback for Iceoryx mode
     *
     * @param callback
     */
    void setIceoryxCallback(IceoryxCallback callback);

    // Get latest timesetamp for publishing
    const builtin_interfaces::msg::Time& getLastRosTimestamp() const { return last_ros_timestamp_; }
    int64_t                               getLastIceoryxTimestamp() const { return last_iceoryx_timestamp_us_; }

    /**
     * @brief SIMD-optimized binarization with bit-packing (1 bit per pixel)
     *
     * @param threshold Binarization threshold
     */
    void binarizeSIMD(const uint8_t threshold);

    /**
     * @brief Binarize using SimdLib (1 byte per pixel)
     *
     * @param threshold Binarization threshold
     */
    void binarize(uchar threshold);

    /**
     * @brief Get raw image data pointer
     */
    inline unsigned char* getData() const { return data_; }

    /**
     * @brief Convert to OpenCV Mat for compatibility
     *
     * @param output_mat Output OpenCV Mat (will be created/resized as needed)
     */
    inline void toOpenCVMat(cv::Mat& output_mat) const {
        // Create Mat header pointing to our data (no copy)
        output_mat = cv::Mat(height_, width_, cv_type_, (void*)data_);
    }

    /**
     * @brief Get visualization of SIMD binarized image
     * @return cv::Mat representation of packed binary data
     */
    cv::Mat getBinaryVisualization() const;

    /**
     * @brief Get the current grayscale image
     */
    inline const cv::Mat& getGray() const { return gray_; }

    /**
     * @brief Get the binarized image
     */
    inline const cv::Mat& getBinary() const { return binary_; }

    /**
     * @brief Check if image data is valid
     */
    inline bool isValid() const { return data_ != nullptr && size_ > 0; }

    /**
     * @brief Get the Binary Packed object data
     */
    const PackedBinaryImage& getBinaryPacked() const { return *binary_packed_; }
    PackedBinaryImage&       getBinaryPacked() { return *binary_packed_; }

    // Getters
    inline int getWidth() const { return width_; }
    inline int getHeight() const { return height_; }
    inline int getBPP() const { return bpp_; }
    inline int getSize() const { return size_; }

  private:
    Mode mode_;  // Operating mode (ROS or ICEORYX)

    int            width_;    // Image width in pixels
    int            height_;   // Image height in pixels
    int            bpp_;      // Bytes per pixel
    int            size_;     // Total image size in bytes
    int            cv_type_;  // OpenCV type based on bpp
    unsigned char* data_;     // Raw image data buffer

    cv::Mat gray_;    // Grayscale image
    cv::Mat binary_;  // Binarized image

    // Iceoryx parameters
    std::string runtime_name_;
    std::string service_instance_;
    std::string service_method_;
    std::string service_event_;

    aligned_vector<uint8_t>            aligned_gray_;
    std::unique_ptr<PackedBinaryImage> binary_packed_;  // Packed binary image

    // Iceoryx members
    std::unique_ptr<iox::popo::UntypedSubscriber> rgb_subscriber_;
    std::unique_ptr<iox::popo::Listener>          listener_;
    IceoryxCallback                               iceoryx_callback_;

    // Timestamp tracking
    builtin_interfaces::msg::Time last_ros_timestamp_;
    int64_t                       last_iceoryx_timestamp_us_ = 0;

    // Static callback for Iceoryx listener
    static void onIceoryxSampleReceived(iox::popo::UntypedSubscriber* subscriber, ImageHandler* self);

    /**
     * @brief Reallocate buffer if size changed
     */
    void reallocateIfNeeded(int new_width, int new_height, int new_bpp);
};

/**
 * @brief Shared memory header structure for RGB image data
 */
struct RGBImageHeader {
    int      rows;        // Image height in pixels
    int      cols;        // Image width in pixels
    int      type;        // OpenCV image type
    int64_t  timestamp;   // Capture timestamp in microseconds
    uint32_t dataOffset;  // Offset from payload start to image data
    uint32_t dataSize;    // Size of image data in bytes
};

}  // namespace whycon

#endif  // WHYCON_IMAGE_HANDLER_HPP
