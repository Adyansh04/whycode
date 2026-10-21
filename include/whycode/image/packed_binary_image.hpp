#ifndef WHYCON_PACKED_BINARY_IMAGE_HPP
#define WHYCON_PACKED_BINARY_IMAGE_HPP

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "xsimd/xsimd.hpp"

namespace whycon
{

// Aligned allocator for optimal SIMD performance
template <typename T>
using aligned_vector =
    std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

/**
 * @brief Bit-packed binary image: one bit per pixel, set for black (grey below the threshold).
 *
 * Rows are LSB-first and padded to whole 64-bit words (strideFor), so a row can be read a word at a
 * time. The padding bits past the image are always 0 (not black); FreePixels relies on that.
 */
class PackedBinaryImage
{
public:
    /**
     * @brief Construct a new PackedBinaryImage
     *
     * @param width Image width in pixels
     * @param height Image height in pixels
     */
    PackedBinaryImage(int width, int height);

    /**
     * @brief Row stride for an image width: whole 64-bit words, so a row can be read a word at a
     * time without crossing into the next one.
     * @param width Image width in pixels.
     * @return Row stride in bytes.
     */
    static constexpr size_t strideFor(int width)
    {
        return (static_cast<size_t>(width) + 63) / 64 * 8;
    }

    /**
     * @brief Get pixel value using (x,y) coordinates
     * @param x Column coordinate (0 to width-1)
     * @param y Row coordinate (0 to height-1)
     * @return true if the pixel is black (bit set), false if white
     */
    inline bool getPixel(int x, int y) const
    {
        const size_t byte_index = y * stride_ + (x >> 3);  // x / 8
        const int    bit_index  = x & 7;                   // x % 8 (LSB-first)
        return (data_[byte_index] >> bit_index) & 1;
    }

    /**
     * @brief Get row pointer for row scanning
     *
     * @param y Row index
     * @return Pointer to start of row's packed data
     */
    inline const uint8_t* getRowPtr(int y) const { return &data_[y * stride_]; }

    /**
     * @brief Binarize the image using SIMD-optimized bit-packing
     *
     * @param src Source image data (grayscale)
     * @param threshold Grey levels below it are black
     */
    void binarizeBitPacked(const aligned_vector<uint8_t>& src, uint8_t threshold);

    /**
     * @brief Convert packed binary to 8-bit cv::Mat for visualization
     *
     * @return cv::Mat with CV_8UC1 format (0=black, 255=white)
     */
    cv::Mat convertToMat() const;

    /**
     * @brief Get image dimensions
     */
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    /**
     * @brief Get the packed rows
     * @return First byte of row 0; rows are strideFor(width) bytes apart
     */
    inline const uint8_t* data() const { return data_.data(); }

private:
    aligned_vector<uint8_t> data_;    // Bit-packed image data
    int                     width_;   // Image width in pixels
    int                     height_;  // Image height in pixels
    size_t                  stride_;  // Row stride in bytes
};

}  // namespace whycon

#endif  // WHYCON_PACKED_BINARY_IMAGE_HPP