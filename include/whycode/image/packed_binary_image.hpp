#ifndef WHYCON_PACKED_BINARY_IMAGE_HPP
#define WHYCON_PACKED_BINARY_IMAGE_HPP

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "xsimd/xsimd.hpp"

namespace whycon {

// Aligned allocator for optimal SIMD performance
template <typename T>
using aligned_vector = std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

/**
 * @brief Optimized bit-packed binary image class
 *
 * This class provides efficient storage and manipulation of binary images
 * where each pixel is represented by a single bit (1 bit per pixel).
 * It offers SIMD-optimized operations for high-performance image processing.
 */
class PackedBinaryImage {
  public:
    /**
     * @brief Construct a new PackedBinaryImage
     *
     * @param width Image width in pixels
     * @param height Image height in pixels
     */
    PackedBinaryImage(int width, int height);

    /**
     * @brief Get pixel value using (x,y) coordinates
     * @param x Column coordinate (0 to width-1)
     * @param y Row coordinate (0 to height-1)
     * @return true if pixel is set (white), false if clear (black)
     */
    inline bool getPixel(int x, int y) const {
        const size_t byte_index = y * stride_ + (x >> 3);  // x / 8
        const int    bit_index  = x & 7;                   // x % 8 (LSB-first)
        return (data_[byte_index] >> bit_index) & 1;
    }

    /**
     * @brief Set pixel value using (x,y) coordinates
     *
     * @param x Column coordinate (0 to width-1)
     * @param y Row coordinate (0 to height-1)
     * @param value Pixel value (true for white, false for black)
     */
    inline void setPixel(int x, int y, bool value) {
        const size_t byte_index = y * stride_ + (x >> 3);  // x / 8
        const int    bit_index  = x & 7;                   // x % 8
        if (value) {
            data_[byte_index] |= (1 << bit_index);  // Set bit
        } else {
            data_[byte_index] &= ~(1 << bit_index);  // Clear bit
        }
    }

    /**
     * @brief Get pixel using linear index [FASTEST IN SEQUENTIAL ACCESS]
     *
     * @param linear_index Linear pixel index (0 to width*height-1)
     * @return true if pixel is set (white), false if clear (black)
     */
    inline bool getPixelLinear(int linear_index) const {
        const size_t byte_index = linear_index >> 3;  // linear_index / 8
        const int    bit_index  = linear_index & 7;   // linear_index % 8
        return (data_[byte_index] >> bit_index) & 1;
    }

    /**
     * @brief Direct byte access [FASTEST FOR BULK PROCESSING]
     *
     * @param byte_index Byte index in packed data
     * @return uint8_t containing 8 pixels
     */
    inline uint8_t getByte(size_t byte_index) const { return data_[byte_index]; }

    /**
     * @brief Extract 8 pixels from a byte [OPTIMIZED FOR BATCH PROCESSING]
     *
     * @param byte_value uint8_t containing 8 packed pixels
     * @param pixel_bits Output array of 8 bool values
     */
    static inline void unpackByte(uint8_t byte_value, bool pixel_bits[8]) {
        for (int i = 0; i < 8; ++i) {
            pixel_bits[i] = (byte_value >> i) & 1;
        }
    }

    /**
     * @brief Get row pointer for row scanning [FASTER FOR ROW PROCESSING]
     *
     * @param y Row index
     * @return Pointer to start of row's packed data
     */
    inline const uint8_t* getRowPtr(int y) const { return &data_[y * stride_]; }

    /**
     * @brief Check if pixel matches expected value (OPTIMIZED FOR COMPARISON)
     *
     * @param x Column coordinate
     * @param y Row coordinate
     * @param expected Expected pixel value
     * @return true if pixel matches expected value
     */
    inline bool pixelEquals(int x, int y, bool expected) const { return getPixel(x, y) == expected; }

    /**
     * @brief Check if byte contains any black pixels [FASTEST BULK SCAN]
     *
     * @param byte_value Packed byte containing 8 pixels
     * @return true if any pixel in byte is black
     */
    static inline bool hasByteBlackPixel(uint8_t byte_value) {
        return byte_value != 0xFF;  // All bits set means all white pixels
    }

    /**
     * @brief Find first black pixel position in byte [OPTIMIZED SEARCH]
     *
     * @param byte_value Packed byte containing 8 pixels
     * @return bit position (0-7) of first black pixel, or -1 if none
     */
    static inline int findFirstBlackPixelInByte(uint8_t byte_value) {
        if (byte_value == 0)
            return -1;
        return __builtin_ctz(byte_value);  // Count trailing zeros = first set bit
    }

    /**
     * @brief Get all black pixel positions in byte [BATCH PROCESSING]
     * @param byte_value Packed byte containing 8 pixels
     * @param positions Output array for pixel positions (0-7)
     * @return count of black pixels found
     */
    static inline int getAllBlackPixelsInByte(uint8_t byte_value, int positions[8]) {
        int count = 0;
        for (int bit = 0; bit < 8; ++bit) {
            if (byte_value & (1u << bit)) {
                positions[count++] = bit;
            }
        }
        return count;
    }

    /**
     * @brief Optimized row scanner for black pixels [ROW-WISE SEARCH]
     * @param y Row to scan
     * @param start_x Starting x coordinate (will be rounded down to byte boundary)
     * @param end_x Ending x coordinate
     * @param first_black_x Output: x coordinate of first black pixel found
     * @return true if black pixel found, false otherwise
     */
    bool findFirstBlackPixelInRow(int y, int start_x, int end_x, int& first_black_x) const;

    /**
     * @brief Binarize the image using SIMD-optimized bit-packing
     *
     * @param src Source image data (grayscale)
     * @param threshold Binarization threshold
     */
    void binarizeBitPacked(const aligned_vector<uint8_t>& src, uint8_t threshold);

    /**
     * @brief Convert packed binary to 8-bit cv::Mat for visualization
     *
     * @return cv::Mat with CV_8UC1 format (0=black, 255=white)
     */
    cv::Mat convertToMat() const;

    /**
     * @brief Clear all pixels to black (set all bits to 0)
     */
    inline void clear() { std::fill(data_.begin(), data_.end(), 0); }

    /**
     * @brief Get image dimensions and stride information
     */
    int    getWidth() const { return width_; }
    int    getHeight() const { return height_; }
    size_t getStride() const { return stride_; }
    size_t getDataSize() const { return data_.size(); }

    inline const uint8_t* data() const { return data_.data(); }
    inline uint8_t*       data() { return data_.data(); }

  private:
    aligned_vector<uint8_t> data_;    // Bit-packed image data
    int                     width_;   // Image width in pixels
    int                     height_;  // Image height in pixels
    size_t                  stride_;  // Row stride in bytes
};

}  // namespace whycon

#endif  // WHYCON_PACKED_BINARY_IMAGE_HPP