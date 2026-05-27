#include "whycode/image/packed_binary_image.hpp"

#include <algorithm>
#include <opencv2/opencv.hpp>

namespace whycon
{
namespace xs = xsimd;

PackedBinaryImage::PackedBinaryImage(int width, int height)
  : width_(width)
  , height_(height)
{
    stride_ = (width + 7) / 8;  // Round up to nearest byte
    data_.resize(stride_ * height, 0);
}

bool PackedBinaryImage::findFirstBlackPixelInRow(
    int y, int start_x, int end_x, int& first_black_x) const
{
    if (y < 0 || y >= height_)
        return false;

    const uint8_t* row_data = getRowPtr(y);

    // Align to byte boundaries for efficiency
    int byte_start = start_x >> 3;      // start_x / 8
    int byte_end   = (end_x + 7) >> 3;  // (end_x + 7) / 8

    for (int byte_idx = byte_start; byte_idx < byte_end && byte_idx < static_cast<int>(stride_);
         ++byte_idx)
    {
        uint8_t byte_value = row_data[byte_idx];

        if (hasByteBlackPixel(byte_value))
        {
            int bit_pos = findFirstBlackPixelInByte(byte_value);
            int pixel_x = byte_idx * 8 + bit_pos;

            // Check if within requested range
            if (pixel_x >= start_x && pixel_x <= end_x && pixel_x < width_)
            {
                first_black_x = pixel_x;
                return true;
            }

            // Check remaining bits in this byte within range
            for (int bit = (pixel_x < start_x) ? (start_x & 7) : bit_pos;
                 bit < 8 && (byte_idx * 8 + bit) <= end_x;
                 ++bit)
            {
                if (byte_value & (1u << bit))
                {
                    first_black_x = byte_idx * 8 + bit;
                    return true;
                }
            }
        }
    }
    return false;
}

void PackedBinaryImage::binarizeBitPacked(const aligned_vector<uint8_t>& src, uint8_t threshold)
{
    using batch_type           = xs::batch<uint8_t>;
    constexpr size_t simd_size = batch_type::size;

    clear();
    const batch_type threshold_vec = batch_type(threshold);

    for (int y = 0; y < height_; y++)
    {
        const uint8_t* src_row = src.data() + y * width_;
        uint8_t*       dst_row = const_cast<uint8_t*>(getRowPtr(y));

        // Process in SIMD chunks
        int           x                 = 0;
        constexpr int vectors_per_chunk = 4;
        const int     chunk_size        = vectors_per_chunk * simd_size;

        for (; x + chunk_size <= width_; x += chunk_size)
        {
            for (int v = 0; v < vectors_per_chunk; v++)
            {
                const int offset   = x + v * simd_size;
                auto      src_vec  = xs::load_unaligned(&src_row[offset]);
                auto      mask_vec = src_vec < threshold_vec;
                auto      bitmask  = mask_vec.mask();  // lane mask, 1 bit per pixel (LSB-first)

                // Store the bitmask as a scalar
                if constexpr (simd_size == 64)
                {
                    // 64 pixels -> 64-bit mask
                    reinterpret_cast<uint64_t&>(dst_row[offset >> 3]) =
                        static_cast<uint64_t>(bitmask);
                }
                else if constexpr (simd_size == 32)
                {
                    reinterpret_cast<uint32_t&>(dst_row[offset >> 3]) =
                        static_cast<uint32_t>(bitmask);
                }
                else if constexpr (simd_size == 16)
                {
                    reinterpret_cast<uint16_t&>(dst_row[offset >> 3]) =
                        static_cast<uint16_t>(bitmask);
                }
                else
                {
                    dst_row[offset >> 3] = static_cast<uint8_t>(bitmask);
                }
            }
        }

        // Scalar tail: LSB-first
        for (; x < width_; x++)
        {
            if (src_row[x] < threshold)
            {
                const int byte_idx = x >> 3;
                const int bit_idx  = x & 7;
                dst_row[byte_idx] |= (1u << bit_idx);
            }
        }
    }
}

cv::Mat PackedBinaryImage::convertToMat() const
{
    cv::Mat result(height_, width_, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < height_; ++y)
    {
        const uint8_t* row_data   = getRowPtr(y);
        uint8_t*       result_row = result.ptr<uint8_t>(y);

        for (int x = 0; x < width_; ++x)
        {
            int byte_idx = x >> 3;  // x / 8
            int bit_idx  = x & 7;   // x % 8

            // Extract bit and convert to 0 or 255
            result_row[x] = (row_data[byte_idx] & (1 << bit_idx)) ? 255 : 0;
        }
    }

    return result;
}

}  // namespace whycon