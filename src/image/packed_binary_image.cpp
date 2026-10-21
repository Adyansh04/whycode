#include "whycode/image/packed_binary_image.hpp"

#include <cstring>
#include <opencv2/opencv.hpp>

namespace whycon
{
namespace xs = xsimd;

PackedBinaryImage::PackedBinaryImage(int width, int height)
  : width_(width)
  , height_(height)
{
    stride_ = strideFor(width);
    data_.resize(stride_ * height, 0);
}

void PackedBinaryImage::binarizeBitPacked(const aligned_vector<uint8_t>& src, uint8_t threshold)
{
    using batch_type        = xs::batch<uint8_t>;
    constexpr int simd_size = batch_type::size;
    static_assert(64 % simd_size == 0, "a word holds whole vectors");

    // A pixel is black when threshold - grey, saturated at 0, is not 0: two instructions on AVX2,
    // where xsimd's unsigned compare takes two 128-bit halves. The width is a local since the
    // stores may alias the member, and every word is written, the padding too: no clearing first.
    const batch_type threshold_vec(threshold), zero(uint8_t{ 0 });
    const int        width = width_, words = width / 64;
    for (int y = 0; y < height_; y++)
    {
        const uint8_t* src_row = src.data() + static_cast<size_t>(y) * width;
        uint8_t*       dst_row = data_.data() + static_cast<size_t>(y) * stride_;
        for (int w = 0; w < words; w++)
        {
            uint64_t not_black = 0;
            for (int v = 0; v < 64 / simd_size; v++)
            {
                const auto grey = xs::load_unaligned(src_row + w * 64 + v * simd_size);
                not_black |= (xs::ssub(threshold_vec, grey) == zero).mask() << (v * simd_size);
            }
            const uint64_t black = ~not_black;
            std::memcpy(dst_row + w * 8, &black, sizeof(black));
        }

        // The row's last pixels, LSB-first
        if (words * 64 < width)
        {
            uint64_t black = 0;
            for (int x = words * 64; x < width; x++)
            {
                black |= uint64_t{ src_row[x] < threshold } << (x - words * 64);
            }
            std::memcpy(dst_row + words * 8, &black, sizeof(black));
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

            // A set bit is black
            result_row[x] = (row_data[byte_idx] & (1 << bit_idx)) ? 0 : 255;
        }
    }

    return result;
}

}  // namespace whycon