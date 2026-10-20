// Bit-packed binarisation: PackedBinaryImage::binarizeBitPacked, which writes a 64-pixel word per
// step and tests `threshold - grey` saturated at 0 (0 exactly when the pixel is not black), against
// the former version, which compared `grey < threshold` four vectors at a time (on AVX2, xsimd
// lowers that unsigned compare to two 128-bit halves with sign flips). The shipped one must match a
// per-pixel `grey < threshold` for every threshold, also at widths that leave a partial last word,
// with the padding bits 0. Exits 1 on any difference.
//   binarize_packed_bench <frame.pgm>...      (grey frames, e.g. data/perf/frames)
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

#include "bench_util.hpp"
#include "whycode/image/packed_binary_image.hpp"
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

// The former PackedBinaryImage::binarizeBitPacked: clear, then four vectors per step
static void binarizeFormer(
    const whycon::aligned_vector<uint8_t>& src, int width, int height, std::vector<uint8_t>& out,
    uint8_t threshold)
{
    using batch_type           = xs::batch<uint8_t>;
    constexpr size_t simd_size = batch_type::size;
    const size_t     stride    = (static_cast<size_t>(width) + 7) / 8;
    std::fill(out.begin(), out.end(), 0);
    const batch_type threshold_vec(threshold);
    for (int y = 0; y < height; y++)
    {
        const uint8_t* src_row           = src.data() + static_cast<size_t>(y) * width;
        uint8_t*       dst_row           = out.data() + y * stride;
        int            x                 = 0;
        constexpr int  vectors_per_chunk = 4;
        const int      chunk_size        = vectors_per_chunk * simd_size;
        for (; x + chunk_size <= width; x += chunk_size)
        {
            for (int v = 0; v < vectors_per_chunk; v++)
            {
                const int      offset = x + v * simd_size;
                const uint64_t bits = (xs::load_unaligned(&src_row[offset]) < threshold_vec).mask();
                std::memcpy(&dst_row[offset >> 3], &bits, simd_size / 8);
            }
        }
        for (; x < width; x++)
        {
            if (src_row[x] < threshold)
            {
                dst_row[x >> 3] |= 1u << (x & 7);
            }
        }
    }
}

// Pixels whose bit disagrees with grey < threshold, plus padding bits that are not 0
static long differences(
    const whycon::PackedBinaryImage& image, const whycon::aligned_vector<uint8_t>& grey,
    uint8_t threshold)
{
    const int    w = image.getWidth(), h = image.getHeight();
    const size_t stride = whycon::PackedBinaryImage::strideFor(w);
    long         wrong  = 0;
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < static_cast<int>(stride * 8); x++)
        {
            const bool black = x < w && grey[static_cast<size_t>(y) * w + x] < threshold;
            wrong += ((image.data()[y * stride + (x >> 3)] >> (x & 7)) & 1) != black;
        }
    }
    return wrong;
}

int main(int argc, char** argv)
{
    constexpr int REPEATS   = 200;
    double        former_us = 0, current_us = 0;
    long          wrong = 0, checked = 0;
    for (int f = 1; f < argc; f++)
    {
        const cv::Mat frame = bench::loadFrame(argv[f], false);
        // The full frame is timed; every threshold is checked on a strip of it (rows are
        // binarised independently), also cropped to widths that end in a partial word
        for (const int width : { frame.cols, frame.cols - 1, 1000, 848, 100, 63, 1 })
        {
            const int                       height = width == frame.cols ? frame.rows : 16;
            whycon::aligned_vector<uint8_t> grey(static_cast<size_t>(width) * height);
            for (int y = 0; y < height; y++)
            {
                std::memcpy(grey.data() + static_cast<size_t>(y) * width, frame.ptr(y), width);
            }
            whycon::PackedBinaryImage image(width, height);
            for (int t = 0; t < 256; t += width == frame.cols ? 51 : 1)
            {
                image.binarizeBitPacked(grey, static_cast<uint8_t>(t));
                wrong += differences(image, grey, static_cast<uint8_t>(t));
                checked++;
            }
            if (width == frame.cols)
            {
                std::vector<uint8_t> former((static_cast<size_t>(width) + 7) / 8 * height);
                former_us += bench::medianMicros(REPEATS, [&] {
                    binarizeFormer(grey, width, height, former, 128);
                });
                current_us +=
                    bench::medianMicros(REPEATS, [&] { image.binarizeBitPacked(grey, 128); });
            }
        }
    }
    std::cout << std::fixed << std::setprecision(1) << xs::default_arch::name() << ": former "
              << former_us / (argc - 1) << " us, current " << current_us / (argc - 1)
              << " us per binarisation; " << checked << " threshold x width x frame checks, "
              << wrong << " wrong bits\n";
    return wrong == 0 ? 0 : 1;
}
