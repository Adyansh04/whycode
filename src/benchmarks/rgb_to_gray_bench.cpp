// RGB to grey: SimdRgbToGray, as the detector converts every frame, against a plain fixed-point
// loop the compiler vectorises. Both must give identical grey levels. Timed with the frame in
// cache, and with the caches flushed before each conversion, as a frame arrives from the transport.
//   rgb_to_gray_bench <frame.ppm>...      (P6 frames, e.g. data/perf/frames/rgb_00.ppm)
#include <Simd/SimdLib.h>

#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "bench_util.hpp"

// Simd's weights: 0.299, 0.587, 0.114 in 14-bit fixed point, rounded
static void rgbToGrayLoop(const uint8_t* rgb, size_t pixels, uint8_t* gray)
{
    for (size_t i = 0; i < pixels; i++)
    {
        const uint32_t r = rgb[3 * i], g = rgb[3 * i + 1], b = rgb[3 * i + 2];
        gray[i] = static_cast<uint8_t>((4899 * r + 9617 * g + 1868 * b + 8192) >> 14);
    }
}

int main(int argc, char** argv)
{
    constexpr int        REPEATS = 200;
    std::vector<uint8_t> cache_flush(size_t{ 128 } << 20);  // Larger than the last-level cache
    const auto           flush = [&] {
        for (size_t i = 0; i < cache_flush.size(); i += 64)
        {
            cache_flush[i]++;
        }
    };
    long mismatches = 0;
    for (int f = 1; f < argc; f++)
    {
        const cv::Mat        image  = bench::loadFrame(argv[f], true);
        const size_t         pixels = image.total();
        std::vector<uint8_t> simd_gray(pixels), loop_gray(pixels);

        const auto simd = [&] {
            SimdRgbToGray(
                image.data,
                image.cols,
                image.rows,
                image.step,
                simd_gray.data(),
                image.cols);
        };
        const auto loop = [&] { rgbToGrayLoop(image.data, pixels, loop_gray.data()); };

        const double simd_hot  = bench::medianMicros(REPEATS, simd);
        const double loop_hot  = bench::medianMicros(REPEATS, loop);
        const double simd_cold = bench::medianMicros(REPEATS / 4, simd, flush);
        const double loop_cold = bench::medianMicros(REPEATS / 4, loop, flush);
        const long   differ    = std::inner_product(
            simd_gray.begin(),
            simd_gray.end(),
            loop_gray.begin(),
            0L,
            std::plus<>(),
            std::not_equal_to<>());
        mismatches += differ;

        std::cout << argv[f] << " (" << image.cols << "x" << image.rows << "): " << std::fixed
                  << std::setprecision(1) << "SimdRgbToGray " << simd_hot << " us hot, "
                  << simd_cold << " us cold; loop " << loop_hot << " us hot, " << loop_cold
                  << " us cold; " << differ << " grey levels differ\n";
    }
    return mismatches == 0 ? 0 : 1;
}
