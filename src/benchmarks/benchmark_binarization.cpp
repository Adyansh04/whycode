#include <Simd/SimdLib.h>
#include <immintrin.h>  // For direct intrinsics if needed
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>
// Include xsimd for optimized implementations
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

// Aligned allocator for image data
template <typename T>
using aligned_vector = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

// Packed Binary Image class for 1-bit storage
class PackedBinaryImage
{
public:
    std::vector<uint8_t> data;
    int                  width, height;
    size_t               stride;  // bytes per row

    PackedBinaryImage(int w, int h)
      : width(w)
      , height(h)
    {
        stride = (w + 7) / 8;  // round up to nearest byte
        data.resize(stride * h, 0);
    }

    void clear() { std::fill(data.begin(), data.end(), 0); }

    // Alternative: Change getPixel to match SIMD LSB-first ordering
    bool getPixel(int x, int y) const
    {
        size_t byte_idx = y * stride + x / 8;
        int    bit_idx  = x % 8;  // LSB-first (remove the "7 -")
        return (data[byte_idx] >> bit_idx) & 1;
    }

    void setPixel(int x, int y, bool value)
    {
        size_t byte_idx = y * stride + x / 8;
        int    bit_idx  = 7 - (x % 8);
        if (value)
        {
            data[byte_idx] |= (1 << bit_idx);
        }
        else
        {
            data[byte_idx] &= ~(1 << bit_idx);
        }
    }

    size_t getMemoryUsage() const { return data.size(); }
};

// Packed AlignedBinary Image class for 1-bit storage
class PackedAlignedBinaryImage
{
public:
    aligned_vector<uint8_t> data;
    int                     width, height;
    size_t                  stride;  // bytes per row

    PackedAlignedBinaryImage(int w, int h)
      : width(w)
      , height(h)
    {
        stride = (w + 7) / 8;  // round up to nearest byte
        data.resize(stride * h, 0);
    }

    void clear() { std::fill(data.begin(), data.end(), 0); }

    // Alternative: Change getPixel to match SIMD LSB-first ordering
    bool getPixel(int x, int y) const
    {
        size_t byte_idx = y * stride + x / 8;
        int    bit_idx  = x % 8;  // LSB-first (remove the "7 -")
        return (data[byte_idx] >> bit_idx) & 1;
    }

    void setPixel(int x, int y, bool value)
    {
        size_t byte_idx = y * stride + x / 8;
        int    bit_idx  = 7 - (x % 8);
        if (value)
        {
            data[byte_idx] |= (1 << bit_idx);
        }
        else
        {
            data[byte_idx] &= ~(1 << bit_idx);
        }
    }

    size_t getMemoryUsage() const { return data.size(); }
};

// Function to convert PackedBinaryImage to 8-bit for visualization
cv::Mat convertPackedBinaryToMat(const PackedBinaryImage& packed_img)
{
    cv::Mat result(packed_img.height, packed_img.width, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < packed_img.height; y++)
    {
        for (int x = 0; x < packed_img.width; x++)
        {
            if (packed_img.getPixel(x, y))
            {
                result.at<uint8_t>(y, x) = 255;
            }
        }
    }

    return result;
}

// Function to load and process a real image
std::vector<uint8_t> loadImageAsGrayscale(const std::string& filename, int& width, int& height)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cout << "Warning: Could not load image " << filename
                  << ", using synthetic image instead." << std::endl;
        return {};
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    width  = gray.cols;
    height = gray.rows;

    std::vector<uint8_t> result(width * height);
    memcpy(result.data(), gray.data, width * height);

    return result;
}

// Function to load image into aligned memory
aligned_vector<uint8_t>
loadImageAsGrayscaleAligned(const std::string& filename, int& width, int& height)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cout << "Warning: Could not load image " << filename
                  << ", using synthetic image instead." << std::endl;
        return {};
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    width  = gray.cols;
    height = gray.rows;

    aligned_vector<uint8_t> result(width * height);
    memcpy(result.data(), gray.data, width * height);

    return result;
}

// Function to save visualization results
void saveVisualizationResults(
    const std::vector<uint8_t>& original_image, const std::vector<uint8_t>& binary_8bit,
    const PackedBinaryImage& binary_optimized, const PackedBinaryImage& binary_ultra_fast,
    const PackedAlignedBinaryImage& binary_ultra_fast_aligned,
    const PackedBinaryImage& binary_reduce, int width, int height, uint8_t threshold)
{
    // Create original image Mat
    cv::Mat original_mat(height, width, CV_8UC1, (void*)original_image.data());

    // Create 8-bit binary Mat
    cv::Mat binary_8bit_mat(height, width, CV_8UC1, (void*)binary_8bit.data());

    // Convert 1-bit packed images to 8-bit Mats
    cv::Mat binary_optimized_mat  = convertPackedBinaryToMat(binary_optimized);
    cv::Mat binary_ultra_fast_mat = convertPackedBinaryToMat(binary_ultra_fast);
    // cv::Mat binary_ultra_fast_aligned_mat = convertPackedBinaryToMat(binary_ultra_fast_aligned);
    cv::Mat binary_reduce_mat = convertPackedBinaryToMat(binary_reduce);

    // Save individual images
    cv::imwrite("01_original_image.png", original_mat);
    cv::imwrite("02_simdlib_binary_8bit.png", binary_8bit_mat);
    cv::imwrite("03_simd_optimized_1bit.png", binary_optimized_mat);
    cv::imwrite("04_simd_ultra_fast_1bit.png", binary_ultra_fast_mat);
    // cv::imwrite("05_simd_ultra_fast_aligned_1bit.png", binary_ultra_fast_aligned_mat);
    cv::imwrite("06_simd_reduce_1bit.png", binary_reduce_mat);

    std::cout << "\n=== VISUALIZATION RESULTS SAVED ===" << std::endl;
    std::cout << "Individual images saved:" << std::endl;
    std::cout << "  - 01_original_image.png" << std::endl;
    std::cout << "  - 02_simdlib_binary_8bit.png" << std::endl;
    std::cout << "  - 03_simd_optimized_1bit.png" << std::endl;
    std::cout << "  - 04_simd_ultra_fast_1bit.png" << std::endl;
    std::cout << "  - 05_simd_ultra_fast_aligned_1bit.png" << std::endl;
    std::cout << "  - 06_simd_reduce_1bit.png" << std::endl;
}

// Function to create a test pattern image
std::vector<uint8_t> createTestPatternImage(int width, int height)
{
    std::vector<uint8_t> image(width * height);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // Create interesting test patterns
            int idx = y * width + x;

            // Combine multiple patterns
            float pattern1 = sin(x * 0.01) * sin(y * 0.01);  // Wave pattern
            float pattern2 = ((x / 64) + (y / 64)) % 2;      // Checkerboard
            float pattern3 =
                sqrt((x - width / 2) * (x - width / 2) + (y - height / 2) * (y - height / 2)) /
                (width / 4);  // Radial

            // Combine patterns and add some noise
            float combined = (pattern1 + pattern2 + pattern3) / 3.0f;
            combined += (rand() % 20 - 10) / 255.0f;  // Small noise

            // Convert to 0-255 range
            int pixel_value = static_cast<int>((combined + 1.0f) * 127.5f);
            pixel_value     = std::clamp(pixel_value, 0, 255);

            image[idx] = static_cast<uint8_t>(pixel_value);
        }
    }

    return image;
}

// Function to create aligned test pattern image
aligned_vector<uint8_t> createTestPatternImageAligned(int width, int height)
{
    aligned_vector<uint8_t> image(width * height);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // Create interesting test patterns
            int idx = y * width + x;

            // Combine multiple patterns
            float pattern1 = sin(x * 0.01) * sin(y * 0.01);  // Wave pattern
            float pattern2 = ((x / 64) + (y / 64)) % 2;      // Checkerboard
            float pattern3 =
                sqrt((x - width / 2) * (x - width / 2) + (y - height / 2) * (y - height / 2)) /
                (width / 4);  // Radial

            // Combine patterns and add some noise
            float combined = (pattern1 + pattern2 + pattern3) / 3.0f;
            combined += (rand() % 20 - 10) / 255.0f;  // Small noise

            // Convert to 0-255 range
            int pixel_value = static_cast<int>((combined + 1.0f) * 127.5f);
            pixel_value     = std::clamp(pixel_value, 0, 255);

            image[idx] = static_cast<uint8_t>(pixel_value);
        }
    }

    return image;
}

// Method 1: Original simdlib binarization (8-bit output)
void benchmark_simdlib_binarization(
    const std::vector<uint8_t>& src, int width, int height, uint8_t threshold,
    std::vector<uint8_t>& dst)
{
    dst.resize(width * height);
    SimdBinarization(
        src.data(),
        width,
        width,
        height,
        threshold,
        0,
        255,
        dst.data(),
        width,
        SimdCompareGreater);
}

// Method 2: Highly Optimized SIMD with Proper Bitmask Usage
void benchmark_simd_binarization_optimized(
    const std::vector<uint8_t>& src, int width, int height, uint8_t threshold,
    PackedBinaryImage& dst)
{
    using batch_type           = xs::batch<uint8_t>;
    using batch_bool_type      = xs::batch_bool<uint8_t>;
    constexpr size_t simd_size = batch_type::size;

    dst.clear();
    const batch_type threshold_vec = batch_type(threshold);

    for (int y = 0; y < height; y++)
    {
        const uint8_t* src_row = src.data() + y * width;
        uint8_t*       dst_row = &dst.data[y * dst.stride];

        int x = 0;

        // Process full SIMD width at once - most efficient approach
        for (; x + simd_size <= width; x += simd_size)
        {
            // Load SIMD vector
            auto src_vec = xs::load_unaligned(&src_row[x]);

            // Create boolean mask
            auto mask_vec = src_vec < threshold_vec;

            // Extract bitmask efficiently using xsimd's built-in function
            auto bitmask = mask_vec.mask();  // This gets the native bitmask representation

            // Store the bitmask directly - this is the key optimization
            if constexpr (simd_size == 16)
            {
                // SSE: 16 pixels -> 2 bytes
                uint16_t mask16    = static_cast<uint16_t>(bitmask);
                dst_row[x / 8]     = static_cast<uint8_t>(mask16 & 0xFF);
                dst_row[x / 8 + 1] = static_cast<uint8_t>((mask16 >> 8) & 0xFF);
            }
            else if constexpr (simd_size == 32)
            {
                // AVX2: 32 pixels -> 4 bytes
                uint32_t mask32 = static_cast<uint32_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask32, sizeof(uint32_t));
            }
            else if constexpr (simd_size == 64)
            {
                // AVX-512: 64 pixels -> 8 bytes
                uint64_t mask64 = static_cast<uint64_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask64, sizeof(uint64_t));
            }
            else
            {
                // Fallback for other SIMD sizes
                for (size_t i = 0; i < simd_size; i += 8)
                {
                    uint8_t packed_byte = 0;
                    for (size_t j = 0; j < 8 && (i + j) < simd_size; j++)
                    {
                        if (bitmask & (1ULL << (i + j)))
                        {
                            packed_byte |= (1 << (7 - j));
                        }
                    }
                    dst_row[(x + i) / 8] = packed_byte;
                }
            }
        }

        // Handle remaining pixels with scalar code
        for (; x < width; x++)
        {
            if (src_row[x] < threshold)
            {
                int byte_idx = x / 8;
                int bit_idx  = 7 - (x % 8);
                dst_row[byte_idx] |= (1 << bit_idx);
            }
        }
    }
}

// Method 3: Ultra-Fast Implementation Using Direct Intrinsics
void benchmark_simd_binarization_ultra_fast(
    const std::vector<uint8_t>& src, int width, int height, uint8_t threshold,
    PackedBinaryImage& dst)
{
    using batch_type           = xs::batch<uint8_t>;
    constexpr size_t simd_size = batch_type::size;

    dst.clear();
    const batch_type threshold_vec = batch_type(threshold);

    for (int y = 0; y < height; y++)
    {
        const uint8_t* src_row = src.data() + y * width;
        uint8_t*       dst_row = &dst.data[y * dst.stride];

        int x = 0;

        // Process multiple SIMD vectors at once for maximum throughput
        constexpr int vectors_per_chunk = 4;  // Process 4 SIMD vectors at once
        const int     chunk_size        = vectors_per_chunk * simd_size;

        for (; x + chunk_size <= width; x += chunk_size)
        {
            // Process 4 SIMD vectors in parallel
            for (int v = 0; v < vectors_per_chunk; v++)
            {
                int  offset   = x + v * simd_size;
                auto src_vec  = xs::load_unaligned(&src_row[offset]);
                auto mask_vec = src_vec < threshold_vec;
                auto bitmask  = mask_vec.mask();

                // Store bitmask efficiently
                if constexpr (simd_size >= 32)
                {
                    uint32_t mask32 = static_cast<uint32_t>(bitmask);
                    memcpy(&dst_row[offset / 8], &mask32, sizeof(uint32_t));
                }
                else if constexpr (simd_size >= 16)
                {
                    uint16_t mask16 = static_cast<uint16_t>(bitmask);
                    memcpy(&dst_row[offset / 8], &mask16, sizeof(uint16_t));
                }
                else
                {
                    uint8_t mask8       = static_cast<uint8_t>(bitmask);
                    dst_row[offset / 8] = mask8;
                }
            }
        }

        // Process remaining full SIMD vectors
        for (; x + simd_size <= width; x += simd_size)
        {
            auto src_vec  = xs::load_unaligned(&src_row[x]);
            auto mask_vec = src_vec < threshold_vec;
            auto bitmask  = mask_vec.mask();

            if constexpr (simd_size >= 32)
            {
                uint32_t mask32 = static_cast<uint32_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask32, sizeof(uint32_t));
            }
            else if constexpr (simd_size >= 16)
            {
                uint16_t mask16 = static_cast<uint16_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask16, sizeof(uint16_t));
            }
            else
            {
                uint8_t mask8  = static_cast<uint8_t>(bitmask);
                dst_row[x / 8] = mask8;
            }
        }

        // Handle remaining pixels
        for (; x < width; x++)
        {
            if (src_row[x] < threshold)
            {
                int byte_idx = x / 8;
                int bit_idx  = 7 - (x % 8);
                dst_row[byte_idx] |= (1 << bit_idx);
            }
        }
    }
}

// Method 4: Alternative using xsimd reduce operations for comparison
void benchmark_simd_binarization_reduce(
    const std::vector<uint8_t>& src, int width, int height, uint8_t threshold,
    PackedBinaryImage& dst)
{
    using batch_type           = xs::batch<uint8_t>;
    constexpr size_t simd_size = batch_type::size;

    dst.clear();
    const batch_type threshold_vec = batch_type(threshold);

    for (int y = 0; y < height; y++)
    {
        const uint8_t* src_row = src.data() + y * width;
        uint8_t*       dst_row = &dst.data[y * dst.stride];

        int x = 0;

        for (; x + simd_size <= width; x += simd_size)
        {
            auto src_vec  = xs::load_unaligned(&src_row[x]);
            auto mask_vec = src_vec < threshold_vec;

            if (xs::any(mask_vec))
            {
                auto bitmask = mask_vec.mask();

                // Efficient bit storage based on SIMD width
                const size_t bytes_needed = (simd_size + 7) / 8;
                memcpy(&dst_row[x / 8], &bitmask, bytes_needed);
            }
            // If no bits are set, the memory is already zero from clear()
        }

        // Handle remaining pixels
        for (; x < width; x++)
        {
            if (src_row[x] < threshold)
            {
                int byte_idx = x / 8;
                int bit_idx  = 7 - (x % 8);
                dst_row[byte_idx] |= (1 << bit_idx);
            }
        }
    }
}

// Method 5: Ultra-Fast Implementation with Aligned Memory Access
void benchmark_simd_binarization_ultra_fast_aligned(
    const aligned_vector<uint8_t>& src, int width, int height, uint8_t threshold,
    PackedAlignedBinaryImage& dst)
{
    using batch_type           = xs::batch<uint8_t>;
    constexpr size_t simd_size = batch_type::size;

    dst.clear();
    const batch_type threshold_vec = batch_type(threshold);

    for (int y = 0; y < height; y++)
    {
        const uint8_t* src_row = src.data() + y * width;
        uint8_t*       dst_row = &dst.data[y * dst.stride];

        int x = 0;

        // Process multiple SIMD vectors at once for maximum throughput with aligned loads
        constexpr int vectors_per_chunk = 4;  // Process 4 SIMD vectors at once
        const int     chunk_size        = vectors_per_chunk * simd_size;

        for (; x + chunk_size <= width; x += chunk_size)
        {
            // Process 4 SIMD vectors in parallel with aligned loads
            for (int v = 0; v < vectors_per_chunk; v++)
            {
                int  offset   = x + v * simd_size;
                auto src_vec  = xs::load_aligned(&src_row[offset]);  // ALIGNED LOAD
                auto mask_vec = src_vec < threshold_vec;
                auto bitmask  = mask_vec.mask();

                // Store bitmask efficiently
                if constexpr (simd_size >= 32)
                {
                    uint32_t mask32 = static_cast<uint32_t>(bitmask);
                    memcpy(&dst_row[offset / 8], &mask32, sizeof(uint32_t));
                }
                else if constexpr (simd_size >= 16)
                {
                    uint16_t mask16 = static_cast<uint16_t>(bitmask);
                    memcpy(&dst_row[offset / 8], &mask16, sizeof(uint16_t));
                }
                else
                {
                    uint8_t mask8       = static_cast<uint8_t>(bitmask);
                    dst_row[offset / 8] = mask8;
                }
            }
        }

        // Process remaining full SIMD vectors with aligned loads
        for (; x + simd_size <= width; x += simd_size)
        {
            auto src_vec  = xs::load_aligned(&src_row[x]);  // ALIGNED LOAD
            auto mask_vec = src_vec < threshold_vec;
            auto bitmask  = mask_vec.mask();

            if constexpr (simd_size >= 32)
            {
                uint32_t mask32 = static_cast<uint32_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask32, sizeof(uint32_t));
            }
            else if constexpr (simd_size >= 16)
            {
                uint16_t mask16 = static_cast<uint16_t>(bitmask);
                memcpy(&dst_row[x / 8], &mask16, sizeof(uint16_t));
            }
            else
            {
                uint8_t mask8  = static_cast<uint8_t>(bitmask);
                dst_row[x / 8] = mask8;
            }
        }

        // Handle remaining pixels
        for (; x < width; x++)
        {
            if (src_row[x] < threshold)
            {
                int byte_idx = x / 8;
                int bit_idx  = 7 - (x % 8);
                dst_row[byte_idx] |= (1 << bit_idx);
            }
        }
    }
}

// Performance measurement utilities
struct BenchmarkResult
{
    double      time_ms;
    size_t      memory_bytes;
    double      cpu_usage_percent;
    std::string method_name;
};

struct CPUUsage
{
    double user_time;
    double system_time;
    double total_time;
};

CPUUsage get_cpu_usage()
{
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);

    CPUUsage cpu;
    cpu.user_time   = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time  = cpu.user_time + cpu.system_time;
    return cpu;
}

BenchmarkResult run_benchmark(
    const std::function<void()>& func, const std::string& name, size_t memory_usage,
    int iterations = 1000)
{
    std::cout << "Running benchmark: " << name << "..." << std::endl;

    CPUUsage start_cpu  = get_cpu_usage();
    auto     start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++)
    {
        func();
    }

    auto     end_time = std::chrono::high_resolution_clock::now();
    CPUUsage end_cpu  = get_cpu_usage();

    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    double time_ms   = duration / 1000.0;
    double cpu_usage = (end_cpu.total_time - start_cpu.total_time) * 100.0 / (duration / 1e6);

    return { time_ms, memory_usage, cpu_usage, name };
}

int main()
{
    int           width      = 1920;
    int           height     = 1080;
    const uint8_t threshold  = 170;
    const int     iterations = 1000;

    std::cout << "=== Final Optimized SIMD Binarization Benchmark (All 5 Methods) ===" << std::endl;
    std::cout << "Image size: " << width << "×" << height << " pixels" << std::endl;
    std::cout << "Threshold: " << (int)threshold << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << "SIMD width: " << xs::batch<uint8_t>::size << " bytes" << std::endl;
    std::cout << "Architecture: ";

    // Detect SIMD architecture
    if constexpr (xs::batch<uint8_t>::size == 64)
    {
        std::cout << "AVX-512" << std::endl;
    }
    else if constexpr (xs::batch<uint8_t>::size == 32)
    {
        std::cout << "AVX2" << std::endl;
    }
    else if constexpr (xs::batch<uint8_t>::size == 16)
    {
        std::cout << "SSE/NEON" << std::endl;
    }
    else
    {
        std::cout << "Unknown (" << xs::batch<uint8_t>::size << " bytes)" << std::endl;
    }
    std::cout << std::endl;

    // Load image data (both regular and aligned)
    std::vector<uint8_t>    test_image;
    aligned_vector<uint8_t> test_image_aligned;
    std::string image_path = "/root/ros1_ws/src/whycode_vision/src/cpp_tests/image copy.png";

    test_image = loadImageAsGrayscale(image_path, width, height);

    if (test_image.empty())
    {
        // Fallback: create test patterns
        std::cout << "Creating test pattern image..." << std::endl;
        width              = 1920;
        height             = 1080;
        test_image         = createTestPatternImage(width, height);
        test_image_aligned = createTestPatternImageAligned(width, height);
    }
    else
    {
        std::cout << "Loaded image: " << image_path << " (" << width << "x" << height << ")"
                  << std::endl;
        // Copy to aligned memory
        test_image_aligned = loadImageAsGrayscaleAligned(image_path, width, height);
    }

    // Prepare output containers
    std::vector<uint8_t>     binary_8bit;
    PackedBinaryImage        binary_optimized(width, height);
    PackedBinaryImage        binary_ultra_fast(width, height);
    PackedAlignedBinaryImage binary_ultra_fast_aligned(width, height);
    PackedBinaryImage        binary_reduce(width, height);

    std::vector<BenchmarkResult> results;

    // Benchmark 1: Original simdlib method (baseline)
    results.push_back(run_benchmark(
        [&]() { benchmark_simdlib_binarization(test_image, width, height, threshold, binary_8bit); },
        "Original SimdBinarization (8-bit)",
        width * height,
        iterations));

    // Benchmark 2: Optimized SIMD with proper bitmask
    results.push_back(run_benchmark(
        [&]() {
            benchmark_simd_binarization_optimized(
                test_image,
                width,
                height,
                threshold,
                binary_optimized);
        },
        "SIMD Optimized Bitmask",
        binary_optimized.getMemoryUsage(),
        iterations));

    // Benchmark 3: Ultra-fast chunked processing (unaligned)
    results.push_back(run_benchmark(
        [&]() {
            benchmark_simd_binarization_ultra_fast(
                test_image,
                width,
                height,
                threshold,
                binary_ultra_fast);
        },
        "SIMD Ultra-Fast Chunked",
        binary_ultra_fast.getMemoryUsage(),
        iterations));

    // Benchmark 4: Reduce-based approach
    results.push_back(run_benchmark(
        [&]() {
            benchmark_simd_binarization_reduce(test_image, width, height, threshold, binary_reduce);
        },
        "SIMD Reduce-Based",
        binary_reduce.getMemoryUsage(),
        iterations));

    // Benchmark 5: NEW Ultra-fast with aligned memory access
    results.push_back(run_benchmark(
        [&]() {
            benchmark_simd_binarization_ultra_fast_aligned(
                test_image_aligned,
                width,
                height,
                threshold,
                binary_ultra_fast_aligned);
        },
        "SIMD Ultra-Fast Aligned",
        binary_ultra_fast_aligned.getMemoryUsage(),
        iterations));

    // Display results
    std::cout << "\n=== FINAL BENCHMARK RESULTS (All 5 Methods) ===" << std::endl;
    std::cout << std::left << std::setw(32) << "Method" << std::setw(12) << "Time (ms)"
              << std::setw(15) << "Memory (bytes)" << std::setw(12) << "CPU Usage" << std::setw(10)
              << "Speedup" << std::setw(12) << "Efficiency" << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    double baseline_time   = results[0].time_ms;
    size_t baseline_memory = results[0].memory_bytes;

    for (const auto& result : results)
    {
        double speedup            = baseline_time / result.time_ms;
        double memory_efficiency  = (double)baseline_memory / result.memory_bytes;
        double overall_efficiency = speedup * memory_efficiency;

        std::cout << std::left << std::setw(32) << result.method_name << std::setw(12) << std::fixed
                  << std::setprecision(2) << result.time_ms << std::setw(15) << result.memory_bytes
                  << std::setw(12) << std::fixed << std::setprecision(1) << result.cpu_usage_percent
                  << "%" << std::setw(10) << std::fixed << std::setprecision(2) << speedup << "x"
                  << std::setw(12) << std::fixed << std::setprecision(1) << overall_efficiency
                  << std::endl;
    }

    // Memory and performance analysis
    std::cout << "\n=== PERFORMANCE ANALYSIS ===" << std::endl;
    size_t memory_8bit      = results[0].memory_bytes;
    size_t memory_1bit      = results[1].memory_bytes;
    double memory_reduction = (1.0 - (double)memory_1bit / memory_8bit) * 100;
    double best_speedup =
        baseline_time /
        std::min({ results[1].time_ms, results[2].time_ms, results[3].time_ms, results[4].time_ms });

    std::cout << "Memory reduction: " << std::fixed << std::setprecision(1) << memory_reduction
              << "% (" << memory_8bit << " → " << memory_1bit << " bytes)" << std::endl;
    std::cout << "Best speedup achieved: " << std::fixed << std::setprecision(2) << best_speedup
              << "x" << std::endl;
    std::cout << "Theoretical max speedup: " << xs::batch<uint8_t>::size << "x (SIMD width)"
              << std::endl;
    std::cout << "Efficiency: " << std::fixed << std::setprecision(1)
              << (best_speedup / xs::batch<uint8_t>::size * 100) << "% of theoretical maximum"
              << std::endl;

    // Correctness verification
    std::cout << "\n=== CORRECTNESS VERIFICATION ===" << std::endl;
    int total_tests       = 1000;
    int correct_optimized = 0, correct_ultra = 0, correct_ultra_aligned = 0, correct_reduce = 0;

    for (int i = 0; i < total_tests; i++)
    {
        int x = (i * 17) % width;
        int y = (i * 13) % height;

        uint8_t original = test_image[y * width + x];
        bool    expected = original < threshold;

        if (binary_optimized.getPixel(x, y) == expected)
            correct_optimized++;
        if (binary_ultra_fast.getPixel(x, y) == expected)
            correct_ultra++;
        if (binary_ultra_fast_aligned.getPixel(x, y) == expected)
            correct_ultra_aligned++;
        if (binary_reduce.getPixel(x, y) == expected)
            correct_reduce++;
    }

    std::cout << "Optimized method: " << correct_optimized << "/" << total_tests << " ("
              << (100.0 * correct_optimized / total_tests) << "%)" << std::endl;
    std::cout << "Ultra-fast method: " << correct_ultra << "/" << total_tests << " ("
              << (100.0 * correct_ultra / total_tests) << "%)" << std::endl;
    std::cout << "Ultra-fast aligned method: " << correct_ultra_aligned << "/" << total_tests
              << " (" << (100.0 * correct_ultra_aligned / total_tests) << "%)" << std::endl;
    std::cout << "Reduce method: " << correct_reduce << "/" << total_tests << " ("
              << (100.0 * correct_reduce / total_tests) << "%)" << std::endl;

    std::cout << "\n=== PERF COMMANDS ===" << std::endl;
    std::cout << "# Comprehensive analysis:" << std::endl;
    std::cout << "perf stat -e "
                 "cycles,instructions,cache-references,cache-misses,branches,branch-misses,LLC-"
                 "loads,LLC-load-misses ./final_benchmark"
              << std::endl;
    std::cout << std::endl;
    std::cout << "# SIMD instruction analysis:" << std::endl;
    std::cout << "perf stat -e "
                 "fp_arith_inst_retired.128b_packed_single,fp_arith_inst_retired.256b_packed_"
                 "single,fp_arith_inst_retired.512b_packed_single ./final_benchmark"
              << std::endl;
    std::cout << std::endl;
    std::cout << "# Memory bandwidth:" << std::endl;
    std::cout << "perf stat -e uncore_imc/data_reads/,uncore_imc/data_writes/ ./final_benchmark"
              << std::endl;

    // === VISUALIZATION SECTION ===
    std::cout << "\n=== GENERATING VISUALIZATIONS ===" << std::endl;

    // Run the algorithms one more time for visualization
    benchmark_simdlib_binarization(test_image, width, height, threshold, binary_8bit);
    benchmark_simd_binarization_optimized(test_image, width, height, threshold, binary_optimized);
    benchmark_simd_binarization_ultra_fast(test_image, width, height, threshold, binary_ultra_fast);
    benchmark_simd_binarization_ultra_fast_aligned(
        test_image_aligned,
        width,
        height,
        threshold,
        binary_ultra_fast_aligned);
    benchmark_simd_binarization_reduce(test_image, width, height, threshold, binary_reduce);

    // Save visualization results
    saveVisualizationResults(
        test_image,
        binary_8bit,
        binary_optimized,
        binary_ultra_fast,
        binary_ultra_fast_aligned,
        binary_reduce,
        width,
        height,
        threshold);

    std::cout << "Visualization complete! Check the generated PNG files." << std::endl;

    return 0;
}
