
#include <iostream>
#include <numeric>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/hal/intrin.hpp"

// Scalar implementation for comparing two arrays (src1 > src2)
void scalar_compare(
    const std::vector<float>& src1, const std::vector<float>& src2, std::vector<uchar>& dst)
{
    for (size_t i = 0; i < src1.size(); ++i)
    {
        dst[i] = src1[i] > src2[i] ? 255 : 0;
    }
}
void simd_compare(
    const std::vector<float>& src1, const std::vector<float>& src2, std::vector<uchar>& dst)
{
    const int    step     = cv::v_float32::nlanes;
    const size_t vec_size = src1.size() - (src1.size() % step);

    for (size_t i = 0; i < vec_size; i += step)
    {
        // Load data into SIMD registers
        cv::v_float32 v_src1 = cv::v_load(src1.data() + i);
        cv::v_float32 v_src2 = cv::v_load(src2.data() + i);

        // Perform the parallel comparison and reinterpret the resulting bitmask as unsigned integers.
        cv::v_uint32x4 v_mask32 = cv::v_reinterpret_as_u32(v_src1 > v_src2);

        // Pack the 32-bit integer results down to 16-bit.
        cv::v_uint16x8 v_mask16 = cv::v_pack(v_mask32, v_mask32);

        // Pack the 16-bit results down to 8-bit.
        // Use v_pack, which truncates, correctly converting 0xFFFF to 0xFF (255).
        cv::v_uint8x16 v_mask8 = cv::v_pack(v_mask16, v_mask16);

        // Store the 8-bit results.
        cv::v_store_low(dst.data() + i, v_mask8);
    }
}
int main()
{
    cv::setUseOptimized(true);
    std::cout << "OpenCV optimizations are " << (cv::useOptimized() ? "ENABLED" : "DISABLED")
              << std::endl;
    std::cout << "Using SIMD vector width (nlanes): " << cv::v_float32::nlanes << " floats"
              << std::endl
              << std::endl;

    const int size       = 4;
    const int iterations = 50000000;  // 50 million iterations

    std::cout << "Benchmarking vector comparison for size: " << size << " over " << iterations
              << " iterations." << std::endl;

    // Initialize data vectors
    std::vector<float> src1(size), src2(size);
    std::vector<uchar> dst_scalar(size), dst_simd(size);
    cv::randu(src1, cv::Scalar::all(0), cv::Scalar::all(100));
    cv::randu(src2, cv::Scalar::all(0), cv::Scalar::all(100));

    // --- Benchmark Scalar Operation ---
    double scalar_start = (double)cv::getTickCount();
    for (int i = 0; i < iterations; ++i)
    {
        scalar_compare(src1, src2, dst_scalar);
    }
    double scalar_time = ((double)cv::getTickCount() - scalar_start) / cv::getTickFrequency();
    std::cout << "Scalar comparison time: " << scalar_time << " seconds" << std::endl;

    // --- Benchmark SIMD Operation ---
    double simd_start = (double)cv::getTickCount();
    for (int i = 0; i < iterations; ++i)
    {
        simd_compare(src1, src2, dst_simd);
    }
    double simd_time = ((double)cv::getTickCount() - simd_start) / cv::getTickFrequency();
    std::cout << "SIMD comparison time:   " << simd_time << " seconds" << std::endl;

    // --- Verify results ---
    std::cout << "\nScalar result: ";
    for (const auto& val : dst_scalar)
    {
        std::cout << (int)val << " ";
    }
    std::cout << "\nSIMD result:   ";
    for (const auto& val : dst_simd)
    {
        std::cout << (int)val << " ";
    }
    std::cout << std::endl;

    double diff = cv::norm(dst_scalar, dst_simd, cv::NORM_L1);
    std::cout << "\nDifference between scalar and SIMD results: " << diff << std::endl;
    if (diff > 1e-5)
    {
        std::cerr << "Verification FAILED: Results do not match." << std::endl;
    }
    else
    {
        std::cout << "Verification PASSED: Results match." << std::endl;
    }

    return 0;
}