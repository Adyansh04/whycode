#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

// Include xsimd
#include "xsimd/memory/xsimd_aligned_allocator.hpp"
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

// Define aligned vector type
template <typename T>
using aligned_vector = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

struct Cache {
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float sum_yy = 0.0f;

    void reset() { sum_x = sum_y = sum_xx = sum_xy = sum_yy = 0.0f; }

    void print() const {
        std::cout << "sum_x=" << sum_x << ", sum_y=" << sum_y << ", sum_xx=" << sum_xx << ", sum_xy=" << sum_xy
                  << ", sum_yy=" << sum_yy << std::endl;
    }
};

// Original scalar implementation
void compute_stats_scalar(const std::vector<int>& queue, int start, int end, int width, Cache& cache) {
    for (int p = start; p < end; p++) {
        const int   idx        = queue[p];
        auto        div_result = std::div(idx, width);
        const float px         = static_cast<float>(div_result.rem);   // x-coordinate
        const float py         = static_cast<float>(div_result.quot);  // y-coordinate
        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

// SIMD type definitions and constants
using batch_int                 = xs::batch<int>;
using batch_float               = xs::batch<float>;
constexpr std::size_t simd_size = batch_int::size;

// Pre-allocated SIMD vectors for reuse
static batch_float sum_x_vec_storage(0.0f);
static batch_float sum_y_vec_storage(0.0f);
static batch_float sum_xx_vec_storage(0.0f);
static batch_float sum_xy_vec_storage(0.0f);
static batch_float sum_yy_vec_storage(0.0f);

// Cache for width vector
static int       cached_width = -1;
static batch_int cached_width_vec;

// SIMD optimized implementation
void compute_stats_simd(const std::vector<int>& queue, int start, int end, int width, Cache& cache) {
    // Reset accumulators
    sum_x_vec_storage  = batch_float(0.0f);
    sum_y_vec_storage  = batch_float(0.0f);
    sum_xx_vec_storage = batch_float(0.0f);
    sum_xy_vec_storage = batch_float(0.0f);
    sum_yy_vec_storage = batch_float(0.0f);

    // Cache width vector if it hasn't changed
    if (cached_width != width) {
        cached_width     = width;
        cached_width_vec = batch_int(width);
    }

    // Process SIMD-sized chunks
    int       p        = start;
    const int simd_end = start + ((end - start) / simd_size) * simd_size;

    for (; p < simd_end; p += simd_size) {
        // Load indices directly as integers
        auto idx_vec = xs::load_unaligned(&queue[p]);

        // Compute coordinates using cached width vector
        auto py_vec = idx_vec / cached_width_vec;
        auto px_vec = idx_vec % cached_width_vec;

        // Convert to float for accumulation
        auto px_f = xs::batch_cast<float>(px_vec);
        auto py_f = xs::batch_cast<float>(py_vec);

        // Accumulate statistics
        sum_x_vec_storage += px_f;
        sum_y_vec_storage += py_f;
        sum_xx_vec_storage += px_f * px_f;
        sum_xy_vec_storage += px_f * py_f;
        sum_yy_vec_storage += py_f * py_f;
    }

    // Reduce SIMD accumulators to scalars
    cache.sum_x  = xs::reduce_add(sum_x_vec_storage);
    cache.sum_y  = xs::reduce_add(sum_y_vec_storage);
    cache.sum_xx = xs::reduce_add(sum_xx_vec_storage);
    cache.sum_xy = xs::reduce_add(sum_xy_vec_storage);
    cache.sum_yy = xs::reduce_add(sum_yy_vec_storage);

    // Process remaining elements
    for (; p < end; p++) {
        const int   idx = queue[p];
        const float px  = idx % width;
        const float py  = idx / width;
        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

// Pre-allocated SIMD vectors for reuse (Aligned SIMD)
static batch_float sum_x_vec_aligned(0.0f);
static batch_float sum_y_vec_aligned(0.0f);
static batch_float sum_xx_vec_aligned(0.0f);
static batch_float sum_xy_vec_aligned(0.0f);
static batch_float sum_yy_vec_aligned(0.0f);

// SIMD implementation with aligned memory
void compute_stats_simd_aligned(const aligned_vector<int>& queue, int start, int end, int width, Cache& cache) {
    // Reset accumulators
    sum_x_vec_aligned  = batch_float(0.0f);
    sum_y_vec_aligned  = batch_float(0.0f);
    sum_xx_vec_aligned = batch_float(0.0f);
    sum_xy_vec_aligned = batch_float(0.0f);
    sum_yy_vec_aligned = batch_float(0.0f);

    // Cache width vector if it hasn't changed
    if (cached_width != width) {
        cached_width     = width;
        cached_width_vec = batch_int(width);
    }

    // Process SIMD-sized chunks
    int       p        = start;
    const int simd_end = start + ((end - start) / simd_size) * simd_size;

    for (; p < simd_end; p += simd_size) {
        // Load indices directly as integers
        auto idx_vec = xs::load_aligned(&queue[p]);

        // Compute coordinates using cached width vector
        auto py_vec = idx_vec / cached_width_vec;
        auto px_vec = idx_vec % cached_width_vec;

        // Convert to float for accumulation
        auto px_f = xs::batch_cast<float>(px_vec);
        auto py_f = xs::batch_cast<float>(py_vec);

        // Accumulate statistics
        sum_x_vec_aligned += px_f;
        sum_y_vec_aligned += py_f;
        sum_xx_vec_aligned += px_f * px_f;
        sum_xy_vec_aligned += px_f * py_f;
        sum_yy_vec_aligned += py_f * py_f;
    }

    // Reduce SIMD accumulators to scalars
    cache.sum_x  = xs::reduce_add(sum_x_vec_aligned);
    cache.sum_y  = xs::reduce_add(sum_y_vec_aligned);
    cache.sum_xx = xs::reduce_add(sum_xx_vec_aligned);
    cache.sum_xy = xs::reduce_add(sum_xy_vec_aligned);
    cache.sum_yy = xs::reduce_add(sum_yy_vec_aligned);

    // Process remaining elements
    for (; p < end; p++) {
        const int   idx = queue[p];
        const float px  = idx % width;
        const float py  = idx / width;
        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

// Pre-allocated SIMD vectors for optimized version
static batch_float sum_x_vec_opt(0.0f);
static batch_float sum_y_vec_opt(0.0f);
static batch_float sum_xx_vec_opt(0.0f);
static batch_float sum_xy_vec_opt(0.0f);
static batch_float sum_yy_vec_opt(0.0f);

// Cache for optimized version
static int cached_width_opt = -1;

// SIMD optimized implementation - with loop unrolling
void compute_stats_simd_optimized(const std::vector<int>& queue, int start, int end, int width, Cache& cache) {
    // Reset pre-allocated SIMD vectors
    sum_x_vec_opt  = batch_float(0.0f);
    sum_y_vec_opt  = batch_float(0.0f);
    sum_xx_vec_opt = batch_float(0.0f);
    sum_xy_vec_opt = batch_float(0.0f);
    sum_yy_vec_opt = batch_float(0.0f);

    const batch_int width_vec(width);

    // Process larger chunks with loop unrolling for better instruction-level parallelism
    int       p                 = start;
    const int unroll_factor     = 4;  // Process 4 SIMD vectors per iteration
    const int unroll_chunk_size = unroll_factor * simd_size;
    const int unroll_end        = start + ((end - start) / unroll_chunk_size) * unroll_chunk_size;

    // Unrolled SIMD loop - processes 4 SIMD vectors at once
    for (; p < unroll_end; p += unroll_chunk_size) {
        // Prefetch next cache lines to improve memory access patterns
        __builtin_prefetch(&queue[p + unroll_chunk_size], 0, 3);

        // Process 4 SIMD vectors in parallel
        for (int unroll = 0; unroll < unroll_factor; unroll++) {
            int offset = p + unroll * simd_size;

            // Load indices
            auto idx_vec = xs::load_unaligned(&queue[offset]);

            // Compute coordinates using integer operations
            auto py_vec = idx_vec / width_vec;
            auto px_vec = idx_vec % width_vec;

            // Convert to float for accumulation
            auto px_f = xs::batch_cast<float>(px_vec);
            auto py_f = xs::batch_cast<float>(py_vec);

            // Accumulate statistics
            sum_x_vec_opt += px_f;
            sum_y_vec_opt += py_f;
            sum_xx_vec_opt += px_f * px_f;
            sum_xy_vec_opt += px_f * py_f;
            sum_yy_vec_opt += py_f * py_f;
        }
    }

    // Process remaining full SIMD chunks
    const int simd_end = start + ((end - start) / simd_size) * simd_size;
    for (; p < simd_end; p += simd_size) {
        auto idx_vec = xs::load_unaligned(&queue[p]);
        auto py_vec  = idx_vec / width_vec;
        auto px_vec  = idx_vec % width_vec;

        auto px_f = xs::batch_cast<float>(px_vec);
        auto py_f = xs::batch_cast<float>(py_vec);

        sum_x_vec_opt += px_f;
        sum_y_vec_opt += py_f;
        sum_xx_vec_opt += px_f * px_f;
        sum_xy_vec_opt += px_f * py_f;
        sum_yy_vec_opt += py_f * py_f;
    }

    // Reduce SIMD accumulators to scalars
    cache.sum_x  = xs::reduce_add(sum_x_vec_opt);
    cache.sum_y  = xs::reduce_add(sum_y_vec_opt);
    cache.sum_xx = xs::reduce_add(sum_xx_vec_opt);
    cache.sum_xy = xs::reduce_add(sum_xy_vec_opt);
    cache.sum_yy = xs::reduce_add(sum_yy_vec_opt);

    // Optimized scalar fallback with fewer operations
    for (; p < end; p++) {
        const int   idx = queue[p];
        const float py  = static_cast<float>(idx / width);  // Integer division, then convert
        const float px  = static_cast<float>(idx % width);  // Integer modulo, then convert

        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

// Pre-allocated SIMD vectors for aligned version
static batch_float sum_x_vec_opt_aligned(0.0f);
static batch_float sum_y_vec_opt_aligned(0.0f);
static batch_float sum_xx_vec_opt_aligned(0.0f);
static batch_float sum_xy_vec_opt_aligned(0.0f);
static batch_float sum_yy_vec_opt_aligned(0.0f);

// SIMD optimized implementation with aligned memory - with loop unrolling
void compute_stats_simd_optimized_aligned(const aligned_vector<int>& queue, int start, int end, int width,
                                          Cache& cache) {
    // Reset pre-allocated SIMD vectors
    sum_x_vec_opt_aligned  = batch_float(0.0f);
    sum_y_vec_opt_aligned  = batch_float(0.0f);
    sum_xx_vec_opt_aligned = batch_float(0.0f);
    sum_xy_vec_opt_aligned = batch_float(0.0f);
    sum_yy_vec_opt_aligned = batch_float(0.0f);

    const batch_int width_vec(width);

    // Process larger chunks with loop unrolling for better instruction-level parallelism
    int       p                 = start;
    const int unroll_factor     = 4;  // Process 4 SIMD vectors per iteration
    const int unroll_chunk_size = unroll_factor * simd_size;
    const int unroll_end        = start + ((end - start) / unroll_chunk_size) * unroll_chunk_size;

    // Unrolled SIMD loop - processes 4 SIMD vectors at once
    for (; p < unroll_end; p += unroll_chunk_size) {
        // Prefetch next cache lines to improve memory access patterns
        __builtin_prefetch(&queue[p + unroll_chunk_size], 0, 3);

        // Process 4 SIMD vectors in parallel
        for (int unroll = 0; unroll < unroll_factor; unroll++) {
            int offset = p + unroll * simd_size;

            // Load indices - using aligned load
            auto idx_vec = xs::load_aligned(&queue[offset]);

            // Compute coordinates using integer operations
            auto py_vec = idx_vec / width_vec;
            auto px_vec = idx_vec % width_vec;

            // Convert to float for accumulation
            auto px_f = xs::batch_cast<float>(px_vec);
            auto py_f = xs::batch_cast<float>(py_vec);

            // Accumulate statistics
            sum_x_vec_opt_aligned += px_f;
            sum_y_vec_opt_aligned += py_f;
            sum_xx_vec_opt_aligned += px_f * px_f;
            sum_xy_vec_opt_aligned += px_f * py_f;
            sum_yy_vec_opt_aligned += py_f * py_f;
        }
    }

    // Process remaining full SIMD chunks
    const int simd_end = start + ((end - start) / simd_size) * simd_size;
    for (; p < simd_end; p += simd_size) {
        auto idx_vec = xs::load_aligned(&queue[p]);
        auto py_vec  = idx_vec / width_vec;
        auto px_vec  = idx_vec % width_vec;

        auto px_f = xs::batch_cast<float>(px_vec);
        auto py_f = xs::batch_cast<float>(py_vec);

        sum_x_vec_opt_aligned += px_f;
        sum_y_vec_opt_aligned += py_f;
        sum_xx_vec_opt_aligned += px_f * px_f;
        sum_xy_vec_opt_aligned += px_f * py_f;
        sum_yy_vec_opt_aligned += py_f * py_f;
    }

    // Reduce SIMD accumulators to scalars
    cache.sum_x  = xs::reduce_add(sum_x_vec_opt_aligned);
    cache.sum_y  = xs::reduce_add(sum_y_vec_opt_aligned);
    cache.sum_xx = xs::reduce_add(sum_xx_vec_opt_aligned);
    cache.sum_xy = xs::reduce_add(sum_xy_vec_opt_aligned);
    cache.sum_yy = xs::reduce_add(sum_yy_vec_opt_aligned);

    // Optimized scalar fallback with fewer operations
    for (; p < end; p++) {
        const int   idx = queue[p];
        const float py  = static_cast<float>(idx / width);  // Integer division, then convert
        const float px  = static_cast<float>(idx % width);  // Integer modulo, then convert

        cache.sum_x += px;
        cache.sum_y += py;
        cache.sum_xx += px * px;
        cache.sum_xy += px * py;
        cache.sum_yy += py * py;
    }
}

// CPU usage measurement
struct CPUUsage {
    double user_time;
    double system_time;
    double total_time;
};

CPUUsage get_cpu_usage() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);

    CPUUsage cpu;
    cpu.user_time   = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time  = cpu.user_time + cpu.system_time;
    return cpu;
}

// Benchmark function
template <typename Func, typename QueueType>
double benchmark_function(const std::string& name, Func&& func, const QueueType& queue, int start, int end, int width,
                          int iterations = 1000) {
    Cache    cache;
    CPUUsage start_cpu = get_cpu_usage();

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        cache.reset();
        func(queue, start, end, width, cache);
    }

    auto     end_time = std::chrono::high_resolution_clock::now();
    CPUUsage end_cpu  = get_cpu_usage();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    double cpu_usage = (end_cpu.total_time - start_cpu.total_time) * 100.0 / (duration / 1e6);

    std::cout << "=== " << name << " ===" << std::endl;
    std::cout << "CPU Usage: " << std::fixed << std::setprecision(2) << cpu_usage << "%" << std::endl;

    // Print final cache values for verification
    cache.print();

    double avg_time = duration / 1000.0;  // Convert to milliseconds
    std::cout << "Average time: " << std::fixed << std::setprecision(3) << avg_time << " ms" << std::endl << std::endl;

    return avg_time;
}

int main() {
    const int width        = 1920;
    const int height       = 1080;
    const int total_pixels = width * height;
    const int test_size    = 100000;

    // Generate test data - standard vector
    std::vector<int>                   queue;
    std::random_device                 rd;
    std::mt19937                       gen(42);  // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dis(0, total_pixels - 1);

    queue.reserve(test_size);
    for (int i = 0; i < test_size; ++i) {
        queue.push_back(dis(gen));
    }

    // Create aligned version of the test data
    aligned_vector<int> aligned_queue(queue.begin(), queue.end());

    const int iterations = 1000;

    std::cout << "=== Performance Benchmark ===" << std::endl;
    std::cout << "Test size: " << test_size << " elements" << std::endl;
    std::cout << "Width: " << width << ", Height: " << height << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << "SIMD width: " << xs::batch<int>::size << " integers" << std::endl;
    std::cout << "Unroll factor: 4x" << std::endl;
    std::cout << std::endl;

    // Benchmark all versions
    double scalar_time =
            benchmark_function("Scalar Implementation", compute_stats_scalar, queue, 0, test_size, width, iterations);

    double simd_time = benchmark_function("SIMD Implementation (Unaligned)", compute_stats_simd, queue, 0, test_size,
                                          width, iterations);

    double simd_aligned_time = benchmark_function("SIMD Implementation (Aligned)", compute_stats_simd_aligned,
                                                  aligned_queue, 0, test_size, width, iterations);

    double simd_opt_time = benchmark_function("SIMD Optimized Implementation (Unaligned)", compute_stats_simd_optimized,
                                              queue, 0, test_size, width, iterations);

    double simd_opt_aligned_time =
            benchmark_function("SIMD Optimized Implementation (Aligned)", compute_stats_simd_optimized_aligned,
                               aligned_queue, 0, test_size, width, iterations);

    // Performance comparison
    std::cout << "=== Performance Analysis ===" << std::endl;

    // Standard SIMD vs Scalar
    double speedup_simd     = scalar_time / simd_time;
    double improvement_simd = ((scalar_time - simd_time) / scalar_time) * 100.0;
    std::cout << "Scalar vs SIMD (Unaligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_simd << "x" << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_simd << "%" << std::endl;
    std::cout << std::endl;

    // Aligned SIMD vs Scalar
    double speedup_aligned     = scalar_time / simd_aligned_time;
    double improvement_aligned = ((scalar_time - simd_aligned_time) / scalar_time) * 100.0;
    std::cout << "Scalar vs SIMD (Aligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_aligned << "x" << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_aligned << "%" << std::endl;
    std::cout << std::endl;

    // Unaligned vs Aligned SIMD
    double speedup_align_vs_unalign     = simd_time / simd_aligned_time;
    double improvement_align_vs_unalign = ((simd_time - simd_aligned_time) / simd_time) * 100.0;
    std::cout << "SIMD (Unaligned) vs SIMD (Aligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_align_vs_unalign << "x" << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_align_vs_unalign << "%"
              << std::endl;
    std::cout << std::endl;

    // Optimized SIMD vs Scalar
    double speedup_opt     = scalar_time / simd_opt_time;
    double improvement_opt = ((scalar_time - simd_opt_time) / scalar_time) * 100.0;
    std::cout << "Scalar vs SIMD Optimized (Unaligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_opt << "x" << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_opt << "%" << std::endl;
    std::cout << std::endl;

    // Optimized Aligned SIMD vs Scalar
    double speedup_opt_aligned     = scalar_time / simd_opt_aligned_time;
    double improvement_opt_aligned = ((scalar_time - simd_opt_aligned_time) / scalar_time) * 100.0;
    std::cout << "Scalar vs SIMD Optimized (Aligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_opt_aligned << "x" << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_opt_aligned << "%" << std::endl;
    std::cout << std::endl;

    // Optimized Unaligned vs Aligned
    double speedup_opt_align_vs_unalign     = simd_opt_time / simd_opt_aligned_time;
    double improvement_opt_align_vs_unalign = ((simd_opt_time - simd_opt_aligned_time) / simd_opt_time) * 100.0;
    std::cout << "SIMD Optimized (Unaligned) vs SIMD Optimized (Aligned):" << std::endl;
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) << speedup_opt_align_vs_unalign << "x"
              << std::endl;
    std::cout << "  Improvement: " << std::fixed << std::setprecision(1) << improvement_opt_align_vs_unalign << "%"
              << std::endl;

    return 0;
}