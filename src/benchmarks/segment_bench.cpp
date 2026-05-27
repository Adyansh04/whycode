#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <random>
#include <sys/resource.h>

// xsimd
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

template<typename T>
using aligned_vec = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

// LUT structure for fast trig calculations
struct TrigLUTAligned {
    aligned_vec<float> cosv;
    aligned_vec<float> sinv;
    int n = 0;
};

void ensure_trig_lut(TrigLUTAligned& lut, int n) {
    if (lut.n == n && !lut.cosv.empty())
        return;
    lut.n = n;
    lut.cosv.resize(n);
    lut.sinv.resize(n);
    const float k = 2.0f * float(M_PI) / n;
#pragma GCC unroll 4
    for (int a = 0; a < n; ++a) {
        float ang = a * k;
        lut.cosv[a] = cosf(ang);
        lut.sinv[a] = sinf(ang);
    }
}

// Analysis data structure
struct AnalysisData {
    std::vector<int> smooth;
    int num_points = 0;
    
    void resize(int n) {
        smooth.resize(n);
        num_points = 0;
    }
    
    void reset() {
        num_points = 0;
    }
};

// CPU usage measurement
struct CPUUsage {
    double user_time, system_time, total_time;
};

static CPUUsage get_cpu_usage() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    CPUUsage cpu;
    cpu.user_time = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time = cpu.user_time + cpu.system_time;
    return cpu;
}

// Original scalar implementation
void scalar_smooth_accumulation(const AnalysisData& analysis_data, int solution_idx, 
                               int id_samples, int segmentWidth,
                               float& sx, float& sy, int& num_points) {
    sx = 0.0f;
    sy = 0.0f;
    num_points = 0;
    
    const float two_pi_over_segment = 2.0f * M_PI / segmentWidth;
    
    for (int a = 1; a < id_samples; a++) {
        if (analysis_data.smooth[a] != analysis_data.smooth[a - 1]) {
            sx += cos(two_pi_over_segment * a);
            sy += sin(two_pi_over_segment * a);
            num_points++;
        }
    }
}

// Scalar with LUT optimization
void scalar_lut_smooth_accumulation(const AnalysisData& analysis_data, int solution_idx,
                                   int id_samples, int segmentWidth,
                                   const TrigLUTAligned& lut,
                                   float& sx, float& sy, int& num_points) {
    sx = 0.0f;
    sy = 0.0f;
    num_points = 0;
    
    for (int a = 1; a < id_samples; a++) {
        if (analysis_data.smooth[a] != analysis_data.smooth[a - 1]) {
            sx += lut.cosv[a];
            sy += lut.sinv[a];
            num_points++;
        }
    }
}

// SIMD optimized with LUT and vectorized accumulation
void simd_lut_smooth_accumulation(const AnalysisData& analysis_data, int solution_idx,
                                 int id_samples, int segmentWidth,
                                 const TrigLUTAligned& lut,
                                 float& sx, float& sy, int& num_points) {
    using bf = xs::batch<float>;
    using bi = xs::batch<int>;
    constexpr int VS = bf::size;
    
    sx = 0.0f;
    sy = 0.0f;
    num_points = 0;
    
    bf sum_x_vec(0.0f);
    bf sum_y_vec(0.0f);
    bi count_vec(0);
    
    int a = 1;
    
    // Process SIMD-sized chunks
    for (; a + VS <= id_samples; a += VS) {
        // Load current and previous smooth values
        alignas(64) int curr_vals[VS], prev_vals[VS];
        alignas(64) float cos_vals[VS], sin_vals[VS];
        alignas(64) float mask_vals[VS];
        
        for (int i = 0; i < VS; ++i) {
            int idx = a + i;
            curr_vals[i] = analysis_data.smooth[idx];
            prev_vals[i] = analysis_data.smooth[idx - 1];
            
            // Load LUT values for this index
            cos_vals[i] = lut.cosv[idx];
            sin_vals[i] = lut.sinv[idx];
            
            // Create mask: 1.0f if condition is true, 0.0f otherwise
            mask_vals[i] = (curr_vals[i] != prev_vals[i]) ? 1.0f : 0.0f;
        }
        
        // Load into SIMD vectors
        bf cos_vec = xs::load_aligned(cos_vals);
        bf sin_vec = xs::load_aligned(sin_vals);
        bf mask_vec = xs::load_aligned(mask_vals);
        
        // Conditional accumulation using mask
        sum_x_vec += cos_vec * mask_vec;
        sum_y_vec += sin_vec * mask_vec;
        
        // Count true conditions
        for (int i = 0; i < VS; ++i) {
            if (mask_vals[i] > 0.5f) {
                num_points++;
            }
        }
    }
    
    // Reduce SIMD accumulators
    sx = xs::reduce_add(sum_x_vec);
    sy = xs::reduce_add(sum_y_vec);
    
    // Handle remaining elements
    for (; a < id_samples; a++) {
        if (analysis_data.smooth[a] != analysis_data.smooth[a - 1]) {
            sx += lut.cosv[a];
            sy += lut.sinv[a];
            num_points++;
        }
    }
}

// Ultra-optimized SIMD with unrolled loop and prefetching
void simd_ultra_lut_smooth_accumulation(const AnalysisData& analysis_data, int solution_idx,
                                       int id_samples, int segmentWidth,
                                       const TrigLUTAligned& lut,
                                       float& sx, float& sy, int& num_points) {
    using bf = xs::batch<float>;
    constexpr int VS = bf::size;
    constexpr int UNROLL = 4;  // Process 4 SIMD vectors per iteration
    
    sx = 0.0f;
    sy = 0.0f;
    num_points = 0;
    
    bf sum_x_vec1(0.0f), sum_x_vec2(0.0f), sum_x_vec3(0.0f), sum_x_vec4(0.0f);
    bf sum_y_vec1(0.0f), sum_y_vec2(0.0f), sum_y_vec3(0.0f), sum_y_vec4(0.0f);
    
    int a = 1;
    const int unroll_end = id_samples - (UNROLL * VS);
    
    // Unrolled SIMD loop for better ILP
    for (; a <= unroll_end; a += UNROLL * VS) {
        // Prefetch data for next iteration
        if (a + UNROLL * VS * 2 < id_samples) {
            __builtin_prefetch(&analysis_data.smooth[a + UNROLL * VS * 2], 0, 3);
            __builtin_prefetch(&lut.cosv[a + UNROLL * VS * 2], 0, 3);
            __builtin_prefetch(&lut.sinv[a + UNROLL * VS * 2], 0, 3);
        }
        
        // Process 4 SIMD vectors in parallel
        for (int unroll = 0; unroll < UNROLL; ++unroll) {
            int base_idx = a + unroll * VS;
            
            alignas(64) float cos_vals[VS], sin_vals[VS], mask_vals[VS];
            
            for (int i = 0; i < VS; ++i) {
                int idx = base_idx + i;
                bool condition = (analysis_data.smooth[idx] != analysis_data.smooth[idx - 1]);
                
                cos_vals[i] = lut.cosv[idx];
                sin_vals[i] = lut.sinv[idx];
                mask_vals[i] = condition ? 1.0f : 0.0f;
                
                if (condition) num_points++;
            }
            
            bf cos_vec = xs::load_aligned(cos_vals);
            bf sin_vec = xs::load_aligned(sin_vals);
            bf mask_vec = xs::load_aligned(mask_vals);
            
            // Accumulate to different vectors to reduce dependency chains
            switch (unroll) {
                case 0:
                    sum_x_vec1 += cos_vec * mask_vec;
                    sum_y_vec1 += sin_vec * mask_vec;
                    break;
                case 1:
                    sum_x_vec2 += cos_vec * mask_vec;
                    sum_y_vec2 += sin_vec * mask_vec;
                    break;
                case 2:
                    sum_x_vec3 += cos_vec * mask_vec;
                    sum_y_vec3 += sin_vec * mask_vec;
                    break;
                case 3:
                    sum_x_vec4 += cos_vec * mask_vec;
                    sum_y_vec4 += sin_vec * mask_vec;
                    break;
            }
        }
    }
    
    // Combine all accumulators
    bf sum_x_total = sum_x_vec1 + sum_x_vec2 + sum_x_vec3 + sum_x_vec4;
    bf sum_y_total = sum_y_vec1 + sum_y_vec2 + sum_y_vec3 + sum_y_vec4;
    
    sx = xs::reduce_add(sum_x_total);
    sy = xs::reduce_add(sum_y_total);
    
    // Handle remaining full SIMD vectors
    for (; a + VS <= id_samples; a += VS) {
        alignas(64) float cos_vals[VS], sin_vals[VS], mask_vals[VS];
        
        for (int i = 0; i < VS; ++i) {
            int idx = a + i;
            bool condition = (analysis_data.smooth[idx] != analysis_data.smooth[idx - 1]);
            
            cos_vals[i] = lut.cosv[idx];
            sin_vals[i] = lut.sinv[idx];
            mask_vals[i] = condition ? 1.0f : 0.0f;
            
            if (condition) num_points++;
        }
        
        bf cos_vec = xs::load_aligned(cos_vals);
        bf sin_vec = xs::load_aligned(sin_vals);
        bf mask_vec = xs::load_aligned(mask_vals);
        
        sx += xs::reduce_add(cos_vec * mask_vec);
        sy += xs::reduce_add(sin_vec * mask_vec);
    }
    
    // Handle remaining elements
    for (; a < id_samples; a++) {
        if (analysis_data.smooth[a] != analysis_data.smooth[a - 1]) {
            sx += lut.cosv[a];
            sy += lut.sinv[a];
            num_points++;
        }
    }
}

// Benchmark harness
struct BenchResult {
    std::string name;
    double time_ms;
    double cpu_percent;
    float sx, sy;
    int num_points;
};

template<typename F>
BenchResult benchmark_function(const std::string& name, F&& func, int iterations = 1000) {
    CPUUsage start_cpu = get_cpu_usage();
    auto start_time = std::chrono::high_resolution_clock::now();
    
    float sx = 0.0f, sy = 0.0f;
    int num_points = 0;
    
    for (int i = 0; i < iterations; ++i) {
        func(sx, sy, num_points);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    CPUUsage end_cpu = get_cpu_usage();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();
    
    double time_ms = duration / 1000.0;
    double cpu_usage = (end_cpu.total_time - start_cpu.total_time) * 100.0 / 
                       (duration / 1e6);
    
    return {name, time_ms, cpu_usage, sx, sy, num_points};
}

int main() {
    // Test configuration
    const int id_samples = 720;        // As specified
    const int segmentWidth = id_samples; // As specified
    const int solution_idx = 0;
    const int iterations = 10000;      // More iterations for precise timing
    
    std::cout << "=== SIMD Smooth Accumulation Benchmark ===" << std::endl;
    std::cout << "Samples: " << id_samples << std::endl;
    std::cout << "Segment width: " << segmentWidth << std::endl;
    std::cout << "SIMD width: " << xs::batch<float>::size << " floats" << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << std::endl;
    
    // Generate test data with realistic smooth signal pattern
    AnalysisData analysis_data;
    analysis_data.resize(id_samples);
    
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    
    // Create a smooth signal with occasional transitions
    // This simulates the typical behavior of smooth signal in marker detection
    std::uniform_int_distribution<int> transition_dist(0, 4);
    std::uniform_real_distribution<float> noise_dist(0.0f, 1.0f);
    
    analysis_data.smooth[0] = 0;
    for (int i = 1; i < id_samples; ++i) {
        // 15% chance of transition
        if (noise_dist(rng) < 0.15f) {
            analysis_data.smooth[i] = transition_dist(rng);
        } else {
            analysis_data.smooth[i] = analysis_data.smooth[i - 1];
        }
    }
    
    // Pre-compute trigonometric LUT
    TrigLUTAligned trig_lut;
    ensure_trig_lut(trig_lut, segmentWidth);
    
    std::cout << "Generated test data with " << segmentWidth << " LUT entries" << std::endl;
    
    // Count actual transitions for verification
    int expected_transitions = 0;
    for (int i = 1; i < id_samples; ++i) {
        if (analysis_data.smooth[i] != analysis_data.smooth[i - 1]) {
            expected_transitions++;
        }
    }
    std::cout << "Expected transitions: " << expected_transitions << std::endl;
    std::cout << std::endl;
    
    // Run benchmarks
    std::vector<BenchResult> results;
    
    // Scalar baseline
    results.push_back(benchmark_function("Scalar Original", [&](float& sx, float& sy, int& num_points) {
        scalar_smooth_accumulation(analysis_data, solution_idx, id_samples, segmentWidth, sx, sy, num_points);
    }, iterations));
    
    // Scalar with LUT
    results.push_back(benchmark_function("Scalar LUT", [&](float& sx, float& sy, int& num_points) {
        scalar_lut_smooth_accumulation(analysis_data, solution_idx, id_samples, segmentWidth, trig_lut, sx, sy, num_points);
    }, iterations));
    
    // SIMD with LUT
    results.push_back(benchmark_function("SIMD LUT", [&](float& sx, float& sy, int& num_points) {
        simd_lut_smooth_accumulation(analysis_data, solution_idx, id_samples, segmentWidth, trig_lut, sx, sy, num_points);
    }, iterations));
    
    // Ultra-optimized SIMD
    results.push_back(benchmark_function("SIMD Ultra", [&](float& sx, float& sy, int& num_points) {
        simd_ultra_lut_smooth_accumulation(analysis_data, solution_idx, id_samples, segmentWidth, trig_lut, sx, sy, num_points);
    }, iterations));
    
    // Display results
    std::cout << "=== BENCHMARK RESULTS ===" << std::endl;
    std::cout << std::left << std::setw(18) << "Method"
              << std::setw(12) << "Time (ms)"
              << std::setw(10) << "CPU %"
              << std::setw(10) << "Speedup"
              << std::setw(12) << "sx"
              << std::setw(12) << "sy"
              << std::setw(8) << "Points" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    
    double baseline_time = results[0].time_ms;
    
    for (const auto& result : results) {
        double speedup = baseline_time / result.time_ms;
        
        std::cout << std::left << std::setw(18) << result.name
                  << std::setw(12) << std::fixed << std::setprecision(3) << result.time_ms
                  << std::setw(10) << std::setprecision(1) << result.cpu_percent
                  << std::setw(10) << std::setprecision(2) << speedup << "x"
                  << std::setw(12) << std::setprecision(6) << result.sx
                  << std::setw(12) << std::setprecision(6) << result.sy
                  << std::setw(8) << result.num_points << std::endl;
    }
    
    // Correctness verification
    std::cout << "\n=== CORRECTNESS VERIFICATION ===" << std::endl;
    const float epsilon = 1e-5f;
    bool all_correct = true;
    
    for (size_t i = 1; i < results.size(); ++i) {
        bool sx_ok = std::abs(results[i].sx - results[0].sx) < epsilon;
        bool sy_ok = std::abs(results[i].sy - results[0].sy) < epsilon;
        bool points_ok = results[i].num_points == results[0].num_points;
        bool correct = sx_ok && sy_ok && points_ok;
        
        std::cout << results[i].name << ": ";
        if (correct) {
            std::cout << "✓ PASS" << std::endl;
        } else {
            std::cout << "✗ FAIL - ";
            if (!sx_ok) std::cout << "sx diff=" << (results[i].sx - results[0].sx) << " ";
            if (!sy_ok) std::cout << "sy diff=" << (results[i].sy - results[0].sy) << " ";
            if (!points_ok) std::cout << "points diff=" << (results[i].num_points - results[0].num_points) << " ";
            std::cout << std::endl;
            all_correct = false;
        }
    }
    
    std::cout << "\nOverall: " << (all_correct ? "All tests PASSED" : "Some tests FAILED") << std::endl;
    
    // Performance analysis
    std::cout << "\n=== OPTIMIZATION SUMMARY ===" << std::endl;
    std::cout << "✓ Pre-computed trigonometric LUT (eliminates cos/sin calls)" << std::endl;
    std::cout << "✓ SIMD vectorization of conditional accumulation" << std::endl;
    std::cout << "✓ Loop unrolling with multiple accumulators (reduces dependency chains)" << std::endl;
    std::cout << "✓ Software prefetching for better memory throughput" << std::endl;
    std::cout << "✓ Aligned memory access for optimal SIMD performance" << std::endl;
    
    double best_speedup = baseline_time / std::min({results[2].time_ms, results[3].time_ms});
    std::cout << "\nBest speedup achieved: " << std::fixed << std::setprecision(2) 
              << best_speedup << "x over scalar baseline" << std::endl;
    
    std::cout << "\n=== PERF ANALYSIS COMMANDS ===" << std::endl;
    std::cout << "perf stat -e cycles,instructions,cache-references,cache-misses ./smooth_benchmark" << std::endl;
    std::cout << "perf stat -e fp_arith_inst_retired.128b_packed_single,fp_arith_inst_retired.256b_packed_single ./smooth_benchmark" << std::endl;
    std::cout << "perf record -g --call-graph=dwarf ./smooth_benchmark && perf report" << std::endl;
    
    return 0;
}
