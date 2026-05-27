#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
#include <cmath>
#include <immintrin.h> 

constexpr int RGB_STEP = 3;

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

struct ImageHandler {
    int getBPP() const { return RGB_STEP; }
};

struct AnalysisData {
    std::vector<float> x_coords, y_coords, signal;
    void resize(int n) { x_coords.resize(n); y_coords.resize(n); signal.resize(n); }
};

// Original implementation (matching your exact code)
void compute_signal_original(const unsigned char* image_data, int width, int height,
                           AnalysisData& analysis_data, int solution_idx, int id_samples,
                           const ImageHandler& image_handler) {
    float gx, gy;
    int px, py, pos;
    const unsigned char* ptr = image_data;
    int step = image_handler.getBPP();
    
    for (int a = 0; a < id_samples; a++) {
        px = static_cast<int>(analysis_data.x_coords[a]);
        py = static_cast<int>(analysis_data.y_coords[a]);
        gx = analysis_data.x_coords[a] - px;
        gy = analysis_data.y_coords[a] - py;
        pos = (px + py * width);

        // Detection from the image - exactly as in your code
        analysis_data.signal[a] =
            ptr[(pos + 0) * step + 0] * (1 - gx) * (1 - gy) + ptr[(pos + 1) * step + 0] * gx * (1 - gy) +
            ptr[(pos + width) * step + 0] * (1 - gx) * gy + ptr[step * (pos + width + 1) + 0] * gx * gy;
        analysis_data.signal[a] +=
            ptr[(pos + 0) * step + 1] * (1 - gx) * (1 - gy) + ptr[(pos + 1) * step + 1] * gx * (1 - gy) +
            ptr[(pos + width) * step + 1] * (1 - gx) * gy + ptr[step * (pos + width + 1) + 1] * gx * gy;
        analysis_data.signal[a] +=
            ptr[(pos + 0) * step + 2] * (1 - gx) * (1 - gy) + ptr[(pos + 1) * step + 2] * gx * (1 - gy) +
            ptr[(pos + width) * step + 2] * (1 - gx) * gy + ptr[step * (pos + width + 1) + 2] * gx * gy;
    }
}

// Level 1: Basic optimizations (hoisted calculations, grouped channel reads)
void compute_signal_optimized_basic(const unsigned char* image_data, int width, int height,
                                   AnalysisData& analysis_data, int solution_idx, int id_samples,
                                   const ImageHandler& image_handler) {
    const unsigned char* ptr = image_data;
    const int step = RGB_STEP;  // Compile-time constant
    
    for (int a = 0; a < id_samples; ++a) {
        const float xf = analysis_data.x_coords[a];
        const float yf = analysis_data.y_coords[a];
        const int px = static_cast<int>(xf);
        const int py = static_cast<int>(yf);
        const float gx = xf - px;
        const float gy = yf - py;
        
        // Precompute weights once
        const float w00 = (1.0f - gx) * (1.0f - gy);
        const float w10 = gx * (1.0f - gy);
        const float w01 = (1.0f - gx) * gy;
        const float w11 = gx * gy;
        
        // Hoisted base pointers
        const int base_idx = (px + py * width) * step;
        const unsigned char* p00 = ptr + base_idx;
        const unsigned char* p10 = p00 + step;
        const unsigned char* p01 = p00 + width * step;
        const unsigned char* p11 = p01 + step;
        
        // Sum all RGB channels per corner, then weighted sum
        const float S00 = float(p00[0]) + float(p00[1]) + float(p00[2]);
        const float S10 = float(p10[0]) + float(p10[1]) + float(p10[2]);
        const float S01 = float(p01[0]) + float(p01[1]) + float(p01[2]);
        const float S11 = float(p11[0]) + float(p11[1]) + float(p11[2]);
        
        analysis_data.signal[a] = S00 * w00 + S10 * w10 + S01 * w01 + S11 * w11;
    }
}

// Level 2: FMA + prefetch optimization
void compute_signal_optimized_fma_prefetch(const unsigned char* image_data, int width, int height,
                                         AnalysisData& analysis_data, int solution_idx, int id_samples,
                                         const ImageHandler& image_handler) {
    const unsigned char* ptr = image_data;
    const int step = RGB_STEP;
    
    for (int a = 0; a < id_samples; ++a) {
        const float xf = analysis_data.x_coords[a];
        const float yf = analysis_data.y_coords[a];
        const int px = static_cast<int>(xf);
        const int py = static_cast<int>(yf);
        const float gx = xf - px;
        const float gy = yf - py;
        
        // Hoisted row base pointers
        const unsigned char* row0 = ptr + py * width * step;
        const unsigned char* row1 = row0 + width * step;
        const int px_step = px * step;
        
        // Aggressive prefetch: next 2 samples
        if (a + 8 < id_samples) {
            const int px_next = static_cast<int>(analysis_data.x_coords[a + 8]);
            const int py_next = static_cast<int>(analysis_data.y_coords[a + 8]);
            const unsigned char* row0_next = ptr + py_next * width * step;
            const unsigned char* row1_next = row0_next + width * step;
            const int px_step_next = px_next * step;
            
            __builtin_prefetch(row0_next + px_step_next, 0, 3);
            __builtin_prefetch(row0_next + px_step_next + step, 0, 3);
            __builtin_prefetch(row1_next + px_step_next, 0, 3);
            __builtin_prefetch(row1_next + px_step_next + step, 0, 3);
        }
        
        // Read RGB sums for each corner
        const float S00 = float(row0[px_step + 0]) + float(row0[px_step + 1]) + float(row0[px_step + 2]);
        const float S10 = float(row0[px_step + step + 0]) + float(row0[px_step + step + 1]) + float(row0[px_step + step + 2]);
        const float S01 = float(row1[px_step + 0]) + float(row1[px_step + 1]) + float(row1[px_step + 2]);
        const float S11 = float(row1[px_step + step + 0]) + float(row1[px_step + step + 1]) + float(row1[px_step + step + 2]);
        
        // FMA-based bilinear interpolation (fewer operations)
        const float row0_interp = std::fmaf(S10 - S00, gx, S00);
        const float row1_interp = std::fmaf(S11 - S01, gx, S01);
        analysis_data.signal[a] = std::fmaf(row1_interp - row0_interp, gy, row0_interp);
    }
}

// Level 3: Maximum optimization with loop unrolling and register reuse
void compute_signal_optimized_ultra(const unsigned char* image_data, int width, int height,
                                   AnalysisData& analysis_data, int solution_idx, int id_samples,
                                   const ImageHandler& image_handler) {
    const unsigned char* ptr = image_data;
    const int step = RGB_STEP;
    const int width_step = width * step;
    
    int a = 0;
    
    // Unrolled loop: process 4 samples at once for better ILP
    for (; a + 4 <= id_samples; a += 4) {
        // Prefetch for next batch
        if (a + 16 < id_samples) {
            const int px_next = static_cast<int>(analysis_data.x_coords[a + 16]);
            const int py_next = static_cast<int>(analysis_data.y_coords[a + 16]);
            const unsigned char* prefetch_addr = ptr + py_next * width_step + px_next * step;
            __builtin_prefetch(prefetch_addr, 0, 3);
            __builtin_prefetch(prefetch_addr + step, 0, 3);
            __builtin_prefetch(prefetch_addr + width_step, 0, 3);
            __builtin_prefetch(prefetch_addr + width_step + step, 0, 3);
        }
        
        // Process 4 samples in parallel (better instruction scheduling)
        for (int unroll = 0; unroll < 4; ++unroll) {
            const int idx = a + unroll;
            const float xf = analysis_data.x_coords[idx];
            const float yf = analysis_data.y_coords[idx];
            const int px = static_cast<int>(xf);
            const int py = static_cast<int>(yf);
            const float gx = xf - px;
            const float gy = yf - py;
            
            const unsigned char* base = ptr + py * width_step + px * step;
            
            // Load and sum RGB values efficiently
            const float S00 = float(base[0]) + float(base[1]) + float(base[2]);
            const float S10 = float(base[step]) + float(base[step + 1]) + float(base[step + 2]);
            const float S01 = float(base[width_step]) + float(base[width_step + 1]) + float(base[width_step + 2]);
            const float S11 = float(base[width_step + step]) + float(base[width_step + step + 1]) + float(base[width_step + step + 2]);
            
            // Optimized FMA interpolation
            const float row0_interp = std::fmaf(S10 - S00, gx, S00);
            const float row1_interp = std::fmaf(S11 - S01, gx, S01);
            analysis_data.signal[idx] = std::fmaf(row1_interp - row0_interp, gy, row0_interp);
        }
    }
    
    // Handle remaining samples
    for (; a < id_samples; ++a) {
        const float xf = analysis_data.x_coords[a];
        const float yf = analysis_data.y_coords[a];
        const int px = static_cast<int>(xf);
        const int py = static_cast<int>(yf);
        const float gx = xf - px;
        const float gy = yf - py;
        
        const unsigned char* base = ptr + py * width_step + px * step;
        
        const float S00 = float(base[0]) + float(base[1]) + float(base[2]);
        const float S10 = float(base[step]) + float(base[step + 1]) + float(base[step + 2]);
        const float S01 = float(base[width_step]) + float(base[width_step + 1]) + float(base[width_step + 2]);
        const float S11 = float(base[width_step + step]) + float(base[width_step + step + 1]) + float(base[width_step + step + 2]);
        
        const float row0_interp = std::fmaf(S10 - S00, gx, S00);
        const float row1_interp = std::fmaf(S11 - S01, gx, S01);
        analysis_data.signal[a] = std::fmaf(row1_interp - row0_interp, gy, row0_interp);
    }
}

// Benchmark harness
struct BenchResult {
    std::string name;
    double ms;
    double cpu_percent;
    float avg_signal;
};

template<typename F>
BenchResult benchmark_function(const std::string& name, F&& func, int iterations = 1000) {
    CPUUsage start_cpu = get_cpu_usage();
    auto start_time = std::chrono::high_resolution_clock::now();
    
    float avg_signal = 0.0f;
    for (int i = 0; i < iterations; ++i) {
        func();
        // Compute average on last iteration
        if (i == iterations - 1) {
            float sum = 0.0f;
            // Access the signal from the analysis data used in func
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    CPUUsage end_cpu = get_cpu_usage();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();
    
    double time_ms = duration / 1000.0;
    double cpu_usage = (end_cpu.total_time - start_cpu.total_time) * 100.0 / 
                       (duration / 1e6);
    
    return {name, time_ms, cpu_usage, avg_signal};
}

int main() {
    // Test configuration
    const int width = 1920, height = 1080;
    const int id_samples = 4096;
    const int solution_idx = 0;
    const int iterations = 1000;
    
    // Create synthetic image data
    std::vector<unsigned char> image(width * height * RGB_STEP);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> pixel_dist(0, 255);
    for (auto& p : image) p = static_cast<unsigned char>(pixel_dist(rng));
    
    // Create realistic sampling coordinates
    AnalysisData analysis_data;
    analysis_data.resize(id_samples);
    
    std::uniform_real_distribution<float> coord_dist_x(0.0f, width - 1.001f);
    std::uniform_real_distribution<float> coord_dist_y(0.0f, height - 1.001f);
    
    for (int i = 0; i < id_samples; ++i) {
        analysis_data.x_coords[i] = coord_dist_x(rng);
        analysis_data.y_coords[i] = coord_dist_y(rng);
    }
    
    ImageHandler image_handler;
    
    // Create copies for each test
    AnalysisData data_original = analysis_data;
    AnalysisData data_basic = analysis_data;
    AnalysisData data_fma = analysis_data;
    AnalysisData data_ultra = analysis_data;
    
    // Benchmark all variants
    std::vector<BenchResult> results;
    
    results.push_back(benchmark_function("Original", [&]() {
        compute_signal_original(image.data(), width, height, data_original, solution_idx, id_samples, image_handler);
    }, iterations));
    
    results.push_back(benchmark_function("Basic Optimized", [&]() {
        compute_signal_optimized_basic(image.data(), width, height, data_basic, solution_idx, id_samples, image_handler);
    }, iterations));
    
    results.push_back(benchmark_function("FMA + Prefetch", [&]() {
        compute_signal_optimized_fma_prefetch(image.data(), width, height, data_fma, solution_idx, id_samples, image_handler);
    }, iterations));
    
    results.push_back(benchmark_function("Ultra Optimized", [&]() {
        compute_signal_optimized_ultra(image.data(), width, height, data_ultra, solution_idx, id_samples, image_handler);
    }, iterations));
    
    // Verify correctness
    std::cout << "\n=== Correctness Verification ===" << std::endl;
    auto check_correctness = [&](const AnalysisData& test_data, const char* name) {
        double max_diff = 0.0, rms = 0.0;
        for (int i = 0; i < id_samples; ++i) {
            double diff = std::abs(test_data.signal[i] - data_original.signal[i]);
            max_diff = std::max(max_diff, diff);
            rms += diff * diff;
        }
        rms = std::sqrt(rms / id_samples);
        std::cout << std::left << std::setw(20) << name 
                  << " max_diff=" << std::scientific << std::setprecision(3) << max_diff
                  << " rms=" << rms << std::endl;
    };
    
    check_correctness(data_basic, "Basic Optimized");
    check_correctness(data_fma, "FMA + Prefetch");
    check_correctness(data_ultra, "Ultra Optimized");
    
    // Report performance results
    std::cout << "\n=== Performance Results ===" << std::endl;
    std::cout << std::left << std::setw(20) << "Method"
              << std::setw(12) << "Time (ms)"
              << std::setw(12) << "CPU Usage"
              << std::setw(10) << "Speedup"
              << std::setw(15) << "Improvement" << std::endl;
    std::cout << std::string(70, '-') << std::endl;
    
    double baseline_time = results[0].ms;
    for (const auto& result : results) {
        double speedup = baseline_time / result.ms;
        double improvement = ((baseline_time - result.ms) / baseline_time) * 100.0;
        
        std::cout << std::left << std::setw(20) << result.name
                  << std::setw(12) << std::fixed << std::setprecision(3) << result.ms
                  << std::setw(12) << std::setprecision(1) << result.cpu_percent << "%"
                  << std::setw(10) << std::setprecision(2) << speedup << "x"
                  << std::setw(15) << std::setprecision(1) << improvement << "%" << std::endl;
    }
    
    // Compute and display average signal
    float avg = 0.0f;
    for (const auto& val : data_ultra.signal) avg += val;
    avg /= id_samples;
    std::cout << "\nAverage signal: " << std::fixed << std::setprecision(3) << avg << std::endl;
    
    std::cout << "\n=== Optimization Summary ===" << std::endl;
    std::cout << "✓ Eliminated redundant (1-gx)*(1-gy) weight calculations" << std::endl;
    std::cout << "✓ Hoisted pointer arithmetic outside inner computations" << std::endl;
    std::cout << "✓ Grouped RGB channel reads per corner pixel" << std::endl;
    std::cout << "✓ Used FMA instructions for reduced latency" << std::endl;
    std::cout << "✓ Aggressive prefetching 8-16 samples ahead" << std::endl;
    std::cout << "✓ Loop unrolling for better instruction-level parallelism" << std::endl;
    std::cout << "✓ Compile-time constants and reduced branching" << std::endl;
    
    std::cout << "\n=== Perf Analysis Commands ===" << std::endl;
    std::cout << "perf stat -e cycles,instructions,cache-references,cache-misses,branches,branch-misses ./bilinear_scalar_bench" << std::endl;
    std::cout << "perf stat -e L1-dcache-loads,L1-dcache-load-misses,LLC-loads,LLC-load-misses ./bilinear_scalar_bench" << std::endl;
    std::cout << "perf record -g --call-graph=dwarf ./bilinear_scalar_bench && perf report" << std::endl;
    
    return 0;
}
