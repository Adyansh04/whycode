#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
#include <cmath>

// xsimd
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

template<typename T>
using aligned_vec = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

struct CPUUsage {
    double user_time, system_time, total_time;
};
static CPUUsage get_cpu_usage() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    CPUUsage cpu;
    cpu.user_time   = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time  = cpu.user_time + cpu.system_time;
    return cpu;
}

// Data container
struct AnalysisData {
    std::vector<float> signal;  // length N
    std::vector<float> smooth;  // length N (0.0f or 1.0f)
    void resize(int n) { signal.resize(n); smooth.resize(n); }
};

struct AnalysisDataAligned {
    aligned_vec<float> signal;
    aligned_vec<float> smooth;
    void resize(int n) { signal.resize(n); smooth.resize(n); }
};

// 1) Scalar baseline
void binarize_scalar(AnalysisData& A) {
    const int N = (int)A.signal.size();
    float sum = 0.0f;
    for (int i = 0; i < N; ++i) sum += A.signal[i];
    const float avg = sum / float(N);
    for (int i = 0; i < N; ++i) {
        A.smooth[i] = (A.signal[i] > avg) ? 1.0f : 0.0f;
    }
}

// 2) Scalar Welford 
void binarize_scalar_welford(AnalysisData& A) {
    const int N = (int)A.signal.size();
    // Welford mean only
    float mean = 0.0f;
    for (int i = 0; i < N; ++i) {
        mean += (A.signal[i] - mean) / float(i + 1);
    }
    const float avg = mean;
    for (int i = 0; i < N; ++i) {
        A.smooth[i] = (A.signal[i] > avg) ? 1.0f : 0.0f;
    }
}

// 3) SIMD unaligned 
void binarize_simd_unaligned(AnalysisData& A) {
    using bf = xs::batch<float>;
    const int VS = bf::size;
    const int N = (int)A.signal.size();

    // Sum
    int i = 0;
    bf acc(0.0f);
    for (; i + VS <= N; i += VS) {
        acc += xs::load_unaligned(&A.signal[i]);
    }
    float sum = xs::reduce_add(acc);
    for (; i < N; ++i) sum += A.signal[i];
    const float avg = sum / float(N);

    // Thresholding
    const bf avgv(avg);
    i = 0;
    for (; i + VS <= N; i += VS) {
        bf v = xs::load_unaligned(&A.signal[i]);
        auto m = v > avgv;
        // Convert mask to 0.0f/1.0f via select
        bf out = xs::select(m, bf(1.0f), bf(0.0f));
        xs::store_unaligned(&A.smooth[i], out);
    }
    for (; i < N; ++i) {
        A.smooth[i] = (A.signal[i] > avg) ? 1.0f : 0.0f;
    }
}

// 4) SIMD unrolled (better ILP)
void binarize_simd_unrolled(AnalysisData& A) {
    using bf = xs::batch<float>;
    const int VS = bf::size;
    const int N = (int)A.signal.size();

    int i = 0;
    bf acc1(0.0f), acc2(0.0f), acc3(0.0f), acc4(0.0f);
    for (; i + 4*VS <= N; i += 4*VS) {
        acc1 += xs::load_unaligned(&A.signal[i + 0*VS]);
        acc2 += xs::load_unaligned(&A.signal[i + 1*VS]);
        acc3 += xs::load_unaligned(&A.signal[i + 2*VS]);
        acc4 += xs::load_unaligned(&A.signal[i + 3*VS]);
    }
    bf acc = acc1 + acc2 + acc3 + acc4;
    for (; i + VS <= N; i += VS) {
        acc += xs::load_unaligned(&A.signal[i]);
    }
    float sum = xs::reduce_add(acc);
    for (; i < N; ++i) sum += A.signal[i];
    const float avg = sum / float(N);

    // Threshold unrolled
    const bf avgv(avg);
    i = 0;
    for (; i + 4*VS <= N; i += 4*VS) {
        bf v0 = xs::load_unaligned(&A.signal[i + 0*VS]);
        bf v1 = xs::load_unaligned(&A.signal[i + 1*VS]);
        bf v2 = xs::load_unaligned(&A.signal[i + 2*VS]);
        bf v3 = xs::load_unaligned(&A.signal[i + 3*VS]);
        xs::store_unaligned(&A.smooth[i + 0*VS], xs::select(v0 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_unaligned(&A.smooth[i + 1*VS], xs::select(v1 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_unaligned(&A.smooth[i + 2*VS], xs::select(v2 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_unaligned(&A.smooth[i + 3*VS], xs::select(v3 > avgv, bf(1.0f), bf(0.0f)));
    }
    for (; i + VS <= N; i += VS) {
        bf v = xs::load_unaligned(&A.signal[i]);
        xs::store_unaligned(&A.smooth[i], xs::select(v > avgv, bf(1.0f), bf(0.0f)));
    }
    for (; i < N; ++i) {
        A.smooth[i] = (A.signal[i] > avg) ? 1.0f : 0.0f;
    }
}

// 5) SIMD aligned (aligned allocator + aligned loads/stores)
void binarize_simd_aligned(AnalysisDataAligned& A) {
    using bf = xs::batch<float>;
    const int VS = bf::size;
    const int N = (int)A.signal.size();

    int i = 0;
    bf acc1(0.0f), acc2(0.0f), acc3(0.0f), acc4(0.0f);
    for (; i + 4*VS <= N; i += 4*VS) {
        acc1 += xs::load_aligned(&A.signal[i + 0*VS]);
        acc2 += xs::load_aligned(&A.signal[i + 1*VS]);
        acc3 += xs::load_aligned(&A.signal[i + 2*VS]);
        acc4 += xs::load_aligned(&A.signal[i + 3*VS]);
    }
    bf acc = acc1 + acc2 + acc3 + acc4;
    for (; i + VS <= N; i += VS) {
        acc += xs::load_aligned(&A.signal[i]);
    }
    float sum = xs::reduce_add(acc);
    for (; i < N; ++i) sum += A.signal[i];
    const float avg = sum / float(N);

    const bf avgv(avg);
    i = 0;
    for (; i + 4*VS <= N; i += 4*VS) {
        bf v0 = xs::load_aligned(&A.signal[i + 0*VS]);
        bf v1 = xs::load_aligned(&A.signal[i + 1*VS]);
        bf v2 = xs::load_aligned(&A.signal[i + 2*VS]);
        bf v3 = xs::load_aligned(&A.signal[i + 3*VS]);
        xs::store_aligned(&A.smooth[i + 0*VS], xs::select(v0 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_aligned(&A.smooth[i + 1*VS], xs::select(v1 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_aligned(&A.smooth[i + 2*VS], xs::select(v2 > avgv, bf(1.0f), bf(0.0f)));
        xs::store_aligned(&A.smooth[i + 3*VS], xs::select(v3 > avgv, bf(1.0f), bf(0.0f)));
    }
    for (; i + VS <= N; i += VS) {
        bf v = xs::load_aligned(&A.signal[i]);
        xs::store_aligned(&A.smooth[i], xs::select(v > avgv, bf(1.0f), bf(0.0f)));
    }
    for (; i < N; ++i) {
        A.smooth[i] = (A.signal[i] > avg) ? 1.0f : 0.0f;
    }
}

// Benchmark harness
struct BenchResult {
    std::string name;
    double ms;
    double cpu_percent;
};

template<typename F>
BenchResult bench(const std::string& name, F&& f, int iters = 20000) {
    CPUUsage s_cpu = get_cpu_usage();
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; ++i) f();
    auto t1 = std::chrono::high_resolution_clock::now();
    CPUUsage e_cpu = get_cpu_usage();

    double dt_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    double cpu = (e_cpu.total_time - s_cpu.total_time) * 100.0 / (dt_ms / 1000.0);
    return {name, dt_ms, cpu};
}

int main() {
    // Config
    const int N = 720; // id_samples_
    const int iters = 2000000;

    std::cout << "=== SIMD Binarization (avg/threshold) Benchmark ===\n";
    std::cout << "N = " << N << ", SIMD width = " << xs::batch<float>::size << ", iterations = " << iters << "\n\n";

    // Generate signals (repeatable)
    std::mt19937 rng(42);
    std::normal_distribution<float> nd(128.0f, 40.0f);

    AnalysisData A_scalar, A_simd_u, A_simd_unroll;
    A_scalar.resize(N);
    A_simd_u.resize(N);
    A_simd_unroll.resize(N);

    for (int i = 0; i < N; ++i) {
        float v = std::clamp(nd(rng), 0.0f, 255.0f);
        A_scalar.signal[i] = v;
        A_simd_u.signal[i] = v;
        A_simd_unroll.signal[i] = v;
    }

    AnalysisDataAligned A_aligned;
    A_aligned.resize(N);
    for (int i = 0; i < N; ++i) A_aligned.signal[i] = A_scalar.signal[i];

    // Warmup + correctness
    binarize_scalar(A_scalar);
    binarize_simd_unaligned(A_simd_u);
    binarize_simd_unrolled(A_simd_unroll);
    binarize_simd_aligned(A_aligned);

    auto check = [&](const char* name, const std::vector<float>& s){
        int diff = 0;
        for (int i = 0; i < N; ++i) diff += (int)(s[i] != A_scalar.smooth[i]);
        std::cout << std::left << std::setw(26) << name << " diffs=" << diff << "\n";
    };
    check("SIMD unaligned vs scalar", A_simd_u.smooth);
    check("SIMD unrolled vs scalar ", A_simd_unroll.smooth);
    check("SIMD aligned vs scalar  ", std::vector<float>(A_aligned.smooth.begin(), A_aligned.smooth.end()));

    // Benchmarks
    auto r_scalar = bench("Scalar", [&](){ binarize_scalar(A_scalar); }, iters);
    auto r_simd_u = bench("SIMD unaligned", [&](){ binarize_simd_unaligned(A_simd_u); }, iters);
    auto r_simd_unroll = bench("SIMD unrolled", [&](){ binarize_simd_unrolled(A_simd_unroll); }, iters);
    auto r_simd_aligned = bench("SIMD aligned", [&](){ binarize_simd_aligned(A_aligned); }, iters);

    // Report
    auto report = [&](const BenchResult& br, double base_ms){
        double speed = base_ms / br.ms;
        std::cout << std::left << std::setw(16) << br.name
                  << " time=" << std::setw(10) << std::fixed << std::setprecision(3) << br.ms << " ms"
                  << " CPU="  << std::setw(6)  << std::setprecision(1) << br.cpu_percent << "%"
                  << " speedup=" << std::setprecision(2) << speed << "x\n";
    };
    std::cout << "\n=== Results ===\n";
    report(r_scalar, r_scalar.ms);
    report(r_simd_u, r_scalar.ms);
    report(r_simd_unroll, r_scalar.ms);
    report(r_simd_aligned, r_scalar.ms);

    std::cout << "\nPerf tips:\n";
    std::cout << "perf stat -e cycles,instructions,cache-references,cache-misses ./simd_binarize_bench\n";
    std::cout << "perf record -g --call-graph=dwarf ./simd_binarize_bench && perf report\n";
    return 0;
}
