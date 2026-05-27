#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// xsimd
#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

// Helper types for aligned storage
template <typename T>
using aligned_vec = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

struct Segment
{
    float x, y;    // center
    float m0, m1;  // axes lengths (radii)
    float v0, v1;  // orientation vector components
};

struct Solution
{
    Segment segment;
};

struct EllipseProcessing
{
    std::vector<Solution> solutions;
};

struct AnalysisData
{
    std::vector<float> x_coords;
    std::vector<float> y_coords;
};

struct AnalysisDataAligned
{
    aligned_vec<float> x_coords;
    aligned_vec<float> y_coords;
};

struct CPUUsage
{
    double user_time;
    double system_time;
    double total_time;
};
static CPUUsage get_cpu_usage()
{
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    CPUUsage cpu;
    cpu.user_time   = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    cpu.system_time = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    cpu.total_time  = cpu.user_time + cpu.system_time;
    return cpu;
}

struct Context
{
    int                       width;
    int                       height;
    int                       id_samples;
    int                       solution_idx;
    EllipseProcessing         ellipse_processing;
    std::vector<AnalysisData> analysis_data;
};

// Scalar baseline
bool compute_positions_scalar(Context& ctx)
{
    const int width        = ctx.width;
    const int height       = ctx.height;
    const int solution_idx = ctx.solution_idx;
    const int id_samples   = ctx.id_samples;

    const float two_pi_over_samples = 2.0f * float(M_PI) / id_samples;

    const Segment& s = ctx.ellipse_processing.solutions[solution_idx].segment;

    float cos_angle;
    float sin_angle;
    auto& A = ctx.analysis_data[solution_idx];
    for (int a = 0; a < id_samples; ++a)
    {
        cos_angle = cosf(a * two_pi_over_samples);
        sin_angle = sinf(a * two_pi_over_samples);

        A.x_coords[a] = s.x + (s.m0 * cos_angle * s.v0 + s.m1 * sin_angle * s.v1) * 2.0f;
        A.y_coords[a] = s.y + (s.m0 * cos_angle * s.v1 - s.m1 * sin_angle * s.v0) * 2.0f;

        if (A.x_coords[a] < 0 || width <= A.x_coords[a] || A.y_coords[a] < 0 ||
            height <= A.y_coords[a])
        {
            return false;
        }
    }
    return true;
}

// SIMD version using xsimd sin/cos and vectorized arithmetic
bool compute_positions_simd_xsimd_trig(Context& ctx)
{
    using bf         = xs::batch<float>;
    const int      W = ctx.width, H = ctx.height;
    const int      N   = ctx.id_samples;
    const int      idx = ctx.solution_idx;
    const Segment& s   = ctx.ellipse_processing.solutions[idx].segment;
    auto&          A   = ctx.analysis_data[idx];

    const float k  = 2.0f * float(M_PI) / N;
    const int   VS = bf::size;

    // Broadcast constants
    const bf bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f), bk(k);

    bool in_bounds = true;

    int a = 0;
    for (; a + VS <= N; a += VS)
    {
        // angles = (a + [0..VS-1]) * k
        float idxs[VS];
        for (int i = 0; i < VS; ++i)
            idxs[i] = float(a + i);
        bf ia  = xs::load_unaligned(idxs);
        bf ang = ia * bk;

        // Vector trig
        bf c  = xs::cos(ang);
        bf s_ = xs::sin(ang);

        // Compute:
        // dx = 2*(m0*c*v0 + m1*s*v1)
        // dy = 2*(m0*c*v1 - m1*s*v0)
        bf dx = two * (bm0 * c * bv0 + bm1 * s_ * bv1);
        bf dy = two * (bm0 * c * bv1 - bm1 * s_ * bv0);

        bf x = bx + dx;
        bf y = by + dy;

        // Store
        xs::store_unaligned(&A.x_coords[a], x);
        xs::store_unaligned(&A.y_coords[a], y);

        // Bounds check vectorized: if any out-of-bounds, mark false
        auto mask_x_lo = x < bf(0.0f);
        auto mask_x_hi = x >= bf(float(W));
        auto mask_y_lo = y < bf(0.0f);
        auto mask_y_hi = y >= bf(float(H));

        auto any_oob = xs::any(mask_x_lo | mask_x_hi | mask_y_lo | mask_y_hi);
        if (any_oob)
        {
            in_bounds = false;
        }
    }
    // Tail
    for (; a < N; ++a)
    {
        float ang     = a * k;
        float c       = cosf(ang);
        float s_      = sinf(ang);
        float x       = s.x + 2.0f * (s.m0 * c * s.v0 + s.m1 * s_ * s.v1);
        float y       = s.y + 2.0f * (s.m0 * c * s.v1 - s.m1 * s_ * s.v0);
        A.x_coords[a] = x;
        A.y_coords[a] = y;
        if (x < 0 || x >= W || y < 0 || y >= H)
            in_bounds = false;
    }
    return in_bounds;
}

// Variant with aligned memory for all arrays, stores, and loads
bool compute_positions_simd_xsimd_trig_aligned(
    int width, int height, int N, const Segment& s, aligned_vec<float>& x_coords,
    aligned_vec<float>& y_coords)
{
    using bf     = xs::batch<float>;
    const int VS = bf::size;

    bf bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f);
    bf bk(2.0f * float(M_PI) / N);

    alignas(xs::default_arch::alignment()) float idxs[VS];

    bool in_bounds = true;
    int  a         = 0;
    for (; a + VS <= N; a += VS)
    {
        for (int i = 0; i < VS; ++i)
            idxs[i] = float(a + i);
        bf ia  = xs::load_aligned(idxs);  // load_aligned
        bf ang = ia * bk;
        bf c   = xs::cos(ang);
        bf s_  = xs::sin(ang);
        bf dx  = two * (bm0 * c * bv0 + bm1 * s_ * bv1);
        bf dy  = two * (bm0 * c * bv1 - bm1 * s_ * bv0);

        bf x = bx + dx, y = by + dy;

        xs::store_aligned(&x_coords[a], x);
        xs::store_aligned(&y_coords[a], y);

        auto mask_x_lo = x < bf(0.0f);
        auto mask_x_hi = x >= bf(float(width));
        auto mask_y_lo = y < bf(0.0f);
        auto mask_y_hi = y >= bf(float(height));
        if (xs::any(mask_x_lo | mask_x_hi | mask_y_lo | mask_y_hi))
            in_bounds = false;
    }
    for (; a < N; ++a)
    {
        float ang = a * (2.0f * float(M_PI) / N);
        float c = cosf(ang), s_ = sinf(ang);
        float x     = s.x + 2.0f * (s.m0 * c * s.v0 + s.m1 * s_ * s.v1);
        float y     = s.y + 2.0f * (s.m0 * c * s.v1 - s.m1 * s_ * s.v0);
        x_coords[a] = x;
        y_coords[a] = y;
        if (x < 0 || x >= width || y < 0 || y >= height)
            in_bounds = false;
    }
    return in_bounds;
}

// SIMD version using precomputed trig LUT + vectorized affine math
struct TrigLUT
{
    std::vector<float> cosv;
    std::vector<float> sinv;
    int                n = 0;
};
static void ensure_trig_lut(TrigLUT& lut, int n)
{
    if (lut.n == n && !lut.cosv.empty())
        return;
    lut.n = n;
    lut.cosv.resize(n);
    lut.sinv.resize(n);
    const float k = 2.0f * float(M_PI) / n;
    for (int a = 0; a < n; ++a)
    {
        float ang   = a * k;
        lut.cosv[a] = cosf(ang);
        lut.sinv[a] = sinf(ang);
    }
}

struct TrigLUTAligned
{
    aligned_vec<float> cosv;
    aligned_vec<float> sinv;
    int                n = 0;
};
static void ensure_trig_lut(TrigLUTAligned& lut, int n)
{
    if (lut.n == n && !lut.cosv.empty())
        return;
    lut.n = n;
    lut.cosv.resize(n);
    lut.sinv.resize(n);
    const float k = 2.0f * float(M_PI) / n;
    for (int a = 0; a < n; ++a)
    {
        float ang   = a * k;
        lut.cosv[a] = cosf(ang);
        lut.sinv[a] = sinf(ang);
    }
}

bool compute_positions_simd_lut_trig(Context& ctx, TrigLUT& lut)
{
    using bf         = xs::batch<float>;
    const int      W = ctx.width, H = ctx.height;
    const int      N   = ctx.id_samples;
    const int      idx = ctx.solution_idx;
    const Segment& s   = ctx.ellipse_processing.solutions[idx].segment;
    auto&          A   = ctx.analysis_data[idx];

    ensure_trig_lut(lut, N);

    const int VS = bf::size;

    const bf bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f);

    bool in_bounds = true;

    int a = 0;
    for (; a + VS <= N; a += VS)
    {
        bf c  = xs::load_unaligned(&lut.cosv[a]);
        bf s_ = xs::load_unaligned(&lut.sinv[a]);

        bf dx = two * (bm0 * c * bv0 + bm1 * s_ * bv1);
        bf dy = two * (bm0 * c * bv1 - bm1 * s_ * bv0);

        bf x = bx + dx;
        bf y = by + dy;

        xs::store_unaligned(&A.x_coords[a], x);
        xs::store_unaligned(&A.y_coords[a], y);

        auto mask_x_lo = x < bf(0.0f);
        auto mask_x_hi = x >= bf(float(W));
        auto mask_y_lo = y < bf(0.0f);
        auto mask_y_hi = y >= bf(float(H));
        if (xs::any(mask_x_lo | mask_x_hi | mask_y_lo | mask_y_hi))
            in_bounds = false;
    }
    for (; a < N; ++a)
    {
        float c       = lut.cosv[a];
        float s_      = lut.sinv[a];
        float x       = s.x + 2.0f * (s.m0 * c * s.v0 + s.m1 * s_ * s.v1);
        float y       = s.y + 2.0f * (s.m0 * c * s.v1 - s.m1 * s_ * s.v0);
        A.x_coords[a] = x;
        A.y_coords[a] = y;
        if (x < 0 || x >= W || y < 0 || y >= H)
            in_bounds = false;
    }
    return in_bounds;
}

bool compute_positions_simd_lut_trig_aligned(
    int width, int height, int N, const Segment& s, aligned_vec<float>& x_coords,
    aligned_vec<float>& y_coords, TrigLUTAligned& lut)
{
    using bf     = xs::batch<float>;
    const int VS = bf::size;

    ensure_trig_lut(lut, N);

    bf   bx(s.x), by(s.y), bv0(s.v0), bv1(s.v1), bm0(s.m0), bm1(s.m1), two(2.0f);
    bool in_bounds = true;
    int  a         = 0;

    for (; a + VS <= N; a += VS)
    {
        bf c  = xs::load_aligned(&lut.cosv[a]);
        bf s_ = xs::load_aligned(&lut.sinv[a]);

        bf dx = two * (bm0 * c * bv0 + bm1 * s_ * bv1);
        bf dy = two * (bm0 * c * bv1 - bm1 * s_ * bv0);

        bf x = bx + dx;
        bf y = by + dy;

        xs::store_aligned(&x_coords[a], x);
        xs::store_aligned(&y_coords[a], y);

        auto mask_x_lo = x < bf(0.0f);
        auto mask_x_hi = x >= bf(float(width));
        auto mask_y_lo = y < bf(0.0f);
        auto mask_y_hi = y >= bf(float(height));
        if (xs::any(mask_x_lo | mask_x_hi | mask_y_lo | mask_y_hi))
            in_bounds = false;
    }
    for (; a < N; ++a)
    {
        float c = lut.cosv[a], s_ = lut.sinv[a];
        float x     = s.x + 2.0f * (s.m0 * c * s.v0 + s.m1 * s_ * s.v1);
        float y     = s.y + 2.0f * (s.m0 * c * s.v1 - s.m1 * s_ * s.v0);
        x_coords[a] = x;
        y_coords[a] = y;
        if (x < 0 || x >= width || y < 0 || y >= height)
            in_bounds = false;
    }
    return in_bounds;
}

// Benchmark harness
struct BenchResult
{
    std::string name;
    double      ms;
    double      cpu_percent;
    bool        in_bounds;
};

template <typename F>
BenchResult bench_once(const std::string& name, F&& f, int iters = 200)
{
    CPUUsage s_cpu = get_cpu_usage();
    auto     t0    = std::chrono::high_resolution_clock::now();
    bool     ok    = true;
    for (int i = 0; i < iters; ++i)
    {
        ok = f();
    }
    auto     t1    = std::chrono::high_resolution_clock::now();
    CPUUsage e_cpu = get_cpu_usage();
    double   dt  = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    double   cpu = (e_cpu.total_time - s_cpu.total_time) * 100.0 / (dt / 1000.0);
    return { name, dt, cpu, ok };
}

int main()
{
    // Test configuration
    const int width = 1920, height = 1080;
    const int id_samples   = 720;  // per your note: constant 720
    const int solution_idx = 0;

    // Create a plausible ellipse configuration
    EllipseProcessing ep;
    ep.solutions.resize(1);
    ep.solutions[0].segment.x  = width * 0.5f;
    ep.solutions[0].segment.y  = height * 0.5f;
    ep.solutions[0].segment.m0 = 160.0f;  // semi-major
    ep.solutions[0].segment.m1 = 100.0f;  // semi-minor
    // Orientation vector (unit direction for major axis)
    float angle                = 0.7f;
    ep.solutions[0].segment.v0 = cosf(angle);
    ep.solutions[0].segment.v1 = sinf(angle);

    // Output buffers for ctx (unaligned variants)
    std::vector<AnalysisData> analysis(1);
    analysis[0].x_coords.resize(id_samples);
    analysis[0].y_coords.resize(id_samples);

    Context ctx{ width, height, id_samples, solution_idx, ep, analysis };

    // Make independent output buffers per variant (unaligned)
    std::vector<float> x_scalar(id_samples), y_scalar(id_samples);
    std::vector<float> x_simd_trig(id_samples), y_simd_trig(id_samples);
    std::vector<float> x_simd_lut(id_samples), y_simd_lut(id_samples);

    // Aligned buffers for aligned variants
    aligned_vec<float> x_simd_trig_al(id_samples), y_simd_trig_al(id_samples);
    aligned_vec<float> x_simd_lut_al(id_samples), y_simd_lut_al(id_samples);

    // LUTs
    TrigLUT        lut;     // unaligned LUT for unaligned function
    TrigLUTAligned lut_al;  // aligned LUT for aligned function

    // Helper to swap ctx outputs (for unaligned ctx.analysis_data)
    auto set_outputs = [&](std::vector<float>& x, std::vector<float>& y) {
        ctx.analysis_data[solution_idx].x_coords.assign(x.size(), 0.0f);
        ctx.analysis_data[solution_idx].y_coords.assign(y.size(), 0.0f);
        ctx.analysis_data[solution_idx].x_coords.swap(x);
        ctx.analysis_data[solution_idx].y_coords.swap(y);
    };
    auto get_outputs = [&](std::vector<float>& x, std::vector<float>& y) {
        x.swap(ctx.analysis_data[solution_idx].x_coords);
        y.swap(ctx.analysis_data[solution_idx].y_coords);
    };

    // Warm-up + baseline prepare (scalar)
    set_outputs(x_scalar, y_scalar);
    bool in_bounds_scalar = compute_positions_scalar(ctx);
    get_outputs(x_scalar, y_scalar);

    // SIMD with xsimd trig (unaligned)
    set_outputs(x_simd_trig, y_simd_trig);
    bool in_bounds_simd_trig = compute_positions_simd_xsimd_trig(ctx);
    get_outputs(x_simd_trig, y_simd_trig);

    // SIMD with LUT trig (unaligned)
    set_outputs(x_simd_lut, y_simd_lut);
    bool in_bounds_simd_lut = compute_positions_simd_lut_trig(ctx, lut);
    get_outputs(x_simd_lut, y_simd_lut);

    // Aligned variants compute into their own aligned buffers (no ctx swap)
    const Segment& seg                    = ctx.ellipse_processing.solutions[solution_idx].segment;
    bool           in_bounds_simd_trig_al = compute_positions_simd_xsimd_trig_aligned(
        width,
        height,
        id_samples,
        seg,
        x_simd_trig_al,
        y_simd_trig_al);

    bool in_bounds_simd_lut_al = compute_positions_simd_lut_trig_aligned(
        width,
        height,
        id_samples,
        seg,
        x_simd_lut_al,
        y_simd_lut_al,
        lut_al);

    // Correctness check vs scalar
    auto diff_stats = [&](auto&                     xs,
                          auto&                     ys,
                          const std::vector<float>& x0,
                          const std::vector<float>& y0,
                          const char*               name) {
        double max_dx = 0, max_dy = 0, rms = 0;
        for (int i = 0; i < id_samples; ++i)
        {
            double dx = xs[i] - x0[i];
            double dy = ys[i] - y0[i];
            max_dx    = std::max(max_dx, std::abs(dx));
            max_dy    = std::max(max_dy, std::abs(dy));
            rms += dx * dx + dy * dy;
        }
        rms = std::sqrt(rms / (2.0 * id_samples));
        std::cout << std::left << std::setw(26) << name << " max|dx|=" << max_dx
                  << " max|dy|=" << max_dy << " RMS=" << rms << "\n";
    };

    std::cout << "=== Correctness (vs scalar) ===\n";
    diff_stats(x_simd_trig, y_simd_trig, x_scalar, y_scalar, "SIMD xsimd trig (unaligned)");
    diff_stats(x_simd_lut, y_simd_lut, x_scalar, y_scalar, "SIMD LUT trig (unaligned)");
    diff_stats(x_simd_trig_al, y_simd_trig_al, x_scalar, y_scalar, "SIMD xsimd trig (aligned)");
    diff_stats(x_simd_lut_al, y_simd_lut_al, x_scalar, y_scalar, "SIMD LUT trig (aligned)");

    std::cout << "Bounds: scalar=" << (in_bounds_scalar ? "true" : "false")
              << " simd_trig=" << (in_bounds_simd_trig ? "true" : "false")
              << " simd_lut=" << (in_bounds_simd_lut ? "true" : "false")
              << " simd_trig_al=" << (in_bounds_simd_trig_al ? "true" : "false")
              << " simd_lut_al=" << (in_bounds_simd_lut_al ? "true" : "false") << "\n\n";

    // Benchmark harness for 5 functions
    auto bench_once_vec = [&](const std::string& name, auto&& f, int iters = 200) {
        CPUUsage s_cpu = get_cpu_usage();
        auto     t0    = std::chrono::high_resolution_clock::now();
        bool     ok    = true;
        for (int i = 0; i < iters; ++i)
            ok = f();
        auto     t1    = std::chrono::high_resolution_clock::now();
        CPUUsage e_cpu = get_cpu_usage();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
        double cpu = (e_cpu.total_time - s_cpu.total_time) * 100.0 / (dt / 1000.0);
        return BenchResult{ name, dt, cpu, ok };
    };

    // 1) Scalar
    auto bench_scalar = bench_once_vec("Scalar", [&]() {
        set_outputs(x_scalar, y_scalar);
        bool ok = compute_positions_scalar(ctx);
        get_outputs(x_scalar, y_scalar);
        return ok;
    });

    // 2) SIMD xsimd trig (unaligned)
    auto bench_simd_trig = bench_once_vec("SIMD xsimd trig (unaligned)", [&]() {
        set_outputs(x_simd_trig, y_simd_trig);
        bool ok = compute_positions_simd_xsimd_trig(ctx);
        get_outputs(x_simd_trig, y_simd_trig);
        return ok;
    });

    // 3) SIMD LUT trig (unaligned)
    auto bench_simd_lut = bench_once_vec("SIMD LUT trig (unaligned)", [&]() {
        set_outputs(x_simd_lut, y_simd_lut);
        bool ok = compute_positions_simd_lut_trig(ctx, lut);
        get_outputs(x_simd_lut, y_simd_lut);
        return ok;
    });

    // 4) SIMD xsimd trig (aligned)
    auto bench_simd_trig_al = bench_once_vec("SIMD xsimd trig (aligned)", [&]() {
        // compute directly into aligned buffers
        return compute_positions_simd_xsimd_trig_aligned(
            width,
            height,
            id_samples,
            seg,
            x_simd_trig_al,
            y_simd_trig_al);
    });

    // 5) SIMD LUT trig (aligned)
    auto bench_simd_lut_al = bench_once_vec("SIMD LUT trig (aligned)", [&]() {
        return compute_positions_simd_lut_trig_aligned(
            width,
            height,
            id_samples,
            seg,
            x_simd_lut_al,
            y_simd_lut_al,
            lut_al);
    });

    // Report helper
    auto report = [](const BenchResult& br, double base_ms) {
        double speedup = base_ms / br.ms;
        std::cout << std::left << std::setw(30) << br.name << " time=" << std::setw(10)
                  << std::fixed << std::setprecision(3) << br.ms << " ms"
                  << " CPU=" << std::setw(6) << std::setprecision(1) << br.cpu_percent << "%"
                  << " speedup=" << std::setprecision(2) << speedup << "x"
                  << " bounds=" << (br.in_bounds ? "ok" : "oob") << "\n";
    };

    std::cout << "=== Benchmark (id_samples=" << id_samples
              << ", SIMD width=" << xs::batch<float>::size << ") ===\n";
    report(bench_scalar, bench_scalar.ms);
    report(bench_simd_trig, bench_scalar.ms);
    report(bench_simd_lut, bench_scalar.ms);
    report(bench_simd_trig_al, bench_scalar.ms);
    report(bench_simd_lut_al, bench_scalar.ms);

    return 0;
}
