// Eigen against xsimd on the detector's float kernels, as LocalizationSystem runs them per code
// ring: the 720 ring coordinates with their bounds check (computeEllipseCoordinates), and the
// signal's mean and binarisation (binarizeSignal). The bit-packed binarisation has no Eigen
// counterpart: Eigen has no lane masks to pack into bits.
//   eigen_vs_xsimd_bench
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "xsimd/xsimd.hpp"

namespace xs = xsimd;

using bf = xs::batch<float>;
template <typename T>
using aligned_vec = std::vector<T, xs::aligned_allocator<T, xs::default_arch::alignment()>>;

constexpr int   SAMPLES = 720;
constexpr int   WIDTH = 1280, HEIGHT = 720;
constexpr float X = 640.3f, Y = 360.7f, M0 = 21.4f, M1 = 13.9f, V0 = 0.8f, V1 = -0.6f;

// computeEllipseCoordinates' loop
static bool ringXsimd(const float* cosv, const float* sinv, float* xs_out, float* ys_out)
{
    constexpr int VS = bf::size;
    bf            bx(X), by(Y), bv0(V0), bv1(V1), bm0(M0), bm1(M1), two(2.0f);
    bf            zeros(0.0f), width_b(static_cast<float>(WIDTH - 1)),
        height_b(static_cast<float>(HEIGHT - 1));
    for (int a = 0; a < SAMPLES; a += VS)
    {
        const bf co = xs::load_aligned(&cosv[a]);
        const bf si = xs::load_aligned(&sinv[a]);
        const bf x  = bx + two * (bm0 * co * bv0 + bm1 * si * bv1);
        const bf y  = by + two * (bm0 * co * bv1 - bm1 * si * bv0);
        xs::store_aligned(&xs_out[a], x);
        xs::store_aligned(&ys_out[a], y);
        if (xs::any((x < zeros) | (x >= width_b) | (y < zeros) | (y >= height_b)))
        {
            return false;
        }
    }
    return true;
}

static bool ringEigen(const float* cosv, const float* sinv, float* xs_out, float* ys_out)
{
    const Eigen::Map<const Eigen::ArrayXf> co(cosv, SAMPLES), si(sinv, SAMPLES);
    Eigen::Map<Eigen::ArrayXf>             x(xs_out, SAMPLES), y(ys_out, SAMPLES);
    x = X + 2.0f * (M0 * co * V0 + M1 * si * V1);
    y = Y + 2.0f * (M0 * co * V1 - M1 * si * V0);
    // Min and max reduce in vectors; Eigen 3.4 evaluates a boolean any() a sample at a time
    return x.minCoeff() >= 0.0f && x.maxCoeff() < WIDTH - 1 && y.minCoeff() >= 0.0f &&
           y.maxCoeff() < HEIGHT - 1;
}

// binarizeSignal: the mean, then 1 above it and 0 elsewhere
static float signalXsimd(const float* signal, float* smooth)
{
    constexpr int VS = bf::size;
    bf            acc0(0.f), acc1(0.f), acc2(0.f), acc3(0.f);
    int           i = 0;
    for (; i + 4 * VS <= SAMPLES; i += 4 * VS)
    {
        acc0 += xs::load_aligned(&signal[i + 0 * VS]);
        acc1 += xs::load_aligned(&signal[i + 1 * VS]);
        acc2 += xs::load_aligned(&signal[i + 2 * VS]);
        acc3 += xs::load_aligned(&signal[i + 3 * VS]);
    }
    bf acc = acc0 + acc1 + acc2 + acc3;
    for (; i + VS <= SAMPLES; i += VS)
        acc += xs::load_aligned(&signal[i]);
    const float avg = xs::reduce_add(acc) / float(SAMPLES);
    const bf    avgv(avg);
    for (i = 0; i < SAMPLES; i += VS)
    {
        xs::store_aligned(
            &smooth[i],
            xs::select(xs::load_aligned(&signal[i]) > avgv, bf(1.f), bf(0.f)));
    }
    return avg;
}

static float signalEigen(const float* signal, float* smooth)
{
    const Eigen::Map<const Eigen::ArrayXf> s(signal, SAMPLES);
    const float                            avg  = s.sum() / float(SAMPLES);
    Eigen::Map<Eigen::ArrayXf>(smooth, SAMPLES) = (s > avg).cast<float>();
    return avg;
}

template <typename F>
static double medianNanos(F kernel)
{
    constexpr int       CALLS = 1000, REPEATS = 101;
    std::vector<double> times;
    for (int r = 0; r < REPEATS; r++)
    {
        const auto t0 = std::chrono::steady_clock::now();
        for (int c = 0; c < CALLS; c++)
            kernel();
        const auto t1 = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / CALLS);
    }
    std::nth_element(times.begin(), times.begin() + REPEATS / 2, times.end());
    return times[REPEATS / 2];
}

int main()
{
    static_assert(SAMPLES % bf::size == 0, "whole vectors: no scalar tails");
    aligned_vec<float> cosv(SAMPLES), sinv(SAMPLES), x1(SAMPLES), y1(SAMPLES), x2(SAMPLES),
        y2(SAMPLES), signal(SAMPLES), s1(SAMPLES), s2(SAMPLES);
    for (int a = 0; a < SAMPLES; a++)
    {
        cosv[a] = std::cos(2.0 * M_PI * a / SAMPLES);
        sinv[a] = std::sin(2.0 * M_PI * a / SAMPLES);
        // A code ring's signal: three channels of a bilinear sample, 0-765
        signal[a] =
            382.5f + 300.0f * std::sin(6.0 * 2.0 * M_PI * a / SAMPLES) + 17.0f * std::cos(a);
    }

    bool         inside1 = false, inside2 = false;
    float        avg1 = 0, avg2 = 0;
    const double ring_xsimd =
        medianNanos([&] { inside1 = ringXsimd(cosv.data(), sinv.data(), x1.data(), y1.data()); });
    const double ring_eigen =
        medianNanos([&] { inside2 = ringEigen(cosv.data(), sinv.data(), x2.data(), y2.data()); });
    const double signal_xsimd = medianNanos([&] { avg1 = signalXsimd(signal.data(), s1.data()); });
    const double signal_eigen = medianNanos([&] { avg2 = signalEigen(signal.data(), s2.data()); });

    float coordinate_difference = 0;
    int   bits_differ           = 0;
    for (int a = 0; a < SAMPLES; a++)
    {
        coordinate_difference =
            std::max({ coordinate_difference, std::abs(x1[a] - x2[a]), std::abs(y1[a] - y2[a]) });
        bits_differ += s1[a] != s2[a];
    }
    std::cout << std::fixed << std::setprecision(1) << "xsimd " << xs::default_arch::name()
              << ", Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "."
              << EIGEN_MINOR_VERSION << " " << Eigen::SimdInstructionSetsInUse() << "; " << SAMPLES
              << " samples per ring\n"
              << "  ring coordinates: xsimd " << ring_xsimd << " ns, Eigen " << ring_eigen
              << " ns; largest difference " << std::setprecision(6) << coordinate_difference
              << " px, both inside: " << (inside1 && inside2) << "\n"
              << std::setprecision(1) << "  signal mean and bits: xsimd " << signal_xsimd
              << " ns, Eigen " << signal_eigen << " ns; means " << std::setprecision(6) << avg1
              << " / " << avg2 << ", " << bits_differ << " bits differ\n";
    return 0;
}
