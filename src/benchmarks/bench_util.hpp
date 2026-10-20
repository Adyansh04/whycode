#ifndef WHYCON_BENCH_UTIL_HPP
#define WHYCON_BENCH_UTIL_HPP

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace bench
{
/**
 * @brief Loads a frame saved from a bag, or exits: a missing frame must not pass as an empty one.
 * @param path PGM or PPM file.
 * @param colour False for a grey CV_8UC1 frame, true for an RGB CV_8UC3 one (the bag's byte order).
 * @return The frame.
 */
inline cv::Mat loadFrame(const std::string& path, bool colour)
{
    cv::Mat frame = cv::imread(path, colour ? cv::IMREAD_COLOR : cv::IMREAD_GRAYSCALE);
    if (frame.empty())
    {
        std::cerr << "cannot read " << path << "\n";
        std::exit(2);
    }
    if (colour)
    {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);  // imread gives BGR
    }
    return frame;
}

/**
 * @brief Median wall time of repeated runs of a kernel.
 * @param repeats Number of timed runs.
 * @param kernel The code to time.
 * @param before Untimed work before each run, e.g. flushing the caches.
 * @return Median time of one run, in microseconds.
 */
template <typename Kernel, typename Before>
double medianMicros(int repeats, Kernel&& kernel, Before&& before)
{
    std::vector<double> times;
    for (int r = 0; r < repeats; r++)
    {
        before();
        const auto t0 = std::chrono::steady_clock::now();
        kernel();
        const auto t1 = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::nth_element(times.begin(), times.begin() + repeats / 2, times.end());
    return times[repeats / 2];
}

template <typename Kernel>
double medianMicros(int repeats, Kernel&& kernel)
{
    return medianMicros(repeats, kernel, [] {});
}
}  // namespace bench

#endif  // WHYCON_BENCH_UTIL_HPP
