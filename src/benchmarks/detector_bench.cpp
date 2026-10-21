// Detector: MultiMarkerDetector::detectMarkers on real frames, as the node runs it every frame.
//
// Each frame is detected REPEATS times in a row: the found markers are then searched from their
// last centre, and the first target without a marker searches the whole image, as in a replay.
// The checksum covers every call's valid markers bit for bit: it changes with any output, and can
// also change with the compiler or its flags, since floats are compared exactly.
//   detector_bench <frame.ppm>...      (P6 frames in the message's RGB order, data/perf/frames)
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "bench_util.hpp"
#include "whycode/core/multi_marker_detector.hpp"
#include "whycode/image/image_handler.hpp"
#include "whycode/utils/marker_geometry.hpp"

// A frame as the camera message the node receives
static sensor_msgs::msg::Image toMessage(const cv::Mat& rgb)
{
    sensor_msgs::msg::Image image;
    image.width    = rgb.cols;
    image.height   = rgb.rows;
    image.encoding = "rgb8";
    image.step     = rgb.cols * 3;
    image.data.assign(rgb.data, rgb.data + rgb.total() * 3);
    return image;
}

// FNV-1a over a marker's outputs; only a valid marker's fields go on to the localisation
static uint64_t hashMarker(uint64_t hash, const whycon::MarkerDetector::Marker& m)
{
    const auto mix = [&hash](const void* data, size_t size) {
        for (size_t i = 0; i < size; i++)
        {
            hash = (hash ^ static_cast<const uint8_t*>(data)[i]) * 1099511628211ull;
        }
    };
    mix(&m.valid, sizeof(m.valid));
    if (m.valid)
    {
        const float values[] = { m.x, m.y, m.m0, m.m1, m.v0, m.v1, m.angle, m.roundness, m.bwRatio };
        const int   ints[]   = { m.size, m.minx, m.maxx, m.miny, m.maxy, m.mean, m.type };
        mix(values, sizeof(values));
        mix(ints, sizeof(ints));
    }
    return hash;
}

int main(int argc, char** argv)
{
    constexpr int REPEATS = 100;
    constexpr int TARGETS = 6;  // As the Isaac config: more targets than markers in view

    // The Isaac config's detector parameters, the marker's derived from its design as in the node
    const whycon::MarkerGeometry marker(0.20, 6, std::nullopt);
    whycon::DetectorParameters   parameters;
    parameters.min_size                        = 10;
    parameters.max_size                        = 10000000;
    parameters.center_distance_tolerance_ratio = marker.centreToleranceRatio();
    parameters.center_distance_tolerance_abs   = 2.0;
    parameters.roundness_tolerance             = 0.35;
    parameters.circularity_tolerance           = 0.15;
    parameters.ratio_tolerance                 = 0.4;
    parameters.max_eccentricity                = 0.9;
    parameters.inner_diameter                  = marker.whiteAreaDiameter();
    parameters.outer_diameter                  = 0.20;
    parameters.image_gamma                     = 2.2;

    double   total_us = 0;
    uint64_t checksum = 14695981039346656037ull;
    for (int f = 1; f < argc; f++)
    {
        const sensor_msgs::msg::Image frame = toMessage(bench::loadFrame(argv[f], true));
        whycon::ImageHandler          image_handler(frame.width, frame.height, "rgb8");
        whycon::MultiMarkerDetector   detector(TARGETS, frame.width, frame.height, parameters);
        image_handler.updateFromROS(frame);

        std::vector<double> times;
        int                 found = 0;
        for (int r = 0; r < REPEATS; r++)
        {
            const auto t0 = std::chrono::steady_clock::now();
            detector.detectMarkers(image_handler);
            const auto t1 = std::chrono::steady_clock::now();
            times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            found = 0;
            for (int i = 0; i < TARGETS; i++)
            {
                checksum =
                    hashMarker(hashMarker(checksum, detector.circles[i]), detector.outer_circles[i]);
                found += detector.circles[i].valid;
            }
        }
        std::nth_element(times.begin(), times.begin() + REPEATS / 2, times.end());
        total_us += times[REPEATS / 2];
        std::cout << argv[f] << ": " << found << " markers, " << std::fixed << std::setprecision(1)
                  << times[REPEATS / 2] << " us per frame (median)\n";
    }
    std::cout << "mean " << std::fixed << std::setprecision(1) << total_us / (argc - 1)
              << " us per frame; checksum " << std::hex << checksum << "\n";
    return 0;
}
