// Flood fill: the detector's scanline fill (MarkerDetector::analyzeMarkerCandidate searching with
// FreePixels) against the per-pixel 4-connected fill it replaced. Both flood every black, then
// every white segment of a frame in row-major seed order, from the same grey image and threshold;
// their segments must agree in size and bounding box. Real frames are timed. Random frames at
// widths around multiples of 64 check the row padding, which 1280-wide frames never use. Exits 1 on
// any difference.
//   floodfill_bench <frame.pgm>...      (grey frames, e.g. data/perf/frames)
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "bench_util.hpp"
#include "whycode/core/marker_detector.hpp"
#include "whycode/image/free_pixels.hpp"
#include "whycode/image/image_handler.hpp"

struct Segment
{
    int  size, minx, maxx, miny, maxy;
    bool operator==(const Segment&) const = default;
};

// The former fill: a queue of pixels, testing one 4-neighbour at a time
static void floodPerPixel(const cv::Mat& grey, int threshold, std::vector<Segment>& out)
{
    const int            w = grey.cols, h = grey.rows;
    std::vector<uint8_t> visited(static_cast<size_t>(w) * h, 0);
    std::vector<int>     queue(static_cast<size_t>(w) * h);
    out.clear();
    for (const bool black : { true, false })
    {
        for (int seed = 0; seed < w * h; seed++)
        {
            if (visited[seed] || (grey.data[seed] < threshold) != black)
            {
                continue;
            }
            int     head = 0, tail = 0;
            Segment s{ 0, seed % w, seed % w, seed / w, seed / w };
            queue[tail++] = seed;
            visited[seed] = 1;
            while (head < tail)
            {
                const int p = queue[head++], x = p % w, y = p / w;
                s.size++;
                s.minx                     = std::min(s.minx, x);
                s.maxx                     = std::max(s.maxx, x);
                s.miny                     = std::min(s.miny, y);
                s.maxy                     = std::max(s.maxy, y);
                const int neighbours[4][2] = { { x + 1 < w, p + 1 },
                                               { x > 0, p - 1 },
                                               { y + 1 < h, p + w },
                                               { y > 0, p - w } };
                for (const auto& [inside, q] : neighbours)
                {
                    if (inside && !visited[q] && (grey.data[q] < threshold) == black)
                    {
                        visited[q]    = 1;
                        queue[tail++] = q;
                    }
                }
            }
            out.push_back(s);
        }
    }
}

// The detector's fill on the frame its ImageHandler holds, binarised at its threshold
static void floodScanline(
    const whycon::ImageHandler& handler, whycon::MarkerDetector::DetectionContext& context,
    const whycon::DetectorParameters& parameters, std::vector<Segment>& out)
{
    context.cleanupBuffer();
    context.reset();
    whycon::MarkerDetector         detector(&context, parameters);
    whycon::MarkerDetector::Marker marker;
    const int                      w = handler.getWidth(), h = handler.getHeight();
    out.clear();
    for (const bool black : { true, false })
    {
        const whycon::FreePixels free(
            handler.getBinaryPacked().data(),
            context.visited.data(),
            w,
            static_cast<size_t>(context.stride),
            black);
        for (int y = 0; y < h; y++)
        {
            for (int x = free.next(y, 0, w - 1); x >= 0;
                 x     = x + 1 < w ? free.next(y, x + 1, w - 1) : -1)
            {
                detector.analyzeMarkerCandidate(handler, marker, y * w + x, 1.0f, false);
                out.push_back({ marker.size, marker.minx, marker.maxx, marker.miny, marker.maxy });
            }
        }
    }
}

// Loads a grey frame into an ImageHandler as a mono8 message
static void load(whycon::ImageHandler& handler, sensor_msgs::msg::Image& msg, const cv::Mat& grey)
{
    msg.width    = grey.cols;
    msg.height   = grey.rows;
    msg.encoding = "mono8";
    msg.step     = grey.cols;
    msg.data.assign(grey.data, grey.data + grey.total());
    handler.updateFromROS(msg);
}

int main(int argc, char** argv)
{
    constexpr int REPEATS = 20;
    // Every segment counts: no size limit, and the inner (disc) mode skips the roundness test
    whycon::DetectorParameters parameters;
    parameters.min_size = 0;
    parameters.max_size = std::numeric_limits<int>::max();

    long                    segments = 0, mismatches = 0, frames = 0;
    double                  per_pixel_us = 0, scanline_us = 0;
    std::vector<Segment>    a, b;
    sensor_msgs::msg::Image msg;
    for (int f = 1; f < argc; f++)
    {
        const cv::Mat                            grey = bench::loadFrame(argv[f], false);
        whycon::ImageHandler                     handler(grey.cols, grey.rows, "mono8");
        whycon::MarkerDetector::DetectionContext context(grey.cols, grey.rows);
        load(handler, msg, grey);
        for (const int threshold : { 96, 128, 160 })
        {
            handler.binarizeSIMD(static_cast<uint8_t>(threshold));
            per_pixel_us +=
                bench::medianMicros(REPEATS, [&] { floodPerPixel(grey, threshold, a); });
            scanline_us += bench::medianMicros(REPEATS, [&] {
                floodScanline(handler, context, parameters, b);
            });
            segments += static_cast<long>(a.size());
            mismatches += a != b;
            frames++;
        }
    }

    // Random blobs at widths around word boundaries, several heights and thresholds
    long    padded_segments = 0, padded_mismatches = 0, padded_images = 0;
    cv::RNG rng(1);
    for (const int width : { 1, 2, 7, 63, 64, 65, 100, 127, 129, 191, 200, 848 })
    {
        for (const int height : { 1, 3, 40 })
        {
            cv::Mat noise(height, width, CV_8UC1), grey;
            rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
            cv::GaussianBlur(noise, grey, cv::Size(5, 5), 1.5);
            whycon::ImageHandler                     handler(width, height, "mono8");
            whycon::MarkerDetector::DetectionContext context(width, height);
            load(handler, msg, grey);
            for (const int threshold : { 0, 100, 128, 156, 255 })
            {
                handler.binarizeSIMD(static_cast<uint8_t>(threshold));
                floodPerPixel(grey, threshold, a);
                floodScanline(handler, context, parameters, b);
                padded_segments += static_cast<long>(a.size());
                padded_mismatches += a != b;
                padded_images++;
            }
        }
    }

    std::cout << std::fixed << std::setprecision(1) << frames << " frame x threshold floods, "
              << segments << " segments, " << mismatches << " differ; per-pixel fill "
              << per_pixel_us / frames << " us, scanline fill " << scanline_us / frames
              << " us per flood (" << per_pixel_us / scanline_us << "x)\n"
              << padded_images << " random images at widths 1-848, " << padded_segments
              << " segments, " << padded_mismatches << " differ\n";
    return mismatches + padded_mismatches == 0 ? 0 : 1;
}
