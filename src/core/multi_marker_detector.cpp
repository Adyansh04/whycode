#include "whycode/core/multi_marker_detector.hpp"

#include <algorithm>
#include <cmath>

#include "whycode/utils/whycon_config.h"

whycon::MultiMarkerDetector::MultiMarkerDetector(
    int _number_of_circles, int _width, int _height, const whycon::DetectorParameters& parameters)
  : context(_width, _height)
  , width(_width)
  , height(_height)
  , number_of_circles(_number_of_circles)
  , full_scan_interval_(parameters.full_scan_interval)
{
    circles.resize(number_of_circles);
    outer_circles.resize(number_of_circles);
    last_valid_circles.resize(number_of_circles);
    last_valid_outer_.resize(number_of_circles);
    found_last_frame_.assign(number_of_circles, 0);
    last_motion_px_.assign(number_of_circles, 0.0f);
    // Reserve space once to avoid reallocations
    detectors.reserve(number_of_circles);
    for (int i = 0; i < number_of_circles; i++)
    {
        // Construct MarkerDetector objects directly inside the vector
        detectors.emplace_back(&context, parameters);
    }
}

whycon::MultiMarkerDetector::~MultiMarkerDetector(void) {}

bool whycon::MultiMarkerDetector::detectMarkers(
    ImageHandler& image_handler, bool reset, DebugImageManager* debug_manager)
{
    bool all_detected        = true;
    int  binarized_threshold = -1;

    // if reset was asked, looking for circles anywhere in the image
    if (reset)
    {
        last_valid_circles.clear();
        last_valid_circles.resize(number_of_circles);
    }

    // Between full scans, markers are looked for only near where they were in the last frame;
    // new ones wait for the next full scan
    const bool full_scan = reset || force_full_scan_ || full_scan_interval_ <= 1 ||
                           frame_counter_ % full_scan_interval_ == 0;
    frame_counter_++;
    force_full_scan_ = false;

    for (int i = 0; i < number_of_circles; i++)
    {
        WHYCON_DEBUG("detecting circle " << i);
        circles[i] = last_valid_circles[i];  // start from last known valid circle's position

        MarkerDetector::SearchWindow window{};
        if (!full_scan)
        {
            if (!found_last_frame_[i])
            {
                circles[i].valid = outer_circles[i].valid = false;
                all_detected                              = false;
                continue;
            }
            const auto& last   = last_valid_outer_[i];
            const int   margin = SEARCH_MARGIN_PX + static_cast<int>(last_motion_px_[i]);
            window             = { std::max(last.minx - margin, 0),
                                   std::max(last.miny - margin, 0),
                                   std::min(last.maxx + margin + 1, width),
                                   std::min(last.maxy + margin + 1, height) };
        }

        WHYCON_DEBUG("using threshold " << detectors[i].getCurrentThreshold());
        const int current_threshold = detectors[i].getCurrentThreshold();
        // Detectors usually share a threshold; binarise the frame once per distinct value
        if (current_threshold != binarized_threshold)
        {
            image_handler.binarizeSIMD(static_cast<uchar>(current_threshold));
            binarized_threshold = current_threshold;
        }

        detectors[i].detectMarkerPair(
            image_handler,
            marker_pair_result_,
            circles[i],
            debug_manager,
            full_scan ? nullptr : &window);
        circles[i]       = marker_pair_result_.inner;
        outer_circles[i] = marker_pair_result_.outer;

        WHYCON_DEBUG("threshold is now " << detectors[i].getCurrentThreshold());

        if (circles[i].valid && outer_circles[i].valid)
        {
            WHYCON_INFO("Detection of circle " << i << " succeeded.");
            last_motion_px_[i]    = found_last_frame_[i] ?
                                        std::hypot(
                                         outer_circles[i].x - last_valid_outer_[i].x,
                                         outer_circles[i].y - last_valid_outer_[i].y) :
                                        0.0f;
            last_valid_circles[i] = circles[i];
            last_valid_outer_[i]  = outer_circles[i];
            found_last_frame_[i]  = 1;
        }
        else if (!full_scan)
        {
            // A tracked marker left its window: look everywhere next frame
            all_detected         = false;
            found_last_frame_[i] = 0;
            force_full_scan_     = true;
        }
        else
        {
            all_detected = false;
            // Make all outer and circle valid false after current detector index
            for (int j = i; j < number_of_circles; j++)
            {
                circles[j].valid       = false;
                outer_circles[j].valid = false;
                found_last_frame_[j]   = 0;
            }

            WHYCON_INFO("Circle " << i << " detection failed, continuing with remaining markers");

            break;  // if one circle fails(full image is already scanned), stop processing further circles
        }
    }

    // Every visited pixel stays marked until cleared here; a stale mark hides that pixel from the
    // next frame's search. Clearing the whole bitset is one ~115 KB fill.
    context.cleanupBuffer();

    //  reset internal context's ids
    context.reset();

    return all_detected;
}
