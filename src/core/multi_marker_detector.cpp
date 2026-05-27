#include "whycode/core/multi_marker_detector.hpp"

#include "whycode/utils/whycon_config.h"

whycon::MultiMarkerDetector::MultiMarkerDetector(
    int _number_of_circles, int _width, int _height, const whycon::DetectorParameters& parameters)
  : context(_width, _height)
  , width(_width)
  , height(_height)
  , number_of_circles(_number_of_circles)
{
    circles.resize(number_of_circles);
    outer_circles.resize(number_of_circles);
    last_valid_circles.resize(number_of_circles);
    // Reserve space once to avoid reallocations
    detectors.reserve(number_of_circles);
    for (int i = 0; i < number_of_circles; i++)
    {
        // Construct MarkerDetector objects directly inside the vector
        detectors.emplace_back(width, height, &context, parameters);
    }
}

whycon::MultiMarkerDetector::~MultiMarkerDetector(void) {}

bool whycon::MultiMarkerDetector::detectMarkers(
    ImageHandler& image_handler, bool reset, DebugImageManager* debug_manager)
{
    bool all_detected = true;

    // if reset was asked, looking for circles anywhere in the image
    if (reset)
    {
        last_valid_circles.clear();
        last_valid_circles.resize(number_of_circles);
    }

    for (int i = 0; i < number_of_circles; i++)
    {
        WHYCON_DEBUG("detecting circle " << i);
        circles[i] = last_valid_circles[i];  // start from last known valid circle's position

        bool is_fast_cleanup_possible = false;

        WHYCON_DEBUG("using threshold " << detectors[i].getCurrentThreshold());
        uchar current_threshold = detectors[i].getCurrentThreshold();
        image_handler.binarizeSIMD(current_threshold);

        detectors[i].detectMarkerPair(
            image_handler,
            is_fast_cleanup_possible,
            marker_pair_result_,
            circles[i],
            debug_manager);
        circles[i]       = marker_pair_result_.inner;
        outer_circles[i] = marker_pair_result_.outer;

        WHYCON_DEBUG("threshold is now " << detectors[i].getCurrentThreshold());

        if (circles[i].valid && outer_circles[i].valid)
        {
            WHYCON_INFO("Detection of circle " << i << " succeeded.");
            last_valid_circles[i] = circles[i];

            WHYCON_DEBUG(
                "adding segment ids: " << context.total_segments - 1 << " and "
                                       << context.total_segments - 2);

            //  insert segment_ids corresponding to inner and outer parts of the valid circle detected
            context.valid_segment_ids.insert(context.total_segments - 1);
            context.valid_segment_ids.insert(context.total_segments - 2);
        }
        else
        {
            all_detected = false;
            // Make all outer and circle valid false after current detector index
            for (int j = i; j < number_of_circles; j++)
            {
                circles[j].valid       = false;
                outer_circles[j].valid = false;
            }

            context.cleanupBuffer();
            WHYCON_INFO("Circle " << i << " detection failed, continuing with remaining markers");

            break;  // if one circle fails(full image is already scanned), stop processing further circles
        }
    }

    //  reset internal context's ids
    context.reset();

    return all_detected;
}
