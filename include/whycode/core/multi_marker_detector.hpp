#ifndef CIRCLE_MANY_DETECTOR_H
#define CIRCLE_MANY_DETECTOR_H

#include <vector>

#include "whycode/core/marker_detector.hpp"
#include "whycode/image/debug_image_manager.hpp"
namespace whycon
{
class MultiMarkerDetector
{
public:
    MultiMarkerDetector(
        int number_of_circles, int width, int height,
        const DetectorParameters& parameters = DetectorParameters());
    ~MultiMarkerDetector(void);

    /* Main function to detect markers*/
    bool detectMarkers(
        ImageHandler& image_handler, bool reset = false, DebugImageManager* debug_manager = nullptr);

    std::vector<MarkerDetector::Marker> circles, outer_circles, last_valid_circles;

    MarkerDetector::DetectionContext context;

private:
    MarkerDetector::MarkerPair  marker_pair_result_;
    std::vector<MarkerDetector> detectors;
    int                         width, height, number_of_circles;
};
}  // namespace whycon

#endif  // CIRCLE_MANY_DETECTOR_H
