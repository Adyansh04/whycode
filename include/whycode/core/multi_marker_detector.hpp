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
    static constexpr int SEARCH_MARGIN_PX = 20;  // Window margin beyond a marker's last motion;
                                                 // its image speed changes < 3 px per frame

    MarkerDetector::MarkerPair  marker_pair_result_;
    std::vector<MarkerDetector> detectors;
    int                         width, height, number_of_circles;

    // Search near known markers between full scans (full_scan_interval > 1)
    int                                 full_scan_interval_;
    long                                frame_counter_   = 0;
    bool                                force_full_scan_ = true;
    std::vector<MarkerDetector::Marker> last_valid_outer_;
    std::vector<char>                   found_last_frame_;
    std::vector<float>                  last_motion_px_;

    // Each detector's search threshold this frame, -1 if it did not search; kept for the binary
    // debug panel only
    std::vector<int> debug_thresholds_;

    // Writes the thresholds on the binary panel
    void drawThresholds(cv::Mat& image) const;
};
}  // namespace whycon

#endif  // CIRCLE_MANY_DETECTOR_H
