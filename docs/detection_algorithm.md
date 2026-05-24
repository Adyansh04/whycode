# WhyCon Multi-Marker Detection Algorithm Analysis - Partial Detection Implementation

## Overview
This comment outlines the WhyCon localization system's approach to detecting multiple markers in a 1280x720 camera feed, including the implementation of partial marker detection capabilities. The system has been modified to continue processing when not all target markers are present in the frame.

## Algorithm Architecture

### Sequential Detection Strategy
The system processes markers using a sequential approach where each `MarkerDetector` instance handles one target:

```cpp
// From multi_marker_detector.cpp - detectMarkers()
for (int i = 0; i < number_of_circles; i++) {  // Process each target sequentially
    for (int j = 0; j < max_attempts; j++) {
        MarkerDetector::MarkerPair pair = detectors[i]->detectMarkerPair(input, fast_cleanup_possible, circles[i]);
        
        if (circles[i].valid) {
            context.valid_segment_ids.insert(context.total_segments - 1);  // Prevent re-detection
            break;
        }
    }
    
    if (!circles[i].valid) {
        all_detected = false;
        // Continue processing remaining markers (modified behavior)
    }
}
```

## Image Processing Flow

### Pixel Classification Process
Each detector uses `classifyPixelByThreshold()` to categorize pixels:

```cpp
inline int classifyPixelByThreshold(uchar* ptr) {
    return ((ptr[0] + ptr[1] + ptr[2]) > threshold) + BLACK;  // Returns BLACK or WHITE
}
```

**Processing metrics for 1280x720 resolution:**
- **Full scan**: Up to 921,600 pixel classifications per detector
- **Local window mode**: ~40,000 classifications (96% reduction)
- **Shared buffer**: Prevents redundant pixel processing across detectors

### Scanning Strategies

#### Strategy 1: Local Window Search (Tracking Mode)
When `previous_circle.valid` is true:

```cpp
if (previous_circle.valid) {
    seed_pixel_index = (int(previous_circle.y)) * width + int(previous_circle.x);
    
    if (use_local_window) {
        local_window_width = local_window_multiplier * (previous_circle.maxx - previous_circle.minx);
        local_window_height = local_window_multiplier * (previous_circle.maxy - previous_circle.miny);
        search_in_window = true;
    }
}
```

#### Strategy 2: Full Image Scan (Initial Detection)
When no previous detection exists, the detector scans the entire 1280x720 frame systematically.

### Flood-Fill Segmentation

When a BLACK pixel is detected, `analyzeMarkerCandidate()` performs connected component analysis:

```cpp
// Initialize segment with unique ID
int segment_id = context->total_segments++;
buffer[seed_pixel_index] = segment_id;

// Flood-fill algorithm
while (queue_end > queue_start) {
    position = queue[queue_start++];
    
    // Check 4-connected neighbors (right, left, up, down)
    for (each neighbor) {
        if (isPixelUnclassified(pixel_class)) {
            pixel_class = classifyPixelByThreshold(ptr);
        }
        if (pixel_class == type) {
            queue[queue_end++] = neighbor_pos;
            buffer[neighbor_pos] = segment_id;
        }
    }
}
```

**Segmentation continues until:**
- Queue becomes empty (all connected pixels processed)
- `MAX_SEGMENTS` limit reached (10,000 segments)

### Geometric Validation

Valid segments undergo ellipse parameter computation via `computeEllipseParameters()`:

```cpp
// Calculate covariance matrix for ellipse fitting
float cov_xx = (sum_xx - mean_x * mean_x * num_points) / num_points;
float cov_xy = (sum_xy - mean_x * mean_y * num_points) / num_points;
float cov_yy = (sum_yy - mean_y * mean_y * num_points) / num_points;

// Compute eigenvalues for axis lengths
float lambda1 = (trace + sqrt_term) / 2.0f;  // major axis squared
float lambda2 = (trace - sqrt_term) / 2.0f;  // minor axis squared
```

## Multi-Marker Conflict Resolution

### Shared Context Management
All detectors share a single `DetectionContext`:

```cpp
class DetectionContext {
    std::vector<int> buffer, queue;              // Shared segmentation buffer
    std::unordered_set<int> valid_segment_ids;   // Tracks successfully detected markers
    int next_detector_id;                        // Ensures unique detector identification
    int total_segments;                          // Running segment counter
};
```

### Pixel Ownership Prevention
`isPixelUnclassified()` prevents multiple detections of the same marker:

```cpp
inline bool isPixelUnclassified(int pixel_class) {
    if (pixel_class < 0) {
        return (pixel_class != BLACK && pixel_class != WHITE);
    } else {
        return (pixel_class < initial_segment_id &&
                context->valid_segment_ids.find(pixel_class) == context->valid_segment_ids.end());
    }
}
```

## Partial Detection Implementation

### Modified Continuation Logic
The system now continues processing when individual markers fail:

```cpp
if (!circles[i].valid) {
    all_detected = false;
    WHYCON_INFO("Circle " << i << " detection failed, continuing with remaining markers");
    // Removed break statement - continues to next marker
}
```

### Result Publishing Modifications
`publishResults()` now handles partial detections:

```cpp
int detected_count = system->get_detected_marker_count();
if (detected_count > 0) {
    for (int i = 0; i < system->targets; i++) {
        if (!system->is_marker_detected(i)) {
            continue;  // Skip invalid markers
        }
        // Process valid markers only
    }
}
```

## Performance Characteristics

### Detection Scenarios (1280x720 resolution)

**Scenario 1: All 4 markers present**
- Sequential processing through 4 detectors
- Each subsequent detector avoids pixels marked by previous detectors

**Scenario 2: Partial detection (2/4 markers)**
- Detectors 0,1: Successful detection with local windows (Lesser time)
- Detectors 2,3: Full frame scan, fail to find markers (approx 3x more time)
- System publishes results for 2 detected markers

**Scenario 3: Tracking mode**
- All detectors use local windows around previous positions
- Processing time reduced 
- 90%+ pixel classification reduction compared to full scan

### Function Call Frequency Analysis

**Full scan mode (no tracking):**
- `classifyPixelByThreshold()`: Up to 3.7M calls (4 detectors × 921,600 pixels)
- `analyzeMarkerCandidate()`: 10-100 calls per detector
- `computeEllipseParameters()`: 2-10 calls per valid detection

**Tracking mode:**
- `classifyPixelByThreshold()`: ~160,000 calls (97% reduction)
- Detection functions called within local windows only

## Threshold Adaptation

When detection fails, `adjustThreshold()` modifies pixel classification:

```cpp
void adjustThreshold(void) {
    threshold_counter++;
    int step = 256 / div;
    threshold = 3 * (step * (threshold_counter - div) + step / 2);
    if (step <= 16) threshold_counter = 0;
}
```

This systematic threshold adjustment attempts to improve detection on subsequent frames.
