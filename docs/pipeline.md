# Detection and Tracking Pipeline

This document details the exact processing flow for one frame in `whycode_vision`, and maps each stage to concrete source files.

## End-to-End Frame Flow

```mermaid
flowchart TD
  A["ROS image message"] --> B["WhyconRosInterface onRosImageReceived"]
  B --> C["ImageHandler updateFromROS"]
  C --> D["set new_frame_available"]
  D --> E["process timer callback"]
  E --> F["processLatestFrame"]

  F --> G["localizeMarkers: ellipse detection"]
  G --> H["collect valid detections"]
  H --> I["MarkerTracker update"]
  I --> J["ID and pose stabilization"]
  J --> K["estimateMarkerPose per target"]
  K --> L["publish pose image tf markers"]
```

## WhyCode Decode Sub-Flow

```mermaid
flowchart LR
  S1["outer and inner ellipse"] --> S2["calc ellipse centers two solutions"]
  S2 --> S3["sample ring signal for solution zero"]
  S2 --> S4["sample ring signal for solution one"]
  S3 --> S5["binarize and edge analysis"]
  S4 --> S6["binarize and edge analysis"]
  S5 --> S7["choose lower variance solution"]
  S6 --> S7
  S7 --> S8["CNecklace decode and hamming validation"]
  S8 --> S9["pose orientation and euler update"]
```

## Stage-by-Stage Mapping

| Stage | Main Functions | Files |
|---|---|---|
| Image receive | `onRosImageReceived`, `updateFromROS` | `src/ros/whycon_ros_interface.cpp`, `src/image/image_handler.cpp` |
| Detector pass | `localizeMarkers`, `detectMarkers`, `detectMarkerPair` | `src/core/whycon_localization.cpp`, `src/core/multi_marker_detector.cpp`, `src/core/marker_detector.cpp` |
| WhyCode decode | `processMarkerAmbiguityAndIdentify`, `processSingleSolution`, `selectSolutionAndDecodeID`, `CNecklace::decode` | `src/core/whycon_localization.cpp`, `src/core/CNecklace.cpp` |
| Tracking | `MarkerTracker::update` | `src/tracking/marker_tracker.cpp` |
| Stabilization | `IDStabilizer::stabilize`, `PoseStabilizer::stabilize` | `src/tracking/id_stabilizer.cpp`, `src/tracking/pose_stabilizer.cpp` |
| Publish | `publishResults`, `publishSingleTF`, `createMarkerVisualization` | `src/ros/whycon_ros_interface.cpp` |

## Exact Publish Flow

```mermaid
flowchart TD
  P0["publishResults start"] --> P1["optional image buffer export"]
  P1 --> P2["loop targets 0 to N"]
  P2 --> P3["is marker detected and mature track"]
  P3 --> P4["estimate pose and stabilize"]
  P4 --> P5["append WhyCodePose message"]
  P4 --> P6["draw overlays on image"]
  P4 --> P7["send TF transform"]
  P4 --> P8["append visualization marker"]
  P5 --> P9["publish WhyCodePoseArray"]
  P6 --> P10["publish image out"]
  P8 --> P11["publish MarkerArray with stale deletes"]
```

## Triangulation Flow

- Input for both nodes: `whycode_vision/msg/WhyCodePoseArray` on `/whycon/poses`.
- Two-marker node:
  - Matches configured marker pair.
  - Estimates midpoint frame and orientation.
  - Publishes pose, optional TF, and camera odometry.
- Four-marker node:
  - Computes left-center and right-center intermediate poses.
  - Computes final center from intermediate centers.
  - Publishes intermediate/final TF and camera odometry.

## Determinism Notes

- Processing is driven by a timer with a `new_frame_available` gate.
- Stabilizers make output less noisy but introduce expected temporal smoothing behavior.
- Marker age gates are enforced before publish to reject very young tracks.
