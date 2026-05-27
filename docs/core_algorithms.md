# Core Algorithms by File

This document maps each core source file to its primary algorithm and runtime role.

## ROS2 Interface Layer

### `src/ros/whycon_ros_interface.cpp`

- Loads YAML-backed runtime parameters and camera intrinsics.
- Creates subscriber, timer, service, and publishers.
- Runs frame-gated processing (`new_frame_available`) and publishes all outputs.
- Key path:
  - `onRosImageReceived` captures latest frame metadata.
  - `processLatestFrame` runs detection and tracking.
  - `publishResults` converts algorithm outputs to ROS messages and TF.

### `src/ros/whycon_component.cpp`

- Registers `whycon::WhyconComponent` for composable node loading.
- Instantiates `WhyconRosInterface` inside a component container process.

### `src/ros/whycon_localization_node.cpp`

- Standalone executable entrypoint using `rclcpp::spin`.

## Image Processing Layer

### `src/image/image_handler.cpp`

- Owns contiguous frame buffer and aligned grayscale storage.
- Converts incoming RGB data to grayscale using SimdLib (`SimdRgbToGray`).
- Provides binary threshold paths and packed-binary visualization.

### `src/image/packed_binary_image.cpp`

- Maintains bit-packed binary frame (1 bit per pixel conceptually).
- Supports row scanning helpers and conversion back to display matrix.

### `src/image/debug_image_manager.cpp`

- Collects stage debug images and stitches them into a tiled dashboard.
- Uses grid layout heuristics and aspect-preserving resize.

## Detection and Decode Layer

### `src/core/multi_marker_detector.cpp`

- Orchestrates repeated `MarkerDetector` runs for `targets` markers.
- Handles reset vs incremental detection strategy across frames.

### `src/core/marker_detector.cpp`

- High-performance ellipse candidate extraction and validation.
- Implements scalar and SIMD statistics for ellipse fitting.
- Validates inner-outer ring pairing via geometric constraints.
- Writes detection metadata used later by pose estimation and decode.

### `src/core/whycon_localization.cpp`

- Main localization logic around detected ellipses.
- Resolves dual-solution ambiguity from perspective projection.
- Performs ring sampling and binarization for WhyCode decode.
- Computes orientation quaternion and Euler angles.
- Handles undistortion mapping and camera-space geometry math.

### `src/core/CNecklace.cpp`

- Encodes/decodes circular binary codes.
- Computes Hamming distances and optional probabilistic confidence handling.
- Produces decoded marker ID and angular phase.

## Tracking and Stabilization Layer

### `src/tracking/marker_tracker.cpp`

- Maintains persistent tracks using Kalman filtering per marker.
- Performs greedy association between predictions and new detections.
- Handles track creation, timeout, and removal bookkeeping.

### `src/tracking/id_stabilizer.cpp`

- Stabilizes decoded IDs over time per tracking ID.
- Uses confidence counters and switch thresholds to avoid flicker.

### `src/tracking/pose_stabilizer.cpp`

- Applies EMA filtering to marker positions per tracking ID.

## Triangulation Layer

### `src/triangulate/two_marker_whycode_triangulation.cpp`

- Uses two marker poses with known baseline distance.
- Builds a local frame from marker geometry.
- Estimates center pose and optional camera odometry/TF.

### `src/triangulate/four_marker_whycode_triangulation.cpp`

- Hierarchical triangulation:
  - left column center from top-left and bottom-left,
  - right column center from top-right and bottom-right,
  - final center from left/right centers.
- Publishes intermediate and final TF outputs.

## Configuration and Utility Layer

### `src/utils/param_loader.cpp`

- Type-safe YAML parameter navigation and conversion.
- Camera intrinsics loading into OpenCV matrices.

### `src/utils/whycon_config.cpp`

- Global singleton-style access to loaded parameter set.

### `src/utils/coord_lut.cpp`

- Shared coordinate lookup table used by detection/localization hot paths.
