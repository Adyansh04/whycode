# WhyCon/WhyCode Localization ROS Package

## Overview

`whycon_whycode_localization` is a ROS package for detecting and localizing circular WhyCon/WhyCode markers and decoding embedded WhyCode IDs. It estimates the 3D pose of these markers relative to a camera.

## Features

* **Multi-Marker Detection**: Detects multiple circular black and white WhyCon markers simultaneously.
* **WhyCode Decoding**: Decodes WhyCode IDs embedded within the markers.
* **Pose Estimation**: Estimates the 6-DOF pose (position and orientation) of each detected marker.
* **Flexible Build Options**: Can be built as a standard ROS package or as a standalone C++ library by setting the `DISABLE_ROS` flag.
* **Shared Memory Mode (Iceoryx)**: High-throughput image ingestion via Iceoryx with listener-based, latest-only semantics and fixed-rate processing; consistent `rgb8` pipeline end-to-end. Tools `bag_to_shm` and `shm_viewer` included for quick validation.
* **ID Tracking & Stabilization**: Reduces ID flicker using a hysteresis-based ID stabilizer and maintains tracks over time with lightweight 2D tracking.

## WhyCon Marker Detection Range

The maximum detection range of WhyCon markers depends on both the **physical size of the marker** and the **camera resolution**. Below are reference values measured in typical warehouse lighting with a RealSense camera:

| Marker Outer Diameter (m) | Marker Inner Diameter (m) | Resolution      | Max Detection Range (m) | CPU Usage (%) |
|--------------------------:|--------------------------:|:---------------:|------------------------:|--------------:|
| 0.146                     | 0.088                     | 640×480         | 10                      | 10-13         |
| 0.146                     | 0.088                     | 1280×720        | 14                      | 30-33         |
| 0.146                     | 0.088                     | 1920×1080       | 17                      | 80 - 85       |
| 0.206                     | 0.124                     | 1280×720        | 20                      | 30-33         |
| 0.206                     | 0.124                     | 1920×1080       | 32                      | 80 - 85       |

**Notes:**

* CPU usage was measured using `top` in a Docker container with specified CPU cores, ensuring no other process runs on the same core.
* CPU usage values can fluctuate by approximately ±5% depending on system load and runtime conditions.
* Shared memory mode was used in this test.

## Tools

Utility tools live in `src/tools` and are built when `BUILD_TOOLS=ON` (default). They help generate markers, stream images via shared memory, and validate the pipeline quickly.

### WhyCode Marker Generator (`whycon-id-gen`)

Generates printable WhyCon/WhyCode markers for lab testing and simulation assets.

* Help: `rosrun whycon_whycode_localization whycon-id-gen -h`
* Examples:
  * Basic WhyCon marker: `whycon-id-gen -l`
  * WhyCode with 6 bits: `whycon-id-gen 6`
  * WhyCode with 8 bits, Hamming distance 2: `whycon-id-gen -d 2 8`

### bag_to_shm

Publishes RGB8 images from a ROS bag file to Iceoryx shared memory. Useful to drive the SHM-based acquisition path without a live camera.

* Input: a `.bag` containing `sensor_msgs/Image`.
* Output: Iceoryx service (default event: `RGB`) carrying an image header + payload.
* Key options:
  * `-r <rate>`: playback rate multiplier (e.g., `0.5` = half speed, `2.0` = double)
  * `-l`: loop the bag
  * `-s <sec>`: skip initial seconds

Where it fits:

* Feed `whycon_ros_interface` when `input_source=iceoryx` in the YAML config and `iceoryx.service_event` matches (default `RGB`).
* Sanity-check end-to-end with `shm_viewer` below.

### shm_viewer

Subscribes to Iceoryx and displays frames using OpenCV, logging FPS and dropped frames.

* Input: Iceoryx event (default `RGB`) produced by `bag_to_shm` or any SHM publisher.
* Behavior: converts RGB→BGR for display correctness, shows a window, logs throughput.

Where it fits:

* Quick validation of SHM image publishing, color consistency, and frame rate without running the full WhyCon stack.

### Bag Image Publisher Nodelet

A ROS nodelet that publishes images from a bag via `image_transport`. Use it for ROS-only pipelines when SHM is not needed.

* Configure via private parameters (e.g., `bag_path`, image topic) in a launch file like `whycon.launch`.
* Integrates naturally with the nodelet-based `whycon_ros_interface` for zero-copy intra-process performance.

## Triangulation nodes (src/triangulate)

These ROS nodes estimate poses using known marker layouts. They are built when `BUILD_TRIANGULATION=ON` (default). The AprilTag variant is built only if `apriltag_ros` is available.

### two_marker_whycode_triangulation_node

* Purpose: Compute a pose from two WhyCode markers with a known separation.
* Input: `whycon_whycode_localization/WhyCodePoseArray` (from the main WhyCon node).
* Output: `geometry_msgs/PoseStamped` and TF (broadcast), plus optional `nav_msgs/Odometry` if enabled.
* Notes: Expects two specific marker IDs; includes utilities to compute roll/pitch/yaw from the camera plane.

### two_marker_apriltag_triangulation_node

* Purpose: Compute a pose from two AprilTags with a known separation.
* Input: `apriltag_ros/AprilTagDetectionArray`.
* Output: `geometry_msgs/PoseStamped` and TF (broadcast), plus optional `nav_msgs/Odometry` if enabled.
* Notes: Built only if `apriltag_ros` is present. Configure target tag IDs and the known distance.

### four_marker_whycode_triangulation_node

* Purpose: Hierarchical triangulation using four WhyCode markers to improve robustness.
* Input: `whycon_whycode_localization/WhyCodePoseArray`.
* Output: `geometry_msgs/PoseStamped` and TF (broadcast), plus optional `nav_msgs/Odometry`.
* Notes: Designed for rigs with four arranged WhyCode markers; see `src/triangulate/four_marker_whycode_triangulation.cpp` for parameter hints.

Where these fit:

* Run alongside the main detector node. Point their input topic to the detector’s published WhyCode poses (or AprilTag detections).
* Use TF to integrate the estimated pose into your robot’s frame tree.

## Benchmarks (src/benchmarks)

Built when `BUILD_BENCHMARKS=ON` (default). Requires xsimd. These small programs measure micro-performance of core routines:

* `benchmark`: End-to-end timing of core WhyCon algorithms in a synthetic setup.
* `segment_bench`: Benchmarks segment computation used in marker decoding.
* `ellipse_bench`: Tests ellipse-related computations (fit/evaluate) for detector performance.
* `binarize_loop_bench`: Measures thresholding/binarization loop throughput.
* `bilinear_bench`: Times bilinear interpolation used in image sampling.
* `benchmark_binarization`: Compares different binarization strategies and parameters.
* `cv_simd`: Sanity test for SIMD-accelerated OpenCV/xsimd integration.
* `branch_pred`: Micro-benchmark to see impact of branch prediction on tight loops.
* `class_test`: Evaluates class/object overhead patterns relevant to hot paths.

## Architecture

![WhyCon Architecture](docs/architecture.svg)

## Dependencies

### System Dependencies

* OpenCV 4.2
* yaml-cpp
* [SIMD](https://github.com/ermig1979/Simd)
* [xSimd](https://github.com/xtensor-stack/xsimd)
* [iceoryx](https://iceoryx.io/) (for shared memory)

### ROS Dependencies

* `roscpp`
* `std_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`
* `image_transport`, `cv_bridge`
* `nodelet`, `pluginlib`
* `tf`
* `rosbag` (for the bag publisher nodelet)

## Building the Package

The build is modular and controlled via CMake options exposed to catkin. By default, all modules are enabled.

Build options (defaults in parentheses):

* BUILD_TOOLS (ON): Build utility tools under `src/tools`.

  * Executables: `bag_to_shm`, `shm_viewer`, and the bag publisher nodelet is added to the nodelet library.

* BUILD_TRIANGULATION (ON): Build triangulation nodes under `src/triangulate`.

  * Executables: `two_marker_whycode_triangulation_node`, `two_marker_apriltag_triangulation_node` (if `apriltag_ros` is found), `four_marker_whycode_triangulation_node`.

* BUILD_BENCHMARKS (ON): Build benchmarks under `src/benchmarks` (requires xsimd).

  * Executables vary by files in `src/benchmarks` (e.g., `benchmark`, `segment_bench`, etc.).

* DISABLE_ROS (OFF): Build only the core C++ library without ROS targets.

  * Library: `whycon_core` and tool `whycon-id-gen`.

Typical builds:

1. Default (everything enabled)

```bash
cd /path/to/your/catkin_ws
catkin_make
source devel/setup.bash
```

1. Minimal runtime (no tools, no benchmarks, no triangulation)

```bash
catkin_make -DBUILD_TOOLS=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TRIANGULATION=OFF
```

1. Only tools

```bash
catkin_make -DBUILD_TRIANGULATION=OFF -DBUILD_BENCHMARKS=OFF
```

1. Only triangulation nodes

```bash
catkin_make -DBUILD_TOOLS=OFF -DBUILD_BENCHMARKS=OFF
```

1. Standalone core library (no ROS)

```bash
catkin_make -DDISABLE_ROS=ON
```

Optimization notes:

* The build enables high-performance flags by default (e.g., `-O3`, vectorization, and `-mavx2` when supported).
* Benchmarks make use of xsimd; ensure xsimd is installed if `BUILD_BENCHMARKS=ON`.

### Install prerequisite libraries (Simd, xsimd, Iceoryx)

These projects depend on Simd, xsimd, and Iceoryx. Expand the sections below to see installation commands.

<details>
<summary><strong>Install Simd (ermig1979/Simd)</strong></summary>

```bash
git clone https://github.com/ermig1979/Simd.git

cd Simd
mkdir -p build
cd build

cmake ../prj/cmake \
  -DSIMD_TOOLCHAIN="" \
  -DSIMD_TARGET="" \
  -DSIMD_AVX512=ON \
  -DSIMD_AVX512VNNI=ON \
  -DSIMD_AMXBF16=ON \
  -DSIMD_TEST=ON \
  -DSIMD_INFO=ON \
  -DSIMD_PERF=OFF \
  -DSIMD_SHARED=ON \
  -DSIMD_GET_VERSION=ON \
  -DSIMD_SYNET=ON \
  -DSIMD_INT8_DEBUG=OFF \
  -DSIMD_HIDE=OFF \
  -DSIMD_RUNTIME=ON \
  -DSIMD_OPENCV=ON \
  -DSIMD_INSTALL=ON \
  -DSIMD_UNINSTALL=ON \
  -DSIMD_PYTHON=ON

make -j 20
sudo make install
```

 </details>

<details>
<summary><strong>Install xsimd (xtensor-stack/xsimd)</strong></summary>

```bash
git clone https://github.com/xtensor-stack/xsimd.git

cd xsimd
mkdir -p build
cd build

cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j"$(nproc)"
sudo make install
```

 </details>

<details>
<summary><strong>System packages and Iceoryx</strong></summary>

```bash
sudo apt update
sudo apt install -y gcc g++ cmake libacl1-dev libncurses5-dev pkg-config

git clone https://github.com/eclipse-iceoryx/iceoryx.git
cd iceoryx
cmake -Bbuild -Hiceoryx_meta
cmake --build build
sudo cmake --build build --target install
```

 </details>

## Usage

The primary way to run the package is through the `whycon.launch` file, which can start the system as a standard node or as a nodelet.

### Running as a Nodelet (Recommended)

This is the default and recommended mode for performance.

```bash
roslaunch whycon_whycode_localization whycon.launch nodelet:=true
```

### Running as a Standard Node

```bash
roslaunch whycon_whycode_localization whycon.launch nodelet:=false
```

### Launch Arguments

* `nodelet` (bool, default: `true`): If true, runs as a nodelet. Otherwise, runs as a standard node.
* `config_file` (string, default: `.../config/whycon_config_rs.yaml`): Path to the main configuration file.
* `image_view` (bool, default: `false`): If true, launches an `image_view` node to display the annotated image output.

### Testing with the Bag File Publisher

To test the system without a live camera, you can use the included `whycon_nodelet.launch` file, which loads both the bag publisher and the WhyCon detector in the same nodelet manager.

1. Update the `bag_path` parameter in `whycon_nodelet.launch` to point to your bag file.
2. Launch the test:

    ```bash
    roslaunch whycon_whycode_localization whycon_nodelet.launch
    ```

## Running in Shared Memory (Iceoryx) mode

Use this mode to read images from shared memory instead of a ROS image topic. This is handy for high-throughput pipelines or when driving from a bag via `bag_to_shm`.

1. Update your config YAML to enable Iceoryx input and match the service event name. The tools in this repo publish RGB images on the `RGB` event by default.

   Example (`config/whycon_config_rs.yaml`):

   ```yaml
   system:
     input_source: "iceoryx"     # ros or iceoryx
     process_rate_hz: 25.0

   iceoryx:
     runtime_name: "WhyconImageHandler"  # Any unique runtime name for this process
     service_instance: "Camera"             # Service instance name
     service_method: "Image"                # Service method name  
     service_event: "RGB"                   # Service event name

   detector:
     # … your detector params …

   camera:
     image_width: 1280
     image_height: 720
   ```

   Notes:
   * The `service names` must match with its publisher. The `runtime_name` just identifies this subscriber to Iceoryx and can be any unique string.
   * The shared-memory pipeline uses RGB8 end-to-end; keep `image_encoding` set to `rgb8`.

2. Start the Iceoryx RouDi daemon if your system requires it:

   ```bash
   iox-roudi -c custom_iceoryx.toml  # Larger size and chunk for bigger payloads
   ```

3. Publish frames to shared memory (from a bag) with `bag_to_shm`.

4. Launch WhyCon with your Iceoryx-enabled config:

   ```bash
   roslaunch whycon_whycode_localization whycon.launch config_file:=/path/to/whycon_config_rs.yaml
   ```

5. Validate the stream with the lightweight SHM viewer (optional):

   ```bash
   rosrun whycon_whycode_localization shm_viewer
   ```

## Configuration

* **`config/whycon_config_*.yaml`**: Main configuration for the detector, including number of targets, marker dimensions, and tracking parameters.
* **`config/camera_intrinsics_*.yaml`**: Camera calibration parameters, including the camera matrix and distortion coefficients.

## Subscribed Topics

* The node subscribes to the camera image topic. The name of this topic is specified in the YAML configuration file (see "Configuration" section below).

* Camera calibration parameters are loaded from a dedicated YAML file specified in the main configuration file (see `system.camera_intrinsics_file` parameter).

## Published Topics

* `~poses` (`geometry_msgs/PoseArray`): An array of poses for all detected markers. See the "Coordinate Frames" section for details on the frame_id.
* `~image_out` (`sensor_msgs/Image`): An annotated image showing detected WhyCode/WhyCon markers.
* `~debug_image_out` (`sensor_msgs/Image`): A consolidated debug image showing the input image overlaid with detected ellipses, marker IDs, and other geometric debug information. Very useful for tuning detection parameters.
* `/tf` (`tf/tfMessage`): Broadcasts TF transforms. See the "Coordinate Frames" section.
* `visualization_markers` (`visualization_msgs/MarkerArray`): RViz markers for visualizing detected markers and poses.

## Configuration and Tuning

The `whycon_whycode_localization` node is configured via a main YAML file, specified by the `config_file` ROS parameter at launch (see the `whycon.launch` file). If this parameter is not set, the node attempts to load a default configuration.

**Camera calibration parameters** (intrinsics and distortion coefficients) are loaded from a separate YAML file, whose path must be specified in the main configuration YAML under `camera.config_path`.

---

### Main Configuration Structure

Below are the main sections and parameters you can set in your YAML file, with explanations and typical/Default values.

---

#### **System**

* `targets` (int): Maximum number of markers to search for and track.
  *Default: `4`*
* `frame_id` (string): Frame ID for published poses.  
  *Default: `"whycon"`*
* `world_frame_id` (string): World frame ID.  
  *Default: `"world"`*
* `input_source` (string): Where images come from: `ros` (subscribe to a ROS topic) or `iceoryx` (read from shared memory). Set to `iceoryx` for SHM mode.  
  *Typical: `"ros"`; set `"iceoryx"` for shared memory*
* `process_rate_hz` (double): Processing loop rate (Hz) when decoupled from ROS callbacks. Used for fixed-rate processing, e.g., in Iceoryx mode.  
  *Default example: `25.0`*

---

#### **Iceoryx (Shared Memory)**

These parameters configure the shared memory subscriber when `system.input_source` is `iceoryx`.

* `runtime_name` (string): Iceoryx runtime identifier for this process. Should be unique.  
  *Example: `"WhyconImageHandler"`*
* `service_instance` (string): Service instance name. Must match the publisher if your deployment uses service/instance/method.  
  *Typical: `"Camera"` (depends on your publisher)*
* `service_method` (string): Service method name. Must match the publisher if used.  
  *Typical: `"Image"` (depends on your publisher)*
* `service_event` (string): Event name to subscribe to. Must match the publisher. Tools in this repo publish RGB images on `RGB` by default.  
  *Default here: `"RGB"`*

---

#### **Detector**

* `min_size` (int): Minimum marker size in pixels.  
  *Default: `10`*
* `max_size` (int): Maximum marker size in pixels.  
  *Default: `10000`*
* `center_distance_tolerance_ratio` (double): Tolerance for center distance (ratio).  
  *Default: `0.01`*
* `center_distance_tolerance_abs` (double): Tolerance for center distance (absolute, pixels).
  *Default: `2.0`*

  The allowed offset between the centers of the inner and outer ellipses is computed as:

    $$
    \Delta x = |x_{inner} - x_{outer}| \\
    \Delta y = |y_{inner} - y_{outer}| \\
    tolerance_x = center\_distance\_tolerance\_abs + center\_distance\_tolerance\_ratio \times (outer.maxx - outer.minx) \\
    tolerance_y = center\_distance\_tolerance\_abs + center\_distance\_tolerance\_ratio \times (outer.maxy - outer.miny)
    $$

  A candidate is accepted only if:

    $$
    \Delta x \leq tolerance_x \quad \text{and} \quad \Delta y \leq tolerance_y
    $$  
* `circularity_tolerance` (double): Allowed deviation from ideal circularity(1).  
  *Default: `0.35`*
* `roundness_tolerance` (double): How much the detected shape can deviate from a perfect circle. Values closer to 0 are stricter.  
  *Default: `0.25`*
* `ratio_tolerance` (double): Tolerance for the ratio of inner to outer ellipse diameters.  
  *Default: `1.2`*
* `max_eccentricity` (double): Maximum allowed eccentricity (0 = perfect circle).  
  *Default: `0.9`*
* `inner_diameter` (double): Physical inner diameter of the marker (meters).  
* `outer_diameter` (double): Physical outer diameter of the marker (meters).  
* `outer_diameter_multiplier` (double): Multiplier applied to `outer_diameter` to scale detection size (useful for quick tuning and error correction).  
  *Default example: `1.0`*

---

#### **Identification** (WhyCode decoding)

* `enabled` (bool): Enable WhyCode ID decoding.  
  *Default: `true`*
* `id_bits` (int): Number of bits in the WhyCode ID.  
  *Default: `6`*
* `id_samples` (int): Number of samples along the circumference for decoding.  
  *Default: `720`*
* `hamming_distance` (int): Hamming distance for error correction.  
  *Default: `1`*
* `diameter_ratio_correction` (double): Adjusted ratio for inner ellipse check (ideally it should be outer/inner).  
  *Default: `0.45`*
* `variance_threshold` (double): Minimum variance difference between black and white segments for reliable ID bit detection.
  *Default: `0.35`*
* `min_marker_pixels` (int): Minimum number of pixels the outer segment must occupy for decoding. This relates to the marker's apparent size.
  *Default: `77`*

---

#### **Tracking (Lukas Kanade)**

* `max_association_dist` (double): Max distance (pixels) to associate a prediction with a detection.  
  *Default: `10.0`*
* `max_unseen_frames` (int): Frames a track can be lost before removal.  
  *Default: `10`*
* `id_switch_threshold` (int):  Num of consecutive different ID detections needed to switch the locked ID.  
  *Default: `5`*
* `pose_alpha` (double): Smoothing factor for pose (Moving average filter) (0.0 = smooth, 1.0 = fast).  
  *Default: `0.5`*
* `min_track_age` (int): Min frames a track must exist to be considered "confirmed" and published.  
  *Default: `5`*

---

#### **ROS Interfaces**

* `publish_poses` (bool): Publish pose array.  
  *Default: `true`*
* `publish_images` (bool): Publish processed images.  
  *Default: `false`*
* `publish_tf` (bool): Publish TF transforms.  
  *Default: `false`*
* `publish_debug_images` (bool): Publish consolidated debug images.  
  *Default: `false`*
* `publish_visualization_markers` (bool): Publish `visualization_msgs/MarkerArray` for RViz visualization of detections/poses.  
  *Default: `false`*

* `tf_frame_prefix` (string): Prefix for TF frames.  
  *Default: `"whycode_"`*
* `parent_frame_id` (string): Parent frame for TF transforms.  
  *Default: `"camera_link"`*

* `input_queue_size` (int): Input image queue size.  
  *Default: `1`*
* `image_encoding` (string): Expected image encoding (e.g., `rgb8`).  
  *Default: `"rgb8"`*

* `image_input_topic` (string): Input image topic.  
  *Default: `"/camera/color/image_raw"`*
* `poses_output_topic` (string): Output topic for marker poses.  
  *Default: `"/whycon/poses"`*
* `image_output_topic` (string): Output topic for processed images.  
  *Default: `"/whycon/image_out"`*
* `debug_images_topic` (string): Output topic for debug images.  
  *Default: `"/whycon/debug_images"`*
* `visualization_markers_topic` (string): Output topic for visualization markers.  
  *Default: `"/whycon/visualization_markers"`*
* `detection_enabled_service_name` (string): Service name to enable/disable detection.  
  *Default: `"set_detection_enabled"`*

---

#### **Camera**

* `package_name` (string): Package containing camera config.  
  *Default: `"whycon_whycode_localization"`*
* `config_path` (string): Relative path to camera config file (intrinsics YAML).  
  *Default: `"config/camera_intrinsics_rs.yaml"`*
* `image_width` (int): Image width in pixels.  
  *Default: `640`*
* `image_height` (int): Image height in pixels.  
  *Default: `480`*

---

### Camera Intrinsics YAML File

This file (e.g., `config/camera_intrinsics_rs.yaml`) contains the standard ROS camera calibration parameters. It **must** be accurate for correct 3D localization.
Example structure:

```yaml
image_width: 640
image_height: 480
camera_name: head_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, t1, t2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
```

* **CRITICAL Parameters**: `camera_matrix.data` (fx, fy, cx, cy) and `distortion_coefficients.data`.
* Ensure `image_width` and `image_height` match your camera's resolution.

## Further docs and test assets

* Detailed documentation: Bookstack — [WhyCode Tags](https://bookstack.addverb.com/books/releases/page/whycode-tags)
* Gazebo testing: WhyCode marker worlds and models — [fiducial-gazebo-sim](https://github.com/addverb-sandbox/fiducial-gazebo-sim)  
  This repository is also available locally under `whycode_sim/` for quick simulation.
* Sample bag (AMR 200, shop floor):  
  [SharePoint link](https://addverbtech-my.sharepoint.com/:f:/g/personal/gourav_kumar_addverb_com/Ei0gJ1sGWPtKhzgSBNua8U4BoHmDA9xyMq4Ns1sj6wOPFw?e=vYWEJl)  
  Use it with the ROS bag publisher nodelet (ROS path) or `bag_to_shm` (shared memory path) as documented in the Tools and Usage sections.

## Known Limitations

* **Lighting Conditions:** Detection performance is sensitive to illumination. Strong glare, deep shadows, or very low light can significantly hinder detection. Uniform, diffuse lighting is ideal.
* **Marker Occlusion:** Markers must be clearly and fully visible. Partial occlusion will likely lead to detection failure or inaccurate pose.
* **Motion Blur:** Fast camera or marker motion can cause image blur, degrading detection accuracy and reliability.
* **Computational Load:** Processing very high-resolution images or attempting to track a very large number of targets can be computationally intensive.
* **WhyCode Decoding:** Requires the marker to be reasonably large, clear, and well-lit in the image. Small, blurry, or poorly contrasted markers may not be decoded correctly or at all.
* **Planarity Assumption:** The system assumes markers are planar. Non-planar markers will result in inaccurate pose estimation.

## Troubleshooting

* **No markers detected:**
  * Adjust lighting conditions. Try to reduce glare and ensure markers are adequately illuminated.
  * Ensure `detector.outer_diameter` is correctly set (though this primarily affects pose accuracy, not initial detection).
  * If markers appear distorted or oddly shaped in the image, try relaxing some detector parameters (e.g., circularity, eccentricity constraints) in your YAML configuration.
* **Incorrect marker poses (e.g., wrong distance or orientation):**
  * **Double-check `detector.outer_diameter`.** This is the most common cause of scaling errors in the estimated pose.
  * Verify the camera calibration in your `camera_intrinsics_file` thoroughly. Even small errors can lead to significant pose inaccuracies. Ensure the file is correctly formatted and the values precisely match your camera.
  * Ensure the `frame_id` used in your camera intrinsics file matches the `frame_id` of the incoming images if TF consistency is important.
* **Incorrect or no marker IDs:**
  * Ensure `identification.enabled` is `true` in the configuration.
  * The marker needs to be large enough in the image. Try increasing `identification.min_marker_pixels` if decoding is noisy or failing for markers that appear small.
  * Check for good contrast and clarity of the marker's black and white pattern in the image. Lighting is key.
  * Ensure the physical marker pattern matches the expected WhyCode bit length (e.g., `id_bits` in configuration).

## References

* [jiriUlr/whycon-ros](https://github.com/jiriUlr/whycon-ros/tree/master)
* [lrse/whycon](https://github.com/lrse/whycon)
* [gestom/whycon-orig](https://github.com/gestom/whycon-orig)
