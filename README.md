# WhyCon/WhyCode Localization ROS Package

## Overview

`whycode_vision` is a ROS2 package for detecting and localizing circular WhyCon/WhyCode markers and decoding embedded WhyCode IDs. It estimates the 3D pose of these markers relative to a camera.

## Features

* **Multi-Marker Detection**: Detects multiple circular black and white WhyCon markers simultaneously.
* **WhyCode Decoding**: Decodes WhyCode IDs embedded within the markers.
* **Pose Estimation**: Estimates the 6-DOF pose (position and orientation) of each detected marker.
* **ROS2 Deployment Options**: Run as a standalone node or as a composable component.
* **ID Tracking and Stabilization**: Reduces ID flicker using a hysteresis-based ID stabilizer and maintains tracks over time with lightweight 2D tracking.
* **Modular Build Options**: Enable or disable triangulation and benchmarks using CMake options.

## Demo

A lap of the warehouse loop in Isaac Sim, replayed through the detector with every debug output on:
the annotated image, the detections over the ground truth in 3D, the binary, candidates and marker
panels, and Isaac Sim's view of the robot. Recorded with
[whycode-bench](https://github.com/Adyansh04/whycode-bench)'s `scripts/demos/whycode.sh replay`.

<!-- DEMO VIDEO: replace this line with the uploaded mp4's https://github.com/user-attachments/assets/... link -->

## Detection range and CPU

| D435 colour camera | WhyCode 0.20 m | WhyCode 0.146 m | WhyCon 0.20 m | WhyCon 0.146 m | WhyCode CPU per frame |
|:---|---:|---:|---:|---:|---:|
| 640x480, HFOV 55.6 deg | 16.5 m | 12.5 m | 14.5 m | 10.5 m | 0.75 ms (2.3%) |
| 1280x720, HFOV 70.3 deg | 23.5 m | 18.5 m | 20.0 m | 15.0 m | 1.34 ms (4.0%) |
| 1920x1080, HFOV 70.3 deg | 34.0 m | 27.5 m | 31.5 m | 24.0 m | 2.42 ms (7.3%) |

* Measured in Isaac Sim with [whycode-bench](https://github.com/Adyansh04/whycode-bench): the
  camera looks back at two markers while the robot drives 41.5 m straight away from them. The
  sizes are the marker's outer diameter.
* Range: the distance out to which the detector finds the marker in at least 90% of frames, with
  the right ID for WhyCode. Plain WhyCon falls short of WhyCode as its white centre is smaller:
  17% of the marker's area, against WhyCode's 24% with the teeth.
* CPU: the detector process's CPU time per frame, every thread, with only `/whycon/poses`
  published, on an Intel i9-14900HX with nothing else running; in brackets, the share of one core
  at 30 Hz. Repeat runs agree within about 5%.
* A rendered scene has no noise, blur or glare; expect less range from a real camera.

## Compared with AprilTag

![WhyCode against AprilTag: range and CPU at 1280x720](docs/media/benchmark_summary.svg)

The same runs on AprilTag 36h11 tags whose black square is as wide as the ring, so it covers 27%
more than the disc, with AprilTag 3 through `apriltag_ros` at its default half-resolution
decimation and at full resolution, NVIDIA's GPU detector in Isaac ROS, and OpenCV's ArUco
module.

| 1280x720 | WhyCode | WhyCon | AprilTag 3 | AprilTag 3, full res | Isaac ROS (GPU) | OpenCV ArUco |
|:---|---:|---:|---:|---:|---:|---:|
| Range, 0.20 m marker | 23.5 m | 20.0 m | 10.0 m | 14.5 m | 5.5 m | 5.5 m |
| Range, 0.146 m marker | 18.5 m | 15.0 m | 6.0 m | 9.5 m | 4.0 m | 4.0 m |
| Range error at 2-10 m, median | 0.20% | 0.33% | 1.50% | 1.51% | 5.30% | 1.17% |
| Steepest view, 90% of frames | 60 deg | 60 deg | 65 deg | 70 deg | 60 deg | - |
| Warehouse lap: markers found | 99.3% | 99.5% | 96.0% | 100.0% | 67.1% | 72.2% |
| CPU per frame | 1.3 ms | 1.2 ms | 9.0 ms | 23.5 ms | 3.5 ms + GPU | 16.9 ms |

* No detector reported a wrong ID, or an ID on anything but its marker, in any run.
* WhyCode and WhyCon also find other black rings on white, such as the 0s on the aisle signs:
  WhyCode publishes them without an ID (`id_valid` false), plain WhyCon cannot tell them apart.
* The viewing angle counts the 2 and 4 m circles, within every detector's range. WhyCode stops
  where a circle seen off head-on reaches `max_eccentricity` (0.9, 64 deg); AprilTag keeps tags
  further round. OpenCV ArUco misses half the head-on frames even on the 2 m circle.
* The AprilTag detectors read a tag about half a pixel narrower than it is (two pixels for Isaac
  ROS), so their range error grows as the tag shrinks in the image.
* At 640x480 and 1920x1080, and per marker size:
  [detection against distance](docs/media/range.png),
  [pose error against distance](docs/media/accuracy.png),
  [detection against viewing angle](docs/media/angle.png).

![The benchmark's three scenes, from above](docs/media/benchmark_scenes.svg)

## Tools

Utility tools live in `src/tools`. The ROS2 port currently exposes the marker generator below; other tools in this folder are not ported yet and are skipped by default.

### WhyCode Marker Generator (`whycon-id-gen`)

Generates printable WhyCon/WhyCode markers for lab testing and simulation assets.

* Help: `ros2 run whycode_vision whycon-id-gen -h`
* Examples:
  * Basic WhyCon marker: `ros2 run whycode_vision whycon-id-gen -- -l`
  * WhyCode with 6 bits: `ros2 run whycode_vision whycon-id-gen -- 6`
  * WhyCode with 8 bits, Hamming distance 2: `ros2 run whycode_vision whycon-id-gen -- -d 2 8`

## Triangulation nodes (src/triangulate)

These ROS2 nodes estimate poses using known marker layouts. They are built when `BUILD_TRIANGULATION=ON` (default).

### two_marker_whycode_triangulation_node

* Purpose: Compute a pose from two WhyCode markers with a known separation.
* Input: `whycode_vision/msg/WhyCodePoseArray` (from the main WhyCon node).
* Output: `geometry_msgs/msg/PoseStamped` and TF (broadcast), plus optional `nav_msgs/msg/Odometry` if enabled.
* Notes: Expects two specific marker IDs; includes utilities to compute roll, pitch, and yaw from the camera plane.

### four_marker_whycode_triangulation_node

* Purpose: Hierarchical triangulation using four WhyCode markers to improve robustness.
* Input: `whycode_vision/msg/WhyCodePoseArray`.
* Output: `geometry_msgs/msg/PoseStamped` and TF (broadcast), plus optional `nav_msgs/msg/Odometry`.
* Notes: Designed for rigs with four arranged WhyCode markers; see `src/triangulate/four_marker_whycode_triangulation.cpp` for parameter hints.

Where these fit:

* Run alongside the main detector node. Point their input topic to the detector's published WhyCode poses.
* Use TF to integrate the estimated pose into your robot's frame tree.

## Benchmarks (src/benchmarks)

Built when `BUILD_BENCHMARKS=ON` (default). Requires xsimd. Timing and self-check programs for
the per-frame kernels; [src/benchmarks/README.md](src/benchmarks/README.md) says what each
measures, how to run it, and its latest results.

## Architecture

One camera frame through the detector, a stage per panel with the class that runs it. The first
four panels replay the detector's own steps on a coarse pixel grid: the search meets clutter first,
then the scanline fill takes the ring and its centre run by run, in the order the code does.
[docs/architecture.md](docs/architecture.md) covers the runtime, composition and triangulation.

![A camera frame through the detector](docs/architecture.svg)

## Documentation

* [docs/architecture.md](docs/architecture.md) - System architecture and module overview
* [docs/configuration/README.md](docs/configuration/README.md) - Parameter reference and tuning

## Dependencies

### System Dependencies

* OpenCV 4.2
* yaml-cpp
* [SIMD](https://github.com/ermig1979/Simd)
* [xSimd](https://github.com/xtensor-stack/xsimd)

### ROS Dependencies

* `rclcpp`, `rclcpp_components`
* `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
* `cv_bridge`
* `tf2`, `tf2_ros`, `tf2_geometry_msgs`
* `std_srvs`, `rosidl_default_runtime`

## Building the Package

The build is modular and controlled via CMake options exposed to ament. Defaults are shown in parentheses.

Build options:

* BUILD_TOOLS (OFF): Build utility tools under `src/tools` (currently skipped in the ROS2 port).
* BUILD_TRIANGULATION (ON): Build triangulation nodes under `src/triangulate`.
  * Executables: `two_marker_whycode_triangulation_node`, `four_marker_whycode_triangulation_node`.
* BUILD_BENCHMARKS (ON): Build benchmarks under `src/benchmarks` (requires xsimd).
* DISABLE_ROS (OFF): Not supported in the ROS2 port yet.

Typical builds:

1. Default (recommended)

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

1. Minimal runtime (no tools, no benchmarks, no triangulation)

```bash
colcon build --symlink-install --cmake-args -DBUILD_TOOLS=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TRIANGULATION=OFF
```

1. Only triangulation nodes

```bash
colcon build --symlink-install --cmake-args -DBUILD_TOOLS=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TRIANGULATION=ON
```

Optimization notes:

* The build enables high-performance flags by default (e.g., `-O3`, vectorization, and `-mavx2` when supported).
* Benchmarks make use of xsimd; ensure xsimd is installed if `BUILD_BENCHMARKS=ON`.

### Install prerequisite libraries (Simd, xsimd)

These projects depend on Simd and xsimd. Expand the sections below to see installation commands.

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

## Usage

The primary way to run the package is through the `whycon.launch.py` file, which can start the system as a standard node or as a composable node.

### Running with composition

```bash
ros2 launch whycode_vision whycon.launch.py use_composition:=true
```

The image subscription accepts intra-process messages. It pays off when the camera driver is loaded
into the same container (`whycon_container`) with `use_intra_process_comms` on the driver: frames
then arrive without DDS serialisation, fragmentation or copies.

### Running as a standalone node

```bash
ros2 launch whycode_vision whycon.launch.py use_composition:=false
```

### Launch Arguments

* `use_composition` (bool, default: `false`): If true, runs as a composable node in a component container.
* `config_file` (string, default: `.../config/whycon_config_sim.yaml`): Path to the main configuration file.
* `image_view` (bool, default: `false`): If true, launches an `image_view` node to display the annotated image output.

### Debugging and tuning

```bash
ros2 launch whycode_vision whycon_debug.launch.py bag:=/root/data/whycode_1lap
```

The same detector with every output switched on (poses, annotated image, TF, RViz markers and the
debug panels; the YAML file itself is not changed), and RViz laid out for a full-screen 3840x1600
window: the detected markers next to the bag's ground truth in 3D, the annotated image and the
three debug panels described under Published Topics. `bag:=` plays a bag on its own clock, in a
loop; without it, the detector reads the configured camera topic. The ground truth needs
`whycode_sim`, whose `ground_truth_viz_node` renders `/marker_ground_truth`.

Arguments: those of `whycon.launch.py` (here `config_file` defaults to
`whycon_config_isaac.yaml`), plus `bag`, `rate`, `loop`, `use_sim_time` (`auto`: true when a bag is
played), `ground_truth`, `marker_size_m` and `rviz`.

## Configuration

* **`config/whycon_config_*.yaml`**: Main configuration for the detector, including number of targets, marker dimensions, and tracking parameters.
* **`config/camera_intrinsics_*.yaml`**: Camera calibration parameters, including the camera matrix and distortion coefficients.

[docs/configuration/README.md](docs/configuration/README.md) describes every parameter, how the
`auto` values are derived from the camera and the marker, and how to tune the checks. The loader has
no defaults: a configuration must set every key. A configuration detects one marker type: WhyCode
with `identification.enabled`, WhyCon without.

## Subscribed Topics

* The node subscribes to the camera image topic (raw `sensor_msgs/msg/Image`, in the encoding `ros_interface.image_encoding` names; frames of another size or encoding are dropped). The name of this topic is specified in the YAML configuration file.
* Camera calibration parameters are loaded from a dedicated YAML file specified in the main configuration file (see the `camera.config_path` parameter).

## Published Topics

* `/whycon/poses` (`whycode_vision/msg/WhyCodePoseArray`): Poses for all detected markers. A tilted
  circle fits two orientations, mirror images about the line of sight; `orientation_confidence` is the
  probability that the published one is right (0.5: undecided, typical below ~20 px).
* `/whycon/image_out` (`sensor_msgs/msg/Image`): Annotated image showing detected WhyCode/WhyCon markers.
* `/whycon/debug_images/{binary,candidates,markers}` (`sensor_msgs/msg/Image`, rgb8): debug panels
  for tuning, published when `ros_interface.publish_debug_images` is true (false in every config).
  Each panel is drawn only while something subscribes to it, so they cost nothing otherwise.
  * `binary`: the frame binarised at the first searching detector's threshold, with every
    detector's threshold.
  * `candidates`: every segment the search examined, painted by the check that dropped it (key in
    the corner) and labelled with its value against the limit; markers are green, with their fitted
    outlines.
  * `markers`: a zoomed tile per marker with its outline, the code ring's samples by the bit each
    reads, every segment's bit, both pose solutions' centres (the favoured one green), and the ID,
    the code, the log-odds between the solutions and every ID check that failed.
* `/tf` (`tf2_msgs/msg/TFMessage`): A frame per marker under `ros_interface.parent_frame_id`, stamped
  with its image: `<tf_frame_prefix><ID>` (e.g. `whycode_3`), or `<tf_frame_prefix>track_<track id>`
  while the marker has no ID.
* `/whycon/visualization_markers` (`visualization_msgs/msg/MarkerArray`): RViz markers for visualizing detections and poses.

## Further docs and test assets

* Gazebo testing: WhyCode marker worlds and models - [Whycode Sim](https://github.com/Adyansh04/whycode-sim/tree/jazzy-harmonic)

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
