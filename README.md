# whycode_vision

A ROS 2 detector for WhyCon and WhyCode markers: black rings on white, with an ID coded in the
centre for WhyCode. In every camera frame it finds each marker, decodes its ID, estimates its 6-DoF
pose from the ring's outline and tracks it to the next frame, in about 1.3 ms of one CPU core at
1280x720. It runs as a node or as a composable component.

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
  the right ID for WhyCode. An ID is decoded from about 9 px across and the tracker keeps it as the
  marker shrinks, so here it lasts to the full range. Plain WhyCon falls short of WhyCode as its
  white centre is smaller: 17% of the marker's area, against WhyCode's 24% with the teeth.
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

## Build

Needs ROS 2 Jazzy, OpenCV 4, yaml-cpp, [Simd](https://github.com/ermig1979/Simd) and the
header-only [xsimd](https://github.com/xtensor-stack/xsimd), which the binarisation and
localisation kernels use.

```bash
colcon build --symlink-install --packages-select whycode_vision \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`-DBUILD_TRIANGULATION=OFF` and `-DBUILD_BENCHMARKS=OFF` leave out the triangulation nodes and the
benchmarks, both built by default. The release flags include `-march=native`, so build on the
machine that runs the detector.

<details>
<summary>Installing Simd and xsimd from source</summary>

```bash
git clone https://github.com/ermig1979/Simd.git && cd Simd && mkdir -p build && cd build
cmake ../prj/cmake -DSIMD_TOOLCHAIN="" -DSIMD_TARGET="" -DSIMD_AVX512=ON -DSIMD_AVX512VNNI=ON \
  -DSIMD_AMXBF16=ON -DSIMD_TEST=ON -DSIMD_INFO=ON -DSIMD_PERF=OFF -DSIMD_SHARED=ON \
  -DSIMD_GET_VERSION=ON -DSIMD_SYNET=ON -DSIMD_INT8_DEBUG=OFF -DSIMD_HIDE=OFF -DSIMD_RUNTIME=ON \
  -DSIMD_OPENCV=ON -DSIMD_INSTALL=ON -DSIMD_UNINSTALL=ON -DSIMD_PYTHON=ON
make -j"$(nproc)" && sudo make install
```

```bash
git clone https://github.com/xtensor-stack/xsimd.git && cd xsimd && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && make -j"$(nproc)" && sudo make install
```

</details>

## Run

```bash
ros2 launch whycode_vision whycon.launch.py                          # as a node
ros2 launch whycode_vision whycon.launch.py use_composition:=true     # in whycon_container
ros2 launch whycode_vision whycon_debug.launch.py bag:=/root/data/whycode_1lap
```

`whycon.launch.py` takes `config_file` (default `config/whycon_config_sim.yaml`),
`use_composition` and `image_view`, a window on the annotated image. Load the camera driver into
the same container with its `use_intra_process_comms` on, and frames arrive without DDS copies.

`whycon_debug.launch.py` runs the same detector with every output on, in a copy of the
configuration, and opens RViz laid out for a 3840x1600 screen: the detections over a bag's ground
truth (from whycode_sim), the annotated image and the debug panels. `bag:=` plays a bag on its own
clock, in a loop. It also takes `rate`, `loop`, `use_sim_time`, `ground_truth`, `marker_size_m` and
`rviz`, and its `config_file` defaults to `whycon_config_isaac.yaml`.

## Configuration

Each setup is a YAML file in `config/` that names a camera intrinsics file.
[docs/configuration/README.md](docs/configuration/README.md) explains every parameter, the values
derived at startup (`auto`) and how to tune the checks. The loader has no defaults, so a file sets
every key. For a new camera or marker, set:

| Parameter | To |
|---|---|
| `camera.config_path` | intrinsics calibrated at the resolution the camera streams |
| `ros_interface.image_input_topic`, `image_encoding` | the camera's raw image topic and its encoding |
| `detector.outer_diameter` | the printed ring's outer diameter in metres; the pose scales with it |
| `identification.enabled`, `id_bits` | WhyCode (`true`, the generator's bit count) or plain WhyCon (`false`) |
| `system.targets` | at least the number of markers in view at once |
| `ros_interface.parent_frame_id` | the camera's body frame (X forward, Y left, Z up) in your TF tree |

## Topics

| Topic | Type | Carries |
|---|---|---|
| `image_input_topic` (in) | `sensor_msgs/Image` | raw frames; another size or encoding is dropped with a logged error |
| `/whycon/poses` | `whycode_vision/WhyCodePoseArray` | each marker's pose, ID (`id_valid`), track and `orientation_confidence` |
| `/whycon/image_out` | `sensor_msgs/Image` | the annotated frame |
| `/whycon/debug_images/{binary,candidates,markers}` | `sensor_msgs/Image` | tuning panels, drawn only while subscribed (`publish_debug_images`) |
| `/tf` | `tf2_msgs/TFMessage` | `<tf_frame_prefix><ID>` under `parent_frame_id`, or `<prefix>track_<n>` without an ID |
| `/whycon/visualization_markers` | `visualization_msgs/MarkerArray` | the markers as RViz discs |

* `orientation_confidence`: a tilted circle fits two orientations, mirror images about the line of
  sight. It is the probability that the published one is right; 0.5 means undecided, typical below
  about 20 px.
* `binary` shows the frame thresholded at the search's level. `candidates` paints every segment the
  search examined by the check that rejected it, with its value against the limit; markers are
  green. `markers` zooms on each marker: its outline, the code ring's samples and bits, both pose
  solutions and every ID check that failed.

## Markers

```bash
ros2 run whycode_vision whycon-id-gen -- -l        # a plain WhyCon marker
ros2 run whycode_vision whycon-id-gen -- 6         # WhyCode markers with 6 bits
ros2 run whycode_vision whycon-id-gen -- -d 2 8    # 8 bits, Hamming distance 2
```

Print each ring with a white border around it; `-h` lists the options.

## Triangulation

`two_marker_whycode_triangulation_node` and `four_marker_whycode_triangulation_node` read
`/whycon/poses` and publish a `geometry_msgs/PoseStamped`, a TF frame and optionally
`nav_msgs/Odometry`: one from two WhyCode markers a known distance apart, the other from a rig of
four. Their marker IDs and distances are ROS parameters, declared in `src/triangulate/`.

## Benchmarks

`src/benchmarks` holds timing and self-check programs for the per-frame kernels;
[src/benchmarks/README.md](src/benchmarks/README.md) says what each measures, how to run it, and
its latest results.

## Architecture

One camera frame through the detector, a stage per panel with the class that runs it. The first
four panels replay the detector's own steps on a coarse pixel grid: the search meets clutter first,
then the scanline fill takes the ring and its centre run by run, in the order the code does.
[docs/architecture.md](docs/architecture.md) covers the runtime, composition and triangulation.

![A camera frame through the detector](docs/architecture.svg)

## Limitations

* **Steep views.** A marker more than about 64 deg off head-on is lost: its outline passes
  `detector.max_eccentricity` (0.9). Raising it reaches further round at the cost of more false
  circles; AprilTag 3 holds tags to about 70 deg.
* **Orientation.** Position is accurate, 0.2% of the range at 2-10 m; orientation less so. A small
  or face-on ring fits two mirror-image poses, and past about 5 m a 0.20 m marker's normal is off by
  5-10 deg at 1280x720. `orientation_confidence` says how sure each pose is.
* **IDs need pixels.** An ID is decoded once the ring has `identification.min_marker_pixels` (75,
  about 9 px across), and a tracked marker keeps it as it shrinks. A marker first seen further away
  is found at once but identified only as it comes closer.
* **Ring-like clutter.** Any black ring on white, such as a printed 0, looks like a WhyCon marker.
  WhyCode publishes it without an ID; plain WhyCon cannot tell it apart.
* **Whole rings only.** A ring cut by the image edge, partly hidden, or touching a dark background
  without a white border is not found.
* **Measured on rendered frames.** The numbers above have no motion blur, sensor noise or glare.
  The threshold follows the last marker found, so slow changes in light are fine, but glare or a
  hard shadow across a ring breaks it.

## Troubleshooting

* **Nothing is detected.**
  * A logged `Dropping WxH ... frame` means the camera's frame size or encoding differs from the
    configuration's (`camera.config_path`, `ros_interface.image_encoding`).
  * Run `whycon_debug.launch.py` and look at the candidates panel: it shows which check rejected
    each segment, and by how much.
  * Give the printed ring a white border. If more markers are in view than `system.targets`,
    raise it.
* **The distance is wrong.** Set `detector.outer_diameter` to the printed ring's outer diameter in
  metres, and use intrinsics calibrated at the resolution the camera streams. The startup log prints
  the marker geometry and the camera the detector's limits were derived from.
* **No IDs, or wrong ones.** Check `identification.enabled`, and that `id_bits` matches the
  generator's. Lowering `min_marker_pixels` decodes smaller markers, but below 75, 1-5% of reads
  come out wrong. The markers panel shows each marker's code samples, its bits and the ID checks
  that failed.
* **The pose is in the wrong frame.** Poses and TF are in `ros_interface.parent_frame_id`, the
  camera's body frame (X forward, Y left, Z up), which must exist in your TF tree.

## References

* [jiriUlr/whycon-ros](https://github.com/jiriUlr/whycon-ros/tree/master)
* [lrse/whycon](https://github.com/lrse/whycon)
* [gestom/whycon-orig](https://github.com/gestom/whycon-orig)
* [Adyansh04/whycode-sim](https://github.com/Adyansh04/whycode-sim): the Isaac Sim warehouse these
  numbers come from; its `jazzy-harmonic` branch has the earlier Gazebo worlds
