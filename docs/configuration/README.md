## Configuration and Tuning

The `whycode_vision` node is configured via a main YAML file, specified by the `config_file` launch argument (see `whycon.launch.py`). If this parameter is not set, the node attempts to load a default configuration.

**Camera calibration parameters** (intrinsics and distortion coefficients) are loaded from a separate YAML file, whose path must be specified in the main configuration YAML under `camera.config_path`.

---

### Main Configuration Structure

Below are the main sections and parameters you can set in your YAML file, with explanations and typical/default values.

---

#### **System**

* `targets` (int): Maximum number of markers to search for and track.
  *Default: `4`*
* `frame_id` (string): Frame ID for published poses.
  *Default: `"whycon"`*
* `world_frame_id` (string): World frame ID.
  *Default: `"world"`*
* `process_rate_hz` (double): Processing loop rate (Hz) when decoupled from ROS callbacks.
  *Default example: `25.0`*

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

* `circularity_tolerance` (double): Allowed deviation from ideal circularity (1).
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

#### **Tracking (EKF)**

* `max_association_dist` (double): Max distance (pixels) to associate a prediction with a detection.
  *Default: `10.0`*
* `max_unseen_frames` (int): Frames a track can be lost before removal.
  *Default: `10`*
* `id_switch_threshold` (int): Num of consecutive different ID detections needed to switch the locked ID.
  *Default: `5`*
* `pose_alpha` (double): Smoothing factor for pose (moving average filter) (0.0 = smooth, 1.0 = fast).
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
  *Default: `"whycode_vision"`*
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
