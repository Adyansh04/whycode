# Configuration and tuning

The node reads one YAML file, named by the `config_file` launch argument (`whycon.launch.py`
defaults to `config/whycon_config_sim.yaml`; `whycon_debug.launch.py` runs a copy of the chosen file
with every output switched on). Camera intrinsics come from a second file, named by
`camera.config_path`. The loader has no defaults: a configuration sets every key, and a missing key
stops the node with its name. `whycon_config_isaac.yaml` is tuned on the Isaac Sim bags against
ground truth.

A configuration detects one marker type: WhyCode with `identification.enabled`, WhyCon without.

## Values derived at startup

Five keys take `auto`. The node derives them once at startup and logs the result; a number
overrides the derivation.

| Key | `auto` | From |
|---|---|---|
| `detector.max_size` | image width x height | `camera.image_width`, `camera.image_height` |
| `detector.inner_diameter` | 0.6 x `outer_diameter` (WhyCon 0.41 x) | the `whycode_gen` design |
| `detector.center_distance_tolerance_ratio` | the teeth's widest offset + 0.03 | `id_bits`, the centre's size |
| `tracking.max_association_dist` | 40 px x s | focal length, `camera.frame_rate` |
| `tracking.max_unseen_frames` | round(10 x frame rate / 30), at least 1 | `camera.frame_rate` |

**Camera.** A marker's image motion per frame grows with the focal length in pixels and shrinks
with the frame rate. The motion limits were tuned on the Isaac Sim camera (fx 762.7 px, 30 Hz), so
they scale by

    s = (fx / 762.7) x (30 / frame rate)

with fx from the intrinsics file. Besides the association gate, s scales the search window margin
of `full_scan_interval` (20 px x s), a new track's position uncertainty (by s^2) and the tracker's
acceleration noise (by s x 30 / frame rate). `max_unseen_frames` keeps a third of a second.

**Marker.** With identification on, the marker is a WhyCode marker as `whycode_gen` draws it: a
black ring around a white centre of 0.6 of the outer diameter, whose outer band, from 0.343 to 0.6,
holds the teeth. Each bit is one black and one white segment (Manchester coding), so half of the
band is black for every ID and tooth count. The area checks expect the white area of a disc of
diameter

    d = D x sqrt((0.343^2 + c^2) / 2),    D = outer_diameter, c = inner_diameter / D

which is 0.0977 m for a 0.20 m marker. A WhyCon marker's centre is a plain white disc (0.41 of the
outer diameter from `whycode_gen -l`), so d = `inner_diameter`.

The teeth also move the white centre's bounding box off the ring's. At most two black segments
meet, so along an axis the white can reach only max(0.343, c cos(pi/n)) of the outer radius on one
side and c on the other, n being `id_bits`. Over the ring's box that is

    (c - max(0.343, c cos(pi/n))) / 4

0.064 for 3 bits, 0.044 for 4, 0.029 for 5, 0.020 for 6, 0.015 for 7 and 0.011 for 8. Perspective
adds up to about 0.03 for markers up to a quarter of the focal length across.

The other pixel limits and the shape tolerances hold at any camera resolution, and the tolerances
for any marker design: a new camera or marker needs no retuning of them.

## How a candidate is checked

The frame is binarised and every black segment flood-filled and tried as a marker's ring, with the
white segment at the ring's centre as its centre. A marker passes, in order:

1. **Size:** each segment has more than `min_size` and fewer than `max_size` pixels.
2. **Roundness** (ring): box width x box height x pi (1 - q^2) / 4 / ring pixels, with q = d / D,
   lies within `roundness_tolerance` of 1.
3. **Area ratio:** ring pixels / centre pixels x q^2 / (1 - q^2) lies within `ratio_tolerance` of 1.
4. **Concentricity:** along x and along y, the centres of the two bounding boxes are at most
   `center_distance_tolerance_abs` + `center_distance_tolerance_ratio` x the ring's box size apart.
5. **Shape:** ring and centre together are fitted with an ellipse of semi-axes a and b. Its area,
   pi a b, over their pixel count lies within `circularity_tolerance` of 1, and its eccentricity,
   sqrt(1 - (b/a)^2), is below `max_eccentricity`.

The `candidates` debug panel paints every examined segment by the check that dropped it, with its
value against the limit. Run `whycon_debug.launch.py`, then loosen the check that drops real markers
or tighten the one that lets clutter through.

## Parameters

Values in parentheses are the Isaac configuration's.

### `system`

* `targets` (int): the most markers searched for per frame, one detector each (6). A frame with
  fewer markers in view ends in one whole-image search that finds nothing, so set it to the most
  markers expected in view at once.
* `frame_id`, `world_frame_id` (string): not read by the node.

### `detector`

* `min_size` (int, px): smallest segment tried as a ring or a centre (10). The centre is the
  smaller, about 0.19 D^2 pixels for a WhyCode marker D pixels across (0.13 D^2 for WhyCon), so 10
  admits markers from about 7 px. Raising it skips small clutter at the cost of range.
* `max_size` (int or `auto`, px): largest segment tried. `auto`, the image area, never limits;
  lower it only to ignore markers closer than a given size.
* `center_distance_tolerance_ratio` (double or `auto`) and `center_distance_tolerance_abs` (double,
  px): the concentricity check (`auto`, 2.0). The absolute part covers the integer box centres.
  `auto` covers 3 to 8 teeth with room for perspective; raise it for markers seen wider than a
  quarter of the focal length.
* `roundness_tolerance` (double): a head-on marker measures 1 (0.35). An ellipse of axis ratio k
  measures up to (1 + k^2) / 2k when its axes run diagonally in the image, so accepting every
  rotation up to `max_eccentricity` e takes (1 + k^2) / 2k - 1, with k = sqrt(1 - e^2): 0.365 for
  0.9.
* `circularity_tolerance` (double): the fit is exact, and true markers sit within about 0.1 of 1
  (0.15). Clutter, merged blobs and occluded markers fail it.
* `ratio_tolerance` (double): true markers measure 0.77-1.15 of the expected area ratio on the
  Isaac bag, small markers lowest (0.4). A ratio far from 1 for every marker means a wrong
  `inner_diameter` or marker type.
* `max_eccentricity` (double): 0.9 is an axis ratio of 0.44, a marker about 64 degrees from
  head-on (0.9). Raise it toward 1 to accept more oblique views.
* `inner_diameter` (double or `auto`, m): the printed white centre, teeth included (`auto`). Set a
  number only for markers printed with another centre.
* `outer_diameter` (double, m): the outer diameter of the black ring (0.20). It sets the pose's
  scale, so a 1% error is a 1% range error. In the sim it must equal `marker.size_m` in
  `whycode_sim/config/scene.yaml`.
* `image_gamma` (double): encoding gamma of the grey levels (2.2). The binarisation threshold sits
  halfway between the ring's and the centre's grey in linear light: 2.2 for sRGB images (most
  cameras, Isaac renders), 1.0 for linear ones. A wrong value moves every edge, which lengthens or
  shortens ranges by about 1% and skews small markers' area ratios.
* `full_scan_interval` (int): frames between whole-image searches (1). At 1 every frame is searched
  in full. Above 1 the frames between search only near known markers, within their last motion plus
  20 px x s, which is cheaper, but a new marker can appear up to that many frames late; a marker
  lost from its window forces a full search on the next frame.

### `identification`

* `enabled` (bool): decode WhyCode IDs and let the code choose between a marker's two pose
  solutions (true). Off, the markers are WhyCon markers: no IDs, and the centre picks the solution.
  It also sets the marker type for the `auto` values.
* `id_bits` (int): the teeth, or bits, of the printed markers: `whycode_gen`'s first argument (6).
* `id_samples` (int): samples read around the code ring (720). A multiple of 2 x `id_bits` gives
  every segment the same number; 720 suits every tooth count up to 10 but 7.
* `hamming_distance` (int): the smallest Hamming distance between valid codes (1). It must match the
  set the markers were generated with, since it numbers the IDs.
* `diameter_ratio_correction` (double): radius of the ring the code is read on, over the outer
  radius (0.52). The teeth span 0.343-0.6, and 0.52 reads small markers most reliably.
* `ambiguity_noise_px` (double, px): edge noise of the code (0.3). It turns each frame's comparison
  of the two pose solutions into log-odds, which the tracker sums per marker; 0.3 is calibrated on
  the sim. Raise it for a noisier camera, which makes each frame count for less.
* `min_marker_pixels` (int, px): ring pixels below which no ID is reported (75). Reads whose
  Manchester pairs disagree are rejected at any size, but below 75 pixels some wrong reads still
  pass. More teeth need larger markers before a code reads at all.

### `tracking`

* `max_association_dist` (double or `auto`, px): the farthest a detection may lie from a track's
  prediction and still join it (`auto`: 40 px on the Isaac camera, where markers move up to 37 px
  per frame in turns). It is what holds a new track, whose velocity is not known yet; an
  established track is held tighter by its filter's gate (99.9% of the predicted error) and a size
  check.
* `max_unseen_frames` (int or `auto`): frames a track survives without a detection (`auto`: a third
  of a second). Longer bridges occlusions but keeps stale tracks.
* `id_switch_threshold` (int): consecutive frames reading another ID before a track's ID changes
  (5).
* `pose_alpha` (double): exponential smoothing of the position, 1.0 off (1.0). Smaller values smooth
  jitter but lag, and on a moving camera the lag shows as range error.
* `min_track_age` (int): frames a track must exist before it is published (2). Clutter rarely lives
  two frames.

### `ros_interface`

* `publish_poses`, `publish_images` (the annotated `image_out`), `publish_tf`,
  `publish_debug_images` (panels on `<debug_images_topic>/{binary,candidates,markers}`, drawn only
  while subscribed) and `publish_visualization_markers` (bool).
* `tf_frame_prefix`, `parent_frame_id` (string): each marker's frame is `<prefix><ID>` under the
  parent frame, or `<prefix>track_<track id>` while it has no ID.
* `input_queue_size` (int): depth of the image subscription (30). Frames are processed in the
  callback, so a deep queue keeps every frame of a bag replay, while a live camera wants 1-2 to stay
  current.
* `input_reliable` (bool): reliable QoS for the image subscription (true). Bag playback and Isaac
  Sim need it, since a best-effort subscription loses most of their 2.8 MB frames; camera drivers
  that publish best effort need false.
* `image_encoding` (string): `rgb8`, `bgr8`, `rgba8`, `bgra8` or `mono8`. Frames of another encoding
  or size are dropped.
* `image_input_topic`, `poses_output_topic`, `image_output_topic`, `debug_images_topic`,
  `visualization_markers_topic` (string): topic names.
* `detection_enabled_service_name` (string): a `std_srvs/SetBool` service that pauses and resumes
  detection.

### `camera`

* `package_name`, `config_path` (string): the intrinsics file, relative to that package's share
  directory.
* `image_width`, `image_height` (int, px): the frame size. Frames of another size are dropped, and
  the intrinsics must be calibrated at this resolution.
* `frame_rate` (double, Hz): the camera's frame rate, which sets the per-frame motion limits. It
  must be positive.

## Camera intrinsics file

The standard ROS calibration format. The node reads `camera_matrix` and `distortion_coefficients`;
the size comes from the main configuration. Both must be accurate for correct poses.

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

## Setting up a new camera or marker

1. Calibrate the camera at the resolution it runs, then set `camera.image_width`, `image_height`,
   `frame_rate`, `ros_interface.image_encoding` and `detector.image_gamma`.
2. Measure `outer_diameter` on the print, and set `id_bits` and `identification.enabled` to match
   the markers. Leave the `auto` values on.
3. Run `whycon_debug.launch.py` and watch the `candidates` panel; change only a check that drops
   real markers or passes clutter.
4. For range, `min_size` sets the smallest marker found and `min_marker_pixels` the smallest one
   identified; raising either costs range.
