# Detection and Tracking Pipeline

This document describes the core processing pipeline for WhyCon/WhyCode localization, focusing on the high-CPU stages and data flow between them.

## Pipeline Flow

```mermaid
flowchart TD
  A[Input image] --> B[Preprocess + resize/convert]
  B --> C[Binarize and pack bits]
  C --> D[Find contours / candidate ellipses]
  D --> E[Ellipse validation + geometry checks]
  E --> F[WhyCode decode (ring sampling)]
  F --> G[Tracking and ID stabilization]
  G --> H[Pose estimation + stabilization]
  H --> I[Publish poses, images, TF, markers]
```

## Stage Details

### 1) Image input and preprocessing

* Subscribes to the configured ROS image topic via `image_transport`.
* Converts to the internal format expected by the detector.
* Uses a fixed-rate processing timer when configured.

### 2) Binarization and packed representation

* Converts grayscale to a binary mask using fast thresholding.
* Packs bits to reduce memory footprint and improve cache behavior.

### 3) Ellipse detection

* Extracts candidate ellipses and validates geometry constraints.
* Uses center distance, circularity, and size thresholds from config.

### 4) WhyCode decoding

* Samples ring segments around the detected ellipse.
* Applies Hamming distance checks with error correction.

### 5) Tracking and stabilization

* Associates detections across frames.
* Stabilizes IDs and poses to reduce flicker.

### 6) Pose estimation and outputs

* Computes 6-DOF pose using camera intrinsics and marker geometry.
* Publishes pose arrays, debug images, TF, and visualization markers.

## Related Modules

* `image_handler`, `packed_binary_image`
* `marker_detector`, `multi_marker_detector`
* `CNecklace`
* `marker_tracker`, `id_stabilizer`, `pose_stabilizer`
* `whycon_localization`
