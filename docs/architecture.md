# Architecture

This document describes the high-level structure of the WhyCon/WhyCode localization stack and how its major modules connect.

## System Overview

At a high level, the system is a single ROS2 node (or component) that:

1. Subscribes to a camera image stream.
2. Detects elliptical markers and decodes WhyCode IDs.
3. Tracks markers across frames for stability.
4. Publishes poses, images, and TF for downstream consumers.

## High-Level Flow

```mermaid
flowchart LR
  Camera[Camera image stream] --> ImageTransport[image_transport subscription]
  ImageTransport --> ImageHandler[ImageHandler: preprocessing + binarization]
  ImageHandler --> Detector[MarkerDetector + MultiMarkerDetector]
  Detector --> Decoder[CNecklace WhyCode decoding]
  Decoder --> Tracker[MarkerTracker + IDStabilizer]
  Tracker --> Pose[LocalizationSystem + PoseStabilizer]
  Pose --> Topics[ROS outputs]
  Topics --> Poses[/whycon/poses]
  Topics --> Images[/whycon/image_out]
  Topics --> Debug[/whycon/debug_images]
  Topics --> TF[/tf]
  Topics --> Markers[/whycon/visualization_markers]
```

## Core Modules

* **WhyconComponent / WhyconRosInterface**: ROS2 glue for parameters, subscriptions, and publishing.
* **ImageHandler / PackedBinaryImage**: Image preprocessing, binarization, and packed binary representations.
* **MarkerDetector / MultiMarkerDetector**: Ellipse detection and candidate verification.
* **CNecklace**: WhyCode decoding and Hamming distance checks.
* **MarkerTracker / IDStabilizer / PoseStabilizer**: Tracking and stability filters for IDs and poses.
* **LocalizationSystem**: Pose estimation from ellipse geometry and camera intrinsics.

## Component vs Standalone

* **Standalone**: The `whycon` executable runs the node directly.
* **Composable**: `whycon::WhyconComponent` can be loaded into a component container for intra-process use.

For launch options and configuration details, see [configuration/README.md](configuration/README.md).
