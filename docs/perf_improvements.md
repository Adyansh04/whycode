# Performance Improvement Ideas (Detailed, Code-Referenced)

This file lists concrete optimization opportunities with exact source anchors.

## Ground Rules

- Verify every change with profiler data (`perf`, VTune, callgrind, or tracy).
- Preserve algorithmic behavior and detection accuracy.
- Prioritize hot path improvements before micro-optimizing cold code.

## 1) Image Ingest and Preprocessing

### Current Hotspots

- `src/image/image_handler.cpp:94` `ImageHandler::updateFromROS`
  - Performs full-frame `memcpy` every callback.
  - Runs `SimdRgbToGray` over entire frame each time.

### Potential Improvements

- Add fast path for encodings that already match grayscale pipeline to skip unnecessary conversion.
- Reuse incoming buffer view when safe for zero-copy read-only operations (if lifecycle permits).
- Evaluate row-stride aware conversion if incoming data alignment differs from expected `width_ * bpp_`.

## 2) Candidate Detection and Ellipse Analysis

### Current Hotspots

- `src/core/marker_detector.cpp:221` `analyzeMarkerCandidate`
- `src/core/marker_detector.cpp:416` `detectMarkerPair`
- `src/core/marker_detector.cpp:97` `computeEllipseStatsSIMD`

### Potential Improvements

- Reduce branch density in candidate rejection checks by grouping cheap rejects first.
- Avoid recomputing geometric invariants when a candidate survives multiple checks.
- Add explicit likely/unlikely branch hints for dominant rejection paths.
- Benchmark replacing some scalar post-processing with wider SIMD batches.

## 3) WhyCode Sampling and Decode

### Current Hotspots

- `src/core/whycon_localization.cpp:194` `computeSignal`
- `src/core/whycon_localization.cpp:225` `binarizeSignal`
- `src/core/whycon_localization.cpp:278` `processSingleSolution`
- `src/core/CNecklace.cpp:218` `CNecklace::decode`

### Potential Improvements

- `computeSignal`: cache-friendly access by precomputing row pointers for sampled Y values.
- `binarizeSignal`: test unroll depth and alignment assumptions per target architecture.
- `processSingleSolution`: reduce repeated modulo operations inside tight loops.
- `CNecklace::decode`: optional LUT/popcount acceleration for Hamming comparisons.

## 4) Tracking and Association

### Current Hotspots

- `src/tracking/marker_tracker.cpp:74` `MarkerTracker::update`
  - O(track × detection) distance matrix and sort of potential matches.

### Potential Improvements

- Early gating by coarse cell bins before exact squared-distance checks.
- Reuse `potential_matches_` capacity aggressively for higher marker counts.
- Benchmark partial selection (min-heap or bucketed gating) vs full sort.

## 5) Pose and Publish Path

### Current Hotspots

- `src/ros/whycon_ros_interface.cpp:299` `publishResults`
- `src/ros/whycon_ros_interface.cpp:441` `publishSingleTF`

### Potential Improvements

- Build output messages only when subscribers are present on each topic.
- Avoid repeated string formatting for overlays when image publishing is disabled.
- Batch TF sends when marker count is large.

## 6) Triangulation Nodes

### Current Hotspots

- `src/triangulate/two_marker_whycode_triangulation.cpp:154` `estimatePose`
- `src/triangulate/four_marker_whycode_triangulation.cpp:167` `performHierarchicalTriangulation`

### Potential Improvements

- Reuse temporary Eigen matrices/vectors across timer iterations.
- Short-circuit intermediate computations when marker timeout state is stale.
- For 4-marker path, avoid duplicate quaternion conversions in publish branches.

## 7) YAML and Config Access

### Current Hotspots

- `src/utils/param_loader.cpp:74` template `getParams`

### Potential Improvements

- Keep this out of frame path (already mostly true).
- If runtime reconfigure is added later, cache parsed numeric values and avoid repeated YAML node traversal.

## 8) Build and Toolchain

### Current Anchors

- `CMakeLists.txt:18` optimization flags
- `CMakeLists.txt:22` AVX2 feature check

### Potential Improvements

- Profile-guided optimization for deployment build.
- Validate `-march=native` portability requirements for target hosts.
- Use vectorization reports to confirm hot loops are actually autovectorized.

## Suggested Benchmark Order

1. `ImageHandler::updateFromROS`
2. `MarkerDetector::detectMarkerPair`
3. `LocalizationSystem::computeSignal`
4. `MarkerTracker::update`
5. `WhyconRosInterface::publishResults`

This order usually captures the largest latency contributors first.
