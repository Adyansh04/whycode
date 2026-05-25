# Performance Improvement Ideas (Private)

This file is for internal reference only. Do not link it from other docs.

## Image input and preprocessing

* Reduce per-frame allocations in the ROS image callback; reuse buffers where possible.
* Avoid repeated color conversions when the input encoding already matches the detector expectations.
* Consider using a fixed thread pool for image handling if camera rates are high and CPU cores are available.

## Binarization and packed binary image

* Fuse preprocessing + binarization passes to reduce memory bandwidth.
* Use SIMD-friendly memory alignment for packed bit buffers.
* Add explicit prefetching on large images when scanning rows.

## Marker detection and ellipse fitting

* Vectorize geometric checks (center distance, circularity, ratio) for candidate ellipses.
* Minimize branching in the candidate validation loop; prefer branchless comparisons.
* Cache intermediate ellipse stats to avoid recomputation across filters.

## WhyCode decoding (CNecklace)

* Replace scalar Hamming distance loops with SIMD popcount or LUT-based popcount.
* Batch ring samples and evaluate multiple hypotheses per loop iteration.
* Cache ring sampling offsets for common radii to reduce trig calls.

## Tracking and stabilization

* Vectorize the association distance computations when tracking multiple markers.
* Reuse Kalman filter state buffers and avoid per-frame object creation.
* Consider a faster association strategy for large marker counts (early pruning by gating).

## Pose computation and TF publishing

* Avoid repeated matrix inversions when values are unchanged across frames.
* Cache common transforms (camera frame to marker frame) when possible.
* Consider using small fixed-size math types with SIMD for 3x3 operations.

## Memory layout and allocations

* Consolidate per-frame temporary buffers into a single scratch arena.
* Prefer contiguous storage for marker candidates and tracking state.
* Use small-vector or fixed-capacity containers to avoid heap churn.

## Parallelization

* Partition image processing by rows or tiles (OpenMP or std::execution) for high-res inputs.
* Keep the per-frame critical path single-threaded if latency matters more than throughput.
* Guard shared structures with minimal locking; consider lock-free queues for image handoff.

## Build and compiler tuning

* Verify vectorization reports for hot loops (binarization, ellipse scoring).
* Profile with perf or VTune to confirm hot spots before refactoring.
* Use link-time optimization (already enabled) and consider profile-guided optimization.
