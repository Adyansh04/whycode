# Benchmarks

Timing and self-check programs for the detector's per-frame kernels. They build with the package
(`BUILD_BENCHMARKS`, on by default) into `build/whycode_vision/src/benchmarks/` and are not
installed.

## Running

Pin each run to one performance core: on a hybrid CPU an unpinned run moves between core types
and its times swing. On the i9-14900HX below, cores 0-15 are performance cores.

```bash
cd build/whycode_vision/src/benchmarks
taskset -c 2 ./detector_bench <frames>/rgb_*.ppm
```

The first group reads real frames, 1280x720: colour as P6 `.ppm` (RGB, the message's order) and
grey as P5 `.pgm`. The results below use ten frames of the 1-lap Isaac bag, every 186th. The
second group generates its own data and takes no arguments.

## Results

Measured on 21 Oct 2026: Intel i9-14900HX laptop, performance power profile, pinned to core 2;
GCC with `-O3 -march=native -mavx -mfma -ffast-math -flto`; xsimd on AVX2 (reported as
avxvnni); OpenCV 4.6.

Absolute times move by up to a quarter with the laptop's temperature: `detector_bench` has
measured from 270 to 345 µs per frame on this machine. Compare times within one run, not across
runs.

### Detector kernels on real frames

These exercise the code the detector ships. All except `detector_bench` exit 1 if the shipped
code disagrees with its reference.

| Benchmark | What it measures | Result |
|---|---|---|
| `detector_bench <rgb.ppm>...` | `MultiMarkerDetector::detectMarkers`, 6 targets with the Isaac configuration's parameters, 100 calls per frame. The checksum covers every valid marker's outputs bit for bit, and changes with any change in what is detected. | 344 µs per frame on average (median per frame 170-552 µs, more for frames with more markers); checksum `fd52e3931934b804` |
| `floodfill_bench <grey.pgm>...` | The scanline flood fill (`analyzeMarkerCandidate` with `FreePixels`) against a per-pixel 4-connected fill, flooding every segment at three thresholds; then random images at widths 1-848, which exercise the row padding. | 412 µs vs 5566 µs per flood (13.5x); 27,744 segments on the frames and 3,321 on the random images, 0 differ |
| `binarize_packed_bench <grey.pgm>...` | `PackedBinaryImage::binarizeBitPacked` against the former version, and checked against `grey < threshold` for every threshold, also at widths that end in a partial word. | 16.8 µs vs 26.8 µs per 1280x720 binarisation (1.6x); 15,420 checks, 0 wrong bits |
| `rgb_to_gray_bench <rgb.ppm>...` | `SimdRgbToGray`, the grey conversion of every frame, against a plain fixed-point loop, with the frame in cache (hot) and after flushing the caches (cold). | Simd 133-161 µs hot, 201-220 µs cold; loop 335-376 µs; 0 grey levels differ |
| `eigen_vs_xsimd_bench` | The code-ring kernels in xsimd, as shipped, against Eigen: the 720 ring coordinates with their bounds check, and the ring signal's mean and bits. | Coordinates 241 ns vs 354 ns; mean and bits 72 ns vs 496 ns; largest coordinate difference 0.00006 px, 0 bits differ |

### Alternatives explored during optimisation

Synthetic data, comparing candidate implementations. The note on each says what the detector
uses.

| Benchmark | What it measures | Result |
|---|---|---|
| `benchmark` | A segment's centroid and second moments over 100,000 random pixel indices of a 1920x1080 image, 1000 times: scalar against xsimd, aligned and unaligned. | Scalar 301 ms; SIMD 156-165 ms (1.8-1.9x); the SIMD sums differ in the last float digits. The detector now sums exact integer moments over each segment's runs. |
| `segment_bench` | The code ring's edge-phase sum (`readCode`), 10,000 rings of 720 samples: `cos`/`sin` per sample, a lookup table, and two SIMD versions of it. | Per-sample trig 10.5 ms; lookup table 1.2 ms (8.6x); SIMD 11.9-13.6 ms, slower than the scalar table. The detector uses the scalar table. |
| `ellipse_bench` | The code ring's 720 sample coordinates: scalar, xsimd `sin`/`cos`, and a lookup table with xsimd, aligned and unaligned. | Scalar 1.06 ms; xsimd trig 0.24-0.25 ms but off by up to 553 px, wrong; lookup table 0.031-0.040 ms (27-34x), within 0.0002 px. The detector uses the lookup table. |
| `binarize_loop_bench` | The ring signal's mean and binarisation, 720 samples, 2,000,000 times. | Scalar 167 ms; SIMD unaligned 401 ms (0.42x); unrolled 145 ms and aligned 148 ms (1.13-1.15x); all agree. The detector uses the unrolled, aligned loop. |
| `bilinear_bench` | Bilinear sampling of the RGB frame at the ring's points: four variants of the scalar loop. | Original 35.7 ms; FMA with prefetch 25.8 ms (1.38x); the others 29.5-31.2 ms; all within 0.00006 of each other. |
| `benchmark_binarization` | Binarising a 1920x1080 frame 1000 times: Simd's 8-bit binarisation against four bit-packed xsimd versions. Its test image path is from the ROS 1 tree, so it uses a synthetic pattern; it writes six PNGs into the working directory. | 8-bit 104.3 ms; bit-packed 67.5-85.9 ms (1.2-1.55x) in an eighth of the memory; the "ultra-fast aligned" version matches on only 97.5% of the checks. The detector's binarisation is the one `binarize_packed_bench` checks. |
| `cv_simd` | OpenCV's universal intrinsics comparing four floats, 50,000,000 times. | SIMD 0.079 s vs scalar 0.182 s (2.3x); same results. |
| `branch_pred` | The edge-phase sum with a branch per sample against a branch-free version, at 0, 1, 10 and 50% sign changes, 307,200 rings each. | With the branch 66.6, 113.9, 397.4 and 1516 ms; branch-free 2.9-3.0 s at every rate. The branch wins even at 50% changes. |
| `class_test` | Calling methods through a `std::unique_ptr` against a direct object, and constructing each. | Equal within noise (0.94-1.10x); construction 0.005 ms either way. |
