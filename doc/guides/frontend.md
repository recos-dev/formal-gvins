# Building and testing the vision frontend

The frontend is optional and separate from the numerical core. It uses OpenCV
4.13.0; `lib/opencv` and `lib/opencv_contrib` use matching release revisions.
Only the CUDA profile needs contrib. No system-wide OpenCV installation is needed.
Use CMake 3.20+, a C++20 compiler and the core build dependencies (Eigen by
default). Frontend tests and benchmark reporting also use Python 3.10+ (standard
library only). OpenCV itself is built in C++17 mode; project targets use C++20.

## CPU build

```bash
git submodule update --init --depth 1 lib/opencv
cmake -S lib/opencv -B build/opencv-cpu -C configs/opencv-cpu.cmake
cmake --build build/opencv-cpu --parallel 4
cmake -S . -B build/frontend-cpu -DCMAKE_BUILD_TYPE=Release \
    -DFORMAL_GVINS_BUILD_FRONTEND=ON -DOpenCV_DIR="$PWD/build/opencv-cpu"
cmake --build build/frontend-cpu --parallel 4
ctest --test-dir build/frontend-cpu --output-on-failure
```

The profile builds core, imgproc, imgcodecs, features2d and calib3d, plus their
required dependencies. PNG decoding is enabled for EuRoC. OpenCL, IPP, parallel
processing frameworks, OpenCV SIMD baseline and extra CPU dispatch are disabled.
Compiler auto-vectorization remains available. These choices define
an initial comparison profile, not a claim of maximum CPU performance.

## CUDA build

Install a CUDA development toolkit compatible with the installed NVIDIA driver.
Select architecture `8.6` for RTX 3060 or `8.7` for Orin Nano; those targets need
separate timing evidence. `nvidia-smi` reports driver compatibility, not whether
the compiler and development libraries are installed.

```bash
git submodule update --init --depth 1 lib/opencv lib/opencv_contrib
cmake -S lib/opencv -B build/opencv-cuda -DCUDA_ARCH_BIN=8.6 \
    -C configs/opencv-cuda.cmake
cmake --build build/opencv-cuda --parallel 4
cmake -S . -B build/frontend-cuda -DCMAKE_BUILD_TYPE=Release \
    -DFORMAL_GVINS_BUILD_FRONTEND=ON -DFORMAL_GVINS_FRONTEND_CUDA=ON \
    -DOpenCV_DIR="$PWD/build/opencv-cuda"
cmake --build build/frontend-cuda --parallel 4
ctest --test-dir build/frontend-cuda --output-on-failure
```

For a toolkit outside the default search paths, add
`-DCUDA_TOOLKIT_ROOT_DIR=/absolute/path/to/toolkit` to both configurations.
Use different build directories for CPU and CUDA. The GPU CTest requires an
accessible NVIDIA device; it fails rather than silently falling back or skipping.

`OrbExtractor` is a blocking, single-owner adapter. CUDA work finishes before
return, and results own host descriptor storage. This first implementation includes
uploads/downloads in execution cost; descriptors selected on the host are uploaded
for GPU matching. Device-resident compaction is a future optimization, not an
assumed property. GPU and CPU ORB need not select identical keypoints/descriptors.
CUDA accelerates ORB extraction and Hamming matching. Image decoding,
spatial selection, geometric checks and track bookkeeping remain on the CPU.
Measure total pipeline latency and tracking quality, not just kernel time.

Both paths enable 7x7 Gaussian descriptor smoothing with sigma 2 and reflected
borders. Equal preprocessing settings do not make their implementations or
descriptor bytes identical. See the [reference settings and next
checks](../analysis/orb-frontend-settings.md) before selecting deployment defaults.

The extractor uses WTA_K=2 and Hamming matching. Configuration accepts 1--10000
features, 1--16 pyramid levels, scale factor in (1, 2] and FAST threshold 0--255.
These are adapter limits, not approved estimator or flight-quality thresholds.
OpenCV exceptions are contained at operation boundaries; allocation failures can
still propagate in this host layer. Core builds retain their no-exception profile.

## EuRoC frontend replay

### Tracker API

Include `formal_gvins/vision/orb_tracker.hpp` to construct `OrbTracker` with
`OrbTrackingOptions`. Call `process(identity, image, mask, tracked)` for the common
`TrackedImage` result, and `reset()` before changing stream bindings. The result
contains image identity, an explicit pixel domain and owning observations; it
does not contain ORB descriptors or OpenCV matching containers. The current path
expects raw images and publishes `PixelDomain::raw`. Each observation carries its raw pixel and derived normalized ray.

For ORB inspection, supply an optional fifth argument pointing to
`OrbTrackingDiagnostics`. Its keypoints/descriptors follow the observation order;
match indices refer to the preceding successful image and the current image.
Diagnostics own their data independently of tracker history. Returned failures
preserve both outputs. Without diagnostics, callers need only the shared result
and the `process`/`reset` operations; another tracking method may supply those
operations without descriptors. See [V-EXECUTION](../spec/requirements/vision/frontend-execution.md#tracker-interface).

### Replay command

Download and extract an ASL/CSV sequence using the [dataset guide](datasets.md).
Pass its `mav0` directory, not a ROS bag. The first profile uses cam0 monocular
images, fixed pinhole/radial-tangential calibration and raw pixels with
the original K. Both camera streams and their extrinsics are read, but this demo
does not perform stereo association. It requires the 17-column processed ground
truth and an identity IMU/body extrinsic; unsupported layouts fail explicitly.

```bash
./build/frontend-cpu/euroc_frontend datasets/euroc/MH_01_easy/mav0 --frames 500
./build/frontend-cuda/euroc_frontend datasets/euroc/MH_01_easy/mav0 --frames 500 --cuda
```

Output defaults to `output/<sequence>/cpu` or `output/<sequence>/cuda`. `--output`
selects another directory. CSV files are replaced; snapshots are written every
100 images and at the end. Old snapshots from longer runs are not deleted; the
current `frames.csv` identifies this run's frames. PNGs show accepted temporal
matches. There is no GUI or display-server dependency.

The profile publishes at most 400 observations over an 8x6 grid, selected from
up to 1000 ORB candidates. Empty cells do not donate their quota. Matching uses
strict 0.75 two-neighbor ratios in both directions, mutual-best selection,
and a calibrated essential-matrix check using OpenCV's five-point RANSAC
(0.003 normalized-coordinate threshold, 0.99 confidence, at most 2000 iterations). It requires at
least eight unique candidates and eight accepted inliers. The tracker requires
the actual radial-tangential calibration to unproject each pixel. RANSAC operates
on these normalized coordinates with identity intrinsics; the threshold is
explicitly normalized, not a pixel-noise specification.
This is a geometric screen, not a parallax, pose-observability or triangulation
acceptance test. See [the OpenCV calibration API](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
Geometry failure creates new IDs; it never confirms all descriptor matches.
Reset retires history without reusing IDs. Camera, calibration and noise-model
changes need an explicit reset.
These are development settings awaiting requirement review, not calibrated
measurement-noise or flight-acceptance values.

Integer nanosecond timestamps preserve the acquisition clock. IMU noise densities
are read without converting them into per-sample variances. Ground truth stays in
the dataset reference frame; no NED/ENU/local-W alignment or interpolation is
implicit. Published quaternion coefficients are preserved, not silently normalized;
nonzero finite coefficients do not certify unit norm. A later scoring adapter must
qualify that data and any normalization. IMU and truth are loaded for interface
validation but are not fused or used to score a trajectory: this executable is
a frontend demo, not an MSCKF.

`frames.csv` separates image read time from complete frontend time. The
summary excludes the first 10 frames for warmup; visualization and CSV writes are
outside that latency. `observations.csv` records image/time/track identities and
raw pixels, normalized rays and per-camera observation ages. `events.csv` records
started/continued/ended IDs; `profile.txt` binds configuration, calibration and OpenCV build.
Session/calibration/noise IDs are local to that run. Resampling changes image noise;
binding 1 is an unqualified development model, not a covariance ready for fusion.

Regression fixtures check timestamp precision, calibration direction, invalid
data, descriptor encoding, known geometric outliers, spatial quotas and track
lifetime. Actual-image inlier counts and timing do not establish physical match
accuracy, navigation accuracy or target real-time bounds.

## Fixed-dataset ORB benchmark

Benchmarks are grouped by subject under `benchmark/`. The `features/` category
contains its C++ program, runner, scorer, workload profile and dedicated tests.
Additional benchmark subjects belong in sibling directories when implemented.

### Sequential CPU/CUDA run

After building the CUDA frontend and extracting the dataset, run:

```bash
./scripts/run_benchmark.py features
```

This runs CPU and then CUDA through the same `build/frontend-cuda/orb_benchmark`
executable, freezes one profile for both, and prints completion counts, latency
percentiles, retained-feature counts and synthetic quality metrics. It reuses the
single-backend runner below. A failed backend does not prevent attempting the
next one. Missing CUDA is a failure, not a skipped comparison or CPU fallback.

Every invocation creates a new `output/benchmark/features/<UTC timestamp>/`:

```text
profile.json       Frozen workload shared by CPU and CUDA
report.md          Readable CPU/CUDA metric tables and total elapsed time
summary.json       Combined results, source/input/build/device identities and status
cpu/               Native results, frame CSV, images and OpenCV build information
cuda/              The corresponding CUDA evidence
```

`summary.json` embeds the backend summaries and is published as complete only
after both runs, binding checks and the readable report finish. Differences in
input hashes, profile, executable/evaluator identity or reported build/adapter
settings make the comparison fail while retaining both results. This check does
not certify GPU determinism or controlled clocks/load. Quality metrics remain
separate from timing; partial results cannot serve as a baseline.

Use `--backend cpu` for a CPU-only run with `build/frontend-cpu/orb_benchmark`,
or `--backend cuda` for CUDA only. `--executable PATH` overrides the executable
for all selected backends. `--dataset`, `--profile` and `--output` select inputs
or a named result directory; an existing output directory is rejected, preserving
earlier review candidates. The script runs prebuilt binaries, not a build step.

Saved results start with `baseline_review: unreviewed`. Before pinning a baseline,
rebuild from a reviewed clean revision, inspect input/configuration identities and
quality, and establish repeated-run timing/quality variation on the named host.
Then select explicit acceptance tolerances and the compact records to version.
This command does not approve, replace or compare against a regression baseline.
Raw run directories remain Git-ignored.

### Workloads and individual runs

The first runnable benchmark uses the production C++ `OrbTracker`, the EuRoC
reader and a shared Python evaluator. It does not replace the frontend with a
Python ORB implementation. The [profile](../../benchmark/features/profile.json)
selects MH_01_easy cam0, zero-based frames 100--199, single-threaded OpenCV and a
fixed RNG seed. Its `tracking` section explicitly sets every configurable ORB and
tracking option; omitted options fail instead of inheriting production defaults.
The runner writes `resolved-profile.json` once, passes that snapshot to both C++
processes and checks their reported options against it. Fixed adapter properties
such as WTA_K and descriptor smoothing are recorded and checked by the runner.

With the CPU build and extracted dataset above:

```bash
cmake --build build/frontend-cpu --target orb_benchmark --parallel 4
python3 benchmark/features/run_feature_benchmark.py
```

The runner defaults to `datasets/euroc/MH_01_easy/mav0` and
`build/frontend-cpu/orb_benchmark`. Use `--dataset MAV0_DIR` or `--executable PATH`
to locate the same inputs/build elsewhere. `--profile FILE` selects a different
explicit workload. With the CUDA build above:

```bash
cmake --build build/frontend-cuda --target orb_benchmark --parallel 4
python3 benchmark/features/run_feature_benchmark.py --cuda
```

`--cuda` selects the CUDA executable/backend with no CPU fallback. Both workloads
record the selected device's name, index, compute capability and total device
memory; this is device capacity, not measured allocation. An unavailable or
incompatible device fails the run. CPU and CUDA execution have been tested on the
fixed window using an RTX 3060 host; Orin execution remains untested.

For a CPU/CUDA comparison using the same executable and OpenCV build, also run:

```bash
python3 benchmark/features/run_feature_benchmark.py \
    --executable build/frontend-cuda/orb_benchmark
```

Run the two backends sequentially. Keep power, clock and background load controlled
for timing comparisons; these development presets disable CPU SIMD/IPP, so their
ratio does not represent maximum CPU performance.
The fixed RNG seed does not guarantee repeatable CUDA detections or track IDs.
Local repeated runs show small count/quality variation; GPU regression baselines
need reviewed tolerances and repeated-run evidence, not raw-output hash equality.

Two workloads use the same selected images:

| Workload | Measurement |
|---|---|
| Actual consecutive frames | Production temporal tracking, observation/track IDs, accepted-match counts, read latency and complete frontend latency. Per-feature precision, recall and RMSE remain unavailable without correspondence truth. |
| Synthetic translation | Translate each raw image by `(12, -8)` pixels with a translated validity mask. Run the same tracker on the original/translated pair, then independently score its accepted matches against that known translation at 3 pixels. This is a geometry sanity test, not physical landmark ground truth or nonplanar epipolar qualification. |

The workloads run sequentially in separate C++ processes, each with its own
tracker and RNG state. The second workload is still attempted if the first fails;
each has separate completion, failure and missing-case counts. Synthetic setup
or tracking failures cannot reset real temporal tracks. Timings cover blocking production calls, including CUDA
transfers/synchronization when selected; image loading, warping, scoring and
visualization are outside frontend timing. The pair timing covers two tracker
calls; it is not a single-frame latency. This is an offline per-call measurement,
not a real-time arrival/scheduling test or an isolated hardware performance gate.

Latency summaries exclude the first 10 scheduled frames and report median,
nearest-rank P95/P99 and maximum over the remaining successful samples. Quality
summaries report equally weighted per-frame means over defined metrics across all
100 frames, with evaluated counts. Failed/missing cases remain in the scheduled
case denominator; empty detections do not manufacture precision or RMSE.

Results go to `output/feature-benchmark/cpu/` (or `cuda/`):

- `temporal.json`, `translation.json`: separate native results, options, timings
  and CUDA device identity when selected.
- `resolved-profile.json`: the configuration snapshot consumed by both processes.
- `frames.csv`: one row per scheduled case and workload, including failed/missing
  cases. Column names and order are fixed; unavailable measurements are empty.
- `summary.json`: per-workload completion/failure counts, latency/quality summaries, selected
  input timestamps/SHA-256 hashes, executable/evaluator identities and host details.
- `temporal.png`, `translation.png`: accepted matches for the final frame/pair.
- `opencv-build.txt`: OpenCV build information.

Use one writer per output directory. Rerunning replaces the named outputs and
invalidates previous completion before cleanup or execution. CSV publication
finishes before the final summary is atomically replaced. A failed publication
leaves a failed, unfinished or absent completion marker; `running` and `writing`
are unfinished states. This is a host file-publication protocol, not a power-loss
durability guarantee.

Top-level `complete` means both workloads executed/scored every requested case
and their required report files were published. One failed workload does not
discard the other's successful measurements. Completion does not mean
an approved quality or regression threshold passed. Baseline comparison and
per-stage profiling remain later work in the [benchmark proposal](../analysis/feature-benchmarks.md).

Unit tests run without a dataset. Opt in to the same full dataset pipeline in
CTest explicitly; it fails on missing inputs rather than downloading or skipping:

```bash
python3 -m unittest discover -s benchmark/features/tests -p 'test_*.py' -v
cmake -S . -B build/frontend-cpu \
    -DFORMAL_GVINS_BENCHMARK_DATASET="$PWD/datasets/euroc/MH_01_easy/mav0"
ctest --test-dir build/frontend-cpu -R feature_benchmark --output-on-failure
```

For an already configured CUDA build, register the same dataset and run GPU tests:

```bash
cmake -S . -B build/frontend-cuda \
    -DFORMAL_GVINS_BENCHMARK_DATASET="$PWD/datasets/euroc/MH_01_easy/mav0"
ctest --test-dir build/frontend-cuda -L gpu --output-on-failure
```

This adds `feature_benchmark_euroc_cuda` alongside the CPU dataset test. GPU tests
share a CTest resource lock to avoid competing with one another in a parallel run;
external GPU users and other CTest processes are outside that lock.

CTest keeps its separate temporary run outputs in its build directory. Dataset
files and run outputs remain Git-ignored. Numerical-core-only builds do not
require OpenCV, Python or these dataset tests.

### Pointwise camera inverse

`camera/radtan.hpp` is shared with the numerical forward projection. The inverse
uses a bounded Newton solve initialized by `(u-cx)/fx,(v-cy)/fy`, with a normalized
radius of 2, at most 20 evaluations (hard limit 32), maximum component step 0.5,
minimum slope 1e-4 and forward residual at most 1e-8 pixels. These are the current
double-precision EuRoC trial settings; no image is resampled. The tracker rejects
features whose inverse fails and publishes only raw pixels paired with valid rays.

The calibration check bounds both radial Jacobian eigenvalues over the disk and
subtracts a conservative tangential-distortion norm bound. A positive margin
excludes folding in exact arithmetic; the floating-point check is not an interval
proof. Unit tests independently project known rays in long double, recover them,
cover the EuRoC image grid and reject folded domains/nonconvergence atomically.
Image bounds and calibration identity are checked by the frontend.

### Subpixel localization

Extraction now detects first, then refines in each octave before computing a
descriptor. CPU and CUDA use the same host localization stage: 11x11 gradient
window, at most 20 one-step `cornerSubPix` calls, 0.001 octave-pixel stopping
tolerance and at most 2 raw-pixel displacement from detection. Checking each
sampled center prevents an unobserved iteration path through an excluded mask.
The library still does not report singularity/reset/termination per point;
unchanged coordinates are not labeled certified convergence.

The full 32-octave-pixel support is checked against raw image and mask bounds.
Post-refinement admission requires OpenCV's 3x3 minimum gradient eigenvalue at
least 1e-4 and eigenvalue ratio at most 100. These are explicit trial limits.
Orientation is the intensity centroid in a radius-15 disk sampled at the accepted
location in the same pyramid (`INTER_LINEAR_EXACT` resizing). ORB descriptors
are then computed with that location, octave and refreshed orientation; OpenCV
rounds their sampling centers to octave pixels. Subpixel measurement coordinates
do not imply continuously interpolated binary descriptors.

The CPU path uses provided-keypoint ORB. CUDA uses GPU detection, the same host
localization and GPU provided-keypoint description/Hamming matching. A fresh
CUDA descriptor instance prevents stale empty-octave counts in the pinned adapter;
its creation, transfer and synchronization belong in complete latency. This is
an explicit mixed CPU/GPU pipeline, never an automatic CPU fallback.

`orb_localization` measures absolute error on independently rendered checker
intersections with varied angle, contrast and blur, plus failure/support tests.
Its 0.15 raw-pixel test bound applies to those fixtures; it is not a measured
real-camera noise covariance. Real-image and per-octave noise qualification
remains necessary before using observations in an estimator.

### Reciprocal descriptor association

Temporal association now computes 2-NN Hamming candidates in both directions.
Each direction must pass the strict ratio (initial value 0.75), and both best
choices must point to each other. Equal distances, including two zero distances,
and a missing second candidate reject. A collision is resolved by mutual best,
not a greedy distance sort. Malformed indices, repeated queries or invalid
distances fail without publishing a partial result. This gives one-to-one
associations before geometry; no absolute Hamming cutoff is enabled.

### Temporal geometry

Both views use calibrated normalized rays. Five-point essential RANSAC proposes
a model; it must have sufficient two-dimensional sample spread and at least
eight supporting correspondences. The model is normalized to unit Frobenius
norm and every accepted RANSAC inlier must also satisfy the explicit squared
Sampson residual at threshold 0.003. Denominators at or below 1e-12 reject.
Sample spread requires scatter trace above 1e-12 and determinant/trace-squared
above 1e-8; this detects repeated/collinear samples, not insufficient depth parallax.

For rotation-dominated data, E alone can fit distractors because translation
direction is underdetermined. A supplementary proper-rotation fit uses up to
100 deterministic three-pair trials, unit-ray SVD and all-support refinement.
Only when at least eight and 90% of E's inliers also support that rotation is
the normalized reprojection-distance screen applied, at the same threshold.
It only removes E inliers; it never rescues failed E geometry or establishes
zero translation/observable depth. The rank check requires the second singular
value above 1e-6 of the first.

`orb_geometry` checks known correspondences with distractors across static, pure
rotation, small baseline, planar and general scenes, over eight seeds each.
This exercises a declared degeneracy policy; epipolar/rotation agreement still
does not certify physical feature identity or triangulation validity.

### Continuing-track selection

The tracker matches the bounded detected/localized candidates before applying
the publication grid. Verified continuing tracks rank first by descending age,
then stable ID. New candidates rank by response and original candidate index.
An 8 raw-pixel minimum Euclidean separation applies across every cell and octave.
Per-cell quotas still apply to old and new tracks; unused quotas are not moved
to other cells. A continuing candidate can therefore be retired by quota/spacing,
but a stronger new detector response cannot displace an otherwise eligible old
track. This cannot preserve a point that the current detector did not find.

IDs are allocated only after selection. Descriptor rows, rays, ages and diagnostic
match indices are reindexed together. Diagnostics distinguish pre-selection
reciprocal/geometric counts from published matches, and report quota/spacing
rejections. `feature_selection` tests ordering, exact spacing boundaries, identity
validation and dense candidate accounting independently of ORB detection.

### Track lifecycle and failures

`TrackedImage.events` explicitly publishes `started`, `continued` and `ended`
for this camera. A valid featureless frame ends its old tracks; a missing image
is an error and preserves history. Event and observation vectors own their data.
Diagnostic age counts successful observations in this camera, not elapsed time.
Reset clears history without reusing IDs and leaves already returned snapshots
unchanged. A new object requires a fresh session if its counters could overlap
old records; its numerical calibration is fixed at construction.

`tracker_failure` uses an ELF linker wrapper in its test executable to force
an OpenCV geometry exception after history exists, then verifies output/history
atomicity and a retry against a control tracker. This adds no production callback
or mock tracker and does not claim to emulate device loss. Other host tests cover
backwards/duplicate time, image size and binding changes, reset, featureless
images, unavailable backends and caller mutation of returned data.

### Instrumented regression reports

The v4 benchmark pins localization, inverse-camera and geometric-screen settings
as well as ORB/selection options. Reports include candidate rejection counts,
grid occupancy, completed track-length distributions and still-active lengths
reported separately as right-censored observations. A missing/failed temporal
frame makes lifetime statistics unavailable; it is not silently omitted.

Optional frontend diagnostics measure detection, localization, description,
unprojection, matching, geometry, selection and publication preparation. Calls
without diagnostics perform no clock or OS memory queries. Stage timings include
blocking GPU work and transfers in their owning stages. Image I/O, visualization
and report writes are outside frontend time. Per-stage RSS is the Linux process
high-water mark observed at stage exit, including warmup and library allocations;
it is neither stage-owned allocation nor GPU VRAM. Zero/unavailable measurements
are rendered as N/A. Instrumentation can affect latency; these are host observations,
not worst-case execution or Orin qualification.

To compare with an earlier complete report, use a new output directory:

```bash
python3 scripts/run_benchmark.py features --backend both \
    --baseline output/prior-run/summary.json --output output/new-run
```

The runner requires matching dataset hashes, profile, scoring tools, machine and
native build configuration before reporting deltas. Code/executable revisions may
change. It does not invent regression limits or approve a baseline automatically.
Translation tests have an independently known pixel correspondence, not ground
truth for general 3D motion. EuRoC temporal precision/recall and trajectory RMSE
remain N/A; its pose truth does not identify each image feature.
