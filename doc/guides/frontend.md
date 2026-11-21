# Building and testing the vision frontend

The host frontend is optional and separate from the numerical core. OpenCV and
contrib are pinned to 4.13.0 by matching submodules; no system-wide installation
is needed. Use CMake 3.20+, a C++20 compiler and the core dependencies (Eigen by
default). Reporting/tests require Python 3.10+ and its standard library. OpenCV
itself builds in C++17 mode. Numerical-core-only builds need none of these image
processing tools.

[The frontend profile](../spec/profiles/orb-frontend.md) owns trial values,
[EuRoC](../spec/profiles/euroc.md) owns dataset interpretation, and
[V-ORB](../spec/requirements/vision/orb-frontend.md) owns required behavior.
These remain Draft; working examples do not approve deployment settings.

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

The profile builds core, imgproc, imgcodecs, features2d, calib3d and required
dependencies, with PNG support. OpenCL, IPP, parallel frameworks, OpenCV SIMD
baseline and extra CPU dispatch are disabled; compiler auto-vectorization remains
available. This is a comparison profile, not maximum CPU performance.

## CUDA build

Install a toolkit compatible with the NVIDIA driver. Use architecture 8.6 for
RTX 3060 or 8.7 for Orin Nano; each target needs its own timing evidence.
`nvidia-smi` reports driver compatibility, not toolkit/compiler availability.

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

For a nonstandard toolkit location, add `-DCUDA_TOOLKIT_ROOT_DIR=/absolute/path`
to both configurations. Use separate CPU/CUDA build directories. GPU tests fail
on an unavailable device; they do not silently skip or fall back to CPU.

CUDA performs detection, description and Hamming matching on the device;
localization and geometry run on the host. Calls block until results own host
storage. Host selection means descriptors are uploaded again for matching.
Measure complete latency, including transfers, synchronization and adapter setup.
A fixed RNG seed does not guarantee identical CUDA candidate sets or track IDs:
FAST insertion/culling can vary between repeated executions. CPU/CUDA descriptor
compatibility and map/vocabulary reuse require qualification beyond byte shape.

## Tracker interface

`vision/tracker.hpp` defines method-neutral identities, observations, per-camera
lifecycle events, statuses and `TrackedImage`. A frontend implements named
`process` and `reset` operations without requiring a base class. ORB descriptors
and OpenCV match containers do not enter the filter interface.

Construct `OrbTracker` with fixed `OrbTrackingOptions`. Call
`process(identity, image, mask, output)`; request `OrbTrackingDiagnostics` through
the optional fifth argument for owning descriptor rows, match indices, ages and
stage measurements. Rows follow observation order. Processing failures preserve
outputs and history. A valid featureless image ends previous camera tracks;
a missing image is an error. Reset retires history without recycling IDs.
A replacement object needs a fresh session if its counters could overlap old IDs.
Numerical calibration is fixed at construction; changing a binding token alone
does not change the camera model.

Images are raw `CV_8UC1` with an optional same-size validity mask. Observations
retain raw pixels plus derived rays `[x,y,1]`, not additional measurements. The
[frontend architecture](../spec/architecture/visual-frontend.md) explains inverse
projection, descriptors, temporal geometry, selection and stereo publication.
The [profile](../spec/profiles/orb-frontend.md) owns settings and qualification gaps.

### Subpixel localization

CPU/CUDA use shared host localization before descriptor generation. Its library
calls, support/quality checks, sampling recipe and parameter units are defined in
the [profile](../spec/profiles/orb-frontend.md#raw-image-qualification). Passing
those observable checks does not certify per-point convergence or physical noise.

## Mono replay

Extract an ASL/CSV sequence using the [dataset guide](datasets.md), then pass its
`mav0` directory. The reader requires the supported 17-column processed truth and
identity IMU/body binding. It preserves integer acquisition timestamps, raw camera
calibration, IMU noise densities and unnormalized truth quaternion coefficients.
It performs no truth interpolation, normalization or reference-frame alignment.

```bash
./build/frontend-cpu/euroc_frontend datasets/euroc/MH_01_easy/mav0 --frames 500
./build/frontend-cuda/euroc_frontend datasets/euroc/MH_01_easy/mav0 --frames 500 --cuda
```

Output defaults to `output/<sequence>/cpu` or `cuda`; `--output DIR` overrides it.
`frames.csv` contains count/rejection/timing diagnostics; `observations.csv` records
raw pixels, normalized rays and camera-local ages; `events.csv` lists lifecycle
changes; `profile.txt` records the example configuration and OpenCV build.
PNGs every 100 images and at the end show temporal links without a GUI. CSVs are
replaced on rerun; older PNGs are not deleted, so use current `frames.csv` to identify
this run's frames. Complete frontend timing excludes reads and output writes;
the console timing summary excludes ten warmup frames.

IMU and ground truth are loaded for interface validation but not fused or scored.
This application is a frontend replay, not a complete MSCKF or trajectory estimate.

## Calibrated stereo replay

```bash
./build/frontend-cpu/euroc_stereo datasets/euroc/MH_01_easy/mav0 --frames 100
./build/frontend-cuda/euroc_stereo datasets/euroc/MH_01_easy/mav0 --frames 100 --cuda
```

The application constructs `StereoOrbTracker` from two camera models and
`R_RL,t_RL` derived from camera-to-body extrinsics. Exact integer acquisition
timestamps pair exposures; unmatched exposures are skipped and counted. Arrival
time and unbounded nearest-frame pairing are not used. The reusable tracker
accepts a caller-qualified maximum skew, default zero. Timestamp labels alone
do not qualify physical exposure synchronization.

Output defaults to `output/<sequence>/stereo-cpu` or `stereo-cuda` and supports
`--output`. CSVs record both cameras, per-camera ages/events, accepted stereo links,
geometry/identity rejection counts and complete pair latency. PNGs show cross-camera
links. This uses raw, unrectified images and known-rig E, without assuming equal
rows or estimating an unconstrained F. It does not triangulate or estimate motion.

New verified partners share an ID. Existing different IDs are never renamed to
force a match, and one ID cannot appear twice within a camera image. A camera can
reacquire an ID still active in its partner; a restarted camera segment has age one.
This is not reuse of a globally retired ID. Stereo rejection/skew/backend failure
preserves valid mono results. Invalid camera input or temporal failure preserves
both outputs/histories; all fallible work precedes either history commit.
See [V-STEREO](../spec/requirements/vision/stereo-association.md).

## Fixed-dataset ORB benchmark

The `benchmark/features/` category contains the native program, workload profile,
runner, independent scorer and tests. The v4 [profile](../../benchmark/features/profile.json)
pins MH_01_easy cam0 frames 100--199, single-threaded OpenCV, RNG seed and every
configurable extraction/localization/inverse/association/selection option.
Omitted settings fail instead of inheriting current library defaults.

After building the CUDA frontend, run CPU then CUDA through the same executable:

```bash
./scripts/run_benchmark.py features
```

The runner freezes one profile and creates `output/benchmark/features/<UTC timestamp>/`
with `profile.json`, readable `report.md`, combined `summary.json` and separate
`cpu/` and `cuda/` native results, frame CSVs, PNGs and OpenCV build information.
It runs prebuilt binaries. Missing CUDA is a failure, not a skipped comparison.
A failed backend does not prevent attempting the next one.

Use `--backend cpu` or `--backend cuda` for one backend. `--executable`, `--dataset`
and `--profile` select the build/data/profile; `--output DIR` requires a new
result directory. Compare with an earlier complete report using:

```bash
./scripts/run_benchmark.py features --backend both \
    --baseline output/prior-run/summary.json --output output/new-run
```

The runner checks workload/input hashes, scoring tools, machine and native build
settings before reporting deltas. The code/executable revision may change across
baselines; CPU/CUDA comparisons within one run require the same executable.
Saved results remain `baseline_review: unreviewed`: completion and measured changes
do not establish acceptance. Select tolerances only after reviewing labeled quality
and repeated-run variation. There is no automatic baseline replacement or invented
regression PASS threshold. Control power, clocks and background load for comparisons.

### Workloads and metrics

| Workload | Meaning |
|---|---|
| Actual consecutive frames | Production temporal tracking, lifecycle, coverage and timing. EuRoC pose truth does not identify individual features, so temporal precision/recall and feature RMSE are N/A. |
| Synthetic translation | Translate each raw image by `(12,-8)` pixels, propagate a validity mask, and score known correspondence at a 3-pixel tolerance. This is a labeled image-transform check, not physical 3D motion or nonplanar geometry qualification. |

Each workload has its own C++ process/tracker; both are attempted independently.
Complete frontend latency includes blocking transfers and synchronization, excluding
image I/O, synthetic warping, scoring and visualization. Synthetic pair time covers
two calls, not one frame. Reports exclude ten scheduled warmup frames from timing
and retain median, nearest-rank P95/P99, maximum and sample count. Quality reports
use per-frame means over defined values across all requested cases. Failed/missing
cases remain in the schedule; undefined precision/error stays N/A.

Optional stage timing covers detection, localization, description, unprojection,
matching, geometry, selection and publication preparation. No diagnostics means
no clock/OS profiling calls. Instrumentation may affect latency. Per-stage RSS is
Linux process high-water observed at stage exit, including warmup/libraries;
it is not stage-owned allocation or GPU VRAM. Device capacity is recorded separately.
Unavailable measurements stay N/A. These observations are not WCET or Orin evidence.

Reports include rejection counts, grid occupancy, completed track-length statistics
and separate still-active, right-censored lengths. Missing/failed temporal frames
make lifetime statistics unavailable instead of bridging unknown history. Identity,
age and diagnostic accounting are checked during ingestion, not trusted blindly.

For a single-backend workload run:

```bash
python3 benchmark/features/run_feature_benchmark.py
python3 benchmark/features/run_feature_benchmark.py --cuda
```

These commands default to `output/feature-benchmark/cpu` or `cuda`, replacing only
the named outputs and invalidating previous completion before work starts. Use one
writer per directory. Each backend writes `temporal.json`, `translation.json`,
`resolved-profile.json`, `frames.csv`, `summary.json`, two final-pair PNGs and
`opencv-build.txt`. CSV publication precedes the completion summary. Unfinished
or failed outputs cannot be a baseline; this is not a power-loss durability promise.

### Tests and evidence limits

```bash
python3 -m unittest discover -s benchmark/features/tests -p 'test_*.py' -v
cmake -S . -B build/frontend-cpu \
    -DFORMAL_GVINS_BENCHMARK_DATASET="$PWD/datasets/euroc/MH_01_easy/mav0"
ctest --test-dir build/frontend-cpu -R feature_benchmark --output-on-failure
```

For CUDA, configure the same dataset path in `build/frontend-cuda`, then run
`ctest --test-dir build/frontend-cuda -L gpu --output-on-failure`. GPU CTests share
a resource lock; external GPU users/processes are outside it. Dataset tests fail
on missing inputs rather than downloading or skipping. Data and reports stay ignored.

| Check | Scope |
|---|---|
| `camera_inverse` | Independent known-ray forward/inverse checks, EuRoC image domain, invalid/folded domains and atomic failures. No uniform floating-point/interval proof. |
| `orb_localization` | Independently rendered corners with known positions, varied angle/contrast/blur, octave/support/quality tests. Fixture error bounds are not a real-camera covariance. |
| `orb_geometry` | Static, rotation, small baseline, planar and general scenes with known distractors. No physical-association guarantee from epipolar agreement alone. |
| `feature_selection`, `orb_tracking` | Quotas, cross-cell spacing, continuation priority, pairing, events, reset and failures. |
| `stereo_tracking` | Nonparallel distorted cameras, labeled outliers, degeneracy, skew, shared IDs, conflicts and reacquisition. |
| `tracker_failure` | ELF-only dependency injection into real geometry/matching calls: atomic mono/pair retry and stereo-only failure preserving mono results. Not an actual driver-loss test. |
| Python benchmark tests | Independent scores, lifecycle/count accounting, failed/missing cases, publication ordering and baseline compatibility. |

CPU uses exact independent-control retry comparisons. CUDA checks exact failure
snapshots and row/ID/quality contracts without requiring independently repeated
candidate sets to be identical. Real-camera noise, held-out association quality,
synchronization, deployment deadlines and complete VINS accuracy remain separate
qualification work. Geometry acceptance is not triangulation/depth acceptance.
