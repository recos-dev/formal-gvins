# Building and testing the vision frontend

The frontend is optional and separate from the numerical core. It uses OpenCV
4.13.0; `lib/opencv` and `lib/opencv_contrib` use matching release revisions.
Only the CUDA profile needs contrib. No system-wide OpenCV installation is needed.
Use CMake 3.20+, a C++20 compiler and the core build dependencies (Eigen by
default). OpenCV itself is built in C++17 mode; project targets use C++20.

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
CUDA accelerates ORB extraction and Hamming matching. Image decoding/remapping,
spatial selection, geometric checks and track bookkeeping remain on the CPU.
Measure total pipeline latency and tracking quality, not just kernel time.

The current CUDA call disables descriptor smoothing, whereas CPU ORB smooths
before computing descriptors. This is a known preprocessing difference, not
just an execution-backend difference. See the [reference settings and next
checks](../analysis/orb-frontend-settings.md) before interpreting timings as a
same-settings speedup or selecting deployment defaults.

The extractor uses WTA_K=2 and Hamming matching. Configuration accepts 1--10000
features, 1--16 pyramid levels, scale factor in (1, 2] and FAST threshold 0--255.
These are adapter limits, not approved estimator or flight-quality thresholds.
OpenCV exceptions are contained at operation boundaries; allocation failures can
still propagate in this host layer. Core builds retain their no-exception profile.

## EuRoC frontend replay

Download and extract an ASL/CSV sequence using the [dataset guide](datasets.md).
Pass its `mav0` directory, not a ROS bag. The first profile uses cam0 monocular
images, fixed pinhole/radial-tangential calibration and undistorted pixels with
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
a strict 0.75 two-neighbor ratio, deterministic one-to-one conflict resolution,
and a calibrated essential-matrix check using OpenCV's five-point RANSAC
(1.5 pixel threshold, 0.99 confidence, at most 2000 iterations). It requires at
least eight unique candidates and eight accepted inliers. The tracker requires
the actual undistorted-pixel K; it does not assume identity intrinsics. OpenCV
normalizes coordinates by K and scales the threshold by the mean focal length.
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

`frames.csv` separates image read/remap time from complete frontend time. The
summary excludes the first 10 frames for warmup; visualization and CSV writes are
outside that latency. `observations.csv` records image/time/track identities and
undistorted pixels; `profile.txt` binds configuration, calibration and OpenCV build.
Session/calibration/noise IDs are local to that run. Resampling changes image noise;
binding 1 is an unqualified development model, not a covariance ready for fusion.

Regression fixtures check timestamp precision, calibration direction, invalid
data, descriptor encoding, known geometric outliers, spatial quotas and track
lifetime. Actual-image inlier counts and timing do not establish physical match
accuracy, navigation accuracy or target real-time bounds.
