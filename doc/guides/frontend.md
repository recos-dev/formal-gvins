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
