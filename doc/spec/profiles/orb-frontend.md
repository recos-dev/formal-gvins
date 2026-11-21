# ORB development profile

**Status:** Draft; current development settings, not deployment acceptance.

**Requirements:** [V-ORB](../requirements/vision/orb-frontend.md),
[V-EXECUTION](../requirements/vision/frontend-execution.md),
[V-OBSERVATION](../requirements/vision/observation-admission.md).

## Current implementation binding

This profile describes [OrbOptions/OrbTracker](../../../include/formal_gvins/vision/orb_tracker.hpp)
and the [EuRoC frontend executable](../../../examples/euroc_frontend.cpp).
OpenCV and contrib are pinned by their Git submodules; CMake requests exactly
4.13.0. [CPU](../../../configs/opencv-cpu.cmake) and
[CUDA](../../../configs/opencv-cuda.cmake) build settings are separate bindings.
The [reference comparison](../../analysis/orb-frontend-settings.md) gives the
basis for experiments, not permission to inherit another estimator's thresholds.

| Setting | Current development value/policy |
|---|---|
| Input | Undistorted zero-skew pinhole pixels; required actual K; `CV_8UC1`, optional same-size `CV_8UC1` mask with nonzero meaning admitted. |
| Candidate extraction | 1000 features, scale 1.2, 8 levels, FAST 20, Harris score, patch/edge 31, first level 0. |
| Descriptor | WTA_K=2, 32-byte binary descriptor, Hamming distance; CPU and CUDA enable 7x7 Gaussian smoothing, sigma 2, `BORDER_REFLECT_101`. |
| Publication | At most 400 features, 8 by 6 grid; stable response ordering, per-cell quotas; unused quotas are not redistributed. |
| Association | Forward two-nearest candidates; strict ratio 0.75; greedily resolve one-to-one conflicts by distance, query index, then train index. |
| Geometry | Calibrated essential-matrix five-point RANSAC; 1.5-pixel threshold, 0.99 confidence, 2000 maximum iterations; at least eight unique candidates and eight inliers. |
| Failure to obtain geometry | Successful frame with no confirmed continuations; assign new IDs, never accept all matches as inliers. |
| Stream identity | Nonzero session/calibration/noise IDs; increasing image IDs and timestamps after the first image; fixed camera/binding/image size until reset. |
| Execution | Explicit CPU or CUDA, blocking single-owner calls; CUDA ORB and Hamming matching, other stages on CPU; no automatic fallback. |

Constructor validation allows candidate count 1--10000, scale `(1,2]`, levels
1--16 and FAST threshold 0--255. Tracking capacity is 1--10000, grid dimensions
1--64 each, capacity at least the cell count and no greater than candidate count.
Ratio/confidence lie in `(0,1)`, geometric threshold is finite positive, and
iterations are 1--10000. These validation limits are not calibrated quality bounds.
Initialization/configuration status is checked before operation-specific input;
OpenCV exceptions are contained, while host allocation exceptions may propagate.

## Local VINS target

The [local VINS profile](local-vins.md) selects extraction/description on original
distorted images and publication of raw pixel coordinates. The current tracker
above still assumes undistorted pinhole input; its K-only essential-matrix call
cannot be used on raw distorted coordinates unchanged.

The migration must bind K and the lens model/coefficients, derive normalized
coordinates pointwise for geometric verification, and retain raw coordinates for
the estimator's residuals. Derived coordinates may be cached or recomputed, but
must stay paired with the same raw feature and calibration. Inverse failures
cannot become geometric inliers. Declare the geometric residual metric and
threshold units before replacing the current 1.5-pixel test; its value is not a
normalized-coordinate threshold.

Keep the extraction, descriptor and capacity settings above as starting values.
Qualify tracking, raw-pixel localization noise and CPU/CUDA benchmarks again for
this domain. Retained remapped-image results remain an identified earlier input
profile, not a baseline that silently changes meaning. This migration has no
production implementation yet.

## CPU/CUDA compatibility and acceptance gaps

CPU and CUDA enable the same descriptor-smoothing parameters. This does not
establish interchangeable descriptors or identical keypoints/track IDs. Check
pairing, bounds, matching quality and complete latency for each backend. Stored
map descriptors and vocabulary compatibility require separate qualification.

The geometric test is an epipolar screen, not a depth/parallax, pose-observability
or triangulation certificate. Minimum spacing, cross-scale deduplication, reciprocal
matching, subpixel refinement and detector fallback are not part of this profile.
Changes require named experiments and renewed acceptance, not implicit adoption.

Before estimator/deployment acceptance, bind pixel-noise covariance in the selected
image domain, tracking accuracy limits, exposure/clock assumptions, image-size/resource limits,
allocation-failure recovery and target deadlines. RTX 3060 measurements cannot
qualify Orin execution. The replay's development noise-model ID does not discharge
these obligations. [Per-module benchmarks](../../roadmap.md#b-01-per-module-unit-benchmarks)
remain planned evidence.
