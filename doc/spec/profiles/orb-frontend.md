# ORB development profile

**Status:** Draft; current development settings, not deployment acceptance.

**Requirements:** [V-ORB](../requirements/vision/orb-frontend.md),
[V-LOCALIZE](../requirements/vision/feature-localization.md),
[V-STEREO](../requirements/vision/stereo-association.md) when enabled,
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
| Input | Raw radial-tangential pixels; required intrinsics and distortion; `CV_8UC1`, optional same-size `CV_8UC1` mask with nonzero meaning admitted. |
| Candidate extraction | 1000 features, scale 1.2, 8 levels, FAST 20, Harris score, patch/edge 31, first level 0. |
| Descriptor | WTA_K=2, 32-byte binary descriptor, Hamming distance; CPU and CUDA enable 7x7 Gaussian smoothing, sigma 2, `BORDER_REFLECT_101`. |
| Publication | At most 400 features, 8 by 6 grid; verified continuing tracks first by age/ID, new candidates by response/index; 8 raw-pixel minimum spacing across cells/scales; per-cell quotas are not redistributed. |
| Association | Two-nearest candidates in both directions; strict ratio 0.75 in both and mutual best; reject ties and missing runners-up. |
| Geometry | Calibrated essential-matrix five-point RANSAC followed by explicit unit-model Sampson gating; 0.003 normalized-coordinate threshold, 0.99 confidence, 2000 maximum iterations; at least eight unique candidates and eight inliers. |
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

The tracker publishes raw pixels and their derived normalized rays. Pointwise
inverse semantics are defined by [M-UNPROJECT](../requirements/msckf/camera-unprojection.md).
The current double-precision inverse uses radius 2, 20 evaluations, maximum
component step 0.5, minimum slope 1e-4 and forward residual 1e-8 pixels.
These trial values are checked against known rays and the EuRoC image domain;
they are not a deployment accuracy or interval certificate. Failed inverses
are excluded, with keypoint/descriptor pairing preserved. RANSAC uses normalized
rays and an explicitly normalized threshold. Benchmark protocol v4 identifies
this raw-image path separately from earlier remapped-image evidence.

## Raw-image qualification

[V-LOCALIZE](../requirements/vision/feature-localization.md) uses octave-local
refinement before descriptor computation. Trial settings: window radius 5,
20 maximum one-step calls, stopping tolerance 0.001 octave pixels, maximum
raw-pixel displacement 2, minimum gradient eigenvalue 1e-4 and maximum condition
100. Full 32-octave-pixel support must be inside image/mask. Each iteration's
sample center is checked. OpenCV does not expose every per-point failure reason;
these checks and synthetic error tests do not certify convergence or real-image
noise. Intensity-centroid orientation is refreshed after refinement. CUDA retains
GPU detection/description/matching with shared host localization. See the
[implementation guide](../../guides/frontend.md#subpixel-localization) for the
sampling recipe and scope of the known-corner tests.

## Temporal degeneracy policy

The normalized Sampson denominator floor is 1e-12 for a unit-Frobenius model.
Samples require scatter trace above 1e-12 and determinant/trace-squared above
1e-8. No minimum depth parallax is imposed on temporal association.

E alone is underdetermined under pure rotation. An additional proper-rotation
screen is selected only when at least eight and 90% of E inliers support it.
It uses at most 100 three-pair trials (fixed seed 0), unit-ray SVD, second/first
singular-value ratio above 1e-6 and all-support refinement. Its normalized
reprojection threshold is the same 0.003. It only removes E inliers; no failed
E fit is rescued. These are explicit trial settings, not an estimator motion
classification. [Geometry fixtures](../../../tests/test_epipolar.cpp) cover five
motion/scene regimes over eight seeds each and known distractors. Physical
correspondence and depth observability remain separate obligations.

## CPU/CUDA compatibility and acceptance gaps

CPU and CUDA enable the same descriptor-smoothing parameters. This does not
establish interchangeable descriptors or identical keypoints/track IDs. Check
pairing, bounds, matching quality and complete latency for each backend. Stored
map descriptors and vocabulary compatibility require separate qualification.

The geometric test is an epipolar screen, not a depth/parallax, pose-observability
or triangulation certificate. Detector fallback is not part of this profile.
Changes require named experiments and renewed acceptance, not implicit adoption.

Before estimator/deployment acceptance, bind pixel-noise covariance in the selected
image domain, tracking accuracy limits, exposure/clock assumptions, image-size/resource limits,
allocation-failure recovery and target deadlines. RTX 3060 measurements cannot
qualify Orin execution. The replay's development noise-model ID does not discharge
these obligations. [Per-module benchmarks](../../roadmap.md#b-01-per-module-unit-benchmarks)
remain planned evidence.

## Optional stereo binding

[StereoOrbTracker](../../../include/formal_gvins/vision/stereo_tracker.hpp) applies
both mono trackers, then reciprocal association using fixed rig geometry. The
[Stereo demo](../../../examples/euroc_stereo.cpp) uses EuRoC cam0/cam1 and converts
camera-to-body extrinsics using V-STEREO. Images remain raw and unrectified.

| Setting | Development value/policy |
|---|---|
| Synchronization | Exact acquisition timestamp equality in the EuRoC demo; unmatched exposures are skipped. The reusable tracker accepts a caller-qualified maximum skew, default zero. No asynchronous motion compensation. |
| Rig | Finite proper R_RL, orthogonality/determinant residuals at most 1e-8, finite nonzero t_RL. Structural checks do not qualify physical calibration. |
| Stereo screen | Reciprocal ratio 0.75; known E=[t_RL]_x R_RL normalized to unit Frobenius norm; Sampson threshold 0.003 and denominator floor 1e-12. No estimated F, horizontal-row assumption or depth gate. |
| Identity | One allocator for both cameras; temporal IDs take priority, old/old conflicts reject, new partners may share/inherit an ID, no duplicate ID within a camera image. |
| Failure | Invalid camera input/temporal failure preserves both outputs/histories; stereo rejection/skew/backend failure keeps valid mono results with explicit association status. |
| Lifecycle | Events/ages are per-camera segments. A camera may reacquire an ID still live in its partner; globally retired numbers are not recycled. |

The same camera, ORB and publication defaults apply to each view. A geometry link
alone does not establish triangulation, positive depth, exposure synchronization,
physical correspondence or estimator acceptance. Stereo noise/overlap/motion limits
remain unqualified. Numerical outputs can agree with the declared model while the
physical calibration or synchronization assumptions are wrong.

CUDA FAST insertion/culling may choose different candidate sets even for repeated
inputs on one device. Independent GPU runs are therefore not a bit-exact identity
oracle. Tests retain exact failure-snapshot checks, row/ID invariants and labeled
quality checks; deterministic CPU executions additionally compare retries with an
untouched control. This does not waive reproducibility metadata or acceptance limits.
