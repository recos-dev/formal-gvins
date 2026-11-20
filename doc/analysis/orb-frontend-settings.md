# ORB frontend reference settings

**Status:** Draft comparison and tuning basis; not an approved requirement.

The project keeps ORB as its selected descriptor frontend. The references below
identify which settings have a concrete precedent and which still need project
experiments. Equal parameter values do not establish equivalent pipelines or
descriptor compatibility. This assessment does not change implementation defaults.

## Reference boundaries

| Reference | Reviewed profile and revision | Relevant use |
|---|---|---|
| OpenVINS | [EuRoC configuration][ov-config], `69488123ed93` | MSCKF-oriented feature tracking. The supplied profile selects **KLT**, with an optional ORB-descriptor path when `use_klt=false`. |
| ORB-SLAM3 | [EuRoC mono-inertial][orb-mono] and [stereo-inertial][orb-stereo], `4452a3c4ab75` | Custom ORB extraction, map tracking, relocalization and loop matching. |
| VINS-Mono | [EuRoC configuration][vins-config], `90dabb5ec799` | KLT local tracking and a separate FAST/BRIEF loop frontend. |
| GVINS | [VI-Sensor/F9P configuration][gvins-config], `d2cf40b49c6e` | KLT visual tracking in a GNSS/VINS estimator; not an ORB configuration precedent. |
| OpenCV / contrib | [CPU API][cv-api] and [CUDA API][cuda-api], 4.13.0 pins used by this repository | Exact library defaults and implementation differences. |

Links pin source revisions. Read configuration together with its caller and
extractor/matcher implementation; a YAML field can be inactive on the selected
path.

## Extraction and observation budgets

| Setting | Current formal-gvins | OpenVINS descriptor path with EuRoC values | ORB-SLAM3 EuRoC |
|---|---|---|---|
| Feature budget | 1000 requested extraction candidates, then a strict 400-observation cap | `num_pts=200` target per camera; grid extraction/filtering determines actual count | 1000 mono / mono-inertial; 1200 stereo-inertial per extractor |
| Detector | OpenCV multiscale ORB, Harris ranking | Grid FAST, followed by `ORB::compute`; not the full `detectAndCompute` path | Custom pyramid FAST and spatial-tree distribution, retaining FAST responses |
| Pyramid | Scale 1.2, 8 levels | Do not interpret the default ORB object's 8 levels as an 8-level detection pass: points are supplied by grid FAST | Scale 1.2, 8 levels |
| FAST threshold | 20 | 20 | Start at 20; retry an empty cell at 7 |
| Spatial selection | 8x6 quotas after extraction; no explicit minimum separation | 5x5 extraction grid; `min_px_dist=10` occupancy cells in the descriptor path | Adaptive subdivision within each pyramid level |
| Subpixel refinement | None beyond the extractor's reported coordinates | Grid detector calls `cornerSubPix`: 5x5 window, 20 iterations, epsilon 0.001 | No equivalent separate refinement in the reviewed extraction path |
| Preprocessing | Undistort whole image; no histogram equalization; 752x480 in the demo | Histogram equalization; detect on the original image domain, undistort coordinates for geometry | Mono / mono-inertial YAML requests 600x350 from 752x480; stereo-inertial keeps 752x480 |
| Descriptor smoothing | CPU: 7x7 Gaussian, sigma 2; CUDA: currently disabled | CPU OpenCV descriptor computation includes smoothing | 7x7 Gaussian, sigma 2 |

Sources: [OpenVINS configuration][ov-config], [grid detector][ov-grid],
[descriptor extraction][ov-desc], [ORB-SLAM3 extractor][orb-extractor] and
the pinned EuRoC profiles above.

The CPU/CUDA smoothing difference follows the actual descriptor kernels:
[CPU preprocessing](https://github.com/opencv/opencv/blob/fe38fc608f6acb8b68953438a62305d8318f4fcd/modules/features2d/src/orb.cpp#L1234),
[CUDA conditional preprocessing](https://github.com/opencv/opencv_contrib/blob/d99ad2a188210cc35067c2e60076eed7c2442bc3/modules/cudafeatures2d/src/orb.cpp#L811),
and [our constructor arguments](../../src/vision/orb.cpp).

OpenCV's API defaults are 500 features, scale 1.2, 8 levels, border 31,
first level 0, WTA_K=2, Harris score, patch 31 and FAST threshold 20.
Our 1000-candidate setting changes that default budget. WTA_K=2 uses ordinary
Hamming distance. ORB-SLAM3 also emits 32-byte descriptors with a 31-pixel
pattern, but its border constant is 19 and its extractor is different;
copying its YAML does not reproduce it. Its monocular initialization constructs
a separate extractor with five times the configured budget. [CPU API][cv-api],
[ORB-SLAM3 extractor][orb-extractor], [extractor construction][orb-tracking].

Three counts must remain distinct: detector candidates, retained observations,
and features consumed in a filter update. For example, OpenVINS configures
200 tracked points per camera and `max_msckf_in_update=40`; these are not the
same budget. Its target count and occupancy filtering are also not our strict
published-cap contract. An occupancy cell is not a guaranteed Euclidean
minimum-distance test across cell boundaries. Mask polarity also differs:
OpenVINS uses 255 for excluded regions; our OpenCV-facing mask uses nonzero
for admitted pixels. A borrowed mask needs an explicit conversion.

## Temporal matching and geometry

| Stage | Current formal-gvins | OpenVINS descriptor path |
|---|---|---|
| Nearest neighbors | Previous to current, two Hamming neighbors | Both directions, two Hamming neighbors |
| Ratio | Strict `d1 < 0.75*d2`; ties including two zero distances rejected | Configured ratio 0.70, followed by reciprocal matching |
| Conflicts | Lowest-distance deterministic one-to-one selection | Accept only reciprocal best matches after both ratio tests |
| Geometric model | Calibrated essential matrix, five-point RANSAC | Fundamental matrix RANSAC on undistorted normalized coordinates |
| Threshold | 1.5 undistorted-pixel units; OpenCV scales by mean focal length | `1 / max(fx0,fy0,fx1,fy1)` in normalized coordinates |
| Robust-estimation parameters | Confidence 0.99, explicit 2000-iteration cap; at least 8 candidates and 8 inliers | Confidence 0.999; at least 10 candidates; no explicit max-iteration argument at this call |

[OpenVINS matching source][ov-match] is a suitable comparison for our MSCKF
frontend. Reciprocal matching and greedy uniqueness are different acceptance
rules. Preserve our explicit tie rejection: the reference's division-based ratio
test rejects only values greater than its threshold, so copying that expression
would change equality and zero-distance behavior.

ORB-SLAM3 has **no single global ratio setting**. Its matcher constructor
defaults to 0.6, while reviewed tracking call sites use 0.9 for initialization,
0.7 for reference-keyframe BoW matching, and 0.75 for relocalization BoW matching.
It also uses context-dependent Hamming thresholds (50/100), projected search
regions, scale restrictions and orientation checks. Those values do not justify
a global 0.75 ratio or a global absolute-distance gate in our temporal frontend.
[Matcher interface][orb-matcher-h], [matcher implementation][orb-matcher],
[tracking call sites][orb-tracking].

VINS-Mono and GVINS instead replenish `goodFeaturesToTrack` corners and track them
with pyramidal LK (21x21 window, maximum level 3). Both reviewed profiles use
150 features, 30-pixel spacing, equalization enabled and F threshold 1.0; the
VINS-Mono profile publishes at 10 Hz, while GVINS uses 0 to follow the input rate.
Their F check operates on reprojected undistorted coordinates using a synthetic
focal length of 460, so “1 pixel” is not automatically one raw-image pixel.
VINS-Mono's loop path uses FAST threshold 20 and BRIEF, not the same local tracker.
[Tracker][vins-tracker], [coordinate setup][vins-parameters],
[loop descriptors][vins-loop], [GVINS tracker][gvins-tracker],
[GVINS coordinate setup][gvins-parameters].

## Decisions and next checks

| Item | Recommendation and evidence boundary |
|---|---|
| Basic descriptor profile | Keep scale 1.2, 8 levels, FAST 20, WTA_K=2/Hamming and patch 31 as a source-backed starting point. This does not validate accuracy or budgets. |
| CPU/CUDA comparability | First enable CUDA descriptor blur to match the CPU preprocessing intention, then rerun quality and timing checks. Equal blur settings still do not imply bitwise-identical extraction/descriptors. |
| Association policy | Compare reciprocal matching with the current greedy rule; use OpenVINS's 0.70 as a reference trial alongside 0.75. Do not copy its ratio edge-case behavior. |
| Spatial distribution | Evaluate proximity suppression and cross-scale duplicate selection before raising feature counts. Test 200 and 400 retained observations separately from the 1000-candidate budget. Grid-first detection and post-selection quotas are not equivalent. |
| Low texture and pixel precision | Evaluate FAST 20-to-7 fallback, histogram processing and subpixel refinement separately. Each changes behavior, cost or measurement-noise assumptions. |
| Geometry | Retain the declared calibrated essential model for the initial comparison; test thresholds 1.0/1.5 in its stated domain. The reference F model/threshold is not a drop-in equivalent. Neither a RANSAC threshold nor confidence is a calibrated measurement covariance. |
| Future map descriptors | Bind extraction/preprocessing/version to stored descriptors and the vocabulary. Do not assume CPU, CUDA, OpenVINS or ORB-SLAM3 descriptors are interchangeable because each is called ORB. |

**Known comparison limitation:** the earlier local CPU/CUDA timings were measured
with CPU smoothing enabled and CUDA smoothing disabled, and with CPU SIMD/IPP
disabled. They establish execution of those two profiles, not a controlled
same-settings acceleration ratio. They must not become a target performance
requirement or an accuracy-equivalence claim.

Use MH_01 for controlled development comparisons, then separate MH sequences for
validation. Change one policy at a time, keeping image domain, resolution,
implementation revisions, threading, device and warmup fixed. Record actual
feature count, spatial coverage, track lengths, rejection/restart behavior,
synthetic known-correspondence errors and complete frontend latency. Inlier counts
alone do not measure false associations. Once the estimator exists, add trajectory
and consistency evaluation before selecting deployment defaults.

This survey changes neither code nor Draft requirement approval. The
[frontend guide](../guides/frontend.md) owns build/replay commands; experimental
outputs stay outside versioned documentation.

[ov-config]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/config/euroc_mav/estimator_config.yaml
[ov-grid]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/Grider_FAST.h#L73
[ov-desc]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/TrackDescriptor.cpp#L355
[ov-match]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/TrackDescriptor.cpp#L480
[orb-mono]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Monocular-Inertial/EuRoC.yaml
[orb-stereo]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/EuRoC.yaml
[orb-extractor]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/ORBextractor.cc
[orb-tracking]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc
[orb-matcher-h]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/ORBmatcher.h#L40
[orb-matcher]: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/ORBmatcher.cc
[vins-config]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/config/euroc/euroc_config.yaml
[vins-tracker]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/feature_tracker/src/feature_tracker.cpp
[vins-parameters]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/feature_tracker/src/parameters.cpp#L65
[vins-loop]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/pose_graph/src/keyframe.cpp#L87
[gvins-config]: https://github.com/HKUST-Aerial-Robotics/GVINS/blob/d2cf40b49c6eb0e6ad3caa4f613713983be3fd74/config/visensor_f9p/visensor_left_f9p_config.yaml
[gvins-tracker]: https://github.com/HKUST-Aerial-Robotics/GVINS/blob/d2cf40b49c6eb0e6ad3caa4f613713983be3fd74/feature_tracker/src/feature_tracker.cpp
[gvins-parameters]: https://github.com/HKUST-Aerial-Robotics/GVINS/blob/d2cf40b49c6eb0e6ad3caa4f613713983be3fd74/feature_tracker/src/parameters.cpp#L65
[cv-api]: https://github.com/opencv/opencv/blob/fe38fc608f6acb8b68953438a62305d8318f4fcd/modules/features2d/include/opencv2/features2d.hpp#L460
[cuda-api]: https://github.com/opencv/opencv_contrib/blob/d99ad2a188210cc35067c2e60076eed7c2442bc3/modules/cudafeatures2d/include/opencv2/cudafeatures2d.hpp#L463
