# Offline datasets and regression plan

**Status:** Draft proposal

**Source review date:** 2026-11-20

Select EuRoC for the first ORB/MSCKF regression suite. Qualify MARS-LVIG for
airborne GNSS integration and the HKUST GVINS dataset for raw-GNSS experiments.
Keep UrbanNav as a later urban stress suite. These serve different purposes;
no single dataset establishes frontend, inertial, GNSS and flight performance.

This is a source and calibration review, not a completed replay experiment.
Downloadable selections are defined by the [download guide](../guides/datasets.md).
Large sensor archives have not been downloaded in full or validated. No sequence is
yet admitted to an executable regression suite, and no accuracy result is
claimed. The repository currently provides mathematical layers and draft
requirements; the frontend, MSCKF implementation and GNSS profile remain work
to do. This proposal does not approve or extend those requirements.

## Selection

| Dataset | Relevant inputs | Reference trajectory | Proposed role and limitation |
|---|---|---|---|
| [EuRoC MAV](https://projects.asl.ethz.ch/datasets/euroc-mav/) | Stereo monochrome images at 20 Hz; IMU at 200 Hz | Vicon pose and Leica position measurements, with processed aligned trajectories | First ORB/VIO regression. No GNSS. Global-shutter cameras fit the proposed acquisition model. |
| [HKUST GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset) | Stereo images at 20 Hz; IMU at 200 Hz; F9P raw GNSS and receiver solutions at 10 Hz | RTK solution from the same F9P receiver | Raw-GNSS integration and availability changes. Reference and input share observations; receiver PVT cannot also be independent ground truth. |
| [MARS-LVIG](https://mars.hku.hk/dataset_setup.html) | Monocular global-shutter RGB at 10 Hz; Livox IMU at 200 Hz; F9P raw GNSS and receiver solutions | Separate DJI M300 RTK position/velocity reference | Closest combined dataset to the UAV target. Qualify timing, reference validity and extrinsics before an accuracy gate. Only select GNSS-equipped sequences. |
| [UrbanNav Hong Kong](https://github.com/IPNL-POLYU/UrbanNavDataset) | Stereo images, IMU and raw GNSS/RINEX | SPAN-CPT GNSS/INS postprocessed reference | Later multipath, tunnel and revisit tests. ZED2 rolling shutter is outside the current global-shutter model. |
| [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) | Aerial stereo images and IMU; other sensors can be excluded | Laser-tracked prism position | Optional outdoor VIO cross-check with an external position reference. No GNSS input; not a combined GNSS suite. |

### EuRoC: establish the visual-inertial baseline

Use the native image/CSV/calibration distribution where available. Run both
single-camera and stereo profiles on the same sequences when those modes are
implemented. A dataset's stereo availability does not decide our product camera
configuration.

Proposed sequence roles are `MH_01_easy` for development, `V1_02_medium` for
held-out room evaluation, and `MH_04_difficult` for motion/track-loss stress.
Keep startup in the input; any initialization pre-roll and scoring interval
must be fixed before evaluating changes. Do not crop out a failed startup
after seeing the result.

The publisher describes separate pose and position reference instruments and
limitations in synchronization and dynamic laser tracking. Do not interpret
every processed state component as an independently measured truth.
[Dataset description](https://projects.asl.ethz.ch/datasets/euroc-mav/).
OpenVINS also identifies an orientation-reference problem in `V1_01_easy` and
provides a recomputed trajectory. Avoid that sequence for the first attitude
gate; any later use must pin the reference variant.
[OpenVINS dataset notes](https://docs.openvins.com/gs-datasets.html).

### GVINS: preserve the GNSS and camera semantics

Start with `sports_field`, then reserve `complex_environment` for evaluation.
The publisher lists approximately 22.0 GB and 17.3 GB respectively, and provides
PPS/external-trigger information, ephemerides and receiver status. `cam0` is the
right camera and `cam1` the left. The current distribution is linked through
Hugging Face. The download catalogue pins its revision and published SHA-256
values; selected file headers have been checked, but full-bag validation remains
part of dataset qualification.
[Dataset metadata](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset).

The published camera calibration uses **MEI**, not the draft's undistorted
pinhole projection. An adapter must unproject/rectify using the actual model,
bind observations to the resulting calibration, and account for transformed
measurement uncertainty. MEI `gamma1`/`gamma2` are not interchangeable with a
pinhole focal length. Fix the image conversion, valid mask and ORB coordinate
policy in the acquisition profile.
[Published calibration](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset/blob/main/data/visensor_parameters/right_camera.yaml).

For raw-GNSS evaluation, retain the shared-receiver limitation of the RTK
reference. For PVT fusion, use a separately justified reference or restrict the
test to integration/robustness claims. Locate and validate the antenna-to-IMU
lever arm; it was not established by the inspected camera/IMU calibration.

### MARS-LVIG: qualify the airborne GNSS suite

Use `HKairport_GNSS01` as the first candidate and a different scene, such as
`HKisland_GNSS02`, for held-out evaluation. The publisher lists 790 s and 465 s
respectively. Evening and featureless sequences are later stress cases.
The download catalogue uses the publisher's updated Drive folder: the selected
bags are approximately 20.18 GB and 12.09 GB. The sequence table also contains
older links; use the checked catalogue rather than assuming those links work.
[Sequence catalogue](https://mars.hku.hk/dataset.html).

Keep F9P observations/solutions as estimator inputs and DJI RTK topics as
evaluation references. The official reference includes position, velocity,
yaw and status at 5 Hz; this is not an independently measured full attitude
trajectory. The DJI L1 map accuracy experiment is a map-distance check, not a
camera/IMU trajectory accuracy bound.
[Reference definition](https://mars.hku.hk/dataset_ground_truth.html).

The publisher documents hardware synchronization and releases calibration
material, but each timestamp path and the camera/IMU/antenna transform chain
still require archive inspection.
[Synchronization](https://mars.hku.hk/dataset_sensor_synchronlization.html),
[calibration](https://mars.hku.hk/dataset_sensor_calibration.html).
A separate postprocessed [MARS-LVIG-GT repository](https://github.com/thlsealight/MARS-LVIG-GT)
uses the Livox IMU and DJI GNSS and reports estimated time alignment with about
0.1 s accuracy. Do not silently substitute it for the official reference or
treat it as independent of the input IMU. As a sensitivity illustration,
0.1 s timing error at 6 m/s corresponds to roughly 0.6 m displacement; this is
not a measured error of our estimator or a bound for every sequence.

### Deferred coverage

UrbanNav `Medium-Urban-1` (33.7 GB) and `Tunnel-1` (17 GB) are useful later
targets. The catalogue reports repeated loops, GNSS receiver issues in the
medium sequence, and only partially validated truth for `Harsh-Urban-1`.
Select receiver and valid reference intervals explicitly.
[UrbanNav catalogue and issue links](https://github.com/IPNL-POLYU/UrbanNavDataset).
ZED2 uses rolling shutter; stereo synchronization or spatial rectification does
not remove the within-frame time distortion.
[Camera manufacturer](https://support.stereolabs.com/articles/9796499033-what-kind-of-the-shutter-mode-do-the-zed-cameras-use-is-it-a-rolling-shutter-or-a-global-shutter).

For NTU VIRAL, compare the estimated prism position against the measured prism,
accounting for its approximately 0.4 m body offset. A pose-shaped message does
not imply an attitude reference.
[Publisher evaluation guide](https://ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html).

Do not use KAIST's released SLAM baseline as an accuracy oracle: the publisher
explicitly discourages that interpretation, and some early sequences lack raw
IMU acceleration/angular rate.
[KAIST format description](https://sites.google.com/view/complex-urban-dataset/format).
Other datasets need not become first-version adapters merely to enlarge coverage.

## Regression design

Keep two complementary execution paths:

| Path | What it detects | What it cannot establish |
|---|---|---|
| Images + IMU, later GNSS, through ORB and estimator | Frontend, adapter, timing and estimator interaction regressions | Mathematical correctness over all admissible inputs |
| Frozen feature observations + IMU, later GNSS | Estimator regressions isolated from feature extraction changes | ORB detection, matching or image-conversion correctness |

Frozen observations retain image/camera/track identity, acquisition time,
calibration/noise binding and original source hashes. They are derived fixtures,
not manually repaired tracks or a replacement for raw-image tests.
Use separate synthetic fixtures for Jacobians, gauge identities, covariance,
duplicate observations and failure paths. Artificial GNSS generated from a
trajectory is a simulation fixture, not real-GNSS performance evidence.

Pin OpenCV version/build, ORB parameters, image preprocessing, robust-estimation
random seeds, ordering and threading policy. Test repeatability on an identified
profile. Do not require identical ORB outputs across arbitrary CPU/GPU/OpenCV
implementations or interpret a descriptor score as geometric ground truth.

### Minimum dataset manifest

| Information | Purpose |
|---|---|
| Publisher URL/revision, file size and SHA-256, license record | Identify the exact external inputs and permissible distribution. |
| Sequence/session, input interval, initialization pre-roll, scoring interval | Reproduce execution and prevent selective omission of failures. |
| Input topics versus evaluation-only topics | Prevent reference leakage, including initialization from truth. |
| Camera model, intrinsics/extrinsics, IMU noise units, antenna/prism lever arm | Bind geometry and uncertainty to the actual hardware. |
| Clock/epoch/unit, exposure reference, time mapping, sensor rates and gaps | Associate images, IMU, GNSS and reference at acquisition time. |
| Reference source/hash, estimated quantity, frame, status mask and gap limit | Score only supported quantities over valid intervals. |
| Adapter/configuration revision and derived-fixture hashes | Detect preprocessing changes independently of estimator changes. |
| Development or held-out role; evaluator and threshold version | Separate tuning from acceptance and make baseline changes reviewable. |

For GNSS clips, retain the preceding synchronization, ephemeris and ionospheric
context needed to interpret the interval. ROS bag storage does not require ROS
inside the estimator. The offline adapter owns message decoding and conversion.
Use a bounded streaming reader rather than loading a complete bag into memory.

### Metrics and acceptance

| Scope | Proposed measurements |
|---|---|
| ORB/association | Image coverage, accepted tracks, track length, geometry inliers, reprojection residuals and time per frame. Counts alone do not prove correct associations. |
| Execution | Initialization success/time, scored trajectory coverage, dropped/rejected observations, numerical failures and resets. A stopped estimator cannot pass on its surviving prefix. |
| VIO accuracy | Position ATE/RPE and attitude error where the reference supports it. Preserve metric scale; after known frame conversion, use declared yaw/translation gauge alignment. Label any additional SE(3)-aligned benchmark separately. |
| GNSS integration | Horizontal/vertical error, RMSE, percentile and maximum error, reference coverage, outage drift and recovery. Use the declared geodetic/local transform and lever arm; do not fit the full trajectory to remove absolute position or heading errors. |
| Target execution | Latency distribution and peak memory on Orin Nano with recorded build, power mode and workload. Host replay speed is not an Orin real-time guarantee. |

Fix reference interpolation and maximum gap rules; never interpolate across a
GNSS-reference outage as if it were measured truth. Convert frames according to
[V-FRAMES](../spec/requirements/common/frames.md), including quaternion order,
gravity direction and geodetic height datum. The local world frame is not
automatically ENU or NED. Compare the same physical point on the vehicle.

Set numerical and trajectory thresholds after qualifying inputs and measuring
a reviewed baseline on development data, then freeze them before held-out
evaluation. Distinguish regression tolerance from a product accuracy target.
Do not automatically replace expected results when a change fails. Keep
frontend, local odometry and future loop-corrected map results separate; future
relocalization tests need fixed map-building/query splits and rejected-loop cases.

### Traceability and execution tiers

Link concrete tests to [V-ORB](../spec/requirements/vision/orb-frontend.md),
[V-TRACK](../spec/requirements/vision/feature-tracks.md) and
[V-FRAMES](../spec/requirements/common/frames.md). Math acceptance follows
[C-NUMERICAL-ACCEPTANCE](../spec/requirements/common/numerical-acceptance.md).
The MSCKF/GNSS performance contracts still need review; datasets do not create
approved requirement IDs or replace formal proofs.

Start with small deterministic numerical fixtures and a qualified short image
clip for routine checks. Run complete selected sequences for release/local
regression; add slower stress suites separately. Store manifests, configurations
and test definitions in Git, external sensor archives and transient run outputs
outside versioned source. Preserve identified release evidence under the
project's evidence policy. No network fetch is needed during test execution.

## Access and licensing

EuRoC now directs downloads to the
[ETH Research Collection](https://www.research-collection.ethz.ch/items/bcaf173e-5dac-484b-bc37-faf97a594f1f);
its Machine Hall archive contains 12,683,729,426 bytes (about 12.68 decimal GB).
Inspect the distribution's
rights record before redistribution or commercial use; do not infer data rights
from an implementation or dataset-tool license.

[GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset#5-license) and
[MARS-LVIG](https://mars.hku.hk/dataset.html) specify CC-BY-NC-SA-4.0 and
noncommercial restrictions. The current
[UrbanNav license statement](https://github.com/IPNL-POLYU/UrbanNavDataset#license)
explicitly includes linked dataset files under MIT. Apache-2.0 for this project
does not relicense any of these datasets. Do not vendor archives or image
fixtures until their distribution terms have been checked.

## Next actions

1. Qualify the proposed EuRoC sequences: acquire exact files, check calibration,
   timestamps, reference coverage and image decoding; define a reproducible clip
   and full-sequence manifest.
2. Decide the first GNSS profile, PVT or raw observations. Qualify a MARS-LVIG
   GNSS sequence for separate-input/reference evaluation. If timing or calibration
   cannot support an accuracy claim, keep it as a robustness experiment.
3. Qualify GVINS for raw-observation integration, including MEI conversion,
   PPS alignment, receiver status and antenna geometry. Preserve the correlated
   reference limitation in every reported result.
4. Implement replay and both regression paths once the corresponding frontend
   and estimator exist. Establish reviewed thresholds before expanding datasets.
