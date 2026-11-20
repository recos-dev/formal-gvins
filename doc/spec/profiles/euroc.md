# EuRoC development acquisition profile

**Status:** Draft; frontend/adapter profile, not an estimator scoring baseline.

**Requirements:** [V-ACQUISITION](../requirements/vision/acquisition.md),
[V-FRAMES](../requirements/common/frames.md).

## Current implementation binding

The [reader](../../../src/dataset/euroc.cpp) accepts an ASL/CSV `mav0` directory;
download identity is controlled by the [dataset guide](../../guides/datasets.md).
It reads both cameras, IMU and processed reference truth. The first frontend
executable processes cam0 only; stereo association and IMU fusion are absent.

| Quantity | Interpretation / validation |
|---|---|
| Time | Raw integer uint64 nanoseconds in the recording's clock; each CSV strictly increasing. No arrival-time substitution or implicit inter-stream resampling. |
| Camera | Pinhole/radial-tangential model; finite positive focal lengths, declared resolution and rigid sensor-to-body T_BS interpreted as T_BC. |
| Body/IMU binding | IMU T_BS must match identity within the reader's declared `1e-9` matrix-norm tolerance; unsupported offsets fail. This tolerance is format validation, not physical calibration accuracy. |
| Images | On-demand grayscale, declared dimensions; undistort using unchanged K/optical axes, linear remap and constant border. Valid mask admits mapped image locations, not an entire descriptor patch guarantee. |
| IMU | Angular rate in rad/s then specific force in m/s^2, calibrated IMU axes; metadata noise densities/random walks are preserved, not converted to sample variances. |
| Reference truth | Processed visual-inertial, 17 columns, Hamilton `[w,x,y,z]` q_RB; position/velocity in recording frame R. Require finite nonzero quaternion coefficients and supported identity reference sensor/body transform. Preserve coefficients without normalization. |

Malformed/empty streams, non-finite numeric data, unsupported models/transforms,
bad image dimensions/types and image paths escaping the camera data directory
raise host errors. Errors shall not be presented as valid zero measurements.
Whole-sequence metadata loading returns a complete value or fails; it does not
modify a caller's existing estimator. Camera calibration/noise IDs in the demo
are local to a run, not global database identities.

## Local VINS target

The [local VINS profile](local-vins.md) selects original distorted grayscale
images and raw-pixel observations. Before estimator integration, the reader path
must expose those images with their original K, radial-tangential coefficients,
dimensions and T_BC, without the current full-image remap. Any input mask must
describe valid pixels in that raw image, not reuse the remapped valid mask.
Pointwise undistortion belongs to geometry processing, preserving raw observations.

The implementation/table above still describe the remapped-image frontend.
Existing fixtures and benchmark outputs cover that path; they do not establish
the new binding. Add raw-image/calibration preservation checks and qualify the
new observation noise before connecting the estimator. Merely changing a
calibration/noise ID does not convert existing observations to raw coordinates.

## Acceptance and unresolved bindings

[Reader fixtures](../../../tests/test_euroc.cpp) check timestamp precision, column
order, nonidentity camera transforms/lever arms and malformed input. Acceptance
also requires coordinate/calibration checks against the selected recording.

The raw clock epoch, exposure reference, inter-sensor clock error and IMU interval
representative must be qualified before estimator integration; monotonic CSV times
alone do not establish those physical facts. The reader performs no truth/frame
alignment, interpolation or scoring. W has gravity along positive z and need not
equal R. A scoring profile must declare R-to-W alignment, valid truth intervals,
interpolation/normalization and error metric conventions before reporting RMSE.
The current remapped-image noise and the selected raw-image ORB localization
noise both remain unqualified; they are separate evidence domains. This profile
therefore supports frontend experiments without claiming readiness for a complete
MSCKF regression.
