# V-ACQUISITION: Sensor time and calibration binding

**ID:** `V-ACQUISITION`

**Status:** Draft

**Applies to:** Sensor/dataset adapters supplying local visual-inertial inputs.

**Uses:** [V-FRAMES](../common/frames.md).

## Basis

The estimator cannot recover correct time, units or extrinsic direction from
unlabelled samples. The [EuRoC development profile](../../profiles/euroc.md)
binds one file format; it is not the general sensor interface.

## Required behavior

Each stream shall identify clock/epoch/unit, sensor, calibration revision and
sample semantics. Acquisition time shall remain distinct from arrival time.
Adapters shall preserve integer timestamps until computing a bounded relative
interval; large absolute times shall not be rounded through a scalar conversion.
Cross-stream use requires a declared clock conversion and uncertainty/tolerance.

Camera input shall bind exposure reference, image dimensions, optical model,
intrinsics, camera-to-IMU extrinsics and coordinate domain. Resizing, cropping or
undistortion shall update the coordinate/noise binding consistently. Shared stereo
times shall require an explicit synchronization model. IMU input shall identify
specific force and angular rate, their units/axes, calibration, sample/interval
semantics and noise-density versus variance convention.

Malformed input, unsupported calibration/format and out-of-order data shall be
rejected or handled by a declared ordering policy before estimator consumption.
Adapters shall not replace missing samples with zero, fabricate synchronization,
invert extrinsics implicitly or treat IDs as validated noise covariance.
Offline reference truth shall retain its source frame, time and coefficient
semantics; alignment, interpolation and quaternion normalization for scoring
shall be explicit transformations with their own acceptance.

## Acceptance and verification

Test exact timestamp ordering/relative differences, nonidentity extrinsics with
lever arm, coordinate transformations and unsupported/malformed data. Verify
measurement covariance in the same residual domain. Source tests cannot prove
physical calibration or clock accuracy. Missing exposure/noise qualification
blocks estimator use even when a frontend-only recording can be replayed.

## Open decisions

Deployment exposure reference, clock-error bounds and calibrated measurement-noise
values remain profile decisions. The EuRoC adapter currently implements parsing
and image conversion, not temporal estimator integration or truth scoring.
