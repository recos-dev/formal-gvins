# H-RECORDING: Host sensor recording access

**ID:** `H-RECORDING`

**Status:** Draft

**Applies to:** Offline HKUST GVINS bag conversion and sequential host access.

**Uses:** [C-CPP-SUBSET](../common/cpp-subset.md),
[V-ACQUISITION](../vision/acquisition.md).

## Basis

Expose camera, IMU and genuine receiver GNSS data without requiring ROS at runtime.
Retain all message fields so initial PVT use does not discard future raw-GNSS
inputs. The [host acquisition architecture](../../architecture/sensor-replay.md)
allocates conversion, source access and publication separately.

## Required behavior

The converter shall read embedded message definitions, retain every topic and
selected message, and preserve native message timestamps, receiver flags, numeric
values and raw image bytes. It shall identify each stream by connection ID, topic,
type and message definition. Unsupported message schemas shall remain accessible
as structured fields; failure to decode shall terminate conversion, not skip data.

Recording timestamps shall remain integer nanoseconds, ordered nondecreasingly.
Each converted event shall have a contiguous sequence number, preserving the
reader's order for equal timestamps. Equal-time ordering does not establish a
physical causal order between independent sensors. Optional clipping shall use
an explicit half-open interval from the first recording timestamp; the manifest
shall identify that selection and actual per-stream counts, including absent data.

The output shall contain a versioned manifest, event records and original image
buffers. The manifest shall identify source/output byte counts and SHA-256 hashes
and copied calibration files. Conversion shall refuse existing output and publish
its directory only after successful completion. Inputs shall remain immutable.
NaN and infinities shall have explicit encodings; NaN payload bits are not retained.
Raw image encoding, dimensions, stride, padding and endianness shall be preserved.
No rectification, interpolation, clock alignment or frame conversion is implied.

The host source shall provide ordered owning events, stream metadata, and access
to all original fields. Typed IMU, image and GNSS PVT access shall preserve original
units, axes, times and validity/quality fields. PVT access shall distinguish
ellipsoid and mean-sea-level heights, expose NED velocity, and keep receiver
accuracy estimates separate from covariance. Invalid fixes and nonfinite sensor
values shall remain visible; they are not admitted estimator measurements.
Known measurements shall be decoded once by the dataset source and remain
available as typed values without consulting schema strings or parsing again.
The common source/event interface shall not depend on JSON or ROS types. Recording
provenance and auxiliary fields shall remain accessible through the dataset layer,
including after the source has been destroyed. Native-field access may parse the
retained record on demand; it shall not change the typed measurement or source.

A source shall distinguish EOF from invalid format, sequence/time reversal,
unknown stream ID, truncated files, oversized records/images and invalid image
extents. Failure shall be terminal and preserve the caller's output. EOF shall
also preserve output. Reading shall be sequential with bounded input record/image
sizes, without loading an entire recording. The source is single-owner and does
not provide thread-safe concurrent reads or implicit rewind.

The C++ implementation shall be a host-only optional target. ROS, image decoding,
JSON and operating-system dependencies shall not enter numerical core interfaces.
Calibration availability does not establish clock synchronization, exposure/noise
qualification, or a supported numerical camera model.

## Acceptance and verification

- Fixture conversion: compare original timestamps and IMU/PVT fields, integer
  epochs beyond exact double precision, invalid-fix flags, nonfinite values,
  unknown message types, image padding, clipping, calibration and output hashes.
- Reader: test valid streams, equal timestamps, bad sequence/time/ID, malformed
  image extents, truncated/missing records and counts, terminal error and unchanged
  output on error/EOF. Verify typed PVT units/flags and full raw field access.
- Automated integration: create a small bag, run the production converter, then
  the C++ source and publication loop. Verify typed/raw values, exact offsets,
  invalid-fix preservation, image padding and ownership after source destruction.
- Integration: convert actual HKUST data and access images, IMU, PVT and available
  auxiliary GNSS/trigger messages. Identify full recording versus selected clip
  and input checksum qualification with the execution results.

These checks establish data transport, not GNSS fusion, physical calibration,
noise qualification or independent navigation ground truth. The streaming reader
checks structure and file sizes, not cryptographic integrity; evidence reuse must
check the recorded hashes against immutable input/output files.
