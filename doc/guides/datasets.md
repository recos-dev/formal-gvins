# Development dataset downloads

Use Python 3.10 or later and `curl`; no pip dependencies are required. Commands
below run from the project root. The script resolves paths from its own location,
so downloads always go to the project's Git-ignored `datasets/` directory.

```bash
python3 scripts/download_datasets.py --list
python3 scripts/download_datasets.py --dry-run euroc/machine_hall
python3 scripts/download_datasets.py euroc/machine_hall
```

Select recordings as needed; there is no automatic download of the full corpus.
Sizes are decimal GB, before extraction. Downloads are sequential.

| Selection | Approximate recording size | Contents and intended use |
|---|---:|---|
| `euroc/machine_hall` | 12.68 GB | Publisher archive containing the Machine Hall sequences, including the proposed development and stress sequences. |
| `euroc/vicon_room1` | 6.04 GB | Vicon Room 1 archive, including the proposed held-out sequence. |
| `euroc/vicon_room2` | 6.01 GB | Additional room sequences. |
| `gvins/sports_field` | 22.01 GB | Initial raw-GNSS/visual/inertial development recording. |
| `gvins/complex_environment` | 17.34 GB | Reserved GNSS integration evaluation recording. |
| `gvins/urban_driving` | 35.89 GB | Additional driving recording. |
| `mars-lvig/HKairport_GNSS01` | 20.18 GB | Initial airborne GNSS candidate. |
| `mars-lvig/HKisland_GNSS02` | 12.09 GB | Different-scene airborne evaluation candidate. |
| `urbannav/Medium-Urban-1` | 35.24 GB | Later urban stress data, with separate GNSS ZIP, calibration and reference files. |
| `urbannav/Tunnel-1` | 18.04 GB | Later GNSS-outage stress data, with separate GNSS ZIP, calibration and reference files. |

The catalogue includes the supporting calibration and ephemeris files published
in the GVINS repository, and the MARS HK GNSS camera/LiDAR calibration and CAD
extrinsics. They download automatically with their recordings. To retrieve just
those small supporting collections:

```bash
python3 scripts/download_datasets.py gvins/support mars-lvig/support
```

GNSS and reference topics remain inside the GVINS/MARS bags; downloading does
not extract them or establish their timing/ground-truth validity. EuRoC archives
retain the publisher's directory structure and associated calibration/reference
data. ZIP archives are not automatically extracted. Dataset-specific decoding,
frame conversion and regression qualification remain separate work; see the
[selection and regression proposal](../analysis/offline-datasets.md).

## Reuse and integrity

Rerun the same command after interruption to resume `.part` downloads. Servers
without range support restart that partial transfer. A file becomes available
under its final name only after size, format and available checksum checks.
Existing files are verified and reused; corrupted completed files are not
silently overwritten. Follow the reported path to remove a failed partial or
damaged file before explicitly downloading it again.

```bash
python3 scripts/download_datasets.py --verify euroc/machine_hall
python3 scripts/download_datasets.py --verify gvins/support mars-lvig/support
```

`--verify` does not access the network. Download receipts (`*.download.json`)
record the URL, byte count and SHA-256 alongside each local file. One download
or verification process owns the dataset directory at a time.

[The catalogue](../../configs/datasets.json) pins EuRoC publisher MD5 checksums,
GVINS Hugging Face revision/SHA-256 values, and hashes of inspected supporting
files. MARS and UrbanNav recording hashes are not publisher-pinned: their first
download establishes a local SHA-256 receipt. That detects later local changes
but does not establish source authenticity or a shared regression baseline.
Before adopting those recordings for regression, review and pin their complete
file hashes. Dropbox-generated GNSS ZIPs likewise use local receipts.

Publisher terms are linked in the catalogue and printed by the script. The
project's Apache-2.0 license does not relicense downloaded datasets. Do not
commit recordings, extracted images or local receipts.

## Downloader checks

```bash
python3 tests/test_download_datasets.py
```

These tests use a small localhost HTTP fixture to exercise transfer, resume,
validation, corruption and error handling. They do not download public datasets
or qualify an estimator's accuracy.

## HKUST conversion and local publication

Install the offline converter's Python dependencies; ROS is not required:

```bash
python3 -m pip install -r scripts/requirements-datasets.txt
python3 scripts/download_datasets.py gvins/sports_field
python3 scripts/convert_hkust.py datasets/gvins/sports_field.bag
```

The default output is `datasets/gvins/sports_field/`. All topics are converted;
PVT is not substituted for raw GNSS. Supporting calibration is copied from the
adjacent `visensor_parameters/` directory. To select a short development window:

```bash
python3 scripts/convert_hkust.py datasets/gvins/sports_field.bag \
    --duration 20 --output datasets/gvins/sports_field-20s
```

Existing output is refused, not deleted. Use an explicit new output directory or
remove a known disposable output yourself. Conversion writes a temporary sibling
and publishes it only after completion. Full conversion requires storage for the
bag and another set of uncompressed image bytes plus metadata. `--duration` limits
converted events, but source hashing still reads the entire original bag.

| Output | Contents |
|---|---|
| `manifest.json` | Format version, source/file hashes and sizes, selection, time range, counts, per-stream original message definitions, calibration file identity. |
| `events.jsonl` | One ordered event per line: contiguous `sequence`, `stream_id`, integer `record_time_ns`, and decoded native `payload` fields. |
| `images.bin` | Original image bytes, concatenated in event order. Image payload `data` contains byte `offset` and `size`; stride/encoding/endianness remain in the payload. |
| `calibration/` | Original IMU and left/right camera YAML files when supplied. |

Nonfinite floats use `{"$float":"nan"}`, `{"$float":"+inf"}` or
`{"$float":"-inf"}`. Typed decoding restores their numerical classification;
NaN payload bits are not preserved. Integer timestamps and identifiers never
round through floating-point seconds. A clipped interval may contain zero samples
from a declared topic; the manifest's counts, not schema presence, describe data.

Install a compiler/CMake and the external nlohmann/json development package
(`nlohmann-json3-dev` on Ubuntu), then build the optional host target:

```bash
cmake -S . -B build/recording \
    -DFORMAL_GVINS_BUILD_DATASET_REPLAY=ON \
    -DFORMAL_GVINS_BUILD_EIGEN_BACKEND=OFF -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build/recording --parallel "$(nproc)"
ctest --test-dir build/recording --output-on-failure
```

This target needs no OpenCV, Eigen, ROS or MAVLink. With an externally installed
JSON package, provide its `nlohmann_json_DIR` if CMake cannot find it.

```bash
# Read/validate all records immediately, without pacing.
./build/recording/replay_sensors datasets/gvins/sports_field-20s --inspect

# Publish locally at 1x arrival timing; preparation precedes simulated boot.
./build/recording/replay_sensors datasets/gvins/sports_field-20s

# Also record observed delivery times; the parent directory must exist.
mkdir -p output
./build/recording/replay_sensors datasets/gvins/sports_field-20s \
    --timing output/sports-field-timing.csv
```

The example receives every event and prints topic counts. It does not run ORB,
MSCKF or GNSS fusion. `io::Replay::next()` is the local delivery point: applications
can route its owning event to their own consumers. `event.measurement` holds one
of `ImuMeasurement`, `ImageMeasurement`, `GnssPvt` or `AuxiliaryMeasurement`.
Known values are decoded once in `RecordingSource`; consumers use ordinary
`std::get_if` without parsing JSON or matching schema strings:

```cpp
if (auto const * imu = std::get_if<formal_gvins::io::ImuMeasurement>(&event.measurement))
{
    // Read imu->time_ns, imu->angular_velocity and imu->linear_acceleration.
}
```

Images own their original bytes in `ImageMeasurement::data`. Every recording
event also retains its original event line in `source_record`. When full native
fields are needed, `dataset::recording_fields(event)` decodes that retained record
and returns its payload, including satellite arrays, ephemerides, ionosphere,
PPS, triggers and IMU fields outside the common measurement. This optional access
is independent of typed measurement access. `RecordingSource::manifest()` provides
the full source schema/calibration descriptions. Events remain usable after their
source is destroyed. Another source may publish typed events without a serialized
record; the recording-specific accessor does not apply to those events.

For record time r, first record r0 and simulated boot t0, the publication deadline
is t0 + (r - r0). Both native measurement time and recorded arrival time remain
unchanged. This replays availability; it does not synchronize camera/IMU/GNSS
clocks. The default reports lateness without dropping events. Library callers
may set `ReplayOptions::maximum_lateness` to a positive budget to fail before
an overdue delivery. CSV timing includes caller/I/O contention and is not WCET.
The timing report refuses overwrite. Verify recording hashes before reusing
results as regression evidence; the C++ reader checks structure and size only.

The [architecture](../spec/architecture/sensor-replay.md) records SOTA comparisons,
ownership, bounds and remaining camera/time qualification. In particular, HKUST
MEI images/calibration are preserved, not silently treated as the current
pinhole/radtan frontend profile.

```bash
python3 tests/test_convert_hkust.py
```

Converter tests create small bags with embedded definitions locally. CTest also
passes the `recording_pipeline` executable to this test script: the fixture goes
through the production converter, C++ reader and paced publication, checking
native/typed values, image bytes, ownership and exact schedule offsets. Install
`scripts/requirements-datasets.txt` in the Python environment selected by CMake
before running these tests. No public dataset or ROS installation is needed.
