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
