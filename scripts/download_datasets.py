#!/usr/bin/env python3
"""Download selected development recordings and their supporting files."""

import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile


REPO_DIR = Path(__file__).resolve().parents[1]
CATALOG = REPO_DIR / "configs" / "datasets.json"
DATA_DIR = REPO_DIR / "datasets"


def list_datasets(catalog):
    for dataset, description in catalog.items():
        print(f"{dataset}: {description['source']}")
        print(f"  Terms: {description['license']}")
        if description.get("support"):
            print(f"  {dataset}/support  (calibration and supporting files)")
        for sequence, files in description["sequences"].items():
            all_files = description.get("support", []) + files
            size = sum(item.get("bytes", 0) for item in all_files)
            extra = " + files of unlisted size" if any("bytes" not in item for item in all_files) else ""
            print(f"  {dataset}/{sequence}  (~{size / 1e9:.2f} GB{extra})")


def select_files(catalog, selections):
    files = {}
    for selection in selections:
        dataset, separator, sequence = selection.partition("/")
        if not separator or dataset not in catalog:
            raise ValueError(f"unknown selection: {selection}; see --list")
        description = catalog[dataset]
        selected = list(description.get("support", []))
        if sequence != "support":
            if sequence not in description["sequences"]:
                raise ValueError(f"unknown selection: {selection}; see --list")
            selected += description["sequences"][sequence]
        elif not selected:
            raise ValueError(f"{dataset} has no separately published support files")
        for item in selected:
            previous = files.setdefault(item["path"], item)
            if previous != item:
                raise ValueError(f"conflicting catalogue entries: {item['path']}")
    return list(files.values())


def check_file(path, item):
    size = path.stat().st_size
    if not size or ("bytes" in item and size != item["bytes"]):
        raise ValueError(f"{path}: unexpected size {size}; expected {item.get('bytes', 'nonzero')}")
    with path.open("rb") as stream:
        prefix = stream.read(512)
    kind = item["format"]
    if kind == "rosbag" and not prefix.startswith(b"#ROSBAG V2.0\n"):
        raise ValueError(f"{path}: not a ROS1 bag (possibly a download error page)")
    if kind == "zip" and not zipfile.is_zipfile(path):
        raise ValueError(f"{path}: not a complete ZIP archive")
    if kind == "text" and (b"\x00" in prefix or b"<html" in prefix.lower()
                           or b"<!doctype html" in prefix.lower()):
        raise ValueError(f"{path}: invalid text download")
    if kind not in ("rosbag", "zip", "text"):
        raise ValueError(f"unknown file format: {kind}")

    sha256 = hashlib.sha256()
    md5 = hashlib.md5(usedforsecurity=False) if "md5" in item else None
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            sha256.update(block)
            if md5 is not None:
                md5.update(block)
    digest = sha256.hexdigest()
    if "sha256" in item and digest != item["sha256"]:
        raise ValueError(f"{path}: SHA-256 mismatch")
    if md5 is not None and md5.hexdigest() != item["md5"]:
        raise ValueError(f"{path}: publisher MD5 mismatch")
    return {"url": item["url"], "bytes": size, "sha256": digest}


def save_receipt(path, receipt):
    temporary = path.with_name(path.name + ".part")
    temporary.write_text(json.dumps(receipt, indent=2) + "\n")
    temporary.replace(path)


def transfer(url, partial):
    command = [
        "curl", "--disable", "--fail", "--location", "--progress-bar",
        "--retry", "3", "--retry-delay", "2", "--connect-timeout", "30",
        "--speed-time", "120", "--speed-limit", "1024",
        "--output", str(partial),
    ]
    result = subprocess.run(command + ["--continue-at", "-", url], check=False)
    if result.returncode == 33:
        # Some publisher-generated ZIPs do not support byte ranges.
        print("Server cannot resume this file; restarting its partial download.", flush=True)
        result = subprocess.run(command + [url], check=False)
    if result.returncode:
        raise RuntimeError(f"curl failed ({result.returncode}); rerun to resume {partial}")


def download_file(item, directory, verify_only=False):
    path = directory / item["path"]
    if not path.resolve().is_relative_to(directory.resolve()):
        raise ValueError(f"dataset path leaves the destination: {item['path']}")
    receipt_path = path.with_name(path.name + ".download.json")
    if path.exists():
        receipt = check_file(path, item)
        if receipt_path.exists():
            if json.loads(receipt_path.read_text()) != receipt:
                raise ValueError(f"{path}: differs from its download receipt; not overwriting")
        elif "sha256" not in item and "md5" not in item:
            raise ValueError(f"{path}: missing checksum baseline; not adopting an unverified file")
        elif not verify_only:
            save_receipt(receipt_path, receipt)
        print(f"Verified: {path}", flush=True)
        return
    if verify_only:
        raise ValueError(f"missing dataset file: {path}")

    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + ".part")
    size = partial.stat().st_size if partial.exists() else 0
    expected_size = item.get("bytes")
    if expected_size is not None and size > expected_size:
        raise ValueError(f"oversized partial download: {partial}; remove it before retrying")
    if expected_size is None or size != expected_size:
        if expected_size is not None and shutil.disk_usage(path.parent).free < expected_size - size:
            raise ValueError(f"insufficient free disk space for {path}")
        print(f"Downloading: {path}", flush=True)
        transfer(item["url"], partial)
    try:
        receipt = check_file(partial, item)
    except ValueError as error:
        raise ValueError(f"{error}; remove {partial} before retrying") from error
    save_receipt(receipt_path, receipt)
    partial.replace(path)
    print(f"Downloaded: {path}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selections", nargs="*", metavar="DATASET/SEQUENCE")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="list downloads and publisher terms")
    mode.add_argument("--dry-run", action="store_true", help="show files without downloading")
    mode.add_argument("--verify", action="store_true", help="check selected local files without network access")
    args = parser.parse_args(argv)
    catalog = json.loads(CATALOG.read_text())
    if args.list or (not args.selections and not args.verify and not args.dry_run):
        list_datasets(catalog)
        return
    if not args.selections:
        parser.error("select at least one DATASET/SEQUENCE; see --list")
    files = select_files(catalog, args.selections)
    for dataset in dict.fromkeys(selection.split("/")[0] for selection in args.selections):
        print(f"{dataset} terms: {catalog[dataset]['license']}", flush=True)
    if args.dry_run:
        for item in files:
            checksum = "pinned checksum" if "sha256" in item or "md5" in item else "local SHA-256 on download"
            print(f"{DATA_DIR / item['path']}  ({item.get('bytes', 'unknown')} bytes; {checksum})")
            print(f"  {item['url']}")
        return
    if not args.verify and shutil.which("curl") is None:
        raise RuntimeError("curl was not found in PATH; install curl first")
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with (DATA_DIR / ".download.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another dataset download/verification is running") from error
        for item in files:
            download_file(item, DATA_DIR, args.verify)
    print(f"Complete: {len(files)} files verified. Archives are not extracted.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted; rerun the same command to resume.", file=sys.stderr)
        sys.exit(130)
