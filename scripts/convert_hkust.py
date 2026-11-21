#!/usr/bin/env python3
"""Convert a HKUST GVINS bag to a ROS-independent, arrival-ordered recording."""

import argparse
from collections import Counter
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile


KINDS = {
    "sensor_msgs/msg/Image": "image",
    "sensor_msgs/msg/Imu": "imu",
    "sensor_msgs/msg/NavSatFix": "gnss_fix",
    "gnss_comm/msg/GnssPVTSolnMsg": "gnss_pvt",
    "gnss_comm/msg/GnssMeasMsg": "gnss_observations",
    "gnss_comm/msg/GnssEphemMsg": "ephemeris",
    "gnss_comm/msg/GnssGloEphemMsg": "glonass_ephemeris",
    "gnss_comm/msg/StampedFloat64Array": "ionosphere",
    "gnss_comm/msg/GnssTimePulseInfoMsg": "time_pulse",
    "gvins/msg/LocalSensorExternalTrigger": "external_trigger",
    "gvins/msg/VISensorExternalTrigger": "external_trigger",
}
MAX_RECORD_BYTES = 4 * 1024 * 1024
MAX_IMAGE_BYTES = 16 * 1024 * 1024
MAX_STREAMS = 256


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def json_value(value):
    """Preserve every message field; non-finite floats have an explicit encoding."""
    if dataclasses.is_dataclass(value):
        return {field.name: json_value(getattr(value, field.name))
                for field in dataclasses.fields(value) if not field.name.startswith("__")}
    if hasattr(value, "tolist"):
        return json_value(value.tolist())
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"$float": "nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf")}
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise ValueError(f"Unsupported message value: {type(value).__name__}")


def image_payload(message, images):
    size = len(message.data)
    if (message.width <= 0 or message.height <= 0 or message.step <= 0
            or message.step * message.height != size or size > MAX_IMAGE_BYTES):
        raise ValueError("Invalid or oversized image buffer")
    # Keep original encoding, stride, endianness and padding, without resampling.
    payload = {field.name: json_value(getattr(message, field.name))
               for field in dataclasses.fields(message)
               if field.name != "data" and not field.name.startswith("__")}
    payload["data"] = {"offset": images.tell(), "size": size}
    images.write(message.data.tobytes())
    return payload


def copy_calibration(directory, output):
    if directory is None:
        return []
    if not directory.is_dir():
        raise ValueError(f"Missing calibration directory: {directory}")
    records = []
    destination = output / "calibration"
    destination.mkdir()
    for name in ("imu.yaml", "left_camera.yaml", "right_camera.yaml"):
        source = directory / name
        if not source.is_file():
            raise ValueError(f"Missing calibration: {source}")
        shutil.copyfile(source, destination / name)
        records.append({"path": f"calibration/{name}", "sha256": sha256(destination / name)})
    return records


def describe_streams(reader):
    streams = [{"id": item.id, "topic": item.topic, "schema": item.msgtype,
                "kind": KINDS.get(item.msgtype, "other"),
                "schema_digest": item.digest, "definition": item.msgdef.data}
               for item in sorted(reader.connections, key=lambda item: item.id)]
    if len(streams) > MAX_STREAMS:
        raise ValueError("Too many recording streams")
    if not {"imu", "image", "gnss_pvt"}.issubset({item["kind"] for item in streams}):
        raise ValueError("Expected HKUST camera, IMU and GNSS PVT streams")
    return streams


def write_events(reader, directory, duration_ns):
    """Stream messages and image bytes; return the actual selection statistics."""
    end = None if duration_ns is None else reader.start_time + duration_ns
    first = None
    last = None
    count = 0
    counts = Counter()
    with (
        (directory / "events.jsonl").open("w", encoding="utf-8") as events,
        (directory / "images.bin").open("wb") as images,
    ):
        for connection, timestamp, raw in reader.messages(stop=end):
            if timestamp < 0 or timestamp > 2**64 - 1 or (last is not None and timestamp < last):
                raise ValueError("Invalid or reversed recording timestamp")
            message = reader.deserialize(raw, connection.msgtype)
            payload = (image_payload(message, images) if connection.msgtype == "sensor_msgs/msg/Image"
                       else json_value(message))
            record = {"sequence": count, "stream_id": connection.id,
                      "record_time_ns": timestamp, "payload": payload}
            line = json.dumps(record, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
            if len(line) > MAX_RECORD_BYTES:
                raise ValueError("Oversized event record")
            events.write(line + "\n")
            counts[str(connection.id)] += 1
            first = timestamp if first is None else first
            last = timestamp
            count += 1
            if count % 10000 == 0:
                print(f"Converted {count} events", flush=True)
    if not count:
        raise ValueError("Empty recording window")
    return {"start_time_ns": first, "end_time_ns": last, "event_count": count, "stream_counts": counts}


def build_manifest(bag, directory, streams, summary, calibration_files, duration_ns):
    return {
        "format": "formal-gvins.sensor-recording", "version": 1,
        "source": {"path": bag.name, "bytes": bag.stat().st_size, "sha256": sha256(bag)},
        "time_basis": "recorded_arrival",
        "start_time_ns": summary["start_time_ns"], "end_time_ns": summary["end_time_ns"],
        "event_count": summary["event_count"],
        "stream_counts": {str(item["id"]): summary["stream_counts"][str(item["id"])] for item in streams},
        "streams": streams,
        "max_record_bytes": MAX_RECORD_BYTES, "max_image_bytes": MAX_IMAGE_BYTES,
        "calibration": calibration_files,
        "files": {name: {"bytes": (directory / name).stat().st_size, "sha256": sha256(directory / name)}
                  for name in ("events.jsonl", "images.bin")},
        "selection": {"duration_ns": duration_ns, "all_topics": True},
    }


def write_manifest(directory, manifest):
    text = json.dumps(manifest, indent=2, ensure_ascii=True) + "\n"
    if len(text) > MAX_RECORD_BYTES:
        raise ValueError("Oversized recording manifest")
    (directory / "manifest.json").write_text(text, encoding="utf-8")


def convert(bag, output, calibration=None, duration_ns=None):
    try:
        from rosbags.highlevel import AnyReader, AnyReaderError
        from rosbags.serde import SerdeError
    except ImportError as error:
        raise RuntimeError("Install dependencies: python3 -m pip install -r scripts/requirements-datasets.txt") from error
    if output.exists():
        raise ValueError(f"Refusing to overwrite {output}")
    if duration_ns is not None and duration_ns <= 0:
        raise ValueError("Duration must be positive")
    source_stat = bag.stat()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    try:
        calibration_files = copy_calibration(calibration, temporary)
        with AnyReader([bag]) as reader:
            streams = describe_streams(reader)
            summary = write_events(reader, temporary, duration_ns)
        manifest = build_manifest(bag, temporary, streams, summary, calibration_files, duration_ns)
        final_stat = bag.stat()
        if (source_stat.st_size, source_stat.st_mtime_ns) != (final_stat.st_size, final_stat.st_mtime_ns):
            raise ValueError("Source changed during conversion")
        write_manifest(temporary, manifest)
        # Publish a complete recording only; failed conversion never becomes an input.
        if output.exists():
            raise ValueError(f"Output appeared during conversion: {output}")
        temporary.rename(output)
        return manifest
    except (AnyReaderError, SerdeError) as error:
        raise ValueError(f"Cannot decode bag: {error}") from error
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, help="default: bag path without .bag")
    parser.add_argument("--calibration", type=Path, help="default: visensor_parameters next to bag, if present")
    parser.add_argument("--duration", type=int, help="convert the first N seconds; default: entire recording")
    args = parser.parse_args()
    bag = args.bag.resolve(strict=True)
    output = args.output if args.output is not None else bag.with_suffix("")
    calibration = args.calibration
    if calibration is None and (bag.parent / "visensor_parameters").is_dir():
        calibration = bag.parent / "visensor_parameters"
    manifest = convert(bag, output, calibration, None if args.duration is None else args.duration * 10**9)
    print(f"Converted {manifest['event_count']} events across {len(manifest['streams'])} streams: {output}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
