"""Run independent C++ ORB workloads on a pinned EuRoC window and report results."""

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import sys

from feature_metrics import Match, Point, score_pair


BENCHMARK_DIR = Path(__file__).resolve().parent
REPO_DIR = BENCHMARK_DIR.parent.parent
PROTOCOL = "euroc-orb-v2"
WORKLOADS = ("temporal", "translation")
OUTPUT_FILES = (
    "temporal.json", "translation.json", "resolved-profile.json", "summary.json", "frames.csv",
    "temporal.png", "translation.png", "opencv-build.txt", "raw.json",  # Retire the former interleaved output.
)
CSV_FIELDS = (
    "workload", "frame_index", "time_ns", "status", "error", "features", "ratio_passed", "inliers",
    "geometry_valid", "read_remap_ms", "frontend_ms", "source_points", "target_points",
    "eligible_source_points", "eligible_target_points", "geometric_pairs", "accepted_pairs", "correct_pairs",
    "evaluated_pairs", "repeatability", "precision", "recall", "rmse_all_px", "rmse_correct_px",
)
QUALITY_METRICS = ("repeatability", "precision", "recall", "rmse_all_px", "rmse_correct_px")
ERRORS = (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.SubprocessError)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_profile(path):
    profile = json.loads(path.read_text())
    for name in ("camera", "first_frame", "frame_count", "warmup_frames", "translation_x_px", "translation_y_px", "rng_seed"):
        if type(profile[name]) is not int or not -(2**31) <= profile[name] < 2**31:
            raise ValueError(f"{name} must be a signed 32-bit integer")
    tolerance = profile["tolerance_px"]
    if (profile["protocol"] != PROTOCOL or profile["camera"] != 0 or not isinstance(profile["dataset"], str)
            or profile["first_frame"] < 0 or profile["frame_count"] < 2
            or profile["first_frame"] + profile["frame_count"] >= 2**31
            or not 1 <= profile["warmup_frames"] < profile["frame_count"]
            or type(tolerance) not in (int, float) or not math.isfinite(tolerance) or tolerance < 0):
        raise ValueError("Unsupported benchmark profile or invalid frame/tolerance range")
    options = profile["tracking"]
    integer_limits = {
        "feature_count": (1, 10000), "pyramid_levels": (1, 16), "fast_threshold": (0, 255),
        "feature_capacity": (1, 10000), "grid_columns": (1, 64), "grid_rows": (1, 64),
        "ransac_iterations": (1, 10000),
    }
    for name, (low, high) in integer_limits.items():
        if type(options[name]) is not int or not low <= options[name] <= high:
            raise ValueError(f"Invalid tracking option: {name}")
    for name in ("scale_factor", "ratio", "epipolar_threshold_px", "confidence"):
        if type(options[name]) not in (int, float) or not math.isfinite(options[name]):
            raise ValueError(f"Invalid tracking option: {name}")
    if (not 1 < options["scale_factor"] <= 2 or not 0 < options["ratio"] < 1
            or not 0 < options["confidence"] < 1 or options["epipolar_threshold_px"] <= 0
            or not options["grid_columns"] * options["grid_rows"] <= options["feature_capacity"] <= options["feature_count"]):
        raise ValueError("Invalid tracking option range or capacity")
    return profile


def input_manifest(dataset, profile):
    camera = dataset / "cam0"
    with (camera / "data.csv").open() as stream:
        images = list(csv.reader(line for line in stream if not line.startswith("#")))
    first, count = profile["first_frame"], profile["frame_count"]
    if first + count > len(images):
        raise ValueError("Dataset does not contain the requested frame window")
    frames = []
    for index in range(first, first + count):
        timestamp, name = images[index]
        path = (camera / "data" / name).resolve()
        path.relative_to((camera / "data").resolve())
        frames.append({"frame_index": index, "time_ns": str(int(timestamp)),
                       "file": path.relative_to(dataset).as_posix(), "sha256": sha256(path)})
    metadata = {name: sha256(camera / name) for name in ("data.csv", "sensor.yaml")}
    return {"directory": str(dataset), "cam0_metadata": metadata, "frames": frames}


def timing_summary(values):
    if not values:
        return {"samples": 0, "median_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("Invalid latency sample")
    ordered = sorted(values)
    return {"samples": len(values), "median_ms": statistics.median(values),
            "p95_ms": ordered[math.ceil(0.95 * len(values)) - 1],
            "p99_ms": ordered[math.ceil(0.99 * len(values)) - 1], "max_ms": ordered[-1]}


def points(rows, dx=0, dy=0):
    result = []
    for x, y, eligible in rows:
        if type(eligible) is not int or eligible not in (0, 1):
            raise ValueError("Native eligibility must be 0 or 1")
        result.append(Point(x + dx, y + dy, bool(eligible)))
    return result


def scheduled_rows(manifest, workload):
    return [dict(workload=workload, frame_index=frame["frame_index"], time_ns=frame["time_ns"],
                 status="missing", error="No valid native result") for frame in manifest["frames"]]


def case_counts(rows):
    successful = sum(row["status"] == "success" for row in rows)
    return {"scheduled_cases": len(rows), "successful_cases": successful,
            "failed_cases": sum(row["status"] == "failed" for row in rows),
            "missing_case_ids": [str(row["frame_index"]) for row in rows if row["status"] == "missing"],
            "success_rate": successful / len(rows) if rows else None}


def validate_native(raw, profile, manifest, workload, execution):
    if raw["protocol"] != PROTOCOL or raw["workload"] != workload or raw["execution"] != execution:
        raise ValueError("Native protocol, workload or execution differs from the request")
    for name, expected in profile["tracking"].items():
        # scale_factor and ratio are binary32 in the production API; the other real options are binary64.
        tolerance = 1e-7 if name in ("scale_factor", "ratio") else 1e-12
        if not math.isclose(raw["options"][name], expected, rel_tol=tolerance, abs_tol=0):
            raise ValueError(f"Native tracking option differs from the requested profile: {name}")
    expected = {frame["frame_index"]: frame["time_ns"] for frame in manifest["frames"]}
    frames = {}
    last_index = profile["first_frame"] - 1
    for frame in raw["frames"]:
        index = frame["frame_index"]
        if (type(index) is not int or index not in expected or index <= last_index
                or frame["time_ns"] != expected[index]):
            raise ValueError("Unexpected, duplicate, reordered or incorrectly timestamped native frame")
        if frame["status"] not in ("success", "failed"):
            raise ValueError("Unknown native frame status")
        last_index = index
        frames[index] = frame
    return frames


def temporal_metrics(frame, capacity):
    observations = frame["observations"]
    if (type(frame["features"]) is not int or not 0 <= frame["features"] <= capacity
            or len(observations) != frame["features"]
            or len({observation[2] for observation in observations}) != len(observations)
            or any(not math.isfinite(u) or not math.isfinite(v) or not isinstance(track, str)
                   or not track.isdecimal() or int(track) <= 0 for u, v, track in observations)
            or not 0 <= frame["inliers"] <= min(frame["features"], frame["ratio_passed"])):
        raise ValueError("Malformed native temporal observations")
    return {name: frame[name] for name in ("features", "ratio_passed", "inliers", "geometry_valid", "read_remap_ms", "frontend_ms")}


def translation_metrics(frame, profile):
    scores = score_pair(points(frame["source"], profile["translation_x_px"], profile["translation_y_px"]),
                        points(frame["target"]), [Match(*pair) for pair in frame["matches"]], profile["tolerance_px"])
    return dict(asdict(scores), geometry_valid=frame["geometry_valid"], frontend_ms=frame["frontend_ms"])


def summarize(rows, profile, workload):
    summary = case_counts(rows)
    successful = [row for row in rows if row["status"] == "success"]
    timed = [row for row in successful if row["frame_index"] >= profile["first_frame"] + profile["warmup_frames"]]
    summary["frontend"] = timing_summary([row["frontend_ms"] for row in timed])
    if workload == "temporal":
        summary.update(
            read_remap=timing_summary([row["read_remap_ms"] for row in timed]),
            precision=None, recall=None, rmse_px=None,
            reference="No per-feature ground truth for real adjacent EuRoC frames",
            empty_frames=sum(row["features"] == 0 for row in successful),
            geometry_invalid_frames=sum(row["geometry_valid"] == 0 for row in successful),
            mean_features=statistics.fmean(row["features"] for row in successful) if successful else None,
        )
    else:
        summary["reference"] = "Known synthetic translation; full OrbTracker accepted matches"
        summary["per_frame_mean"] = {}
        for name in QUALITY_METRICS:
            values = [row[name] for row in successful if row[name] is not None]
            summary["per_frame_mean"][name] = {"value": statistics.fmean(values) if values else None,
                                             "evaluated_frames": len(values)}
    return summary


def evaluate(raw, profile, manifest, workload, execution):
    frames = validate_native(raw, profile, manifest, workload, execution)
    rows = scheduled_rows(manifest, workload)
    for row in rows:
        frame = frames.get(row["frame_index"])
        if frame is None:
            continue
        row.update(status=frame["status"], error=frame.get("error", ""))
        if frame["status"] == "success":
            if workload == "temporal":
                row.update(temporal_metrics(frame, profile["tracking"]["feature_capacity"]))
                timing_summary([frame["read_remap_ms"]])
            else:
                row.update(translation_metrics(frame, profile))
            timing_summary([frame["frontend_ms"]])  # Validate warmup samples too.
    return summarize(rows, profile, workload), rows


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def publish_report(directory, report, rows):
    """The final summary is the completion marker; publish it only after the CSV."""
    summary = directory / "summary.json"
    summary.unlink(missing_ok=True)
    write_json(summary, dict(report, status="writing"))
    try:
        temporary = directory / "frames.csv.tmp"
        with temporary.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        temporary.replace(directory / "frames.csv")
        write_json(summary, report)
    except (OSError, ValueError, TypeError) as error:
        report.update(status="failed", error=f"Report publication failed: {error}")
        try:
            write_json(summary, report)
        except OSError:
            pass  # A missing or 'writing' summary is never a completed run.
        raise


def run_identity(executable, snapshot):
    cpu_info = Path("/proc/cpuinfo")
    cpu = platform.processor()
    if cpu_info.is_file():
        cpu = next((line.split(":", 1)[1].strip() for line in cpu_info.read_text().splitlines()
                    if line.startswith("model name")), cpu)
    return {
        "executable_sha256": sha256(executable), "profile_sha256": sha256(snapshot),
        "evaluator_sha256": sha256(Path(__file__).with_name("feature_metrics.py")),
        "runner_sha256": sha256(Path(__file__)), "host": platform.platform(), "cpu": cpu,
        "machine": platform.machine(), "python": platform.python_version(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_DIR, text=True).strip(),
        "git_status": subprocess.check_output(["git", "status", "--short"], cwd=REPO_DIR, text=True),
    }


def execute_workload(executable, dataset, snapshot, directory, workload, execution, profile, manifest):
    rows = scheduled_rows(manifest, workload)
    result = dict(case_counts(rows), status="failed")
    command = [str(executable), str(dataset), str(snapshot), str(directory), workload]
    if execution == "cuda":
        command.append("--cuda")
    print(f"Running {workload}: {profile['frame_count']} frames", flush=True)
    try:
        process = subprocess.run(command, check=False, capture_output=True, text=True, timeout=300)
        result["exit_code"] = process.returncode
        path = directory / f"{workload}.json"
        if path.is_file():
            raw = json.loads(path.read_text())
            metrics, rows = evaluate(raw, profile, manifest, workload, execution)
            result.update(metrics)
            result["native"] = {key: value for key, value in raw.items() if key != "frames"}
        if process.returncode != 0:
            raise RuntimeError(process.stderr.strip() or f"Native exit {process.returncode}")
        if result["successful_cases"] != profile["frame_count"]:
            raise RuntimeError("Incomplete native output")
        result["status"] = "complete"
    except ERRORS as error:
        result["error"] = str(error)
    return result, rows


def print_report(report, directory):
    for workload, result in report.get("results", {}).items():
        print(f"{workload}: {result['status']}; {result['successful_cases']}/{result['scheduled_cases']} successful; "
              f"failed={result['failed_cases']}; missing={len(result['missing_case_ids'])}")
        timing = result.get("frontend", {})
        if timing.get("median_ms") is not None:
            print(f"  Frontend: median={timing['median_ms']:.3f} ms; P95={timing['p95_ms']:.3f} ms; samples={timing['samples']}")
        if workload == "temporal":
            print("  Precision/recall: N/A (no correspondence ground truth)")
        for name, metric in result.get("per_frame_mean", {}).items():
            value = "N/A" if metric["value"] is None else f"{metric['value']:.4f}"
            print(f"  Synthetic {name}: {value} ({metric['evaluated_frames']} frames)")
        if "error" in result:
            print(f"  Error: {result['error']}")
    print(f"Output: {directory}")


def run(args):
    execution = "cuda" if args.cuda else "cpu"
    executable = (args.executable or REPO_DIR / f"build/frontend-{execution}/orb_benchmark").resolve()
    directory = (args.output or REPO_DIR / "output/feature-benchmark" / execution).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    report, rows = {"status": "running", "execution": execution, "results": {}}, []
    # Invalidate completion before cleanup too: a bad old output path can make unlink fail.
    (directory / "summary.json").unlink(missing_ok=True)
    write_json(directory / "summary.json", report)
    try:
        for name in OUTPUT_FILES:
            if name != "summary.json":
                (directory / name).unlink(missing_ok=True)
        profile = load_profile(args.profile)
        report["profile"] = profile
        snapshot = directory / "resolved-profile.json"
        write_json(snapshot, profile)
        dataset = (args.dataset or REPO_DIR / "datasets" / profile["dataset"]).resolve()
        manifest = input_manifest(dataset, profile)
        report.update(input=manifest, identity=run_identity(executable, snapshot))
        for workload in WORKLOADS:
            result, workload_rows = execute_workload(executable, dataset, snapshot, directory, workload,
                                                    execution, profile, manifest)
            report["results"][workload] = result
            rows.extend(workload_rows)
        report["status"] = "complete" if all(result["status"] == "complete" for result in report["results"].values()) else "failed"
    except ERRORS as error:
        report.update(status="failed", error=str(error))
    publish_report(directory, report, rows)
    print_report(report, directory)
    if report["status"] != "complete":
        raise RuntimeError(report.get("error", "Benchmark incomplete; see per-workload results"))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=BENCHMARK_DIR / "profile.json")
    parser.add_argument("--dataset", type=Path, help="Override the pinned sequence's mav0 directory")
    parser.add_argument("--executable", type=Path, help="Path to the compiled C++ orb_benchmark")
    parser.add_argument("--output", type=Path, help="Single-writer output directory; named benchmark outputs are replaced")
    parser.add_argument("--cuda", action="store_true")
    try:
        run(parser.parse_args())
    except ERRORS as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
