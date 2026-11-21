#!/usr/bin/env python3
"""Run a benchmark category sequentially and preserve a reviewable result bundle."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "benchmark/features"))
import run_feature_benchmark as features


def run_backend(args, backend, profile, directory, executable):
    request = argparse.Namespace(profile=profile, dataset=args.dataset, executable=executable,
                                 output=directory / backend, cuda=backend == "cuda")
    try:
        return features.run(request)
    except features.ERRORS as error:
        # Keep partial measurements, but a failed invocation can never be complete.
        try:
            report = json.loads((request.output / "summary.json").read_text())
        except (OSError, ValueError):
            report = {"execution": backend, "results": {}}
        report.update(status="failed", error=str(error))
        return report


def compare_bindings(results):
    if set(results) != {"cpu", "cuda"}:
        return {"status": "not_requested", "differences": []}
    if any(result["status"] != "complete" for result in results.values()):
        return {"status": "unavailable", "differences": []}
    cpu, cuda = results["cpu"], results["cuda"]
    differences = [key for key in ("profile", "input") if cpu[key] != cuda[key]]
    for key in ("executable_sha256", "profile_sha256", "evaluator_sha256", "runner_sha256",
                "git_head", "git_status", "host", "cpu", "machine", "python"):
        if cpu["identity"][key] != cuda["identity"][key]:
            differences.append(f"identity.{key}")
    for workload in features.WORKLOADS:
        a, b = cpu["results"][workload]["native"], cuda["results"][workload]["native"]
        for key in ("opencv", "compiler", "build_type", "width", "height", "options", "adapter"):
            if a[key] != b[key]:
                differences.append(f"{workload}.{key}")
    return {"status": "mismatch" if differences else "matched", "differences": differences}


def display(value):
    if value is None:
        return "N/A"
    return f"{value:.4f}" if isinstance(value, float) else str(value)


def metric_rows(result, workload):
    cases = "N/A"
    if "scheduled_cases" in result:
        cases = f"{result['successful_cases']}/{result['scheduled_cases']}"
    timing = result.get("frontend", {})
    rows = [
        ("Status", result.get("status", "missing")),
        ("Successful / scheduled cases", cases),
        ("Failed cases", result.get("failed_cases")),
        ("Missing cases", len(result["missing_case_ids"]) if "missing_case_ids" in result else None),
        ("Timing samples (after warmup)", timing.get("samples")),
        ("Median (ms)", timing.get("median_ms")),
        ("P95 (ms)", timing.get("p95_ms")),
        ("P99 (ms)", timing.get("p99_ms")),
        ("Maximum observed (ms)", timing.get("max_ms")),
    ]
    if workload == "temporal":
        rows.extend([("Mean retained features", result.get("mean_features")),
                     ("Mean occupied grid fraction", result.get("mean_coverage")),
                     ("Mean observed track age", result.get("mean_observed_track_age")),
                     ("Completed-track median observations", result.get("track_lengths", {}).get("completed", {}).get("median")),
                     ("Completed-track P95 observations", result.get("track_lengths", {}).get("completed", {}).get("p95")),
                     ("Active tracks at window end (censored)", result.get("track_lengths", {}).get("active_right_censored", {}).get("samples")),
                     ("Featureless frames", result.get("empty_frames")),
                     ("Frames without valid geometry", result.get("geometry_invalid_frames"))])
    else:
        for key, label in (("repeatability", "Repeatability"), ("precision", "Precision"),
                           ("recall", "Detection-conditioned recall"),
                           ("rmse_all_px", "All eligible matches RMSE (px)"),
                           ("rmse_correct_px", "Correct matches RMSE (px)")):
            metric = result.get("per_frame_mean", {}).get(key, {})
            rows.append((label, metric.get("value")))
            rows.append((f"{label}: evaluated frames", metric.get("evaluated_frames")))
    for stage in features.STAGES:
        metrics = result.get("stages", {}).get(stage, {})
        rows.append((f"{stage}: median / P95 (ms)", f"{display(metrics.get('median_ms'))} / {display(metrics.get('p95_ms'))}"))
    peaks = [item["process_peak_kib"] for item in result.get("stages", {}).values() if item.get("process_peak_kib") is not None]
    rows.append(("Process high-water RSS (KiB; includes warmup)", max(peaks) if peaks else None))
    for name in ("support_rejected", "displacement_rejected", "quality_rejected", "inverse_rejected", "quota_rejected", "spacing_rejected"):
        rows.append((name, result.get("diagnostics", {}).get(name)))
    return [(label, display(value)) for label, value in rows]


def compare_baseline(report, baseline):
    """Numerical deltas with strict workload/tool bindings; no invented acceptance limits."""
    if (baseline.get("schema") != report["schema"] or baseline.get("suite_sha256") != report.get("suite_sha256")
            or baseline.get("status") != "complete"
            or report["status"] != "complete" or set(baseline["results"]) != set(report["results"])):
        return {"status": "incompatible", "reason": "Incomplete/schema/backend mismatch", "deltas": []}
    deltas = []
    for backend, current in report["results"].items():
        previous = baseline["results"][backend]
        for key in ("profile", "input"):
            if current[key] != previous[key]:
                return {"status": "incompatible", "reason": f"{backend}.{key} differs", "deltas": []}
        for key in ("host", "cpu", "machine", "python", "evaluator_sha256", "runner_sha256"):
            if current["identity"][key] != previous["identity"][key]:
                return {"status": "incompatible", "reason": f"{backend}.identity.{key} differs", "deltas": []}
        for workload in features.WORKLOADS:
            a, b = current["results"][workload], previous["results"][workload]
            for key in ("opencv", "compiler", "build_type", "width", "height", "options", "adapter", "cuda_device"):
                if a["native"].get(key) != b["native"].get(key):
                    return {"status": "incompatible", "reason": f"{backend}.{workload}.{key} differs", "deltas": []}
            paths = [("frontend", "median_ms"), ("frontend", "p95_ms")]
            paths += [("stages", stage, "median_ms") for stage in features.STAGES]
            if workload == "temporal":
                paths += [("mean_features",), ("mean_coverage",), ("mean_observed_track_age",)]
            else:
                paths += [("per_frame_mean", name, "value") for name in features.QUALITY_METRICS]
            for path in paths:
                x, y = a, b
                for key in path:
                    x = x.get(key) if isinstance(x, dict) else None
                    y = y.get(key) if isinstance(y, dict) else None
                if x is not None and y is not None:
                    deltas.append({"metric": ".".join((backend, workload, *path)), "baseline": y, "current": x,
                                   "delta": x-y, "ratio": x/y if y else None})
    return {"status": "compared", "acceptance": "unreviewed; no numerical limits selected", "deltas": deltas}


def render_report(report):
    results = report["results"]
    lines = ["# Feature benchmark", "", f"Started (UTC): {report['started_utc']}",
             f"Status: {report['status']}; baseline review: unreviewed",
             f"Total elapsed: {report['elapsed_seconds']:.2f} s", "",
             f"CPU/CUDA input and build bindings: {report['comparison']['status']}"]
    for difference in report["comparison"]["differences"]:
        lines.append(f"- Different: {difference}")
    if "error" in report:
        lines.extend(["", f"Error: {report['error']}"])
    for workload, title in (("temporal", "Temporal tracking (one frame)"),
                            ("translation", "Synthetic translation (two tracker calls)")):
        lines.extend(["", f"## {title}", ""])
        if not results:
            lines.append("No results.")
            continue
        columns = [metric_rows(result.get("results", {}).get(workload, {}), workload)
                   for result in results.values()]
        lines.append("| Metric | " + " | ".join(name.upper() for name in results) + " |")
        lines.append("|---|" + "---:|" * len(results))
        for row in zip(*columns):
            lines.append("| " + row[0][0] + " | " + " | ".join(value for _, value in row) + " |")
    for backend, result in results.items():
        for scope, item in [(backend, result)] + list(result.get("results", {}).items()):
            if "error" in item:
                lines.extend(["", f"{backend}/{scope}: {item['error']}"])
    if "baseline_comparison" in report:
        baseline = report["baseline_comparison"]
        lines.extend(["", f"Baseline comparison: {baseline['status']}"])
        if "reason" in baseline:
            lines.append(baseline["reason"])
        if baseline["deltas"]:
            lines.extend(["", "| Metric | Baseline | Current | Delta |", "|---|---:|---:|---:|"])
            for value in baseline["deltas"]:
                lines.append(f"| {value['metric']} | {display(value['baseline'])} | {display(value['current'])} | {display(value['delta'])} |")
    lines.extend(["", "Times include blocking frontend transfers/synchronization; image read is excluded.",
                  "Temporal precision/recall/RMSE: N/A (no correspondence ground truth).",
                  "Synthetic quality values are per-frame means; rates are fractions in [0, 1].",
                  "Stage RSS is cumulative process high-water, not stage-owned memory; GPU VRAM is not measured.",
                  "Stage timing is instrumented; uninstrumented production latency may differ.",
                  "Completed track lengths exclude still-active (right-censored) tracks.",
                  "Partial results are not a baseline. Completion is not a quality/performance acceptance verdict.",
                  "Matched bindings do not establish identical algorithms, stable GPU outputs or controlled clocks/load.", ""])
    return "\n".join(lines)


def run(args):
    profile = features.load_profile(args.profile)
    baseline = json.loads(args.baseline.read_text()) if getattr(args, "baseline", None) else None
    started = datetime.now(timezone.utc)
    directory = (args.output or REPO_DIR / "output/benchmark/features" /
                 started.strftime("%Y%m%dT%H%M%S.%fZ")).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    backends = ("cpu", "cuda") if args.backend == "both" else (args.backend,)
    build = "frontend-cpu" if args.backend == "cpu" else "frontend-cuda"
    executable = (args.executable or REPO_DIR / "build" / build / "orb_benchmark").resolve()
    report = {"schema": "feature-benchmark-suite-v2", "status": "running", "baseline_review": "unreviewed",
              "started_utc": started.isoformat(), "requested_backends": list(backends),
              "suite_sha256": features.sha256(Path(__file__)), "results": {},
              "comparison": {"status": "unavailable", "differences": []}}
    summary = directory / "summary.json"
    features.write_json(summary, report)
    start = time.monotonic()
    try:
        snapshot = directory / "profile.json"
        features.write_json(snapshot, profile)
        for index, backend in enumerate(backends, 1):
            print(f"\n[{index}/{len(backends)}] ORB {backend.upper()}", flush=True)
            report["results"][backend] = run_backend(args, backend, snapshot, directory, executable)
            features.write_json(summary, report)
        report["comparison"] = compare_bindings(report["results"])
        complete = all(result["status"] == "complete" for result in report["results"].values())
        report["status"] = "complete" if complete and report["comparison"]["status"] != "mismatch" else "failed"
    except features.ERRORS as error:
        report.update(status="failed", error=str(error))
    except KeyboardInterrupt:
        report.update(status="interrupted", error="Interrupted by caller")
    if baseline is not None:
        report["baseline_comparison"] = compare_baseline(report, baseline)
        if report["baseline_comparison"]["status"] != "compared":
            report["status"] = "failed"
    report["elapsed_seconds"] = time.monotonic() - start
    text = render_report(report)
    # The final JSON is the completion marker, published after the readable report.
    (directory / "report.md").write_text(text)
    features.write_json(summary, report)
    print("\n" + text)
    print(f"Results: {directory}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("category", choices=("features",))
    parser.add_argument("--backend", choices=("both", "cpu", "cuda"), default="both")
    parser.add_argument("--profile", type=Path, default=features.BENCHMARK_DIR / "profile.json")
    parser.add_argument("--dataset", type=Path, help="EuRoC mav0 directory override")
    parser.add_argument("--executable", type=Path, help="Use the same executable for every selected backend")
    parser.add_argument("--baseline", type=Path, help="Earlier complete suite summary.json; report deltas without assuming acceptance limits")
    parser.add_argument("--output", type=Path, help="New result directory; existing directories are never overwritten")
    try:
        report = run(parser.parse_args())
        if report["status"] == "interrupted":
            return 130
        return 0 if report["status"] == "complete" else 1
    except features.ERRORS as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
