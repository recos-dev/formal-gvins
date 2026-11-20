"""Check native-result ingestion, independent workloads and report publication."""

import argparse
import contextlib
import copy
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_feature_benchmark as benchmark


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.profile = benchmark.load_profile(benchmark.BENCHMARK_DIR / "profile.json")
        self.profile.update(frame_count=3, warmup_frames=1)
        self.manifest = {"frames": [{"frame_index": i, "time_ns": str(1000000000000000000 + i)}
                                    for i in range(100, 103)]}

    def native(self, workload):
        raw = {"protocol": benchmark.PROTOCOL, "workload": workload, "execution": "cpu",
               "options": dict(self.profile["tracking"]), "frames": []}
        for frame in self.manifest["frames"]:
            result = dict(frame, status="success", frontend_ms=10, geometry_valid=0)
            if workload == "temporal":
                result.update(features=1, ratio_passed=0, inliers=0, read_remap_ms=2, observations=[[20, 20, "1"]])
            else:
                result.update(source=[[20, 20, 1]], target=[[32, 12, 1]], matches=[[0, 0]])
            raw["frames"].append(result)
        return raw

    def evaluate(self, raw):
        return benchmark.evaluate(raw, self.profile, self.manifest, raw["workload"], "cpu")

    def test_native_coordinates_reach_shared_scorer(self):
        temporal, _ = self.evaluate(self.native("temporal"))
        translated, rows = self.evaluate(self.native("translation"))
        self.assertEqual(temporal["successful_cases"], 3)
        self.assertEqual(temporal["frontend"]["samples"], 2)
        self.assertEqual(temporal["frontend"]["median_ms"], 10)
        self.assertIsNone(temporal["precision"])
        self.assertIsNone(temporal["rmse_px"])
        self.assertEqual(translated["per_frame_mean"]["precision"]["value"], 1)
        self.assertTrue(all(row["rmse_all_px"] == 0 for row in rows))

    def test_wrong_reference_transform_changes_scores(self):
        self.profile["translation_x_px"] = 30
        result, _ = self.evaluate(self.native("translation"))
        self.assertEqual(result["per_frame_mean"]["precision"]["value"], 0)
        self.assertEqual(result["per_frame_mean"]["rmse_all_px"]["value"], 18)

    def test_failure_and_missing_frame_are_not_dropped(self):
        for workload in benchmark.WORKLOADS:
            raw = self.native(workload)
            raw["frames"][1] = dict(self.manifest["frames"][1], status="failed", error="read failed")
            raw["frames"].pop()
            with self.subTest(workload=workload):
                result, rows = self.evaluate(raw)
                self.assertEqual(result["successful_cases"], 1)
                self.assertEqual(result["failed_cases"], 1)
                self.assertEqual(result["missing_case_ids"], ["102"])
                self.assertEqual(result["success_rate"], 1 / 3)
                self.assertEqual(result["frontend"]["samples"], 0)
                self.assertEqual(rows[1]["error"], "read failed")
                self.assertEqual(rows[2]["status"], "missing")

    def test_all_missing_and_all_failed_schedules(self):
        for workload in benchmark.WORKLOADS:
            for status in ("missing", "failed"):
                raw = self.native(workload)
                raw["frames"] = [] if status == "missing" else [
                    dict(frame, status="failed", error="failed") for frame in self.manifest["frames"]
                ]
                with self.subTest(workload=workload, status=status):
                    result, rows = self.evaluate(raw)
                    self.assertEqual(result["success_rate"], 0)
                    self.assertEqual(result["failed_cases"], 3 if status == "failed" else 0)
                    self.assertEqual(len(result["missing_case_ids"]), 3 if status == "missing" else 0)
                    self.assertEqual(len(rows), 3)
                    self.assertIsNone(result["frontend"]["median_ms"])

    def test_empty_detections_do_not_fabricate_quality(self):
        raw = self.native("translation")
        for frame in raw["frames"]:
            frame.update(source=[], target=[], matches=[])
        result, _ = self.evaluate(raw)
        self.assertEqual(result["success_rate"], 1)
        self.assertIsNone(result["per_frame_mean"]["precision"]["value"])
        self.assertEqual(result["per_frame_mean"]["precision"]["evaluated_frames"], 0)

    def test_duplicate_reordered_or_mistimed_results_fail(self):
        for mode in ("duplicate", "reordered", "timestamp"):
            raw = self.native("temporal")
            if mode == "duplicate":
                raw["frames"][1] = raw["frames"][0]
            elif mode == "reordered":
                raw["frames"].reverse()
            else:
                raw["frames"][0]["time_ns"] = "0"
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                self.evaluate(raw)

    def test_mismatched_profile_or_backend_is_rejected(self):
        for mode in ("options", "execution", "protocol"):
            raw = self.native("temporal")
            if mode == "options":
                raw["options"]["feature_count"] += 1
            else:
                raw[mode] = "wrong"
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                self.evaluate(raw)

    def test_malformed_native_observations_fail(self):
        raw = self.native("temporal")
        raw["frames"][0]["observations"].append([20, 20, "1"])
        with self.assertRaisesRegex(ValueError, "observations"):
            self.evaluate(raw)

    def test_invalid_warmup_timing_is_not_ignored(self):
        raw = self.native("temporal")
        raw["frames"][0]["frontend_ms"] = -1
        with self.assertRaisesRegex(ValueError, "latency"):
            self.evaluate(raw)

    def test_nearest_rank_percentiles_and_empty_timing(self):
        self.assertEqual(benchmark.timing_summary(list(range(1, 101))),
                         {"samples": 100, "median_ms": 50.5, "p95_ms": 95, "p99_ms": 99, "max_ms": 100})
        self.assertIsNone(benchmark.timing_summary([])["median_ms"])

    def test_profile_requires_every_tracking_option(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "profile.json"
            for name in self.profile["tracking"]:
                profile = copy.deepcopy(self.profile)
                del profile["tracking"][name]
                path.write_text(json.dumps(profile))
                with self.subTest(option=name), self.assertRaises(KeyError):
                    benchmark.load_profile(path)
            profile = copy.deepcopy(self.profile)
            profile["tracking"]["ratio"] = 1
            path.write_text(json.dumps(profile))
            with self.assertRaises(ValueError):
                benchmark.load_profile(path)

    def test_csv_columns_are_fixed_for_failed_or_missing_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for workload in benchmark.WORKLOADS:
                rows = benchmark.scheduled_rows(self.manifest, workload)
                rows[0].update(status="failed", error="read failed")
                benchmark.publish_report(directory, {"status": "failed"}, rows)
                with (directory / "frames.csv").open() as stream:
                    reader = csv.DictReader(stream)
                    self.assertEqual(tuple(reader.fieldnames), benchmark.CSV_FIELDS)
                    self.assertEqual(len(list(reader)), 3)

    def test_csv_write_failure_cannot_leave_success_summary(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "summary.json").write_text('{"status": "complete"}')
            (directory / "frames.csv.tmp").mkdir()  # Block CSV creation using the real filesystem.
            with self.assertRaises(OSError):
                benchmark.publish_report(directory, {"status": "complete"}, [])
            self.assertEqual(json.loads((directory / "summary.json").read_text())["status"], "failed")

    def test_csv_replace_failure_cannot_publish_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "frames.csv").mkdir()
            with self.assertRaises(OSError):
                benchmark.publish_report(directory, {"status": "complete"}, [])
            self.assertEqual(json.loads((directory / "summary.json").read_text())["status"], "failed")

    def test_success_is_published_after_complete_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            rows = benchmark.scheduled_rows(self.manifest, "temporal")
            write_json = benchmark.write_json

            def observe_publication(path, value):
                if value["status"] == "complete":
                    with (directory / "frames.csv").open() as stream:
                        self.assertEqual(len(list(csv.DictReader(stream))), 3)
                write_json(path, value)

            with mock.patch.object(benchmark, "write_json", side_effect=observe_publication):
                benchmark.publish_report(directory, {"status": "complete"}, rows)

    def test_invalid_profile_cannot_leave_previous_success_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            profile = root / "profile.json"
            profile.write_text("{}")
            output = root / "output"
            output.mkdir()
            (output / "summary.json").write_text('{"status": "complete"}')
            (output / "temporal.png").write_bytes(b"old plot")
            args = argparse.Namespace(profile=profile, output=output, dataset=None, executable=None, cuda=False)
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                benchmark.run(args)
            self.assertEqual(json.loads((output / "summary.json").read_text())["status"], "failed")
            self.assertFalse((output / "temporal.png").exists())

    def test_cleanup_failure_invalidates_previous_completion(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "temporal.json").mkdir()
            (directory / "summary.json").write_text('{"status": "complete"}')
            args = argparse.Namespace(profile=directory / "unused.json", output=directory,
                                      dataset=None, executable=None, cuda=False)
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                benchmark.run(args)
            self.assertEqual(json.loads((directory / "summary.json").read_text())["status"], "failed")

    def test_both_jobs_run_from_snapshot_even_when_first_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "profile.json"
            path.write_text(json.dumps(self.profile))
            args = argparse.Namespace(profile=path, output=root / "output", dataset=root, executable=root / "binary", cuda=False)
            visited = []

            def execute(executable, dataset, snapshot, directory, workload, execution, profile, manifest):
                visited.append(workload)
                self.assertEqual(json.loads(snapshot.read_text()), self.profile)
                path.write_text("{}")  # Editing the source profile cannot change the next process's input.
                rows = benchmark.scheduled_rows(manifest, workload)
                for row in rows:
                    row["status"] = "failed" if workload == "temporal" else "success"
                result = dict(benchmark.case_counts(rows), status="failed" if workload == "temporal" else "complete")
                return result, rows

            with mock.patch.object(benchmark, "input_manifest", return_value=self.manifest), \
                    mock.patch.object(benchmark, "run_identity", return_value={}), \
                    mock.patch.object(benchmark, "execute_workload", side_effect=execute):
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                    benchmark.run(args)
            self.assertEqual(visited, list(benchmark.WORKLOADS))
            result = json.loads((args.output / "summary.json").read_text())
            self.assertEqual(result["results"]["translation"]["successful_cases"], 3)
            self.assertEqual(result["status"], "failed")


if __name__ == "__main__":
    unittest.main()
