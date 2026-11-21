"""Check suite orchestration and evidence publication independently of GPU availability."""

import argparse
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
import run_benchmark as suite


def completed_backend(execution):
    identity = {name: "fixture" for name in (
        "executable_sha256", "profile_sha256", "evaluator_sha256", "runner_sha256",
        "git_head", "git_status", "host", "cpu", "machine", "python")}
    native = {name: "fixture" for name in (
        "opencv", "compiler", "build_type", "width", "height", "options", "adapter")}
    result = {"status": "complete", "scheduled_cases": 3, "successful_cases": 3,
              "failed_cases": 0, "missing_case_ids": [], "native": native,
              "frontend": {"samples": 2, "median_ms": 2.5, "p95_ms": 3.0, "p99_ms": 3.0, "max_ms": 3.0}}
    return {"status": "complete", "execution": execution, "profile": {}, "input": {}, "identity": identity,
            "results": {name: copy.deepcopy(result) for name in suite.features.WORKLOADS}}


class SuiteTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.profile = suite.features.load_profile(suite.features.BENCHMARK_DIR / "profile.json")
        path = self.root / "profile.json"
        path.write_text(json.dumps(self.profile))
        self.args = argparse.Namespace(category="features", backend="both", profile=path,
                                       dataset=None, executable=None, output=self.root / "run")

    def run_suite(self):
        with contextlib.redirect_stdout(io.StringIO()):
            return suite.run(self.args)

    def test_sequential_backends_share_frozen_profile_and_executable(self):
        calls = []

        def execute(request):
            backend = "cuda" if request.cuda else "cpu"
            calls.append((backend, request.executable))
            self.assertEqual(json.loads(request.profile.read_text()), self.profile)
            self.args.profile.write_text("{}")  # An edit cannot change CUDA's workload.
            return completed_backend(backend)

        with mock.patch.object(suite.features, "run", side_effect=execute):
            result = self.run_suite()
        self.assertEqual([name for name, _ in calls], ["cpu", "cuda"])
        self.assertEqual(calls[0][1], calls[1][1])
        self.assertEqual(calls[0][1], suite.REPO_DIR / "build/frontend-cuda/orb_benchmark")
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["comparison"]["status"], "matched")
        self.assertEqual(result["baseline_review"], "unreviewed")
        self.assertEqual(json.loads((self.args.output / "summary.json").read_text()), result)
        self.assertIn("| Metric | CPU | CUDA |", (self.args.output / "report.md").read_text())

    def test_baseline_allows_code_changes_but_requires_identical_experiment(self):
        old = {"schema": "feature-benchmark-suite-v2", "status": "complete",
               "results": {"cpu": completed_backend("cpu")}}
        current = copy.deepcopy(old)
        current["results"]["cpu"]["identity"].update(git_head="new", executable_sha256="new")
        current["results"]["cpu"]["results"]["temporal"]["frontend"]["median_ms"] = 5
        result = suite.compare_baseline(current, old)
        self.assertEqual(result["status"], "compared")
        metric = next(row for row in result["deltas"] if row["metric"] == "cpu.temporal.frontend.median_ms")
        self.assertEqual(metric["ratio"], 2)
        self.assertEqual(metric["delta"], 2.5)
        current["results"]["cpu"]["identity"]["cpu"] = "another machine"
        self.assertEqual(suite.compare_baseline(current, old)["status"], "incompatible")
        current = copy.deepcopy(old)
        current["results"]["cpu"]["profile"]["frame_count"] = 99
        self.assertEqual(suite.compare_baseline(current, old)["status"], "incompatible")
        old["status"] = "failed"
        self.assertEqual(suite.compare_baseline(current, old)["status"], "incompatible")

    def test_cpu_only_uses_cpu_build(self):
        self.args.backend = "cpu"
        with mock.patch.object(suite.features, "run", return_value=completed_backend("cpu")) as execute:
            result = self.run_suite()
        self.assertEqual(execute.call_args.args[0].executable, suite.REPO_DIR / "build/frontend-cpu/orb_benchmark")
        self.assertEqual(result["requested_backends"], ["cpu"])
        self.assertEqual(result["comparison"]["status"], "not_requested")

    def test_backend_failure_preserves_partial_result_and_still_runs_next(self):
        calls = []

        def execute(request):
            calls.append(request.cuda)
            if request.cuda:
                return completed_backend("cuda")
            partial = completed_backend("cpu")
            partial.update(status="failed", error="native failure")
            request.output.mkdir()
            suite.features.write_json(request.output / "summary.json", partial)
            raise RuntimeError("native failure")

        with mock.patch.object(suite.features, "run", side_effect=execute):
            result = self.run_suite()
        self.assertEqual(calls, [False, True])
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["results"]["cpu"]["results"]["temporal"]["successful_cases"], 3)
        self.assertEqual(result["results"]["cuda"]["status"], "complete")
        self.assertEqual(result["comparison"]["status"], "unavailable")

    def test_changed_bindings_prevent_a_completed_comparison(self):
        for name in ("input", "profile", "executable_sha256", "adapter"):
            cpu, cuda = completed_backend("cpu"), completed_backend("cuda")
            if name in ("input", "profile"):
                cuda[name] = {"different": True}
            elif name == "executable_sha256":
                cuda["identity"][name] = "different"
            else:
                cuda["results"]["temporal"]["native"][name] = "different"
            with self.subTest(name=name):
                self.assertEqual(suite.compare_bindings({"cpu": cpu, "cuda": cuda})["status"], "mismatch")
        with mock.patch.object(suite.features, "run", side_effect=[cpu, cuda]):
            result = self.run_suite()
        self.assertEqual(result["status"], "failed")

    def test_existing_directory_is_not_overwritten(self):
        self.args.output.mkdir()
        report = self.args.output / "summary.json"
        report.write_text("review candidate")
        with mock.patch.object(suite.features, "run") as execute, self.assertRaises(FileExistsError):
            self.run_suite()
        execute.assert_not_called()
        self.assertEqual(report.read_text(), "review candidate")

    def test_report_failure_cannot_publish_complete_json(self):
        write_text = Path.write_text

        def fail_report(path, text, *args, **kwargs):
            if path.name == "report.md":
                raise OSError("disk full")
            return write_text(path, text, *args, **kwargs)

        with mock.patch.object(suite.features, "run", side_effect=[completed_backend("cpu"), completed_backend("cuda")]), \
                mock.patch.object(Path, "write_text", fail_report), self.assertRaisesRegex(OSError, "disk full"):
            self.run_suite()
        self.assertEqual(json.loads((self.args.output / "summary.json").read_text())["status"], "running")

    def test_interrupt_is_not_completion(self):
        with mock.patch.object(suite.features, "run", side_effect=KeyboardInterrupt):
            result = self.run_suite()
        self.assertEqual(result["status"], "interrupted")
        self.assertEqual(result["results"], {})

    def test_missing_quality_is_na_not_zero(self):
        rows = dict(suite.metric_rows({}, "translation"))
        self.assertEqual(rows["Precision"], "N/A")
        self.assertEqual(rows["All eligible matches RMSE (px)"], "N/A")
        self.assertEqual(rows["Missing cases"], "N/A")


if __name__ == "__main__":
    unittest.main()
