"""Offline downloader checks using a local HTTP fixture, not public datasets."""

import contextlib
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "download_datasets.py"
SPEC = importlib.util.spec_from_file_location("download_datasets", SCRIPT)
downloader = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(downloader)
PAYLOAD = b"#ROSBAG V2.0\n" + bytes(range(256)) * 16


class DownloadHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        requested_range = self.headers.get("Range")
        self.server.requests.append((self.path, requested_range))
        if self.path == "/missing":
            self.send_error(404)
            return
        content = b"<!DOCTYPE html><html>quota exceeded</html>" if self.path == "/html" else PAYLOAD
        offset = 0
        if requested_range and self.path != "/no-range":
            offset = int(requested_range.removeprefix("bytes=").removesuffix("-"))
            self.send_response(206)
            self.send_header("Content-Range", f"bytes {offset}-{len(content)-1}/{len(content)}")
        else:
            self.send_response(200)
        self.send_header("Content-Length", str(len(content) - offset))
        self.end_headers()
        self.wfile.write(content[offset:])

    def log_message(self, *_args):
        pass


class DownloadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), DownloadHandler)
        cls.server.requests = []
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.thread.join()
        cls.server.server_close()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.item = {
            "path": "sample/flight.bag",
            "url": f"http://127.0.0.1:{self.server.server_port}/bag",
            "format": "rosbag",
            "bytes": len(PAYLOAD),
            "sha256": hashlib.sha256(PAYLOAD).hexdigest(),
        }
        self.path = self.directory / self.item["path"]
        self.receipt = self.path.with_name(self.path.name + ".download.json")
        self.partial = self.path.with_name(self.path.name + ".part")
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def test_download_repeat_and_offline_verification(self):
        downloader.download_file(self.item, self.directory)
        self.assertEqual(self.path.read_bytes(), PAYLOAD)
        self.assertEqual(json.loads(self.receipt.read_text())["sha256"], self.item["sha256"])
        count = len(self.server.requests)
        downloader.download_file(self.item, self.directory)
        downloader.download_file(self.item, self.directory, verify_only=True)
        self.assertEqual(len(self.server.requests), count)
        self.assertFalse(self.partial.exists())

    def test_resume(self):
        self.path.parent.mkdir()
        self.partial.write_bytes(PAYLOAD[:1024])
        downloader.download_file(self.item, self.directory)
        self.assertEqual(self.server.requests[-1], ("/bag", "bytes=1024-"))
        self.assertEqual(self.path.read_bytes(), PAYLOAD)

    def test_server_without_resume_restarts_partial(self):
        self.item["url"] = self.item["url"].replace("/bag", "/no-range")
        self.path.parent.mkdir()
        self.partial.write_bytes(PAYLOAD[:1024])
        downloader.download_file(self.item, self.directory)
        self.assertEqual(self.server.requests[-2:], [("/no-range", "bytes=1024-"), ("/no-range", None)])
        self.assertEqual(self.path.read_bytes(), PAYLOAD)

    def test_bad_checksum_never_publishes_file(self):
        self.item["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            downloader.download_file(self.item, self.directory)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.receipt.exists())

    def test_incomplete_file_never_published(self):
        self.item["bytes"] += 1
        with self.assertRaisesRegex(ValueError, "unexpected size"):
            downloader.download_file(self.item, self.directory)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.receipt.exists())

    def test_html_and_http_failure_never_publish_files(self):
        del self.item["bytes"]
        for endpoint, error in [("html", ValueError), ("missing", RuntimeError)]:
            with self.subTest(endpoint=endpoint):
                self.partial.unlink(missing_ok=True)
                self.item["url"] = f"http://127.0.0.1:{self.server.server_port}/{endpoint}"
                with self.assertRaises(error):
                    downloader.download_file(self.item, self.directory)
                self.assertFalse(self.path.exists())
                self.assertFalse(self.receipt.exists())

    def test_local_receipt_detects_corruption_without_publisher_hash(self):
        del self.item["sha256"]
        downloader.download_file(self.item, self.directory)
        damaged = PAYLOAD[:-1] + b"X"
        self.path.write_bytes(damaged)
        with self.assertRaisesRegex(ValueError, "differs from its download receipt"):
            downloader.download_file(self.item, self.directory)
        self.assertEqual(self.path.read_bytes(), damaged)

    def test_verify_missing_file_does_not_download(self):
        count = len(self.server.requests)
        with self.assertRaisesRegex(ValueError, "missing dataset file"):
            downloader.download_file(self.item, self.directory, verify_only=True)
        self.assertEqual(len(self.server.requests), count)

    def test_completed_partial_is_verified_before_promotion(self):
        self.path.parent.mkdir()
        self.partial.write_bytes(PAYLOAD)
        count = len(self.server.requests)
        downloader.download_file(self.item, self.directory)
        self.assertEqual(len(self.server.requests), count)
        self.assertEqual(self.path.read_bytes(), PAYLOAD)

    def test_manual_file_requires_checksum_baseline(self):
        self.path.parent.mkdir()
        self.path.write_bytes(PAYLOAD)
        del self.item["sha256"]
        with self.assertRaisesRegex(ValueError, "missing checksum baseline"):
            downloader.download_file(self.item, self.directory)

    def test_md5_and_zip_validation(self):
        import zipfile

        path = self.directory / "fixture.zip"
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("imu.csv", "time,x,y,z\n0,1,2,3\n")
        item = {"url": "https://example.invalid/data", "format": "zip",
                "md5": hashlib.md5(path.read_bytes(), usedforsecurity=False).hexdigest()}
        downloader.check_file(path, item)
        item["md5"] = "0" * 32
        with self.assertRaisesRegex(ValueError, "publisher MD5 mismatch"):
            downloader.check_file(path, item)
        path.write_bytes(path.read_bytes()[:32])
        with self.assertRaisesRegex(ValueError, "not a complete ZIP"):
            downloader.check_file(path, item)

    def test_catalogue_selections_include_support_once(self):
        catalog = json.loads(downloader.CATALOG.read_text())
        files = downloader.select_files(catalog, ["gvins/sports_field", "gvins/complex_environment"])
        paths = [item["path"] for item in files]
        self.assertEqual(len(paths), len(set(paths)))
        self.assertIn("gvins/visensor_parameters/left_camera.yaml", paths)
        self.assertIn("gvins/sports_field.bag", paths)
        self.assertIn("gvins/complex_environment.bag", paths)
        with self.assertRaisesRegex(ValueError, "unknown selection"):
            downloader.select_files(catalog, ["gvins/not-a-sequence"])


if __name__ == "__main__":
    unittest.main()
