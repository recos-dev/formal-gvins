"""ROS-free bag conversion checks with embedded native message definitions."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

PIPELINE = None
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('convert_hkust', ROOT / 'scripts/convert_hkust.py')
CONVERT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONVERT)
PVT = '''uint32 week
float64 tow
'''
PVT_FIELDS = '''gnss_comm/GnssTimeMsg time
uint8 fix_type
bool valid_fix
bool diff_soln
uint8 carr_soln
uint8 num_sv
float64 latitude
float64 longitude
float64 altitude
float64 height_msl
float64 h_acc
float64 v_acc
float64 p_dop
float64 vel_n
float64 vel_e
float64 vel_d
float64 vel_acc
'''


def write_fixture(path, invalid_image=False):
    store = get_typestore(Stores.ROS1_NOETIC)
    store.register(get_types_from_msg(PVT, 'gnss_comm/msg/GnssTimeMsg'))
    store.register(get_types_from_msg(PVT_FIELDS, 'gnss_comm/msg/GnssPVTSolnMsg'))
    store.register(get_types_from_msg('uint64 id\nfloat64[] values\n', 'custom/msg/Extra'))
    types = store.types
    header = types['std_msgs/msg/Header'](7, types['builtin_interfaces/msg/Time'](12, 345), 'imu')
    vector = types['geometry_msgs/msg/Vector3']
    imu = types['sensor_msgs/msg/Imu'](header, types['geometry_msgs/msg/Quaternion'](0., 0., 0., 1.),
        np.zeros(9), vector(1., 2., 3.), np.zeros(9), vector(4., 5., float('nan')), np.zeros(9))
    image = types['sensor_msgs/msg/Image'](header, 2, 2, 'mono8', 0, 3,
        np.array([1, 2, 99, 3, 4] if invalid_image else [1, 2, 99, 3, 4, 88], dtype=np.uint8))
    pvt = types['gnss_comm/msg/GnssPVTSolnMsg'](types['gnss_comm/msg/GnssTimeMsg'](2134, 83911.4),
        3, False, True, 2, 28, 22.3, 114.2, 11., 12., .1, .2, 1.2, 1., 2., 3., .05)
    extra = types['custom/msg/Extra'](2**64 - 1, np.array([float('inf'), float('-inf'), 4.]))
    messages = [('/imu', imu), ('/cam', image), ('/pvt', pvt), ('/extra', extra)]
    epoch = 1_606_691_893_000_000_001
    with Writer(path) as writer:
        for index, (topic, message) in enumerate(messages):
            connection = writer.add_connection(topic, message.__msgtype__, typestore=store)
            writer.write(connection, epoch + (index // 2) * 1_000_000,
                         store.serialize_ros1(message, message.__msgtype__))
    return epoch


class ConversionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'fixture.bag'
        self.output = self.root / 'recording'

    def test_native_fields_times_and_images(self):
        epoch = write_fixture(self.bag)
        manifest = CONVERT.convert(self.bag, self.output)
        records = [json.loads(line) for line in (self.output / 'events.jsonl').read_text().splitlines()]
        self.assertEqual([r['sequence'] for r in records], [0, 1, 2, 3])
        self.assertEqual([r['record_time_ns'] for r in records], [epoch, epoch, epoch+1000000, epoch+1000000])
        imu, image, pvt, extra = [r['payload'] for r in records]
        self.assertEqual(imu['header']['stamp'], {'sec': 12, 'nanosec': 345})
        self.assertEqual(imu['angular_velocity'], {'x': 1., 'y': 2., 'z': 3.})
        self.assertEqual(imu['linear_acceleration']['z'], {'$float': 'nan'})
        self.assertEqual((self.output / 'images.bin').read_bytes(), bytes([1, 2, 99, 3, 4, 88]))
        self.assertEqual(image['step'], 3)
        self.assertEqual(pvt['time'], {'week': 2134, 'tow': 83911.4})
        self.assertFalse(pvt['valid_fix'])
        self.assertEqual([pvt[f'vel_{axis}'] for axis in 'ned'], [1., 2., 3.])
        self.assertEqual(extra['id'], 2**64-1)
        self.assertEqual(extra['values'], [{'$float': '+inf'}, {'$float': '-inf'}, 4.])
        self.assertEqual(manifest['event_count'], 4)
        self.assertEqual(manifest['streams'][3]['kind'], 'other')
        self.assertIn('uint64 id', manifest['streams'][3]['definition'])
        for name, record in manifest['files'].items():
            self.assertEqual(record['sha256'], CONVERT.sha256(self.output/name))
        self.assertEqual(manifest['source']['sha256'], CONVERT.sha256(self.bag))

    def test_clip_and_no_overwrite(self):
        write_fixture(self.bag)
        manifest = CONVERT.convert(self.bag, self.output, duration_ns=1)
        self.assertEqual(manifest['event_count'], 2)
        self.assertEqual(len(manifest['streams']), 4)
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            CONVERT.convert(self.bag, self.output)
        self.assertEqual(json.loads((self.output/'manifest.json').read_text())['event_count'], 2)

    def test_failed_conversion_is_not_published(self):
        write_fixture(self.bag, invalid_image=True)
        with self.assertRaisesRegex(ValueError, 'image buffer'):
            CONVERT.convert(self.bag, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob('.recording-*')), [])

    def test_cpp_pipeline(self):
        if PIPELINE is None:
            self.skipTest("Pass --pipeline with the C++ recording_pipeline executable")
        write_fixture(self.bag)
        CONVERT.convert(self.bag, self.output)
        subprocess.run([str(PIPELINE), str(self.output)], check=True, timeout=10)

    def test_calibration_and_duration_validation(self):
        write_fixture(self.bag)
        calibration = self.root/'calibration'
        calibration.mkdir()
        for name in ('imu.yaml', 'left_camera.yaml', 'right_camera.yaml'):
            (calibration/name).write_text('original: calibration\n')
        manifest = CONVERT.convert(self.bag, self.output, calibration)
        self.assertEqual(len(manifest['calibration']), 3)
        for record in manifest['calibration']:
            self.assertEqual((self.output/record['path']).read_text(), 'original: calibration\n')
        with self.assertRaisesRegex(ValueError, 'positive'):
            CONVERT.convert(self.bag, self.root/'bad', duration_ns=0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--pipeline', type=Path)
    args, test_args = parser.parse_known_args()
    PIPELINE = args.pipeline.resolve(strict=True) if args.pipeline is not None else None
    unittest.main(argv=[sys.argv[0], *test_args])
