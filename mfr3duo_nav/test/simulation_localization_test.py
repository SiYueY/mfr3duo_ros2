"""SE2 composition and freshness gates for the simulation-only localization source."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import TimeReference

spec = importlib.util.spec_from_file_location('simulation_localization',
    Path(__file__).resolve().parents[1] / 'tools/simulation_localization.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pose(x, y, angle):
    value = Pose()
    value.position.x, value.position.y = float(x), float(y)
    value.orientation.z, value.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
    return value


class LocalizationTests(unittest.TestCase):
    def test_correction_composes_with_odometry(self):
        measured, odometry = pose(2.4, -1.4, 1.3), pose(.7, -.3, -.2)
        x, y, angle = module.correction(measured, odometry)
        self.assertAlmostEqual(x + math.cos(angle) * .7 + math.sin(angle) * .3, 2.4)
        self.assertAlmostEqual(y + math.sin(angle) * .7 - math.cos(angle) * .3, -1.4)
        self.assertAlmostEqual(angle - .2, 1.3)

    def test_malformed_pose_rejected(self):
        value = pose(0, 0, 0)
        self.assertTrue(module.valid_pose(value))
        value.orientation.w = 2.
        self.assertFalse(module.valid_pose(value))
        value = pose(0, 0, 0)
        value.position.x = float('nan')
        self.assertFalse(module.valid_pose(value))

    def test_publish_requires_active_fresh_matching_samples(self):
        measured = PoseStamped()
        measured.header.stamp.sec = 10
        measured.pose = pose(2.4, -1.4, 1.3)
        close, distant = Odometry(), Odometry()
        close.header.stamp.sec, distant.header.stamp.sec = 10, 9
        close.pose.pose, distant.pose.pose = pose(.7, -.3, -.2), pose(5, 5, .9)
        clock = SimpleNamespace(nanoseconds=10_000_000_000, to_msg=Mock(return_value=measured.header.stamp))
        fake = SimpleNamespace(active=True, measured=measured, odometry=[distant, close],
            physics_received=20., get_clock=lambda: SimpleNamespace(now=lambda: clock), broadcaster=Mock())
        with patch.object(module.time, 'monotonic', return_value=20.):
            module.SimulationLocalization.publish(fake)
            transform = fake.broadcaster.sendTransform.call_args.args[0]
            expected = module.correction(measured.pose, close.pose.pose)
            self.assertAlmostEqual(transform.transform.translation.x, expected[0])
            self.assertEqual(transform.header.frame_id, 'map')
            self.assertEqual(transform.child_frame_id, 'odom')
            for key, value in [('active', False), ('physics_received', 19.), ('odometry', [distant])]:
                original = getattr(fake, key)
                setattr(fake, key, value)
                fake.broadcaster.reset_mock()
                module.SimulationLocalization.publish(fake)
                fake.broadcaster.sendTransform.assert_not_called()
                setattr(fake, key, original)
            for sec in (9, 11):
                measured.header.stamp.sec = sec
                fake.broadcaster.reset_mock()
                module.SimulationLocalization.publish(fake)
                fake.broadcaster.sendTransform.assert_not_called()

    def test_repeated_physics_clock_does_not_refresh_frozen_simulation(self):
        fake = SimpleNamespace(physics_time=None, physics_received=0.)
        message = TimeReference()
        message.time_ref.sec = 7
        with patch.object(module.time, 'monotonic', return_value=20.):
            module.SimulationLocalization.on_time(fake, message)
        with patch.object(module.time, 'monotonic', return_value=21.):
            module.SimulationLocalization.on_time(fake, message)
        self.assertEqual(fake.physics_received, 20.)


if __name__ == '__main__':
    unittest.main()
