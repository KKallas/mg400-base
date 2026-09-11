"""
Driver tests against the fake MG400 — no robot needed:

    python -m unittest discover -s tests -v

Each test injects one fault seen on real robots and checks that the driver
notices it, instead of locking up or answering the wrong command.
"""

import math
import time
import unittest

from fake_mg400 import FakeMG400, wait_for
from mg400.driver import DobotError, DobotMG400, slew


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def _dist(a, b):
    return _norm([a[i] - b[i] for i in range(3)])


def _off_line(p, a, b):
    """Distance of point p from the line through a and b."""
    d = [b[i] - a[i] for i in range(3)]
    w = [p[i] - a[i] for i in range(3)]
    cross = [w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]]
    return _norm(cross) / _norm(d)


class DriverTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeMG400()
        self.robot = DobotMG400("127.0.0.1", command_timeout=0.5, ports=self.fake.ports)
        self.robot.connect()
        self.assertTrue(wait_for(lambda: self.robot.get_state()["feedback_ok"]))

    def tearDown(self):
        self.robot.close()
        self.fake.close()

    def state(self, key):
        return self.robot.get_state()[key]

    def follow(self):
        self.robot.enable()
        self.robot.start_servo()

    # -- enabling ---------------------------------------------------------------
    def test_enable_waits_until_the_arm_is_enabled(self):
        self.fake.enable_delay = 0.5
        t = time.monotonic()
        self.follow()
        self.assertGreaterEqual(time.monotonic() - t, 0.4)
        x0 = self.fake.pose[0]
        self.robot.set_target_pose(x0 + 30, 0, 0, 0)
        self.assertTrue(wait_for(lambda: abs(self.fake.pose[0] - (x0 + 30)) < 0.01))
        self.assertTrue(self.state("servo_active"))

    def test_enable_gives_up_if_the_arm_never_enables(self):
        self.fake.never_enable = True
        with self.assertRaises(DobotError) as ctx:
            self.robot.enable(timeout=0.5)
        self.assertIn("DISABLED", str(ctx.exception))

    # -- a bad link is dropped, never reused -------------------------------------
    def test_late_motion_reply_drops_the_link(self):
        self.follow()
        self.fake.delay_next["motion"] = 1.0
        self.assertTrue(wait_for(lambda: self.state("link_error")))
        self.assertIn("ServoP", self.state("link_error"))
        self.assertFalse(self.state("connected"))
        self.assertFalse(self.state("servo_active"))
        self.assertTrue(self.state("servo_error"))
        t = time.monotonic()
        with self.assertRaises(DobotError) as ctx:
            self.robot.get_pose()
        self.assertLess(time.monotonic() - t, 0.1)      # fails at once, no timeout wait
        self.assertIn("connection lost", str(ctx.exception))

    def test_late_dashboard_reply_never_answers_the_next_command(self):
        self.fake.delay_next["dash"] = 1.0
        with self.assertRaises(DobotError):
            self.robot.clear_error()
        time.sleep(0.7)                   # the late reply has arrived by now
        with self.assertRaises(DobotError):
            self.robot.get_pose()         # the old driver returned ClearError's reply

    def test_reply_to_another_command_drops_the_link(self):
        self.fake.extra_reply["dash"] = "0,{},ClearError();"
        with self.assertRaises(DobotError):
            self.robot.get_pose()
        self.assertIn("out of step", self.state("link_error"))

    def test_frozen_feedback_drops_the_link(self):
        self.fake.feedback_paused = True
        self.assertTrue(wait_for(lambda: not self.state("connected")))
        self.assertIn("no feedback", self.state("link_error"))

    def test_robot_closing_the_connection_drops_the_link(self):
        self.fake.drop_connections()
        self.assertTrue(wait_for(lambda: self.state("link_error")))

    def test_close_does_not_hang_on_a_silent_robot(self):
        self.robot.close()
        self.robot = DobotMG400("127.0.0.1", ports=self.fake.ports)   # 5 s command timeout
        self.robot.connect()
        self.fake.silent_dash = True
        t = time.monotonic()
        self.robot.close()
        self.assertLess(time.monotonic() - t, 1.5)

    # -- follower ---------------------------------------------------------------
    def test_unlock_and_drag_does_not_snap_back(self):
        self.follow()
        x0 = self.fake.pose[0]
        self.fake.drag_by_hand([x0 + 60, 0.0, 0.0, 0.0])
        time.sleep(0.5)
        self.fake.mode = 5                 # locked again
        time.sleep(1.0)
        self.assertAlmostEqual(self.fake.pose[0], x0 + 60, places=2)
        self.assertFalse(self.state("servo_active"))
        self.assertIn("Enable", self.state("servo_error"))

    def test_wedged_controller_is_flagged_stalled(self):
        self.follow()
        self.fake.ignore_servo = True
        self.robot.set_target_pose(self.fake.pose[0] + 50, 0, 0, 0)
        self.assertTrue(wait_for(lambda: self.state("stalled"), timeout=4.0))

    def test_xyz_moves_along_a_straight_line(self):
        self.follow()
        p0 = list(self.fake.pose[:3])
        p1 = [p0[0] + 60, p0[1] + 30, p0[2] - 40]
        self.fake.servo_points.clear()
        self.robot.set_target_pose(*p1, 0)
        self.assertTrue(wait_for(lambda: _dist(self.fake.pose, p1) < 0.01, timeout=5.0))
        self.assertGreater(len(self.fake.servo_points), 10)
        for p in self.fake.servo_points:
            self.assertLess(_off_line(p, p0, p1), 0.01)

    def test_hold_brakes_along_the_line(self):
        self.follow()
        p0 = list(self.fake.pose[:3])
        p1 = [p0[0] + 150, p0[1] + 80, p0[2] - 50]
        self.robot.set_target_pose(*p1, 0)
        time.sleep(0.8)
        self.robot.hold()
        time.sleep(1.0)
        end = self.fake.pose[:3]
        self.assertLess(_off_line(end, p0, p1), 0.01)
        self.assertLess(_dist(p0, end), _dist(p0, p1) - 10)     # stopped short of it


class SlewTest(unittest.TestCase):
    def test_speed_and_acceleration_caps_hold_through_a_new_target(self):
        sp, v = [0.0] * 4, [0.0] * 4
        target = [300.0, 100.0, -50.0, 90.0]
        dt, v_max, acc = 0.04, 80.0, 400.0
        for tick in range(2000):
            if tick == 30:
                target = [0.0, 300.0, 0.0, 0.0]     # changes while moving
            prev = v
            sp, v = slew(sp, v, target, v_max, acc, 60.0, 300.0, dt)
            if sp == target:
                break
            if sp[:3] != target[:3]:                # the arrival tick stops dead
                self.assertLessEqual(_norm(v[:3]), v_max + 1e-6)
                self.assertLessEqual(_norm([v[i] - prev[i] for i in range(3)]), acc * dt + 1e-6)
        self.assertEqual(sp, target)


if __name__ == "__main__":
    unittest.main()
