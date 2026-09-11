"""Server tests: the page's JSON API driving the fake MG400, and the limits."""

import math
import unittest

from fake_mg400 import FakeMG400, wait_for
from mg400 import server
from mg400.driver import DobotMG400
from mg400.limits import clamp_pose


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeMG400()
        ports = self.fake.ports

        class Robot(DobotMG400):              # the server's robot, pointed at the fake
            def __init__(self, ip):
                super().__init__(ip, command_timeout=0.5, ports=ports)

        self._real_robot = server.DobotMG400
        server.DobotMG400 = Robot
        self.api = server.app.test_client()

    def tearDown(self):
        self.api.post("/api/disconnect")
        server.DobotMG400 = self._real_robot
        server._speed_ratio = 10
        self.fake.close()

    def post(self, path, body=None):
        return self.api.post(path, json=body or {}).get_json()

    def test_drive_then_lose_the_link(self):
        self.assertTrue(self.post("/api/connect", {"ip": "127.0.0.1"})["ok"])
        self.assertTrue(self.post("/api/enable")["ok"])
        self.assertTrue(self.post("/api/speed", {"ratio": 100})["ok"])
        x0 = self.fake.pose[0]
        res = self.post("/api/move", {"x": x0 + 20, "y": 0, "z": -500})
        self.assertTrue(res["ok"])
        self.assertEqual(res["clamped"]["z"], server.Z_FLOOR)
        self.assertTrue(wait_for(lambda: abs(self.fake.pose[2] - server.Z_FLOOR) < 0.01,
                                 timeout=5.0))

        self.fake.feedback_paused = True
        self.assertTrue(wait_for(lambda: self.api.get("/api/status").get_json()["link_error"]))
        res = self.post("/api/move", {"x": x0, "y": 0, "z": 0})
        self.assertFalse(res["ok"])
        self.assertIn("Connection lost", res["error"])


class LimitsTest(unittest.TestCase):
    def test_z_floor(self):
        self.assertEqual(clamp_pose(300, 0, -200, 0, z_floor=-72)[2], -72)
        self.assertEqual(clamp_pose(300, 0, -100, 0)[2], -100)

    def test_reach_ring(self):
        x, y, _, _ = clamp_pose(50, 50, 0, 0)
        self.assertAlmostEqual(math.hypot(x, y), 150)
        x, y, _, _ = clamp_pose(500, 0, 0, 0)
        self.assertAlmostEqual(math.hypot(x, y), 440)


if __name__ == "__main__":
    unittest.main()
