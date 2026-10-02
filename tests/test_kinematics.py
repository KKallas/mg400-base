"""The joint-limit model against poses read back from a real lab MG400."""

import unittest

from mg400 import kinematics as k

# (J2, J3) from GetAngle and (x, y, z) from GetPose on the same arm, and what
# the controller said about that pose.
MEASURED = [
    ((16.873, 69.115), (216.50, 51.22, -49.86), "ok"),
    ((65.013, 105.0), (216.149, 51.035, -148.519), "alarm 34 (J3 stop)"),
    ((-9.347, 50.918), (186.84, 42.338, -17.004), "alarm 73"),
    ((-8.114, 50.918), (190.454, 43.157, -16.435), "ok"),
    ((-9.543, 50.497), (190.296, 25.395, -16.286), "alarm 73"),
    ((-8.144, 50.497), (194.464, 25.952, -15.632), "ok"),
]


class KinematicsTest(unittest.TestCase):
    def test_inverse_matches_the_robot(self):
        for (j2, j3), p, _ in MEASURED:
            _, a2, a3 = k.inverse(*p)
            self.assertAlmostEqual(a2, j2, delta=0.1)
            self.assertAlmostEqual(a3, j3, delta=0.1)

    def test_forward_inverse_round_trip(self):
        for j in ((10, 20, 60), (-30, 0, 40), (45, 50, 90)):
            self.assertEqual([round(v, 6) for v in k.inverse(*k.forward(*j))],
                             [round(float(v), 6) for v in j])

    def test_alarm_poses_are_rejected_safe_poses_pass(self):
        for _, p, said in MEASURED:
            ok = k.check(*p, margin=0.5)[0]   # an arm sitting on its stop alarms
            self.assertEqual(ok, said == "ok", f"{p}: robot said {said}")

    def test_jog_out_picks_the_reverse_joint(self):
        self.assertEqual(k.jog_out((12.8, -9.35, 50.92)), "J2+")   # alarm 73
        self.assertEqual(k.jog_out((13.3, 65.0, 105.0)), "J3-")    # J3 stop
        self.assertIsNone(k.jog_out((13.3, 16.9, 69.1)))

    def test_line_toward_the_base_stops_before_the_limit(self):
        start, end = (300.0, 0.0, 0.0), (120.0, 0.0, 0.0)
        stop = k.furthest_reachable(start, end)
        self.assertTrue(k.check(*stop)[0])
        self.assertGreater(stop[0], 150.0)


if __name__ == "__main__":
    unittest.main()


class GuardTest(unittest.TestCase):
    """The follower stops a move toward the base before the parallelogram limit."""

    def test_follower_holds_at_the_limit(self):
        from fake_mg400 import FakeMG400, wait_for
        from mg400 import server
        from mg400.driver import DobotMG400

        fake = FakeMG400()
        ports = fake.ports

        class Robot(DobotMG400):
            def __init__(self, ip):
                super().__init__(ip, command_timeout=0.5, ports=ports)

        real = server.DobotMG400
        server.DobotMG400 = Robot
        api = server.app.test_client()
        try:
            api.post("/api/connect", json={"ip": "127.0.0.1"})
            self.assertTrue(api.post("/api/enable").json["ok"])
            api.post("/api/speed", json={"ratio": 100})
            api.post("/api/move", json={"x": 150, "y": 0, "z": 0})
            self.assertTrue(wait_for(lambda: api.get("/api/status").json["limit_warning"], 6.0))
            x = fake.pose[0]
            self.assertGreater(x, 150.5)
            self.assertTrue(k.check(*fake.pose[:3], margin=0.0)[0])
            # backing away is allowed
            api.post("/api/move", json={"x": 300, "y": 0, "z": 0})
            self.assertTrue(wait_for(lambda: fake.pose[0] > x + 50, 6.0))
        finally:
            api.post("/api/disconnect")
            server.DobotMG400 = real
            fake.close()
