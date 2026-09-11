"""
A fake MG400 on localhost, for tests. It serves the three TCP ports with the
real framing — "ErrorID,{values},Command(args);" replies and 1440-byte feedback
packets every 8 ms — and has switches for the faults seen on real robots.
"""

import math
import socket
import struct
import threading
import time

FEEDBACK_SIZE = 1440
MAGIC = 0x0123456789ABCDEF


def wait_for(cond, timeout=3.0):
    """Poll cond() until it is truthy or `timeout` seconds pass."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return False


def _close(sock):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    sock.close()


class FakeMG400:
    def __init__(self):
        self.mode = 4                         # DISABLED
        self.pose = [300.0, 0.0, 0.0, 0.0]
        self.joints = self._joints_for(self.pose)
        self.servo_points = []                # every ServoP pose received

        # fault switches
        self.enable_delay = 0.0       # EnableRobot replies at once; ENABLED this much later
        self.never_enable = False
        self.feedback_paused = False  # stop sending feedback, keep the socket open
        self.ignore_servo = False     # accept ServoP but don't move (wedged controller)
        self.silent_dash = False      # the dashboard stops replying
        self.delay_next = {"dash": 0.0, "motion": 0.0}     # delay the next reply (s)
        self.extra_reply = {"dash": None, "motion": None}  # sent before the next reply

        self._stop = False
        self._conns = []
        self._listeners = []
        ports = []
        for name in ("dash", "motion", "feed"):
            ls = socket.socket()
            ls.bind(("127.0.0.1", 0))
            ls.listen(4)
            self._listeners.append(ls)
            ports.append(ls.getsockname()[1])
            threading.Thread(target=self._accept, args=(ls, name), daemon=True).start()
        self.ports = tuple(ports)             # (dashboard, motion, feedback)

    # -- test controls ----------------------------------------------------------
    def drag_by_hand(self, pose):
        """The unlock button is pressed and the arm is moved by hand to `pose`."""
        self.mode = 6
        self.pose = list(pose)
        self.joints = self._joints_for(self.pose)

    def drop_connections(self):
        """The robot closes every open connection."""
        for c in list(self._conns):
            _close(c)

    def close(self):
        self._stop = True
        for s in self._listeners + self._conns:
            _close(s)

    # -- internals --------------------------------------------------------------
    @staticmethod
    def _joints_for(pose):
        x, y, z, r = pose
        return [math.degrees(math.atan2(y, x)), (math.hypot(x, y) - 300.0) / 4.0, -z / 4.0, r]

    def _set_mode(self, mode):
        self.mode = mode

    def _accept(self, ls, name):
        while not self._stop:
            try:
                c, _ = ls.accept()
            except OSError:
                return
            self._conns.append(c)
            fn = self._feed if name == "feed" else self._serve
            threading.Thread(target=fn, args=(c, name), daemon=True).start()

    def _packet(self):
        b = bytearray(FEEDBACK_SIZE)
        struct.pack_into("<Q", b, 24, self.mode)
        struct.pack_into("<Q", b, 48, MAGIC)
        struct.pack_into("<6d", b, 432, *(self.joints + [0.0, 0.0]))
        struct.pack_into("<6d", b, 624, *(self.pose + [0.0, 0.0]))
        return bytes(b)

    def _feed(self, c, name):
        try:
            while not self._stop:
                if not self.feedback_paused:
                    c.sendall(self._packet())
                time.sleep(0.008)
        except OSError:
            pass

    def _serve(self, c, name):
        buf = b""
        try:
            while not self._stop:
                data = c.recv(1024)
                if not data:
                    return
                buf += data
                while b")" in buf:
                    cmd, buf = buf.split(b")", 1)
                    self._reply(c, name, cmd.decode() + ")")
        except OSError:
            pass

    def _reply(self, c, name, cmd):
        op, _, args = cmd[:-1].partition("(")
        if name == "dash" and self.silent_dash:
            return
        value = "{}"
        if op == "EnableRobot":
            if not self.never_enable:
                t = threading.Timer(self.enable_delay, self._set_mode, (5,))
                t.daemon = True
                t.start()
        elif op == "DisableRobot":
            self.mode = 4
        elif op == "GetPose":
            value = "{" + ",".join(f"{v:.3f}" for v in self.pose + [0.0, 0.0]) + "}"
        elif op == "GetErrorID":
            value = "{[[],[],[],[],[],[],[]]}"
        elif op in ("ServoP", "MovL"):
            pose = [float(a) for a in args.split(",")]
            if op == "ServoP":
                self.servo_points.append(pose)
            if self.mode in (5, 7) and not self.ignore_servo:
                self.pose = pose
                self.joints = self._joints_for(pose)
        delay, self.delay_next[name] = self.delay_next[name], 0.0
        extra, self.extra_reply[name] = self.extra_reply[name], None
        time.sleep(delay)   # in order: a slow reply holds up everything after it
        if extra:
            c.sendall(extra.encode())
        c.sendall(f"0,{value},{cmd};".encode())
