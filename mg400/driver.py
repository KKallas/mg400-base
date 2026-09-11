"""
Dobot MG400 TCP/IP driver — Cartesian (TCP / workspace) prototype.

Same three communication layers as the joint prototype, but live control drives
the **Cartesian tool pose** (X, Y, Z, R) instead of joint angles:

  * Dashboard  (29999) — enable/disable/clear/stop/estop/speed/digital outputs.
  * Motion     (30003) — ServoP streamed pose setpoints for smooth following
                         (MovL also available for point-to-point linear moves).
  * Feedback   (30004) — 1440-byte real-time packet @ ~8 ms: robot mode, actual
                         joint angles, and the actual TCP pose (tool_vector_actual
                         at byte offset 624).

X/Y/Z are millimetres, R is the end-effector rotation about Z in degrees. As with
the joint prototype, a background thread streams setpoints toward a target while
velocity-limiting, so dragging a slider produces smooth motion rather than queued
point-to-point moves.
"""

import socket
import struct
import threading
import time
import json


# ---- ports ----------------------------------------------------------------
PORT_DASHBOARD = 29999
PORT_MOTION = 30003
PORT_FEEDBACK = 30004

# ---- feedback packet (offsets per Dobot's MyType struct) ------------------
FEEDBACK_SIZE = 1440
FEEDBACK_MAGIC = 0x0123456789ABCDEF
OFF_DIGITAL_IN = 8       # int64
OFF_DIGITAL_OUT = 16     # int64
OFF_ROBOT_MODE = 24      # int64
OFF_TEST_VALUE = 48      # int64 (magic, validates alignment)
OFF_Q_ACTUAL = 432       # 6 x double (actual joint angles, degrees)
OFF_TOOL_VECTOR_ACTUAL = 624  # 6 x double (actual TCP pose: x,y,z,rx,ry,rz)

ROBOT_MODES = {
    1: "INIT", 2: "BRAKE_OPEN", 3: "RESERVED", 4: "DISABLED",
    5: "ENABLED (idle)", 6: "BACKDRIVE", 7: "RUNNING", 8: "SINGLE_MOVE",
    9: "ERROR", 10: "PAUSE", 11: "JOG",
}
ENABLED_MODES = {5, 6, 7, 8, 10, 11}


class DobotError(Exception):
    def __init__(self, errid, resp, command):
        self.errid = errid
        self.resp = resp
        self.command = command
        super().__init__(f"{command} -> ErrorID {errid}: {resp}")


def parse_feedback(packet):
    """Parse a 1440-byte feedback packet; return dict or None if misaligned."""
    if len(packet) < FEEDBACK_SIZE:
        return None
    if struct.unpack_from("<Q", packet, OFF_TEST_VALUE)[0] != FEEDBACK_MAGIC:
        return None
    robot_mode = struct.unpack_from("<Q", packet, OFF_ROBOT_MODE)[0]
    di = struct.unpack_from("<Q", packet, OFF_DIGITAL_IN)[0]
    do = struct.unpack_from("<Q", packet, OFF_DIGITAL_OUT)[0]
    q_actual = struct.unpack_from("<6d", packet, OFF_Q_ACTUAL)
    tool = struct.unpack_from("<6d", packet, OFF_TOOL_VECTOR_ACTUAL)
    return {
        "robot_mode": int(robot_mode),
        "digital_in": int(di),
        "digital_out": int(do),
        "joints": [round(v, 3) for v in q_actual[:4]],
        "pose": [round(v, 3) for v in tool[:4]],  # x, y, z, r
    }


class DobotMG400:
    def __init__(self, ip, connect_timeout=5.0, command_timeout=5.0):
        self.ip = ip
        self.connect_timeout = connect_timeout
        self.command_timeout = command_timeout

        self._dashboard = None
        self._motion = None
        self._feedback = None

        self._dash_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._feed_thread = None
        self._running = False

        # Cartesian follower state
        self._servo_thread = None
        self._servo_running = False
        self._servo_lock = threading.Lock()
        self._target = None       # desired pose [x, y, z, r]
        self._setpoint = None     # current streamed pose
        self._vel = [0.0, 0.0, 0.0, 0.0]   # current setpoint velocity per axis
        self._max_lin = 80.0      # mm/s
        self._max_ang = 60.0      # deg/s
        self._max_lin_acc = 400.0 # mm/s^2 (ramp ~0.2 s to 80 mm/s; lower = gentler)
        self._max_ang_acc = 300.0 # deg/s^2

        self._state = self._blank_state()

    # -- state ----------------------------------------------------------------
    @staticmethod
    def _blank_state():
        return {
            "connected": False,
            "robot_mode": 0,
            "mode_name": "DISCONNECTED",
            "enabled": False,
            "error": False,
            "joints": [0.0, 0.0, 0.0, 0.0],
            "pose": [0.0, 0.0, 0.0, 0.0],
            "target": None,
            "digital_in": 0,
            "digital_out": 0,
            "last_feedback": 0.0,
            "feedback_ok": False,
            "servo_active": False,
            "servo_error": None,
        }

    def get_state(self):
        with self._state_lock:
            return dict(self._state)

    def get_target(self):
        """The commanded tool-pose target the follower is slewing toward, or
        None before the servo loop is initialised. Exposing it lets several
        control windows sync their sliders to the same setpoint."""
        with self._servo_lock:
            return list(self._target) if self._target is not None else None

    def is_connected(self):
        with self._state_lock:
            return self._state["connected"]

    # -- connection -----------------------------------------------------------
    def _open(self, port, read_timeout):
        s = socket.create_connection((self.ip, port), timeout=self.connect_timeout)
        s.settimeout(read_timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return s

    def connect(self):
        try:
            self._dashboard = self._open(PORT_DASHBOARD, self.command_timeout)
            self._motion = self._open(PORT_MOTION, self.command_timeout)
            self._feedback = self._open(PORT_FEEDBACK, 2.0)
        except OSError as e:
            self.close()
            raise DobotError(-1, str(e), "connect")
        self._running = True
        with self._state_lock:
            self._state["connected"] = True
        self._feed_thread = threading.Thread(
            target=self._feed_loop, name="dobot-feedback", daemon=True
        )
        self._feed_thread.start()

    def close(self):
        self.stop_servo()
        self._running = False
        for sock in (self._feedback, self._motion, self._dashboard):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
        self._dashboard = self._motion = self._feedback = None
        if self._feed_thread and self._feed_thread.is_alive():
            self._feed_thread.join(timeout=1.0)
        self._feed_thread = None
        with self._state_lock:
            self._state = self._blank_state()

    # -- low-level command I/O ------------------------------------------------
    def _recv_response(self, sock):
        data = b""
        while b";" not in data:
            chunk = sock.recv(1024)
            if not chunk:
                raise DobotError(-1, "connection closed by robot", "recv")
            data += chunk
        return data.decode("utf-8", errors="replace").strip()

    @staticmethod
    def _parse_errid(resp):
        try:
            return int(resp.split(",", 1)[0])
        except (ValueError, IndexError):
            return -1

    def _command(self, sock, lock, command, raise_on_error=True):
        if sock is None:
            raise DobotError(-1, "not connected", command)
        with lock:
            try:
                sock.sendall(command.encode("utf-8"))
                resp = self._recv_response(sock)
            except (OSError, socket.timeout) as e:
                raise DobotError(-1, str(e), command)
        errid = self._parse_errid(resp)
        if raise_on_error and errid != 0:
            raise DobotError(errid, resp, command)
        return errid, resp

    def _dash(self, command, raise_on_error=True):
        return self._command(self._dashboard, self._dash_lock, command, raise_on_error)

    def _move(self, command, raise_on_error=True):
        return self._command(self._motion, self._motion_lock, command, raise_on_error)

    # -- dashboard (control) --------------------------------------------------
    def enable(self):
        return self._dash("EnableRobot()")

    def disable(self):
        return self._dash("DisableRobot()")

    def clear_error(self):
        return self._dash("ClearError()")

    def reset(self):
        return self._dash("ResetRobot()")

    def speed_factor(self, ratio):
        ratio = max(1, min(100, int(ratio)))
        return self._dash(f"SpeedFactor({ratio})")

    def set_digital_output(self, index, status, immediate=True):
        value = 1 if status else 0
        cmd = (
            f"DOExecute({int(index)},{value})"
            if immediate
            else f"DO({int(index)},{value})"
        )
        return self._dash(cmd, raise_on_error=False)

    def set_pump(self, mode, suck_do, blow_do):
        """Air pump box (I/O mode): two independent lines (suck / blow). Energise
        at most one; both low = off. See the joint prototype for the rationale."""
        mode = (mode or "").lower()
        resp = []
        errid = 0

        def out(index, value, label):
            nonlocal errid
            e, r = self.set_digital_output(index, value)
            resp.append(f"{label}={r}")
            errid = e or errid

        if mode == "suck":
            out(blow_do, 0, "blow")
            out(suck_do, 1, "suck")
        elif mode == "blow":
            out(suck_do, 0, "suck")
            out(blow_do, 1, "blow")
        elif mode == "off":
            out(suck_do, 0, "suck")
            out(blow_do, 0, "blow")
        else:
            raise DobotError(-1, f"unknown pump mode {mode!r}", "set_pump")
        return errid, "; ".join(resp)

    def get_pose(self):
        """Query the current TCP pose via the dashboard. Returns [x, y, z, r]."""
        _, resp = self._dash("GetPose()")
        vals = self._extract_floats(resp)
        return vals[:4]

    # -- motion ---------------------------------------------------------------
    def mov_l(self, x, y, z, r):
        """Point-to-point linear Cartesian move (queued). Returns (errid, resp)."""
        return self._move(
            f"MovL({x:.3f},{y:.3f},{z:.3f},{r:.3f})", raise_on_error=False
        )

    def servo_p(self, x, y, z, r):
        """Stream one Cartesian servo setpoint. ServoP takes no optional params
        and should be sent at <= ~33 Hz."""
        return self._move(
            f"ServoP({x:.3f},{y:.3f},{z:.3f},{r:.3f})", raise_on_error=False
        )

    # -- smooth live following (ServoP streaming) -----------------------------
    def set_max_velocity(self, lin_mm_s, ang_deg_s):
        with self._servo_lock:
            self._max_lin = max(1.0, float(lin_mm_s))
            self._max_ang = max(1.0, float(ang_deg_s))

    def set_max_accel(self, lin_mm_s2, ang_deg_s2):
        """Acceleration caps for the follower. Lower = gentler ramp up/down and a
        longer braking distance, which removes the overshoot that a hard velocity
        step causes; higher = snappier but can overshoot on stop."""
        with self._servo_lock:
            self._max_lin_acc = max(1.0, float(lin_mm_s2))
            self._max_ang_acc = max(1.0, float(ang_deg_s2))

    def set_target_pose(self, x, y, z, r=None):
        with self._servo_lock:
            if self._target is None:
                self._target = [0.0, 0.0, 0.0, 0.0]
            self._target[0] = float(x)
            self._target[1] = float(y)
            self._target[2] = float(z)
            if r is not None:
                self._target[3] = float(r)

    def hold(self):
        """Smooth stop: aim the target at the follower's natural braking point so
        it decelerates to rest instead of snapping (which would overshoot)."""
        with self._servo_lock:
            if self._setpoint is None:
                return
            tgt = list(self._setpoint)
            accels = (self._max_lin_acc, self._max_lin_acc,
                      self._max_lin_acc, self._max_ang_acc)
            for i, a in enumerate(accels):
                v = self._vel[i]
                if v:                       # coast to a stop one braking-distance ahead
                    tgt[i] += (1.0 if v > 0 else -1.0) * (v * v) / (2.0 * max(1.0, a))
            self._target = tgt

    def start_servo(self):
        self.stop_servo()
        deadline = time.time() + 1.0
        while time.time() < deadline and not self.get_state()["feedback_ok"]:
            time.sleep(0.02)
        pose = [float(v) for v in self.get_state()["pose"]]
        with self._servo_lock:
            self._setpoint = list(pose)
            self._target = list(pose)
            self._vel = [0.0, 0.0, 0.0, 0.0]
        self._servo_running = True
        with self._state_lock:
            self._state["servo_active"] = True
            self._state["servo_error"] = None
        self._servo_thread = threading.Thread(
            target=self._servo_loop, name="dobot-servo", daemon=True
        )
        self._servo_thread.start()

    def stop_servo(self):
        self._servo_running = False
        thread = self._servo_thread
        if thread and thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=1.0)
        self._servo_thread = None
        with self._state_lock:
            self._state["servo_active"] = False

    def _servo_loop(self):
        interval = 0.04   # 25 Hz (ServoP minimum cycle is ~30 ms)
        consecutive_errors = 0
        next_t = time.monotonic()
        while self._servo_running:
            with self._servo_lock:
                target = list(self._target) if self._target is not None else None
                setpoint = list(self._setpoint) if self._setpoint is not None else None
                vel = list(self._vel)
                caps = ((self._max_lin, self._max_lin_acc),
                        (self._max_lin, self._max_lin_acc),
                        (self._max_lin, self._max_lin_acc),
                        (self._max_ang, self._max_ang_acc))
            if target is None or setpoint is None:
                time.sleep(interval)
                continue
            dt = interval
            # Acceleration-limited slew: ramp velocity up toward the cap, then brake
            # early enough (v <= sqrt(2*a*dist)) to arrive at the target with ~zero
            # speed. This eases the start AND the stop, so the arm tracks a smooth
            # velocity profile instead of overshooting and creeping back.
            for i, (v_max, a) in enumerate(caps):
                remaining = target[i] - setpoint[i]
                dist = abs(remaining)
                direction = 1.0 if remaining >= 0 else -1.0
                v_brake = (2.0 * a * dist) ** 0.5        # fastest we can still stop from
                v_des = min(v_max, v_brake) * direction   # desired signed velocity
                dv = v_des - vel[i]                        # ramp velocity by <= a*dt
                max_dv = a * dt
                dv = max(-max_dv, min(max_dv, dv))
                vel[i] += dv
                step = vel[i] * dt
                if abs(step) >= dist and (step >= 0) == (remaining >= 0):
                    setpoint[i] = target[i]               # would reach/pass it this tick
                    vel[i] = 0.0
                else:
                    setpoint[i] += step
            with self._servo_lock:
                self._setpoint = list(setpoint)
                self._vel = list(vel)
            errid, resp = self.servo_p(*setpoint)
            if errid != 0:
                consecutive_errors += 1
                if consecutive_errors >= 3:
                    with self._state_lock:
                        self._state["servo_error"] = f"ServoP ErrorID {errid}: {resp}"
                    break
            else:
                consecutive_errors = 0
            next_t += interval
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
        self._servo_running = False
        with self._state_lock:
            self._state["servo_active"] = False

    # -- response parsing helpers ---------------------------------------------
    @staticmethod
    def _extract_braces(resp):
        start = resp.find("{")
        end = resp.rfind("}")
        if start == -1 or end == -1 or end <= start:
            return ""
        return resp[start + 1 : end]

    def _extract_floats(self, resp):
        out = []
        for tok in self._extract_braces(resp).split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                out.append(float(tok))
            except ValueError:
                pass
        return out

    def get_error_id(self):
        _, resp = self._dash("GetErrorID()", raise_on_error=False)
        try:
            nested = json.loads(self._extract_braces(resp))
        except (ValueError, TypeError):
            return []
        ids = []

        def walk(x):
            if isinstance(x, list):
                for v in x:
                    walk(v)
            elif isinstance(x, (int, float)) and int(x) != 0:
                ids.append(int(x))

        walk(nested)
        return ids

    # -- feedback loop --------------------------------------------------------
    def _feed_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._feedback.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= FEEDBACK_SIZE:
                parsed = parse_feedback(buf[:FEEDBACK_SIZE])
                if parsed is None:
                    buf = buf[1:]
                    continue
                buf = buf[FEEDBACK_SIZE:]
                self._apply_feedback(parsed)
        with self._state_lock:
            self._state["connected"] = False
            self._state["feedback_ok"] = False
            self._state["mode_name"] = "DISCONNECTED"

    def _apply_feedback(self, parsed):
        mode = parsed["robot_mode"]
        with self._state_lock:
            self._state["robot_mode"] = mode
            self._state["mode_name"] = ROBOT_MODES.get(mode, f"UNKNOWN({mode})")
            self._state["enabled"] = mode in ENABLED_MODES
            self._state["error"] = mode == 9
            self._state["joints"] = parsed["joints"]
            self._state["pose"] = parsed["pose"]
            self._state["digital_in"] = parsed["digital_in"]
            self._state["digital_out"] = parsed["digital_out"]
            self._state["last_feedback"] = time.time()
            self._state["feedback_ok"] = True
