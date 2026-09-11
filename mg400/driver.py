"""
Dobot MG400 TCP/IP driver — Cartesian (TCP / workspace) control.

Three connections to the controller:

  * Dashboard  (29999) — enable/disable/clear/stop/speed/digital outputs.
  * Motion     (30003) — ServoP streamed pose setpoints for smooth following
                         (MovL also available for point-to-point linear moves).
  * Feedback   (30004) — 1440-byte real-time packet @ ~8 ms: robot mode, actual
                         joint angles, and the actual TCP pose (tool_vector_actual
                         at byte offset 624).

X/Y/Z are millimetres, R is the end-effector rotation about Z in degrees. A
background thread streams setpoints toward a target with velocity and
acceleration caps, so dragging a slider produces smooth motion rather than
queued point-to-point moves. X/Y/Z move together, so a move from rest runs along
the straight line to its target.

When the link goes bad. Dashboard and motion are strict request/reply: one
command, one reply. A reply that does not come in time, or that belongs to a
different command, means every later reply would answer the wrong command — so
the driver never reuses the link after that. It marks the connection lost
(`link_error` in the state), stops the follower and shuts all three sockets;
the caller connects again. Feedback that stops for FEEDBACK_TIMEOUT seconds is
treated the same way: that is how a pulled cable or a hung controller shows up,
since TCP alone notices neither.

Carried over from relay_arm.py in KKallas/manual-override (Kaur3 branch): the
follower stops when the arm is unlocked, disabled or faulted instead of snapping
back to a stale target afterwards; a controller that accepts ServoP but does not
move is flagged `stalled`; and while the arm is in ERROR its alarm IDs are read
and classified.
"""

import json
import math
import socket
import struct
import threading
import time


# ---- ports ----------------------------------------------------------------
PORT_DASHBOARD = 29999
PORT_MOTION = 30003
PORT_FEEDBACK = 30004

# ---- timing ---------------------------------------------------------------
FEEDBACK_TIMEOUT = 1.0     # s without a feedback packet → connection lost
ENABLE_TIMEOUT = 5.0       # s for the arm to report enabled after EnableRobot
CLOSE_STOP_TIMEOUT = 0.5   # s for the StopRobot sent while closing
SERVO_INTERVAL = 0.04      # 25 Hz (ServoP minimum cycle is ~30 ms)

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
# Modes in which the arm is NOT following our ServoP stream: hand-dragged via the
# unlock button (BACKDRIVE), disabled, faulted, or still starting up. Streaming
# into these is ACKed but ignored, and across unlock/lock cycles it wedges the
# controller; resuming afterwards would snap the arm back to a stale setpoint.
NO_SERVO_MODES = {1, 2, 3, 4, 6, 9}
READY_MODES = ENABLED_MODES - NO_SERVO_MODES
NO_SERVO_TRIP = 3          # consecutive follower ticks in NO_SERVO_MODES → stop

# Suspected wedged controller: connected, enabled, ACKs every ServoP with
# ErrorID 0, but never moves. Flag it when the follower is commanding the arm
# somewhere it isn't and the joints have not moved for a grace period.
STALL_GRACE_SECS = 1.5     # commanded-but-static this long → stalled
STALL_MOVE_EPS = 0.2       # deg of joint travel that counts as "the arm moved"
STALL_ERR = 6.0            # mm between streamed setpoint and actual TCP

# Alarm groups from Dobot's MG400 alarm tables. Feedback reports every alarm as
# mode 9 (ERROR); GetErrorID() tells an E-stop from a limit or a collision.
EMERGENCY_ALARM_IDS = {85, 86, 12288, 12289, 20484, 21570}
WORKSPACE_ALARM_IDS = {
    17, 18, 23, 24, 29, 30, 32, 33, 34,
    64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
    20480, 20481, 20482, 20483, 36122,
}
COLLISION_ALARM_IDS = {-3, -2, 112, 12294}


def classify_alarm_ids(alarm_ids):
    """(kind, label) for a list of GetErrorID values."""
    ids = {int(v) for v in (alarm_ids or [])}
    if ids & EMERGENCY_ALARM_IDS:
        return "emergency_lock", "Emergency stop / safety lock"
    if ids & WORKSPACE_ALARM_IDS:
        return "workspace_limit", "Workspace or joint limit"
    if ids & COLLISION_ALARM_IDS:
        return "collision", "Collision stop"
    if ids:
        return "controller_fault", "Controller fault"
    return "unknown_fault", "Fault (no alarm code)"


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


def _reply_name(resp):
    """The command a reply echoes ("0,{},EnableRobot();" → "EnableRobot"), or
    None if the reply does not have that shape."""
    i = resp.rfind("},")
    if i < 0:
        return None
    name = resp[i + 2:].split("(", 1)[0].strip()
    return name if name.isidentifier() else None


def slew(setpoint, vel, target, max_lin, lin_acc, max_ang, ang_acc, dt):
    """One follower tick: returns the next (setpoint, vel) as new lists.

    X/Y/Z are one vector. Its velocity is steered toward the target, capped in
    speed and in acceleration, and braked early enough (v <= sqrt(2*a*dist)) to
    arrive at rest. From rest the setpoint therefore runs along the straight
    line to the target; a target that changes mid-move bends the path smoothly
    instead of cornering. R is ramped the same way on its own."""
    sp, v = list(setpoint), list(vel)

    delta = [target[i] - sp[i] for i in range(3)]
    dist = math.sqrt(sum(d * d for d in delta))
    if dist > 1e-9:
        scale = min(max_lin, math.sqrt(2.0 * lin_acc * dist)) / dist
        desired = [d * scale for d in delta]
    else:
        desired = [0.0, 0.0, 0.0]
    dv = [desired[i] - v[i] for i in range(3)]
    dv_len = math.sqrt(sum(d * d for d in dv))
    if dv_len > lin_acc * dt:
        dv = [d * lin_acc * dt / dv_len for d in dv]
    v[:3] = [v[i] + dv[i] for i in range(3)]
    step = [v[i] * dt for i in range(3)]
    step_len = math.sqrt(sum(s * s for s in step))
    heading_in = sum(step[i] * delta[i] for i in range(3)) > 0
    if dist <= 1e-9 or (heading_in and step_len >= dist):
        sp[:3] = list(target[:3])       # reaches (or would pass) it this tick
        v[:3] = [0.0, 0.0, 0.0]
    else:
        sp[:3] = [sp[i] + step[i] for i in range(3)]

    remaining = target[3] - sp[3]
    direction = 1.0 if remaining >= 0 else -1.0
    v_des = min(max_ang, math.sqrt(2.0 * ang_acc * abs(remaining))) * direction
    v[3] += max(-ang_acc * dt, min(ang_acc * dt, v_des - v[3]))
    r_step = v[3] * dt
    if abs(r_step) >= abs(remaining) and (r_step >= 0) == (remaining >= 0):
        sp[3] = target[3]
        v[3] = 0.0
    else:
        sp[3] += r_step
    return sp, v


class DobotMG400:
    def __init__(self, ip, connect_timeout=5.0, command_timeout=5.0, ports=None):
        self.ip = ip
        self.connect_timeout = connect_timeout
        self.command_timeout = command_timeout
        # (dashboard, motion, feedback); only a test robot on localhost changes it
        self.ports = tuple(ports) if ports else (PORT_DASHBOARD, PORT_MOTION, PORT_FEEDBACK)

        self._dashboard = None
        self._motion = None
        self._feedback = None

        self._dash_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._feed_thread = None
        self._alarm_thread = None
        # Set whenever the link is down (before connect, after close or loss);
        # background threads exit on it.
        self._quit = threading.Event()
        self._quit.set()

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

        # stall detection: when the joints last moved, and from where
        self._last_motion_ts = 0.0
        self._motion_ref = None

        self._state = self._blank_state()

    # -- state ----------------------------------------------------------------
    @staticmethod
    def _blank_state():
        return {
            "connected": False,
            "link_error": None,     # why the connection was dropped, if it was
            "robot_mode": 0,
            "mode_name": "DISCONNECTED",
            "enabled": False,
            "error": False,
            "alarm_ids": [],
            "fault_kind": None,
            "fault_label": None,
            "joints": [0.0, 0.0, 0.0, 0.0],
            "pose": [0.0, 0.0, 0.0, 0.0],
            "target": None,
            "digital_in": 0,
            "digital_out": 0,
            "last_feedback": 0.0,
            "feedback_ok": False,
            "servo_active": False,
            "servo_error": None,
            "stalled": False,       # commanded to move but not moving
            "stall_error": 0.0,     # setpoint-vs-actual gap (mm) behind `stalled`
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

    def _offline_reason(self):
        with self._state_lock:
            err = self._state["link_error"]
        return f"connection lost ({err}) — connect again" if err else "not connected"

    # -- connection -----------------------------------------------------------
    def _open(self, port, read_timeout):
        s = socket.create_connection((self.ip, port), timeout=self.connect_timeout)
        s.settimeout(read_timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return s

    def connect(self):
        dash_port, motion_port, feed_port = self.ports
        try:
            self._dashboard = self._open(dash_port, self.command_timeout)
            self._motion = self._open(motion_port, self.command_timeout)
            self._feedback = self._open(feed_port, 0.2)
        except OSError as e:
            self.close()
            raise DobotError(-1, str(e), "connect")
        self._quit.clear()
        with self._state_lock:
            self._state["connected"] = True
        self._feed_thread = threading.Thread(
            target=self._feed_loop, name="dobot-feedback", daemon=True
        )
        self._feed_thread.start()
        self._alarm_thread = threading.Thread(
            target=self._alarm_loop, name="dobot-alarms", daemon=True
        )
        self._alarm_thread.start()

    def close(self):
        self.stop_servo()
        # Stop anything the controller still has queued before dropping the
        # links, so the arm is not left mid-move. Short timeout: a hung robot
        # must not hold up a disconnect.
        if self.is_connected():
            try:
                self._dashboard.settimeout(CLOSE_STOP_TIMEOUT)
                self.stop_robot()
            except (DobotError, OSError):
                pass
        self._quit.set()
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
        for thread in (self._feed_thread, self._alarm_thread):
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
        self._feed_thread = self._alarm_thread = None
        with self._state_lock:
            self._state = self._blank_state()

    def _link_lost(self, reason):
        """Give up on the connection: record why, stop the follower and shut all
        three sockets, which wakes every thread blocked on them. Not reused
        after this — connect again. Safe to call more than once."""
        with self._state_lock:
            if self._quit.is_set():
                return
            self._quit.set()
            self._state.update(
                connected=False, link_error=reason, feedback_ok=False,
                enabled=False, mode_name="DISCONNECTED", servo_active=False,
            )
        self._servo_running = False
        for sock in (self._feedback, self._motion, self._dashboard):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    # -- low-level command I/O ------------------------------------------------
    def _recv_response(self, sock):
        """Read one reply, up to its ';'. Returns (reply, extra): anything the
        robot sent after the ';' is a reply nobody asked for."""
        data = b""
        while b";" not in data:
            chunk = sock.recv(1024)
            if not chunk:
                raise ConnectionError("connection closed by the robot")
            data += chunk
        reply, _, extra = data.partition(b";")
        return reply.decode("utf-8", errors="replace").strip() + ";", extra.strip()

    @staticmethod
    def _parse_errid(resp):
        try:
            return int(resp.split(",", 1)[0])
        except (ValueError, IndexError):
            return -1

    def _broken(self, reason, command):
        self._link_lost(reason)
        return DobotError(-1, reason, command)

    def _command(self, sock, lock, command, raise_on_error=True):
        name = command.split("(", 1)[0]
        with lock:
            if sock is None or not self.is_connected():
                raise DobotError(-1, self._offline_reason(), command)
            try:
                sock.sendall(command.encode("utf-8"))
                resp, extra = self._recv_response(sock)
            except OSError as e:   # includes timeouts
                raise self._broken(f"no reply to {name}: {e}", command)
            echoed = _reply_name(resp)
            if extra or (echoed and echoed != name):
                raise self._broken(f"replies out of step: sent {name}, got {resp}", command)
        errid = self._parse_errid(resp)
        if raise_on_error and errid != 0:
            raise DobotError(errid, resp, command)
        return errid, resp

    def _dash(self, command, raise_on_error=True):
        return self._command(self._dashboard, self._dash_lock, command, raise_on_error)

    def _move(self, command, raise_on_error=True):
        return self._command(self._motion, self._motion_lock, command, raise_on_error)

    # -- dashboard (control) --------------------------------------------------
    def enable(self, timeout=ENABLE_TIMEOUT):
        """EnableRobot, then wait until the feedback shows the arm ready. The
        controller can reply before the arm has finished enabling; a follower
        started in that gap would see a disabled arm and stop at once."""
        errid, resp = self._dash("EnableRobot()")
        deadline = time.monotonic() + timeout
        while True:
            st = self.get_state()
            if not st["connected"]:
                raise DobotError(-1, self._offline_reason(), "EnableRobot()")
            if st["robot_mode"] in READY_MODES:
                return errid, resp
            if time.monotonic() > deadline:
                raise DobotError(
                    -1, f"arm still {st['mode_name']} {timeout:g} s after EnableRobot",
                    "EnableRobot()")
            time.sleep(0.02)

    def disable(self):
        return self._dash("DisableRobot()")

    def clear_error(self):
        return self._dash("ClearError()")

    def reset(self):
        return self._dash("ResetRobot()")

    def stop_robot(self):
        """Halt queued motion on the controller (StopRobot)."""
        return self._dash("StopRobot()", raise_on_error=False)

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
        self._last_motion_ts = time.time()   # a fresh move gets its stall grace

    def hold(self):
        """Smooth stop: aim the target at the follower's natural braking point so
        it decelerates to rest instead of snapping (which would overshoot). The
        braking point lies on the current direction of travel."""
        with self._servo_lock:
            if self._setpoint is None:
                return
            tgt = list(self._setpoint)
            v = self._vel
            speed = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            if speed:
                reach = speed / (2.0 * self._max_lin_acc)   # braking distance / speed
                for i in range(3):
                    tgt[i] += v[i] * reach
            if v[3]:
                tgt[3] += (1.0 if v[3] > 0 else -1.0) * v[3] * v[3] / (2.0 * self._max_ang_acc)
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
        self._last_motion_ts = time.time()
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
            self._state["stalled"] = False
            self._state["stall_error"] = 0.0

    def _set_servo_error(self, message):
        with self._state_lock:
            self._state["servo_error"] = message

    def _servo_loop(self):
        consecutive_errors = 0
        no_servo_count = 0
        next_t = time.monotonic()
        try:
            while self._servo_running:
                st = self.get_state()
                if not st["connected"]:
                    self._set_servo_error(self._offline_reason())
                    break
                if st["robot_mode"] in NO_SERVO_MODES:
                    # The arm is not consuming our stream (unlocked, disabled or
                    # faulted). Follow its real position so nothing is waiting to
                    # snap it back, and stop if it stays that way; Enable restarts.
                    if st["feedback_ok"]:
                        with self._servo_lock:
                            self._setpoint = list(st["pose"])
                            self._target = list(st["pose"])
                            self._vel = [0.0, 0.0, 0.0, 0.0]
                    no_servo_count += 1
                    if no_servo_count >= NO_SERVO_TRIP:
                        self._set_servo_error(
                            f"arm is {st['mode_name']} (unlocked, disabled or faulted) "
                            "— press Enable to resume")
                        break
                else:
                    no_servo_count = 0
                    with self._servo_lock:
                        setpoint, vel = slew(
                            self._setpoint, self._vel, self._target,
                            self._max_lin, self._max_lin_acc,
                            self._max_ang, self._max_ang_acc, SERVO_INTERVAL)
                        self._setpoint, self._vel = setpoint, vel
                    errid, resp = self.servo_p(*setpoint)
                    if errid != 0:
                        consecutive_errors += 1
                        if consecutive_errors >= 3:
                            self._set_servo_error(f"ServoP ErrorID {errid}: {resp}")
                            break
                    else:
                        consecutive_errors = 0
                next_t += SERVO_INTERVAL
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.monotonic()
        except DobotError as e:     # the link failed; _command already dropped it
            self._set_servo_error(e.resp)
        finally:
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

    # -- background threads ---------------------------------------------------
    def _alarm_loop(self):
        """While the arm reports ERROR, read GetErrorID about once a second so
        the state says which stop it is. Read-only: never clears or enables."""
        while not self._quit.wait(0.25):
            st = self.get_state()
            if not st["error"]:
                if st["fault_kind"]:
                    with self._state_lock:
                        self._state.update(alarm_ids=[], fault_kind=None, fault_label=None)
                continue
            try:
                ids = self.get_error_id()
            except DobotError:
                continue
            kind, label = classify_alarm_ids(ids)
            with self._state_lock:
                if self._state["error"]:
                    self._state.update(alarm_ids=ids, fault_kind=kind, fault_label=label)
            self._quit.wait(0.75)

    def _feed_loop(self):
        buf = b""
        last_packet = time.monotonic()
        reason = "feedback connection closed by the robot"
        while not self._quit.is_set():
            try:
                chunk = self._feedback.recv(4096)
            except socket.timeout:
                chunk = None
            except OSError:
                break
            else:
                if not chunk:
                    break
            if chunk:
                buf += chunk
                while len(buf) >= FEEDBACK_SIZE:
                    parsed = parse_feedback(buf[:FEEDBACK_SIZE])
                    if parsed is None:
                        buf = buf[1:]
                        continue
                    buf = buf[FEEDBACK_SIZE:]
                    last_packet = time.monotonic()
                    self._apply_feedback(parsed)
            if time.monotonic() - last_packet > FEEDBACK_TIMEOUT:
                reason = f"no feedback from the robot for {FEEDBACK_TIMEOUT:g} s"
                break
        self._link_lost(reason)   # no-op when close() or another thread got there first

    def _apply_feedback(self, parsed):
        mode = parsed["robot_mode"]
        now = time.time()
        # Read the follower's intent first; the two locks are never nested.
        with self._servo_lock:
            running = self._servo_running
            setpoint = list(self._setpoint) if self._setpoint is not None else None
        joints = parsed["joints"]
        ref = self._motion_ref
        if ref is None or max(abs(a - b) for a, b in zip(joints, ref)) > STALL_MOVE_EPS:
            self._motion_ref = joints
            self._last_motion_ts = now          # the arm physically moved
        err = 0.0
        stalled = False
        if running and setpoint is not None:
            err = max(abs(setpoint[i] - parsed["pose"][i]) for i in range(3))
            stalled = err > STALL_ERR and now - self._last_motion_ts > STALL_GRACE_SECS
        with self._state_lock:
            self._state["robot_mode"] = mode
            self._state["mode_name"] = ROBOT_MODES.get(mode, f"UNKNOWN({mode})")
            self._state["enabled"] = mode in ENABLED_MODES
            self._state["error"] = mode == 9
            self._state["joints"] = joints
            self._state["pose"] = parsed["pose"]
            self._state["digital_in"] = parsed["digital_in"]
            self._state["digital_out"] = parsed["digital_out"]
            self._state["last_feedback"] = now
            self._state["feedback_ok"] = True
            self._state["stalled"] = stalled
            self._state["stall_error"] = round(err, 2)
