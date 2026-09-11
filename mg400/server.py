"""
MG400 base station — one Flask app: the control page at / and its JSON API
under /api/. Start it with `mg400 serve` (or `python -m mg400 serve`).

Derived from the "cartesian-xyz-test" prototype in
github.com/KKallas/manual-override, with the hub and relay layers removed so it
runs on its own. A slider on the page sets a target pose; a background thread in
the driver streams ServoP setpoints toward it with velocity limiting, so dragging
gives smooth, bounded-speed motion instead of queued point-to-point moves.

Saved locations are ten fixed slots persisted to a JSON file (see --locations,
default ./locations.json), so /api/recall/<n> is always a valid call.

Put the robot in API mode first — see README.md. Keep the hardware E-stop within
reach and start at a low speed.
"""

import argparse
import json
import os
import threading

from flask import Flask, jsonify, request, send_from_directory

from . import live
from .driver import DobotMG400, DobotError
from .limits import DEFAULT_Z_FLOOR, RADIUS_MAX, RADIUS_MIN, WORKSPACE, clamp_pose

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")

# ---- configuration --------------------------------------------------------
DEFAULT_IP = os.environ.get("MG400_IP", "192.168.1.6")

# Workspace limits and the clamp live in limits.py (shared with the CLI). The Z
# floor is the lowest Z any target may have; set it for your table with
# --z-floor or MG400_Z_FLOOR.
Z_FLOOR = float(os.environ.get("MG400_Z_FLOOR", DEFAULT_Z_FLOOR))

# Following speed at 100% on the speed slider.
MAX_LIN_VEL = 200.0  # mm/s
MAX_ANG_VEL = 90.0   # deg/s
# Follower easing: time to ramp from rest to the speed cap (and to brake to a
# stop). Larger ramp = gentler start/stop and a longer braking distance.
RAMP_SECS = 0.50

# Air pump box on two DO lines: at most one energised, both low = off.
# Verify these against the pump box manual before the first Enable.
SUCK_DO_INDEX = int(os.environ.get("MG400_SUCK_DO", 2))
BLOW_DO_INDEX = int(os.environ.get("MG400_BLOW_DO", 1))

NUM_SLOTS = 10

app = Flask(__name__, static_folder=None)

_robot = None
_robot_lock = threading.Lock()
# Sampled state (live pose/feedback from the arm), so the SSE stream re-snapshots
# on a short interval rather than on a bump. See live.py.
_live = live.LiveState()

# Motion shaping: last speed % and ramp time, pushed to the follower together as
# velocity + acceleration caps.
_speed_ratio = 10     # % of MAX_LIN_VEL / MAX_ANG_VEL → 20 mm/s, 9 deg/s
_ramp_secs = RAMP_SECS


def _apply_motion(robot):
    """Push the current speed + smoothness to the follower as velocity and
    acceleration caps (acceleration = speed-cap / ramp-time)."""
    frac = _speed_ratio / 100.0
    secs = max(0.05, _ramp_secs)
    robot.set_max_velocity(frac * MAX_LIN_VEL, frac * MAX_ANG_VEL)
    robot.set_max_accel(frac * MAX_LIN_VEL / secs, frac * MAX_ANG_VEL / secs)


# ---- saved locations: NUM_SLOTS fixed slots, persisted ---------------------
LOCATIONS_PATH = os.path.join(os.getcwd(), "locations.json")
_loc_lock = threading.Lock()
_slots = [{"name": "", "x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0, "set": False}
          for _ in range(NUM_SLOTS)]


def _slot_public(i):
    """Slot i (0-based) as sent to clients; `slot` is the 1-based number."""
    s = _slots[i]
    return {"slot": i + 1, "name": s["name"], "set": s["set"],
            "x": s["x"], "y": s["y"], "z": s["z"], "r": s["r"]}


def _load_locations():
    try:
        with open(LOCATIONS_PATH) as f:
            saved = json.load(f).get("slots", [])
    except (OSError, ValueError, TypeError):
        return
    for i in range(min(NUM_SLOTS, len(saved))):
        s = saved[i]
        if not isinstance(s, dict):
            continue
        try:
            _slots[i] = {
                "name": str(s.get("name", ""))[:40],
                "x": float(s.get("x", 0)), "y": float(s.get("y", 0)),
                "z": float(s.get("z", 0)), "r": float(s.get("r", 0)),
                "set": bool(s.get("set", False)),
            }
        except (TypeError, ValueError):
            pass


def _save_locations():
    try:
        with open(LOCATIONS_PATH, "w") as f:
            json.dump({"slots": _slots}, f, indent=2)
    except OSError:
        pass


def configure(locations_path=None, z_floor=None):
    """Set the locations file and Z floor, load the locations. Called by run()
    before serving."""
    global LOCATIONS_PATH, Z_FLOOR
    if locations_path:
        LOCATIONS_PATH = os.path.abspath(locations_path)
    if z_floor is not None:
        Z_FLOOR = float(z_floor)
    _load_locations()


def _current():
    return _robot


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _clamp_pose(x, y, z, r):
    """Clamp a target into the reachable workspace and above the Z floor."""
    return clamp_pose(x, y, z, r, Z_FLOOR)


# ---- programmatic API (import mg400.server from your own code) -------------
def robot_ready():
    """True if the arm is connected and enabled (follower running)."""
    r = _robot
    if r is None or not r.is_connected():
        return False
    return bool(r.get_state().get("enabled"))


def current_pose():
    """Live [x, y, z, r] from the feedback stream, or None if not connected."""
    r = _robot
    if r is None or not r.is_connected():
        return None
    pose = r.get_state().get("pose")
    return list(pose) if pose else None


def move_to(x, y, z, r=None, wait=True, timeout=12.0, tol=2.0):
    """Move the tool to a workspace pose via the ServoP follower, clamped into the
    reachable workspace. If `wait`, poll the live pose until within `tol` mm on
    XYZ or `timeout`. Returns (ok, reason)."""
    import time
    robot = _robot
    if robot is None or not robot.is_connected():
        return False, "robot not connected"
    if r is None:
        cur = current_pose()
        r = cur[3] if cur else 0.0
    x, y, z, r = _clamp_pose(float(x), float(y), float(z), float(r))
    robot.set_target_pose(x, y, z, r)
    if not wait:
        return True, None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        cur = current_pose()
        if cur and abs(cur[0] - x) <= tol and abs(cur[1] - y) <= tol and abs(cur[2] - z) <= tol:
            return True, None
        time.sleep(0.03)
    return False, "move timed out"


def pump(mode):
    """Set the air pump: 'suck' | 'blow' | 'off'. Returns (ok, reason)."""
    robot = _robot
    if robot is None or not robot.is_connected():
        return False, "robot not connected"
    if mode not in ("suck", "blow", "off"):
        return False, "bad pump mode"
    try:
        errid, _ = robot.set_pump(mode, SUCK_DO_INDEX, BLOW_DO_INDEX)
        return errid == 0, (None if errid == 0 else f"pump errid {errid}")
    except Exception as e:  # pragma: no cover - defensive
        return False, str(e)


def _ok(**kw):
    return jsonify({"ok": True, **kw})


def _fail(error, **kw):
    return jsonify({"ok": False, "error": error, **kw})


def _not_connected(robot):
    """The failure reply for a command with no working link, saying why the
    link was dropped if it was."""
    err = robot.get_state().get("link_error") if robot is not None else None
    return _fail(f"Connection lost ({err}). Press Connect." if err else "Not connected")


def _command(fn):
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    try:
        errid, resp = fn(robot)
        return jsonify({"ok": errid == 0, "errid": errid, "resp": resp})
    except DobotError as e:
        return _fail(str(e), errid=e.errid)
    except Exception as e:  # pragma: no cover
        return _fail(str(e))


# ---- pages ----------------------------------------------------------------
@app.route("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.route("/api/config")
def config():
    return jsonify({
        "workspace": WORKSPACE,
        "radius_min": RADIUS_MIN,
        "radius_max": RADIUS_MAX,
        "z_floor": Z_FLOOR,
        "default_ip": DEFAULT_IP,
        "suck_do": SUCK_DO_INDEX,
        "blow_do": BLOW_DO_INDEX,
    })


def _pump_mode(do_bits):
    suck = bool(do_bits & (1 << (SUCK_DO_INDEX - 1)))
    blow = bool(do_bits & (1 << (BLOW_DO_INDEX - 1)))
    if suck and blow:
        return "conflict"
    if suck:
        return "suck"
    if blow:
        return "blow"
    return "off"


def _status_dict():
    robot = _current()
    st = DobotMG400._blank_state() if robot is None else robot.get_state()
    st["pump_mode"] = _pump_mode(st.get("digital_out", 0))
    # The commanded target lets other open windows sync their sliders to it.
    st["target"] = None if robot is None else robot.get_target()
    st["ramp_secs"] = _ramp_secs
    st["speed_ratio"] = _speed_ratio
    with _loc_lock:
        st["slots"] = [_slot_public(i) for i in range(NUM_SLOTS)]
    return st


@app.route("/api/status")
def status():
    return jsonify(_status_dict())


@app.route("/api/events")
def events():
    """Push the live arm status (pose, mode, pump, target) ~5x/s while it changes."""
    return _live.stream(_status_dict, interval=0.2)


@app.route("/api/connect", methods=["POST"])
def connect():
    """Open the three TCP links to the MG400: {"ip": "192.168.1.6"}."""
    global _robot
    ip = (request.json or {}).get("ip", DEFAULT_IP)
    with _robot_lock:
        if _robot is not None:
            _robot.close()
            _robot = None
        robot = DobotMG400(ip)
        try:
            robot.connect()
        except DobotError as e:
            return _fail(f"Could not connect to {ip}: {e}")
        _robot = robot
    return _ok(ip=ip)


@app.route("/api/disconnect", methods=["POST"])
def disconnect():
    global _robot
    with _robot_lock:
        if _robot is not None:
            _robot.close()
            _robot = None
    return _ok()


@app.route("/api/enable", methods=["POST"])
def enable():
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    try:
        robot.clear_error()
    except DobotError:
        pass
    try:
        errid, resp = robot.enable()
    except DobotError as e:
        return _fail(str(e), errid=e.errid)
    if errid == 0:
        robot.start_servo()
        _apply_motion(robot)   # push current speed + smoothness to the follower
    return jsonify({"ok": errid == 0, "errid": errid, "resp": resp})


@app.route("/api/disable", methods=["POST"])
def disable():
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    robot.stop_servo()
    return _command(lambda r: r.disable())


@app.route("/api/clear_error", methods=["POST"])
def clear_error():
    return _command(lambda r: r.clear_error())


@app.route("/api/speed", methods=["POST"])
def speed():
    global _speed_ratio
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    ratio = _clamp(int((request.json or {}).get("ratio", 30)), 1, 100)
    _speed_ratio = ratio
    _apply_motion(robot)
    return _command(lambda r: r.speed_factor(ratio))


@app.route("/api/smoothness", methods=["POST"])
def smoothness():
    """Set the follower's ramp/brake time in seconds (the 'Smoothness' knob):
    larger = gentler start/stop with no overshoot; smaller = snappier."""
    global _ramp_secs
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    try:
        secs = float((request.json or {}).get("secs", RAMP_SECS))
    except (TypeError, ValueError):
        return _fail("secs must be a number")
    _ramp_secs = max(0.05, min(1.5, secs))
    _apply_motion(robot)
    return _ok(ramp_secs=_ramp_secs)


@app.route("/api/stop", methods=["POST"])
def stop():
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    robot.hold()
    return _ok()


@app.route("/api/pump", methods=["POST"])
def pump_route():
    mode = (request.json or {}).get("mode", "off")
    if mode not in ("suck", "blow", "off"):
        return _fail("mode must be 'suck', 'blow' or 'off'")
    return _command(lambda r: r.set_pump(mode, SUCK_DO_INDEX, BLOW_DO_INDEX))


@app.route("/api/do", methods=["POST"])
def digital_output():
    """Raw digital output: {"index": 1, "value": 1}. The pump buttons are the
    two-line special case of this."""
    data = request.json or {}
    try:
        index = int(data["index"])
        value = int(data["value"])
    except (KeyError, TypeError, ValueError):
        return _fail("Expected integer index and value")
    return _command(lambda r: r.set_digital_output(index, value))


@app.route("/api/move", methods=["POST"])
def move():
    """Update the Cartesian target pose. Clamped to the reachable workspace; the
    follower streams ServoP toward it at the velocity cap."""
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    data = request.json or {}
    try:
        x = float(data["x"])
        y = float(data["y"])
        z = float(data["z"])
        r = float(data.get("r", robot.get_state()["pose"][3]))
    except (KeyError, ValueError, TypeError):
        return _fail("Expected numeric x, y, z (r optional)")
    x, y, z, r = _clamp_pose(x, y, z, r)
    robot.set_target_pose(x, y, z, r)
    return _ok(clamped={"x": round(x, 2), "y": round(y, 2),
                        "z": round(z, 2), "r": round(r, 2)})


# ---- saved locations: NUM_SLOTS fixed slots (edit / set / recall) ----------
@app.route("/api/locations", methods=["GET"])
def list_locations():
    with _loc_lock:
        return jsonify({"slots": [_slot_public(i) for i in range(NUM_SLOTS)]})


@app.route("/api/locations/<int:n>", methods=["POST", "PATCH"])
def set_location(n):
    """Edit slot n: set its label (`name`) and/or its pose (`pose`). Passing a
    pose marks the slot filled and clamps it to the reachable workspace; the
    page sends its current slider pose for the 'Set' button."""
    if not (1 <= n <= NUM_SLOTS):
        return _fail("slot out of range")
    data = request.json or {}
    with _loc_lock:
        s = _slots[n - 1]
        if "name" in data:
            s["name"] = str(data["name"])[:40]
        pose = data.get("pose")
        if isinstance(pose, dict):
            try:
                x, y, z, r = float(pose["x"]), float(pose["y"]), float(pose["z"]), float(pose.get("r", 0))
            except (KeyError, TypeError, ValueError):
                return _fail("bad pose")
            x, y, z, r = _clamp_pose(x, y, z, r)
            s["x"], s["y"], s["z"], s["r"] = round(x, 2), round(y, 2), round(z, 2), round(r, 2)
            s["set"] = True
        _save_locations()
        out = _slot_public(n - 1)
    _live.bump()   # push the change to every open window now
    return _ok(slot=out)


@app.route("/api/locations/<int:n>/clear", methods=["POST"])
def clear_location(n):
    """Empty slot n (label + pose) but keep the slot itself in place."""
    if not (1 <= n <= NUM_SLOTS):
        return _fail("slot out of range")
    with _loc_lock:
        _slots[n - 1] = {"name": "", "x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0, "set": False}
        _save_locations()
    _live.bump()
    return _ok()


@app.route("/api/recall/<int:n>", methods=["POST"])
def recall_location(n):
    """Send the arm to slot n (sets the follower target). ok:false if the slot is
    empty or the robot isn't connected."""
    robot = _current()
    if robot is None or not robot.is_connected():
        return _not_connected(robot)
    if not (1 <= n <= NUM_SLOTS):
        return _fail("slot out of range")
    with _loc_lock:
        s = _slots[n - 1]
        if not s["set"]:
            return _fail(f"slot {n} is empty")
        x, y, z, r = _clamp_pose(s["x"], s["y"], s["z"], s["r"])
        out = _slot_public(n - 1)
    robot.set_target_pose(x, y, z, r)
    return _ok(slot=out)


# ---- entry point ----------------------------------------------------------
def run(host="0.0.0.0", port=8000, locations=None, z_floor=None):
    configure(locations, z_floor)
    print(f"MG400 base station on http://{host}:{port}/  "
          f"(robot default {DEFAULT_IP}, Z floor {Z_FLOOR:g} mm, "
          f"locations {LOCATIONS_PATH})")
    # threaded so the SSE stream does not block the command routes
    app.run(host=host, port=port, threaded=True, debug=False)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m mg400.server", description=__doc__.split("\n\n")[0])
    p.add_argument("--host", default="0.0.0.0", help="bind address (default: all interfaces)")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--locations", default=None,
                   help="saved-locations JSON file (default: ./locations.json)")
    p.add_argument("--z-floor", type=float, default=None,
                   help=f"lowest target Z in mm (default: MG400_Z_FLOOR or {DEFAULT_Z_FLOOR:g})")
    args = p.parse_args(argv)
    return run(args.host, args.port, args.locations, args.z_floor)


if __name__ == "__main__":
    main()
