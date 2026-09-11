"""
mg400 — command line for the base station.

    mg400 status                    robot mode, pose, DO bits
    mg400 enable | disable | clear  dashboard commands
    mg400 pose                      current tool pose X Y Z R
    mg400 move X Y Z [R]            MovL to a pose, waits until it arrives
    mg400 pump suck|blow|off        the pump box on its two DO lines
    mg400 do INDEX 0|1              one digital output
    mg400 speed N                   SpeedFactor 1–100
    mg400 serve [--port 8000]       the control page + HTTP API

The robot IP comes from --ip, else the MG400_IP environment variable, else
192.168.1.6 (Dobot factory default on LAN1).
"""

import argparse
import os
import sys
import time

from .driver import DobotMG400, DobotError

DEFAULT_IP = os.environ.get("MG400_IP", "192.168.1.6")


def _connect(ip):
    robot = DobotMG400(ip)
    robot.connect()
    # wait for the first feedback packet so state/pose are real
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline and not robot.get_state()["feedback_ok"]:
        time.sleep(0.02)
    return robot


def _fmt_pose(pose):
    return "X {:8.2f}  Y {:8.2f}  Z {:8.2f}  R {:7.2f}".format(*pose)


def cmd_status(robot, args):
    st = robot.get_state()
    print(f"robot   {robot.ip}")
    print(f"mode    {st['robot_mode']} · {st['mode_name']}")
    print(f"enabled {st['enabled']}   error {st['error']}   feedback {st['feedback_ok']}")
    print(f"pose    {_fmt_pose(st['pose'])}")
    print(f"joints  {st['joints']}")
    print(f"DO bits {st['digital_out']:08b}   DI bits {st['digital_in']:08b}")
    if st["error"]:
        print(f"errors  {robot.get_error_id()}")
    return 0


def cmd_enable(robot, args):
    try:
        robot.clear_error()
    except DobotError:
        pass
    errid, resp = robot.enable()
    print(resp)
    return 0 if errid == 0 else 1


def cmd_disable(robot, args):
    errid, resp = robot.disable()
    print(resp)
    return 0 if errid == 0 else 1


def cmd_clear(robot, args):
    errid, resp = robot.clear_error()
    print(resp)
    return 0 if errid == 0 else 1


def cmd_pose(robot, args):
    print(_fmt_pose(robot.get_pose()))
    return 0


def cmd_move(robot, args):
    st = robot.get_state()
    if not st["enabled"]:
        print("robot is not enabled — run `mg400 enable` first", file=sys.stderr)
        return 1
    r = st["pose"][3] if args.r is None else args.r
    errid, resp = robot.mov_l(args.x, args.y, args.z, r)
    print(resp)
    if errid != 0:
        return 1
    if args.no_wait:
        return 0
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        p = robot.get_state()["pose"]
        if all(abs(p[i] - v) <= args.tol for i, v in enumerate((args.x, args.y, args.z))):
            print(_fmt_pose(p))
            return 0
        time.sleep(0.05)
    print("timed out waiting for the move to finish", file=sys.stderr)
    return 1


def cmd_pump(robot, args):
    errid, resp = robot.set_pump(args.mode, args.suck_do, args.blow_do)
    print(resp)
    return 0 if errid == 0 else 1


def cmd_do(robot, args):
    errid, resp = robot.set_digital_output(args.index, args.value)
    print(resp)
    return 0 if errid == 0 else 1


def cmd_speed(robot, args):
    errid, resp = robot.speed_factor(args.ratio)
    print(resp)
    return 0 if errid == 0 else 1


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(prog="mg400", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default=DEFAULT_IP, help=f"robot address (default {DEFAULT_IP})")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("enable").set_defaults(fn=cmd_enable)
    sub.add_parser("disable").set_defaults(fn=cmd_disable)
    sub.add_parser("clear").set_defaults(fn=cmd_clear)
    sub.add_parser("pose").set_defaults(fn=cmd_pose)

    m = sub.add_parser("move", help="MovL to X Y Z [R] and wait")
    m.add_argument("x", type=float)
    m.add_argument("y", type=float)
    m.add_argument("z", type=float)
    m.add_argument("r", type=float, nargs="?", default=None)
    m.add_argument("--no-wait", action="store_true")
    m.add_argument("--timeout", type=float, default=15.0)
    m.add_argument("--tol", type=float, default=2.0, help="arrival tolerance, mm")
    m.set_defaults(fn=cmd_move)

    pm = sub.add_parser("pump", help="pump box: suck | blow | off")
    pm.add_argument("mode", choices=["suck", "blow", "off"])
    pm.add_argument("--suck-do", type=int, default=int(os.environ.get("MG400_SUCK_DO", 2)))
    pm.add_argument("--blow-do", type=int, default=int(os.environ.get("MG400_BLOW_DO", 1)))
    pm.set_defaults(fn=cmd_pump)

    d = sub.add_parser("do", help="set one digital output")
    d.add_argument("index", type=int)
    d.add_argument("value", type=int, choices=[0, 1])
    d.set_defaults(fn=cmd_do)

    s = sub.add_parser("speed", help="SpeedFactor 1–100")
    s.add_argument("ratio", type=int)
    s.set_defaults(fn=cmd_speed)

    sv = sub.add_parser("serve", help="run the control page + HTTP API")
    sv.add_argument("--host", default="0.0.0.0", help="bind address (default: all interfaces)")
    sv.add_argument("--port", type=int, default=8000)
    sv.add_argument("--locations", default=None,
                    help="saved-locations JSON file (default: ./locations.json)")

    args = p.parse_args(argv)

    if args.cmd == "serve":
        from . import server
        server.DEFAULT_IP = args.ip
        return server.run(args.host, args.port, args.locations)

    try:
        robot = _connect(args.ip)
    except DobotError as e:
        print(f"cannot connect to {args.ip}: {e.resp}\n"
              "Is the robot on, cabled to LAN1, in API mode, and your Ethernet "
              "on 192.168.1.x? See README.md.", file=sys.stderr)
        return 2
    try:
        return args.fn(robot, args)
    except DobotError as e:
        print(f"{e}", file=sys.stderr)
        return 1
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
