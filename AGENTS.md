# AGENTS.md — mg400-base

What this is: a small Python package that talks to a Dobot MG400 over its TCP/IP
API and serves one control page. The files that matter:

| File | Role |
|------|------|
| `mg400/driver.py` | `DobotMG400` — the three sockets (29999 dashboard, 30003 motion, 30004 feedback), a feedback thread with a watchdog, an alarm thread, a ServoP follower thread for smooth slider motion. No Flask. |
| `mg400/limits.py` | Workspace limits, the Z floor, `clamp_pose()`. Shared by the server and the CLI. |
| `mg400/server.py` | Flask app: `/` serves `static/index.html`, `/api/*` is the JSON API. Module-level globals hold the one robot. |
| `mg400/cli.py` | `mg400 …` commands. Each one connects, does one thing, closes. `mg400 serve` hands off to `server.run`. |
| `tests/` | `fake_mg400.py` (a fake robot on localhost) and unittest suites for the driver and the server. |

Run: `pip install -e .` then `mg400 serve` → http://localhost:8000/. Without a
robot the page loads and every command answers `{"ok": false, "error": "Not connected"}`.

Before changing anything:

* The robot must be in API mode (TCP/IP secondary development). It is a
  one-time switch per robot; the lab robots have it on. If not,
  `docs/dobot-api-mode.md` is the one-time procedure from a Mac,
  `docs/dobot-api-mode-windows.md` from a Windows PC with a wired port.
* Only one program may send motion commands at a time.
* Dashboard and motion are strict request/reply. After a timeout or a reply
  that echoes a different command, the driver drops the whole link on purpose
  (`link_error`); do not add retries that reuse the sockets, every later reply
  would be off by one. Recovery is a new `connect()`.
* `enable()` waits for the feedback to show the arm ready before returning.
  Starting the follower earlier trips its unlock guard and stops it.
* Pump DO indices (`SUCK_DO_INDEX`, `BLOW_DO_INDEX`) are a guess at the pump box
  wiring. Check them against the box before trusting a pick.
* Workspace limits in `limits.py` are approximate; the controller is the
  authority and rejects unreachable poses as ServoP errors. The Z floor
  (default −72 mm) is per setup: `--z-floor` / `MG400_Z_FLOOR`.

Extending it: add routes to `server.py` and controls to `static/index.html`. To
drive the arm from another Python program in the same process, import
`mg400.server` and use `robot_ready()`, `current_pose()`, `move_to()`, `pump()`.
For a separate process, use the HTTP API (table in README.md).

Testing without hardware: `python -m unittest discover -s tests -v` (about 15 s).
Add a test with a new `FakeMG400` switch when you fix a fault seen on the robot.
The fake checks protocol handling, not motion quality; motion code is still
tried on the robot at 20 % speed with the E-stop in reach.
