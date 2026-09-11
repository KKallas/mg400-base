# AGENTS.md — mg400-base

What this is: a small Python package that talks to a Dobot MG400 over its TCP/IP
API and serves one control page. Three files matter:

| File | Role |
|------|------|
| `mg400/driver.py` | `DobotMG400` — the three sockets (29999 dashboard, 30003 motion, 30004 feedback), a feedback thread, a ServoP follower thread for smooth slider motion. No Flask. |
| `mg400/server.py` | Flask app: `/` serves `static/index.html`, `/api/*` is the JSON API. Module-level globals hold the one robot. |
| `mg400/cli.py` | `mg400 …` commands. Each one connects, does one thing, closes. `mg400 serve` hands off to `server.main`. |

Run: `pip install -e .` then `mg400 serve` → http://localhost:8000/. Without a
robot the page loads and every command answers `{"ok": false, "error": "Not connected"}`.

Before changing anything:

* The robot must be in API mode (TCP/IP secondary development). It is a
  one-time switch per robot; the lab robots have it on. If not,
  `docs/api-mode-utm.md` is the one-time procedure from a Mac.
* Only one program may send motion commands at a time.
* Pump DO indices (`SUCK_DO_INDEX`, `BLOW_DO_INDEX`) are a guess at the pump box
  wiring. Check them against the box before trusting a pick.
* Workspace limits in `server.py` are approximate; the controller is the
  authority and rejects unreachable poses as ServoP errors.

Extending it: add routes to `server.py` and controls to `static/index.html`. To
drive the arm from another Python program in the same process, import
`mg400.server` and use `robot_ready()`, `current_pose()`, `move_to()`, `pump()`.
For a separate process, use the HTTP API (table in README.md).

Testing without hardware: `python -m mg400 serve` and `curl localhost:8000/api/status`.
There is no simulator; motion code is tested on the robot at 20 % speed with the
E-stop in reach.
