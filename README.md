# mg400-base

The starting point for driving a **Dobot MG400** from Python. One package, three
things:

| | |
|---|---|
| `mg400 …` | a command line: status, enable, move, pump, digital outputs |
| `mg400 serve` | one web page: X/Y/Z/R sliders, speed, ten saved locations, pump buttons, a command log — and the same as a JSON API |
| `mg400.driver` | the `DobotMG400` class behind both, for your own code |

The driver and the page come from the `cartesian-xyz-test` prototype in
[KKallas/manual-override](https://github.com/KKallas/manual-override); the hub
and relay layers are removed so it runs on its own. The follower and fault
fixes come from the same project's relay driver (`relay_arm.py`, Kaur3 branch).
Dropping a bad connection instead of locking up is new here — see
[When something goes wrong](#when-something-goes-wrong). The Mac setup below is
from the same project.

## Install

```sh
git clone https://github.com/KKallas/mg400-base.git
cd mg400-base
python3 -m venv .venv && source .venv/Scripts/activate
pip install -e .
mg400 --help
```

Python 3.10 or newer; the only dependency is Flask.

## 1. Cable and address

The MG400 ships at **192.168.1.6** on its **LAN1** port. Your computer goes on
the same subnet, with a fixed address:

| | |
|---|---|
| cable | your Ethernet port (or USB-C → Ethernet adapter) straight to the robot's LAN1 |
| your IPv4 | **192.168.1.50**, manually |
| subnet mask | 255.255.255.0 |
| router / DNS | leave empty |

macOS: System Settings → Network → the Ethernet adapter → Details → TCP/IP →
Configure IPv4 *Manually*. Then `ping 192.168.1.6` from a terminal. No reply
means cable, port, or address — nothing below works until this does.

## 2. API mode

The controller serves its TCP ports only while **TCP/IP secondary development**
("API mode") is switched on. It is a one-time setting per robot and the robot
keeps it while powered; the lab robots normally already have it on. Check:

```sh
mg400 status
```

Ports the robot now serves:

| Port | Channel | Used for |
|------|---------|----------|
| 29999 | dashboard | EnableRobot, DisableRobot, ClearError, SpeedFactor, DO, GetPose |
| 30003 | motion | MovL (point to point), ServoP (streamed setpoints) |
| 30004 | feedback | 1440-byte status packet every ~8 ms: mode, joints, tool pose, DO bits |

If `mg400 status` is refused while the ping works, API mode is off. Switching it
on is done once with Dobot Studio Pro. On a Windows PC with a wired port:
[docs/dobot-api-mode-windows.md](docs/dobot-api-mode-windows.md). On a Mac it runs in a UTM Windows VM —
steps in [docs/dobot-api-mode.md](docs/dobot-api-mode.md). Nothing else in this
package needs Studio Pro, and **only one program may send motion commands at a
time**, so it is not running while the page is.

## 3. Run the page

```sh
mg400 serve                 # http://localhost:8000/
mg400 serve --port 8080 --locations data/positions.json
```

On the page: **Connect → Enable**. Enable waits (up to 5 s) until the arm
reports it is enabled, then the sliders sync to the current pose, so the first
move does not jump. The speed starts low — 10 % (20 mm/s) with a 0.5 s ramp —
then drag X / Y / Z / R. A slider sets a target; the server streams `ServoP`
toward it at ~25 Hz with velocity and acceleration caps, so dragging gives
smooth bounded motion instead of queued moves. X, Y and Z move together: a
move from rest runs along the straight line to the target.

**Stop motion** freezes the pose. The hardware **E-stop** on the robot base is
the only stop to trust.

Saved locations are ten fixed slots: **Set** captures the slider pose, **Recall**
sends the arm there, the label and numbers are editable. They are written to
`locations.json` in the directory you started from (or `--locations`), so they
survive a restart and can live in your repo.

## 4. The pump box

The pump box hangs on two of the robot's digital outputs — one line for suction,
one for blow — and the page's three buttons set them so that at most one is
high. The package assumes **DO2 = suck, DO1 = blow**. That is a guess from an
earlier setup: **check the pump box manual and the wiring before the first pick**,
and change it with environment variables if it differs:

```sh
MG400_SUCK_DO=2 MG400_BLOW_DO=1 mg400 serve
mg400 pump suck --suck-do 2 --blow-do 1
```

Wire the DO lines with the robot disabled and the box unplugged; it runs on 24 V.

## Command line

| Command | Does |
|---------|------|
| `mg400 status` | mode, enabled/error flags, pose, joints, DO/DI bits |
| `mg400 enable` / `disable` / `clear` | EnableRobot (after ClearError) / DisableRobot / ClearError |
| `mg400 pose` | current tool pose X Y Z R |
| `mg400 move X Y Z [R]` | MovL to a pose and wait for arrival (`--no-wait`, `--timeout`, `--tol`); refuses Z below the floor |
| `mg400 pump suck\|blow\|off` | the pump box on its two DO lines |
| `mg400 do INDEX 0\|1` | one digital output |
| `mg400 speed N` | SpeedFactor 1–100 |
| `mg400 serve` | the page + API (`--host`, `--port`, `--locations`) |

`--ip` (or `MG400_IP`) selects the robot; default `192.168.1.6`. `--z-floor`
(or `MG400_Z_FLOOR`) sets the lowest Z, default −72 mm. Both go before the
command: `mg400 --z-floor -60 serve`. Every command connects, runs, and
disconnects, so they are safe to call from a shell script or from another
program.

## HTTP API

All JSON. Every reply carries `ok`; failures carry `error`, robot errors also
`errid`.

| Method | Path | Body | Effect |
|--------|------|------|--------|
| GET | `/api/config` | | workspace limits, Z floor, default IP, pump DO indices |
| GET | `/api/status` | | live state: mode, enabled, error and `fault_label`, pose, joints, DO bits, pump mode, target, slots, `link_error`, `servo_error`, `stalled` |
| GET | `/api/events` | | the same as a Server-Sent Events stream, ~5×/s while it changes |
| POST | `/api/connect` | `{"ip":"192.168.1.6"}` | open the three links |
| POST | `/api/disconnect` | | close them |
| POST | `/api/enable` | | ClearError + EnableRobot, wait until the arm reports enabled, start the follower |
| POST | `/api/disable` | | stop the follower, DisableRobot |
| POST | `/api/clear_error` | | ClearError |
| POST | `/api/speed` | `{"ratio":10}` | SpeedFactor and the follower's velocity cap (100 % = 200 mm/s) |
| POST | `/api/smoothness` | `{"secs":0.5}` | follower ramp time |
| POST | `/api/stop` | | hold the current pose |
| POST | `/api/move` | `{"x":250,"y":0,"z":50,"r":0}` | new target pose, clamped to the workspace |
| POST | `/api/pump` | `{"mode":"suck"}` | `suck` / `blow` / `off` |
| POST | `/api/do` | `{"index":1,"value":1}` | one digital output |
| GET | `/api/locations` | | the ten slots |
| POST | `/api/locations/<n>` | `{"name":"source","pose":{…}}` | label and/or pose of slot n (1–10) |
| POST | `/api/locations/<n>/clear` | | empty slot n |
| POST | `/api/recall/<n>` | | send the arm to slot n |

From another Python program in the same process, `mg400.server` also exposes
`robot_ready()`, `current_pose()`, `move_to(x, y, z, r, wait=True)` and
`pump(mode)`.

## Workspace limits

The reachable area of an MG400 is a ring, not a box. Targets are clamped to
Z −150…230 mm, R ±160°, and an X/Y radius of 150–440 mm from the base axis
(`WORKSPACE`, `RADIUS_MIN`, `RADIUS_MAX` in `mg400/limits.py`). These are
approximate; the controller is the final authority and rejects the rest as a
ServoP error, which shows in the page's log. After three in a row the follower
stops; **Clear Error** re-arms it.

On top of that, no target goes below the **Z floor** (default −72 mm, the value
the Manual Override rig used), so the tool stops above the table. It depends on
your table and tool, so measure it once: jog down slowly until the tool nearly
touches, read Z, add a few millimetres, and start the server with
`mg400 --z-floor <that> serve` (or set `MG400_Z_FLOOR`). The page's Z slider
stops at the floor, saved locations are clamped to it, and `mg400 move` refuses
anything below it.

## When something goes wrong

The page says what happened instead of silently ignoring the sliders:

| What happened | What the page shows | What to do |
|---|---|---|
| The robot answered late or out of turn, closed the connection, or sent no status for 1 s (cable, adapter, Mac asleep, hung controller) | red **Connection lost** with the reason | check the cable, then **Connect** |
| The arm's unlock button was pressed, or it was disabled or faulted | *Servo follower stopped … press Enable* in the log | **Enable**; the arm stays where it is, no jump back |
| The controller accepts moves but the arm does not move | Follower **STALLED** | check the E-stop and unlock button, then **Clear Error** |
| The arm stopped with an alarm | Errors shows which: E-stop, workspace/joint limit, collision | fix the cause, then **Clear Error** |

Why a bad connection is dropped rather than retried: every command gets exactly
one reply. After a late reply, each later answer would belong to the command
before it (a pose read would return the last command's answer), so the only
safe fix is fresh connections. Nothing restarts on its own; **Connect** and
**Enable** are always your call.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| no ping to 192.168.1.6 | address, mask, cable, or the wrong robot port | redo step 1; cable in LAN1 |
| ping works, `mg400 status` refuses | API mode not enabled | docs/dobot-api-mode-windows.md or docs/dobot-api-mode.md |
| connects but will not move | not enabled, or another program holds control | **Enable**; stop the other program |
| *Connection lost* | cable or adapter, the computer went to sleep, or another program (Dobot Studio) talking to the robot | one program at a time; **Connect** again |
| *arm still DISABLED 5 s after EnableRobot* | E-stop pressed, or the robot is in an alarm | release the E-stop, **Clear Error**, **Enable** |
| macOS asks about incoming connections | firewall prompt for python | allow it |
| pump does the opposite | DO indices swapped | section 4 |

## Tests without a robot

```sh
python -m unittest discover -s tests -v
```

`tests/fake_mg400.py` is a fake robot on localhost with the MG400's three ports
and reply format. The tests use it to inject late replies, stopped feedback, the
unlock button and a controller that ignores moves, and check how the driver and
the server handle each (about 15 s). They test the software, not the arm: new
motion code still gets its first run on the robot at low speed.

## Safety

Hands out of the 440 mm radius while a command is pending. First run of any new
move at 20 % speed, tool 20 mm above the surface. E-stop within reach. Pump box
wiring only with the robot disabled and the box unpowered.
