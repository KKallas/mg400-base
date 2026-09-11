# Putting the Dobot MG400 into API Mode

Before any of our software can talk to a robot, the MG400 controller has to be
switched into **TCP/IP secondary development** mode (what we call "API mode").
This opens the control ports our driver uses:

| Port | Channel | Used for |
|------|---------|----------|
| 29999 | Dashboard | enable, disable, clear error, stop, speed, digital outputs |
| 30003 | Motion | `MovL` / `ServoP` / `JointMovJ` etc. |
| 30004 | Feedback | 1440-byte real-time status @ ~8 ms |

It is a one-time setting per robot; the controller keeps it as long as it stays
powered. The lab robots normally arrive with it already on. Check first:

```bash
ping 192.168.1.6
mg400 status
```

Ping works but `mg400 status` is refused → API mode is off. Follow this guide.

This guide documents the exact setup we use: a **Mac (Apple Silicon)** host with
a **USB-Ethernet dongle** wired to the robot, and **Dobot Studio Pro** running in
a **Windows 11 ARM64** VM under [UTM](https://mac.getutm.app) to flip the mode
on. On a Windows PC with a wired Ethernet port, use
[dobot-api-mode-windows.md](dobot-api-mode-windows.md) instead.

---

## 1. Why this layout

Dobot Studio Pro is Windows software, so it runs in the VM. But our control
software (the `mg400` CLI and the `mg400 serve` page in this package) is Python
and runs **directly on the Mac host**, which has the dongle and a direct route to
the robot.

So the two machines have different jobs:

- **Windows VM (Dobot Studio Pro)** — used *once* to enable API mode (and for
  manual jogging / diagnostics).
- **Mac host (our Python client)** — the actual API client during sessions.

```
   ┌──────────────────────── Mac host (Apple Silicon) ─────────────────────────┐
   │                                                                           │
   │   Python client  ──────────────┐                                          │
   │   (mg400 CLI / serve page)     │ direct, 192.168.1.x                      │
   │                                ▼                                          │
   │   USB-Ethernet dongle  en?  192.168.1.50  ───────cable────►  ┌──────────┐ │
   │        ▲                                                     │  MG400   │ │
   │        │ host NAT (UTM shared / emulated VLAN)               │ 192.168. │ │
   │   ┌────┴───────────────┐                                     │  1.6     │ │
   │   │  UTM: Windows 11   │  Dobot Studio Pro ─► enable API ───►└──────────┘ │
   │   │  ARM64 VM          │                                                  │
   │   └────────────────────┘                                                  │
   └───────────────────────────────────────────────────────────────────────────┘
```

The VM reaches the robot **through the host** (NAT), so we keep UTM's network as
the **emulated VLAN (Shared Network)** — we do **not** USB-pass-through the dongle
into the VM. The dongle stays owned by macOS.

| Device | IP | Set where |
|--------|----|-----------|
| Robot (MG400 LAN1) | `192.168.1.6` (factory default) | on the robot |
| Mac USB-Ethernet dongle | `192.168.1.50` / `255.255.255.0` | macOS Network settings |
| Windows VM | NAT address (e.g. `192.168.64.x`) | automatic, via UTM shared |

---

## 2. Wire it up

1. Plug the USB-Ethernet dongle into the Mac.
2. Connect an Ethernet cable from the dongle to the robot's **LAN1** port.
3. Power on the MG400 and wait for it to finish booting.

---

## 3. Set the Mac dongle to a static IP

macOS → **System Settings → Network →** the USB-Ethernet adapter:

- **Configure IPv4:** Manually
- **IP address:** `192.168.1.50`
- **Subnet mask:** `255.255.255.0`
- **Router:** leave blank

This puts the host on the robot's subnet (`192.168.1.0/24`). Verify from a Mac
terminal:

```bash
ping 192.168.1.6
```

If the robot replies, the host ↔ robot link is good. (If not, see
[Troubleshooting](#7-troubleshooting).)

---

## 4. Keep UTM on the emulated VLAN (shared network)

In UTM, open the Windows 11 VM's settings → **Network**:

- **Network Mode:** *Shared Network* (this is the emulated VLAN / NAT mode).
- Leave it as the **emulated** adapter — do **not** bridge it and do **not**
  pass the USB dongle through to the VM.

In Shared Network mode the guest is NAT'd behind macOS and can reach anything the
host can reach — including `192.168.1.6` via the dongle. That's why this mode
works without bridging.

---

## 5. Enable API mode in Dobot Studio Pro

Boot the Windows 11 VM and install **Dobot Studio Pro** if it isn't already.

1. Launch **Dobot Studio Pro**.
2. **Connect to the robot** at IP `192.168.1.6`.
   - If the VM can't see the robot, confirm `ping 192.168.1.6` works *from inside
     Windows* (open `cmd` → `ping 192.168.1.6`). If the host can ping but the VM
     can't, recheck the Shared Network setting in step 4.
3. Go to **Settings → Remote Control → TCP/IP secondary development** and
   **enable** it.
4. Apply / confirm. The controller now serves ports **29999 / 30003 / 30004**.

> **One controller at a time.** While our Python client is driving the robot,
> don't also jog it from Dobot Studio Pro — only one program should send motion
> commands. Once API mode is enabled, you can leave Studio Pro idle, or close it;
> the robot keeps serving the API ports as long as it stays powered.

---

## 6. Verify from the Mac host

API mode is enabled at the controller, so the Mac host can now connect directly.

Quick port check from a Mac terminal:

```bash
# dashboard port should accept a TCP connection
nc -vz 192.168.1.6 29999
```

Then use this package (see the [README](../README.md)):

```bash
mg400 status
mg400 serve
```

Open <http://localhost:8000>, press **Connect → Enable**. Live pose feedback on the
page means all three channels are up.

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Mac can't `ping 192.168.1.6` | Dongle IP/subnet wrong, cable, or wrong robot port | Re-check step 3; confirm cable in LAN1; confirm robot IP is still `.6` |
| Host pings robot, VM can't | UTM not on Shared Network, or dongle passed through to VM | Set Network Mode = Shared Network (step 4); keep dongle owned by macOS |
| Studio Pro connects but API ports refuse | TCP/IP secondary development not enabled | Re-do step 5 and apply |
| Our client connects but robot won't move | Robot not enabled, or Studio Pro still holds control | Press **Enable** in our UI; stop commanding from Studio Pro |
| Connects then drops | Two controllers fighting, or robot in error | Use one controller; **Clear Error** then **Enable** |
| macOS blocks the connection | Firewall prompt for `python` | Allow it in System Settings → Network → Firewall |

---

## 8. Reverting to normal mode

To hand the robot back to Dobot Studio Pro for manual use, disable
**Settings → Remote Control → TCP/IP secondary development** (or simply stop the
Python client and jog from Studio Pro — but don't run both at once).
