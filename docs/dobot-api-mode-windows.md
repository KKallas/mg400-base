# Putting the Dobot MG400 into API Mode — Windows PC with a wired Ethernet port

This is the Windows version of [dobot-api-mode.md](dobot-api-mode.md). Same
goal: switch the MG400 controller into **TCP/IP secondary development** mode
("API mode") so it serves the ports our driver uses:

| Port | Channel | Used for |
|------|---------|----------|
| 29999 | Dashboard | enable, disable, clear error, stop, speed, digital outputs |
| 30003 | Motion | `MovL` / `ServoP` / `JointMovJ` etc. |
| 30004 | Feedback | 1440-byte real-time status @ ~8 ms |

It is a one-time setting per robot; the controller keeps it as long as it stays
powered. The lab robots normally arrive with it already on. Check first:

```powershell
ping 192.168.1.6
mg400 status
```

Ping works but `mg400 status` is refused → API mode is off. Follow this guide.

On Windows everything runs natively — **Dobot Studio Pro** and our Python client
on the same machine, no VM. The only thing to get right is the network adapter.

---

## 1. Two layouts

**Computer-lab PC, two Ethernet ports.** One port stays on the school network
(internet, DHCP); the other goes straight to the robot with a static IP. This is
the layout we use in the lab.

```
   ┌──────────────────── Windows PC ────────────────────┐
   │                                                    │
   │   Ethernet 1  (DHCP, school network)  ───►  internet
   │                                                    │
   │   Python client (mg400 CLI / serve page)           │
   │   Dobot Studio Pro (once, to enable API mode)      │
   │            │                                       │
   │            ▼                                       │
   │   Ethernet 2  192.168.1.50  ───cable───►  ┌──────────┐
   │                                           │  MG400   │
   │                                           │ 192.168. │
   │                                           │  1.6     │
   │                                           └──────────┘
   └────────────────────────────────────────────────────┘
```

**Laptop, one Ethernet port.** Either unplug the internet cable and use the one
port for the robot, or add a USB-Ethernet dongle as the second port. WiFi can
stay on for internet; it does not interfere.

| Device | IP | Set where |
|--------|----|-----------|
| Robot (MG400 LAN1) | `192.168.1.6` (factory default) | on the robot |
| PC Ethernet port to the robot | `192.168.1.50` / `255.255.255.0`, no gateway | Windows network settings |
| PC Ethernet port to the school network | DHCP, untouched | — |

---

## 2. Wire it up

1. Connect an Ethernet cable from the PC's second port (or the dongle) to the
   robot's **LAN1** port.
2. Power on the MG400 and wait for it to finish booting.
3. Note which Windows adapter that cable is. **Settings → Network & internet →
   Ethernet** lists them; the one that just changed from "Network cable
   unplugged" to "Unidentified network" is the robot port. Rename it to
   `Dobot` there (or in Control Panel → Network Connections) so nobody sets the
   wrong one later.

---

## 3. Set the robot port to a static IP

**Settings → Network & internet → Ethernet →** the `Dobot` adapter → **IP
assignment → Edit**:

- **Manual**, IPv4 on
- **IP address:** `192.168.1.50`
- **Subnet mask:** `255.255.255.0` (Windows 11 may ask for prefix length: `24`)
- **Gateway:** leave blank
- **DNS:** leave blank

Leave the internet adapter exactly as it is (DHCP).

**Why gateway must be blank.** Windows sends everything it does not know where to
put through the adapter that has a gateway. With no gateway on the `Dobot`
port, only `192.168.1.x` goes to the robot and everything else keeps going out
through the school network. If you fill in a gateway on the robot port, the PC
may lose internet, and the robot still works — so it goes unnoticed for a while.

Windows will label the robot port **"No internet"** / **"Unidentified
network"**. That is correct; there is no internet behind it.

Verify from PowerShell or `cmd`:

```powershell
ping 192.168.1.6
```

If the robot replies, the link is good. (If not, see
[Troubleshooting](#6-troubleshooting).)

---

## 4. Enable API mode in Dobot Studio Pro

Install **Dobot Studio Pro** (from dobot.cc, Windows x64) if it isn't already.

1. Launch **Dobot Studio Pro**.
2. **Connect to the robot** at IP `192.168.1.6`. If the robot is not in the list
   and cannot be added, the ping in step 3 is the thing to fix first.
3. Go to **Settings → Remote Control → TCP/IP secondary development** and
   **enable** it.
4. Apply / confirm. The controller now serves ports **29999 / 30003 / 30004**.

> **One controller at a time.** While our Python client is driving the robot,
> don't also jog it from Dobot Studio Pro — only one program should send motion
> commands. Once API mode is enabled, you can leave Studio Pro idle, or close it;
> the robot keeps serving the API ports as long as it stays powered.

---

## 5. Verify

Quick port check from PowerShell:

```powershell
Test-NetConnection 192.168.1.6 -Port 29999
```

`TcpTestSucceeded : True` means the dashboard port is open. Then use this
package (see the [README](../README.md)):

```powershell
mg400 status
mg400 serve
```

Open <http://localhost:8000>, press **Connect → Enable**. Live pose feedback on
the page means all three channels are up.

The first time Python opens a listening socket, **Windows Defender Firewall**
asks whether to allow it. Allow it on **private** networks. If the prompt was
dismissed, the page still works on `localhost` but a phone or another PC cannot
reach it; fix it under Windows Security → Firewall → Allow an app.

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ping 192.168.1.6` gets "Destination host unreachable" or times out | Static IP set on the wrong adapter, cable not in LAN1, or robot still booting | Step 2–3: check which adapter says "Unidentified network"; cable in LAN1; wait for the robot's boot |
| `ping` says "General failure" | Adapter disabled, or IP not applied yet | Disable and re-enable the `Dobot` adapter, then ping again |
| Ping works, Studio Pro doesn't see the robot | Studio Pro is scanning the other adapter | Type the IP `192.168.1.6` in manually |
| Studio Pro connects but API ports refuse | TCP/IP secondary development not enabled | Re-do step 4 and apply |
| Internet stopped working after setup | Gateway was filled in on the `Dobot` port | Step 3: clear the gateway and DNS on the robot port |
| School network is also `192.168.1.x` | Two adapters on the same subnet; Windows picks one at random | Change the robot's IP in Studio Pro (e.g. `192.168.5.6`) and the `Dobot` port to match (`192.168.5.50`), then run the package with `--ip 192.168.5.6` (or set `MG400_IP`) |
| Our client connects but robot won't move | Robot not enabled, or Studio Pro still holds control | Press **Enable** in our UI; stop commanding from Studio Pro |
| Connects then drops | Two controllers fighting, or robot in error | Use one controller; **Clear Error** then **Enable** |
| Phone / other PC can't open the `serve` page | Firewall blocked Python | Windows Security → Firewall → Allow an app → Python, private networks |

---

## 7. Reverting to normal mode

To hand the robot back to Dobot Studio Pro for manual use, disable
**Settings → Remote Control → TCP/IP secondary development** (or simply stop the
Python client and jog from Studio Pro — but don't run both at once).

To give the Ethernet port back to the school network, set its IP assignment back
to **Automatic (DHCP)**.
