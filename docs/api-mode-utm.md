# Switching the MG400 into API mode (once per robot)

The MG400 controller serves its TCP ports (29999 dashboard, 30003 motion, 30004
feedback) only while **TCP/IP secondary development** — "API mode" — is on. It
is a one-time setting per robot; the controller keeps it as long as it stays
powered. Nothing else in this package needs the steps below, and the lab robots
normally arrive with it already on. Check first:

```sh
ping 192.168.1.6
mg400 status
```

Ping works but `mg400 status` is refused → API mode is off. It is switched with
**Dobot Studio Pro**, a Windows program. On a Windows laptop, install it and go
to step 4. On a Mac it runs in a **Windows 11 ARM** virtual machine under
[UTM](https://mac.getutm.app). (Procedure from the `dualdobottest` setup, single
robot.)

## Steps

1. **Cable and power.** Ethernet (or USB-C → Ethernet adapter) straight to the
   robot's **LAN1** port. Power the robot on and let it boot.
2. **Static IP on the Mac side.** System Settings → Network → the Ethernet
   adapter: Configure IPv4 *Manually*, IP `192.168.1.50`, subnet
   `255.255.255.0`, router blank. Verify with `ping 192.168.1.6` from a Mac
   terminal.
3. **UTM network = Shared Network** (the NAT mode). Don't bridge it and don't
   pass the adapter through to the VM — the guest sits behind macOS and can
   reach anything the host can, including the robot.
4. In the VM, launch **Dobot Studio Pro** and connect to `192.168.1.6`. If it
   can't see the robot, `ping 192.168.1.6` from inside Windows; if the host
   pings but the VM can't, it is the network mode in step 3.
5. **Settings → Remote Control → TCP/IP secondary development → enable**, then
   apply. The controller now serves ports 29999 / 30003 / 30004.

Close Studio Pro afterwards. **One controller at a time:** while the Python
client is driving the robot, nothing is jogged from Studio Pro, and the other
way round.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Mac can't `ping 192.168.1.6` | adapter IP/subnet wrong, cable, or wrong robot port | redo step 2; cable in LAN1; robot IP is still `.6` |
| Host pings the robot, VM can't | UTM not on Shared Network, or adapter passed through | step 3 |
| Studio Pro connects but the API ports refuse | TCP/IP secondary development not enabled | step 5, then apply |
| Client connects but the robot won't move | robot not enabled, or Studio Pro still holds control | **Enable** in the page; stop commanding from Studio Pro |
| Connects, then drops | two controllers fighting, or robot in error | one controller; **Clear Error** then **Enable** |
| macOS asks about incoming connections | firewall prompt for `python` | allow it |

## Reverting

To hand the robot back to Studio Pro for manual use, disable the same setting —
or just stop the Python client and jog from Studio Pro, never both at once.
