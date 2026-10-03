"""
MG400 joint-space model: inverse kinematics plus the joint and parallelogram
limits, so a target can be checked *before* it is streamed. Without this the
controller is the only judge, and it answers an out-of-range ServoP with an
alarm (34 out of working area, 73/74 parallelogram limit, J3 at its stop) that
leaves the arm faulted until it is jogged back out by hand.

Geometry: J1 turns the arm about the base axis; J2 (rear arm) and J3 (fore arm)
form a parallelogram that keeps the flange level, so in the vertical plane

    radius = R0 + L1*sin(J2) + L2*cos(J3)
    z      = Z0 + L1*cos(J2) - L2*sin(J3)

with J2 measured from vertical and J3 from horizontal. The parallelogram adds a
coupled limit on (J3 - J2), the angle between the two arms. J4 is the tool
rotation and does not affect position.
"""

import math

# Link lengths and offsets (mm). Fitted to poses read back from a lab MG400
# (joints from GetAngle vs pose from GetPose, 6 poses, rms 0.01 mm; nominal
# MG400 links are 175 mm each).
L1 = 174.2
L2 = 175.07
R0 = 109.49
Z0 = -52.97

# Joint limits (deg). The controller also enforces these; we stop short of
# them by MARGIN so a streamed setpoint never reaches the alarm.
J1_LIMITS = (-160.0, 160.0)
J2_LIMITS = (-25.0, 85.0)
J3_LIMITS = (-25.0, 105.0)
# J3 - J2: the parallelogram. Seen on the lab arm: alarm 73 (negative limit) at
# J3-J2 = 60.0 .. 60.3 deg, fine at 59.0. The other side is a guess until seen.
PARALLELOGRAM_LIMITS = (-60.0, 60.0)
MARGIN = 2.0


def forward(j1, j2, j3):
    """Joint angles (deg) -> tool point (x, y, z) in mm."""
    a1, a2, a3 = (math.radians(v) for v in (j1, j2, j3))
    radius = R0 + L1 * math.sin(a2) + L2 * math.cos(a3)
    z = Z0 + L1 * math.cos(a2) - L2 * math.sin(a3)
    return radius * math.cos(a1), radius * math.sin(a1), z


def inverse(x, y, z):
    """Tool point (mm) -> (j1, j2, j3) in degrees, or None if out of reach.
    Picks the elbow-up branch the MG400 always uses."""
    j1 = math.degrees(math.atan2(y, x))
    u = math.hypot(x, y) - R0          # horizontal from the J2 axis
    w = z - Z0                         # vertical from the J2 axis
    d2 = u * u + w * w
    d = math.sqrt(d2)
    if d < 1e-6 or d > L1 + L2 or d < abs(L1 - L2):
        return None
    # rear arm points at angle a2 from vertical, fore arm at a3 below horizontal:
    #   u = L1 sin a2 + L2 cos a3,  w = L1 cos a2 - L2 sin a3
    # angle at the elbow between the two links:
    cos_elbow = (L1 * L1 + L2 * L2 - d2) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.acos(cos_elbow)
    # direction from the J2 axis to the tool, measured from vertical
    phi = math.atan2(u, w)
    # rear arm is rotated toward the tool by the base angle of the triangle
    cos_beta = (L1 * L1 + d2 - L2 * L2) / (2 * L1 * d)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    a2 = phi - beta
    # fore arm: from the elbow point to the tool
    ex, ez = L1 * math.sin(a2), L1 * math.cos(a2)
    a3 = math.atan2(-(w - ez), u - ex)  # below horizontal is positive
    _ = elbow
    return j1, math.degrees(a2), math.degrees(a3)


def check(x, y, z, margin=MARGIN):
    """(ok, reason, joints). reason names the limit that would be hit."""
    j = inverse(x, y, z)
    if j is None:
        return False, "out of reach (too far or too close for the arms)", None
    j1, j2, j3 = j
    para = j3 - j2
    for name, v, (lo, hi) in (
        ("J1", j1, J1_LIMITS),
        ("J2", j2, J2_LIMITS),
        ("J3", j3, J3_LIMITS),
        ("parallelogram J3-J2", para, PARALLELOGRAM_LIMITS),
    ):
        if v < lo + margin:
            return False, f"{name} {v:.1f}° at its low limit ({lo:g}°)", j
        if v > hi - margin:
            return False, f"{name} {v:.1f}° at its high limit ({hi:g}°)", j
    return True, None, j


def furthest_reachable(start, end, margin=MARGIN, steps=40):
    """Walk the straight line start -> end (x, y, z) and return the last point
    that passes check(). start itself is returned if nothing further is OK."""
    last = list(start)
    for i in range(1, steps + 1):
        t = i / steps
        p = [s + (e - s) * t for s, e in zip(start, end)]
        if not check(*p, margin=margin)[0]:
            break
        last = p
    return last


def severity(x, y, z, margin=MARGIN):
    """(depth, reason): how far (deg) the point is inside the safety margin of
    its worst limit; 0 when clear of every margin, inf when unreachable. The
    follower lets a step through only if it does not make this worse, so the
    arm can always back away from a limit but never walk into one."""
    j = inverse(x, y, z)
    if j is None:
        return math.inf, "out of reach (too far or too close for the arms)"
    j1, j2, j3 = j
    worst, reason = 0.0, None
    for name, v, (lo, hi) in (
        ("J1", j1, J1_LIMITS),
        ("J2", j2, J2_LIMITS),
        ("J3", j3, J3_LIMITS),
        ("parallelogram J3-J2", j3 - j2, PARALLELOGRAM_LIMITS),
    ):
        for depth, side, lim in (((lo + margin) - v, "low", lo), (v - (hi - margin), "high", hi)):
            if depth > worst:
                worst, reason = depth, f"{name} {v:.1f}° near its {side} limit ({lim:g}°)"
    return worst, reason


def jog_out(joints):
    """Which MoveJog axis backs the arm out of the limit it is in, or None.
    joints: (j1, j2, j3) in degrees, as read from GetAngle()."""
    j1, j2, j3 = joints[:3]
    para = j3 - j2
    if para > PARALLELOGRAM_LIMITS[1] - MARGIN:
        return "J2+"            # alarm 73: arms folded too far, lean J2 forward
    if para < PARALLELOGRAM_LIMITS[0] + MARGIN:
        return "J2-"
    if j3 > J3_LIMITS[1] - MARGIN:
        return "J3-"
    if j3 < J3_LIMITS[0] + MARGIN:
        return "J3+"
    if j2 > J2_LIMITS[1] - MARGIN:
        return "J2-"
    if j2 < J2_LIMITS[0] + MARGIN:
        return "J2+"
    if j1 > J1_LIMITS[1] - MARGIN:
        return "J1-"
    if j1 < J1_LIMITS[0] + MARGIN:
        return "J1+"
    return None
