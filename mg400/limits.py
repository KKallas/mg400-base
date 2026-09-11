"""
Where the MG400 may be sent: the approximate reachable workspace, the Z floor,
and the clamp every server target goes through. No Flask here, so the CLI and
your own code can use it too.

The reachable area is a ring (annulus), not a box, so X/Y are clamped to a
min/max radius from the base axis as well as to the X/Y box. These are
conservative starting values; the controller enforces the true workspace and
rejects anything unreachable (a ServoP error). Verify and tighten on your
hardware.
"""

import math

WORKSPACE = {
    "x": [-450.0, 450.0],
    "y": [-450.0, 450.0],
    "z": [-150.0, 230.0],
    "r": [-160.0, 160.0],
}
RADIUS_MIN = 150.0   # mm — inside this the arm can't reach (too folded)
RADIUS_MAX = 440.0   # mm — max horizontal reach

# Lowest Z (mm, robot base frame) a target may have, so the tool stops above the
# table instead of pressing into it. -72 is the Manual Override relay's default.
# Measure your own: jog down slowly until the tool nearly touches the table,
# read Z, add a few mm. Override with --z-floor or MG400_Z_FLOOR.
DEFAULT_Z_FLOOR = -72.0


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp_pose(x, y, z, r, z_floor=None):
    """Clamp a target pose into the (approximate) reachable workspace: Z to its
    range and the floor, R to its range, and X/Y to the reachable annulus, then
    to the X/Y box."""
    z_lo, z_hi = WORKSPACE["z"]
    if z_floor is not None:
        z_lo = min(max(z_lo, z_floor), z_hi)
    z = _clamp(z, z_lo, z_hi)
    r = _clamp(r, *WORKSPACE["r"])
    radius = math.hypot(x, y)
    if radius == 0.0:
        x, y = RADIUS_MIN, 0.0  # base axis is unreachable; nudge outward
    elif radius > RADIUS_MAX:
        s = RADIUS_MAX / radius
        x, y = x * s, y * s
    elif radius < RADIUS_MIN:
        s = RADIUS_MIN / radius
        x, y = x * s, y * s
    x = _clamp(x, *WORKSPACE["x"])
    y = _clamp(y, *WORKSPACE["y"])
    return x, y, z, r
