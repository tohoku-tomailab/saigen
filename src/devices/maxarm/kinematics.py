"""MaxArm kinematics on the PC side: predict whether the arm's own firmware will
accept a target *before* anything is sent.

The Hiwonder MaxArm solves its inverse kinematics on the ESP32 in a compiled
module (``__espmax.mpy``) and its ``set_position`` silently returns ``False``
when that solver raises. The SAIGEN firmware answers ``ERR UNREACHABLE`` in that
case and nothing moves, but a flow that reaches such a target still ends as a
failed run. This module reproduces the acceptance rule so that ``--validate-only``,
``--mock`` and the safety wrapper can reject the target up front.

The model is the standard 3-axis "elbow" arm: a base rotation and two links in
the vertical plane solved with the law of cosines. Link lengths were measured
against the arm's own solver on the bench (2026-09-24): with these constants the
joint angles and servo pulses match ``__espmax.inverse`` / ``deg_to_pulse`` to
four decimals at every point tried (see tests/test_maxarm_kinematics.py). Note
that they differ from the values written in Hiwonder's ``espmax.py`` text
(84.0 / 8.2 / 128.0), which the compiled module does not use.

Acceptance rule of the arm (``ESPMax.set_position``):

- ``hypot(x, y) < 50`` -> refused (returns None);
- ``z > 225`` -> clamped by the vendor, refused by the SAIGEN firmware;
- the solver is called with ``(-x, y, z)`` (the vendor mirrors x);
- planar distance beyond ``L2 + L3`` -> ValueError "Unreachable";
- any joint angle outside 0..240 degrees -> ValueError "Invalid angle" (this is
  what bites near the top of the workspace: the wrist angle goes through 0 and
  the pose the arm rests in after the factory ``go_home`` is already outside).

Everything here is pure Python with no hardware or safety-layer imports.
"""

import math
from typing import Optional, Tuple

# Link lengths (mm) of the compiled vendor solver, verified on the bench.
L0 = 84.4      # base height to the shoulder axis
L1 = 8.14      # shoulder axis to the upper-arm plane (horizontal offset)
L2 = 128.4     # upper arm
L3 = 138.0     # forearm
L4 = 16.8      # tool offset (suction nozzle) used by the vendor library

REACH = L2 + L3
Z_MAX = 225.0            # SAIGEN firmware refuses above this (vendor clamps)
R_MIN = 50.0             # vendor refuses hypot(x, y) below this
JOINT_MIN_DEG = 0.0      # vendor deg_to_pulse refuses outside 0..240
JOINT_MAX_DEG = 240.0
PULSE_RANGE = 1000.0

Joints = Tuple[float, float, float]
Pulses = Tuple[float, float, float]


class Unreachable(ValueError):
    """The vendor solver would refuse this target (``reason`` says why)."""

    def __init__(self, reason: str, axis: str = "reach", value: float = 0.0,
                 low: float = 0.0, high: float = math.inf):
        super().__init__(reason)
        self.reason = reason
        self.axis = axis         # "R", "Z", "reach" or "joint1".."joint3"
        self.value = value
        self.low = low
        self.high = high


def joints_to_pulses(joints: Joints) -> Pulses:
    """Joint angles (deg) -> servo pulses (0..1000); Unreachable outside 0..240 deg.

    Servo 1: 0 deg = pulse 0, 240 deg = pulse 1000.
    Servo 2: mounted mirrored: 210 deg = pulse 0, -30 deg = pulse 1000.
    Servo 3: -120 deg = pulse 0, 120 deg = pulse 1000.
    """
    for i, a in enumerate(joints):
        if not (JOINT_MIN_DEG <= a <= JOINT_MAX_DEG):
            raise Unreachable("joint %d angle %.1f deg outside %.0f..%.0f"
                              % (i + 1, a, JOINT_MIN_DEG, JOINT_MAX_DEG),
                              axis="joint%d" % (i + 1), value=a,
                              low=JOINT_MIN_DEG, high=JOINT_MAX_DEG)
    a1, a2, a3 = joints
    return (a1 * PULSE_RANGE / 240.0,
            (210.0 - a2) * PULSE_RANGE / 240.0,
            (a3 + 120.0) * PULSE_RANGE / 240.0)


def pulses_to_joints(pulses: Pulses) -> Joints:
    """Servo pulses (0..1000) -> joint angles (deg), inverse of joints_to_pulses."""
    for p in pulses:
        if not (0.0 <= p <= PULSE_RANGE):
            raise ValueError("pulse %r outside 0..1000" % (p,))
    p1, p2, p3 = pulses
    return (p1 * 240.0 / PULSE_RANGE,
            210.0 - p2 * 240.0 / PULSE_RANGE,
            p3 * 240.0 / PULSE_RANGE - 120.0)


def _solver_frame_inverse(xs: float, ys: float, z: float, l4: float = L4) -> Joints:
    """Joint angles for a target given in the solver's own frame (x already mirrored)."""
    theta1 = math.atan2(ys, xs) % (2.0 * math.pi)           # 0..2pi
    r = math.hypot(xs, ys) - L1 - l4                          # planar distance to the wrist
    h = z - L0
    d = math.hypot(r, h)
    if d > REACH:
        raise Unreachable("target %.1f mm from the shoulder, reach is %.1f mm" % (d, REACH),
                          axis="reach", value=d, low=0.0, high=REACH)
    if r <= 0.0:
        raise Unreachable("target behind the shoulder axis", axis="reach", value=d,
                          low=0.0, high=REACH)
    psi = math.atan(h / r)
    cos_elbow = (L2 * L2 + L3 * L3 - d * d) / (2.0 * L2 * L3)
    cos_shoulder = (L2 * L2 + d * d - L3 * L3) / (2.0 * L2 * d)
    if not (-1.0 <= cos_elbow <= 1.0 and -1.0 <= cos_shoulder <= 1.0):
        raise Unreachable("no elbow solution", axis="reach", value=d, low=0.0, high=REACH)
    elbow = math.acos(cos_elbow)
    shoulder = math.acos(cos_shoulder)
    joint2 = math.pi - (psi + shoulder)
    joint3 = math.pi - (psi + elbow + shoulder)
    a1 = math.degrees(theta1)
    if a1 <= 30.0:
        a1 += 360.0
    a1 -= 150.0                                               # 0 deg = servo 1 at pulse 0
    return (a1, math.degrees(joint2), math.degrees(joint3))


def inverse(x: float, y: float, z: float, l4: float = L4) -> Joints:
    """Joint angles (deg) the arm would command for user coordinates (x, y, z).

    Applies the same gates as the arm: R_MIN, Z_MAX, the mirrored x, reach and
    the 0..240 deg joint range. Raises Unreachable with the reason.
    """
    for name, v in (("x", x), ("y", y), ("z", z)):
        if not math.isfinite(v):
            raise Unreachable("%s is not finite" % name, axis=name.upper(), value=v)
    r_xy = math.hypot(x, y)
    if r_xy < R_MIN:
        raise Unreachable("hypot(x, y) = %.1f mm is below %.0f mm" % (r_xy, R_MIN),
                          axis="R", value=r_xy, low=R_MIN, high=math.inf)
    if z > Z_MAX:
        raise Unreachable("z = %.1f mm is above %.0f mm" % (z, Z_MAX), axis="Z", value=z,
                          low=-math.inf, high=Z_MAX)
    joints = _solver_frame_inverse(-x, y, z, l4)
    joints_to_pulses(joints)                                  # joint-range gate
    return joints


def forward(pulses: Pulses, l4: float = L4) -> Tuple[float, float, float]:
    """Servo pulses -> user coordinates (x, y, z), as the arm's read_position reports
    (before its rounding to whole mm)."""
    theta1, theta2, theta3 = (math.radians(a) for a in pulses_to_joints(pulses))
    theta1 = (theta1 + math.radians(150.0)) % (2.0 * math.pi)
    phi = theta2 - theta3
    r = math.sqrt(L2 * L2 + L3 * L3 - 2.0 * L2 * L3 * math.cos(phi))
    c = min(1.0, (r * r + L2 * L2 - L3 * L3) / (2.0 * r * L2))
    alpha = math.acos(c)
    gamma = (math.pi - theta2) - alpha
    z = r * math.sin(gamma) + L0
    rho = math.sqrt(max(0.0, r * r - (z - L0) ** 2)) + L1 + l4
    xs = rho * math.cos(theta1)
    ys = rho * math.sin(theta1)
    return (-xs, ys, z)                                       # back to user frame


def reachability(x: float, y: float, z: float) -> Optional[Unreachable]:
    """None if the arm would accept (x, y, z), else the Unreachable describing why."""
    try:
        inverse(x, y, z)
    except Unreachable as e:
        return e
    return None


def is_reachable(x: float, y: float, z: float) -> bool:
    return reachability(x, y, z) is None


def highest_reachable_z(x: float, y: float, z_low: float, z_high: float,
                        step: float = 1.0) -> Optional[float]:
    """Highest z in [z_low, z_high] (scanned downwards in ``step`` mm) the arm
    accepts at (x, y), or None. For error messages and workspace surveys."""
    if not (math.isfinite(z_low) and math.isfinite(z_high)) or step <= 0:
        return None
    z = z_high
    while z >= z_low:
        if is_reachable(x, y, z):
            return z
        z -= step
    return None
