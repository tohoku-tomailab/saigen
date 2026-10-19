"""Pure decision logic of the tube-balancing runner (no hardware, no I/O).

Both tubes stand in a rack on the balance for the whole run, so the balance
always reads ``rack-tared total = m_A + m_B``. The runner attributes every
change of that reading to the tube it has just dispensed into, which gives the
gross mass of each tube without ever lifting one. The functions here only decide
*what to dispense next*; the runner does the moving, dispensing and weighing.

Units: grams, millilitres. ``g_per_mL`` is the *effective* density: grams that
actually land in the tube per commanded millilitre (liquid density times the
pipette's own bias), measured on this run.
"""
import math
from dataclasses import dataclass
from typing import List, Optional

TUBES = ("A", "B")

#: Pipette volumes are commanded with this resolution (mL).
VOLUME_STEP_ML = 0.01


def round_volume(v: float) -> float:
    """Round a volume to the pipette's command resolution (half up)."""
    return math.floor(v / VOLUME_STEP_ML + 0.5) * VOLUME_STEP_ML


def split_volume(total_ml: float, max_ml: float, min_ml: float) -> List[float]:
    """Split ``total_ml`` into the fewest equal portions of at most ``max_ml``.

    Every portion is at least ``min_ml`` (the pipette's smallest volume). The
    portions are rounded to ``VOLUME_STEP_ML``; the last one takes the rounding
    remainder so that the portions add up to the rounded total.

    Raises:
        ValueError: ``total_ml`` is below ``min_ml`` or the limits are inconsistent.
    """
    if not (math.isfinite(total_ml) and math.isfinite(max_ml) and math.isfinite(min_ml)):
        raise ValueError("volumes must be finite")
    if min_ml <= 0 or max_ml < min_ml:
        raise ValueError(f"inconsistent pipette limits: min {min_ml} mL, max {max_ml} mL")
    total = round_volume(total_ml)
    if total < min_ml - 1e-9:
        raise ValueError(f"{total_ml:.2f} mL is below the pipette minimum of {min_ml} mL")
    n = max(1, math.ceil(total / max_ml - 1e-9))
    part = round_volume(total / n)
    parts = [part] * (n - 1)
    parts.append(round_volume(total - part * (n - 1)))
    return parts


def bulk_volume_for_b(need_g: float, g_per_mL: float, reserve_ml: float, min_ml: float) -> float:
    """Volume to put into B in one go before the weighed top-ups.

    Aims ``reserve_ml`` short of the target so that the final top-up is a single
    dispense of at least the pipette minimum instead of an overshoot that cannot
    be taken back. Returns 0.0 when the bulk would be below ``min_ml`` (the
    top-up loop then does all of it).
    """
    if g_per_mL <= 0:
        raise ValueError(f"g_per_mL must be > 0, got {g_per_mL}")
    v = round_volume(need_g / g_per_mL - reserve_ml)
    return v if v >= min_ml else 0.0


@dataclass(frozen=True)
class Decision:
    """What the runner does next.

    kind: "done" (balanced), "dispense" (``tube`` gets ``volume_ml``) or
    "fail" (cannot reach the tolerance; ``reason`` says why).
    """
    kind: str
    tube: Optional[str] = None
    volume_ml: float = 0.0
    reason: str = ""


def next_action(m_a: float, m_b: float, *, g_per_mL: float, tolerance_g: float,
                min_ml: float, max_ml: float, allow_extra_in_a: bool) -> Decision:
    """Decide the next dispense from the current gross masses of A and B.

    - ``|m_A - m_B| <= tolerance_g``: done.
    - The lighter tube gets the volume that closes the gap, if that volume is
      at least the pipette minimum (capped at ``max_ml``; the loop continues).
    - Liquid is never added to A unless ``allow_extra_in_a``: if B is the
      heavier tube, or the gap is smaller than the pipette minimum, the run fails
      with a reason the operator can act on.
    - With ``allow_extra_in_a`` and a gap below the pipette minimum, the
      *heavier* tube gets the minimum volume first. That widens the gap to at
      least the minimum, so the next decision is a normal dispense into the
      lighter tube.
    """
    if g_per_mL <= 0 or not math.isfinite(g_per_mL):
        raise ValueError(f"g_per_mL must be finite and > 0, got {g_per_mL}")
    diff = m_a - m_b
    if abs(diff) <= tolerance_g + 1e-9:          # balance readings are 0.01 g steps
        return Decision("done")
    lighter = "B" if diff > 0 else "A"
    heavier = "A" if lighter == "B" else "B"
    gap_ml = round_volume(abs(diff) / g_per_mL)

    if lighter == "A" and not allow_extra_in_a:
        return Decision("fail", reason=(
            f"B is heavier than A by {-diff:.2f} g; liquid cannot be taken out of B "
            "(allow_extra_in_a=1 would let the runner add liquid to A instead)"))
    if gap_ml >= min_ml:
        return Decision("dispense", tube=lighter, volume_ml=min(gap_ml, max_ml))
    if not allow_extra_in_a:
        return Decision("fail", reason=(
            f"B is lighter than A by {diff:.2f} g, which is less than the pipette "
            f"minimum of {min_ml} mL (~{min_ml * g_per_mL:.2f} g); top up B by hand "
            "or set allow_extra_in_a=1"))
    return Decision("dispense", tube=heavier, volume_ml=min_ml)


def effective_density(dispensed_g: float, commanded_ml: float) -> float:
    """Grams per commanded millilitre from a run's own dispenses."""
    if commanded_ml <= 0:
        raise ValueError("nothing has been dispensed yet")
    return dispensed_g / commanded_ml


def check_density(measured: float, nominal: float, max_deviation_pct: float) -> Optional[str]:
    """None if ``measured`` is within ``max_deviation_pct`` % of ``nominal``, else a reason."""
    dev = (measured - nominal) / nominal * 100.0
    if abs(dev) <= max_deviation_pct:
        return None
    return (f"the balance saw {measured:.3f} g per commanded mL, {dev:+.1f} % off the "
            f"expected {nominal:.3f} g/mL (limit +/-{max_deviation_pct} %): liquid may be "
            "missing the tube, dripping, or the pipette may not be dispensing")
