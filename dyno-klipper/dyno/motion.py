"""Excitation planning: parked, quasi-static XY+E legs.

The installed kinematics apply pressure advance only to extruding moves with
X or Y travel, so each leg pairs its extrusion with a tiny wobble of one axis.
Consecutive legs reverse the wobble, so the junction speed between them is
zero and every leg is its own short trapezoid; the toolhead barely moves and
adds no inertial load to the force signal. A block is a lead-in slow leg
followed by `cycles` fast/slow pairs at one PA value.
"""
import math


class Profile:
    def __init__(self, flow_low=2.0, flow_high=12.0, time_low=0.40, time_high=0.30, cycles=5,
                 lead=0.8, wobble=0.05, accel=1000.0, gap=0.4):
        self.flow_low, self.flow_high = flow_low, flow_high       # mm^3/s
        self.time_low, self.time_high = time_low, time_high       # s per leg
        self.cycles, self.lead, self.gap = cycles, lead, gap      # gap: idle before each block
        self.wobble, self.accel = wobble, accel                   # mm, mm/s^2

    def as_dict(self):
        return dict(vars(self))


class PlanError(Exception):
    pass


def block_legs(profile, filament_area):
    """[(duration_s, e_mm)] for one block."""
    def leg(flow, duration):
        return (duration, flow * duration / filament_area)

    legs = [leg(profile.flow_low, profile.lead)]
    for _ in range(profile.cycles):
        legs += [leg(profile.flow_high, profile.time_high), leg(profile.flow_low, profile.time_low)]
    return legs


def block_duration(profile):
    return profile.lead + profile.cycles * (profile.time_low + profile.time_high) + profile.gap


def check(profile, filament_area, nozzle_diameter, max_extrude_ratio, max_velocity, max_accel):
    """Validate a profile against the printer's limits before any motion."""
    if not 0 < profile.flow_low < profile.flow_high:
        raise PlanError("need 0 < FLOW_LOW < FLOW_HIGH")
    if min(profile.time_low, profile.time_high, profile.lead) < 0.05:
        raise PlanError("legs shorter than 50 ms cannot settle")
    if profile.cycles < 2:
        raise PlanError("need at least two cycles per block")
    if not 0.005 <= profile.wobble <= 0.5:
        raise PlanError("WOBBLE must be within 0.005..0.5 mm")
    if profile.accel > max_accel:
        raise PlanError("ACCEL exceeds the printer's max_accel")
    tiny = nozzle_diameter * max_extrude_ratio
    for duration, e in block_legs(profile, filament_area):
        # Same rule as the extruder's check: a ratio above the limit is only
        # permitted for an extrusion shorter than nozzle_diameter * ratio.
        if e / profile.wobble > max_extrude_ratio and e > tiny:
            raise PlanError("a %.2f mm leg exceeds max_extrude_cross_section with a %.3f mm "
                            "wobble; raise WOBBLE or shorten the fast leg" % (e, profile.wobble))
        if profile.wobble / duration > max_velocity:
            raise PlanError("wobble speed exceeds max_velocity")
        ramp = (profile.wobble / duration) / profile.accel
        if 2 * ramp > 0.1 * duration:
            raise PlanError("ACCEL too low: wobble ramps take over 10% of a leg")


def filament_per_block(profile, filament_area):
    return sum(e for _, e in block_legs(profile, filament_area))


def candidates(tau, fractions=(0.0, 0.6, 0.8, 1.0, 1.2, 1.4), k_max=0.5):
    """PA values bracketing a model estimate for the verification sweep."""
    out = []
    for f in fractions:
        k = round(min(k_max, max(0.0, f * tau)), 4)
        if k not in out:
            out.append(k)
    return out


def schedule(ks, repeats, rng):
    """Randomized, interleaved order with every K repeated."""
    order = []
    for _ in range(repeats):
        batch = list(ks)
        rng.shuffle(batch)
        if order and batch[0] == order[-1] and len(batch) > 1:
            batch[0], batch[-1] = batch[-1], batch[0]
        order += batch
    return order


def smooth_time_ok(smooth_time, max_smooth):
    return 0.0 < smooth_time <= max_smooth and math.isfinite(smooth_time)
