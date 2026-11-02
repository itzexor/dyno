"""Plant simulator for tests: wobble legs -> trapq-like segments -> force."""
import numpy as np

from dyno import model


def leg_segments(legs, t_start, e_start, wobble, accel):
    """Trapezoidal XY+E legs that reverse direction (junction speed 0).

    legs: [(duration, e_mm)]. Returns segment rows and the end time."""
    rows, t, e = [], t_start, e_start
    for duration, amount in legs:
        v = wobble / duration
        ratio = amount / wobble
        ta = v / accel
        cruise = duration - ta
        ev, ea = v * ratio, accel * ratio
        rows.append((t, ta, 0.0, ea, e, 1.0))
        e += 0.5 * ea * ta * ta
        rows.append((t + ta, cruise, ev, 0.0, e, 1.0))
        e += ev * cruise
        rows.append((t + ta + cruise, ta, ev, -ea, e, 1.0))
        e += 0.5 * ea * ta * ta
        t += duration + ta
    return rows, t, e


def block_legs(flow_low, flow_high, t_low, t_high, cycles, lead, area=2.405):
    legs = [(lead, flow_low * lead / area)]
    for _ in range(cycles):
        legs += [(t_high, flow_high * t_high / area), (t_low, flow_low * t_low / area)]
    return legs


def simulate_force(seg, pa, smooth_time, tau, odr, t0, t1, delay=1.2e-3, gain=4000.0,
                   offset=1e5, drift=300.0, noise=40.0, rng=None, fine=20):
    """Force samples at DRDY times for a block, from the true plant."""
    rng = rng or np.random.default_rng(1)
    dt = 1.0 / (odr * fine)
    grid = t0 - 0.6 + dt * np.arange(int((t1 - t0 + 0.7) / dt))
    p = model.stepper_position(seg, grid, pa, smooth_time)
    s = p - model.low_pass(p, dt, tau)
    sensed = model.sinc3(s, fine)
    period = 1.0 / odr
    times = np.arange(t0 + 0.37 * period, t1, period)
    force = offset + drift * (times - t0) + gain * np.interp(times - delay, grid, sensed)
    return times, force + rng.normal(0, noise, len(times))
