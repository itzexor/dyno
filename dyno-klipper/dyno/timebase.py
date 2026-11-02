"""Device-tick to master-clock mapping from latched status queries."""
import numpy as np


class ClockMap:
    """master_clock = offset + rate * (device_tick - tick0)."""

    def __init__(self, tick0, offset, rate, rms, used, total):
        self.tick0, self.offset, self.rate = tick0, offset, rate
        self.rms, self.used, self.total = rms, used, total

    def __call__(self, ticks):
        return self.offset + self.rate * (np.asarray(ticks, dtype=np.float64) - self.tick0)

    def as_dict(self):
        return dict(tick0=self.tick0, offset=self.offset, rate=self.rate,
                    residual_rms_ticks=self.rms, used=self.used, total=self.total)


def fit_clock(latches, nominal_rate, max_iterations=6):
    """Robust linear fit of master clock against device tick.

    latches: rows (device_tick_us, master_clock, query_ticks). Slow queries
    (more than twice the median duration) are dropped first: an interrupted
    transaction widens the bracket. Then residual outliers beyond 4 MAD are
    removed iteratively. Returns None if fewer than 8 points survive.
    """
    rows = np.asarray(latches, dtype=np.float64)
    if len(rows) < 8:
        return None
    total = len(rows)
    rows = rows[rows[:, 2] <= 2.0 * np.median(rows[:, 2])]
    tick0 = rows[0, 0]
    x = rows[:, 0] - tick0
    y = rows[:, 1]
    y0 = y[0]
    keep = np.ones(len(rows), dtype=bool)
    for _ in range(max_iterations):
        if keep.sum() < 8:
            return None
        A = np.column_stack((np.ones(keep.sum()), x[keep]))
        coef, *_ = np.linalg.lstsq(A, y[keep] - y0, rcond=None)
        resid = y - y0 - (coef[0] + coef[1] * x)
        mad = 1.4826 * np.median(np.abs(resid[keep] - np.median(resid[keep])))
        limit = max(4.0 * mad, 2.0 * nominal_rate)  # never tighter than ~2 us
        new_keep = np.abs(resid) <= limit
        if (new_keep == keep).all():
            break
        keep = new_keep
    if abs(coef[1] / nominal_rate - 1.0) > 1e-3:
        return None  # more than 1000 ppm apart: not a plausible crystal pair
    rms = float(np.sqrt(np.mean(resid[keep] ** 2)))
    return ClockMap(tick0, y0 + coef[0], coef[1], rms, int(keep.sum()), total)
