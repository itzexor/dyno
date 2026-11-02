"""Pressure-advance estimation from nozzle force and commanded extrusion.

Melt model: the extruder stepper position P(t) pushes filament into a
compliant melt chamber that drains through the nozzle with time constant tau:

    ds/dt = dP/dt - s/tau    =>    s = P - LP_tau(P)

where LP_tau is a first-order low-pass and s is the stored compression.
Force is a + c*t + b*s, observed through the ADC's sinc3 filter and an
unknown latency d. Klipper's pressure advance is exact for this model when
K = tau: then s = tau * S[v_nominal], so force tracks the smoothed nominal
flow. Two estimates follow:

- identify(): tau from all blocks jointly (per-block baselines, shared gain).
- sweep(): model-free; the K whose block best tracks S[v_nominal].
"""
import math

import numpy as np

try:
    from scipy.signal import lfilter, lfilter_zi
except ImportError:  # pragma: no cover - the printer has scipy; keep a fallback
    lfilter = None

TAU_RANGE = (0.002, 0.300)
DELAYS = np.arange(0.0, 5.01e-3, 0.25e-3)


class Segments:
    """Extruder trapq moves: print_time, move_t, start_v, accel, start_e, pa_flag."""

    def __init__(self, rows):
        rows = np.asarray(rows, dtype=np.float64).reshape(-1, 6)
        rows = rows[np.argsort(rows[:, 0])]
        self.t0, self.dur, self.v0, self.acc, self.e0, self.flag = rows.T

    @classmethod
    def from_pull_moves(cls, moves):
        return cls([(m.print_time, m.move_t, m.start_v, m.accel, m.start_x, m.y_r) for m in moves])

    def __len__(self):
        return len(self.t0)

    def start(self):
        return float(self.t0[0])

    def end(self):
        return float(self.t0[-1] + self.dur[-1])

    def nominal(self, t):
        """Nominal E position, velocity and PA flag at times t."""
        t = np.asarray(t, dtype=np.float64)
        i = np.clip(np.searchsorted(self.t0, t, side="right") - 1, 0, len(self.t0) - 1)
        before = t < self.t0[0]
        dt = np.clip(t - self.t0[i], 0.0, self.dur[i])
        inside = (t >= self.t0[i]) & (t <= self.t0[i] + self.dur[i]) & ~before
        pos = self.e0[i] + (self.v0[i] + 0.5 * self.acc[i] * dt) * dt
        pos = np.where(before, self.e0[0], pos)
        vel = np.where(inside, self.v0[i] + self.acc[i] * dt, 0.0)
        flag = np.where(inside, self.flag[i] != 0.0, False)
        return pos, vel, flag


def triangle_smooth(x, dt, smooth_time):
    """Klipper's PA smoothing: a triangular window of total width smooth_time."""
    half = int(round(0.5 * smooth_time / dt))
    if half < 1:
        return x
    w = half - np.abs(np.arange(-half + 1, half))
    w = w / w.sum()
    padded = np.concatenate((np.full(half, x[0]), x, np.full(half, x[-1])))
    return np.convolve(padded, w, mode="same")[half:-half]


def stepper_position(seg, grid, pa, smooth_time):
    """Commanded stepper position under Kalico's PA (no smoothing at PA=0)."""
    pos, vel, flag = seg.nominal(grid)
    if pa <= 0.0 or smooth_time <= 0.0:
        return pos
    return triangle_smooth(pos + pa * vel * flag, grid[1] - grid[0], smooth_time)


def reference_flow(seg, grid, smooth_time):
    """The outflow an exact PA would produce: smoothed nominal velocity."""
    _, vel, _ = seg.nominal(grid)
    return triangle_smooth(vel, grid[1] - grid[0], smooth_time)


def low_pass(x, dt, tau):
    """First-order low-pass, exact for input linear between grid points."""
    alpha = math.exp(-dt / tau)
    if lfilter is not None:
        b0 = 1.0 - tau / dt * (1.0 - alpha)
        b = [b0, 1.0 - alpha - b0]
        y, _ = lfilter(b, [1.0, -alpha], x, zi=lfilter_zi(b, [1.0, -alpha]) * x[0])
        return y
    n = len(x)  # FFT convolution with a truncated exponential kernel
    k = np.exp(-dt / tau * np.arange(min(n, int(12 * tau / dt) + 1))) * (1.0 - alpha)
    size = 1 << int(math.ceil(math.log2(n + len(k))))
    padded = x - x[0]
    y = np.fft.irfft(np.fft.rfft(padded, size) * np.fft.rfft(k, size), size)[:n]
    return y + x[0]


def sinc3(x, steps):
    """ADS131M0x sinc3 decimator: three cascaded boxcars of one conversion
    period (`steps` grid points), causal, unity DC gain. Trapezoidal taps give
    each boxcar the continuous one's mean lag of half a period."""
    box = np.ones(steps + 1) / steps
    box[0] = box[-1] = 0.5 / steps
    kernel = np.convolve(np.convolve(box, box), box)
    padded = np.concatenate((np.full(len(kernel) - 1, x[0]), x))
    return np.convolve(padded, kernel, mode="valid")


class Block:
    """One constant-PA excitation block and the force samples covering it."""

    def __init__(self, label, pa, smooth_time, seg, times, force, odr, oversample=4):
        self.label, self.pa, self.smooth_time, self.seg = label, pa, smooth_time, seg
        self.times = np.asarray(times, dtype=np.float64)
        self.force = np.asarray(force, dtype=np.float64)
        self.steps = oversample
        self.dt = 1.0 / (odr * oversample)
        lo = min(seg.start(), self.times[0]) - 0.6
        hi = max(seg.end(), self.times[-1]) + 0.05
        self.grid = lo + self.dt * np.arange(int((hi - lo) / self.dt) + 2)
        self.position = stepper_position(seg, self.grid, pa, smooth_time)
        self.reference = sinc3(reference_flow(seg, self.grid, smooth_time), oversample)
        # Remove a per-block baseline and linear drift from everything we fit.
        tc = self.times - self.times.mean()
        self.basis = np.column_stack((np.ones_like(tc), tc / max(np.ptp(tc), 1e-9)))
        self.q, _ = np.linalg.qr(self.basis)
        self.force_r = self._detrend(self.force)

    def _detrend(self, y):
        return y - self.q @ (self.q.T @ y)

    def response(self, tau):
        """Sensor-filtered compression on the grid for time constant tau."""
        p = self.position
        return sinc3(p - low_pass(p, self.dt, tau), self.steps)

    def sample(self, y, delay):
        return self._detrend(np.interp(self.times - delay, self.grid, y))


def _profile(blocks, tau):
    """Residual sum of squares at each candidate delay for one tau."""
    responses = [b.response(tau) for b in blocks]
    out = np.empty(len(DELAYS))
    for j, d in enumerate(DELAYS):
        sxy = syy = sff = 0.0
        for b, y in zip(blocks, responses):
            r = b.sample(y, d)
            sxy += r @ b.force_r
            syy += r @ r
            sff += b.force_r @ b.force_r
        out[j] = sff - (sxy * sxy / syy if syy > 0 else 0.0)
    return out


def _fit_at(blocks, tau, delay):
    rs = [b.sample(b.response(tau), delay) for b in blocks]
    sxy = sum(r @ b.force_r for r, b in zip(rs, blocks))
    syy = sum(r @ r for r in rs)
    gain = sxy / syy
    resid = np.concatenate([b.force_r - gain * r for r, b in zip(rs, blocks)])
    signal = np.concatenate(rs) * gain
    return gain, resid, signal


def _golden(f, lo, hi, iterations=40):
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(iterations):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    return (a + b) / 2


class _Objective:
    """RSS(tau, delay) with per-tau response caching."""

    def __init__(self, blocks):
        self.blocks = blocks
        self.cache = {}

    def responses(self, tau):
        key = round(math.log(tau), 9)
        if key not in self.cache:
            if len(self.cache) > 64:
                self.cache.clear()
            self.cache[key] = [b.response(tau) for b in self.blocks]
        return self.cache[key]

    def __call__(self, tau, delay):
        sxy = syy = sff = 0.0
        for b, y in zip(self.blocks, self.responses(tau)):
            r = b.sample(y, delay)
            sxy += r @ b.force_r
            syy += r @ r
            sff += b.force_r @ b.force_r
        return sff - (sxy * sxy / syy if syy > 0 else 0.0)


def _minimize(objective, tau, delay, rounds=4):
    """Alternate 1-D searches over log(tau) and delay from a grid start."""
    for _ in range(rounds):
        lt = math.log(tau)
        tau = math.exp(_golden(lambda x: objective(math.exp(x), delay), lt - 0.2, lt + 0.2, 30))
        delay = _golden(lambda d: objective(tau, d), max(0.0, delay - 0.6e-3),
                        min(DELAYS[-1] + 1e-3, delay + 0.6e-3), 25)
    return tau, delay


def identify(blocks, tau_range=TAU_RANGE):
    """Joint first-order fit. Returns a dict with tau, delay, quality and spread."""
    taus = np.geomspace(tau_range[0], tau_range[1], 48)
    grid = np.array([_profile(blocks, t) for t in taus])
    i, j = np.unravel_index(np.argmin(grid), grid.shape)
    objective = _Objective(blocks)
    tau, delay = _minimize(objective, float(taus[i]), float(DELAYS[j]))
    gain, resid, signal = _fit_at(blocks, tau, delay)
    n = len(resid)
    rss = float(resid @ resid)
    sigma2 = rss / max(n - 2 * len(blocks) - 3, 1)
    rho = float(np.corrcoef(resid[:-1], resid[1:])[0, 1]) if n > 3 else 0.0
    rho = min(max(rho, 0.0), 0.99)
    inflation = (1 + rho) / (1 - rho)  # AR(1) effective-sample correction
    # Curvature of RSS in (log tau, delay) gives the profile standard error.
    h = (0.03, 0.1e-3)
    x0 = (math.log(tau), delay)

    def f(dx, dy):
        return objective(math.exp(x0[0] + dx), x0[1] + dy)

    f00 = rss
    fxx = (f(h[0], 0) - 2 * f00 + f(-h[0], 0)) / h[0] ** 2
    fyy = (f(0, h[1]) - 2 * f00 + f(0, -h[1])) / h[1] ** 2
    fxy = (f(h[0], h[1]) - f(h[0], -h[1]) - f(-h[0], h[1]) + f(-h[0], -h[1])) / (4 * h[0] * h[1])
    det = fxx * fyy - fxy * fxy
    se_log = math.sqrt(2 * sigma2 * inflation * fyy / det) if det > 0 and fyy > 0 else float("inf")
    total = float(sum(b.force_r @ b.force_r for b in blocks))
    result = dict(tau=tau, delay=delay, gain=float(gain), r2=1.0 - rss / total if total else 0.0,
                  snr=float(np.std(signal) / math.sqrt(sigma2)) if sigma2 > 0 else float("inf"),
                  residual_rms=math.sqrt(rss / n), autocorrelation=rho, se=tau * se_log,
                  ci68=(tau * math.exp(-se_log), tau * math.exp(se_log)) if math.isfinite(se_log)
                  else (0.0, float("inf")),
                  boundary=bool(i == 0 or i == len(taus) - 1), samples=n)
    if len(blocks) >= 3:
        jk = []
        for k in range(len(blocks)):
            rest = _Objective(blocks[:k] + blocks[k + 1:])
            jk.append(math.exp(_golden(lambda lt: rest(math.exp(lt), delay),
                                       math.log(tau) - 0.5, math.log(tau) + 0.5, 30)))
        jk = np.array(jk)
        result["jackknife_se"] = float(math.sqrt((len(jk) - 1) / len(jk) * ((jk - jk.mean()) ** 2).sum()))
        result["block_taus"] = [float(x) for x in jk]
    return result


def tracking_score(block, delay, windows=None):
    """Normalized error of force against the ideal-PA reference flow.

    Returns (score, per-window scores). Lower is better; 0 is perfect."""
    ref = block.sample(block.reference, delay)
    gain = (ref @ block.force_r) / (ref @ ref)
    resid = block.force_r - gain * ref
    scale = abs(gain) * np.std(ref)
    score = float(np.sqrt(np.mean(resid ** 2)) / scale)
    parts = []
    for lo, hi in windows or []:
        m = (block.times >= lo) & (block.times < hi)
        if m.sum() > 10:
            parts.append(float(np.sqrt(np.mean(resid[m] ** 2)) / scale))
    return score, parts


def sweep(blocks, delay, rng=None):
    """Interpolated K minimizing the tracking score across PA candidates."""
    rng = rng or np.random.default_rng(0)
    by_k = {}
    for b in blocks:
        score, parts = tracking_score(b, delay, cycle_windows(b))
        by_k.setdefault(round(b.pa, 6), []).append((score, parts))
    ks = np.array(sorted(by_k))
    means = np.array([np.mean([s for s, _ in by_k[k]]) for k in ks])
    ses = []
    for k in ks:
        parts = [p for _, ps in by_k[k] for p in ps]
        ses.append(np.std(parts) / math.sqrt(len(parts)) if len(parts) > 1 else 0.0)
    ses = np.array(ses)
    result = dict(k=[float(x) for x in ks], score=[float(x) for x in means],
                  score_se=[float(x) for x in ses])
    i = int(np.argmin(means))
    if i == 0 or i == len(ks) - 1:
        result.update(k_opt=float(ks[i]), boundary=True)
        return result

    def vertex(y):
        x = ks[i - 1:i + 2]
        a, b, _ = np.polyfit(x, y[i - 1:i + 2], 2)
        return float(np.clip(-b / (2 * a), x[0], x[2])) if a > 0 else float(ks[i])

    draws = [vertex(means + rng.normal(0, 1, len(means)) * ses) for _ in range(200)]
    result.update(k_opt=vertex(means), k_se=float(np.std(draws)), boundary=False)
    return result


def cycle_windows(block):
    """Fast-leg windows (each fast leg plus the following slow leg)."""
    seg = block.seg
    v = seg.v0 + seg.acc * seg.dur  # end velocity per segment
    peak = np.maximum(seg.v0, v)
    fast = peak > 0.5 * peak.max()
    starts = seg.t0[1:][fast[1:] & ~fast[:-1]]
    edges = list(starts) + [seg.end()]
    return list(zip(edges[:-1], edges[1:]))
