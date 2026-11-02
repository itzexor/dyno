"""Measurement sessions: stream capture, the PA experiment, and analysis.

Everything here talks to the printer through a `host` object (see extra.py
for the Klipper binding and tests/fakes.py for the simulated one):

    host.monotonic(), host.pause(until)          reactor time
    host.gcode(script), host.respond(text)       G-code side effects and console
    host.toolhead, host.extruder                 Klipper objects (or fakes)
    host.extract_segments(t0, t1)                extruder trapq rows (model.Segments)
    host.clock_to_print_time(clock)              master MCU clock -> print time
    host.master_clock_rate                       master MCU ticks per microsecond
    host.start_timer(callback, interval)         -> handle; host.stop_timer(handle)
    host.run_in_thread(fn)                       run fn off the reactor, wait cooperatively
    host.archive(name, meta, arrays)             persist a run; returns its path

Device control and status queries run on the reactor; assembly and fitting
run in a thread. Nothing moves or changes PA until a command asks for it.
"""
import math
import time

import numpy as np

from . import model, motion, stream, timebase
from .protocol import MODE_NAMES, DeviceError, osr_code

STATUS_INTERVAL = 0.05
TAIL = 0.5


class Cancelled(Exception):
    pass


class RoutineError(Exception):
    pass


class Limits:
    """Acceptance thresholds for a PA result."""

    def __init__(self, min_snr=4.0, max_rel_se=0.10, max_drift=0.15, max_clock_rms_us=50.0,
                 agreement_sigmas=3.0, agreement_floor=0.10):
        self.min_snr, self.max_rel_se, self.max_drift = min_snr, max_rel_se, max_drift
        self.max_clock_rms_us = max_clock_rms_us
        self.agreement_sigmas, self.agreement_floor = agreement_sigmas, agreement_floor


class BlockRecord:
    def __init__(self, kind, pa, smooth_time, t0, t1, rows):
        self.kind, self.pa, self.smooth_time = kind, pa, smooth_time
        self.t0, self.t1, self.rows = t0, t1, rows


class Capture:
    """One streaming session: device capture plus master-side reader."""

    def __init__(self, host, device, reader, rate, gain_code):
        self.host, self.device, self.reader = host, device, reader
        self.rate, self.gain_code = rate, gain_code
        self.timer = None
        self.timer_error = None
        self.in_status = False
        self.overflow = None
        self.period_us = None
        self.device_info = self.adc = None

    def start(self):
        self.device_info = self.device.connect() if self.device.info is None else self.device.info
        status = self.device.status()
        if MODE_NAMES[status["mode"]] in ("armed", "running"):
            self.device.stop()
            status = self.device.status()
        if status["buffered"]:  # a previous session ended before its tail was read
            self.device.drain_stream()
            if self.device.status()["buffered"]:
                raise RoutineError("device holds unread samples that could not be drained")
        adc = self.device.get_adc()
        want = osr_code(self.rate)
        if (adc["osr_code"], adc["gain_code"]) != (want, self.gain_code):
            self.device.set_adc(want, self.gain_code)
            self._settle()
            adc = self.device.get_adc()
        self.adc = adc
        self.period_us = 1e6 / adc["rate"]
        self.device.reset_errors()
        self.device.arm(stream=True)
        deadline = self.host.monotonic() + 1.0
        while True:  # START needs a healthy conversion after any profile change
            try:
                self.device.start()
                break
            except DeviceError:
                if self.host.monotonic() > deadline:
                    raise RoutineError("device not healthy; inspect DYNO_STATUS")
                self.host.pause(self.host.monotonic() + 0.05)
        self.reader.start(4.0 / adc["rate"])
        self.timer = self.host.start_timer(self._status, STATUS_INTERVAL)

    def _settle(self):
        self.host.pause(self.host.monotonic() + 0.1)

    def _status(self):
        self.in_status = True
        try:
            self.reader.poll_status()
        except Exception as e:  # recorded; the session fails at assembly
            self.timer_error = str(e)
            return False
        finally:
            self.in_status = False
        return True

    def stop(self):
        """Stop the device, let the reader drain the padded tail, then stop it."""
        errors = []
        try:
            self.device.stop()
            # Let the reader drain the padded tail at its own pace. Polling the
            # device here would compete with the last bulk blocks for the
            # MCU's transmit buffer, which drops messages when full.
            self.host.pause(self.host.monotonic() + 0.3)
            status = self.device.status()
            deadline = self.host.monotonic() + 1.5
            while status["buffered"] and self.host.monotonic() < deadline:
                self.host.pause(self.host.monotonic() + 0.1)
                status = self.device.status()
            self.overflow = status["overflow"]
            if status["buffered"]:
                errors.append("%d samples were never drained" % status["buffered"])
        except Exception as e:
            errors.append("device stop failed: %s" % e)
        if self.timer is not None:
            self.host.stop_timer(self.timer)
            self.timer = None
        deadline = self.host.monotonic() + 1.0
        while self.in_status and self.host.monotonic() < deadline:
            self.host.pause(self.host.monotonic() + 0.01)
        try:
            self.reader.stop()
            self.host.pause(self.host.monotonic() + 0.05)  # trailing deliveries
        except Exception as e:
            errors.append("reader stop failed: %s" % e)
        return errors

    def snapshot(self):
        return list(self.reader.messages), list(self.reader.statuses)


def decode(host, messages, statuses, period_us, overflow):
    """Assemble a stream and map every sample to print time."""
    cap = stream.assemble(messages, statuses, period_us, device_overflow=overflow)
    clock_map = timebase.fit_clock(cap.latches, host.master_clock_rate) if len(cap.latches) else None
    times = None
    if clock_map is not None and len(cap.ticks):
        c0 = float(clock_map(cap.ticks[:1])[0])
        p0 = host.clock_to_print_time(c0)
        slope = (host.clock_to_print_time(c0 + 1e6) - p0) / 1e6

        def to_time(ticks):  # device ticks -> print time; infinities pass through
            ticks = np.asarray(ticks, dtype=np.float64)
            finite = np.isfinite(ticks)
            out = ticks.copy()
            out[finite] = p0 + (clock_map(ticks[finite]) - c0) * slope
            return out

        times = to_time(cap.ticks)
        cap.gap_times = [tuple(to_time(g)) for g in cap.gaps]
    return cap, clock_map, times


def data_findings(cap, windows):
    """Split stream findings into rejections and notes: a loss only rejects
    when it overlaps a window the analysis uses, or when timing is unreliable."""
    reasons = ["data: " + f for f in cap.fatal]
    hits = [g for g in getattr(cap, "gap_times", [])
            if any(g[0] < hi and g[1] > lo for lo, hi in windows)]
    if hits:
        reasons.append("data: %d loss interval(s) inside analysed blocks (%s)"
                       % (len(hits), "; ".join(cap.problems)))
    notes = [] if hits or not cap.problems else [
        "outside analysed blocks: " + "; ".join(p for p in cap.problems if p not in cap.fatal)]
    return reasons, notes


def build_blocks(records, cap, times, odr):
    good = (cap.entries["flags"] == 0)
    blocks = []
    for rec in records:
        lo, hi = rec.t0 - 0.3, rec.t1 + 0.3
        m = good & (times >= lo) & (times <= hi)
        if m.sum() < 50:
            raise RoutineError("block at %.3f has only %d samples" % (rec.t0, int(m.sum())))
        seg = model.Segments(rec.rows)
        blocks.append(model.Block(rec.kind, rec.pa, rec.smooth_time, seg, times[m],
                                  cap.entries["raw"][m].astype(np.float64), odr))
    return blocks


def evaluate(identification, sweep_result, drift, clock_map, data, limits):
    """Apply acceptance rules. `data` is (reasons, notes) from data_findings.
    Returns (accepted, recommended K, reasons, notes)."""
    reasons, notes = list(data[0]), list(data[1])
    if clock_map is None:
        reasons.append("clock: too few usable latches")
    else:
        rms_us = clock_map.rms / clock_map.rate
        notes.append("clock rms %.1f us (%d/%d latches)" % (rms_us, clock_map.used, clock_map.total))
        if rms_us > limits.max_clock_rms_us:
            reasons.append("clock: latch residual %.0f us" % rms_us)
    tau = identification["tau"]
    se = max(identification["se"], identification.get("jackknife_se", 0.0))
    if identification["boundary"]:
        reasons.append("model: tau at the search boundary")
    if identification["snr"] < limits.min_snr:
        reasons.append("model: SNR %.1f below %.1f" % (identification["snr"], limits.min_snr))
    if not se < limits.max_rel_se * tau:
        reasons.append("model: tau uncertainty %.1f%%" % (100 * se / tau if tau else float("inf")))
    if drift is not None and abs(drift) > limits.max_drift:
        reasons.append("drift: tau changed %.0f%% between first and last blocks" % (100 * drift))
    k = None
    if sweep_result is None:
        reasons.append("sweep: not run")
    elif sweep_result["boundary"]:
        reasons.append("sweep: best K %.4f is at the edge of the tested range" % sweep_result["k_opt"])
    else:
        k = sweep_result["k_opt"]
        sigma = math.hypot(sweep_result.get("k_se", 0.0), se)
        tolerance = max(limits.agreement_sigmas * sigma, limits.agreement_floor * tau)
        if abs(k - tau) > tolerance:
            reasons.append("agreement: sweep K %.4f vs model tau %.4f (tolerance %.4f)"
                           % (k, tau, tolerance))
    return (not reasons and k is not None), k, reasons, notes


class PaExperiment:
    """DYNO_PA: identification, adaptive sweep, repeat identification."""

    def __init__(self, host, capture, profile, extruder_name, smooth_time, repeats, max_filament,
                 limits, cancel_flag, rng=None):
        self.host, self.capture, self.profile = host, capture, profile
        self.extruder_name, self.smooth_time = extruder_name, smooth_time
        self.repeats, self.max_filament, self.limits = repeats, max_filament, limits
        self.cancelled = cancel_flag
        self.rng = rng or np.random.default_rng()
        self.records = []
        self.filament = 0.0
        self.wobble_up = False
        self.base_y = None

    # --- motion -------------------------------------------------------
    def _set_pa(self, k):
        self.host.gcode("SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f SMOOTH_TIME=%.6f"
                        % (self.extruder_name, k, self.smooth_time))

    def _check_cancel(self):
        if self.cancelled():
            raise Cancelled()

    def prime(self, amount):
        th = self.host.toolhead
        th.wait_moves()
        self._set_pa(0.0)
        pos = th.get_position()
        pos[3] += amount
        area = self.host.extruder.filament_area
        th.move(pos, self.profile.flow_low * 3.0 / area)
        self.filament += amount
        th.wait_moves()
        self._check_cancel()

    def run_block(self, k, kind):
        th = self.host.toolhead
        area = self.host.extruder.filament_area
        legs = motion.block_legs(self.profile, area)
        need = sum(e for _, e in legs)
        if self.filament + need > self.max_filament:
            raise RoutineError("filament budget of %.0f mm reached" % self.max_filament)
        th.wait_moves()
        self._set_pa(k)
        th.dwell(self.profile.gap)
        t0 = th.get_last_move_time()
        for duration, e in legs:
            pos = th.get_position()
            if self.base_y is None:
                self.base_y = pos[1]
            self.wobble_up = not self.wobble_up
            pos[1] = self.base_y + (self.profile.wobble if self.wobble_up else 0.0)
            pos[3] += e
            th.move(pos, self.profile.wobble / duration)
        t1 = th.get_last_move_time()
        th.wait_moves()
        self.filament += need
        rows = self.host.extract_segments(t0 - 0.05, t1 + 0.05)
        self.records.append(BlockRecord(kind, k, self.smooth_time, t0, t1, rows))
        self._check_cancel()

    def return_to_base(self):
        if self.base_y is None or not self.wobble_up:
            return
        th = self.host.toolhead
        pos = th.get_position()
        pos[1] = self.base_y
        th.move(pos, 5.0)
        self.wobble_up = False

    # --- analysis -----------------------------------------------------
    def _fit(self, messages, statuses, records, overflow=None):
        records = [r for r in records if r.kind != "warmup"]
        cap, clock_map, times = decode(self.host, messages, statuses, self.capture.period_us,
                                       overflow)
        if times is None:
            raise RoutineError("no usable clock mapping from %d latches%s"
                               % (len(cap.latches), ": " + "; ".join(cap.problems)
                                  if cap.problems else ""))
        blocks = build_blocks(records, cap, times, self.capture.adc["rate"])
        return cap, clock_map, times, blocks

    def quick_tau(self):
        messages, statuses = self.capture.snapshot()
        records = list(self.records)

        def work():
            _, _, _, blocks = self._fit(messages, statuses, records)
            return model.identify(blocks)

        fit = self.host.run_in_thread(work)
        rel = fit["se"] / fit["tau"] if fit["tau"] else float("inf")
        if fit["snr"] < self.limits.min_snr or fit["boundary"] or not rel < 2 * self.limits.max_rel_se:
            raise RoutineError("identification too weak to plan a sweep (SNR %.1f, tau %.4f "
                               "+/- %.0f%%%s)" % (fit["snr"], fit["tau"], 100 * rel,
                                                  ", at boundary" if fit["boundary"] else ""))
        return fit

    def analyse(self):
        messages, statuses = self.capture.snapshot()
        records = list(self.records)
        overflow = self.capture.overflow

        def work():
            cap, clock_map, times, blocks = self._fit(messages, statuses, records, overflow)
            ident = model.identify(blocks)
            idents = [b for b in blocks if b.label == "ident"]
            drift = None
            if len(idents) >= 2:
                first = model.identify(idents[:1])["tau"]
                last = model.identify(idents[-1:])["tau"]
                drift = (last - first) / ident["tau"]
            sweep_blocks = [b for b in blocks if b.label == "sweep"]
            sw = model.sweep(sweep_blocks, ident["delay"]) if sweep_blocks else None
            fitted = [r for r in records if r.kind != "warmup"]
            data = data_findings(cap, [(r.t0 - 0.3, r.t1 + 0.3) for r in fitted])
            accepted, k, reasons, notes = evaluate(ident, sw, drift, clock_map, data, self.limits)
            arrays = dict(raw=cap.entries["raw"], flags=cap.entries["flags"], ticks=cap.ticks,
                          times=times, latches=np.asarray(cap.latches),
                          statuses=np.asarray(statuses, dtype=np.float64),
                          blocks=np.array([(r.t0, r.t1, r.pa) for r in records]),
                          block_kinds=np.array([r.kind for r in records]))
            for i, r in enumerate(records):
                arrays["segments_%d" % i] = np.asarray(r.rows, dtype=np.float64)
            return dict(accepted=accepted, k=k, reasons=reasons, notes=notes,
                        identification=ident, sweep=sw, drift=drift,
                        clock=clock_map.as_dict() if clock_map else None,
                        lost=cap.lost, samples=int(len(cap.ticks))), arrays

        return self.host.run_in_thread(work)


def execute(exp, prime, respond):
    """The experiment: warm-up, identification, adaptive sweep, repeat
    identification. The warm-up block brings the melt to its pulsed steady
    state after priming; it is archived but never fitted."""
    if prime:
        exp.prime(prime)
    exp.run_block(0.0, "warmup")
    exp.run_block(0.0, "ident")
    fit = exp.quick_tau()
    ks = motion.candidates(fit["tau"])
    order = motion.schedule(ks, exp.repeats, exp.rng)
    respond("Dyno PA: tau ~%.4f; sweeping K %s" % (fit["tau"], " ".join("%.4f" % k for k in ks)))
    for i, k in enumerate(order):
        exp.run_block(k, "sweep")
        if i % 3 == 2:
            respond("Dyno PA: %d/%d sweep blocks" % (i + 1, len(order)))
    exp.run_block(0.0, "ident")
    # Keep recording through the last block's relaxation window (t1 + 0.3 s)
    # so the stream's padded tail lies outside everything that is fitted.
    exp.host.pause(exp.host.monotonic() + TAIL)


def restore(host, exp, snap, retract):
    """Undo everything the experiment changed. Returns what could not be undone."""
    def do_retract():
        th = host.toolhead
        pos = th.get_position()
        pos[3] -= retract
        th.move(pos, 20.0)

    steps = [("toolhead position", exp.return_to_base)]
    if retract:
        steps.append(("retract", do_retract))
    steps += [
        ("pressure advance", lambda: host.gcode(
            "SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f SMOOTH_TIME=%.6f"
            % (snap["extruder"], snap["pa"], snap["smooth_time"]))),
        ("acceleration", lambda: host.gcode("SET_VELOCITY_LIMIT ACCEL=%.1f" % snap["accel"])),
        ("G-code position", host.reset_gcode_position),
    ]
    unrestored = []
    for name, step in steps:
        try:
            step()
        except Exception as e:
            unrestored.append("%s (%s)" % (name, e))
    return unrestored


def format_report(result, filament, path, prior):
    ident, sw = result["identification"], result["sweep"]
    se = max(ident["se"], ident.get("jackknife_se", 0.0))
    lines = []
    if result["accepted"]:
        lines.append("Dyno PA: recommended ADVANCE=%.4f (sweep %.4f +/- %.4f; model tau %.4f +/- %.4f)"
                     % (result["k"], sw["k_opt"], sw.get("k_se", 0.0), ident["tau"], se))
    else:
        lines.append("Dyno PA: no reliable estimate (model tau %.4f +/- %.4f)" % (ident["tau"], se))
        lines += ["  rejected: " + r for r in result["reasons"]]
    lines.append("  model: delay %.2f ms, SNR %.1f, R2 %.3f, %d samples%s"
                 % (ident["delay"] * 1e3, ident["snr"], ident["r2"], ident["samples"],
                    ", drift %+.1f%%" % (100 * result["drift"]) if result["drift"] is not None else ""))
    if sw:
        lines.append("  sweep: " + "  ".join("%.4f:%.3f" % kv for kv in zip(sw["k"], sw["score"])))
    lines += ["  " + n for n in result["notes"]]
    lines.append("  filament %.1f mm; data %s" % (filament, path))
    if result["accepted"]:
        lines.append("  DYNO_PA_APPLY sets it (current %.4f); DYNO_PA_ROLLBACK undoes it" % prior)
    return "\n".join(lines)


def stream_test_report(cap, clock_map, period_us, nominal_clock_rate):
    raw = cap.entries["raw"][cap.entries["flags"] == 0].astype(np.float64)
    lines = ["Dyno stream: %d samples, %d lost, %s" % (len(cap.ticks), cap.lost,
                                                       "; ".join(cap.problems) or "no integrity problems")]
    if len(cap.ticks) > 1:
        spacing = np.diff(cap.ticks)
        lines.append("  rate %.3f SPS (nominal %.3f), spacing jitter %.1f us"
                     % (1e6 / spacing.mean(), 1e6 / period_us, spacing.std()))
    if len(raw):
        lines.append("  raw mean %.0f, noise %.1f counts rms, range %d..%d"
                     % (raw.mean(), raw.std(), raw.min(), raw.max()))
    if clock_map is not None:
        lines.append("  clock: device %+.1f ppm vs master, latch residual %.1f us (%d/%d used)"
                     % ((clock_map.rate / nominal_clock_rate - 1) * 1e6,
                        clock_map.rms / clock_map.rate, clock_map.used, clock_map.total))
    else:
        lines.append("  clock: no usable latch fit")
    return "\n".join(lines)


def timestamp():
    return time.strftime("%Y%m%d-%H%M%S")
