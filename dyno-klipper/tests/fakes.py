"""A simulated printer for routine tests: virtual time, toolhead, plant, device.

Print time equals reactor time here. The master MCU clock runs at 64 MHz with
an offset; the device tick runs at 1 MHz with its own crystal error. Force is
produced by the true melt plant from the commanded stepper motion, sampled at
the device's DRDY ticks through the compiled firmware engine and the
emulated FIFO reader.
"""
import math

import numpy as np

from dyno import model
from dyno.device import Device
from tests import firmware

ODR = 4e6 / 4096
PERIOD = 1e6 / ODR
CLOCK = 64e6
CLOCK_OFFSET = 1.234e9
DEVICE_PPM = 23e-6


class Plant:
    def __init__(self, tau=0.04, gain=4000.0, delay=1.2e-3, noise=30.0, offset=150000.0, seed=2):
        self.tau, self.gain, self.delay, self.noise, self.offset = tau, gain, delay, noise, offset
        self.rng = np.random.default_rng(seed)


class FakeHeater:
    def __init__(self, temperature=220.0):
        self.temperature = self.target = temperature
        self.can_extrude = True

    def get_status(self, eventtime):
        return dict(temperature=self.temperature, target=self.target)


class FakeExtruder:
    def __init__(self, sim):
        self.sim = sim
        self.filament_area = math.pi * 0.875 ** 2
        self.nozzle_diameter = 0.4
        self.max_extrude_ratio = 80.0 / self.filament_area
        self.pa, self.smooth_time = 0.04, 0.04
        self.heater = FakeHeater()

    def get_status(self, eventtime):
        return dict(pressure_advance=self.pa, smooth_time=self.smooth_time)

    def get_heater(self):
        return self.heater

    def get_name(self):
        return "extruder"


class FakeToolhead:
    def __init__(self, sim):
        self.sim = sim
        self.position = [100.0, 100.0, 30.0, 0.0]
        self.cursor = 0.0
        self.max_accel, self.max_velocity = 5000.0, 300.0
        self.homed = "xyz"
        self.rows = []      # (print_time, move_t, start_v, accel, start_e, pa_flag, pa, smooth)

    def get_position(self):
        return list(self.position)

    def _start(self):
        self.cursor = max(self.cursor, self.sim.now + 0.1)

    def move(self, newpos, speed):
        self._start()
        ext = self.sim.extruder
        dxy = math.hypot(newpos[0] - self.position[0], newpos[1] - self.position[1])
        de = newpos[3] - self.position[3]
        if dxy > 0:   # reversal legs: start and end at rest
            ratio, accel, dist, flag = de / dxy, self.max_accel, dxy, 1.0
        else:         # extrude-only: no PA
            ratio, accel, dist, flag = math.copysign(1.0, de), 2000.0, abs(de), 0.0
        v = min(speed, math.sqrt(accel * dist))
        ta = v / accel
        cruise = max(0.0, (dist - v * ta) / v)
        e, t = self.position[3], self.cursor
        pa = ext.pa if flag else 0.0
        for dur, v0, a in ((ta, 0.0, accel), (cruise, v, 0.0), (ta, v, -accel)):
            if dur > 0:
                self.rows.append((t, dur, v0 * ratio, a * ratio, e, flag * pa, pa, ext.smooth_time))
                e += (v0 + 0.5 * a * dur) * dur * ratio
                t += dur
        self.cursor = t
        self.position = list(newpos)

    def manual_move(self, coord, speed):
        self._start()
        new = list(self.position)
        for i, v in enumerate(coord):
            if v is not None:
                new[i] = v
        self.cursor += math.dist(new[:3], self.position[:3]) / speed
        self.position = new

    def dwell(self, delay):
        self._start()
        self.cursor += delay

    def get_last_move_time(self):
        self._start()
        return self.cursor

    def wait_moves(self):
        self.sim.advance(self.cursor)

    def get_status(self, eventtime):
        return dict(homed_axes=self.homed, max_accel=self.max_accel, max_velocity=self.max_velocity)


class FakeReader:
    """BulkReader stand-in: the firmware Reader polled on virtual time."""

    def __init__(self, sim):
        self.sim = sim
        self.available = True
        self.active = False
        self.inner = None
        self.drop_every = 0

    @property
    def messages(self):
        return self.inner.messages if self.inner else []

    @property
    def statuses(self):
        return self.inner.statuses if self.inner else []

    def start(self, poll_seconds):
        self.inner = firmware.Reader(self.sim.engine, self.sim.master_clock_of_tick)
        self.poll_seconds = poll_seconds
        self.active = True

    def poll(self):
        if self.active:
            before = len(self.inner.messages)
            self.inner.poll()
            if self.drop_every:  # lose every Nth block in transit
                kept = [m for m in self.inner.messages[before:] if (m[0] + 1) % self.drop_every]
                del self.inner.messages[before:]
                self.inner.messages.extend(kept)

    def poll_status(self):
        self.sim.engine.now = self.sim.device_tick(self.sim.now)
        self.inner.status()

    def stop(self):
        self.poll()
        self.active = False


class FakeDevice(Device):
    """The real control path; the reference engine has no ADC, so report the
    board's boot profile for GET_ADC and accept SET_ADC as a no-op."""

    def get_adc(self):
        return dict(osr_code=5, osr=4096, gain_code=7, gain=128, mode_reg=0x510,
                    clock_reg=0x116, gain_reg=7, rate=ODR)

    def set_adc(self, osr_code, gain_code):
        pass


class Sim:
    def __init__(self, plant=None):
        self.now = 10.0
        self.plant = plant or Plant()
        self.engine = firmware.Engine()
        self.device = FakeDevice(self.engine)
        self.extruder = FakeExtruder(self)
        self.toolhead = FakeToolhead(self)
        self.reader = FakeReader(self)
        self.timers = []
        self.gcode_log = []
        self.cancel_at_block = None
        self.next_drdy = self.now
        self.next_poll = self.now
        self.engine.sample(self.device_tick(self.now), int(self.plant.offset))

    # clocks
    def device_tick(self, t):
        return int((t * 1e6) * (1 + DEVICE_PPM)) & 0xFFFFFFFF

    def master_clock_of_tick(self, tick):
        # inverse of device_tick around now (unwrapped by proximity)
        full = (int(self.now * 1e6 * (1 + DEVICE_PPM)) & ~0xFFFFFFFF) + tick
        t = full / 1e6 / (1 + DEVICE_PPM)
        return CLOCK_OFFSET + t * CLOCK

    # force
    def force(self, times):
        th = self.toolhead
        rows = [r for r in th.rows if r[0] + r[1] > times[0] - 1.0 and r[0] < times[-1]]
        if not rows:
            return np.full(len(times), self.plant.offset)
        dt = 1.0 / (ODR * 10)
        grid = np.arange(times[0] - 1.0, times[-1] + dt, dt)
        seg = model.Segments([r[:6] for r in rows])
        pos, vel, flag = seg.nominal(grid)
        idx = np.clip(np.searchsorted(seg.t0, grid, side="right") - 1, 0, len(rows) - 1)
        pa = np.array([r[6] for r in sorted(rows)])[idx] * flag
        smooth = max(r[7] for r in rows)
        p = pos + pa * vel
        if pa.any():
            p = model.triangle_smooth(p, dt, smooth)
        s = model.sinc3(p - model.low_pass(p, dt, self.plant.tau), 10)
        f = self.plant.offset + self.plant.gain * np.interp(times - self.plant.delay, grid, s)
        return f + self.plant.rng.normal(0, self.plant.noise, len(times))

    # time
    def advance(self, until):
        while self.now < until:
            step = min(until, self.now + 0.05)
            due = [t for t in np.arange(self.next_drdy, step, PERIOD / 1e6)]
            if due:
                forces = self.force(np.array(due))
                for t, f in zip(due, forces):
                    self.engine.sample(self.device_tick(t), int(f))
                    if self.reader.active and t >= self.next_poll:
                        self.engine.now = self.device_tick(t)
                        self.reader.poll()
                        self.next_poll = t + self.reader.poll_seconds
                self.next_drdy = due[-1] + PERIOD / 1e6
            self.now = step
            for timer in list(self.timers):
                if timer["next"] <= self.now:
                    timer["next"] = self.now + timer["interval"]
                    if not timer["callback"]():
                        self.timers.remove(timer)


class FakeHost:
    def __init__(self, sim):
        self.sim = sim
        self.toolhead, self.extruder = sim.toolhead, sim.extruder
        self.master_clock_rate = CLOCK / 1e6
        self.archives = []
        self.messages = []

    def monotonic(self):
        return self.sim.now

    def pause(self, until):
        self.sim.advance(until)

    def gcode(self, script):
        self.sim.gcode_log.append(script)
        heater = self.extruder.heater
        if script.startswith(("M109", "M104")):
            heater.target = float(script.split("S")[1])
            if script.startswith("M109"):
                self.sim.advance(self.sim.now + 30.0)  # heating time
                heater.temperature = heater.target
            return
        if script == "G28":
            self.toolhead.homed = "xyz"
            self.toolhead.position[:3] = [0.0, 0.0, 0.0]
            return
        words = dict(w.split("=") for w in script.split()[1:])
        if script.startswith("SET_PRESSURE_ADVANCE"):
            self.extruder.pa = float(words["ADVANCE"])
            self.extruder.smooth_time = float(words.get("SMOOTH_TIME", self.extruder.smooth_time))
        elif script.startswith("SET_VELOCITY_LIMIT"):
            self.toolhead.max_accel = float(words["ACCEL"])

    def respond(self, text):
        self.messages.append(text)

    def reset_gcode_position(self):
        pass

    def extract_segments(self, t0, t1):
        return [r[:6] for r in self.toolhead.rows if t0 <= r[0] <= t1]

    def clock_to_print_time(self, clock):
        return (clock - CLOCK_OFFSET) / CLOCK

    def start_timer(self, callback, interval):
        timer = dict(callback=callback, interval=interval, next=self.sim.now)
        self.sim.timers.append(timer)
        return timer

    def stop_timer(self, handle):
        if handle in self.sim.timers:
            self.sim.timers.remove(handle)

    def run_in_thread(self, fn):
        return fn()

    def archive(self, name, meta, arrays):
        self.archives.append((name, meta, arrays))
        return "memory:" + name
