"""Kalico/Klipper extra: [dyno] section, G-code commands, host binding."""
import json
import logging
import os
import re
import threading

import numpy as np

if "." in (__package__ or ""):
    from .. import bus  # klippy.extras.bus; a failure here must surface
else:  # loaded as a top-level package by the tests, which inject a stub
    bus = None
from . import motion, routine
from .device import Device
from .motion import PlanError, Profile
from .protocol import CAP_STREAM, MODE_NAMES, DeviceError, DeviceReset, ProtocolError
from .stream import BulkReader

GAIN_CODES = {1 << c: c for c in range(8)}

# Sane nozzle temperature bounds (C) per base material. Variants resolve to
# their leading letters (PETG-CF -> PETG, PA12 -> PA, PLA+ -> PLA); add or
# override with `temp_range_<material>: min, max` in [dyno].
MATERIAL_TEMPS = {
    "PLA": (180, 240), "PETG": (210, 260), "PCTG": (220, 270), "ABS": (220, 270),
    "ASA": (230, 280), "HIPS": (220, 260), "PC": (250, 310), "PA": (240, 300),
    "NYLON": (240, 300), "TPU": (200, 250), "PP": (200, 250), "PVA": (180, 220),
    "PPA": (270, 330), "PET": (250, 300),
}


class KlipperHost:
    """The routine's view of the printer (see routine.py)."""

    def __init__(self, dyno):
        self.dyno = dyno
        self.printer = dyno.printer
        self.reactor = dyno.printer.get_reactor()

    @property
    def toolhead(self):
        return self.printer.lookup_object("toolhead")

    @property
    def extruder(self):
        return self.toolhead.get_extruder()

    @property
    def master_clock_rate(self):
        return self.dyno.reader.mcu.get_constant_float("CLOCK_FREQ") / 1e6

    def monotonic(self):
        return self.reactor.monotonic()

    def pause(self, until):
        self.reactor.pause(until)

    def gcode(self, script):
        self.dyno.gcode.run_script_from_command(script)

    def respond(self, text):
        self.dyno.gcode.respond_info(text)

    def reset_gcode_position(self):
        self.printer.lookup_object("gcode_move").reset_last_position()

    def clock_to_print_time(self, clock):
        return self.dyno.reader.mcu.clock_to_print_time(clock)

    def extract_segments(self, t0, t1):
        import chelper
        ffi_main, ffi_lib = chelper.get_ffi()
        trapq = self.extruder.get_trapq()
        rows, end = [], t1
        while True:
            data = ffi_main.new("struct pull_move[128]")
            count = ffi_lib.trapq_extract_old(trapq, data, len(data), t0, end)
            rows += [(m.print_time, m.move_t, m.start_v, m.accel, m.start_x, m.y_r)
                     for m in (data[i] for i in range(count))]
            if count < len(data):
                break
            end = data[count - 1].print_time
        return sorted(rows)

    def start_timer(self, callback, interval):
        reactor = self.reactor

        def fire(eventtime):
            return eventtime + interval if callback() else reactor.NEVER

        return reactor.register_timer(fire, reactor.NOW)

    def stop_timer(self, handle):
        self.reactor.unregister_timer(handle)

    def run_in_thread(self, fn):
        box = {}

        def work():
            try:
                box["value"] = fn()
            except Exception as e:  # re-raised on the reactor
                logging.exception("dyno analysis failed")
                box["error"] = e

        thread = threading.Thread(target=work, name="dyno-analysis", daemon=True)
        thread.start()
        while thread.is_alive():
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        if "error" in box:
            raise box["error"]
        return box["value"]

    def archive(self, name, meta, arrays):
        path = os.path.join(self.dyno.data_dir, name)
        os.makedirs(path, exist_ok=False)
        with open(os.path.join(path, "run.json"), "w") as f:
            json.dump(meta, f, indent=2, default=_jsonable)
        if arrays:
            np.savez_compressed(os.path.join(path, "data.npz"), **arrays)
        return path


def _jsonable(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


class Dyno:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=42, default_speed=400000)
        self.device = Device(self.i2c)
        self.reader = BulkReader(self.i2c)
        self.host = KlipperHost(self)
        self.rate = config.getfloat("sample_rate", 977.0, minval=240.0, maxval=4000.0)
        gain = config.getint("gain", 128)
        if gain not in GAIN_CODES:
            raise config.error("gain must be a power of two from 1 to 128")
        self.gain_code = GAIN_CODES[gain]
        self.data_dir = os.path.expanduser(config.get("data_directory", "~/printer_data/dyno"))
        self.profile = Profile(
            flow_low=config.getfloat("flow_low", 2.0, above=0.0),
            flow_high=config.getfloat("flow_high", 12.0, above=0.0),
            time_low=config.getfloat("time_low", 0.40, above=0.0),
            time_high=config.getfloat("time_high", 0.30, above=0.0),
            cycles=config.getint("cycles", 5, minval=2),
            lead=config.getfloat("lead_time", 0.8, above=0.0),
            wobble=config.getfloat("wobble", 0.05, above=0.0),
            accel=config.getfloat("accel", 1000.0, above=0.0),
            gap=config.getfloat("block_gap", 0.4, minval=0.1))
        self.repeats = config.getint("repeats", 2, minval=1)
        self.prime = config.getfloat("prime", 10.0, minval=0.0)
        self.retract = config.getfloat("retract", 1.0, minval=0.0)
        self.max_filament = config.getfloat("max_filament", 300.0, above=0.0)
        self.min_z = config.getfloat("min_z", 10.0)
        park = config.get("park_position", None)
        self.park = None
        if park is not None:
            try:
                self.park = [float(v) for v in park.split(",")]
            except ValueError:
                self.park = []
            if len(self.park) != 3 or self.park[2] < self.min_z:
                raise config.error("park_position must be 'x, y, z' with z >= min_z")
        self.soak = config.getfloat("soak_time", 30.0, minval=0.0)
        self.material_temps = dict(MATERIAL_TEMPS)
        for option in config.get_prefix_options("temp_range_"):
            try:
                lo, hi = (float(v) for v in config.get(option).split(","))
            except ValueError:
                raise config.error("%s must be 'min, max'" % option)
            if not 0 < lo < hi:
                raise config.error("%s must satisfy 0 < min < max" % option)
            self.material_temps[option[len("temp_range_"):].upper()] = (lo, hi)
        self.cooldown = config.getboolean("cooldown", True)
        self.limits = routine.Limits(
            min_snr=config.getfloat("min_snr", 4.0),
            max_rel_se=config.getfloat("max_relative_uncertainty", 0.10),
            max_drift=config.getfloat("max_drift", 0.15))
        self.busy = False
        self.cancel = False
        self.last = None        # last accepted result: dict(k, smooth_time, extruder, path)
        self.prior = None       # PA in effect before DYNO_PA_APPLY
        self.printer.register_event_handler("klippy:shutdown", self._on_shutdown)
        for name, fn, desc in (
                ("DYNO_STATUS", self.cmd_STATUS, "Report Dyno device and stream state"),
                ("DYNO_TARE", self.cmd_TARE, "Tare the Dyno load cell"),
                ("DYNO_STREAM_TEST", self.cmd_STREAM_TEST, "Stream without motion and report quality"),
                ("DYNO_PA", self.cmd_PA, "Measure pressure advance (report only unless APPLY=1)"),
                ("DYNO_PA_CALIBRATE", self.cmd_PA_CALIBRATE,
                 "Heat the nozzle, soak, run DYNO_PA, then cool down"),
                ("DYNO_PA_APPLY", self.cmd_PA_APPLY, "Apply the last accepted DYNO_PA result"),
                ("DYNO_PA_ROLLBACK", self.cmd_PA_ROLLBACK, "Restore PA from before DYNO_PA_APPLY"),
                ("DYNO_CANCEL", self.cmd_CANCEL, "Cancel a running Dyno routine")):
            self.gcode.register_command(name, fn, desc=desc)
        webhooks = self.printer.lookup_object("webhooks")
        webhooks.register_endpoint("dyno/cancel", self._web_cancel)

    # --- plumbing -----------------------------------------------------
    def _on_shutdown(self):
        self.cancel = True

    def _web_cancel(self, web_request):
        self.cancel = True
        web_request.send({"cancel_requested": self.busy})

    def _error(self, text):
        return self.printer.command_error("Dyno: " + text)

    def _connect(self):
        try:
            if self.device.info is None:
                self.device.connect()
            return self.device.info
        except (ProtocolError, DeviceError) as e:
            self.device.info = None
            raise self._error(str(e))

    def _guard(self):
        if self.busy:
            raise self._error("a routine is already running")

    def _capture(self):
        if not self.reader.available:
            raise self._error("the I2C MCU firmware lacks the bulk FIFO reader (config_mpu9250)")
        info = self._connect()
        if not info["capabilities"] & CAP_STREAM:
            raise self._error("device firmware has no stream window")
        return routine.Capture(self.host, self.device, self.reader, self.rate, self.gain_code)

    # --- commands -----------------------------------------------------
    def cmd_STATUS(self, gcmd):
        info = self._connect()
        try:
            status, adc, board = self.device.status(), self.device.get_adc(), self.device.board()
        except DeviceReset:
            self.device.info = None
            raise self._error("device reset since connecting; run DYNO_STATUS again")
        except (ProtocolError, DeviceError) as e:
            raise self._error(str(e))
        gcmd.respond_info(
            "Dyno %s %s (build %08x), epoch %08x, capture %d\n"
            "  %s, healthy=%d, buffered=%d, overflow=%d, adc_errors=%d, faults=%#x\n"
            "  ADC %.3f SPS, gain %d (MODE %04x CLOCK %04x GAIN %04x)\n"
            "  bus: %d requests, %d errors; bulk reader %s; last result %s"
            % (info["model"], info["version"], board["build_id"], self.device.epoch,
               self.device.capture, MODE_NAMES[status["mode"]], status["healthy"],
               status["buffered"], status["overflow"], status["adc_errors"], status["faults"],
               adc["rate"], adc["gain"], adc["mode_reg"], adc["clock_reg"], adc["gain_reg"],
               board["i2c_requests"], board["i2c_errors"],
               "available" if self.reader.available else "MISSING",
               "K=%.4f" % self.last["k"] if self.last else "none"))

    def cmd_TARE(self, gcmd):
        self._guard()
        self._connect()
        try:
            gcmd.respond_info("Dyno tare baseline %d" % self.device.tare())
        except (ProtocolError, DeviceError) as e:
            raise self._error("tare refused (%s); hold the nozzle unloaded and steady" % e)

    def cmd_CANCEL(self, gcmd):
        self.cancel = True
        gcmd.respond_info("Dyno cancel requested" if self.busy else "Dyno: nothing running")

    def cmd_STREAM_TEST(self, gcmd):
        self._guard()
        seconds = gcmd.get_float("SECONDS", 3.0, minval=0.5, maxval=60.0)
        capture = self._capture()
        self.busy = True
        try:
            capture.start()
            self.host.pause(self.host.monotonic() + seconds)
        except (ProtocolError, DeviceError, routine.RoutineError) as e:
            capture.stop()
            raise self._error(str(e))
        finally:
            errors = capture.stop() if capture.timer is not None else []
            self.busy = False
        messages, statuses = capture.snapshot()
        cap, clock_map, _ = self.host.run_in_thread(
            lambda: routine.decode(self.host, messages, statuses, capture.period_us,
                                   capture.overflow))
        meta = dict(kind="stream_test", seconds=seconds, device=capture.device_info,
                    adc=capture.adc, problems=cap.problems + errors, lost=cap.lost,
                    clock=clock_map.as_dict() if clock_map else None,
                    timer_error=capture.timer_error)
        path = self.host.run_in_thread(lambda: self.host.archive(
            "stream-" + routine.timestamp(), meta,
            dict(raw=cap.entries["raw"], flags=cap.entries["flags"], ticks=cap.ticks,
                 latches=np.asarray(cap.latches), statuses=np.asarray(statuses, dtype=float))))
        report = routine.stream_test_report(cap, clock_map, capture.period_us,
                                            self.host.master_clock_rate)
        gcmd.respond_info(report + ("\n  " + "; ".join(errors) if errors else "")
                          + "\n  data " + path)

    def _profile_from(self, gcmd):
        p = self.profile
        return Profile(
            flow_low=gcmd.get_float("FLOW_LOW", p.flow_low, above=0.0),
            flow_high=gcmd.get_float("FLOW_HIGH", p.flow_high, above=0.0),
            time_low=gcmd.get_float("TIME_LOW", p.time_low, above=0.0),
            time_high=gcmd.get_float("TIME_HIGH", p.time_high, above=0.0),
            cycles=gcmd.get_int("CYCLES", p.cycles, minval=2),
            lead=gcmd.get_float("LEAD", p.lead, above=0.0),
            wobble=gcmd.get_float("WOBBLE", p.wobble, above=0.0),
            accel=gcmd.get_float("ACCEL", p.accel, above=0.0),
            gap=gcmd.get_float("GAP", p.gap, minval=0.1))

    def _preflight(self, profile):
        eventtime = self.host.monotonic()
        th = self.host.toolhead
        th_status = th.get_status(eventtime)
        homed = th_status.get("homed_axes", "")
        if not all(axis in homed for axis in "xyz"):
            raise self._error("home X, Y and Z first")
        if th.get_position()[2] < self.min_z:
            raise self._error("raise the nozzle to Z >= %.1f; it extrudes in air" % self.min_z)
        ext = self.host.extruder
        heater = ext.get_heater()
        hs = heater.get_status(eventtime)
        if not heater.can_extrude or hs["target"] <= 0 or abs(hs["temperature"] - hs["target"]) > 3.0:
            raise self._error("hotend must be at its target temperature (now %.1f/%.1f)"
                              % (hs["temperature"], hs["target"]))
        es = ext.get_status(eventtime)
        if "pressure_advance" not in es:
            raise self._error("extruder %s has no pressure advance" % ext.get_name())
        try:
            motion.check(profile, ext.filament_area, ext.nozzle_diameter, ext.max_extrude_ratio,
                         th_status["max_velocity"], max(th_status["max_accel"], profile.accel))
        except PlanError as e:
            raise self._error(str(e))
        return dict(pa=es["pressure_advance"], smooth_time=es["smooth_time"],
                    accel=th_status["max_accel"], extruder=ext.get_name(),
                    temperature=hs["temperature"], target=hs["target"],
                    position=list(th.get_position()))

    def cmd_PA(self, gcmd):
        self._guard()
        profile = self._profile_from(gcmd)
        snap = self._preflight(profile)
        smooth = gcmd.get_float("SMOOTH_TIME", snap["smooth_time"], above=0.0, maxval=0.2)
        repeats = gcmd.get_int("REPEATS", self.repeats, minval=1, maxval=5)
        apply = gcmd.get_int("APPLY", 0, minval=0, maxval=1)
        label = gcmd.get("MATERIAL", "")
        area = self.host.extruder.filament_area
        per_block = motion.filament_per_block(profile, area)
        planned = self.prime + per_block * (3 + 6 * repeats)
        if planned > self.max_filament:
            raise self._error("plan needs ~%.0f mm of filament (max_filament %.0f)"
                              % (planned, self.max_filament))
        capture = self._capture()
        self.cancel = False
        self.busy = True
        exp = routine.PaExperiment(self.host, capture, profile, snap["extruder"], smooth, repeats,
                                   self.max_filament, self.limits, lambda: self.cancel)
        errors, unrestored, started = [], [], False
        gcmd.respond_info("Dyno PA: starting (~%.0f s, ~%.0f mm filament)"
                          % ((3 + 6 * repeats) * motion.block_duration(profile), planned))
        try:
            self.host.gcode("SET_VELOCITY_LIMIT ACCEL=%.1f" % profile.accel)
            capture.start()
            started = True
            routine.execute(exp, self.prime, gcmd.respond_info)
        except routine.Cancelled:
            errors.append("cancelled")
        except (routine.RoutineError, ProtocolError, DeviceError, self.printer.command_error) as e:
            errors.append(str(e))
        finally:
            if started:
                errors += capture.stop()
            unrestored = routine.restore(self.host, exp, snap, 0.0 if errors else self.retract)
            self.busy = False
        meta = dict(kind="pa", material=label, profile=profile.as_dict(), smooth_time=smooth,
                    repeats=repeats, snapshot=snap, device=capture.device_info, adc=capture.adc,
                    filament=exp.filament, errors=errors, unrestored=unrestored,
                    blocks=[dict(kind=r.kind, pa=r.pa, t0=r.t0, t1=r.t1) for r in exp.records])
        if errors:
            path = self._archive_partial(exp, capture, meta)
            raise self._error("aborted: %s%s; partial data %s"
                              % ("; ".join(errors),
                                 ". NOT RESTORED: " + "; ".join(unrestored) if unrestored else "",
                                 path))
        result, arrays = exp.analyse()
        meta.update(result=result)
        path = self.host.run_in_thread(lambda: self.host.archive(
            "pa-" + routine.timestamp(), meta, arrays))
        gcmd.respond_info(routine.format_report(result, exp.filament, path, snap["pa"]))
        if unrestored:
            gcmd.respond_info("Dyno: NOT RESTORED: " + "; ".join(unrestored))
        if result["accepted"]:
            self.last = dict(k=result["k"], smooth_time=smooth, extruder=snap["extruder"], path=path)
            if apply:
                self._apply(gcmd, save=False)

    def _park(self):
        """Raise to a safe height first, then travel, then settle to park Z."""
        th = self.host.toolhead
        pos = th.get_position()
        if self.park is None:
            if pos[2] < self.min_z:
                th.manual_move([None, None, self.min_z], 10.0)
            return
        x, y, z = self.park
        th.manual_move([None, None, max(pos[2], z)], 10.0)
        th.manual_move([x, y, None], 100.0)
        th.manual_move([None, None, z], 10.0)

    def material_range(self, material):
        name = material.strip().upper()
        if name in self.material_temps:
            return name, self.material_temps[name]
        base = re.match(r"[A-Z]+", name)
        if base and base.group() in self.material_temps:
            return base.group(), self.material_temps[base.group()]
        return name, None

    def cmd_PA_CALIBRATE(self, gcmd):
        """DYNO_PA_CALIBRATE MATERIAL=<name> TEMP=<C> [SOAK=s] [HOME=1] [PARK=1]
        [COOLDOWN=1] plus any DYNO_PA parameter (APPLY, REPEATS, FLOW_HIGH, ...).

        Everything is validated before anything moves or heats. The heater is
        switched off afterwards only if this command set it."""
        self._guard()
        material = gcmd.get("MATERIAL", "")
        if gcmd.get("TEMP", None) is None or not material:
            raise self._error("DYNO_PA_CALIBRATE needs MATERIAL=<name> and TEMP=<C>; "
                              "it never picks a temperature for you")
        temp = gcmd.get_float("TEMP", above=0.0)
        base, bounds = self.material_range(material)
        if bounds is None:
            raise self._error("no temperature range for %s (known: %s); add "
                              "'temp_range_%s: min, max' to [dyno]"
                              % (material, " ".join(sorted(self.material_temps)), base.lower()))
        if not bounds[0] <= temp <= bounds[1]:
            raise self._error("TEMP=%.0f is outside %.0f-%.0f C for %s"
                              % (temp, bounds[0], bounds[1], base))
        soak = gcmd.get_float("SOAK", self.soak, minval=0.0)
        cooldown = gcmd.get_int("COOLDOWN", int(self.cooldown), minval=0, maxval=1)
        self._connect()  # fail fast, before heating, if the board is unreachable
        self.cancel = False
        heated = False
        try:
            th = self.host.toolhead
            homed = th.get_status(self.host.monotonic()).get("homed_axes", "")
            if not all(a in homed for a in "xyz"):
                if not gcmd.get_int("HOME", 1, minval=0, maxval=1):
                    raise self._error("home X, Y and Z first (or HOME=1)")
                gcmd.respond_info("Dyno: homing")
                self.host.gcode("G28")
            if gcmd.get_int("PARK", 1, minval=0, maxval=1):
                self._park()
                th.wait_moves()
            gcmd.respond_info("Dyno: heating to %.0f C for %s" % (temp, base))
            heated = True
            self.host.gcode("M109 S%.1f" % temp)
            if soak:
                gcmd.respond_info("Dyno: soaking %.0f s at temperature" % soak)
                self.host.pause(self.host.monotonic() + soak)
            if self.cancel:
                raise self._error("cancelled before measuring")
            self.cmd_PA(gcmd)
        finally:
            if cooldown and heated:
                try:
                    self.host.gcode("M104 S0")
                    gcmd.respond_info("Dyno: hotend heater off")
                except Exception as e:
                    gcmd.respond_info("Dyno: COULD NOT TURN THE HEATER OFF: %s" % e)

    def _archive_partial(self, exp, capture, meta):
        messages, statuses = capture.snapshot()
        blob = b"".join(m[1] for m in messages)
        arrays = dict(stream=np.frombuffer(blob, dtype=np.uint8),
                      sequences=np.array([m[0] for m in messages]),
                      statuses=np.asarray(statuses, dtype=float))
        for i, r in enumerate(exp.records):
            arrays["segments_%d" % i] = np.asarray(r.rows, dtype=float)
        try:
            return self.host.run_in_thread(lambda: self.host.archive(
                "pa-aborted-" + routine.timestamp(), meta, arrays))
        except Exception as e:
            return "(archive failed: %s)" % e

    def _apply(self, gcmd, save):
        es = self.host.extruder.get_status(self.host.monotonic())
        self.prior = dict(pa=es["pressure_advance"], smooth_time=es["smooth_time"],
                          extruder=self.last["extruder"])
        self.host.gcode("SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f SMOOTH_TIME=%.6f"
                        % (self.last["extruder"], self.last["k"], self.last["smooth_time"]))
        text = "Dyno: pressure_advance %.4f applied (was %.4f)" % (self.last["k"], self.prior["pa"])
        if save:
            configfile = self.printer.lookup_object("configfile")
            configfile.set(self.last["extruder"], "pressure_advance", "%.4f" % self.last["k"])
            text += "; run SAVE_CONFIG to make it permanent"
        gcmd.respond_info(text)

    def cmd_PA_APPLY(self, gcmd):
        self._guard()
        if self.last is None:
            raise self._error("no accepted DYNO_PA result to apply")
        self._apply(gcmd, save=bool(gcmd.get_int("SAVE", 0, minval=0, maxval=1)))

    def cmd_PA_ROLLBACK(self, gcmd):
        self._guard()
        if self.prior is None:
            raise self._error("nothing to roll back")
        self.host.gcode("SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f SMOOTH_TIME=%.6f"
                        % (self.prior["extruder"], self.prior["pa"], self.prior["smooth_time"]))
        gcmd.respond_info("Dyno: pressure_advance restored to %.4f" % self.prior["pa"])
        self.prior = None


def load_config(config):
    return Dyno(config)

