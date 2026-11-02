"""The [dyno] extra's commands on stub Klipper objects and the simulated printer."""
import tempfile
import unittest

from dyno import extra
from tests import firmware
from tests.fakes import FakeHost, Plant, Sim


class CommandError(Exception):
    pass


class Config:
    def __init__(self, printer, values):
        self.printer, self.values = printer, values

    def get_printer(self):
        return self.printer

    def error(self, text):
        return CommandError(text)

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getfloat(self, key, default=None, **limits):
        return float(self.values.get(key, default))

    def getint(self, key, default=None, **limits):
        return int(self.values.get(key, default))

    def getboolean(self, key, default=None):
        return bool(self.values.get(key, default))

    def get_prefix_options(self, prefix):
        return [k for k in self.values if k.startswith(prefix)]


class Gcmd:
    def __init__(self, **params):
        self.params = params
        self.lines = []

    def respond_info(self, text):
        self.lines.append(text)

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_float(self, key, default=None, **limits):
        return float(self.params.get(key, default))

    def get_int(self, key, default=None, **limits):
        return int(self.params.get(key, default))


class Printer:
    command_error = CommandError

    def __init__(self, sim):
        self.sim = sim
        self.commands = {}
        self.endpoints = {}
        self.saved = {}
        printer = self
        self.objects = dict(
            gcode=type("G", (), dict(
                register_command=lambda s, n, fn, desc=None: printer.commands.__setitem__(n, fn),
                respond_info=lambda s, t: None, run_script_from_command=lambda s, t: None))(),
            webhooks=type("W", (), dict(
                register_endpoint=lambda s, p, fn: printer.endpoints.__setitem__(p, fn)))(),
            configfile=type("C", (), dict(
                set=lambda s, sec, key, val: printer.saved.__setitem__((sec, key), val)))())

    def get_reactor(self):
        return None

    def lookup_object(self, name):
        return self.objects[name]

    def register_event_handler(self, name, fn):
        pass


class StubBus:
    @staticmethod
    def MCU_I2C_from_config(config, default_addr, default_speed):
        return config.printer.sim.engine


@unittest.skipUnless(firmware.available(), "firmware source tree not found")
class ExtraTest(unittest.TestCase):
    def make(self, plant=None, **values):
        sim = Sim(plant)
        printer = Printer(sim)
        extra.bus = StubBus
        extra.BulkReader = lambda i2c: sim.reader
        values.setdefault("data_directory", tempfile.mkdtemp())
        dyno = extra.Dyno(Config(printer, values))
        dyno.device = sim.device
        dyno.host = FakeHost(sim)
        return sim, printer, dyno

    def test_registers_commands_and_endpoint(self):
        _, printer, _ = self.make()
        self.assertEqual(set(printer.commands), {
            "DYNO_STATUS", "DYNO_TARE", "DYNO_STREAM_TEST", "DYNO_PA", "DYNO_PA_CALIBRATE",
            "DYNO_PA_APPLY",
            "DYNO_PA_ROLLBACK", "DYNO_CANCEL"})
        self.assertIn("dyno/cancel", printer.endpoints)

    def test_status_reports_device(self):
        _, _, dyno = self.make()
        gcmd = Gcmd()
        dyno.cmd_STATUS(gcmd)
        self.assertIn("Dyno DYNO-REF 0.2.0", gcmd.lines[0])

    def test_preflight_refuses_low_nozzle_and_cold_hotend(self):
        sim, _, dyno = self.make(min_z=50.0)
        with self.assertRaisesRegex(CommandError, "Z >="):
            dyno.cmd_PA(Gcmd())
        sim2, _, dyno2 = self.make()
        sim2.extruder.heater.get_status = lambda e: dict(temperature=150.0, target=220.0)
        with self.assertRaisesRegex(CommandError, "target temperature"):
            dyno2.cmd_PA(Gcmd())
        self.assertEqual(sim2.toolhead.rows, [])  # nothing moved

    def test_apply_requires_an_accepted_result(self):
        _, _, dyno = self.make()
        with self.assertRaisesRegex(CommandError, "no accepted"):
            dyno.cmd_PA_APPLY(Gcmd())
        with self.assertRaisesRegex(CommandError, "nothing to roll back"):
            dyno.cmd_PA_ROLLBACK(Gcmd())

    def test_full_command_apply_and_rollback(self):
        sim, printer, dyno = self.make(Plant(tau=0.05), repeats=1, cycles=4)
        gcmd = Gcmd(MATERIAL="PLA")
        dyno.cmd_PA(gcmd)
        report = "\n".join(gcmd.lines)
        self.assertIn("recommended ADVANCE=", report)
        self.assertAlmostEqual(dyno.last["k"], 0.05, delta=0.005)
        self.assertEqual(sim.extruder.pa, 0.04)  # report only: PA unchanged
        name, meta, arrays = dyno.host.archives[-1]
        self.assertTrue(name.startswith("pa-"))
        self.assertEqual(meta["material"], "PLA")
        self.assertTrue(meta["result"]["accepted"])
        dyno.cmd_PA_APPLY(Gcmd(SAVE=1))
        self.assertAlmostEqual(sim.extruder.pa, dyno.last["k"], places=5)
        self.assertIn(("extruder", "pressure_advance"), printer.saved)
        dyno.cmd_PA_ROLLBACK(Gcmd())
        self.assertEqual(sim.extruder.pa, 0.04)
        self.assertFalse(dyno.busy)

    def test_calibrate_from_cold_homes_parks_heats_and_cools(self):
        sim, _, dyno = self.make(Plant(tau=0.05), repeats=1, cycles=4,
                                 park_position="150, 120, 40")
        sim.toolhead.homed = ""
        sim.extruder.heater.temperature = sim.extruder.heater.target = 25.0
        gcmd = Gcmd(TEMP=270, SOAK=5, MATERIAL="pc-cf")
        dyno.cmd_PA_CALIBRATE(gcmd)
        log = sim.gcode_log
        self.assertLess(log.index("G28"), log.index("M109 S270.0"))
        self.assertLess(log.index("M109 S270.0"),
                        next(i for i, s in enumerate(log) if s.startswith("SET_PRESSURE_ADVANCE")))
        self.assertEqual(log[-1], "M104 S0")
        self.assertEqual(sim.extruder.heater.target, 0.0)
        self.assertEqual(sim.toolhead.position[:3], [150.0, 120.0, 40.0])
        self.assertAlmostEqual(dyno.last["k"], 0.05, delta=0.005)
        self.assertEqual(sim.extruder.pa, 0.04)  # still report only
        self.assertIn("recommended ADVANCE=", "\n".join(gcmd.lines))

    def test_calibrate_cools_down_after_an_abort(self):
        sim, _, dyno = self.make(Plant(gain=0.0), repeats=1, cycles=4)
        with self.assertRaisesRegex(CommandError, "aborted"):
            dyno.cmd_PA_CALIBRATE(Gcmd(TEMP=250, SOAK=0, MATERIAL="PETG"))
        self.assertEqual(sim.gcode_log[-1], "M104 S0")
        self.assertEqual(sim.extruder.heater.target, 0.0)

    def test_calibrate_respects_home_0_and_leaves_the_heater_alone(self):
        sim, _, dyno = self.make()
        sim.toolhead.homed = "xy"
        with self.assertRaisesRegex(CommandError, "home X, Y and Z"):
            dyno.cmd_PA_CALIBRATE(Gcmd(TEMP=250, HOME=0, MATERIAL="PETG"))
        self.assertEqual(sim.gcode_log, [])  # no homing, no heating, no cooldown
        self.assertEqual(sim.extruder.heater.target, 220.0)

    def test_calibrate_validates_material_and_temperature_first(self):
        sim, _, dyno = self.make(temp_range_pekk="340, 380")
        for params, pattern in (
                (dict(MATERIAL="PC"), "needs MATERIAL=<name> and TEMP"),
                (dict(TEMP=270), "needs MATERIAL=<name> and TEMP"),
                (dict(MATERIAL="PLA", TEMP=270), "outside 180-240 C for PLA"),
                (dict(MATERIAL="PC", TEMP=230), "outside 250-310 C for PC"),
                (dict(MATERIAL="UNOBTAINIUM", TEMP=250), "temp_range_unobtainium"),
                (dict(MATERIAL="PEKK", TEMP=300), "outside 340-380 C for PEKK")):
            with self.assertRaisesRegex(CommandError, pattern):
                dyno.cmd_PA_CALIBRATE(Gcmd(**params))
        self.assertEqual(sim.gcode_log, [])
        self.assertEqual(dyno.material_range("PLA+")[0], "PLA")
        self.assertEqual(dyno.material_range("PA12-CF")[0], "PA")
        self.assertEqual(dyno.material_range("PPA-CF"), ("PPA", (270, 330)))
        self.assertEqual(dyno.material_range("PET-CF"), ("PET", (250, 300)))
        self.assertEqual(dyno.material_range("PETG")[0], "PETG")

    def test_abort_archives_partial_data_and_restores(self):
        sim, _, dyno = self.make(Plant(gain=0.0), repeats=1, cycles=4)
        with self.assertRaisesRegex(CommandError, "aborted: identification too weak"):
            dyno.cmd_PA(Gcmd())
        self.assertTrue(dyno.host.archives[-1][0].startswith("pa-aborted-"))
        self.assertEqual(sim.extruder.pa, 0.04)
        self.assertFalse(dyno.busy)


if __name__ == "__main__":
    unittest.main()
