"""The full DYNO_PA sequence on a simulated printer with a known melt plant."""
import unittest

import numpy as np

from dyno import motion, routine
from tests import firmware
from tests.fakes import ODR, FakeHost, Plant, Sim

SNAP = dict(pa=0.04, smooth_time=0.04, accel=5000.0, extruder="extruder")


def run_pa(sim, repeats=1, cancel_after=None):
    host = FakeHost(sim)
    capture = routine.Capture(host, sim.device, sim.reader, ODR, 7)
    profile = motion.Profile(cycles=4)
    blocks = {"n": 0}
    exp = routine.PaExperiment(host, capture, profile, "extruder", 0.04, repeats, 300.0,
                               routine.Limits(), lambda: blocks["n"] == cancel_after,
                               rng=np.random.default_rng(1))
    original = exp.run_block

    def counted(k, kind):
        blocks["n"] += 1
        original(k, kind)

    exp.run_block = counted
    errors = []
    try:
        host.gcode("SET_VELOCITY_LIMIT ACCEL=%.1f" % profile.accel)
        capture.start()
        routine.execute(exp, 5.0, host.respond)
    except routine.Cancelled:
        errors.append("cancelled")
    except routine.RoutineError as e:
        errors.append(str(e))
    finally:
        errors += capture.stop()
        unrestored = routine.restore(host, exp, SNAP, 0.0 if errors else 1.0)
    return host, exp, errors, unrestored


@unittest.skipUnless(firmware.available(), "firmware source tree not found")
class RoutineTest(unittest.TestCase):
    def assert_restored(self, sim, unrestored, y0=100.0):
        self.assertEqual(unrestored, [])
        self.assertEqual(sim.extruder.pa, SNAP["pa"])
        self.assertEqual(sim.toolhead.max_accel, SNAP["accel"])
        self.assertAlmostEqual(sim.toolhead.position[1], y0)
        self.assertFalse(sim.reader.active)
        self.assertEqual(sim.timers, [])

    def test_recovers_known_pressure_advance(self):
        sim = Sim(Plant(tau=0.045))
        host, exp, errors, unrestored = run_pa(sim)
        self.assertEqual(errors, [])
        self.assert_restored(sim, unrestored)
        kinds = [r.kind for r in exp.records]
        self.assertEqual(kinds[:2], ["warmup", "ident"])
        self.assertEqual(kinds[-1], "ident")
        self.assertGreaterEqual(kinds.count("sweep"), 5)
        result, arrays = exp.analyse()
        self.assertTrue(result["accepted"], result["reasons"])
        self.assertAlmostEqual(result["k"], 0.045, delta=0.0045)
        self.assertAlmostEqual(result["identification"]["tau"], 0.045, delta=0.0025)
        self.assertEqual(result["lost"], 0)
        self.assertLess(result["clock"]["residual_rms_ticks"] / 64.0, 20.0)
        report = routine.format_report(result, exp.filament, "memory:x", SNAP["pa"])
        self.assertIn("recommended ADVANCE=", report)
        self.assertEqual(len(arrays["raw"]), len(arrays["times"]))
        # Every SET_PRESSURE_ADVANCE happened with the toolhead idle, and PA
        # never changed inside a block: every trapq row of a block carries it.
        for rec in exp.records:
            flags = {round(r[5], 6) for r in rec.rows if r[5]}
            self.assertTrue(flags <= {round(rec.pa, 6)}, (rec.pa, flags))

    def test_cancel_restores_everything(self):
        sim = Sim()
        host, exp, errors, unrestored = run_pa(sim, cancel_after=3)
        self.assertEqual(errors, ["cancelled"])
        self.assertEqual(len(exp.records), 3)
        self.assert_restored(sim, unrestored)
        self.assertTrue(sim.gcode_log[-2].startswith("SET_PRESSURE_ADVANCE"))

    def test_weak_signal_stops_before_the_sweep(self):
        sim = Sim(Plant(gain=0.0))
        host, exp, errors, unrestored = run_pa(sim)
        self.assertEqual(len(errors), 1)
        self.assertIn("too weak", errors[0])
        self.assertEqual([r.kind for r in exp.records], ["warmup", "ident"])
        self.assert_restored(sim, unrestored)

    def test_lost_bulk_blocks_reject_the_result(self):
        sim = Sim(Plant(tau=0.03))
        sim.reader.drop_every = 400
        host, exp, errors, unrestored = run_pa(sim)
        self.assertEqual(errors, [])
        result, _ = exp.analyse()
        self.assertFalse(result["accepted"])
        self.assertTrue(any("lost in transit" in r for r in result["reasons"]), result["reasons"])

    def test_final_block_lost_after_motion_is_only_noted(self):
        # As on the printer: the MCU dropped the padded tail block after STOP.
        sim = Sim(Plant(tau=0.045))
        host, exp, errors, unrestored = run_pa(sim)
        self.assertEqual(errors, [])
        del sim.reader.inner.messages[-1]
        result, _ = exp.analyse()
        self.assertTrue(result["accepted"], result["reasons"])
        self.assertTrue(any("never received" in n for n in result["notes"]), result["notes"])

    def test_stalled_delivery_is_detected(self):
        sim = Sim(Plant(tau=0.03))
        host, exp, errors, unrestored = run_pa(sim, cancel_after=1)
        messages, statuses = exp.capture.snapshot()
        cut = len(messages) // 2  # everything after this never arrives
        trimmed = [s[:5] + (min(s[5], cut),) for s in statuses]
        from dyno import stream
        cap = stream.assemble(messages[:cut], trimmed, exp.capture.period_us)
        self.assertTrue(any("never received" in p for p in cap.problems), cap.problems)
        self.assertGreater(cap.lost, 0)


if __name__ == "__main__":
    unittest.main()
