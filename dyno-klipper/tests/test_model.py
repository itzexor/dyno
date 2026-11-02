import unittest

import numpy as np

from dyno import model
from tests import sim

ODR = 4e6 / 4096
SMOOTH = 0.040


def make_block(label, pa, tau, t0, rng, cycles=4, **kw):
    legs = sim.block_legs(2.0, 12.0, 0.40, 0.30, cycles, 0.8)
    rows, t1, _ = sim.leg_segments(legs, t0, 100.0, 0.05, 1000.0)
    seg = model.Segments(rows)
    times, force = sim.simulate_force(seg, pa, SMOOTH, tau, ODR, t0 - 0.3, t1 + 0.3, rng=rng, **kw)
    return model.Block(label, pa, SMOOTH, seg, times, force, ODR), t1


class SegmentsTest(unittest.TestCase):
    def test_nominal_matches_leg_amounts(self):
        rows, t1, e1 = sim.leg_segments([(0.4, 0.33), (0.3, 1.5)], 10.0, 5.0, 0.05, 1000.0)
        seg = model.Segments(rows)
        pos, vel, flag = seg.nominal(np.array([9.0, 10.0 + 0.2, t1 + 1.0]))
        self.assertAlmostEqual(pos[0], 5.0)
        self.assertAlmostEqual(pos[2], e1)
        self.assertAlmostEqual(e1, 5.0 + 0.33 + 1.5, places=9)
        self.assertGreater(vel[1], 0.0)
        self.assertEqual(list(flag), [False, True, False])

    def test_pa_shifts_stepper_ahead_and_conserves_material(self):
        rows, t1, e1 = sim.leg_segments(sim.block_legs(2, 12, 0.4, 0.3, 2, 0.5), 0.0, 0.0, 0.05, 1000.0)
        seg = model.Segments(rows)
        grid = np.arange(-0.5, t1 + 0.5, 1e-4)
        plain = model.stepper_position(seg, grid, 0.0, SMOOTH)
        advanced = model.stepper_position(seg, grid, 0.05, SMOOTH)
        self.assertAlmostEqual(advanced[-1], plain[-1], places=6)
        self.assertGreater(np.max(advanced - plain), 0.05 * 4.0)

    def test_exact_pa_makes_compression_track_reference_flow(self):
        tau = 0.045
        rows, t1, _ = sim.leg_segments(sim.block_legs(2, 12, 0.4, 0.3, 3, 0.8), 0.0, 0.0, 0.05, 1000.0)
        seg = model.Segments(rows)
        dt = 5e-5
        grid = np.arange(-0.8, t1 + 0.3, dt)
        p = model.stepper_position(seg, grid, tau, SMOOTH)
        s = p - model.low_pass(p, dt, tau)
        ref = tau * model.reference_flow(seg, grid, SMOOTH)
        settled = grid > 0.3
        self.assertLess(np.max(np.abs(s - ref)[settled]), 0.01 * np.max(ref))


class EstimationTest(unittest.TestCase):
    def test_identify_recovers_tau_and_delay(self):
        rng = np.random.default_rng(7)
        for tau in (0.018, 0.045, 0.090):
            blocks, t = [], 0.0
            for i in range(3):
                b, t = make_block("id%d" % i, 0.0, tau, t + 1.0, rng)
                blocks.append(b)
            fit = model.identify(blocks)
            self.assertAlmostEqual(fit["tau"], tau, delta=0.04 * tau, msg=str(fit))
            self.assertLessEqual(fit["ci68"][0], fit["tau"])
            self.assertLess(abs(fit["tau"] - tau), 4 * fit["se"] + 0.002 * tau)
            self.assertAlmostEqual(fit["delay"], 1.2e-3, delta=0.5e-3)
            self.assertLess(fit["residual_rms"], 1.1 * 40.0)  # only the injected noise remains
            self.assertFalse(fit["boundary"])
            self.assertIn("jackknife_se", fit)

    def test_identify_uses_blocks_run_with_pa(self):
        rng = np.random.default_rng(3)
        tau = 0.04
        blocks, t = [], 0.0
        for k in (0.0, 0.02, 0.04, 0.06):
            b, t = make_block("k%.2f" % k, k, tau, t + 1.0, rng)
            blocks.append(b)
        fit = model.identify(blocks)
        self.assertAlmostEqual(fit["tau"], tau, delta=0.04 * tau)

    def test_sweep_minimum_matches_tau(self):
        rng = np.random.default_rng(11)
        tau = 0.04
        blocks, t = [], 0.0
        for k in (0.0, 0.024, 0.032, 0.040, 0.048, 0.056, 0.024, 0.040, 0.056):
            b, t = make_block("k%.3f" % k, k, tau, t + 1.0, rng)
            blocks.append(b)
        result = model.sweep(blocks, 1.2e-3)
        self.assertFalse(result["boundary"])
        self.assertAlmostEqual(result["k_opt"], tau, delta=0.004, msg=str(result))
        self.assertGreater(result["k_se"], 0.0)

    def test_sweep_reports_boundary(self):
        rng = np.random.default_rng(5)
        blocks, t = [], 0.0
        for k in (0.0, 0.01, 0.02):
            b, t = make_block("k", k, 0.06, t + 1.0, rng)
            blocks.append(b)
        self.assertTrue(model.sweep(blocks, 1.2e-3)["boundary"])

    def test_pure_noise_is_not_a_confident_fit(self):
        rng = np.random.default_rng(9)
        blocks, t = [], 0.0
        for i in range(3):
            b, t = make_block("n%d" % i, 0.0, 0.04, t + 1.0, rng, gain=0.0)
            blocks.append(b)
        fit = model.identify(blocks)
        self.assertLess(fit["snr"], 3.0)
        self.assertLess(fit["r2"], 0.2)


if __name__ == "__main__":
    unittest.main()
