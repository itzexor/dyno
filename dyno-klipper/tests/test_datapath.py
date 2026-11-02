import unittest

import numpy as np

from dyno import protocol, stream, timebase
from dyno.device import Device
from dyno.protocol import Command, DeviceError
from tests import firmware

PERIOD = 1024  # us, 976.5625 SPS
CLOCK_HZ = 64e6


@unittest.skipUnless(firmware.available(), "firmware source tree not found")
class DataPathTest(unittest.TestCase):
    def setUp(self):
        self.engine = firmware.Engine()
        self.device = Device(self.engine)

    def test_crc_known_answer(self):
        self.assertEqual(protocol.crc16(b"123456789"), 0x29B1)

    def test_control_round_trip_against_engine(self):
        info = self.device.connect()
        self.assertEqual(info["model"], "DYNO-REF")
        self.assertEqual(info["version"], "0.2.0")
        self.assertTrue(info["capabilities"] & protocol.CAP_STREAM)
        epoch = self.device.epoch
        self.assertNotEqual(epoch, 0)
        # A reconnect keeps the device's epoch and capture numbering.
        again = Device(self.engine)
        again.connect()
        self.assertEqual(again.epoch, epoch)
        status = self.device.status()
        self.assertEqual(protocol.MODE_NAMES[status["mode"]], "idle")
        with self.assertRaises(DeviceError) as caught:
            self.device.set_adc(5, 7)
        self.assertEqual(caught.exception.result, 5)  # no ADC setter on the reference engine
        for i in range(20):
            self.engine.sample(1000 + PERIOD * i, 500)
        self.assertEqual(self.device.tare(), 500)
        capture = self.device.arm(stream=True)
        self.assertEqual(capture, 1)
        self.device.start()
        self.assertEqual(self.device.status()["mode"], 2)
        with self.assertRaises(DeviceError):
            self.device.call(Command.READ)

    def test_stream_decoding_latches_and_clock_fit(self):
        self.device.connect()
        offset, ppm = 9.87e9, 37e-6
        rng = np.random.default_rng(4)

        def master_clock(tick):
            return offset + tick * (CLOCK_HZ / 1e6) * (1 + ppm)

        reader = firmware.Reader(self.engine, lambda t: master_clock(t) + rng.normal(0, 200))
        self.engine.sample(0xFFFF0000 - PERIOD, 0)  # START needs a healthy sample
        self.device.arm(stream=True)
        self.device.start()
        tick = 0xFFFF0000  # wraps the device's 32-bit tick mid-capture
        raws = []
        for i in range(5000):
            tick += PERIOD + int(rng.integers(-1, 2))
            raw = int(200000 * np.sin(i / 50.0))
            raws.append(raw)
            self.engine.now = tick
            self.engine.sample(tick, raw)
            if i % 4 == 3:
                reader.poll()
            if i % 50 == 17:
                self.engine.now = tick + int(rng.integers(0, PERIOD))
                reader.status(late_blocks=int(rng.integers(0, 2)))  # as seen on the printer
        self.device.stop()
        reader.poll()
        cap = stream.assemble(reader.messages, reader.statuses, PERIOD)
        self.assertEqual(cap.problems, [])
        self.assertEqual(cap.lost, 0)
        np.testing.assert_array_equal(cap.entries["raw"], raws)
        self.assertTrue((np.diff(cap.ticks) > PERIOD - 3).all())
        # Every latch decodes to the device tick at its read (16 us units).
        clock_map = timebase.fit_clock(cap.latches, CLOCK_HZ / 1e6)
        self.assertIsNotNone(clock_map)
        self.assertGreaterEqual(clock_map.used, 0.9 * len(reader.statuses))
        self.assertAlmostEqual(clock_map.rate / (CLOCK_HZ / 1e6), 1 + ppm, delta=3e-6)
        # Sample times recovered to a few microseconds across the whole capture.
        true_ticks = 0xFFFF0000 + np.cumsum(np.diff(np.r_[0, cap.ticks]))  # same spacing
        est = clock_map(cap.ticks)
        expect = master_clock(cap.ticks - cap.ticks[0] + (true_ticks[0] - true_ticks[0]))
        err_us = (est - est[0] - (expect - expect[0])) / (CLOCK_HZ / 1e6)
        self.assertLess(np.max(np.abs(err_us)), 5.0)

    def test_leftover_stream_is_drained_before_rearming(self):
        self.device.connect()
        self.engine.sample(0, 0)
        self.device.arm(stream=True)
        self.device.start()
        for i in range(37):
            self.engine.sample(PERIOD * (i + 1), i)
        self.device.stop()   # pads 37 -> 40 entries; nobody reads them
        with self.assertRaises(DeviceError):
            self.device.arm(stream=True)
        self.assertEqual(self.device.drain_stream(), 40)
        self.assertEqual(self.device.status()["buffered"], 0)
        self.assertEqual(self.device.arm(stream=True), 2)

    def test_overflow_and_gaps_are_reported(self):
        self.device.connect()
        reader = firmware.Reader(self.engine, lambda t: t * 64.0)
        self.engine.sample(0, 0)
        self.device.arm(stream=True)
        self.device.start()
        tick = 0
        for i in range(900):  # the ring holds 673 before dropping newest
            tick += PERIOD
            self.engine.now = tick
            self.engine.sample(tick, i)
        reader.status()
        reader.poll()
        for i in range(40):
            tick += PERIOD
            self.engine.sample(tick, i)
        self.device.stop()
        reader.poll()
        dropped = self.device.status()["overflow"]
        self.assertEqual(dropped, 900 - 673)
        cap = stream.assemble(reader.messages, reader.statuses, PERIOD, device_overflow=dropped)
        self.assertEqual(cap.lost, dropped)
        self.assertTrue(any("unreliable" in p for p in cap.problems))
        self.assertTrue(any("overflow" in p for p in cap.problems))
        self.assertTrue(any("gaps" in p for p in cap.problems))


if __name__ == "__main__":
    unittest.main()
