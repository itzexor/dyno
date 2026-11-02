"""The firmware's portable engine, compiled for the host and driven via ctypes.

Set DYNO_FIRMWARE to the firmware tree; by default ../dyno beside this repo.
"""
import ctypes
import os
import pathlib
import struct
import subprocess
import tempfile

ROOT = pathlib.Path(os.environ.get(
    "DYNO_FIRMWARE", pathlib.Path(__file__).resolve().parents[2] / "dyno"))
CORE = ROOT / "src" / "core"
_lib = None


def available():
    return (CORE / "dyno.c").exists()


def library():
    global _lib
    if _lib is None:
        out = pathlib.Path(tempfile.mkdtemp()) / "libdyno.so"
        subprocess.run(["cc", "-std=c11", "-O2", "-fPIC", "-shared", "-I", str(CORE),
                        str(CORE / "dyno.c"), str(CORE / "link.c"), "-o", str(out)], check=True)
        lib = ctypes.CDLL(str(out))
        lib.dyno_reg_read.restype = ctypes.c_size_t
        lib.dyno_link_byte.restype = ctypes.c_uint8
        lib.dyno_reg_owns.restype = ctypes.c_bool
        _lib = lib
    return _lib


class Engine:
    """One device: engine state, paged link, and the register window."""

    def __init__(self, epoch=0, tick_hz=1000000):
        self.lib = library()
        self.core = ctypes.create_string_buffer(16384)
        self.link = ctypes.create_string_buffer(512)
        self.lib.dyno_init(self.core, ctypes.c_uint32(epoch), ctypes.c_uint32(tick_hz))
        self.lib.dyno_link_init(self.link, self.core)
        self.now = 0

    def sample(self, tick, raw, flags=0):
        self.lib.dyno_sample(self.core, ctypes.c_uint32(tick & 0xFFFFFFFF), ctypes.c_int32(raw),
                             ctypes.c_uint8(flags))

    # I2C master view -------------------------------------------------
    def i2c_write(self, data):
        data = bytes(data)
        if len(data) == 1 and self.lib.dyno_reg_owns(ctypes.c_uint8(data[0])):
            self.lib.dyno_reg_select(self.core, ctypes.c_uint8(data[0]))
            return
        self.lib.dyno_reg_select(self.core, ctypes.c_uint8(0))
        self.lib.dyno_link_write(self.link, data, ctypes.c_size_t(len(data)))

    def i2c_read(self, write, count):
        write = bytes(write)
        if write == b"\xf1":  # board diagnostic register (served by the I2C driver)
            snapshot = b"DYBD" + struct.pack("<IIIIHHHHBBBB", 0x1234ABCD, self.now, 0, 0,
                                             0x2205, 0x0116, 7, 0x0510, 0, 1, 1, 3)
            return {"response": snapshot[:count]}
        if len(write) == 1 and self.lib.dyno_reg_owns(ctypes.c_uint8(write[0])):
            self.lib.dyno_reg_select(self.core, ctypes.c_uint8(write[0]))
            out = ctypes.create_string_buffer(48)
            n = self.lib.dyno_reg_read(self.core, ctypes.c_uint32(self.now & 0xFFFFFFFF), out)
            data = out.raw[:n] + b"\xff" * max(0, count - n)
            self.lib.dyno_reg_done(self.core, ctypes.c_size_t(min(count, n)))
            return {"response": data[:count]}
        self.i2c_write(write)
        return {"response": bytes(self.lib.dyno_link_byte(self.link, ctypes.c_size_t(i))
                                  for i in range(count))}


class Reader:
    """The master MCU's FIFO-accelerometer reader, transaction for transaction."""

    def __init__(self, engine, clock):
        self.engine, self.clock = engine, clock   # clock(device_tick) -> master clock
        self.cached = 0
        self.sequence = 0
        self.overflows = 0
        self.messages = []
        self.statuses = []

    def _count(self):
        msg = self.engine.i2c_read([0x72], 2)["response"]
        return ((msg[0] & 0x1F) << 8) | msg[1]

    def poll(self):
        while True:
            if self.cached < 48:
                self.cached = self._count()
            if self.cached < 48:
                return
            block = self.engine.i2c_read([0x74], 48)["response"]
            self.messages.append((self.sequence & 0xFFFF, block))
            self.sequence += 1
            self.cached -= 48

    def status(self, query_ticks=7680, late_blocks=0):
        """late_blocks: blocks delivered between the status reply and the
        host reading its message count (the real reactor allows this)."""
        if self.engine.i2c_read([0x3A], 1)["response"][0] & 0x10:
            self.overflows += 1
        time1 = self.clock(self.engine.now) - 0.58 * query_ticks
        latch = self._count()
        sent = self.sequence & 0xFFFF
        for _ in range(late_blocks):
            if self.cached < 48:
                self.cached = self._count()
            if self.cached >= 48:
                block = self.engine.i2c_read([0x74], 48)["response"]
                self.messages.append((self.sequence & 0xFFFF, block))
                self.sequence += 1
                self.cached -= 48
        self.statuses.append((int(round(time1)), query_ticks, latch, self.overflows,
                              sent, len(self.messages)))
