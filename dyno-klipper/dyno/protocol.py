"""Dyno wire protocol v1: framing, CRC, payload layouts and stream entries.

Pure functions only; no I/O. See the firmware's docs/protocol.md.
"""
import struct

import numpy as np

MAGIC = b"DY"
PROTOCOL = 1
HEADER_SIZE = 16
PAYLOAD_MAX = 110
FRAME_SIZE = 128
PAGE_SIZE = 32


class Command:
    INFO, STATUS, GET_CONFIG, SET_CONFIG, TARE, ARM, START, STOP = range(1, 9)
    READ, ACK, PROBE, TIME, RESET_ERRORS, SET_ADC, GET_ADC = range(9, 16)


RESULT_NAMES = {0: "ok", 1: "bad frame", 2: "bad state", 3: "bad value",
                4: "stale", 5: "unsupported"}
RESPONSE_SIZES = (18, 42, 41, 31, 18, 22, 18, 18, 18, 128, 18, 40, 22, 18, 18, 30)
MODE_NAMES = ("idle", "armed", "running", "stopped")

CAP_SYNTHETIC, CAP_ADC, CAP_PAGED, CAP_STREAM, CAP_ADC_CONFIG = 1, 2, 4, 8, 16
FLAG_SATURATED, FLAG_ADC_CRC, FLAG_MISSED = 1, 2, 4
FAULT_OVERFLOW, FAULT_ADC, FAULT_STALE = 1, 2, 4

# Stream window registers and entry layout.
REG_INT_STATUS, REG_FIFO_COUNT, REG_FIFO_DATA = 0x3A, 0x72, 0x74
INT_OVERFLOW = 0x10
ENTRY_SIZE = 6
BLOCK_ENTRIES = 8
BLOCK_SIZE = ENTRY_SIZE * BLOCK_ENTRIES
LATCH_SHIFT = 4
LATCH_BITS = 13


class ProtocolError(Exception):
    """Malformed or inconsistent traffic."""


class DeviceError(Exception):
    """The device executed the request and refused it."""

    def __init__(self, command, result):
        self.command, self.result = command, result
        super().__init__("command %d refused: %s" % (command, RESULT_NAMES.get(result, result)))


class DeviceReset(ProtocolError):
    """Epoch or capture no longer matches: the device restarted or moved on."""


def crc16(data):
    """CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection or xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def response_size(command):
    return RESPONSE_SIZES[command] if 0 <= command < len(RESPONSE_SIZES) else 18


def encode_request(command, request_id, epoch, capture, payload=b""):
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("payload too long")
    head = MAGIC + struct.pack("<BBHBBII", PROTOCOL, command, request_id, 0,
                               len(payload), epoch, capture)
    body = head + bytes(payload)
    return body + struct.pack("<H", crc16(body))


class Frame:
    __slots__ = ("command", "request_id", "result", "epoch", "capture", "payload")

    def __init__(self, command, request_id, result, epoch, capture, payload):
        self.command, self.request_id, self.result = command, request_id, result
        self.epoch, self.capture, self.payload = epoch, capture, payload


def decode_response(data):
    data = bytes(data)
    if len(data) < HEADER_SIZE + 2 or data[:2] != MAGIC or data[2] != PROTOCOL:
        raise ProtocolError("not a Dyno response")
    command, request_id, result, length, epoch, capture = struct.unpack_from("<BHBBII", data, 3)
    size = response_size(command)
    if len(data) != size:
        raise ProtocolError("response length %d, expected %d" % (len(data), size))
    if HEADER_SIZE + length > size - 2:
        raise ProtocolError("payload length exceeds response")
    if crc16(data[:-2]) != struct.unpack_from("<H", data, size - 2)[0]:
        raise ProtocolError("response CRC mismatch")
    return Frame(command, request_id, result, epoch, capture,
                 data[HEADER_SIZE:HEADER_SIZE + length])


def parse_info(p):
    if len(p) != 24:
        raise ProtocolError("invalid INFO payload")
    model, tick_hz, caps, fifo, block_max, version = struct.unpack("<8sIIHB5s", p)
    return dict(model=model.decode("ascii", "replace"), tick_hz=tick_hz, capabilities=caps,
                fifo=fifo, block_max=block_max, version=version.decode("ascii", "replace"))


def parse_status(p):
    if len(p) != 23:
        raise ProtocolError("invalid STATUS payload")
    keys = ("mode", "healthy", "triggered", "buffered", "overflow", "adc_errors",
            "faults", "next_sequence")
    return dict(zip(keys, struct.unpack("<BBBIIIII", p)))


def parse_adc(p):
    if len(p) != 12:
        raise ProtocolError("invalid GET_ADC payload")
    osr, gain, mode, clock, gain_reg, rate_mhz = struct.unpack("<BBHHHI", p)
    return dict(osr_code=osr, osr=128 << osr, gain_code=gain, gain=1 << gain,
                mode_reg=mode, clock_reg=clock, gain_reg=gain_reg, rate=rate_mhz / 1000.0)


def parse_board(p):
    if len(p) != 32 or p[:4] != b"DYBD":
        raise ProtocolError("invalid board diagnostic register")
    keys = ("build_id", "uptime_us", "i2c_requests", "i2c_errors", "adc_id", "adc_clock",
            "adc_gain", "adc_mode", "source", "clock_source", "adc_ready", "flags")
    return dict(zip(keys, struct.unpack_from("<IIIIHHHHBBBB", p, 4)))


OSR_CODES = range(3, 8)  # 3906 .. 244 SPS


def profile_rate(code):
    return 4e6 / (128 << code)


def osr_code(rate):
    """The OSR code whose nominal rate (4 MHz modulator) is nearest `rate`."""
    return min(OSR_CODES, key=lambda c: abs(np.log(profile_rate(c) / rate)))


ENTRY_DTYPE = np.dtype([("raw", "<i4"), ("tick16", "<u2"), ("flags", "u1"),
                        ("seq4", "u1"), ("pad", "?")])


def decode_entries(data):
    """Decode packed 6-byte stream entries into a structured array."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size % ENTRY_SIZE:
        raise ProtocolError("stream data is not a whole number of entries")
    e = buf.reshape(-1, ENTRY_SIZE).astype(np.int32)
    raw = e[:, 0] | (e[:, 1] << 8) | (e[:, 2] << 16)
    out = np.empty(len(e), dtype=ENTRY_DTYPE)
    out["raw"] = np.where(raw & 0x800000, raw - 0x1000000, raw)
    out["tick16"] = e[:, 3] | (e[:, 4] << 8)
    out["flags"] = e[:, 5] & 7
    out["seq4"] = (e[:, 5] >> 3) & 15
    out["pad"] = (e[:, 5] & 0x80) != 0
    return out
