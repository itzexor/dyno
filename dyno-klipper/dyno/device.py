"""Framed control channel to a Dyno over an I2C master.

`bus` provides i2c_write(list_of_bytes) and i2c_read(write_bytes, count),
the latter returning {"response": bytes}. Under Klipper this is MCU_I2C and
every call must run on the reactor (a G-code handler or timer), never on a
thread. Requests are written whole, then read back in 32-byte pages selected
with F0 <page>, so masters limited to short transfers work unchanged.
"""
import random
import struct

from .protocol import (BLOCK_ENTRIES, BLOCK_SIZE, CAP_STREAM, REG_FIFO_COUNT, REG_FIFO_DATA, Command, DeviceError, DeviceReset, PAGE_SIZE, ProtocolError,
                       decode_response, encode_request, parse_adc, parse_board, parse_info,
                       parse_status, response_size)

MAX_WRITE = 32


class Device:
    def __init__(self, bus, retries=3):
        self.bus = bus
        self.retries = retries
        self.epoch = 0
        self.capture = 0
        self.request_id = random.randrange(1, 0x10000)
        self.info = None

    def _exchange(self, wire):
        self.bus.i2c_write(list(wire))
        size = response_size(wire[3])
        data = bytearray()
        for page in range((size + PAGE_SIZE - 1) // PAGE_SIZE):
            count = min(PAGE_SIZE, size - len(data))
            part = bytes(self.bus.i2c_read([0xF0, page], count)["response"])
            if len(part) != count:
                raise ProtocolError("short page read")
            data += part
        return decode_response(data)

    def call(self, command, payload=b""):
        """One request. A retry resends identical bytes, so the device answers
        from its retry cache instead of executing twice."""
        self.request_id = self.request_id % 0xFFFF + 1
        wire = encode_request(command, self.request_id, self.epoch, self.capture, payload)
        if len(wire) > MAX_WRITE:
            raise ValueError("request exceeds the master's write limit")
        error = None
        for _ in range(self.retries):
            try:
                frame = self._exchange(wire)
            except ProtocolError as e:
                error = e
                continue
            if (frame.command, frame.request_id) != (command, self.request_id):
                error = ProtocolError("response does not match request")
                continue
            if command != Command.INFO and frame.epoch != self.epoch:
                raise DeviceReset("device epoch changed (reset?)")
            if frame.result == 4:
                raise DeviceReset("stale epoch or capture")
            if frame.result:
                raise DeviceError(command, frame.result)
            return frame
        raise ProtocolError("no valid response after %d attempts: %s" % (self.retries, error))

    def connect(self):
        nonce = random.randrange(1, 1 << 32)
        frame = self.call(Command.INFO, struct.pack("<I", nonce))
        if not frame.epoch:
            raise ProtocolError("device did not establish an epoch")
        self.epoch = frame.epoch
        self.capture = frame.capture
        self.info = parse_info(frame.payload)
        return self.info

    def status(self):
        return parse_status(self.call(Command.STATUS).payload)

    def tare(self):
        return struct.unpack("<i", self.call(Command.TARE).payload)[0]

    def set_adc(self, osr_code, gain_code):
        self.call(Command.SET_ADC, bytes([osr_code, gain_code]))

    def get_adc(self):
        return parse_adc(self.call(Command.GET_ADC).payload)

    def reset_errors(self):
        self.call(Command.RESET_ERRORS)

    def probe(self):
        p = self.call(Command.PROBE).payload
        raw, baseline, filtered, triggered, tared = struct.unpack_from("<iiiBB", p)
        return dict(raw=raw, baseline=baseline, filtered=filtered,
                    triggered=bool(triggered), tared=bool(tared))

    def arm(self, stream=True):
        """Arm the next capture ID; stream=True selects the register window."""
        if stream and not (self.info["capabilities"] & CAP_STREAM):
            raise ProtocolError("device firmware has no stream window")
        capture = self.capture + 1
        payload = struct.pack("<IB", capture, 1) if stream else struct.pack("<I", capture)
        frame = self.call(Command.ARM, payload)
        if frame.capture != capture:
            raise ProtocolError("ARM acknowledged a different capture")
        self.capture = capture
        return capture

    def start(self):
        self.call(Command.START)

    def stop(self):
        self.call(Command.STOP)

    def drain_stream(self, limit=2000):
        """Discard stream entries a previous session left unread, by reading
        the register window directly (as the MCU's reader would)."""
        discarded = 0
        while discarded < limit:
            count = self.bus.i2c_read([REG_FIFO_COUNT], 2)["response"]
            if (((count[0] & 0x1F) << 8) | count[1]) < BLOCK_SIZE:
                break
            self.bus.i2c_read([REG_FIFO_DATA], BLOCK_SIZE)
            discarded += BLOCK_ENTRIES
        return discarded

    def board(self):
        return parse_board(bytes(self.bus.i2c_read([0xF1], 32)["response"]))
