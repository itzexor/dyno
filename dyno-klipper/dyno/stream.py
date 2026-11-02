"""Streaming through the master MCU's built-in FIFO-accelerometer reader.

The EBB polls the device's stream window on its own timer and forwards
48-byte blocks as `sensor_bulk_data`, so there is no per-sample host round
trip. Its status query reads INT_STATUS then a timed FIFO_COUNT; the device
answers that FIFO_COUNT with its tick latched at the read, which pairs the
two clocks directly (see timebase.py).
"""
import numpy as np

from .protocol import (BLOCK_ENTRIES, BLOCK_SIZE, LATCH_SHIFT, ProtocolError, decode_entries)

QUERY_CMD = "query_mpu9250 oid=%c rest_ticks=%u"
STATUS_CMD = "query_mpu9250_status oid=%c"
STATUS_RESPONSE = ("sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
                   " buffered=%u possible_overflows=%hu")
CONFIG_CMD = "config_mpu9250 oid=%c i2c_oid=%c"


class BulkReader:
    """Klipper glue: configures the reader on the I2C bus's MCU and collects
    its messages. All methods run on the reactor."""

    def __init__(self, i2c):
        self.i2c = i2c
        self.mcu = i2c.get_mcu()
        self.oid = self.mcu.create_oid()
        self.available = False
        self.query_cmd = self.status_cmd = None
        self.messages = []
        self.statuses = []
        self.active = False
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        # Older or trimmed MCU builds lack the reader; streaming is then unavailable.
        if not self.mcu.try_lookup_command(CONFIG_CMD):
            return
        self.mcu.add_config_cmd("config_mpu9250 oid=%d i2c_oid=%d" % (self.oid, self.i2c.get_oid()))
        self.mcu.add_config_cmd("query_mpu9250 oid=%d rest_ticks=0" % (self.oid,), on_restart=True)
        cq = self.i2c.get_command_queue()
        self.query_cmd = self.mcu.lookup_command(QUERY_CMD, cq=cq)
        self.status_cmd = self.mcu.lookup_query_command(STATUS_CMD, STATUS_RESPONSE,
                                                        oid=self.oid, cq=cq)
        self.mcu.register_response(self._handle_data, "sensor_bulk_data", self.oid)
        self.available = True

    def _handle_data(self, params):
        # Kept even after stop(): a block sent before the final status can
        # arrive after it. start() clears the list for each session.
        self.messages.append((params["sequence"], bytes(params["data"])))

    def start(self, poll_seconds):
        if not self.available:
            raise ProtocolError("the I2C MCU's firmware has no bulk FIFO reader")
        self.messages = []
        self.statuses = []
        self.active = True
        rest_ticks = max(1, self.mcu.seconds_to_clock(poll_seconds))
        self.query_cmd.send([self.oid, rest_ticks])

    def poll_status(self):
        """Run one timed status query and record it with the message count."""
        params = self.status_cmd.send([self.oid])
        self.statuses.append((self.mcu.clock32_to_clock64(params["clock"]), params["query_ticks"],
                              params["buffered"], params["possible_overflows"],
                              params["next_sequence"], len(self.messages)))
        return params

    def stop(self):
        if self.available:
            self.query_cmd.send([self.oid, 0])
        self.active = False


class StreamCapture:
    """Decoded, loss-checked stream of one capture."""

    def __init__(self, entries, ticks, lost, problems, latches, gaps=(), fatal=()):
        self.entries = entries          # structured array (raw, tick16, flags, seq4, pad)
        self.ticks = ticks              # int64 device microseconds, unwrapped
        self.lost = lost                # conversions known to be missing
        self.problems = problems        # human-readable integrity findings
        self.latches = latches          # (device_tick, master_clock, query_ticks) rows
        self.gaps = list(gaps)          # (tick_from, tick_to): no valid data in between
        self.fatal = list(fatal)        # findings that invalidate the whole timeline


def assemble(messages, statuses, period_us, device_overflow=None, latch_fraction=0.58):
    """Turn bulk messages and status records into a StreamCapture.

    messages: [(sequence16, 48-byte block)] in arrival order.
    statuses: [(clock64, query_ticks, latch13, overflows, next_sequence, n_messages)].
    period_us: nominal conversion period, used only to size gaps.
    device_overflow: the device's STATUS overflow count for this capture. A
    gap longer than the 16-bit tick window (65 ms) cannot be sized from the
    stream alone; when the device dropped more than the stream accounts for,
    the timeline after the gap is unreliable, which is fatal.

    Every loss is reported in `problems` and located in `gaps` (device-tick
    intervals, open-ended at the start or end), so a caller can decide
    whether it touches the data it uses.
    """
    problems, gaps, fatal = [], [], []
    lost = 0
    if not messages:
        return StreamCapture(decode_entries(b""), np.zeros(0, np.int64), 0, ["no data"], [],
                             fatal=["no data"])
    raw_seqs = np.array([m[0] for m in messages], dtype=np.int64)
    seqs = np.concatenate(([raw_seqs[0]], (np.diff(raw_seqs) & 0xFFFF))).cumsum()
    seq_gaps = np.diff(seqs) - 1
    leading = int(seqs[0]) > 0
    if leading:
        problems.append("first block has sequence %d" % seqs[0])
        lost += int(seqs[0]) * BLOCK_ENTRIES
    if seq_gaps.any():
        missing = int(seq_gaps.sum())
        lost += missing * BLOCK_ENTRIES
        problems.append("%d bulk blocks lost in transit" % missing)
    # Each status reports how many blocks the reader had sent when it ran
    # (16 bits; unwrapped near the host's receive count at that time). Those
    # are the exact latch bounds, and the last one exposes blocks that were
    # sent but never arrived, which in-stream gaps cannot show at the end.
    sent = [s[5] + ((s[4] - s[5] + 0x8000) & 0xFFFF) - 0x8000 for s in statuses]
    trailing = max(sent[-1] - int(seqs[-1]) - 1, 0) if sent else 0
    if trailing:
        lost += trailing * BLOCK_ENTRIES
        problems.append("%d bulk blocks sent but never received" % trailing)
    if any(len(m[1]) != BLOCK_SIZE for m in messages):
        raise ProtocolError("bulk block with unexpected size")
    entries = decode_entries(b"".join(m[1] for m in messages))
    # Latch bound: the last entry of the last block sent before each status
    # (the newest received block with a sequence below the sent count).
    position = np.searchsorted(seqs, sent) - 1
    delivered_before = [int(i) * BLOCK_ENTRIES + BLOCK_ENTRIES - 1 if i >= 0 else -1
                        for i in position]
    keep = ~entries["pad"]
    if (~keep).any() and keep[np.argmax(~keep):].any():
        problems.append("pad entries before the end of the stream")
        fatal.append(problems[-1])
    index_map = np.cumsum(keep) - 1
    entries_all = entries
    entries = entries[keep]
    if len(entries) == 0:
        return StreamCapture(entries, np.zeros(0, np.int64), lost, problems + ["no samples"], [],
                             fatal=fatal + ["no samples"])

    dt16 = np.diff(entries["tick16"].astype(np.int64)) & 0xFFFF
    ticks = np.concatenate(([int(entries["tick16"][0])], dt16)).cumsum()
    if leading:
        gaps.append((-np.inf, int(ticks[0])))
    if trailing:
        gaps.append((int(ticks[-1]), np.inf))
    seq_step = np.diff(entries["seq4"].astype(np.int64)) & 15
    tick_steps = np.rint(dt16 / period_us).astype(np.int64)
    bad = (seq_step != 1) | (tick_steps != 1)
    if bad.any():
        # Prefer the tick spacing for the size of a gap; the 4-bit sequence
        # only disambiguates short ones. In-transit block losses show up here too.
        sizes = np.maximum(tick_steps[bad] - 1, (seq_step[bad] - 1) & 15)
        in_stream = int(sizes.sum())
        lost += max(0, in_stream - int(seq_gaps.sum()) * BLOCK_ENTRIES)
        problems.append("%d gaps in the conversion sequence (%d conversions)"
                        % (int(bad.sum()), in_stream))
        for i in np.nonzero(bad)[0]:
            gaps.append((int(ticks[i]), int(ticks[i + 1])))
    if device_overflow is not None and device_overflow > lost:
        problems.append("device dropped %d conversions but the stream accounts for %d: "
                        "timing after the gap is unreliable" % (device_overflow, lost))
        fatal.append(problems[-1])
        lost = device_overflow
    flagged = np.nonzero(entries["flags"] != 0)[0]
    if len(flagged):
        problems.append("%d flagged conversions" % len(flagged))
        gaps += [(int(ticks[i]) - period_us, int(ticks[i]) + period_us) for i in flagged]
    overflows = [s[3] for s in statuses]
    if overflows and overflows[-1]:
        problems.append("device reported %d ring overflows" % overflows[-1])

    latches = []
    for status, bound_all in zip(statuses, delivered_before):
        if bound_all < 0:
            continue
        bound_all = min(bound_all, len(entries_all) - 1)
        bound = int(ticks[max(0, index_map[bound_all])])
        # The latch carries tick bits 4..16; the unwrapped timeline only shares
        # the low 16 bits with the device tick, so match bits 4..15.
        latch12 = status[2] & 0xFFF
        k = (latch12 - ((bound >> LATCH_SHIFT) & 0xFFF)) & 0xFFF
        device_tick = ((bound >> LATCH_SHIFT) << LATCH_SHIFT) + (k << LATCH_SHIFT) + 8
        latches.append((device_tick, status[0] + latch_fraction * status[1], status[1]))
    return StreamCapture(entries, ticks, lost, problems, np.array(latches, dtype=np.float64),
                         gaps, fatal)
