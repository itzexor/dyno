# Wire protocol v1

Dyno is an I2C slave at 7-bit address **0x2a** (`DYNO_I2C_ADDRESS`). The same
frames are accepted on the USB CDC interface. A master writes one complete
request, then reads the command's fixed-length response, using a repeated start.
The slave stretches SCL at that read address until the reply is ready.

## Frame

Each request is a 16-byte header, then the payload, then CRC16. Every integer is
little-endian, with no implicit alignment.

| Offset | Type | Meaning |
|---|---|---|
| 0 | 2 bytes | ASCII `DY` |
| 2 | u8 | protocol version = 1 |
| 3 | u8 | command |
| 4 | u16 | request ID, echoed |
| 6 | u8 | request = 0; response result |
| 7 | u8 | payload bytes, excluding padding and CRC |
| 8 | u32 | epoch; an INFO request may use zero |
| 12 | u32 | capture ID |
| 16 | bytes | payload (at most 110) |
| last 2 | u16 | CRC16-CCITT-FALSE: poly 1021, init FFFF, no reflection/xorout |

The CRC covers every preceding transmitted byte, including zero padding. Known
answer: `123456789` → `29B1`. Invalid frames do not execute.

Results: 0=OK, 1=bad frame, 2=bad state, 3=bad value, 4=stale, 5=unsupported.
Error responses use the same command-specific transfer length. Unused response
bytes before the CRC are zero. A request whose epoch does not match is stale,
except for INFO. START, STOP, READ and ACK are also stale when the capture ID
does not match the armed capture.

## Commands

| ID | Command | Request payload | Response payload | Read bytes |
|---|---|---|---|---|
| 1 | INFO | empty, or nonzero nonce:u32 | model[8], tick_hz:u32, capabilities:u32, fifo:u16, block_max:u8, version[5] | 42 |
| 2 | STATUS | empty | mode:u8, healthy:u8, triggered:u8, buffered:u32, overflow:u32, adc_errors:u32, faults:u32, next_sequence:u32 | 41 |
| 3 | GET_CONFIG | empty | trigger:u32, release:u32, timeout_ticks:u32, confirmations:u8 | 31 |
| 4 | SET_CONFIG | same as GET_CONFIG | empty | 18 |
| 5 | TARE | empty | baseline:i32 | 22 |
| 6 | ARM | next capture ID:u32, optional mode:u8 (0 = READ/ACK, 1 = stream) | empty; new capture ID in header | 18 |
| 7 | START | empty | empty | 18 |
| 8 | STOP | empty | empty | 18 |
| 9 | READ | optional minimum_count:u8 (default 1) | token:u32, count:u8, records[count] | 128 |
| 10 | ACK | token:u32 | empty | 18 |
| 11 | PROBE | empty | raw:i32, baseline:i32, filtered:i32, triggered:u8, tared:u8, crossing_tick:u32, crossing_sequence:u32 | 40 |
| 12 | TIME | empty | tick:u32 when serviced | 22 |
| 13 | RESET_ERRORS | empty | empty | 18 |
| 14 | SET_ADC | osr:u8 (0–7 → 128·2^n), gain:u8 (0–7 → 2^n) | empty | 18 |
| 15 | GET_ADC | empty | osr:u8, gain:u8, mode_reg:u16, clock_reg:u16, gain_reg:u16, rate_mhz:u32 | 30 |

A record is sequence:u32, event_tick:u32, raw:i24 (signed, LE), flags:u8,
12 bytes in all. Ticks are a free-running 1 MHz counter that wraps every
2^32 µs; event ticks are the measured DRDY time, never a reconstructed
fixed-rate time.

- Sample flags: bit0 saturated, bit1 ADC CRC error, bit2 missed/late DRDY.
  The raw value of a flagged record is a placeholder and should not be used
  as a measurement.
- Fault bits: overflow=1, ADC=2, stale=4. Counters and faults are sticky until
  RESET_ERRORS, which is allowed only while idle or stopped.
- `healthy` describes the latest sample and its age, not the absence of
  sticky faults.
- Models: `DYNO-ADC` (ADS131M02 build), `DYNO-SYN` (synthetic test signal),
  `DYNO-REF` (the portable engine's default identity). Capability bits:
  bit0 synthetic, bit1 ADC, bit2 paged I2C, bit3 stream window, bit4 SET_ADC.
- SET_CONFIG changes only the detector's thresholds, timeout and
  confirmations.
- SET_ADC sets the ADS131M02 oversampling ratio and PGA gain, verifies them by
  register readback, and is allowed only while idle or stopped and drained.
  Output rate = 4 MHz / OSR (8 MHz CLKIN), reported by GET_ADC in milli-Hz.
  The minimum OSR code is 3 (1024, 3906 SPS). The boot profile is OSR code 5
  (976.5625 SPS), gain 128. A change clears the tare and the idle window, and
  the next four conversions are discarded while the filter settles (they
  still consume sequence numbers). A failed readback sets the ADC fault.

## State

Modes: idle=0, armed=1, running=2, stopped=3.

- **Epoch.** The board starts with epoch zero. The first INFO must carry a
  nonzero random nonce, which becomes the epoch for this power cycle. Later
  INFO requests cannot change it. Nothing is written to flash; 32-bit
  collisions between power cycles are possible in principle.
- **ARM** requires no unread or pinned records, and a nonzero capture ID
  strictly greater than the previous one in this epoch.
- **START** requires armed and healthy. Recording begins with the next event.
- **STOP** prevents further appends and keeps unread records.
- **TARE** and **SET_CONFIG** are allowed while idle, or while stopped and
  fully drained. TARE averages the last 16–32 idle samples, and refuses if
  their spread exceeds half the release threshold. TARE is the only way the
  baseline changes; a capture never absorbs load into it.
- **READ** pins up to eight existing records. While running, a master may
  request minimum_count=8 to avoid partial blocks; after STOP, the final
  partial block is always readable. Empty reads consume nothing. Repeated
  reads return the same pinned records regardless of new samples or overflow.
- **ACK** with the pinned token releases those records. Repeating the last
  ACK is harmless. A wrong token or capture cannot discard records.
- **Overflow.** The 256-record FIFO drops the *newest* record, increments the
  overflow counter and sets a sticky fault. Sequence numbers are still
  consumed, so gaps are visible to the reader.

The engine caches the most recent valid request and response byte-for-byte.
A retry with the identical request ID and body gets the identical response
without executing again. Interleaving other requests evicts that cache, so the
bus must have one logical master.

## Stream window

A capture armed with mode 1 delivers samples through three registers instead
of READ/ACK. The register sequence matches a polling FIFO-accelerometer
reader, so a master that polls on its own timer (and timestamps its status
reads) can stream without any per-sample host round trip. Each access is a
one-byte register write followed by a repeated-start read. All three are
served entirely in the bus ISR and never wait for a framed reply.

| Register | Read | Value |
|---|---|---|
| `0x72` FIFO_COUNT | 2 bytes, big-endian | buffered bytes (6 × entries), or the time latch (below) |
| `0x74` FIFO_DATA | 48 bytes | up to 8 entries; bytes past the available entries read `FF` |
| `0x3A` INT_STATUS | 1 byte | bit4: ring overflow since the last INT_STATUS read; bit0: a whole block is buffered |

Entry, 6 bytes: raw:i24 LE, tick_lo:u16 LE (low 16 bits of the DRDY tick),
bits:u8 = flags (bits 0–2, as for records) | sequence low 4 bits (bits 3–6) |
pad (bit 7). Consecutive entries are consecutive conversions; a jump in the
4-bit sequence, or a tick gap, marks lost conversions.

- **Consumption.** Entries leave the ring only when the master clocks out the
  whole block (detected at STOP). A shorter read leaves them for the next one.
- **Overflow.** The ring holds 680 entries (about 0.7 s at 976 SPS). When it
  is full, the newest conversion is dropped, counted in STATUS overflow, and
  INT_STATUS bit4 is set once.
- **STOP** pads the ring with pad entries to a multiple of 8, so a
  block-reading master drains every sample. ARM requires the ring to be empty.
- **Time latch.** A FIFO_COUNT read that *immediately* follows an INT_STATUS
  read returns `(tick >> 4) & 0x1FFF` instead of the count: the 1 MHz tick
  at the read-address interrupt, in 16 µs units, wrapping every 131 ms. A
  master that timestamps that transaction thereby gets a direct pairing of
  its own clock with the device tick. Any other transaction between the two
  reads cancels the latch.
- STATUS `buffered` counts stream entries while a stream capture is active.
  READ and ACK return bad state.

## Paged reads

For masters limited to 32-byte transfers, send the command as an I2C write
with STOP. Then, for each page, write `F0 page` and read up to 32 bytes.
Pages 0–3 select offsets 0, 32, 64 and 96 of the frozen 128-byte reply. Read
only the command's declared response size, then validate its CRC and
identity. Page reads never execute a command or consume FIFO data. All
current commands fit in a 32-byte write.

Selector `F1` reads a 32-byte diagnostic register without the frame envelope or
CRC: `DYBD`, build_id:u32, uptime_us:u32, i2c_requests:u32, i2c_errors:u32,
adc_id:u16, adc_clock:u16, adc_gain:u16, adc_mode:u16, source:u8,
clock_source:u8, adc_ready:u8, flags:u8. Source is 0 for synthetic and 1 for
ADC. Clock source is 1 for HSE/PLL and 0 for the HSI48 fallback. Flags=3 means
startup and vector remapping completed.

## USB CDC

The USB interface accepts the same `DY` frames and returns the same responses.
A line-oriented text interface is also available:

- `board\n` returns the 32-byte diagnostic register.
- `checksums\n` returns `flash_crc32=xxxxxxxx build_id=xxxxxxxx`. The CRC32
  covers the whole 128 KiB flash, computed once at boot.
- Any other line returns the firmware banner.

USB and I2C requests share one engine and one retry cache. Use USB for
read-only inspection while an I2C master owns the capture.
