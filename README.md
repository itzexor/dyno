# Dyno

Dyno estimates pressure advance (PA) on a Kalico/Klipper printer by measuring
nozzle force while extruding in air. It combines replacement firmware for the
[Mellow FLY-ALPS pressure sensor and controller board](https://mellow.klipper.cn/en/docs/ProductDoc/ExtensionBoard/fly-alps/)
(STM32F072 + ADS131M02) with a Klipper host extra that captures the force signal,
runs the experiment, and reports a recommended `ADVANCE` with its uncertainty,
or a specific reason it could not produce one.

Dyno is an independent project and is not affiliated with or endorsed by Mellow.

The application firmware is an independent implementation, not a port or patch
of Mellow's stock ALPS application firmware. Board schematics and stock-firmware
analysis informed hardware compatibility. Third-party components, including
TinyUSB and CMSIS/STM32 headers, are credited in the
[vendored dependencies](dyno-firmware/vendor/README.md).

With the default settings, measurement takes a little over a minute and uses
about 150 mm of filament, excluding heating and soaking. The experiment
temporarily varies PA, then restores the previous PA, smooth time, and
acceleration on completion, error, or cancellation. Applying the recommendation
is a separate, explicit choice.

## Repository layout

| Path | Contents |
|---|---|
| [dyno-firmware/](dyno-firmware/README.md) | Board firmware, hardware pinout, build and flashing instructions, and native C tests |
| [dyno-klipper/](dyno-klipper/README.md) | Klipper host extra, installation instructions, example printer configuration, and simulation tests |
| [dyno-firmware/docs/protocol.md](dyno-firmware/docs/protocol.md) | Shared wire protocol, sample formats, stream registers, and device states |

## Requirements

- The Mellow FLY-ALPS pressure sensor and controller board (STM32F072 +
  ADS131M02), mounted to measure nozzle force. Measurements are raw signed ADC
  codes; force units are not required for the PA fit.
- Dyno firmware 0.2.0 or later, with the stream window and runtime ADC profile
  support (`SET_ADC`).
- Kalico/Klipper with hardware I2C on the toolhead MCU. Its firmware must
  include the FIFO-sensor reader (`CONFIG_WANT_MPU9250=y`);
  `DYNO_STATUS` reports whether it is available.
- An I2C connection to the Dyno at 7-bit address `0x2a` (decimal `42`). Use
  400 kHz: at the default 976.5625 SPS, the stream uses about 74 kbit/s,
  roughly a fifth of that bus capacity but three quarters of a 100 kHz bus.

The host extra uses the MCU's existing FIFO reader, so it does not require a
custom toolhead-MCU implementation. Python dependencies and printer-specific
configuration are covered in the [host installation guide](dyno-klipper/README.md).

> **Bus errors are fatal on this firmware generation.** Kalico/Klipper shut
> the MCU down on an I2C NACK or a 5 ms transaction timeout. Verify a new cable
> or bus speed with `DYNO_STREAM_TEST` before printing.

## Setup and first use

1. Build and flash the board using the
   [firmware instructions](dyno-firmware/README.md#build). Preserve its resident
   bootloader, settings page, and option bytes.
2. Install the [host extra](dyno-klipper/README.md#install), adapt the example
   `[dyno]` configuration to your printer, and restart Klipper.
3. Check communication without motion:

   ```gcode
   DYNO_STATUS
   DYNO_STREAM_TEST SECONDS=10
   ```

   Confirm the ADC model/version and bulk reader availability. At the default
   profile, expect about 976.6 SPS, zero lost samples, no integrity problems,
   and a stable clock fit. The report also shows spacing jitter and noise.
4. With the nozzle cold, push and release it by hand during another stream
   test to check signal polarity and range. Tare with the nozzle unloaded and
   steady using `DYNO_TARE`.
5. Choose a material and temperature for the measurement. The helper can home
   if needed, park, heat, soak, and run the experiment. For example, for PLA
   at a chosen temperature of 210 C:

   ```gcode
   DYNO_PA_CALIBRATE MATERIAL=PLA TEMP=210 APPLY=0
   ```

   Configure `park_position` for a clear location on your printer. Without
   one, the helper keeps the current X/Y position and raises Z to `min_z` if
   needed. It switches the heater off afterwards by default, including after
   an error or cancellation if it set the heater.

   To prepare the printer yourself, home X/Y/Z, raise the nozzle to
   `Z >= min_z` (default 10 mm), leave room for the small Y wobble, and bring
   the hotend to its target temperature. Then run:

   ```gcode
   DYNO_PA MATERIAL=PLA APPLY=0
   ```

6. Repeat two or three times under the same conditions. Accepted estimates
   should agree within their reported uncertainty. Use `DYNO_PA_APPLY` to
   try the last accepted result and print-test it against your current value
   before saving it permanently. `DYNO_PA_ROLLBACK` restores the PA and smooth
   time in effect before the apply command.

Anything the experiment could not restore is reported as `NOT RESTORED`.

## Commands

| Command | Effect |
|---|---|
| `DYNO_STATUS` | Report device identity, build, capture state, faults, ADC profile, bus counters, and bulk reader availability |
| `DYNO_TARE` | Tare the unloaded, steady load cell |
| `DYNO_STREAM_TEST [SECONDS=3]` | Stream without motion; report rate, loss, noise, and clock-fit quality |
| `DYNO_PA [MATERIAL=] [APPLY=0]` | Run the experiment and report; `APPLY=1` also applies an accepted result |
| `DYNO_PA_CALIBRATE MATERIAL= TEMP= [SOAK=30] [HOME=1] [PARK=1] [COOLDOWN=1]` | Validate the chosen temperature against the material's configured range, prepare the printer, and run `DYNO_PA`; accepts its overrides too |
| `DYNO_PA_APPLY [SAVE=0]` | Apply the last accepted result; `SAVE=1` also stages PA for a subsequent `SAVE_CONFIG` |
| `DYNO_PA_ROLLBACK` | Restore the PA and smooth time from before the last apply; does not undo a staged or saved configuration value |
| `DYNO_CANCEL` | Request cancellation; use the `dyno/cancel` API endpoint during a running G-code command |

`DYNO_PA` also accepts `REPEATS`, `FLOW_LOW`, `FLOW_HIGH`, `TIME_LOW`,
`TIME_HIGH`, `CYCLES`, `LEAD`, `WOBBLE`, `ACCEL`, `GAP`, and `SMOOTH_TIME`.
Flow is in mm³/s, time in seconds, wobble in mm, and acceleration in mm/s².
`MATERIAL` labels the run; temperature-range validation is performed by
`DYNO_PA_CALIBRATE`. See the
[example configuration](dyno-klipper/dyno.cfg.example) for defaults.

A G-code command cannot interrupt another running G-code command. Request
cancellation through the `dyno/cancel` endpoint, for example via Moonraker's
API. During measurement it takes effect at the next block boundary, about
4 s with the default profile. Heating and soaking can take longer to return
to the cancellation check.

## How it works

### Acquisition and timing

The board reads conversions in the ADC's DRDY interrupt and records raw
24-bit codes, measured timestamps from a 1 MHz timer, and conversion sequence
information. The default profile is 976.5625 SPS at gain 128; the sample rate
and gain can be changed at runtime. ADC errors, late conversions, overflow,
and stale data are reported explicitly.

The toolhead MCU's built-in FIFO-sensor reader (`config_mpu9250`) polls a
register window on its own timer. It sends sequenced `sensor_bulk_data`
blocks containing eight packed 6-byte entries to the host, avoiding a host
round trip for every sample. Device control uses a framed, CRC-checked
protocol on the same bus.

The reader's periodic status query reads `INT_STATUS`, then a timed
`FIFO_COUNT`. The board answers that count read with its own tick, latched
in the interrupt. A robust linear fit pairs device time with the toolhead
MCU clock and maps samples to print time. Simulation exercises this mapping
with crystal offsets; check its residual on the installed printer with
`DYNO_STREAM_TEST`.

### Excitation and estimation

With the toolhead parked, each block alternates slow and fast extrusion
legs. Each leg includes a 0.05 mm Y wobble by default, reversing on the next
leg, because the targeted kinematics apply PA only to extrusion with X/Y
travel. PA changes occur while the toolhead is idle. The host reads each
block's commanded extruder trajectory from the trapq for analysis.

After priming, a PA=0 warm-up block is recorded but excluded from fitting.
The run starts and ends its analysed sequence with PA=0 identification
blocks to detect drift. Two estimates must agree:

- **Model:** fit a first-order melt model jointly to the blocks. Stored
  compression is `s = P − LP_τ(P)`, where `P` is commanded stepper position.
  Force is modelled as `a + c·t + b·s`, passed through the ADC's sinc3 filter,
  with a fitted latency. The fit estimates the time constant `τ`, the PA
  value that makes this model's outflow track commanded flow, and reports
  curvature-based and leave-one-block-out uncertainty.
- **Sweep:** randomize and repeat blocks at PA values bracketing `τ`. Score
  how closely force follows smoothed nominal flow, interpolate the minimum,
  and estimate its uncertainty from per-cycle scatter.

The result is rejected with a reason if data loss or flagged samples affect
analysed blocks, the timeline is unreliable, the clock fit is poor, the signal
is weak, uncertainty or drift is too high, either estimate reaches its search
boundary, or the estimates disagree beyond the agreement tolerance. Loss
outside analysed blocks is reported as a note when timing remains reliable.
The configurable acceptance limits include `min_snr`,
`max_relative_uncertainty`, and `max_drift`.

## Recorded data

Runs are stored under `data_directory`, which defaults to
`~/printer_data/dyno`, in separate `<kind>-<timestamp>/` directories:

- `run.json` contains device and ADC identity, settings, and findings. PA runs
  also include the printer snapshot, block list, and either analysis results
  (model, sweep, drift, clock fit, decision, and reasons) or abort errors.
- `data.npz` contains raw codes, flags, unwrapped device ticks, clock latches,
  and status records. Completed PA runs also contain print times and each
  block's extruder trapq segments. Aborted PA measurements preserve the
  undecoded stream and available segments.

Build and native-test commands live in the
[firmware README](dyno-firmware/README.md#build); host and end-to-end
simulation tests are documented in the
[host README](dyno-klipper/README.md#tests).
