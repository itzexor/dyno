# dyno-klipper

The Dyno Klipper host extra and its example printer configuration. See the
[project README](../README.md) for requirements, first use, G-code commands,
the estimation method, and recorded data.

## Install

The host extra requires NumPy in the Python environment that runs klippy.
SciPy is optional and is used when available. For a typical `~/klippy-env`
installation:

```sh
~/klippy-env/bin/python -m pip install numpy
```

With this repository at `~/dyno-repo` and Klipper at `~/klipper`, install the
`dyno/` Python package as a Klipper extra:

```sh
ln -s ~/dyno-repo/dyno-klipper/dyno ~/klipper/klippy/extras/dyno
```

Adjust both paths for your installation, including Kalico checkouts. Keep
the source directory in place while the symlink is installed.

Copy [dyno.cfg.example](dyno.cfg.example) into your printer configuration
directory as `dyno.cfg`, then add this to `printer.cfg`:

```ini
[include dyno.cfg]
```

Alternatively, copy the example's `[dyno]` section directly into `printer.cfg`.
Keep only one `[dyno]` section and remove any older sensor section that drives
the same I2C address. Restart Klipper after installing the package and config.

## Printer configuration

Edit these options before restarting:

| Option | What to set |
|---|---|
| `i2c_mcu` | Your toolhead MCU's configured name; the example uses `EBB` |
| `i2c_bus` | That MCU's hardware I2C bus; `i2c1_PA9_PA10` in the example is printer-specific |
| `i2c_address` | Decimal `42` (`0x2a`) for the Dyno |
| `i2c_speed` | `400000`; verify the connection using the root README's bring-up procedure |
| `park_position` | Optional `x, y, z` in mm for the calibration helper, with `z >= min_z` and clearance for extrusion and Y wobble |
| `data_directory` | Writable archive directory on the host; default `~/printer_data/dyno` |

The commented options in [dyno.cfg.example](dyno.cfg.example) document ADC
settings, extrusion profile, filament budget, minimum Z, heater handling,
and acceptance limits. `sample_rate` selects the nearest supported rate;
`gain` must be a power of two from 1 through 128. `soak_time` and `cooldown`
set the calibration helper's default soak duration and heater-off behavior.

### Material temperature ranges

`DYNO_PA_CALIBRATE` requires an explicit `MATERIAL` and `TEMP`. The following
built-in ranges validate the supplied temperature; they do not select it:

| Material | Range (C) |
|---|---|
| PLA | 180–240 |
| PETG | 210–260 |
| PCTG, ABS | 220–270 |
| ASA | 230–280 |
| HIPS | 220–260 |
| PC | 250–310 |
| PA, NYLON | 240–300 |
| TPU, PP | 200–250 |
| PVA | 180–220 |
| PPA | 270–330 |
| PET | 250–300 |

A variant resolves to its leading letters (`PETG-CF` → `PETG`, `PA12` → `PA`).
Add or override a range in `[dyno]` with `temp_range_<material>: min, max`,
for example `temp_range_pekk: 340, 380`.

## Tests

Requirements: Python 3 with NumPy, and a host C compiler (`cc`). SciPy is
optional. Run from this directory (`dyno-klipper/`):

```sh
DYNO_FIRMWARE=../dyno-firmware python3 -m unittest discover -s tests -t .
```

Set `DYNO_FIRMWARE` explicitly for this repository layout: the test helper's
legacy default looks for `../dyno`. Without the override, firmware-backed
tests are skipped if that directory is absent.

The tests compile the sibling firmware's portable engine and drive it through
the same register transactions as the MCU's reader. A simulated printer
(virtual time, toolhead, trapq, a melt plant with known τ, and crystal offsets)
exercises the `DYNO_PA` routine, cancellation, weak-signal rejection, lost and
stalled delivery, and apply/rollback.

## Source layout

| Path | Contents |
|---|---|
| `dyno/` | Klipper extra, device control, stream decoding, clock mapping, motion planning, and PA analysis |
| `dyno.cfg.example` | Example `[dyno]` configuration with optional settings |
| `tests/` | Unit tests, firmware-backed data-path tests, and printer simulation |
