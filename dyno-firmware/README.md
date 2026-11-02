# dyno-firmware

Board firmware for Dyno's STM32F072 + ADS131M02 acquisition hardware. See the
[project README](../README.md) for the system overview, printer workflow,
commands, and recorded data, and [docs/protocol.md](docs/protocol.md) for the
wire protocol.

## Firmware interfaces

- A 256-record FIFO with explicit READ/ACK delivery: READ pins a block and ACK
  releases it, allowing retries without dropping or duplicating records.
- A separate stream delivery mode with packed 6-byte entries in 48-byte
  blocks and a bus-time latch for clock pairing with the host.
- I2C slave with clock stretching, 32-byte paged framed responses for masters
  with short transfer limits, and bus-stall recovery.
- USB CDC for bench diagnostics using the same framed protocol.
- Explicit tare and a hysteresis threshold detector. The detector is for
  diagnostics only and is not validated as a safety endstop.
- Independent watchdog and an HSI48 fallback if the 8 MHz crystal fails.

## Hardware

| Function | Pins / setting |
|---|---|
| MCU | STM32F072CBT6, Cortex-M0, 128 KiB flash, 16 KiB SRAM |
| Clock | 8 MHz HSE → PLL 48 MHz; HSI48 + CRS fallback |
| ADC | ADS131M02 on SPI1: PB3 SCK, PB4 MISO, PB5 MOSI, PA15 CS, PB6 DRDY (falling edge, EXTI6) |
| ADC clock | PA3, TIM15 CH2, 8 MHz |
| ADC profile | Channel 0; boot default gain 128, OSR 4096 (976.5625 SPS), CLOCK=0x0116; MODE=0x0510; runtime SET_ADC, approximately 244–3906 SPS |
| I2C | I2C2 slave, PB13 SCL / PB14 SDA, AF5 open drain, address 0x2a |
| USB | PA11/PA12, full-speed CDC ACM (0483:5740) |
| LED | PC13, toggles every 500 ms |

## Memory map

| Region | Address | Notes |
|---|---|---|
| Resident bootloader | 0x08000000–0x08001FFF | Not touched by this firmware |
| Application | 0x08002000–0x0800F7FF | Linker and image checks enforce the limit |
| Settings page | 0x0800F800– | Preserved; never written |
| RAM | 0x200000C0–0x20003FF7 | First 192 bytes hold the remapped vector table |

The Cortex-M0 has no VTOR, so startup copies the 48-entry vector table to SRAM
and maps SRAM at address zero. The linker requires at least 2 KiB of stack
headroom.

## Build

Requirements: `arm-none-eabi-gcc` and `arm-none-eabi-objcopy`, GNU make,
Python 3 (build ID and image checks), and a host C compiler for the tests.
Firmware dependencies are vendored; see [vendor/README.md](vendor/README.md).

Run from this directory (`dyno-firmware/`):

```sh
make test          # native engine, protocol, stream and I2C watchdog tests
make firmware      # build/adc/dyno.bin and build/synthetic/dyno.bin
make image SOURCE=adc  # build only the ADC image
make sanitize      # protocol and stream tests under ASan/UBSan
```

`SOURCE=adc` reads the ADS131M02. `SOURCE=synthetic` generates a sawtooth at
the configured rate with no ADC, for testing the link. Each image gets a
build ID derived from the SHA-256 of its sources, and `image.json` records
the image size, SHA-256, CRC32, and vector-table checks.

## Flashing

Requirements: `dfu-util` and access to the board's ROM DFU mode.

The image is linked for **0x08002000**. Flash only the application range with
ROM DFU (BOOT0 held at reset), and leave the bootloader, settings page and
option bytes untouched:

```sh
dfu-util -a 0 -s 0x08002000 -D build/adc/dyno.bin
```

Before writing anything, back up the full flash and option bytes, and verify
the written range by reading it back.

## Source layout

| Path | Contents |
|---|---|
| `src/core/` | Portable engine (`dyno.c`) and paged link adapter; no MMIO |
| `src/drivers/` | ADS131M02 SPI/DRDY, I2C slave + watchdog, USB CDC |
| `src/board/` | Clocks, vector remap, main loop, startup, linker script |
| `tests/` | Native tests for the engine, protocol, stream window and I2C watchdog |
| `tools/image.py` | Build ID and image boundary checks |
| `vendor/` | TinyUSB 0.18.0 and CMSIS/STM32F0 headers, with licenses |
