# Vendored build dependencies

The firmware builds offline against these preserved source files:

- TinyUSB **0.18.0**, official tagged GitHub/codeload archive. The exact URL,
  archive SHA-256 and per-file hashes are in `tinyusb/provenance.json`.
  Device/CDC/STM32 FSDEV support is included; other device classes are omitted.
  The original MIT license and per-file notices are retained.
- STM32F072 device and CMSIS-Core headers, taken from the `lib/` tree of the
  Katapult repository at the commit recorded in `cmsis/provenance.json`, with
  per-file hashes. ST's BSD license notices remain in the device headers; Arm's
  Apache-2.0 notices remain in the core/compiler headers, accompanied by
  `cmsis/LICENSE-APACHE`.

The firmware uses register definitions and the TinyUSB device stack only. No
vendor HAL implementation is linked into the image.
