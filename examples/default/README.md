# Example for `ds18x20` driver

## What it does

The example scans the 1-Wire bus on `CONFIG_EXAMPLE_ONEWIRE_GPIO` with
`ds18x20_scan_devices()`, takes the first sensor of a supported family, prints
its address and family, and then reads the temperature once a second. The
address of the sensor is not configured anywhere, so the example works with
any sensor that happens to be on the bus.

When the sensor stops answering, for example because it was unplugged, the
example scans the bus again, so a sensor can be replaced on the fly.

The example works with both 1-Wire backends, and the default one is the
bit-banging driver of `esp-idf-lib/onewire`. Pick the RMT backend of
`espressif/onewire_bus` in `menuconfig` under `Component config` → `DS18x20`
→ `1-Wire backend` if the application needs the bus without disabling
interrupts. See the [main README](../../README.md#1-wire-backends) for the
differences between the backends.

The internal (~47k) pull-ups of the ESP do appear to work, at least for simple
setups (one or two sensors connected with short leads), but do not technically
meet the pull-up requirements from the ds18x20 datasheet and may not always be
reliable. For a real application, a proper 4.7k external pull-up resistor is
recommended instead!

## Wiring

Connect `DQ` pin to `CONFIG_EXAMPLE_ONEWIRE_GPIO`.

| Name | Description | Defaults                                                                          |
|------|-------------|-----------------------------------------------------------------------------------|
| `CONFIG_EXAMPLE_ONEWIRE_GPIO` | GPIO Number of 1-Wire bus, or `DQ` | "17" for `esp32`, `esp32s2`, and `esp32s3`, "4" for `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32c61`, `esp32h2`, and `esp32p4` |
