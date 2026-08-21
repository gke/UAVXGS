# UAVXArmQ Flight Controller Firmware

Quaternion-attitude fork of the UAVX flight control firmware for STM32F4
boards (C, ARM Cortex-M4). Fixed-wing and multirotor: autonomous navigation,
altitude hold, soaring, wind estimation, and failsafe.

## Releases

Prebuilt binaries in `obj/<TARGET>/`:

| Target | File |
|--------|------|
| FLYINGRCF4WINGMINI | `FLYINGRCF4WINGMINIQ_r0.bin` |
| SPEEDYF4WINGMINI   | `SPEEDYF4WINGMINIQ_r11.bin` |
| UAVXF4V3           | `UAVXF4V3Q_r17.bin` |

## Flashing

Flash the `.bin` using either:

- **UAVXGS** — built-in flasher (DFU or UART bootloader), or
- Any STM32 DFU tool (e.g. `dfu-util`, STM32CubeProgrammer).

## Building from source

Requires `arm-none-eabi-gcc` and `make`:

    make BOARD=FLYINGRCF4WINGMINI

## Documentation

See `wiki/docs/` for loading firmware, startup, architecture, and tuning.
