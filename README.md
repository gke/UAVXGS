# UAVXArmQ Flight Controller Firmware

Quaternion-attitude fork of the UAVX flight control firmware for STM32F4
boards (C, ARM Cortex-M4). Fixed-wing and multirotor: autonomous navigation,
altitude hold, soaring, wind estimation, and failsafe.

## Flashing

Flash the `.bin` using either:

- **UAVXGS** — built-in flasher (DFU or UART bootloader), or
- Any STM32 DFU tool (e.g. `dfu-util`, STM32CubeProgrammer).

## Documentation

See `wiki/docs/` for loading firmware, startup, architecture, and tuning.
