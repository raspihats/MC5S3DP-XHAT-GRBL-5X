# MC5S3DP-XHAT-GRBL-5X

Firmware for the **MC5S3DP-XHAT** 5-axis motion control board. It is a port of
[grbl-Mega-5X](https://github.com/fra589/grbl-Mega-5X) (Grbl v1.1f with 4th and 5th
axis support) from the Arduino Mega2560 to the **STM32F411RCT6** (LQFP64,
256 KB flash, 128 KB RAM), using the STM32 LL drivers.

- 5 axes: X, Y, Z (linear) and A, B (rotary)
- 25 MHz crystal, 100 MHz system clock
- Serial: USART1 at 115200 baud, 8N1
- Settings are stored with EEPROM emulation in the STM32's internal flash (sectors 2 and 3).
  The board has no EEPROM chip; "EEPROM" in the Grbl code refers to this emulation.

## Building and flashing

Requires `arm-none-eabi-gcc` (tested with 10.3-2021.07).

```sh
make            # build/grbl.elf, grbl.hex, grbl.bin, grbl.map
make flash      # program through ST-Link with OpenOCD
make stflash    # program through ST-Link with st-flash
make clean
```

Flash the `.hex` or `.elf`, not the `.bin`: the `.bin` covers the EEPROM emulation sectors
(`0x08008000`-`0x0800FFFF`) with padding and would erase the stored `$` settings.

Flash layout: vector table at `0x08000000`, EEPROM emulation at `0x08008000`, code at `0x08010000`.

## Pin map

The CubeMX project (`MC5S3DP-XHAT-GRBL-5X.ioc`) and `Inc/main.h` name every pin;
`Grbl/cpu_map.h` assigns those names to Grbl functions.

### Stepper drivers

| Axis | Step | Direction | Enable |
|------|------|-----------|--------|
| X | PC14 `X_STP` | PC15 `X_DIR` | PC13 `X_EN` |
| Y | PA4 `Y_STP` | PA6 `Y_DIR` | PA3 `Y_EN` |
| Z | PC4 `Z_STP` | PC5 `Z_DIR` | PA7 `Z_EN` |
| A (4th) | PB1 `A_STP` | PB2 `A_DIR` | PB0 `A_EN` |
| B (5th) | PB12 `B_STP` | PB15 `B_DIR` | PB10 `B_EN` |

Enable outputs are active low by default (invert with `$4`).

### Limit switches

| Axis | Min | Max |
|------|-----|-----|
| X | PC12 `X_MIN` | PB7 `X_MAX` |
| Y | PD2 `Y_MIN` | PB8 `Y_MAX` |
| Z | PB3 `Z_MIN` | PB9 `Z_MAX` |
| A | PB4 `A_MIN` | PC0 `A_MAX` |
| B | PB6 `B_MIN` | PC1 `B_MAX` |

All limit inputs have internal pull-ups (switch to GND). Hard limits are disabled in
this build (`DISABLE_HW_LIMITS` in `Grbl/cpu_map.h`); the switches are used for homing.

### Digital inputs

| Pin | Net | Function |
|-----|-----|----------|
| PB13 | `DI_0` | Reset |
| PB14 | `DI_1` | Feed hold |
| PA15 | `DI_2` | Cycle start |
| PC10 | `DI_3` | Safety door |
| PC11 | `DI_4` | Probe |

All with internal pull-ups.

### Digital outputs

| Pin | Net | Function |
|-----|-----|----------|
| PA11 | `DQ_0` | Coolant flood (M8) |
| PA8 | `DQ_1` | Coolant mist (M7) |
| PC9 | `DQ_2` | Spindle PWM: TIM3_CH4, 24.4 kHz, 12-bit (0-4095) |
| PC8 | `DQ_3` | Spindle enable |
| PC7 | `DQ_4` | Spindle direction |
| PC6 | `DQ_5` | Not used |

### Communication and system

| Pin | Function |
|-----|----------|
| PA9 | USART1 TX |
| PA10 | USART1 RX |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PH0 / PH1 | 25 MHz HSE crystal |

### Board pins not used by this firmware

| Pin | Net | Configured as |
|-----|-----|---------------|
| PA0 | `THERM_0` | ADC1_IN0 (initialized, never read) |
| PA1 | `THERM_1` | ADC1_IN1 (initialized, never read) |
| PA2 | `THERM_2` | ADC1_IN2 (initialized, never read) |
| PA5 | `FAULT` | Input |
| PA12 | `ST_LED` | Output (status LED) |
| PC6 | `DQ_5` | Output |

---

# Upstream grbl-Mega-5X README

![GitHub Logo](https://github.com/fra589/grbl-Mega-5X/blob/edge/doc/images/Mega-5X-logo.svg)

***

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. This version of Grbl runs on an Arduino Mega2560 only.

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 30kHz of stable, jitter free control pulses.

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 24 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/fra589/grbl-Mega-5X/blob/edge/COPYING): Grbl is free software, released under the GPLv3 license.

* For more information and help, check out our **[Wiki pages!](https://github.com/gnea/grbl/wiki)** If you find that the information is out-dated, please to help us keep it updated by editing it or notifying our community! Thanks!

* Lead Developer: Gauthier Brière (France) aka @fra589

* Built on the wonderful Grbl v1.1f (2017) firmware originally written by Simen Svale Skogsrud (Norway) and maintained by Sungeun "Sonny" Jeon, Ph.D. (USA) aka @chamnit

***

##Update Summary for v1.1
- **IMPORTANT:** Your EEPROM will be wiped and restored with new settings. This is due to the addition of two new spindle speed '$' settings.

- **Real-time Overrides** : Alters the machine running state immediately with feed, rapid, spindle speed, spindle stop, and coolant toggle controls. This awesome new feature is common only on industrial machines, often used to optimize speeds and feeds while a job is running. Most hobby CNC's try to mimic this behavior, but usually have large amounts of lag. Grbl executes overrides in realtime and within tens of milliseconds.

- **Jogging Mode** : The new jogging commands are independent of the g-code parser, so that the parser state doesn't get altered and cause a potential crash if not restored properly. Documentation is included on how this works and how it can be used to control your machine via a joystick or rotary dial with a low-latency, satisfying response.

- **Laser Mode** : The new "laser" mode will cause Grbl to move continuously through consecutive G1, G2, and G3 commands with spindle speed changes. When "laser" mode is disabled, Grbl will instead come to a stop to ensure a spindle comes up to speed properly. Spindle speed overrides also work with laser mode so you can tweak the laser power, if you need to during the job. Switch between "laser" mode and "normal" mode via a `$` setting.

	- **Dynamic Laser Power Scaling with Speed** : If your machine has low accelerations, Grbl will automagically scale the laser power based on how fast Grbl is traveling, so you won't have burnt corners when your CNC has to make a turn! Enabled by the `M4` spindle CCW command when laser mode is enabled!

- **Sleep Mode** : Grbl may now be put to "sleep" via a `$SLP` command. This will disable everything, including the stepper drivers. Nice to have when you are leaving your machine unattended and want to power down everything automatically. Only a reset exits the sleep state.

- **Significant Interface Improvements**: Tweaked to increase overall performance, include lots more real-time data, and to simplify maintaining and writing GUIs. Based on direct feedback from multiple GUI developers and bench performance testing. _NOTE: GUIs need to specifically update their code to be compatible with v1.1 and later._

	- **New Status Reports**: To account for the additional override data, status reports have been tweaked to cram more data into it, while still being smaller than before. Documentation is included, outlining how it has been changed. 
	- **Improved Error/Alarm Feedback** : All Grbl error and alarm messages have been changed to providing a code. Each code is associated with a specific problem, so users will know exactly what is wrong without having to guess. Documentation and an easy to parse CSV is included in the repo.
	- **Extended-ASCII realtime commands** : All overrides and future real-time commands are defined in the extended-ASCII character space. Unfortunately not easily type-able on a keyboard, but helps prevent accidental commands from a g-code file having these characters and gives lots of space for future expansion.
	- **Message Prefixes** : Every message type from Grbl has a unique prefix to help GUIs immediately determine what the message is and parse it accordingly without having to know context. The prior interface had several instances of GUIs having to figure out the meaning of a message, which made everything more complicated than it needed to be.

- New OEM specific features, such as safety door parking, single configuration file build option, EEPROM restrictions and restoring controls, and storing product data information.
 
- New safety door parking motion as a compile-option. Grbl will retract, disable the spindle/coolant, and park near Z max. When resumed, it will perform these task in reverse order and continue the program. Highly configurable, even to add more than one parking motion. See config.h for details.

- New '$' Grbl settings for max and min spindle rpm. Allows for tweaking the PWM output to more closely match true spindle rpm. When max rpm is set to zero or less than min rpm, the PWM pin D11 will act like a simple enable on/off output.

- Updated G28 and G30 behavior from NIST to LinuxCNC g-code description. In short, if a intermediate motion is specified, only the axes specified will move to the stored coordinates, not all axes as before.

- Lots of minor bug fixes and refactoring to make the code more efficient and flexible.


```
List of Supported G-Codes in Grbl v1.1:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80
  - Feed Rate Modes: G93, G94
  - Unit Modes: G20, G21
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30*
  - Coolant Control: M7*, M8, M9
  - Spindle Control: M3, M4, M5
  - Valid Non-Command Words: F, I, J, K, L, N, P, R, S, T, X, Y, Z, A, B, C
```

-------------
Grbl-Mega-5X is an open-source project and fueled by the free-time of our intrepid administrators and altruistic users. If you'd like to donate, all proceeds will be used to help fund supporting hardware and testing equipment. Thank you!

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://paypal.me/pools/c/842hNSm2It)

