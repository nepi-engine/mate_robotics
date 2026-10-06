# Raspberry Pi Ltd Raspberry Pi 5

## Summary

The Raspberry Pi 5 is a Linux single-board computer built on a Broadcom BCM2712 (quad-core Cortex-A76 at 2.4 GHz). It has 2 USB 3.0 ports, 2 USB 2.0 ports and the standard 40-pin GPIO header [1]. Its USB, Ethernet and GPIO hang off a separate Raspberry Pi-designed I/O chip, the RP1, which connects to the CPU over PCIe [13]. Drivers reach its buses through Linux device files: `/dev/i2c-*`, `/dev/ttyAMA*`, `/dev/gpiochip*`, PWM, and USB.

The most important fact for a driver author: because GPIO moved to the RP1, GPIO libraries that poke SoC registers directly no longer work. RPi.GPIO and pigpio do not run on the Pi 5, and code must use the kernel's gpiochip interface (lgpio, gpiozero, libgpiod) instead [17][18].

## At a Glance

| Item | Value | Source |
|---|---|---|
| CPU | Broadcom BCM2712, 2.4 GHz quad-core Cortex-A76 | [1] |
| I/O controller | RP1, on a PCIe 2.0 x4 link to the BCM2712 | [13] |
| Power input | 5 V / 5 A over USB-C with Power Delivery | [1] |
| USB | 2 × USB 3.0 (simultaneous 5 Gbps) and 2 × USB 2.0 | [1] |
| USB peripheral current | 600 mA total with a 3 A supply; 1.6 A with a 5 A supply | [10][11] |
| GPIO header | Standard 40-pin, 0.1 in (2.54 mm) pitch; 28 GPIOs from RP1 bank 0 | [1][3][4] |
| Logic level | 3.3 V; do not connect 5 V to GPIO | [4][5] |
| I2C | 4 controllers on the header bank (`i2c0-pi5` to `i2c3-pi5`); up to 1000 kbit/s (Fast-mode Plus) | [3][6] |
| UARTs | UART0 to UART4 on the header (PL011, disabled by default) plus UART10 on the debug header; no mini UART | [5][6] |
| Hardware PWM | 4 channels (RP1 PWM0) on GPIO12, 13, 18 and 19 (channels 2 and 3 also on GPIO14 and 15) | [3][4] |
| Operating temperature | 0 to 70 °C | [2] |
| Identify as Pi 5 | `/proc/device-tree/compatible` contains `raspberrypi,5-model-b` and `brcm,bcm2712` | [12] |

## Physical and Electrical Interface

**Power.** The Pi 5 takes 5 V / 5 A over USB-C with Power Delivery [1]. Raspberry Pi recommends a 5.0 A supply, and the typical bare-board active current is 800 mA [11].

With a 3 A supply the Pi still boots, but limits USB peripherals to 600 mA total and warns about it. With a supply it detects as USB-PD 5 A, the limit rises to 1.6 A [10]. The USB ports and the fan header share that power budget [10].

**GPIO header voltage.** The header has two 5 V pins, two 3.3 V pins and several grounds. Every other pin is a 3.3 V GPIO: outputs drive 3.3 V and inputs are 3.3 V tolerant [4]. The UARTs run at 3.3 V, and Raspberry Pi warns that connecting them to 5 V systems causes damage [5].

**RP1 GPIO pads** [3]:

- 28 GPIOs serve the 40-pin header, all in one electrical bank (bank 0).
- That bank can run at 1.8 V or 3.3 V, and interface timings are specified at 3.3 V.
- Output drive strength is selectable at 2, 4, 8 or 12 mA.
- The pads are "fault tolerant": very little current flows while the pin is below 3.63 V and the I/O supply is 0 V.

Raspberry Pi's general power page says all GPIO pins together can safely draw 50 mA, and each pin up to 16 mA [11]. That statement is not specific to the Pi 5, and the GPIO voltage tables on the GPIO page cover only older chips [4]. No Pi-5-specific per-pin current limit was found (Open Question 7).

**Reserved and fixed pins** [4]:

- GPIO2 and GPIO3 have fixed pull-up resistors.
- GPIO0 and GPIO1 (physical pins 27 and 28) are "reserved for advanced use".
- All GPIOs revert to inputs at power-on reset.

**40-pin header map.** Physical pin to GPIO numbering comes from pinout.xyz [20], a third-party site. The Pi 5 functions come from the RP1 function table [3] and the Pi 5 overlay definitions [6].

| Pin | Signal | Pi 5 functions relevant here | Pin | Signal | Pi 5 functions relevant here |
|---|---|---|---|---|---|
| 1 | 3.3 V | — | 2 | 5 V | — |
| 3 | GPIO2 | I2C1 SDA (`i2c_arm` / `i2c1-pi5` default) | 4 | 5 V | — |
| 5 | GPIO3 | I2C1 SCL | 6 | GND | — |
| 7 | GPIO4 | I2C2 SDA (`i2c2-pi5` default), UART2 TX | 8 | GPIO14 | UART0 TX (`uart0-pi5`), PWM0 ch 2 |
| 9 | GND | — | 10 | GPIO15 | UART0 RX, PWM0 ch 3 |
| 11 | GPIO17 | UART0 RTS (with `ctsrts`) | 12 | GPIO18 | PWM0 ch 2 |
| 13 | GPIO27 | — | 14 | GND | — |
| 15 | GPIO22 | I2C3 SDA (`i2c3-pi5,pins_22_23`) | 16 | GPIO23 | I2C3 SCL (`pins_22_23`) |
| 17 | 3.3 V | — | 18 | GPIO24 | — |
| 19 | GPIO10 | SPI0 MOSI; I2C1 SDA (`pins_10_11`) | 20 | GND | — |
| 21 | GPIO9 | SPI0 MISO; UART3 RX; I2C0 SCL (`pins_8_9`) | 22 | GPIO25 | — |
| 23 | GPIO11 | SPI0 SCLK; I2C1 SCL (`pins_10_11`) | 24 | GPIO8 | SPI0 CE0; UART3 TX; I2C0 SDA (`pins_8_9`) |
| 25 | GND | — | 26 | GPIO7 | SPI0 CE1; I2C3 SCL (`i2c3-pi5` default) |
| 27 | GPIO0 | Reserved (ID EEPROM); I2C0 SDA; UART1 TX | 28 | GPIO1 | Reserved (ID EEPROM); I2C0 SCL; UART1 RX |
| 29 | GPIO5 | I2C2 SCL; UART2 RX | 30 | GND | — |
| 31 | GPIO6 | I2C3 SDA (`i2c3-pi5` default) | 32 | GPIO12 | PWM0 ch 0; UART4 TX |
| 33 | GPIO13 | PWM0 ch 1; UART4 RX | 34 | GND | — |
| 35 | GPIO19 | PWM0 ch 3 | 36 | GPIO16 | UART0 CTS (with `ctsrts`) |
| 37 | GPIO26 | — | 38 | GPIO20 | — |
| 39 | GND | — | 40 | GPIO21 | — |

## Communication Protocol

This section covers the host buses a driver uses on the Pi 5. Each device's own protocol is in that device's document.

### I2C

- **Hardware.** RP1 bank 0 has 4 I2C controllers [3]. They support standard mode (up to 100 kbit/s), fast mode (up to 400 kbit/s) and Fast-mode Plus (up to 1000 kbit/s), but not high-speed mode [3].
- **Main header bus.** `dtparam=i2c_arm=on` (alias `i2c`) enables the main header I2C bus. `i2c_arm_baudrate` sets its clock, default 100000 [6].
- **Pi 5 overlays.** `i2c0-pi5` through `i2c3-pi5` enable each controller [6]:

  | Overlay | Default pins | Alternate pins |
  |---|---|---|
  | `i2c0-pi5` | GPIO0/1 | GPIO8/9 |
  | `i2c1-pi5` | GPIO2/3 | GPIO10/11 |
  | `i2c2-pi5` | GPIO4/5 | GPIO12/13 |
  | `i2c3-pi5` | GPIO6/7 | GPIO14/15 or GPIO22/23 |

  Each overlay takes a `baudrate` parameter, default 100000 [6].
- **Software I2C.** `dtoverlay=i2c-gpio` adds a bit-banged I2C bus on any two GPIOs. Its `i2c_gpio_delay_us` default of 2 gives about 100 kHz, and its `bus` parameter picks the `/dev/i2c-<n>` number [6].
- **Clock stretching.** The RP1 datasheet's I2C chapter does not discuss clock stretching (Open Question 8).
- **Kit address conflict.** The Bar depth sensor [26] and the MS8607's pressure die [27] both use address 0x76, so they cannot share a bus. Separate controllers are the plain fix.

### UART

- **Available UARTs.** The Pi 5 has UART0 to UART4 and UART10 (the debug UART), all PL011, all disabled by default, and no mini UART [5]. The primary UART is UART10, which goes to the dedicated 3-pin debug header labelled `UART` [5].
- **Device names.** `/dev/ttyAMA0` is UART0, `/dev/ttyAMA10` is the debug UART, and `/dev/serial0` points to the primary UART (`/dev/ttyAMA10` on a Pi 5) [5].
- **Pi 5 overlays** [6]:

  | Overlay | Pins | RTS/CTS pins (with `ctsrts`) |
  |---|---|---|
  | `uart0-pi5` | GPIO14/15 | GPIO16/17 |
  | `uart1-pi5` | GPIO0/1 | GPIO2/3 |
  | `uart2-pi5` | GPIO4/5 | GPIO6/7 |
  | `uart3-pi5` | GPIO8/9 | GPIO10/11 |
  | `uart4-pi5` | GPIO12/13 | GPIO14/15 |

  Each also supports RS485 mode [6].
- **`enable_uart=1`.** With this set and no cable on the debug header, the Pi 5 automatically routes kernel log output to the UART on GPIO14/15 [5]. To use those pins for a device, disable the serial console (raspi-config: Interface Options > Serial Port, answer No to the login shell and Yes to the hardware) [5].

### PWM

- **Hardware.** RP1 has two PWM blocks, and only one (PWM0) reaches GPIO bank 0. It has 4 independent channels with 32-bit counters [3].
- **Channel to pin mapping** [3]:
  - Channel 0: GPIO12.
  - Channel 1: GPIO13.
  - Channel 2: GPIO14 or GPIO18.
  - Channel 3: GPIO15 or GPIO19.

  The Raspberry Pi GPIO page lists hardware PWM on GPIO12, 13, 18 and 19 [4]. Channels 2 and 3 each drive one output however many pins are routed to them, so the Pi 5 has at most **4 independent hardware PWM outputs**.
- **Overlay caveat.** The `pwm` and `pwm-2chan` overlays take `pin` and `func` parameters. Their legal pin/function table is written in the older SoC's function numbering, and nothing marks it as Pi-5-specific [6]. How they map onto RP1 must be checked on hardware (Open Question 4).
- **PIO-assisted PWM.** The `pwm-pio` overlay (Pi 5 only) makes a PIO-assisted PWM output on any GPIO 0 to 27, up to 4 of them, if nothing else is using PIO [6]. Its pulse accuracy is not documented.
- **No pigpio.** pigpio does not work on the Pi 5 [17], so its DMA-timed servo pulses are not available.
- **What this means for the kit.** A design needing 8 thruster ESCs plus a servo (9 independent pulse trains) exceeds the 4 hardware channels plus 4 PIO channels. A design needing 6 ESCs plus a servo (7) fits only by mixing hardware and PIO-assisted outputs. That PIO mix is untested here (Open Question 4).

### USB

- **Ports.** 2 × USB 3.0 (simultaneous 5 Gbps) and 2 × USB 2.0 [1].
- **Controllers.** RP1 contains two USB host controllers, each with one USB 3.0 port and one USB 2.0 port. Raspberry Pi says this gives more than twice the usable USB bandwidth of a Pi 4 [13]. Which physical port is on which controller is not stated in the sources checked (Open Question 5).
- **Current.** See the power limits above [10].

### GPIO access from software

- **Kernel interface.** The header GPIOs are driven by the `pinctrl-rp1` driver. A kernel change merged on 5 August 2024 gave it the gpiochip0 alias; before that it appeared as gpiochip4, and Raspberry Pi engineers noted the number depends on probe order [14].
- **Stale default in rpi-lgpio.** rpi-lgpio's documentation still says its chip number "defaults to '4' on the Raspberry Pi Model 5B" [15]. That default predates the kernel change, so on current kernels it may point at the wrong chip. A driver should look the chip up by label, not by number (Open Question 3).
- **Library support.** gpiozero's pin-factory table lists only lgpio as working on the Pi 5. RPi.GPIO, pigpio and the native factory are marked "not Pi 5" [17].
- **Why RPi.GPIO fails.** A third-party explanation: RPi.GPIO reads and writes processor registers through `/dev/mem`, but on the Pi 5 the GPIO registers live in the RP1. The library fails with `RuntimeError: This module can only be run on a Raspberry Pi!` [18].
- **Permissions.** A user needs to be in the `gpio` group to use GPIO [4].

## Identification and Detection

- **Board model.** `cat /proc/device-tree/compatible | tr '\0' '\n'` prints `raspberrypi,5-model-b` and `brcm,bcm2712` on a Pi 5 [12]. `/sys/firmware/devicetree/base/model` gives the model string [12].
- **Do not use "Hardware".** Every Pi reports `Hardware : BCM2835` in `/proc/cpuinfo`, so that line must not be used to detect the processor [12].
- **Revision code.** The `Revision` line in `/proc/cpuinfo` holds the revision code. Bits 4 to 11, read as `(code >> 4) & 0xff`, give the model type, and `0x17` means Pi 5 [12]. Published Pi 5 codes include `c04170` (4 GB, rev 1.0), `d04170` (8 GB, rev 1.0) and `d04171` (8 GB, rev 1.1) [12].
- **Unique board serial.** `/proc/cpuinfo` ends with a `Serial` line, the board's unique serial number [12].
- **Firmware-side check.** In `config.txt`, a `[pi5]` section applies to the Pi 5, 500, 500+ and Compute Module 5 [9].

## Startup and Initialization

- **Where settings live.** On current Raspberry Pi OS, bus settings go in `/boot/firmware/config.txt` and take effect after a reboot [5]. `dtparam=` sets base parameters, `dtoverlay=` loads a named overlay, and each line is limited to 98 characters [8].
- **Console on GPIO14/15.** The serial console must be disabled before a device uses GPIO14/15; otherwise kernel messages and a login prompt appear on that UART [5].
- **`enable_rp1_uart=1`.** Firmware sets RP1 UART0 to 115200 bps and does not reset RP1 before starting the OS, which is meant for early-boot debug. Default is 0 [7].
- **GPIO state at boot.** All GPIOs come up as inputs [4]. PWM and UART pins carry no signal until the kernel or the driver configures them.

## Data and Commands

| Resource | Range | Source |
|---|---|---|
| I2C clock | 100 kbit/s default; up to 1000 kbit/s (Fast-mode Plus) | [3][6] |
| UART | PL011; early console default 115200 bps on the debug UART | [5] |
| Hardware PWM | 4 channels, 32-bit counters | [3] |
| PIO-assisted PWM | Up to 4 outputs on GPIO 0 to 27 | [6] |
| USB 3.0 | 5 Gbps per port, both ports at once | [1] |
| USB peripheral current | 600 mA (3 A supply) or 1.6 A (5 A supply) | [10] |

## Configuration and Calibration

The `config.txt` lines a driver set will most likely need [5][6][7]:

```
[pi5]
dtparam=i2c_arm=on                 # I2C1 on GPIO2/3
dtparam=i2c_arm_baudrate=100000    # default 100 kbit/s
dtoverlay=i2c3-pi5,pins_22_23      # an extra I2C bus, e.g. for a second 0x76 device
dtoverlay=uart0-pi5                # UART0 on GPIO14/15
dtoverlay=pwm-pio,gpio=4           # PIO-assisted PWM on one GPIO (Pi 5 only)
```

- **Persistence.** All of these are read at boot and persist until the file is edited [5][8].
- **Checking the USB current limit.** `vcgencmd get_config usb_max_current_enable` reports the current limit setting [11].
- **Inspecting pins.** The `pinctrl` tool displays and sets GPIO and pin-mux state [19]:
  - `pinctrl -p` shows the header pins.
  - `pinctrl funcs 12-19` lists alternate functions.

  It bypasses the kernel drivers, so use it for inspection, not as a runtime driver [19].
- **Calibration.** The Pi has nothing to calibrate itself.

## Failure Modes and Safety

- **5 V on GPIO or UART.** It damages the Pi [4][5]. Every 5 V device needs a level shifter.
- **Too little supply current.** With a supply that cannot provide 5 A, USB peripherals are limited to 600 mA total [10]. A camera plus a USB microcontroller can approach that.
- **Stale GPIO chip numbers.** Code that opens `/dev/gpiochip4` fails, or opens the wrong controller, on kernels that apply the gpiochip0 alias [14][15].
- **Wrong GPIO library.** RPi.GPIO and pigpio fail on the Pi 5 [17][18]. Old example code that imports them will not run.
- **No shared pins.** The kernel gpiochip interface does not let two processes control the same pin [15].
- **Pin overload.** Raspberry Pi's general guidance (not Pi-5-specific) is 16 mA per pin and 50 mA for all GPIO combined [11]. Never drive a motor from a GPIO [4].
- **Temperature.** The operating range is 0 to 70 °C [2]. A sealed enclosure on a vehicle can exceed this.

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| GPIO Zero | High-level GPIO; Raspberry Pi's recommended Python library | Python | BSD-3-Clause [22] | [17][21] |
| lgpio (`lg`) | Low-level GPIO on the kernel gpiochip interface | C, Python | Unlicense [23] | [23] |
| rpi-lgpio | RPi.GPIO-compatible API on top of lgpio, for porting old code | Python | MIT [24] | [15][16] |
| pinctrl (raspberrypi/utils) | Pin state and mux inspection from the shell | C | Not checked | [19] |
| Device Tree overlays README | Every `dtoverlay` and `dtparam` with its parameters | Text | — | [6] |

## Related Parts

- [PJRC Teensy 4.1](pjrc_teensy_4_1.md): the likely PWM and sensor co-processor, reached over USB serial, since the Pi 5 has only 4 hardware PWM channels [3].
- [Blue Robotics Basic ESC](bluerobotics_basic_esc.md): a pulse-driven part; see the PWM section above for how many pulse trains the Pi 5 can produce.
- [Axon MAX MK2 servo](../svx/axon_max_mk2_servo.md): one more pulse train; could use a Pi 5 hardware PWM channel if a wiring decision puts it there.
- [DeepWater Exploration exploreHD camera](../idx/dwe_explorehd_camera.md): USB camera; uses Pi 5 USB bandwidth and peripheral current.
- [Adafruit BNO085 IMU](../npx/adafruit_bno085_imu.md): I2C or UART sensor; Adafruit documents I2C clock-stretching problems with it on Raspberry Pi [25].
- [Blue Robotics Bar depth sensor](../npx/bluerobotics_bar_depth_sensor.md): I2C sensor at 0x76 [26]; needs a different bus from the MS8607.
- [Adafruit MS8607 PHT sensor](adafruit_ms8607_pht_sensor.md): I2C sensor at 0x76 and 0x40 [27]; needs a different bus from the Bar sensor.

## Open Questions

1. **Which devices does the Pi 5 talk to directly?**
   - Why it matters: the Pi might host the I2C sensors and the servo directly, or reach everything through the Teensy. That decision sets which overlays to enable and which pins are free.
   - How to get it: a team wiring decision, recorded as a pin table in this repo.
2. **I2C bus numbering.**
   - Why it matters: drivers open `/dev/i2c-<n>`, and the sources checked give no mapping from controller to `n` on the Pi 5.
   - How to get it: after enabling the overlays, run `ls /dev/i2c-*`, `i2cdetect -l` and `i2cdetect -y <n>` for each bus.
3. **GPIO chip for the header.**
   - Why it matters: code must open the right `/dev/gpiochip*`, and the number has changed between kernels [14][15].
   - How to get it: `cat /proc/device-tree/compatible | tr '\0' '\n'`, then `gpiodetect` and `gpioinfo` (libgpiod tools) or `sudo pinctrl -l` [19] on the vehicle's kernel. Look up the chip by its label in code.
4. **Hardware and PIO PWM on Pi 5.**
   - Why it matters: if any pulses come from the Pi, the driver must know the exact overlay line, the PWM chip number and the pulse jitter.
   - How to get it: enable `dtoverlay=pwm-2chan` (and `pwm-pio`) on a test Pi, then:
     - `ls /sys/class/pwm/` to find the PWM chip.
     - `sudo pinctrl get 12,13,18,19` to confirm the pin functions.
     - A logic analyzer on the output at 50 Hz and 400 Hz to measure jitter.
5. **USB port to controller mapping.**
   - Why it matters: to keep the camera and the Teensy from sharing one controller's bandwidth.
   - How to get it: plug the devices in and run `lsusb -t`.
6. **Power supply.**
   - Why it matters: whether the vehicle supplies 5 V / 5 A with USB-PD signalling decides the 600 mA vs 1.6 A USB current limit [10].
   - How to get it: `vcgencmd get_config usb_max_current_enable` on the vehicle supply [11], and the power-system design.
7. **Per-pin GPIO current for Pi 5 / RP1.**
   - Why it matters: the published per-pin figure is generic, not RP1-specific [11].
   - How to get it: a Raspberry Pi datasheet for the Pi 5 or RP1 electrical characteristics, or ask Raspberry Pi. Until then, keep pin loads at logic-level only.
8. **RP1 I2C and clock stretching.**
   - Why it matters: the BNO085's documented Raspberry Pi problem involves I2C clock stretching [25], and the RP1 datasheet does not mention clock stretching.
   - How to get it: run the BNO085 on the Pi 5 I2C bus under load and log errors, as described in the BNO085 document.
9. **Enclosure temperature.**
   - Why it matters: the Pi is rated 0 to 70 °C [2].
   - How to get it: run `vcgencmd measure_temp` during a closed-enclosure bench test, plus the MS8607 enclosure temperature reading.

## Sources

1. "Raspberry Pi 5", Raspberry Pi Ltd. https://www.raspberrypi.com/products/raspberry-pi-5/. Used for: CPU, USB ports, power input, header, RP1, PCIe.
2. "Raspberry Pi 5 product brief", Raspberry Pi Ltd. https://pip.raspberrypi.com/documents/RP-008348-DS-raspberry-pi-5-product-brief.pdf. Used for: operating temperature, production lifetime.
3. "RP1 Peripherals" (build 2023-11-07), Raspberry Pi Ltd. https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf. Used for: GPIO bank and pads, GPIO function table, PWM block (4 channels, one instance on bank 0), I2C modes.
4. "GPIO and the 40-pin header" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/raspberry-pi/gpio-on-raspberry-pi.adoc. Used for: 3.3 V logic, hardware PWM pins, reserved pins, gpio group, power-on state.
5. "Configure interfaces" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/configuration/interfaces.adoc. Used for: Pi 5 UARTs, device names, primary UART, console, config.txt location, 3.3 V UART warning.
6. "Device Tree overlays README" (rpi-6.12.y), Raspberry Pi Ltd (GitHub). https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.12.y/arch/arm/boot/dts/overlays/README. Used for: `i2c_arm`, `i2c0-3-pi5`, `uart0-4-pi5`, `pwm`, `pwm-2chan`, `pwm-pio`, `i2c-gpio`.
7. "config.txt: boot options" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/config_txt/boot.adoc. Used for: `enable_uart`, `enable_rp1_uart`.
8. "config.txt: common options" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/config_txt/common.adoc. Used for: `dtoverlay` and `dtparam` semantics, line-length limit.
9. "config.txt: conditional filters" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/config_txt/conditional.adoc. Used for: the `[pi5]` filter.
10. "Universal Serial Bus (USB)" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/raspberry-pi/usb-bus-on-raspberry-pi.adoc. Used for: Pi 5 USB current limits, shared budget.
11. "Power supply" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc. Used for: 5.0 A recommendation, 800 mA bare-board current, generic GPIO current guidance, `vcgencmd` check.
12. "Raspberry Pi revision codes" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/raspberry-pi/revision-codes.adoc. Used for: `/proc/cpuinfo` fields, Pi 5 type `0x17` and codes, device-tree compatible strings.
13. "RP1 I/O controller" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/io-controllers/rp1.adoc. Used for: PCIe link, USB controller layout, peripheral list.
14. "Use gpiochip0 for the user-facing GPIOs on Pi 5" (PR #6144), Raspberry Pi Ltd (GitHub). https://github.com/raspberrypi/linux/pull/6144. Used for: gpiochip4 to gpiochip0 change, merged 2024-08-05.
15. "rpi-lgpio: Differences", rpi-lgpio project. https://rpi-lgpio.readthedocs.io/en/latest/differences.html. Used for: chip default on Pi 5, no alternate modes, no shared pin access.
16. "rpi-lgpio", rpi-lgpio project. https://rpi-lgpio.readthedocs.io/en/latest/. Used for: purpose (RPi.GPIO API on gpiochip-only kernels).
17. "API - Pins", GPIO Zero project. https://gpiozero.readthedocs.io/en/stable/api_pins.html. Used for: pin-factory support table ("Only lgpio works on the Pi 5").
18. "Raspberry Pi 5 GPIO: RPi.GPIO Not Working – These Alternatives Exist", raspberry.tips (THIRD-PARTY). https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-5-gpio-rpigpio-not-working-alternatives. Used for: why RPi.GPIO fails (`/dev/mem`, RP1), error text.
19. "pinctrl README", Raspberry Pi Ltd (raspberrypi/utils, GitHub). https://raw.githubusercontent.com/raspberrypi/utils/master/pinctrl/README.md. Used for: `pinctrl` usage, kernel-bypass note.
20. "Raspberry Pi GPIO Pinout", pinout.xyz (THIRD-PARTY). https://pinout.xyz/. Used for: physical pin to GPIO numbering of the 40-pin header.
21. "Use GPIO from Python" (documentation source), Raspberry Pi Ltd. https://raw.githubusercontent.com/raspberrypi/documentation/master/documentation/asciidoc/computers/os/using-gpio.adoc. Used for: GPIO Zero as the recommended library.
22. "GPIO Zero LICENSE.rst", GPIO Zero project (GitHub). https://raw.githubusercontent.com/gpiozero/gpiozero/master/LICENSE.rst. Used for: license.
23. "lg UNLICENCE", joan2937/lg (GitHub). https://raw.githubusercontent.com/joan2937/lg/master/UNLICENCE. Used for: lgpio license.
24. "rpi-lgpio LICENSE.txt", rpi-lgpio project (GitHub). https://raw.githubusercontent.com/waveform80/rpi-lgpio/main/LICENSE.txt. Used for: license.
25. "Raspberry Pi I2C Clock Stretching Fixes", Adafruit. https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes. Used for: the BNO085's I2C clock-stretching problem on Raspberry Pi.
26. "Bar High-Resolution Depth/Pressure Sensors", Blue Robotics. https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/. Used for: Bar sensors use I2C address 0x76.
27. "Adafruit TE MS8607 PHT Sensor" (Learn guide), Adafruit. https://learn.adafruit.com/adafruit-te-ms8607-pht-sensor. Used for: MS8607 addresses 0x76 and 0x40.
