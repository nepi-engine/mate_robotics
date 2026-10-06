# PJRC Teensy 4.1 Microcontroller

## Summary

The Teensy 4.1 is a microcontroller board built on the NXP i.MX RT1062, an ARM Cortex-M7 running at 600 MHz. It has 35 PWM-capable pins, 3 I2C ports and 8 hardware serial ports [1]. A Linux host talks to it over its USB device port. With the usual "Serial" USB type, the board shows up as a CDC-ACM serial device whose baud-rate setting is ignored [1][5][13]. The host-side protocol is whatever firmware the team writes for the Teensy, and none exists yet.

The most important constraint for a driver author is electrical: every I/O pin is 3.3 V only and not 5 V tolerant [1]. The host never touches the PWM outputs or I2C sensors directly. Everything passes through Teensy firmware that the team has to specify, write and version.

## At a Glance

| Item | Value | Source |
|---|---|---|
| Processor | ARM Cortex-M7 at 600 MHz (NXP i.MX RT1062) | [1][19] |
| Host interface | USB device port, 480 Mbit/s | [1] |
| Host-side device (USB type "Serial") | CDC-ACM serial port, `/dev/ttyACM*` on Linux; baud setting ignored | [5][13] |
| USB IDs (USB type "Serial") | VID `0x16C0`, PID `0x0483`; manufacturer string "Teensyduino", product string "USB Serial" | [11] |
| USB IDs (bootloader) | VID `0x16C0`, PID `0x0478` | [16] |
| Unique ID | USB serial-number string derived from the board's factory-fused MAC address | [12][17] |
| Supply (VIN) | 3.6 to 5.5 V; normally 5 V from USB | [1][2] |
| Logic level | 3.3 V; pins are not 5 V tolerant | [1] |
| Pin output current | 4 mA recommended maximum | [1] |
| 3.3 V pin output | 250 mA recommended maximum for external use | [1][2] |
| PWM | 35 pins in 22 frequency groups | [1][3] |
| I2C | 3 ports at 100, 400 or 1000 kbit/s | [1] |
| Hardware UARTs | 8 | [1] |
| Connector | USB Micro-B (PJRC's listed cable for this board) | [20] |

## Physical and Electrical Interface

**Power.** USB power arrives on VUSB, which is connected to VIN [1]. If the USB cable is not used, 5 V may be applied to VIN [1]. The pinout card gives the VIN range as 3.6 to 5.5 V [2].

Do not apply power to VIN while a USB cable is connected, because current can flow back into the host [1]. The other option is to cut the pair of VUSB-VIN pads on the bottom of the board, which separates the two supplies [1]. An onboard regulator makes 3.3 V. External circuits may draw up to 250 mA from the 3.3 V pin [1]. At 600 MHz the board draws about 100 mA [1]. (PJRC's page attributes that figure to "Teensy 4.0" in the 4.1 power section.)

**Logic levels.** Digital and analog pins accept 0 to 3.3 V and are not 5 V tolerant [1]. An output HIGH is 3.3 V, with a recommended maximum output current of 4 mA [1]. NXP's datasheet thresholds for the GPIO, where NVCC is the I/O supply (3.3 V on this board [1]):

- Input high (VIH): at least 0.7 × NVCC [19].
- Input low (VIL): at most 0.3 × NVCC [19].
- Output high: at least NVCC − 0.15 V at −1 mA [19].

Every digital pin has optional pull-up, pull-down or "keeper" resistors. Pins default to INPUT, most with a keeper [1]. Output drive strength can be set in 7 steps from 150 Ω to about 21 Ω, and slew-rate limiting is available for long wires [1].

**Pin functions that matter for this kit.** The I2C and Serial1 pins share pins with PWM outputs (PWM pin list [3], I2C pins [4], Serial1 pins [2]):

| Function | Pins | Also a PWM pin? |
|---|---|---|
| Wire (I2C port 0) | SDA 18, SCL 19 | Yes (QuadTimer3.1 and QuadTimer3.0) |
| Wire1 (I2C port 1) | SDA 17, SCL 16; alternate SDA 44, SCL 45 | 16 and 17: no. 44 and 45: yes (FlexPWM1.0) |
| Wire2 (I2C port 2) | SDA 25, SCL 24 | Yes (FlexPWM1.3 and FlexPWM1.2) |
| Serial1 (UART) | RX1 0, TX1 1 | Yes (FlexPWM1.1 and FlexPWM1.0) |

Using Wire1 on pins 16 and 17 is the only I2C option that costs no PWM-capable pin.

**I2C pull-ups.** The on-chip pull-ups are very weak. PJRC recommends external 1 kΩ to 4.7 kΩ pull-ups. The internal ones are not enough for several chips or for speeds above 100 kHz [4].

**LED.** Pin 13 drives an orange LED. When pin 13 is an input, the external signal must be able to drive that LED [1].

## Communication Protocol

### Transport facts (confirmed)

- **USB link.** The main USB port is a USB device at 480 Mbit/s. The USB "type" is chosen when the firmware is compiled (Tools > USB Type), and several types can run at once [1].
- **Serial and the baud rate.** With USB type "Serial", the host sees a serial device and bytes move at USB speed. The baud rate set with `Serial.begin(baud)` is ignored [1][5].
- **Linux device node.** Teensy USB serial is a CDC-ACM device [5]. PJRC's udev rules match it as `ttyACM*` with VID `16c0` [13].
- **Dual and Triple Serial.** These USB types add `SerialUSB1` and `SerialUSB2` ports [5]. They use PIDs `0x048B` and `0x048C` [11].
- **134 baud is reserved.** PJRC's tools put a running Teensy into programming mode by sending a "serial baud rate" request [1]. `teensy_loader_cli` does this by sending a CDC SET_LINE_CODING request with the line rate set to 134 (`0x86`) [16]. A host driver must never open the port at 134 baud, or it will reboot the Teensy into its bootloader.
- **Transmit buffering (Teensy to host).** `Serial.print()` writes into a USB buffer. A partly filled buffer is sent after a timeout of about 3 to 5 ms, or at once if the firmware calls `Serial.send_now()` [5]. On Linux, a port opened with the "low latency" option wakes the reading process sooner [5].
- **Receive buffering (host to Teensy).** At 480 Mbit/s a USB packet holds up to 512 bytes, and a whole packet becomes available at once [5]. `Serial.available()` counts only the unread bytes of the first buffered packet. A message that the host splits across packets can therefore look shorter than it is [5]. The firmware's framing must not assume a message arrives in one read.
- **DTR.** `Serial.dtr()` goes high when a host program opens the port and low when no program has it open [5]. Firmware can use this as a "host connected" signal.
- **Error checking.** USB applies CRC checking to bulk packets and retransmits corrupted ones, so the USB link itself does not use parity [5].

### PWM generation (for ESCs and servos)

- **Frequency groups.** Pins on the same hardware timer always share one PWM frequency. Changing one pin's frequency with `analogWriteFrequency(pin, Hz)` changes every pin on that timer [3]. The 22 groups below can each have their own frequency [1][3].
- **Resolution.** `analogWriteResolution(bits)` sets the value range and accepts 1 to 16 bits [3][9]. The real resolution depends on the frequency, and slower frequencies get more resolution [3].
- **How the core picks the period.** The FlexPWM code divides the bus clock by the requested frequency and halves the result, raising the prescaler up to 7 steps, until it fits in 16 bits (65535) [9]. At a 600 MHz CPU clock the bus clock (`F_BUS_ACTUAL`) is 150 MHz [10]. The QuadTimer code does the same with a 65534 limit [9].
- **Derived pulse resolution.** This is calculated from the cited code, not a PJRC-published figure. At 50 Hz, 150 MHz / 50 = 3,000,000 counts. Six halvings give 46,875 counts per 20 ms period, which is about **0.43 µs per step**. At 400 Hz, three halvings give 46,875 counts per 2.5 ms, about **0.053 µs per step**. Both are only reachable with `analogWriteResolution(16)` [9][10]. Measure the real pulses on hardware (Open Question 9).
- **PWMServo library.** It sets the pin's timer group to 50 Hz and writes a 12-bit duty value, using `duty = usec / 20000 × 4096` as in its source comment. That is about 4.9 µs per step [8]. Its `write()` takes an integer angle from 0 to 180 [8]. It slows every other pin on the same timer to 50 Hz [7].
- **Servo library.** It drives up to 12 servos on any pin using timer interrupts rather than PWM hardware. It is less tolerant of interrupt latency than PWMServo, and libraries that disable interrupts are incompatible with it [7].

Teensy 4.1 PWM timer groups, quoted from PJRC [3]:

| Timer | Pins | Default frequency |
|---|---|---|
| FlexPWM1.0 | 1, 44, 45 | 4.482 kHz |
| FlexPWM1.1 | 0, 42, 43 | 4.482 kHz |
| FlexPWM1.2 | 24, 46, 47 | 4.482 kHz |
| FlexPWM1.3 | 7, 8, 25 | 4.482 kHz |
| FlexPWM2.0 | 4, 33 | 4.482 kHz |
| FlexPWM2.1 | 5 | 4.482 kHz |
| FlexPWM2.2 | 6, 9 | 4.482 kHz |
| FlexPWM2.3 | 36, 37 | 4.482 kHz |
| FlexPWM3.0 | 54 | 4.482 kHz |
| FlexPWM3.1 | 28, 29 | 4.482 kHz |
| FlexPWM3.3 | 51 | 4.482 kHz |
| FlexPWM4.0 | 22 | 4.482 kHz |
| FlexPWM4.1 | 23 | 4.482 kHz |
| FlexPWM4.2 | 2, 3 | 4.482 kHz |
| QuadTimer1.0 | 10 | 3.611 kHz |
| QuadTimer1.1 | 12 | 3.611 kHz |
| QuadTimer1.2 | 11 | 3.611 kHz |
| QuadTimer2.0 | 13 | 3.611 kHz |
| QuadTimer3.0 | 19 | 3.611 kHz |
| QuadTimer3.1 | 18 | 3.611 kHz |
| QuadTimer3.2 | 14 | 3.611 kHz |
| QuadTimer3.3 | 15 | 3.611 kHz |

**The two PJRC pages disagree on two timer pins.** The product page's "PWM Timers" list gives FlexPWM3 Module0 as pin 53 and FlexPWM3 Module3 as pin 41 [1]. The PWM page gives pins 54 and 51 [3], and its PWM-capable pin list includes 51 and 54 but not 41 or 53 [3]. The product page itself says pinout cards printed before September 2021 wrongly showed pin 53 as PWM [1]. Trust the PWM page [3], and avoid these four pins for thrusters until they are checked on a scope.

Only 42 of the 55 I/O signals are easy to reach from a solderless breadboard [1]. Some pins in the table may need soldering to bottom pads.

### I2C

There are 3 I2C ports, each supporting 100, 400 and 1000 kbit/s [1]. Each chip on one SDA/SCL pair needs a unique address, and the separate ports let the board use more than one chip with the same address [1]. The Arduino `Wire`, `Wire1` and `Wire2` objects map to the ports in the pin table above [4].

### To Be Defined

No firmware or Pi-to-Teensy protocol exists yet. Once written, the protocol specification must cover the following. This list does not design any of them:

1. **Command set.** At minimum: set a pulse width (µs) on a numbered output channel, set several channels at once, read each attached sensor, and request status.
2. **Framing and checksum.** How a message starts and ends, how its length is carried, the checksum or CRC, and what happens to a malformed frame. This must survive the packet behavior described under Receive buffering.
3. **Identify reply.** Device type, firmware version, and the board's unique serial number, so the host can confirm it opened the right port and is running compatible firmware.
4. **Telemetry rate.** How often sensor data and status are pushed or polled, and whether the host can change the rate.
5. **Command watchdog.** A timeout after which the firmware drives every thruster channel to its stop pulse if no valid command has arrived. It needs a defined period, a defined behavior for the servo channel, and an explicit way for the host to re-arm. The ESC's own stop and arming rules are in [the Basic ESC document](bluerobotics_basic_esc.md).
6. **Connect and disconnect behavior.** What the firmware does at power-up before the host connects, and when DTR drops (host closed the port).
7. **Channel and pin map.** Which channel number drives which physical pin, and the PWM frame rate per timer group.
8. **Versioning.** How protocol changes are numbered, and how host and firmware refuse to talk across incompatible versions.

## Identification and Detection

- **USB IDs.** USB type "Serial" enumerates as VID `0x16C0`, PID `0x0483`, manufacturer "Teensyduino", product "USB Serial" [11]. Dual Serial is PID `0x048B` and Triple Serial is PID `0x048C` [11]. The bootloader (programming mode) is VID `0x16C0`, PID `0x0478` [16]. PJRC's udev rules match every Teensy as `idVendor 16c0` with `idProduct 04*` [13].
- **Unique serial number.** The firmware's USB serial-number string is the low 24 bits of the `HW_OCOTP_MAC0` fuse word, written in decimal. Values below 10,000,000 are multiplied by 10, to work around a macOS driver bug [12].
- **Who writes that fuse.** Paul Stoffregen of PJRC states that PJRC writes `HW_OCOTP_MAC0` during product testing, and that NXP ships the chips with those bits all zero [17]. A different forum member said NXP sets it at the factory [17]. Trust the PJRC statement, because PJRC builds the boards.
- **Uniqueness.** PJRC holds the IEEE OUI `04:E9:E5`, and each Teensy 4.1 has a unique MAC address programmed into fuses that cannot be changed [18]. Because the serial number is the 24 device bits under that OUI, it should be unique per board. Uniqueness across many units has only been checked by users, not stated as a guarantee by PJRC [18], so confirm it on the real units (Open Question 7).
- **What the IDs do not prove.** VID, PID and the serial number identify the board, not the firmware it runs. Any Teensyduino sketch with USB type "Serial" uses the same IDs [11]. A driver should confirm the firmware with the identify reply listed under To Be Defined.

## Startup and Initialization

- **Before the firmware runs.** The Teensy does not become a serial device until the program is running [5]. Until the firmware configures them, all pins are inputs with keeper resistors [1], so no PWM pulses come out during power-up, programming or a crash. See [the Basic ESC document](bluerobotics_basic_esc.md) for how the ESC treats a missing signal at power-up.
- **`Serial.begin()` delay.** Calling `Serial.begin()` is optional on Teensy and may wait up to 2.5 s for USB. Firmware that waits with `while (!Serial)` will hang forever with no host, so use a timeout [5].
- **Program button.** Pressing it puts the board into programming mode. It is not a reset button [1]. Holding it for 13 to 17 s wipes the flash back to an LED-blink program [1].
- **Reset.** There is no hardware reset signal. Software can reset the board through the watchdog timers or the `SCB_AIRCR` register [1].

## Data and Commands

The firmware defines the host-facing data and commands (see To Be Defined). The hardware those commands will use:

| Resource | Count and range | Source |
|---|---|---|
| PWM outputs | 35 pins, 22 independent frequency groups | [1][3] |
| PWM value resolution | 1 to 16 bits (`analogWriteResolution`) | [3][9] |
| PWM frequency | Lower limit of a few Hz | [3] |
| I2C | 3 ports at 100, 400 or 1000 kbit/s | [1] |
| Hardware UARTs | 8; normally up to 6 Mbit/s | [1][6] |
| Analog inputs | 18 pins at 0 to 3.3 V; 10-bit default, up to 12-bit hardware | [1] |
| Emulated EEPROM | 4284 bytes | [1] |
| USB to host | 480 Mbit/s; up to 512-byte packets | [1][5] |

## Configuration and Calibration

- **Fixed when the firmware is compiled.** The USB type (Serial, Dual Serial and so on) is chosen in the build [1][5].
- **CPU speed.** It sets the bus clock, which changes the ideal PWM frequency table and the pulse resolution [3][10].
- **Settings that survive power cycles.** The board has 4284 bytes of emulated EEPROM [1]. Any trims or limits stored there are firmware-defined.
- **Calibration.** The board has nothing to calibrate itself. Pulse timing accuracy should be checked once on a logic analyzer (Open Question 9).

## Failure Modes and Safety

- **5 V signals.** Driving any pin above 3.3 V can damage the board [1].
- **Power backfeed.** Powering VIN from the vehicle while the USB cable goes to the host can push current into the host, unless the VUSB-VIN pads are cut [1].
- **Pin overload.** Each pin's recommended maximum is 4 mA, and the 3.3 V pin's is 250 mA [1].
- **Bootloader entry stops all outputs.** Opening the port at 134 baud reboots the board into programming mode [1][16]. In that mode the firmware, including every PWM output, stops. The red LED is dim while the bootloader waits and bright while it writes flash [1].
- **ModemManager.** PJRC's udev rules warn that ModemManager interferes with USB serial devices like the Teensy. Symptoms are missing incoming data and "Unable to open /dev/ttyACM0" errors, and the rules mark Teensy devices for ModemManager to ignore [13].
- **Partial messages.** See Receive buffering: `Serial.available()` can under-report a message the host wrote in pieces [5].
- **Hang with no host.** Firmware that waits on `while (!Serial)` never starts without a host [5].
- **Chip revisions.** Boards made after July 2021 use the MIMXRT1062DVJ6B, whose "B" silicon fixes obscure CAN bus and USB isochronous bugs. Earlier boards use the "A" part [1]. NXP errata documents exist for both [1].
- **Weak I2C pull-ups.** Without external pull-ups, I2C may be slow or unreliable [4].

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| Teensyduino (Arduino IDE + Teensy boards add-on) | Primary firmware development environment | C/C++ | Core library is permissive, with a clause requiring PJRC boards be listed in build systems that list targets [9] | [1] |
| Teensy 4 core (`cores/teensy4`) | PWM, USB and clock code cited in this document | C | Same as above [9] | [9][11][12] |
| Wire | I2C master/slave | C++ | Bundled with Arduino [4] | [4] |
| Servo / PWMServo | Servo pulse generation | C++ | PWMServo: LGPL 2.1 or later [8] | [7][8] |
| Teensy Loader (GUI) | Flashing; Linux builds for x86 and for Raspberry Pi / Jetson 64-bit | — | Not checked | [14] |
| `teensy_loader_cli` | Command-line flashing (`--mcu=TEENSY41`, `-w`, `-s`, `-v`) | C | GPL v3 [16] | [15][16] |
| `00-teensy.rules` | udev rules needed for non-root access on Linux | udev | Not stated | [13][15] |
| pySerial | Host-side Python serial access | Python | BSD-style [23] | [21][22] |

pySerial's `serial.tools.list_ports.comports()` reports each port's USB `vid`, `pid`, `serial_number` and `location` (bus and port path) [22]. That is enough to find a Teensy by serial number rather than by `/dev/ttyACM` index.

**Flashing from Linux.**

1. Install `00-teensy.rules` into `/etc/udev/rules.d/`, then unplug and replug the board [13][14]. Linux needs udev rules for non-root users [15].
2. Flash with the Teensy Loader GUI, or with `teensy_loader_cli --mcu=TEENSY41 -w -v firmware.hex` [15].
3. The `-s` option asks running Teensy USB-serial firmware to reboot into programming mode, with no button press needed [15].

## Related Parts

- [Raspberry Pi 5](raspberry_pi_5.md): the likely host for the Teensy over USB; the GPIO-count tradeoff behind using a Teensy is in that document.
- [Blue Robotics Basic ESC](bluerobotics_basic_esc.md): a likely consumer of this board's PWM outputs; it sets the pulse range, frame rate and signal-loss rules the firmware must honor.
- [Blue Robotics T200 thruster](bluerobotics_t200_thruster.md): connected to this board only through an ESC (see that document).
- [Axon MAX MK2 servo](../svx/axon_max_mk2_servo.md): a likely consumer of one PWM output; its pulse range may differ from the ESCs'.
- [Adafruit BNO085 IMU](../npx/adafruit_bno085_imu.md): an I2C sensor this board may host.
- [Blue Robotics Bar depth sensor](../npx/bluerobotics_bar_depth_sensor.md): an I2C sensor this board may host; see that document for the bus it needs.
- [Adafruit MS8607 PHT sensor](adafruit_ms8607_pht_sensor.md): an I2C sensor this board may host; see that document for the bus it needs.

## Open Questions

1. **Is the Teensy the PWM and sensor host at all?**
   - Why it matters: using the Teensy for PWM generation, and possibly the I2C sensors, while it talks to the Pi 5 over USB serial is a likely design, not a decided one. Every other answer below depends on it.
   - How to get it: a team design decision, recorded in this repo.
2. **Pin map.**
   - Why it matters: the firmware and the driver both need to know which Teensy pin drives which ESC or servo, which I2C port carries which sensor, and the PWM frame rate per timer group. Pins 18/19 (Wire), 24/25 (Wire2), and 0/1 (Serial1) are also PWM pins [2][3][4].
   - How to get it: a wiring decision, written as a pin table in this repo.
3. **Protocol owner and specification.**
   - Why it matters: nothing under To Be Defined exists yet.
   - How to get it: the team names an owner, who writes the specification before any host driver is written.
4. **Frame rate for the ESCs and the servo.**
   - Why it matters: the frame rate decides which pins can share a timer group, and it must stay within each device's maximum accepted update rate.
   - How to get it: the ESC and servo documents, then a team decision.
5. **Power arrangement.**
   - Why it matters: if the Teensy is powered from the vehicle on VIN while USB goes to the Pi, the VUSB-VIN pads must be cut [1].
   - How to get it: a wiring decision; inspect the pads on the installed board.
6. **Board variant and silicon revision.**
   - Why it matters: standard vs lockable, Ethernet or not, and chip "A" vs "B" errata [1].
   - How to get it: read the U1 part number on the board, and the purchase record.
7. **Serial-number uniqueness on the actual units.**
   - Why it matters: the driver should open the Teensy by serial number, so two boards must never report the same one.
   - How to get it: on the host, run `lsusb -v -d 16c0:0483 | grep -i iserial` and `udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID'` for each board, and compare.
8. **ModemManager on the host image.**
   - Why it matters: it can steal or corrupt data on `/dev/ttyACM*` [13].
   - How to get it: on the host, `systemctl status ModemManager`; check that `00-teensy.rules` is installed.
9. **Measured pulse accuracy.**
   - Why it matters: the resolution figures in this document are derived from source code, not measured.
   - How to get it: put a logic analyzer or scope on one ESC channel at the chosen frame rate. Measure the pulse width at 1100, 1500 and 1900 µs command values, plus the jitter.
10. **Watchdog timeout value and servo behavior on timeout.**
    - Why it matters: the firmware must stop the thrusters when commands stop arriving.
    - How to get it: a team decision, informed by the ESC's own signal-loss behavior in [the Basic ESC document](bluerobotics_basic_esc.md).

## Sources

1. "Teensy® 4.1 Development Board", PJRC. https://www.pjrc.com/store/teensy41.html. Used for: processor, memory, USB speeds, pin counts, I2C speeds, voltage tolerance, power, programming button, reset, chip revisions, USB types.
2. "Teensy 4.1 pinout card (front)", PJRC. https://www.pjrc.com/teensy/card11a_rev4_web.pdf. Used for: VIN range (3.6 to 5.5 V), 3.3 V 250 mA, Serial1 and I2C pin labels.
3. "PWM & Tone", PJRC. https://www.pjrc.com/teensy/td_pulse.html. Used for: PWM pin list, timer groups and default frequencies, the shared-frequency rule, resolution behavior.
4. "Wire Library", PJRC. https://www.pjrc.com/teensy/td_libs_Wire.html. Used for: I2C port pins, pull-up guidance.
5. "Using USB Serial with Teensy", PJRC. https://www.pjrc.com/teensy/td_serial.html. Used for: ignored baud rate, buffering and packet behavior, DTR, CDC-ACM, startup waits.
6. "Serial (UART)", PJRC. https://www.pjrc.com/teensy/td_uart.html. Used for: hardware UART baud limits on Teensy 4.
7. "Servo Library", PJRC. https://www.pjrc.com/teensy/td_libs_Servo.html. Used for: Servo and PWMServo pin support, timer and latency behavior.
8. "PWMServo.cpp", Paul Stoffregen (GitHub). https://raw.githubusercontent.com/PaulStoffregen/PWMServo/master/PWMServo.cpp. Used for: 50 Hz frame, 12-bit duty, angle input, LGPL license.
9. "teensy4/pwm.c", PJRC Teensyduino core (GitHub). https://raw.githubusercontent.com/PaulStoffregen/cores/master/teensy4/pwm.c. Used for: FlexPWM and QuadTimer prescaler math, `analogWriteResolution` limits, core license text.
10. "teensy4/clockspeed.c", PJRC Teensyduino core (GitHub). https://raw.githubusercontent.com/PaulStoffregen/cores/master/teensy4/clockspeed.c. Used for: bus clock derivation (150 MHz at 600 MHz).
11. "teensy4/usb_desc.h", PJRC Teensyduino core (GitHub). https://raw.githubusercontent.com/PaulStoffregen/cores/master/teensy4/usb_desc.h. Used for: VID/PID and descriptor strings per USB type.
12. "teensy4/usb_desc.c", PJRC Teensyduino core (GitHub). https://raw.githubusercontent.com/PaulStoffregen/cores/master/teensy4/usb_desc.c. Used for: serial-number derivation from `HW_OCOTP_MAC0`.
13. "00-teensy.rules", PJRC. https://www.pjrc.com/teensy/00-teensy.rules. Used for: udev rules, `ttyACM` match, ModemManager warning.
14. "Teensy Loader app for Ubuntu Linux", PJRC. https://www.pjrc.com/teensy/loader_linux.html. Used for: Linux and Raspberry Pi/Jetson loader builds, udev install.
15. "Teensy Loader, Command Line", PJRC. https://www.pjrc.com/teensy/loader_cli.html. Used for: CLI options, the Linux udev requirement.
16. "teensy_loader_cli.c", Paul Stoffregen (GitHub). https://raw.githubusercontent.com/PaulStoffregen/teensy_loader_cli/master/teensy_loader_cli.c. Used for: bootloader PID `0x0478`, 134-baud soft-reboot request, GPL v3 license.
17. "Production serial number, Teensy 4", PJRC Forum. https://forum.pjrc.com/threads/71991-Production-serial-number-Teensy-4. Used for: who writes `HW_OCOTP_MAC0` (PJRC statement), and the conflicting user claim.
18. "Teensy 4.1 MAC Address", PJRC Forum. https://forum.pjrc.com/threads/62932-Teensy-4-1-MAC-Address?p=252049. Used for: PJRC OUI `04:E9:E5`, per-board unique locked MAC.
19. "i.MX RT1060 Crossover Processors for Consumer Products, Data Sheet Rev. 4", NXP (hosted by PJRC). https://www.pjrc.com/teensy/IMXRT1060CEC_rev4.pdf. Used for: GPIO DC thresholds (VIH, VIL, VOH).
20. "USB Micro-B Cable", PJRC. https://www.pjrc.com/store/cable_usb_micro_b.html. Used for: USB connector type.
21. "pySerial overview", pySerial project. https://pyserial.readthedocs.io/en/latest/pyserial.html. Used for: host-side Python serial library and its platforms.
22. "pySerial Tools", pySerial project. https://pyserial.readthedocs.io/en/latest/tools.html. Used for: `list_ports` fields (`vid`, `pid`, `serial_number`, `location`).
23. "pySerial LICENSE.txt", pySerial project (GitHub). https://raw.githubusercontent.com/pyserial/pyserial/master/LICENSE.txt. Used for: license.
