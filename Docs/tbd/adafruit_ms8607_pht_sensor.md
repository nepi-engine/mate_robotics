# Adafruit MS8607 Pressure Humidity Temperature Sensor Breakout (Product 4716)

## Summary

This Adafruit breakout carries a TE Connectivity MS8607, which holds two sensor dies in one package [2][3]:

- A pressure and temperature (PT) die at I2C address 0x76.
- A relative humidity (RH) die at I2C address 0x40.

Both addresses are fixed. Software reads factory calibration words from the PT die, triggers conversions, and computes pressure and temperature. It reads humidity separately from the RH die, then corrects humidity using the PT die's temperature [3].

The most important fact for a driver author: the PT die uses the same address as the kit's Blue Robotics Bar depth sensor (0x76), so the two cannot share an I2C bus [2][3][11].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Interface | I2C only, up to 400 kHz | [3] |
| Addresses | Pressure/temperature 0x76; humidity 0x40; both fixed | [2][3] |
| Ranges | 10 to 2000 mbar; 0 to 100 %RH; −40 to 85 °C | [1][3] |
| Best resolution | 0.016 mbar; 0.04 %RH; 0.01 °C | [1][3] |
| Accuracy at 25 °C | ±2 mbar (300–1100 mbar); ±3 %RH (20–80 %RH); ±1 °C | [3] |
| Board supply (VIN) | 3 to 5 V; onboard 3.3 V regulator | [2] |
| Logic level | SCL/SDA level-shifted for 3 to 5 V logic; 10 kΩ pull-ups | [2] |
| Connectors | 2 × STEMMA QT / Qwiic (JST SH 4-pin), plus 0.1 in header | [2][8] |
| Chip supply | 1.5 to 3.6 V | [3] |
| Board size | 25.4 × 17.78 mm (from Adafruit's board file) | [8] |

## Physical and Electrical Interface

**Board pins** (Adafruit guide [2]; circuit from Adafruit's board files [8])

| Pin | Function | On-board circuit |
|---|---|---|
| VIN | Power in, 3 to 5 V; "give it the same power as the logic level of your microcontroller" | AP2112K-3.3 regulator input |
| 3Vo | 3.3 V regulator output, up to 100 mA | — |
| GND | Common ground | — |
| SCL | I2C clock; level-shifted for 3 to 5 V logic; 10 kΩ pull-up | BSS138 level shifter |
| SDA | I2C data; level-shifted for 3 to 5 V logic; 10 kΩ pull-up | BSS138 level shifter |
| STEMMA QT ×2 | GND, VIN, SDA, SCL on JST SH 4-pin | Power pin is the VIN net |

**No address jumper.** The board has no address jumper [8], and the chip has no address-select pin [3].

**Sensor chip electrical** [3]:

- **Supply.** 1.5 to 3.6 V. Absolute maximum 3.6 V.
- **Peak current.** 1.25 mA during a PT conversion; 0.45 mA during an RH conversion.
- **Standby current.** 0.03 µA typical.
- **Logic thresholds.** Input high is 80 to 100% of VDD; input low is 0 to 20% of VDD.
- **Board-level current.** Adafruit gives no figure. The board also carries a regulator and a power LED [8].

**Environment.**

- **Operating range.** −40 to 85 °C [3].
- **Overpressure limit.** 6 bar [3].
- **No water rating.** Adafruit and TE give no water, condensation or ingress rating for this part [1][2][3][16].
- **Handling notes.** The datasheet asks that "the sensor opening" be protected from particles and dust during assembly, and warns "Cleaning might damage the sensor!" [3].
- **Placement.** Treat it as a bare, open-cavity board that belongs inside a dry enclosure. That is an inference from the above, not a stated rating.

**Board revision.** Adafruit updated the board's silkscreen in May 2024 [1]. The published board files predate that change [8].

## Communication Protocol

### Pressure and temperature die (address 0x76)

Each command is one byte. ADC reads return 24 bits and PROM reads return 16 bits, both MSB first [3].

| Command | Byte |
|---|---|
| Reset | 0x1E |
| Convert D1 (pressure), OSR 256 / 512 / 1024 / 2048 / 4096 / 8192 | 0x40 / 0x42 / 0x44 / 0x46 / 0x48 / 0x4A |
| Convert D2 (temperature), same OSR order | 0x50 / 0x52 / 0x54 / 0x56 / 0x58 / 0x5A |
| ADC read (then read 3 bytes) | 0x00 |
| PROM read word n (then read 2 bytes) | 0xA0 + 2·n, n = 0 to 6 |

**Conversion time and resolution by oversampling ratio (OSR)** [3]. "Maximum values must be applied to determine waiting times in I2C communication."

| OSR | Conversion time, max (ms) | Pressure resolution (mbar RMS) | Temperature resolution (°C RMS) |
|---|---|---|---|
| 256 | 0.56 | 0.11 | 0.012 |
| 512 | 1.10 | 0.062 | 0.009 |
| 1024 | 2.17 | 0.039 | 0.006 |
| 2048 | 4.32 | 0.028 | 0.004 |
| 4096 | 8.61 | 0.021 | 0.003 |
| 8192 | 17.2 | 0.016 | 0.002 |

**Read rules** [3]:

- An ADC read with no conversion done, or a repeated read, returns 0.
- An ADC read during a conversion returns 0 and corrupts that conversion.

**PROM** [3]:

- Word 0 (0xA0): CRC-4 in bits 15:12; factory data in bits 11:0.
- Words 1 to 6 (0xA2 to 0xAC): C1 to C6, all unsigned 16-bit.

| Word | Coefficient | Meaning |
|---|---|---|
| 1 | C1 | Pressure sensitivity |
| 2 | C2 | Pressure offset |
| 3 | C3 | Temperature coefficient of pressure sensitivity |
| 4 | C4 | Temperature coefficient of pressure offset |
| 5 | C5 | Reference temperature |
| 6 | C6 | Temperature coefficient of the temperature |

**CRC-4 check.** The datasheet's C code for the PT PROM. Compare the result with `(word0 & 0xF000) >> 12` [3]:

```c
unsigned char crc4_PT(unsigned int n_prom[])   // n_prom: 8 words, n_prom[7] used as scratch
{
    int cnt; unsigned int n_rem = 0; unsigned char n_bit;
    n_prom[0] = ((n_prom[0]) & 0x0FFF);         // CRC bits replaced by 0
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else              n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else                  n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    return (n_rem ^ 0x00);
}
```

In Python, mask `n_rem` to 16 bits after each shift, as Adafruit's library does [4].

**First-order calculation** [3]

| Variable | Formula | Notes |
|---|---|---|
| dT | `D2 - C5 * 2^8` | signed 32-bit |
| TEMP | `2000 + dT * C6 / 2^23` | 0.01 °C (2000 = 20.00 °C) |
| OFF | `C2 * 2^17 + (C4 * dT) / 2^6` | signed 64-bit |
| SENS | `C1 * 2^16 + (C3 * dT) / 2^7` | signed 64-bit |
| P | `(D1 * SENS / 2^21 - OFF) / 2^15` | 0.01 mbar (110002 = 1100.02 mbar) |

**Unit-test vector from the datasheet** [3]:

- Inputs: C1–C6 = 46372, 43981, 29059, 27842, 31553, 28165; D1 = 6465444; D2 = 8077636.
- Expected: dT = 68, TEMP = 2000, OFF = 5764707214, SENS = 3039050829, P = 110002.

**Second-order compensation** [3]. TEMP is the first-order value, in 0.01 °C.

```
if TEMP < 2000:                        # low temperature
    T2    = 3 * dT^2 / 2^33
    OFF2  = 61 * (TEMP - 2000)^2 / 2^4
    SENS2 = 29 * (TEMP - 2000)^2 / 2^4
    if TEMP < -1500:                   # very low temperature
        OFF2  = OFF2  + 17 * (TEMP + 1500)^2
        SENS2 = SENS2 + 9  * (TEMP + 1500)^2
else:                                  # high temperature
    T2 = 5 * dT^2 / 2^38
    OFF2 = 0
    SENS2 = 0
TEMP = TEMP - T2
OFF  = OFF - OFF2
SENS = SENS - SENS2
P    = (D1 * SENS / 2^21 - OFF) / 2^15     # recomputed, 0.01 mbar
```

**Libraries agree.** Adafruit's Python and Arduino libraries and TE's reference code all use these constants and branches. They compare against the first-order TEMP and recompute P from the corrected OFF and SENS [4][5][6].

**Related part.** These constants are not the same as either Bar sensor model's (see [the Bar document](../npx/bluerobotics_bar_depth_sensor.md)), even though the command set and the PROM CRC are the same.

### Humidity die (address 0x40)

**Commands** [3]:

| Command | Byte |
|---|---|
| Reset | 0xFE (takes less than 15 ms) |
| Write user register | 0xE6, then 1 data byte |
| Read user register | 0xE7, then read 1 byte |
| Measure RH, hold master (clock held low during measurement) | 0xE5 |
| Measure RH, no hold master (poll; sensor NACKs until done) | 0xF5 |

**User register** [3]:

| Bit | Meaning | Default |
|---|---|---|
| 7 and 0 | Measurement resolution (see the note below) | 00 |
| 6 | Battery state: 1 means VDD below 2.25 V | 0 |
| 2 | On-chip heater: 1 = on | 0 |
| 5, 4, 3, 1 | Reserved; read and keep their values when writing | — |

**Resolution encoding: the datasheet disagrees with every library.**

- The datasheet's own example writes 0x01 and calls it 8-bit resolution [3]. Its register table, read the other way, would make 0x01 OSR 2048.
- TE's reference code and both Adafruit libraries all use a single mapping [4][5][6]:

| Register value | Resolution | OSR |
|---|---|---|
| 0x00 | 12-bit | 4096 |
| 0x81 | 11-bit | 2048 |
| 0x80 | 10-bit | 1024 |
| 0x01 | 8-bit | 256 |

- Use the library mapping. It is the one TE's own code uses (Open Question 4).

**Heater.**

- A soft reset does not clear the heater bit [3].
- The heater adds about 0.5 to 1.5 °C of local heating, and is meant for checking that the RH sensor responds [3].

**Reading humidity** [3]:

1. The sensor returns 3 bytes: data MSB, data LSB (with 2 status bits in the low bits), then a checksum.
2. The status bits must be set to 0 before calculating humidity.
3. The datasheet gives no checksum polynomial. Every library uses CRC-8 with polynomial x^8 + x^5 + x^4 + 1 (0x31), initial value 0, over the two data bytes [4][5][6]. ESPHome writes it as `crc8(bytes, 2, 0, 0x31, true)` [15].

**Measurement time** (maximum) [3]: 15.89, 8.03, 4.08 and 2.12 ms from the highest to the lowest resolution. The datasheet labels these rows "OSR 8192" to "1024", which does not match the register's 4096 to 256 range. Adafruit's Arduino header and SparkFun's library wait 16, 9, 5 and 3 ms [5][7].

**Humidity conversion** [3]. D3 is the 16-bit reading with status bits cleared:

```
RH = -600 + 12500 * D3 / 2^16          # 0.01 %RH
RH = -6 + 125 * D3 / 2^16              # %RH
```

- The output clamps at −6 %RH and 118 %RH, so readings above 100% are possible [3].
- Datasheet example: D3 = 0x7C80 (31872) gives 54.8 %RH [3].

**Temperature compensation of humidity** [3]:

```
RH_compensated = RH + (20 - TEMP) * Tcoeff     # TEMP in °C from the PT die
Tcoeff = -0.18 %RH/°C
```

- The datasheet: "Optimal relative humidity accuracy over [0…+85°C] temperature range is obtained with Tcoeff = -0.18" [3].
- The libraries differ:
  - TE's reference code uses `(25 - temperature) * -0.15` [6].
  - Adafruit's libraries apply no compensation [4][5].
  - ESPHome follows the datasheet [15].
- Use the datasheet form.

## Identification and Detection

- **Presence check.** Probe both addresses for an ACK [6]. Adafruit's Arduino `begin()` fails if either 0x40 or 0x76 does not respond [5].
- **No identity register.** Neither die has a documented chip-ID register [3].
- **The PROM CRC-4 cannot tell the parts apart.** It proves the PT calibration data is intact, not that the part is an MS8607. The Bar sensor's MS5837 chip uses the same address, the same 7-word PROM layout and the same CRC-4 code [3][17]. A Bar sensor on the wrong bus would also pass.
- **The humidity die is the distinguishing check.** An ACK at 0x40 alongside 0x76 is the practical way to confirm an MS8607 [5].
- **Other parts at 0x40.** Adafruit's I2C address list includes other common parts there, such as the INA219, INA260 and PCA9685 [9]. Check for collisions on the chosen bus.

## Startup and Initialization

1. **Reset the PT die.** Send Reset (0x1E) to 0x76 once after power-on, which loads the calibration PROM [3]. The datasheet gives no PT reset time. Adafruit's Arduino library waits 15 ms after its resets [5].
2. **Reset the humidity die.** Send Reset (0xFE) to 0x40, then wait at least 15 ms [3].
3. **Read and check the PT PROM.** Read words 0 to 6 and check the CRC-4 [3].
4. **Configure the humidity die.** Read the user register, change only the resolution and heater bits, and write it back [3][6].
   - Clear the heater bit explicitly, because a soft reset leaves it set [3].
5. **Measure each cycle** [3][6]:
   1. Convert D2, wait, ADC read.
   2. Convert D1, wait, ADC read.
   3. Compute TEMP and P, including second-order compensation.
   4. Send 0xF5 to 0x40, wait or poll, read 3 bytes and check the CRC-8.
   5. Compute RH and compensate it with TEMP.
   6. Discard any ADC result of 0.

**SDA stuck low.** If the chip holds SDA low after power-on, "send several SCLs followed by a reset sequence or … repeat power on reset" [3].

## Data and Commands

| Output | Units | Range | Best resolution | Source |
|---|---|---|---|---|
| Pressure | mbar | 10 to 2000 | 0.016 mbar at OSR 8192 | [3] |
| Temperature | °C | −40 to 85 | 0.002 °C at OSR 8192 | [3] |
| Relative humidity | %RH | 0 to 100 (output −6 to 118) | 0.04 %RH | [1][3] |

**Response time and drift** [3]:

- Humidity response: 5 s to 63% of a step from 33 to 75 %RH, measured with 3 m/s airflow.
- Pressure response: under 5 ms.
- Long-term drift: ±1 mbar/year (pressure), ±0.5 %RH/year (humidity), ±0.3 °C/year (temperature).

Air inside a sealed enclosure is still, so the in-enclosure humidity response will be slower than the 3 m/s figure. That is an inference.

**Maximum rate.** One D1 and one D2 conversion at OSR 8192 take 34.4 ms, plus up to 15.89 ms for a 12-bit humidity reading [3]. That is about 20 complete readings per second before I2C overhead. This figure is derived from the timing tables.

## Configuration and Calibration

**What can be configured.**

- **PT oversampling.** Chosen in each convert command; nothing is stored [3].
- **Humidity.** Resolution and the heater live in the user register [3]. Whether the user register keeps its value across power cycles is not stated. Write it at every startup.

**Calibration.** All calibration is factory-programmed [3]. The driver supplies only:

- the RH temperature-compensation coefficient [3];
- any alarm thresholds the team chooses (Open Question 2).

## Failure Modes and Safety

- **Address conflict with the Bar depth sensor.**
  - The PT die's 0x76 is fixed [3].
  - The Blue Robotics Bar02 and Bar30 also use 0x76 [11].
  - Blue Robotics staff state the Bar30's address "is set by hardware" and cannot be changed [12].
  - Adafruit: "you cannot have two devices with the same address on the same SDA/SCL pins!" Its fix is the TCA9548A multiplexer, at an adjustable address from 0x70 to 0x77 [10]. The multiplexer must not be set to 0x76.
  - The alternative is a separate I2C bus for each 0x76 device, which Blue Robotics also suggests for its Bar sensors [18].
- **Bad reads.** An ADC result of 0 means the read was early, repeated, or overlapped a conversion. Discard it [3].
- **Heater left on.** It skews humidity and temperature, and it survives a humidity soft reset [3]. TE's reference code refuses to compensate humidity while the heater is on [6].
- **Datasheet inconsistencies.** The 06/2017 datasheet contradicts itself on several points [3]:
  - the user-register encoding (see above);
  - the humidity timing labels;
  - the pressure range: 10 to 1200 mbar in one table, 2000 mbar in another.
- **Library quirk.** The Adafruit CircuitPython library reports a PT PROM CRC failure as "CRC Error reading humidity calibration constants", which names the wrong die [4].
- **Condensation.** Blue Robotics staff note that enough condensation can collect inside an enclosure to form droplets, and recommend desiccant [13]. The MS8607 has no stated water rating, so liquid water on the board may damage it [3].

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| Adafruit CircuitPython MS8607 | Driver; runs on Linux through Blinka (`pip3 install adafruit-circuitpython-ms8607`); no humidity temperature compensation | Python | MIT | [2][4] |
| Adafruit_MS8607 | Arduino driver (v1.0.4); no humidity temperature compensation | C++ | MIT | [5] |
| TE MS8607_Arduino_Library | Manufacturer reference code; includes dew point; uses 25 °C / −0.15 humidity compensation | C++ | MIT | [6] |
| SparkFun PHT MS8607 | THIRD-PARTY Arduino driver derived from TE's code | C++ | MIT | [7] |
| ESPHome ms8607 | THIRD-PARTY; follows the datasheet humidity compensation | C++ | GPLv3 | [15] |

ESPHome is GPLv3. A driver under a permissive license should use the datasheet [3] and TE's MIT code [6] as its references.

## Related Parts

- [Blue Robotics Bar depth sensor](../npx/bluerobotics_bar_depth_sensor.md): same I2C address (0x76) [11] and the same PROM CRC [17], so it needs a separate bus or multiplexer channel.
- [Adafruit BNO085 IMU](../npx/adafruit_bno085_imu.md): may share the bus; read that document before putting it behind a multiplexer.
- [PJRC Teensy 4.1](pjrc_teensy_4_1.md): a possible host; its separate I2C ports could keep the two 0x76 devices apart (see that document).
- [Raspberry Pi 5](raspberry_pi_5.md): a possible host with several I2C controllers.

## Open Questions

1. **What is the sensor for on the vehicle?**
   - Why it matters: the likely use is enclosure humidity, temperature and pressure monitoring as an early leak warning. No MS8607 source discusses leak detection.
   - Evidence for the technique: a TI application note describes humidity-based leak detection with a different sensor, using a rising-humidity rate threshold [14]. Blue Robotics discusses enclosure condensation [13].
   - How to get it: a team decision recorded in this repo.
2. **Alarm thresholds.**
   - Why it matters: a leak warning needs numbers. Examples: an absolute RH limit, a rate of RH rise (TI used 10 milli-%RH per second for its 1.7 L test enclosure [14]), an enclosure temperature limit, and an internal pressure change.
   - How to get it: log RH, temperature and pressure in the sealed enclosure for several dry runs, then set thresholds above the normal spread.
3. **Which I2C bus, and how is the 0x76 conflict resolved?**
   - Why it matters: it cannot share a bus with the Bar sensor [3][11].
   - How to get it: a wiring decision. Confirm with `i2cdetect -y <bus>` on each bus: this board should show 0x40 and 0x76, and the Bar sensor's bus should show only its own 0x76.
4. **Humidity resolution encoding on real parts.**
   - Why it matters: the datasheet and all libraries disagree [3][4][5][6].
   - How to get it: write 0x01 to the user register, then time how long the sensor NACKs after 0xF5. About 3 ms means 8-bit, as the libraries say. Repeat for 0x81.
5. **Mounting location inside the enclosure.**
   - Why it matters: a sensor near a warm electronics board reads lower humidity and higher temperature than the enclosure average.
   - How to get it: the enclosure layout. Mount it away from heat sources and where leaking water would first collect, if that is the purpose.

## Sources

1. "Adafruit MS8607 Pressure Humidity Temperature PHT Sensor" (product 4716), Adafruit. https://www.adafruit.com/product/4716. Used for: ranges, resolution, May 2024 board revision.
2. "Adafruit TE MS8607 PHT Sensor" (Learn guide), Adafruit. https://learn.adafruit.com/adafruit-te-ms8607-pht-sensor. Used for: pins, regulator, level shifting, 10 kΩ pull-ups, 3Vo current, both I2C addresses, STEMMA QT, install command.
3. "MS8607-02BA01 PHT Combination Sensor" datasheet (06/2017), TE Connectivity. https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS8607-02BA01&DocType=DS&DocLang=English. Used for: all chip specifications, addresses, commands, timing, PROM, CRC-4, first- and second-order math, test vector, humidity commands, user register, conversion and compensation, clamps, response and drift, handling notes, SDA recovery.
4. "Adafruit_CircuitPython_MS8607", Adafruit (GitHub). https://github.com/adafruit/Adafruit_CircuitPython_MS8607. Used for: Python implementation, resolution encoding, CRC-8, no humidity compensation, CRC error message quirk, MIT license.
5. "Adafruit_MS8607" (Arduino), Adafruit (GitHub). https://github.com/adafruit/Adafruit_MS8607. Used for: `begin()` checks both addresses, resolution encoding, humidity wait times, reset delay, no humidity compensation, MIT license.
6. "MS8607_Arduino_Library", TE Connectivity (GitHub). https://github.com/TEConnectivity/MS8607_Arduino_Library. Used for: manufacturer reference code, presence check, resolution encoding, user-register handling, CRC-8, 25 °C / −0.15 compensation, heater handling, MIT license.
7. "SparkFun_PHT_MS8607_Arduino_Library", SparkFun (THIRD-PARTY, GitHub). https://github.com/sparkfun/SparkFun_PHT_MS8607_Arduino_Library. Used for: humidity wait times, MIT license.
8. "Adafruit-MS8607-PCB", Adafruit (GitHub). https://github.com/adafruit/Adafruit-MS8607-PCB. Used for: regulator and level-shifter parts, connector pinout, no address jumper, board outline.
9. "I2C Addresses: The List", Adafruit. https://learn.adafruit.com/i2c-addresses/the-list. Used for: other common parts at 0x40.
10. "Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout: Overview", Adafruit. https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/overview. Used for: same-address rule, multiplexer fix and its address range.
11. "Bar High-Resolution Depth/Pressure Sensors", Blue Robotics. https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/. Used for: Bar02 and Bar30 at I2C address 0x76.
12. "How to set the address for bar30", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/how-to-set-the-address-for-bar30/2233. Used for: the Bar30 address is set by hardware and cannot be changed.
13. "Condensation and Leak sensor", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/condensation-and-leak-sensor/4936. Used for: enclosure condensation, desiccant recommendation.
14. "Water Ingress Detection Using RH Slew Rate Threshold" (SNAA420A), Texas Instruments (THIRD-PARTY, different sensor). https://www.ti.com/document-viewer/lit/html/SNAA420A/GUID-084660AF-5EED-4B50-A704-D5624E9B1929. Used for: humidity-based leak detection technique and example threshold.
15. "ms8607.cpp" (ESPHome component), ESPHome (THIRD-PARTY, GitHub). https://raw.githubusercontent.com/esphome/esphome/dev/esphome/components/ms8607/ms8607.cpp. Used for: CRC-8 parameters, datasheet-form humidity compensation, GPLv3.
16. "MS8607" product page, TE Connectivity. https://www.te.com/usa-en/product-CAT-BLPS0018.html. Used for: no water or condensation statement.
17. "MS5837-30BA(26)" datasheet REV C2 12/2019, TE Connectivity (hosted by Farnell). https://www.farnell.com/datasheets/2917217.pdf. Used for: the MS5837 uses address 0x76 and the same PROM CRC-4.
18. "Bar High-Resolution Depth/Pressure Sensors Guide", Blue Robotics. https://bluerobotics.com/learn/bar-sensors-guide/. Used for: one Bar sensor per I2C bus; use a second bus or a multiplexer.
