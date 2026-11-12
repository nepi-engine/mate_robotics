# Pi Integration: Adding nepi_link to Your ROV Pi Script

2026-11-02, from the NEPI team. Updated 2026-11-11: the link is now NEPI Connect, link protocol version 2, set up by a vehicle file. Read [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md) first; this file assumes its handover rules.

## Summary

- On the ROV Pi you copy three things into the folder that holds your Pi script (in rov-26, `rpi/modes/`, beside `main.py`): the [nepi_connect](../Nepi_Connect/pi/nepi_connect/) folder, unchanged; [nepi_link.py](../Nepi_Connect/pi/nepi_link.py), unchanged; and your vehicle file, [vehicles/mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml), renamed `nepi_connect_vehicle.yaml` and placed beside `nepi_link.py`. PyYAML must also be installed on the Pi: `sudo apt install python3-yaml`.
- You add five lines to your script: one import and four calls. [main_py_nepi_link.patch](../Nepi_Connect/pi/main_py_nepi_link.patch) shows them against rov-26 `rpi/modes/main.py`. Each hunk is explained below, so you can find the same spot in your current script.
- nepi_link never opens your serial port, never imports your script and never changes your list of inputs. Every public function catches every error and falls back to passing your inputs through untouched. With no vehicle file, an empty `network.nepi_ip`, no PyYAML, or an invalid value in the vehicle file, it opens no sockets and your script behaves exactly as it does today.

Citations of your code point to github.com/CoralCrusaders/rov-26 at commit `1af4984`. Read each one as "rov-26 does this; check whether your current code still does".

## Terms

- **NEPI**: the edge-computing software on your NEPI computer. A NEPI **driver** connects one device to NEPI; ours for your ROV is an **RBX** (robot) driver, `rbx_nepi_connect`, shown as "NEPI Connect" on NEPI's Drivers page. **Discovery** is the driver step that finds the device and starts the driver's node. **IDX** drivers handle cameras.
- **RUI**: NEPI's browser-based user interface. **NavPose**: NEPI's bundle of navigation and pose data, such as depth and heading.
- **NEPI Connect**: the link between your ROV Pi and NEPI, both sides of it. Its wire contract is [LINK_PROTOCOL.md](../Nepi_Connect/LINK_PROTOCOL.md), version 2.
- **nepi_link**: the module `nepi_link.py` on the ROV Pi, which your script imports. It keeps the four hook functions and hands everything to the `nepi_connect` folder beside it. Older docs call the two together "the bridge".
- **Vehicle file**: the YAML file on the ROV Pi that describes your ROV to NEPI Connect: NEPI's address, your inputs, the caps and signs, and your Teensy's telemetry line. Every key is in [VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md). At start the Pi announces it to NEPI in a **vehicle message**, and NEPI builds its driver from that.
- **Hook**: one line in your script that calls nepi_link.
- **NEPI control, pilot mode, takeover point, status, demand**: as defined in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#terms).
- **Slot**: a position in the 18-value input tuple your script encodes for the Teensy, numbered from 0 in `gamepad_map` order.
- **Patch**: a file that lists changes to another file. The `patch` tool applies it.

## The Files and Where They Go

| File | Goes | Notes |
|---|---|---|
| [nepi_link.py](../Nepi_Connect/pi/nepi_link.py) | Beside the Pi script that writes the Teensy (in rov-26, `rpi/modes/`, beside `main.py`) | Copy unchanged. Your script imports it, and Python finds it because it sits in the script's own folder. |
| [nepi_connect/](../Nepi_Connect/pi/nepi_connect/) | The same folder, as a folder named `nepi_connect` | Copy the whole folder unchanged. `nepi_link.py` hands everything to it. Without it the link stays off and says so. |
| [vehicles/mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml) | The same folder, renamed `nepi_connect_vehicle.yaml` | Your vehicle file. The keys you edit are below. |
| [main_py_nepi_link.patch](../Nepi_Connect/pi/main_py_nepi_link.patch) | Only where you apply it | The five hooks, against rov-26 `rpi/modes/main.py` |
| [test_nepi_link.py](../Nepi_Connect/pi/test_nepi_link.py) | In a mate_robotics checkout, with Python 3 and PyYAML | Unit tests; they load the files in `Nepi_Connect/vehicles/` and `Nepi_Connect/tools/fake_nepi.py`. See [TEST_PLAN.md](TEST_PLAN.md#stage-1-desk-no-teensy) |

PyYAML is required on the ROV Pi ([Python Version and Dependencies](#python-version-and-dependencies)).

nepi_link reads the first vehicle file that exists, in this order (`vehicle_candidates` in `nepi_connect/vehicle.py`):

1. The file named by the environment variable `NEPI_CONNECT_VEHICLE`.
2. `nepi_connect_vehicle.yaml` beside `nepi_link.py`.
3. `~/nepi_connect/config/nepi_connect_vehicle.yaml`.

With none of them, the link stays off and says so. If none exists but a version 1 `nepi_link.yaml` from our 2026-11-02 package does, the link stays off and says it must be converted ([VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md#converting-a-version-1-file)); `mate_coral_crusaders.yaml` already holds the same values, so delete the old file.

```bash
# [topside] from a mate_robotics checkout; replace PIUSER and PI_SCRIPT_FOLDER with yours
scp -r Nepi_Connect/pi/nepi_link.py Nepi_Connect/pi/nepi_connect Nepi_Connect/vehicles/mate_coral_crusaders.yaml PIUSER@192.168.68.22:PI_SCRIPT_FOLDER/
```

```bash
# [ROV Pi]
cd PI_SCRIPT_FOLDER && mv mate_coral_crusaders.yaml nepi_connect_vehicle.yaml
sudo apt install python3-yaml
```

## The Five Hooks

The patch has three hunks holding five changed lines. Against rov-26 `rpi/modes/main.py` it applies cleanly:

```bash
# [ROV Pi] in a copy of your Pi script folder; --dry-run first shows whether the hunks match
patch --dry-run main.py < main_py_nepi_link.patch && patch main.py < main_py_nepi_link.patch
```

If your current script has changed, the hunks may not match. Add the five lines by hand instead, using what each one is for.

| # | Line added | rov-26 place | What it does, and why there | How to find the spot in your script |
|---|---|---|---|---|
| 1 | `import nepi_link` | With the other imports, after `import socket` (`main.py:8`) | Loads the module. Importing opens no sockets and starts no threads, so it is safe anywhere before the other hooks. | Your import block |
| 2 | `+ nepi_link.topside_tag() +` inside the `$RPCTL` string | `transmit_topside_socket`, `main.py:78` | Adds `,NEPI:ON`, `,NEPI:WAIT`, `,NEPI:OFF` or `,NEPI:NOTREADY` before `,*FF`, so the pilot sees the handover state on the topside. It returns an empty string while the link is off, so the line is then unchanged. | The line that builds the text your Pi sends the topside |
| 3 | `nepi_link.start()` | In `main()`, after the two threads start and before `while True:` (`main.py:96-99`) | Reads the vehicle file, prints one line saying whether the link is on, and starts nepi_link's two threads: vehicle and status messages out, demands in. Call it once. A second call does nothing. | Just before your main loop |
| 4 | `gamepad_input_tuple = nepi_link.merge_inputs(gamepad_inputs)` in place of `tuple(gamepad_inputs)` | The main loop, `main.py:101`, just before `nmea_encode` | The only place NEPI's demand enters. It reads the pilot's inputs, applies RB and the takeover point, and returns the tuple to encode: your inputs unchanged in pilot mode, or a copy with NEPI's values in NEPI control. It never modifies the list you pass. | Where your 18-input list becomes the value handed to the encoder for the Teensy. It must be the value that is encoded, and it must run on every pass of the loop. |
| 5 | `nepi_link.on_teensy_line(onboard_data)` | After `ser.readline()` is decoded and stripped, `main.py:105` | Hands each Teensy line to nepi_link for NEPI's status. It never changes the line, so your print and topside forwarding are unaffected. Lines that do not match the vehicle file's `telemetry` section, for you 20-value `$TNCTL` lines, are counted (`nav_bad`) and ignored. | Where you read a line from the Teensy |

The order of hooks 3 and 4 matters only in one way: until `start()` runs, `merge_inputs` passes everything through.

## What nepi_link Assumes About Your Code

Each row is something NEPI Connect takes from rov-26. Most of them now live in your vehicle file, so if your current code differs, you change the vehicle file yourselves ([VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) has every key). The rest is code in the `nepi_connect` folder, which is ours to change; tell us what differs and we will.

| # | Assumption | rov-26 value | In your vehicle file, or in nepi_connect | What to check in your code |
|---|---|---|---|---|
| 1 | The inputs are 18 integers in `gamepad_map` order, the same order as the Teensy's input struct | `rpi/modes/main.py:39-58`; `mcu/modes/new_ctl/new_ctl.ino:130-149` | The `slot` of each entry in `control.inputs` | Your `gamepad_map` and your sketch's input struct still put LX, LY, RX, RY in slots 0 to 3, LB and RB in 12 and 13, and LT and RT in 14 and 15 |
| 2 | Movement inputs: LY surge, LX sway, RY depth rate, LT and RT yaw; RY at 0 means depth hold, both triggers at 0 mean heading hold | `new_ctl.ino:426-435, 465-492` | `control.axes`: surge on `ABS_Y`, sway on `ABS_X`, heave on `ABS_RY`, yaw on `ABS_Z` and `ABS_RZ` | Your sketch still mixes these inputs this way |
| 3 | Sticks run -32767 to 32767, and the sketch zeroes \|value\| < 3000, then passes the rest through unscaled | `JOYSTICK_MAGNITUDE`, `JOYSTICK_DEADZONE_CONST`, `new_ctl.ino:27, 29, 563-575` | `min`, `max` and `deadzone` of `ABS_X`, `ABS_Y`, `ABS_RX` and `ABS_RY` in `control.inputs`; `stick_value` in `nepi_connect/mapping.py` | Your sketch's two constants, and that nothing rescales after the dead zone |
| 4 | Triggers run 0 to `TRIGER_MAGNITUDE`, with values under `TRIGGER_DEADZONE_CONST` zeroed | 1023 and 40 in `mcu/modes/tuning/tuning.ino:28, 30`; 255 and 10 in `new_ctl.ino:28, 30` | `max` and `deadzone` of `ABS_Z` and `ABS_RZ` in `control.inputs`; `trigger_value` in `mapping.py` | Both values equal your sketch's constants, and a full pull on your controller reads the maximum (you said 0 to 1023) |
| 5 | Yaw request = (LT - RT) / `TRIGER_MAGNITUDE` x 180 deg/s, so LT is positive yaw | `new_ctl.ino:481-482` | `control.axes.yaw`, a `trigger_pair` with `inputs: [ABS_Z, ABS_RZ]`: positive yaw goes to LT | Which way LT turns the ROV (`control.axes.yaw.sign`), and [SAFETY.md](SAFETY.md#rov-26-findings) finding 1 |
| 6 | Stick directions: LY pushed forward reads negative on our stock Xbox pad | Measured on our bench | `control.axes.surge.sign` and the other signs | Measure each sign: [TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount) |
| 7 | RB (`BTN_TR`, slot 13) is free: neither sketch reads it | `main.py:53`; `new_ctl.ino:144` is declared, never read | `control.toggle_input: BTN_TR`. Any input no axis, servo or reserved input uses is accepted. | Your sketch still ignores RB (you said it is free) |
| 8 | LB toggles slow mode with a "button available" rule, and slow mode halves surge and sway | `new_ctl.ino:364-379, 466-476` | `control.reserved_inputs: [BTN_TL]`: NEPI never writes LB, and it passes through from the pilot. NEPI Connect does not track slow mode. | If slow mode moved to another input, reserve that input instead |
| 9 | Teensy telemetry is one `$TNCTL` line of exactly 20 values plus a `*XX` field, the first 8 integers | `new_ctl.ino:584-629` | `telemetry.prefix`, `telemetry.fields` (8 `int`, then 12 `float`) and `telemetry.trailer`; `parse_line` in `nepi_connect/telemetry.py` | Count the values in a line from your sketch. PHT values or any other change makes every line `nav_bad`; then change `telemetry.fields` to match. NEPI needs no change: its driver reads the names from the vehicle message. |
| 10 | The main loop builds a fresh tuple on every pass, and a separate thread writes the newest encoded frame every 0.1 s | `main.py:84-88, 99-102` | `merge_inputs`; the 0.25 s stale rule and the 2 s timeout change what `merge_inputs` returns | Your loop calls `merge_inputs` often, at least once per serial write. A frame already encoded keeps going out until the next merge, so a loop that blocks (for example on a serial `readline()` with no timeout, `main.py:23, 105`) also freezes NEPI's last demand. |
| 11 | Your script is the only writer to the Teensy, and the encoder takes exactly the 18 inputs | `main.py:13, 23, 87`; `nmea_encode` | nepi_link has no serial code at all | If your current script sends more than the 18 inputs (for example tuning gains), merge only the 18 inputs and add the rest after |
| 12 | The text to the topside is one `$RPCTL,...,*FF` line every 0.2 s, which your topside script prints | `main.py:75-82`; `topside/modes/main.py:24-34` | `topside_tag` | If your topside script parses that line, it must accept the extra field |
| 13 | The ROV Pi and NEPI share one network with fixed addresses | `config.yaml:1-2`; NEPI at 192.168.68.103 in your own scripts | `network.nepi_ip`; demands from any other address are dropped (`parse_demand` in `nepi_connect/link.py`) | Your current addresses |

## The Toggle: RB in the Input State, and a Topside Hotkey

nepi_link reads RB from slot 13 of the inputs you hand to `merge_inputs`, not from the controller. Anything that sets slot 13 to 1 and then back to 0, on two different passes of your main loop, is an RB press. That is why it works with your controller today and needs no topside change.

You said you would prefer a hotkey on the computer for testing. Here is how one could work. This is our suggestion for your code, and we have not built or tested it.

- Your topside script already sends inputs as `$CTCTL,CODE:state` events, and your Pi script stores each one in its slot (`rov-26 rpi/modes/main.py:60-73`). `BTN_TR` is already a known code there, so a hotkey that sends `BTN_TR:1` and then `BTN_TR:0` looks exactly like an RB press to nepi_link. An unknown code would crash the unmodified Pi script at `main.py:72`.
- Send the press and the release as **two separate messages**, about 0.2 s apart. The Pi script applies every event in one message before its next merge (`main.py:70-72`), so `$CTCTL,BTN_TR:1,BTN_TR:0` in one message would leave slot 13 at 0 and the press would be lost.
- `topside/modes/main.py` waits inside `get_gamepad()` until the controller changes (`:38`). A hotkey therefore needs its own thread or window, and both senders must share `conn` under a lock so their messages never interleave.
- The same rules apply to the hotkey as to RB: it hands over only while the tag reads `NEPI:OFF`, and it takes the ROV back from `NEPI:ON` or `NEPI:WAIT`.

## The Vehicle File

Your vehicle file is [mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml), named `nepi_connect_vehicle.yaml` on the Pi. These are the keys you are likely to edit, with the team value and why. [VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) explains every other key, and its "Worked Example" walks through this file.

| Key | Team value | Why |
|---|---|---|
| `network.nepi_ip` | `192.168.68.103` | Your NEPI's address, from your own scripts. Demands from any other address are dropped. Empty turns the link off. |
| `control.axes.surge.cap`, `control.axes.sway.cap` | 0.2 | Cap on the surge and sway demands, 0 to 1 of full stick. Yours to set: [SAFETY.md](SAFETY.md#choosing-the-caps). |
| `control.axes.heave.cap` | 0.2 | The same for heave (RY, a depth-rate request) |
| `control.axes.yaw.cap` | 0.1 | Cap on the yaw demand, 0 to 1 of a full trigger. 0 is allowed and turns NEPI's yaw off. Set 0 until you have checked finding 1 in [SAFETY.md](SAFETY.md#rov-26-findings). |
| `control.axes.surge.sign` | -1 | Measured on our bench with rov-26 code and a stock Xbox pad: LY pushed forward reads negative |
| `control.axes.sway.sign` | 1 | Not measured; measure it on your ROV |
| `control.axes.heave.sign` | 1 | Not measured. In rov-26, a positive RY asks for a positive depth rate, which is down, so 1 is the expected value if your depth hold works; measure it. |
| `control.axes.yaw.sign` | 1 | Not measured; measure it on your ROV |
| `deadzone` of `ABS_Z` (LT) and `ABS_RZ` (RT) in `control.inputs` | 40 | Must equal your sketch's `TRIGGER_DEADZONE_CONST`; 40 is rov-26 `tuning.ino:30` |
| `max` of `ABS_Z` and `ABS_RZ` in `control.inputs` | 1023 | Must equal your sketch's `TRIGER_MAGNITUDE`; you said your triggers read 0 to 1023, which matches `tuning.ino:28` |
| `control.takeover_fraction` | 0.25 | How far a pilot stick or trigger must move to end NEPI control at once. Above 0 and at most 1, and it must put every takeover point past its input's dead zone. |
| `control.servos` | `{}` (none) | Lets NEPI move up to four claw inputs in NEPI control. You said NEPI does not need to move the claw. [NEPI_SETUP.md](NEPI_SETUP.md#optional-servos) describes the option, and [VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) "Servos" lists the keys. |

The rest of the file describes rov-26 as it is: the 18 inputs, RB (`BTN_TR`) as `control.toggle_input`, LB (`BTN_TL`) in `control.reserved_inputs`, and the `$TNCTL` line under `telemetry`. Change those only where your code differs (table above). `network.status_port` (5700) must equal the driver's `status_port` option on NEPI. NEPI reads `network.demand_port` (5701) from the vehicle message, so there is no driver option to match it.

The robot checks every value when it loads the file (`validate_vehicle` in `nepi_connect/vehicle.py`). One bad value, or a key it does not know, turns the whole link off and prints one message naming the key; it never falls back to a default.

The four signs exist because which way "positive" points depends on your controller, wiring and propellers. The caps and the trigger values are applied on the ROV Pi, after NEPI and before your sketch, so nothing on NEPI can exceed them.

With the team file in place, `start()` prints one line like this before your main loop:

```
nepi_link: On: vehicle mate_coral_crusaders, NEPI 192.168.68.103, status to UDP 5700, demands on UDP 5701, axes surge, sway, heave, yaw, takeover at 0.25 of travel, no servos, telemetry 20 fields (PI_SCRIPT_FOLDER/nepi_connect_vehicle.yaml)
```

Anything else, such as `network.nepi_ip is empty: link off` or `PyYAML missing: sudo apt install python3-yaml`, means the link is off and names the reason.

## Python Version and Dependencies

- Python 3.8 or newer, standard library plus PyYAML (`nepi_connect/control.py` header).
- PyYAML is required: `sudo apt install python3-yaml` on the ROV Pi. Without it the link stays off, prints `PyYAML missing: sudo apt install python3-yaml` once, and your inputs pass through untouched. The vehicle file is nested YAML, and there is no fallback parser.
- nepi_link starts two daemon threads, which end when your script ends, and opens two UDP sockets: one bound to the demand port, one for sending vehicle and status messages.
- It prints at most one line per kind of message every 10 s, so a flood of bad datagrams cannot flood your console.

## Testing on the Desk With fake_nepi.py

[fake_nepi.py](../Nepi_Connect/tools/fake_nepi.py) stands in for NEPI, so you can check your hooked script before NEPI is involved. It uses Python 3.8 or newer and the standard library only, and never goes on the ROV.

- `watch` prints the vehicle message once, and again if it changes. Then it prints one line per status: NEPI control, its reason, whether NEPI counts as ready, each axis input's merged value from the status `out` object (`ABS_Y`, `ABS_X`, `ABS_RY`, `ABS_Z` and `ABS_RZ`), `demand_bad`, the navigation values, and the `raw` values, the first eight of which are the thruster values from the newest Teensy line.
- `hold --surge S --sway S --heave S --yaw S --seconds N` waits for the vehicle message, then sends `active` true demands at 10 Hz with those values, for the axes the vehicle message announced, so NEPI counts as ready and RB hands over. Values are protocol axes: surge forward, sway right, heave down, yaw clockwise.
- Ctrl+C ends `hold` by sending `active` false for 0.5 s, which ends NEPI control at once (`nepi_off`). Ctrl+\ quits with no `active` false, which is how to test the 2 s timeout.
- `--pi` names the ROV Pi, the only address it accepts vehicle and status messages from and the one it sends demands to. nepi_link accepts demands only from `network.nepi_ip`, so for these tests `network.nepi_ip` must name the machine fake_nepi runs on: the topside laptop's 192.168.68.12, or 127.0.0.1 with both on one machine.

The steps and what each should show are in [TEST_PLAN.md](TEST_PLAN.md), stages 1 and 2.
