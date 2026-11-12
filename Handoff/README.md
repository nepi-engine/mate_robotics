# NEPI Integration Handoff for the Coral Crusaders ROV

2026-11-02, from the NEPI team, for Victoria Zhang and the Coral Crusaders. Updated 2026-11-11: the link is now NEPI Connect, link protocol version 2, and your settings live in a vehicle file.

## What This Is

NEPI adds one thing to your ROV: a computer on your topside switch that can see your ROV's telemetry and video, and fly it gently when the pilot hands it over with RB. It can hold position with a camera-based process you write or we write together, or be driven from NEPI's browser page. It never talks to your Teensy, never takes the ROV without the pilot's RB, and gives it back at once on RB, on a stick pushed past a quarter of its travel, or when its demands stop. Everything else stays yours: your Pi script remains the only program that writes the Teensy, and your sketch keeps all mixing, depth and heading hold, current limiting and every thruster safety limit. You decided to do the final integration yourselves without sharing your newest code, and this package is written so you can. It names every file we built, explains how the pieces connect, lists every assumption our code makes about yours, and stages the tests from the desk to the water. Nothing in it asks you to send us code.

Citations of your code point to github.com/CoralCrusaders/rov-26 at commit `1af4984`, the newest version we have seen. Read each one as "rov-26 does this; check whether your current code still does".

## Terms

- **NEPI**: the edge-computing software that runs on your NEPI computer, built on ROS (Robot Operating System).
- **RUI**: NEPI's browser-based user interface, served by the NEPI computer at port 5003.
- **Driver**: the part of NEPI that connects one device. **Discovery** is the driver step that finds a device and starts the driver's node (one running program).
- **RBX**: NEPI's driver type for robots. Ours, `rbx_nepi_connect` ("NEPI Connect" on NEPI's Drivers page), presents your ROV to NEPI as a robot.
- **IDX**: NEPI's driver type for cameras. Ours, `idx_net_stream`, can read your FastMJPG video.
- **NavPose**: NEPI's bundle of navigation and pose data, such as depth and heading.
- **NEPI Connect**: the link between your ROV Pi and NEPI, both sides of it.
- **nepi_link**: `nepi_link.py`, the file your Pi script imports. It hands everything to the `nepi_connect` folder beside it. Both go on your ROV Pi.
- **Vehicle file**: the YAML file on your ROV Pi that holds your settings and describes your ROV to NEPI. Yours is [mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml).
- **NEPI control**: the mode in which the ROV follows NEPI's demands, after the pilot presses RB. Our older docs call it "assist".

## Reading Order

1. This file.
2. [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md): how everything connects, the handover rules, the stick mapping and the messages.
3. [SAFETY.md](SAFETY.md): what our code enforces, what stays yours, the ten rov-26 findings and the deadband arithmetic.
4. [PI_INTEGRATION.md](PI_INTEGRATION.md) and your vehicle file, [mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml): adding nepi_link to your Pi script.
5. [NEPI_SETUP.md](NEPI_SETUP.md): the driver on your NEPI, its settings, and the topics your own code would use.
6. [VIDEO.md](VIDEO.md): your relay method and ours, side by side.
7. [TEST_PLAN.md](TEST_PLAN.md): four stages from the desk to the water.

## File Map

Every file you need from this repo. Paths are links, so they open on GitHub (nepi-engine/mate_robotics).

| File | Runs on | What it does | Need |
|---|---|---|---|
| [Nepi_Connect/pi/nepi_link.py](../Nepi_Connect/pi/nepi_link.py) | ROV Pi | The four hook functions your script calls; hands everything to the `nepi_connect` folder | Required |
| [Nepi_Connect/pi/nepi_connect/](../Nepi_Connect/pi/nepi_connect/) | ROV Pi, beside `nepi_link.py` | The bridge: vehicle and status messages to NEPI, demands from NEPI, the RB toggle and the merge | Required |
| [Nepi_Connect/vehicles/mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml) | ROV Pi, as `nepi_connect_vehicle.yaml` beside `nepi_link.py` | Your vehicle file: your settings | Required |
| [Nepi_Connect/VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) | None | Every vehicle file key, with your file as the worked example | Reference |
| [Nepi_Connect/pi/main_py_nepi_link.patch](../Nepi_Connect/pi/main_py_nepi_link.patch) | ROV Pi | The five hook lines, against rov-26 `rpi/modes/main.py` | Reference |
| [Nepi_Connect/vehicles/mate_bench.yaml](../Nepi_Connect/vehicles/mate_bench.yaml) | None | Our bench vehicle file, with our bench NEPI's address | Reference |
| [Nepi_Connect/pi/test_nepi_link.py](../Nepi_Connect/pi/test_nepi_link.py) | Any computer with PyYAML, in a checkout of this repo | Unit tests for nepi_link and the `nepi_connect` folder | Optional |
| [Drivers/rbx_drivers/rbx_nepi_connect_params.yaml](../Drivers/rbx_drivers/rbx_nepi_connect_params.yaml) | NEPI | The robot driver's manifest and discovery options | Required |
| [Drivers/rbx_drivers/rbx_nepi_connect_discovery.py](../Drivers/rbx_drivers/rbx_nepi_connect_discovery.py) | NEPI | Listens for your ROV's vehicle message, then starts the node | Required |
| [Drivers/rbx_drivers/rbx_nepi_connect_node.py](../Drivers/rbx_drivers/rbx_nepi_connect_node.py) | NEPI | The robot: link, demands, NavPose, `imu_mount` | Required |
| [Drivers/rbx_drivers/rbx_nepi_connect_servo_node.py](../Drivers/rbx_drivers/rbx_nepi_connect_servo_node.py) | NEPI | One servo page per NEPI servo; unused unless your vehicle file lists servos | Optional |
| [Drivers/rbx_drivers/deploy_drivers.sh](../Drivers/rbx_drivers/deploy_drivers.sh) | Topside | Copies the robot driver files to a NEPI ([NEPI_SETUP.md](NEPI_SETUP.md#installing-the-driver-files)) | Optional |
| [Drivers/idx_drivers/](../Drivers/idx_drivers/) (`idx_net_stream_params.yaml`, `_discovery.py`, `_node.py`, `_driver.py`, `deploy_drivers.sh`) | NEPI | Network video, including FastMJPG, as NEPI cameras | Optional |
| [Nepi_Connect/tools/fake_nepi.py](../Nepi_Connect/tools/fake_nepi.py) | Topside, or any computer | Stands in for NEPI for desk and bench tests | Optional |
| [Nepi_Connect/tools/fastmjpg_viewer.py](../Nepi_Connect/tools/fastmjpg_viewer.py) | Any computer with a display | Shows a FastMJPG stream without FastMJPG | Optional |
| [Nepi_Connect/tools/sample_link_rates.py](../Nepi_Connect/tools/sample_link_rates.py) | NEPI | Measures telemetry rate and age | Optional |
| [Nepi_Connect/tools/test_rbx_nepi_connect.py](../Nepi_Connect/tools/test_rbx_nepi_connect.py) | Any computer with PyYAML, in a checkout of this repo | Unit tests for the driver's link, against the real nepi_link and your vehicle file | Optional |
| [Nepi_Connect/LINK_PROTOCOL.md](../Nepi_Connect/LINK_PROTOCOL.md) | None | The wire contract, version 2: the vehicle, status and demand messages, and video to NEPI. It matches the code. | Reference |
| [Nepi_Connect/DRIVER_HANDOFF.md](../Nepi_Connect/DRIVER_HANDOFF.md) | None | The telemetry targets, and firmware suggestions for you in "Problems to fix" rows 1, 2 and 6 | Reference |

Other folders in this repo are not part of your setup. `Apps/nepi_app_robot_stab/scripts/` and `nepi_disp.sh` are yours.

## Status

As of 2026-11-02. "Our bench" is our replica of your system: rov-26 code, a real Teensy 4.1, one Blue Robotics Basic ESC with a T200, and an Xbox controller.

| Part | Built | Tested on our bench | Not yet tested |
|---|---|---|---|
| nepi_link | Yes | 108 unit tests, all pass (82 before the move to protocol version 2). The whole pilot chain with your rov-26 scripts. The hook with the ESC supply off: surge 0.1 gave LY 5706 and 34 in `$TNCTL` columns 5 to 8, and 17 in slow mode. RB into NEPI control. Surge sign -1 measured. | Under the current RB design: the stick takeover exit, RB handback, and the 2 s silence timeout. The sway, heave and yaw signs. |
| rbx_nepi_connect driver | Yes | 60 unit tests, all pass, run against the real nepi_link and nepi_connect (46 before version 2). Under NEPI: depth from a Bar02 reached the RUI NavPose panel. RUI Teleop W/S, Q/E and A/D drove the thruster during NEPI control. | The Set State PILOT exit. Orientation and `imu_mount` with a real BNO085. Anything in water. |
| Optional servos | Yes | A servo moved from the RUI during NEPI control, using our own throwaway bench sketch, not your claw code | Servo return to neutral on handback |
| idx_net_stream, `fastmjpg://` mode | Yes | Worked under NEPI with one camera, 2026-10-19. On a simulated link with 3% loss, 1 send round delivered 12 of 60 frames and 2 rounds 58 of 60. | Your four cameras and your tether |
| A stabilization (hold) process | No | None | None. Your `optical_flow.py` gives camera displacement in pixels, and [NEPI_SETUP.md](NEPI_SETUP.md#topics-your-own-code-would-use) names the command topic a hold loop would publish. |
| Your current code | None | We have not seen it | Every assumption in [PI_INTEGRATION.md](PI_INTEGRATION.md#what-nepi_link-assumes-about-your-code) |

The unit-test counts are from `python3 -B -m unittest -v test_nepi_link` in `Nepi_Connect/pi` and `python3 -B -m unittest -v test_rbx_nepi_connect` in `Nepi_Connect/tools`, run on 2026-11-11 after the move to protocol version 2. The bench and NEPI results in this table are from the version 1 code of 2026-11-02; the behavior they test did not change.

On 2026-11-11 the link moved to protocol version 2 and took the name NEPI Connect: the driver was renamed, your settings moved to a vehicle file, and the ROV now announces itself to NEPI. The handover behavior, the caps, the signs and every timing are unchanged. The bench results above were measured with the version 1 code.

## Values Only You Can Set

| Value | Where it is set | How to find it | Default |
|---|---|---|---|
| NEPI's address | `network.nepi_ip` in your vehicle file, `nepi_connect_vehicle.yaml` | Your NEPI | 192.168.68.103 in the team file; empty (link off) in the code |
| ROV Pi's address | Driver option `host` | Your ROV Pi | 192.168.68.22 |
| Caps on surge, sway, heave and yaw | `control.axes.surge.cap`, `sway.cap`, `heave.cap`, `yaw.cap` | [SAFETY.md](SAFETY.md#choosing-the-caps), then stages 3 and 4 of the test plan | 0.2, 0.2, 0.2, 0.1 |
| Surge sign | `control.axes.surge.sign` | [TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount) Part A; confirm with your pad | -1 in the team file (measured on our pad) |
| Sway, heave and yaw signs | `control.axes.sway.sign`, `heave.sign`, `yaw.sign` | The same procedure | 1 |
| Trigger dead zone and range | `deadzone` and `max` of `ABS_Z` (LT) and `ABS_RZ` (RT) in `control.inputs` | Must equal `TRIGGER_DEADZONE_CONST` and `TRIGER_MAGNITUDE` in the sketch you fly | 40 and 1023 in the team file |
| How far a pilot input must move to take the ROV back | `control.takeover_fraction` | Your pilots' preference, above the dead zones | 0.25 |
| IMU mount, as NEPI sees it | Driver setting `imu_mount`, which starts at `telemetry.imu_mount` in your vehicle file | The board's X and Y arrows ([NEPI_SETUP.md](NEPI_SETUP.md#choosing-imu_mount)), then [TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount) Part B | `X_FORWARD_Y_LEFT` (flat) |
| Whether NEPI may drive at all | Driver setting `teleop_movement_enabled` | `FALSE` refuses every NEPI movement command | `TRUE` |
| Thruster pin map and reversals | Your firmware and wiring | Your build | rov-26: `new_ctl.ino:50-66` |
| Current limits | Your firmware | `AMP_LIMIT`, `HOR_AMP_LIMIT`, `VERT_AMP_LIMIT` ([SAFETY.md](SAFETY.md#rov-26-findings), finding 2) | rov-26: 20 A, split 10 and 10 |
| IMU orientation for your own heading hold | Your firmware | [SAFETY.md](SAFETY.md#rov-26-findings), finding 6 | rov-26: flat-board conversion |
| Telemetry line format, including any PHT values | Your firmware, and `telemetry.fields` in your vehicle file, which must list the same columns in order | Count the values in one `$TNCTL` line | rov-26 `new_ctl.ino`: 20 values |
| Telemetry rate and decimals | Your firmware | `SERIAL_TRANSMISSION_WRAP` and `Serial.print` precision ([SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#what-nepi-needs-from-teensy-telemetry)) | rov-26: every 50 loops, 2 decimals |
| Link-loss timeout and IMU-reset recovery on the Teensy | Your firmware | [SAFETY.md](SAFETY.md#rov-26-findings), findings 3 and 4 | rov-26: none |
| Which camera NEPI uses, its ports and send rounds | Your FastMJPG commands | [VIDEO.md](VIDEO.md) | Your camera ports 5600 to 5604 |
| NEPI servos | `control.servos` in your vehicle file | Only if you want NEPI to move a claw input | None |

## Two Ways Forward

1. **Take it and integrate it yourselves.** Follow the reading order, then the test plan stage by stage. You never need to send us code. If something does not behave as these files say, tell us what you saw: a status line, a `$TNCTL` line, the line nepi_link prints at start. We will answer from that. If something on our side has to change, tell us what and we will change it in this repo. A longer `$TNCTL` line no longer needs us: you list the new columns in `telemetry.fields` in your vehicle file, and NEPI reads them from the ROV's vehicle message.
2. **Walk through it with us first.** We go through the package together, and you run stages 1 and 2 of the test plan on your own machines while we watch and answer questions. Your code stays on your machines; we see only what you choose to show.

Either way, ask us whenever a file here is unclear, and especially where it says to ask us.

## About Our Older Notes

The documents in [Nepi_Connect/](../Nepi_Connect/) (`README.md`, `BENCH_SETUP_PLAN.md`, `DRIVER_HANDOFF.md`) were our working notes while the design changed. Parts of them describe earlier designs: a different toggle, per-axis pilot override, and no heave. Where they differ from these files or from the code, this folder and the code are current. `LINK_PROTOCOL.md` (version 2) and `VEHICLE_FILE.md` in the same folder are not working notes: they are current and match the code.

The NEPI team
