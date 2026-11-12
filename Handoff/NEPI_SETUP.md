# NEPI Setup: The ROV Driver on Your NEPI

2026-11-02, from the NEPI team. This file covers your NEPI computer only. [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md) explains what the driver does on the link; [VIDEO.md](VIDEO.md) covers cameras.

## Summary

- The `rbx_nepi_connect` driver, named "NEPI Connect" in the RUI, presents your ROV to NEPI as a robot. It listens for nepi_link's vehicle message and status, sends demands back, shows the handover state on the RUI Robots page, and turns the Teensy's telemetry into NEPI NavPose data (depth and orientation).
- Installing it means copying five files from [Drivers/rbx_drivers/](../Drivers/rbx_drivers/) into NEPI's driver folder, removing the old driver's files, then enabling the driver in the RUI. `deploy_drivers.sh` does the copy. Below is exactly what it needs, and the one part we cannot settle for a NEPI on a Raspberry Pi.
- The link is now version 2 of the protocol, and neither side accepts version 1. Update nepi_link on the ROV Pi ([PI_INTEGRATION.md](PI_INTEGRATION.md)) and this driver together.
- The driver changes nothing on the ROV unless the pilot hands the ROV over with RB.

## Terms

- **NEPI**: the edge-computing software on your NEPI computer, built on ROS. A **node** is one running ROS program, and a **topic** is a named channel that nodes publish on.
- **RUI**: NEPI's browser-based user interface, at `http://192.168.68.103:5003` on your NEPI.
- **Driver**: the part of NEPI that connects one device. Each driver has a params file, a discovery script and a node. NEPI's driver manager runs discovery for every enabled driver.
- **Discovery**: the driver step that finds a device and starts its node. Ours listens for one valid vehicle message from your ROV Pi.
- **RBX**: NEPI's driver type for robots, shown on the RUI Robots page. **Teleop** commands are remote-driving commands sent to an RBX robot.
- **IDX**: NEPI's driver type for cameras.
- **NavPose**: NEPI's bundle of navigation and pose data: position, depth, heading, pitch, roll and their rates.
- **SVX**: NEPI's driver type for single servos.
- **NEPI control, pilot mode, vehicle file, vehicle message, status, demand**: as defined in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#terms).

## The Driver Files

| File | What it is |
|---|---|
| [rbx_nepi_connect_params.yaml](../Drivers/rbx_drivers/rbx_nepi_connect_params.yaml) | The driver's manifest: its name in the RUI ("NEPI Connect", group MATE), its files and classes, and the two discovery options |
| [rbx_nepi_connect_discovery.py](../Drivers/rbx_drivers/rbx_nepi_connect_discovery.py) | `NepiConnectDiscovery`: listens for a vehicle message, then starts the node built from it |
| [rbx_nepi_connect_node.py](../Drivers/rbx_drivers/rbx_nepi_connect_node.py) | `NepiConnectNode`: the RBX robot. Its `NepiConnectLink` class holds the link, and `ImuMount` the IMU correction. |
| [rbx_nepi_connect_servo_node.py](../Drivers/rbx_drivers/rbx_nepi_connect_servo_node.py) | `NepiConnectServoNode`: one SVX servo, used only with the optional servos below |
| [deploy_drivers.sh](../Drivers/rbx_drivers/deploy_drivers.sh) | Copies the files in this folder to a NEPI |

The two node files must stay executable (`chmod +x`), because NEPI starts them as programs. They are executable in the repo, and `rsync -a`, which the script uses, keeps that.

## Installing the Driver Files

### What deploy_drivers.sh needs

From reading the script:

| Item | What the script does |
|---|---|
| `NEPI_REMOTE_SETUP` | Required, or the script stops. `1`: run it on another computer, such as your topside laptop. `0`: run it on the NEPI host itself. |
| Target address | It sets a built-in address, which is our bench device, then always replaces it with the value of `NEPI_IP`. So `NEPI_IP` decides the target, and the built-in address is never used. With `NEPI_IP` unset, remote mode stops with `Remote setup requires env. variable NEPI_TARGET_IP be assigned`. |
| SSH key | Always `/home/$USER/.ssh/nepi_default_ssh_key` on the computer running the script. It sets this path itself, so an environment value is ignored. Your `nepi_disp.sh` uses a key at that same path. |
| Copy 1: NEPI's source tree | As user `nepihost`, SSH port 22, to `/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_drivers/rbx_drivers/` on the target. With `NEPI_REMOTE_SETUP=0` this is a local copy to that path instead. A later NEPI build installs the drivers from there. |
| Copy 2: live | As user `nepi`, SSH port 2222, into the NEPI container at `/opt/nepi/nepi_engine/lib/nepi_drivers/`, the folder NEPI's driver manager reads drivers from. It runs in both modes. Your `nepi_disp.sh` reaches your NEPI container the same way: user `nepi`, port 2222, that key. |
| What it copies | Every file in the folder it sits in, itself included, with `rsync -a`, which keeps permissions |
| What it deletes | Nothing. A file that is no longer in the folder, such as the old driver's files, stays at both destinations until you remove it (below). |
| After the copy | Nothing: no build and no restart |
| Its success messages | It prints `Build Updates Deployed` and `Live Updates Deployed` even when a copy failed. Each check reads the exit status of an `echo`, and the live copy's errors go to `/dev/null`. Check the result yourself (below). |

### The commands

```bash
# [topside] from a mate_robotics checkout; 192.168.68.103 is your NEPI
cd Drivers/rbx_drivers && NEPI_REMOTE_SETUP=1 NEPI_IP=192.168.68.103 ./deploy_drivers.sh
```

The driver was renamed from `rbx_mate_rov_link` to `rbx_nepi_connect`. Both would bind status port 5700, so the old files must go. If the old driver is enabled, disable it first (**MATE ROV Link** on the Drivers page), then remove its files from the live folder:

```bash
# [topside] remove the old driver's files from the live folder
ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key nepi@192.168.68.103 'rm -f /opt/nepi/nepi_engine/lib/nepi_drivers/rbx_mate_rov_link_*'
```

If copy 1 worked on your NEPI, remove the same files from `/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_drivers/rbx_drivers/` on the host too, or a later NEPI build installs them again.

```bash
# [topside] check that the live copy landed: five files, the two node files executable, and no old rbx_mate_rov_link file
ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key nepi@192.168.68.103 'ls -l /opt/nepi/nepi_engine/lib/nepi_drivers/ | grep -e rbx_nepi_connect -e rbx_mate_rov_link -e deploy_drivers'
```

The live copy alone puts the driver where NEPI reads it.

### What we could not determine for a NEPI on a Raspberry Pi

`deploy_drivers.sh` and [Drivers/GETTING_STARTED.md](../Drivers/GETTING_STARTED.md) were written for our NEPI layout: a host user `nepihost` with a source tree under `/mnt/nepi_storage/nepi_src`, and the container's SSH on port 2222. Your own script shows that the container side exists on your NEPI. Those files do not tell us:

- whether your NEPI's host has the `nepihost` user and that source tree, so copy 1 may fail on your NEPI;
- whether NEPI's driver manager lists a newly copied driver at once, or only after a NEPI restart;
- whether files copied live into the container survive a NEPI software update or a container rebuild.

We would rather not guess at these. Ask us and we will work them out with you for your NEPI.

## Enabling the Driver

1. Start your ROV Pi script with nepi_link on. Discovery waits for the ROV Pi's vehicle message, so the ROV Pi must already be sending ([PI_INTEGRATION.md](PI_INTEGRATION.md)).
2. Open the RUI, go to the Drivers page under System, and filter the list to Robots. Select **NEPI Connect**.
3. Under Driver Options, check the two discovery options (next section), then turn on **Enable/Disable Driver**. Options saved for the old MATE ROV Link driver do not carry over, because the driver's package name changed.
4. Within a few seconds the driver manager runs discovery. It binds the status port, waits up to 1.5 s for one valid vehicle message from `host` (`PROBE_TIMEOUT_SEC`), closes the port, and starts the node, built from that vehicle message. The node then holds the port for as long as it runs.

How discovery behaves after that (`NepiConnectDiscovery.discoveryFunction` and `checkOnDevice`):

- If the node process stops, the next discovery cycle cleans it up and listens again. A launch is retried at most once every 10 s (`NODE_LOAD_TIME_SEC`).
- Changing any discovery option stops the node, and it starts again with the new values. The node's shutdown handler (`cleanup_actions`) sends one `active` false demand, so NEPI control ends at once if it was on. A node killed without running it leaves that to the ROV Pi's 2 s timeout.
- The ROV going quiet does not stop the node. Its link reports down, and teleop stops being accepted.
- Only one program on NEPI can hold the status port. If another program holds it (for example a copy of `fake_nepi.py`), discovery hears nothing and never starts the node.
- A ROV Pi still running version 1 of nepi_link is never discovered. Discovery logs once that the robot speaks link protocol version 1.
- If the ROV Pi's vehicle message changes while the node runs (for example after you edit a cap or add a servo in the vehicle file), the node keeps running on the first one and warns once that the driver must be disabled and enabled.

## Driver Options and Settings

**Discovery options**, on the Drivers page ([rbx_nepi_connect_params.yaml](../Drivers/rbx_drivers/rbx_nepi_connect_params.yaml)):

| Option | Default | What it does |
|---|---|---|
| `host` | `192.168.68.22` | Your ROV Pi's address. The driver accepts vehicle and status messages only from it and sends demands only to it. |
| `status_port` | 5700 | The port on NEPI that the vehicle message and status arrive on. It must equal `network.status_port` in the ROV Pi's vehicle file. |

There is no demand port option. The ROV Pi announces its demand port (`network.demand_port`, 5701) in its vehicle message, and the driver sends demands there.

The defaults already match your ROV Pi address and the team vehicle file, so normally nothing needs changing.

**Robot settings**, on the robot's settings panel on the Robots page (`CAP_SETTINGS` and `FACTORY_SETTINGS` in the node):

| Setting | Default | What it does |
|---|---|---|
| `teleop_movement_enabled` | `TRUE` | `TRUE` makes the Robots page offer Teleop as a control type. `FALSE` makes the driver refuse every teleop command, whatever sends it, including your own code. |
| `imu_mount` | The vehicle file's `telemetry.imu_mount`: `X_FORWARD_Y_LEFT` in the team file | Where the IMU board's printed X and Y arrows point in the ROV. The driver uses it to turn the Teensy's angles and gyro rates into the ROV's frame. `X_FORWARD_Y_LEFT` is a flat board with X forward, and passes values through unchanged. A value saved on this panel wins over the vehicle file's. |

If these two settings do not appear on the settings panel, tell us. It would mean the driver and your NEPI version disagree on how settings are declared, and without `teleop_movement_enabled` NEPI can never drive the ROV.

## What Appears in the RUI

Once discovery starts the node:

- **Robots page.** A robot named `nepi_connect_mate_coral_crusaders`, that is `nepi_connect_` plus `vehicle.name` from the vehicle file (unless you give it an alias in NEPI).
  - Its state reads `PILOT` or `NEPI` (`RBX_STATES`), refreshed within about 0.2 s of a change on the ROV Pi.
  - **Set State PILOT** hands the ROV back to the pilot. **Set State NEPI** is refused, since only the pilot's RB hands it over.
  - **Stop** makes every demand neutral, while NEPI keeps the ROV.
  - The control types include Teleop while `teleop_movement_enabled` is `TRUE`.
  - Modes, actions, goto, home, battery and manual motor control are empty, because the ROV has none of them on the NEPI side.
- **NavPose.** Depth, and orientation with its rates in the ROV's frame, at up to 10 Hz (`NAVPOSE_UPDATE_RATE`). There is no position or GPS. On our bench, depth from a Bar02 reached the RUI NavPose panel through nepi_link and this driver, and rose when the sensor face went into water. Orientation has not been tested, because our bench Teensy has no IMU.
- **Logs.** The driver logs `Link up`, `Link down`, `NEPI control on` and `NEPI control off (reason)` as they happen, and a warning if the ROV Pi's vehicle message changes.

## Choosing imu_mount

You told us the BNO085 is on its side with its X axis as the yaw axis, so X points straight up or straight down. Which way Y points we do not know. Pick the row that matches your board; Z follows from X and Y by the right-hand rule, so it is a cross-check.

| Board X arrow | Board Y arrow | Board Z then points | `imu_mount` |
|---|---|---|---|
| Up | Forward | Left | `X_UP_Y_FORWARD` |
| Up | Back | Right | `X_UP_Y_BACK` |
| Up | Left | Back | `X_UP_Y_LEFT` |
| Up | Right | Forward | `X_UP_Y_RIGHT` |
| Down | Forward | Right | `X_DOWN_Y_FORWARD` |
| Down | Back | Left | `X_DOWN_Y_BACK` |
| Down | Left | Forward | `X_DOWN_Y_LEFT` |
| Down | Right | Back | `X_DOWN_Y_RIGHT` |

The other 16 of the 24 options cover the other square mounts. The setting changes only what NEPI shows. Your Teensy's heading hold still uses the raw angles, so with the board on its side it needs its own fix in your firmware ([SAFETY.md](SAFETY.md#rov-26-findings), finding 6). If you fix the mount in your firmware, set `imu_mount` back to `X_FORWARD_Y_LEFT`.

To confirm the choice on the NavPose display, use the driver's frame: x forward, y left, z up (`ImuMount`). In that frame:

- With the ROV level and still, pitch and roll read near 0, not near ±90°.
- A flat turn of 90° counter-clockwise, seen from above, raises yaw by about 90°, and the yaw rate reads positive while it turns.
- Nose down gives a positive pitch, and right side down gives a positive roll.

[TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount) has the full procedure. A wrong mount usually shows at once as pitch or roll far from 0 at rest.

## Teleop From the Robots Page

With `teleop_movement_enabled` at `TRUE` and the robot in state `NEPI`, choose Teleop as the control type and use the keyboard. In the current RUI, each held key adds a full ratio (1) on one axis, re-sent every 200 ms, and releasing the keys sends neutral. The driver drops any command older than 0.5 s, so a stalled browser goes neutral too. The caps in the ROV Pi's vehicle file decide how hard each key actually pushes: with the team file's 0.2, a held W sends LY 8713 of 32767 ([SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#demand-to-stick-mapping)).

The current RUI's default keys send these ROS values, and the driver maps them as ROS defines them (y positive left, yaw rate positive counter-clockwise):

| Key | Sends | ROV, once the signs are set right |
|---|---|---|
| W, S | x +1, -1 | Forward, back |
| R, F | z +1, -1 | Up, down |
| E, Q | y +1, -1 | **Left**, right |
| D, A | yaw rate +1, -1 | **Counter-clockwise** (turns left), clockwise |

E and D therefore move the ROV opposite to their key names (strafe right, turn right) with this driver. Measure your signs with `fake_nepi.py`, whose axes are unambiguous, not with these keys ([TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount)). On our bench, W/S, Q/E and A/D each drove the thruster during NEPI control, but we did not check directions there.

## Topics Your Own Code Would Use

Everything sits under the node's namespace, `/nepi/<device_id>/nepi_connect_mate_coral_crusaders`, where `<device_id>` is your NEPI's device ID. Find it with:

```bash
# [NEPI] in a shell inside the NEPI container, where rostopic works
rostopic list | grep nepi_connect
```

In the table, `<dev>` stands for the device name, `nepi_connect_mate_coral_crusaders`.

| Topic | Type | Direction | What it carries |
|---|---|---|---|
| `<node>/<dev>/rbx/set_teleop_velocity` | `geometry_msgs/Twist` | Into the driver | The streaming command a hold loop publishes: `linear.x` forward, `linear.y` left, `linear.z` up, `angular.z` counter-clockwise, each a ratio from -1 to 1. Other fields are ignored. It acts only in state `NEPI` with `teleop_movement_enabled` `TRUE`. Publish at least every 0.5 s or the demand goes neutral. The queue holds one message, so only the newest counts. |
| `<node>/link_status` | `std_msgs/String`, JSON | Out of the driver | `NepiConnectLink.get_health()`, every 0.1 s: `running`, `link_up`, `nepi_control`, `control`, `control_reason`, `pilot` (true or false per axis), `demand_age_ms`, `nav_age_ms`, `status_bad`, `servos`, `raw`, `vehicle`, `kind`, `cameras`, `vehicle_changed`. Wait for `nepi_control` true before commanding. |
| `<node>/<dev>/npx/navpose/depth` | `nepi_interfaces/NavPoseDepth` | Out | `depth_m`, `depth_m_per_sec` |
| `<node>/<dev>/npx/navpose/orientation` | `nepi_interfaces/NavPoseOrientation` | Out | `roll_deg`, `pitch_deg`, `yaw_deg` and their `_per_sec` rates, in the ROV frame set by `imu_mount` |
| `<node>/<dev>/npx/navpose` | `nepi_interfaces/NavPose` | Out | The whole NavPose |
| `<node>/<dev>/rbx/go_stop` | `std_msgs/Empty` | In | Stop: neutral demands, NEPI keeps the ROV |
| `<node>/<dev>/rbx/set_state` | `std_msgs/Int32` | In | 0 (`PILOT`) hands the ROV back; 1 (`NEPI`) is refused |

The teleop topic and its message type are registered by NEPI's `RBXRobotIF` under `<node>/<dev>/rbx`, and the node passes them to `NepiConnectLink.set_teleop_velocity`. `link_status` is the node's own topic. NEPI 3.4 puts a device's topics under its device name, which for this driver repeats the node name, so the doubled `nepi_connect_mate_coral_crusaders` is correct.

Two cautions for code that reads the NavPose topics:

- While telemetry is missing or older than 0.5 s, the driver keeps publishing the last NavPose. Judge freshness by `nav_age_ms` on `link_status`. (Before NEPI 3.4 the `timestamp` field was a 32-bit float of seconds, far too coarse for this; in 3.4 it is a 64-bit float.)
- The Teensy's line rate limits how fresh these values can be ([SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#what-nepi-needs-from-teensy-telemetry)).

A quick look from the container:

```bash
# [NEPI] replace <device_id>; Ctrl+C stops each one
rostopic echo /nepi/<device_id>/nepi_connect_mate_coral_crusaders/link_status
rostopic echo /nepi/<device_id>/nepi_connect_mate_coral_crusaders/nepi_connect_mate_coral_crusaders/npx/navpose/depth
```

## Optional Servos

The team vehicle file leaves this out, because you said NEPI does not need to move the claw. It is here so you know it exists.

- **What it does.** NEPI can write up to four of your claw inputs (RX, the D-pad or a button) during NEPI control. Each one is a servo name under `control.servos` in the vehicle file, with its input, for example `servos: {claw_pitch: {input: ABS_HAT0Y}}`; our [VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) "Servos" lists the keys.
- **How it runs on NEPI.** Discovery reads the servo names from the vehicle message, and starts one SVX servo node per name, named `nepi_connect_mate_coral_crusaders_servo_<servo>`. Each servo then gets its own page on NEPI.
- **Rules.**
  - A servo value goes out only during NEPI control.
  - The pilot's own input wins on that input whenever it is off neutral.
  - Every value is dropped when NEPI control ends, so each handover starts at neutral.
  - Changing the list on the ROV Pi needs the driver disabled and enabled again; the driver warns once when the vehicle message changes.
- **Tested.** On our bench, a servo moved from the RUI during NEPI control, using our own throwaway sketch, not your claw code.
