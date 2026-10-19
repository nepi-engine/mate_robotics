# NEPI Connect: Position and Heading Assist for the Coral Crusaders ROV

## Summary

- A NEPI computer (a Raspberry Pi 5 running NEPI) joins the Coral Crusaders' topside network switch. It receives a copy of the ROV's camera video and its sensor data. It runs a camera-based hold that keeps the robot in one place and pointed the same way.
- The pilot turns the hold, called **assist**, on and off with one controller button. The pilot can always take over with the sticks.
- The robot stays sealed. The Teensy firmware does not change. The team's Pi script gains about five lines and one new file that we supply (`nepi_link.py`). The video command gains one extra output.
- Today's firmware holds depth and heading, but not pitch or roll, and has no input for them. So NEPI can hold position and help hold heading. Depth stays with the Teensy's pressure sensor. Pitch and roll need an optional firmware change by the team (see Section 3).
- The exact wire contract is in [LINK_PROTOCOL.md](LINK_PROTOCOL.md). This file explains the plan and does not repeat it.

The team's code was read at commit `1af4984` of github.com/CoralCrusaders/rov-26. Citations look like `file:line` and point to that commit, unless they name a NEPI repository or a file in this repo's `Docs` folder.

## Terms

- **ROV**: remotely operated vehicle, an underwater robot driven by a person. **Topside** is everything at the surface. The **tether** is the cable between them.
- **Teensy**: a small microcontroller board. On this robot it reads the sensors and drives the thrusters.
- **Serial port**: a byte-by-byte link between two devices, here USB between the ROV Pi and the Teensy (`/dev/ttyACM0`).
- **IP address, port**: an IP address names a computer on the network. A port number names one program's mailbox on that computer. A **static address** is one that never changes. `192.168.68.0/24` means the addresses 192.168.68.1 to 192.168.68.254. **DHCP** is the router handing out addresses automatically, from its own range. An **alias** is an extra address on the same network port.
- **Hz, Mbit/s**: Hz means times per second. Mbit/s means millions of bits per second, a measure of bandwidth.
- **Checksum**: a small number computed from a message so the receiver can detect damage. The team's checksum is an XOR (bitwise exclusive-or) of the bytes.
- **Byte order**: whether a multi-byte number is sent most significant byte first (**big-endian**) or least significant byte first (**little-endian**).
- **C extension**: a Python module written in C, for speed.
- **TCP**: a network connection that resends lost data and keeps it in order. **UDP**: single messages with no connection and no resending. Faster, but a message can be lost.
- **Thruster, ESC**: a thruster is a motor with a propeller. An ESC (electronic speed controller) turns pulses from the Teensy into motor power.
- **Surge, sway, depth**: moving forward or back, sideways, and up or down.
- **Yaw, pitch, roll**: turning left or right (yaw; its angle is the **heading**), nose up or down (pitch), and tilting side to side (roll).
- **Open loop**: the input sets power directly and nothing checks the result. **Hold**: a closed loop that measures where the robot is and pushes it back to a target.
- **PID controller**: the standard math for a hold. It turns an error (target minus measurement) into a push. The firmware's `PIDFS` class is one (`mcu/modes/new_ctl/new_ctl.ino:69-128`).
- **Dead zone**: the range around a stick's or trigger's center that the firmware treats as zero.
- **Rising edge**: the moment a button goes from released to pressed. Acting only on the edge makes one press count once.
- **Timeout, watchdog, failsafe**: a timeout treats data as missing once it is too old. A watchdog is code that enforces a timeout. A failsafe is what the robot does when that happens.
- **IMU**: inertial measurement unit, a sensor for rotation and tilt. A **magnetometer** is a compass sensor. Without one, the heading slowly **drifts**, meaning it creeps away from the truth.
- **MJPEG**: video sent as a series of separate JPEG pictures. **Latency** is the delay from camera to screen. **Bandwidth** is how much data per second a link carries.
- **systemd unit**: a small file that tells Linux to start a program at boot and restart it if it stops.
- **ROS**: Robot Operating System, the software framework NEPI is built on. A **node** is one running ROS program. **Twist** is ROS's standard message for a velocity command. **OpenCV** is a widely used image-processing library.
- **NEPI terms.** A **driver** connects one device to NEPI. **Discovery** is the driver step that finds a device, often with a quick check called a **probe**, and starts its node. **IDX** drivers handle cameras and **RBX** drivers handle robots. **Teleop** commands are remote-driving commands sent to an RBX robot. A **ProcessIF** process is NEPI's standard way to run a processing task that can be switched on and off. **Navpose** is NEPI's bundle of navigation and pose data: position, depth, heading, pitch and roll. The **RUI** is NEPI's browser-based user interface.

## 1. The Team's System as Found

```
 TOPSIDE LAPTOP 192.168.68.12           ROV PI 192.168.68.22                TEENSY 4.1
 +-------------------------+ TCP 10110 +---------------------------+  USB  +-------------------------+
 | topside/modes/main.py   |<----------| rpi/modes/main.py (client)| serial| mcu/modes/new_ctl/      |
 |  server, one connection |           |  gamepad_inputs[18]       |       |   new_ctl.ino           |
 |  Xbox pad -> $CTCTL ----+---------->|  every 0.1 s: $RPCTL -----+------>|  dead zones, slow mode  |
 |   (changed inputs only) |           |   (154-byte binary packet)|       |  depth hold (Bar02)     |
 |  prints Pi text <-------+-----------+- every 0.2 s: last line <-+-------+- heading hold (BNO08x)  |
 |                         |           |   as $RPCTL,<line>,*FF    |       |  $TNCTL line, ~4 Hz     |
 | quick_render.sh         | UDP 5600  | quick_stream.sh           |       |  8 thrusters via ESCs   |
 |  FastMJPG receive+render|<----------+- FastMJPG capture video0  |       +-------------------------+
 +-------------------------+ each frame+---------------------------+
                             sent twice
```

**Addresses.** The laptop is 192.168.68.12, the ROV Pi is 192.168.68.22, video uses UDP ports 5600 to 5604, and control uses TCP port 10110 (`config.yaml:1-10`). The scripts do not read `config.yaml`. Each one has the addresses typed in (`topside/modes/main.py:9-10`, `rpi/modes/main.py:10-11`).

**Topside.** `topside/modes/main.py` is a TCP server that accepts a single connection (`topside/modes/main.py:14-20`). It reads the Xbox controller with the `inputs` library, whose `get_gamepad()` waits until something on the controller changes (`:38`). It then sends only those changes, framed as a 4-digit length followed by `$CTCTL,CODE:state,...` (`:37-52`). A second thread prints whatever the Pi sends back (`:24-34`).

**ROV Pi.** `rpi/modes/main.py` connects to the topside and exits if the topside is not listening yet (`rpi/modes/main.py:29-37`).
- It keeps all 18 controller inputs in `gamepad_inputs` (`:20`). `gamepad_map` says which slot each input name uses (`:39-58`).
- The `nmea_encode` C extension packs them into a 154-byte packet: `$RPCTL`, then 18 raw 64-bit integers, then `*XX` (a 2-hex-digit XOR checksum), then a zero byte (`rpi/modes/nmea_encode_c_ext/nmea_encode.c:46-54, 78-91`).
- A thread writes the newest packet to the Teensy every 0.1 s (`main.py:84-88`).
- The main loop reads each Teensy line (`:103-108`). Another thread sends the newest line to the topside every 0.2 s as `$RPCTL,<line>,*FF`, with the Teensy's `$TNCTL` header still inside (`:75-82`). `rpi/modes/tuning.py` strips that header (`tuning.py:118-119`).

**Teensy.** `new_ctl.ino` does all the control:
- It reads the packet (`new_ctl.ino:325-362`), then applies dead zones of 3000 out of 32767 on the sticks and 10 out of 255 on the triggers (`:27-30, 563-582`).
- LB toggles slow mode, which halves stick power (`:364-379`).
- It reads the IMU and the depth sensor (`:381-409`), works out vertical and horizontal power (`:411-511`), and limits the total current (`:521-550`).
- Every 50 passes through `loop()` it prints a 20-value `$TNCTL` line (`:14, 584-629, 641-644`).
- In this commit, `power_thrusters()` is commented out (`:640`).
- The IMU is a BNO08x reporting every 5 ms (`:247, 252`). Heading comes from its *game rotation vector*, which does not use the magnetometer (`:252`), so it drifts by about 0.5 degrees per minute (`Docs/npx/adafruit_bno085_imu.md:248`).
- The depth sensor is an MS5837 set to the Bar02 model, in fresh water (`:232-233`).

**Telemetry rate (estimate).** `read_imu()` waits for both 5 ms IMU reports on every pass (`:381-396`), so one pass takes about 5 ms. A line every 50 passes is about one every 250 ms, or about 4 Hz. This is an estimate from the code; Phase 2 measures it.

**Video.** `quick_stream.sh:1` captures `/dev/video0` at 640x480 and 30 frames per second, and sends from the Pi's port 5600 to the laptop's port 5600. It uses 1400-byte packets, and each frame is sent twice (`SEND_ROUNDS` = 2). `quick_render.sh:1` receives and displays it. FastMJPG uses its own UDP packet format, not one of the standard streaming formats (RTSP, HTTP or RTP) that other video tools read (`FastMJPG/README.md:105, 155`).

### Who holds each axis

| Axis | Who controls it today | What holds it when the pilot lets go | How NEPI can influence it |
|---|---|---|---|
| Surge (forward/back) | Pilot, left stick up/down (LY): open-loop power, gain 0.5 times the slow-mode factor (`new_ctl.ino:465-470`) | Nothing. Power goes to 0 and the robot drifts. | Yes. NEPI's surge demand drives LY whenever the pilot's LY is inside the dead zone. |
| Sway (sideways) | Pilot, left stick left/right (LX): open loop, gain 0.4 times the slow-mode factor (`:472-477`) | Nothing. The robot drifts. | Yes, through LX, with the same rule. |
| Depth | Pilot, right stick up/down (RY), as a depth-rate request (`:426-428`) | The Teensy's depth hold on the pressure sensor. The target is captured when RY returns to the dead zone (`:429-435`). | No, on purpose. NEPI never sends RY. The pressure sensor measures depth better than a camera can. NEPI only reads depth. |
| Yaw (heading) | Pilot, triggers: (LT minus RT) / 255 x 180 deg/s (`:481-482`) | The Teensy's heading hold on the IMU heading, captured when both triggers return to zero (`:484-491`) | Yes. NEPI drives one trigger at a time. While NEPI's yaw is 0, the Teensy holds its own heading. The smallest turn rate is about 7 deg/s. |
| Pitch (nose up/down) | Nobody. No input maps to pitch. Pitch controllers are declared but unused (`:208, 213, 218, 222, 417-418`). | Only the robot's own balance (floats above, weights below) | Not through the controller. It needs a firmware change by the team. NEPI reads pitch from telemetry. |
| Roll (tilt) | Nobody. Roll controllers are declared but unused (`:209, 214, 219, 223, 420-421`). | Only the robot's own balance | Same as pitch. |

## 2. Recommended Architecture

```
                          TEAM SWITCH (topside)
   TOPSIDE LAPTOP -------------+------------- NEPI RPi 5 (NEPI_LAN_IP)
   main.py, quick_render.sh    |               idx_net_stream <-- video copy, UDP 5600+N
   (both unchanged)            |               rbx_rov_link   <-- status, UDP 5700, 10 Hz
                               |                              --> demand, UDP 5701, 10 Hz
                            TETHER             hold process (ProcessIF): video + navpose in,
                               |               surge/sway/yaw demands out to rbx_rov_link
                      ROV PI 192.168.68.22
                      main.py + nepi_link.py: merge NEPI's demand into a COPY of the
                      pilot's inputs just before nmea_encode ---> Teensy (unchanged)
                      FastMJPG: send to laptop (unchanged) + send to NEPI (new)
```

**Why the merge happens in the Pi script.**
- It already holds the pilot's full controller state (`rpi/modes/main.py:20`).
- It is the only program that writes to the Teensy (`:13, 23`). If two programs wrote to one serial port, their bytes would mix and break each other's packets.
- It sees every button press, so the assist button works whether NEPI is on or off.
- The firmware and the topside script stay as they are.

**The alternative: merging on the topside laptop.** NEPI's demands could instead be added in `topside/modes/main.py`. That would be the better choice if the team prefers not to change the Pi, or if the tether cannot carry a second copy of the video (then NEPI would get its video from a relay on the laptop, described below). It has two costs. The topside loop waits inside `get_gamepad()` until the controller changes (`topside/modes/main.py:38`), so it would have to be restructured to send a steady stream. And the 250 ms timeout must live after the merge point. If the tether dropped, the Pi would keep resending NEPI's last demand forever, because nothing downstream of the laptop times out today (Section 8, note 2).

**Rates.** The hold can run at most 10 times a second on every axis, because the Pi writes the Teensy every 0.1 s (`rpi/modes/main.py:88`). Telemetry is slower, about 4 Hz (Section 1). So `nepi_link` sends NEPI a status message every 100 ms on its own timer, whether or not a new Teensy line has arrived. That way NEPI sees the assist button within about 100 ms. NEPI uses depth, heading, pitch and roll from telemetry for its navpose and to check the camera's rotation estimate. The camera, not the IMU, is the main heading source, because the IMU heading drifts and arrives slowly.

**Video.** We add a second `send` output to each FastMJPG command, aimed at NEPI. FastMJPG allows one input and any number of outputs (`FastMJPG/src/FastMJPG.c:636-650`), so the pilot's path is unchanged. NEPI's copy uses one send round, because NEPI only wants the newest frame and can skip a lost one.

- **Bandwidth estimate.** A 640x480 MJPEG frame from a USB camera is typically 30 to 80 KB, depending on JPEG quality and the scene. This is an estimate, not measured on their camera.
  - At 30 frames per second, that is 7 to 19 Mbit/s per send round.
  - The pilot's copy (2 rounds) needs 14 to 38 Mbit/s. NEPI's copy (1 round) needs 7 to 19.
  - Together that is 22 to 58 Mbit/s per camera, plus about 3.5% for packet headers. Phase 1 measures the real number.
  - The tether's link speed is an open question (Section 9).
- **Alternative: relay on the laptop.** Replace `quick_render.sh` with one FastMJPG command that receives, renders, and also sends to NEPI, so only one copy crosses the tether. FastMJPG runs its outputs one after another for each frame (`FastMJPG.c:682-725`). So NEPI's copy would wait for the laptop's display each frame, and NEPI would get no video while the laptop is off.
- **Second alternative.** Replacing FastMJPG with ustreamer would serve MJPEG over HTTP, which NEPI's existing camera driver already reads, but it would change the pilot's video tool.
- **Which camera faces which way matters.**
  - A **downward** camera looking at the pool floor sees surge and sway as the picture sliding up, down, left or right. It sees yaw as the picture rotating, and depth as zoom. That matches the hold's math (shift plus rotation), so it suits position hold and gives heading too.
  - A **forward** camera sees yaw as the whole picture sliding sideways, but sway also slides it sideways, so the two are hard to tell apart. Surge shows only as slow zoom. A forward camera suits heading hold, not position hold.

**Pi services (offered, not required).**
- One systemd unit per camera runs FastMJPG with both outputs and restarts it if it stops.
- If the team wants, a second unit runs `rpi/modes/main.py` with `Restart=always`, because the script exits when the topside is not up yet (`rpi/modes/main.py:31-37`) or the Teensy is missing (`:23`).
- One limit: the topside script accepts only one connection (`topside/modes/main.py:20`). If the Pi script restarts in the middle of a session, the topside script must be restarted as well.

```ini
# excerpt of a camera unit; the [Unit] and [Install] sections are routine
[Service]
WorkingDirectory=<rov-26 folder>
ExecStart=<rov-26 folder>/FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 \
  send 192.168.68.22 5600 192.168.68.12 5600 1400 1000000 2 \
  send 192.168.68.22 5610 <NEPI_LAN_IP> 5600 1400 1000000 1
Restart=always
RestartSec=2
```

**Network.** NEPI takes a static address on 192.168.68.0/24. The team picks a free one (Section 9). There is one NEPI quirk: adding an address on NEPI's IP alias page *replaces* NEPI's list of managed alias addresses instead of adding to it (nepi_engine repo, `nepi_managers/scripts/network_mgr.py:982, 990`). After adding the team's address, check that NEPI's factory alias addresses are still there, and add them back if not.

## 3. For the Coral Crusaders

Thank you for sharing your code. It was easy to follow, and your slow-mode toggle and depth hold gave us exactly the patterns we needed. This section stands on its own. It lists what we ask you to change, why, and how small each change is. Nothing here rewrites your system, and every item is your call.

**What we ask.**

| # | Change | Why | Size |
|---|---|---|---|
| 1 | Add `nepi_link.py` next to `rpi/modes/main.py` | It holds all the NEPI logic, so your script barely changes | One new file. We write and test it. |
| 2 | Add a few lines to `rpi/modes/main.py` (diff below) | Lets NEPI's demand reach the Teensy through your existing packet | About 5 lines |
| 3 | Add the `NEPI_*` keys to `config.yaml` | Holds NEPI's address, the assist button, and the safety limits | About 12 lines |
| 4 | Add one `send` output to each FastMJPG command | Gives NEPI its own copy of the video | One line per camera |
| 5 | Optional: systemd units | Starts video and control at boot and restarts them | Two small files |
| 6 | Pick a free address for NEPI and confirm the assist button | Network and controller setup | Answers only |

**What does not change.** The Teensy firmware, the topside script, the pilot's video, and every existing control. If `NEPI_LAN_IP` is empty, or NEPI is switched off, the robot behaves exactly as it does today. Every `nepi_link` function catches its own errors and falls back to passing your inputs through untouched, so a NEPI problem cannot stop your script.

**The hook.** This diff is for illustration; we will hand you a tested version:

```diff
@@ -6,3 +6,4 @@
 import time
 import serial.tools.list_ports
 import socket
+import nepi_link
@@ -77,3 +78,3 @@
     while(True):
-        nmea: str = "$RPCTL," + str(onboard_data) + ",*FF" # dummy checksum
+        nmea: str = "$RPCTL," + str(onboard_data) + nepi_link.topside_tag() + ",*FF" # dummy checksum
         len_str: str = str(len(nmea)).zfill(4)
@@ -98,9 +99,11 @@
     topside_thread.start()
+    nepi_link.start()
     while True:
         monitor_socket_input()
-        gamepad_input_tuple = tuple(gamepad_inputs)
+        gamepad_input_tuple = nepi_link.merge_inputs(gamepad_inputs)
         nmea_bytes = nmea_encode.nmea_encode(gamepad_input_tuple)
         if(ser.in_waiting > 0):
             try:
                 onboard_data = ser.readline().decode('ASCII').rstrip()
+                nepi_link.on_teensy_line(onboard_data)
                 print(onboard_data)
```

- `start()` reads `config.yaml`. If `NEPI_LAN_IP` is empty, it does nothing.
- `merge_inputs()` returns a new tuple and never changes `gamepad_inputs`.
- `on_teensy_line()` only reads the line.
- `topside_tag()` returns `,NEPI:ON`, `,NEPI:OFF` or `,NEPI:WAIT`, or nothing when the link is off. That puts the assist state on the pilot's screen.

The four lines that matter are the import, `start()`, the merge, and the Teensy line. The topside tag is optional.

**The assist button.** We suggest RB (`BTN_TR`), because neither sketch uses it (`new_ctl.ino` and `tuning.ino` read only LX, LY, RY, LT, RT and LB). A press toggles assist, using the same "button available" pattern as your slow mode (`new_ctl.ino:364-372`). The toggle lives on your Pi, so the button works with or without NEPI. RB still reaches the Teensy unchanged.

**How the pilot stays in charge.** NEPI can only supply three things: left stick up/down for surge, left stick left/right for sway, and one trigger at a time for yaw. It never supplies the right stick or any button. On each of those three axes, the moment the pilot's own input leaves the dead zone, the pilot's value is used. A NEPI demand older than 250 ms counts as zero. Each demand is capped by a limit in your `config.yaml`. Because your firmware zeroes anything inside the dead zone, the bridge skips over it. A stick demand of 0.01 to 1 maps evenly onto 3000 to 32767, a yaw demand of 0.01 to 1 maps evenly onto 10 to 255 on one trigger, and anything below 0.01 becomes 0 (full formulas in LINK_PROTOCOL). Slow mode halves NEPI's surge and sway too, because they go through the same stick math (`new_ctl.ino:465-477`). Which stick direction is forward, right or clockwise is not known yet. We will measure it with you on the bench and set it in `config.yaml`.

**Heading and the 7 deg/s step.** While NEPI's yaw is zero, your Teensy holds the heading it captured. When NEPI wants to turn, it pulls a trigger. When it lets go, your Teensy captures the new heading at that moment (`new_ctl.ino:484-491`). The smallest trigger value past the dead zone is 10 out of 255, which asks for about 7 deg/s (`:30, 482`). At 10 updates per second, the smallest correction is a short pulse of roughly 0.7 degrees or more. So NEPI will correct heading in small pulses and ignore errors under about 1 degree, and your hold does the holding in between. A second effect: the Blue Robotics Basic ESC has its own dead zone, called a dead band, and ignores pulses within 25 µs of stop (`Docs/Motors/bluerobotics_basic_esc.md:11`). The smallest stick value (3000) gives 3000/32767 x 0.5 x 400, about 18 µs on a horizontal thruster. So NEPI's smallest surge demands may not spin a propeller at all. Phase 3 measures the smallest demand that does. Two questions for you: would you accept a smaller trigger dead zone, for example 2 (about 1.4 deg/s)? Or would you prefer a heading-target input, where one unused stick axis (RX) nudges `yaw_target` directly? Both are firmware changes, and both are optional.

### Optional: pitch and roll leveling on the Teensy

Today nothing holds pitch or roll, and no input reaches them (Section 1 table). No change on our side can reach them through the controller. First question: does the robot actually tip in the water? Many ROVs stay level by themselves, with floats on top and weight below. If yours does, skip this.

If it tips, we recommend finishing IMU leveling **on the Teensy**. You already declared the pieces: pitch and roll controllers, targets, and set flags (`new_ctl.ino:208-209, 213-214, 218-219, 222-223`), and unused `pitch_pwr`/`roll_pwr` variables in `calc_vert_power()` (`:417-421`). The Teensy reads the IMU about 200 times a second (`:247, 252`), and the IMU measures gravity directly. NEPI's camera loop runs at most 10 times a second and has no gravity reference. So leveling belongs on the Teensy, and NEPI would only read and report pitch and roll. A sketch in your own style:

```cpp
// in calc_vert_power(), after depth_pwr (illustrative; check axes and signs on the bench)
desired_pitch_rate = pitch_rate_controller.compute(sensor_data.pitch_deg); // pitch_target 0
pitch_pwr_controller.set_target(desired_pitch_rate);
pitch_pwr = pitch_pwr_controller.compute(sensor_data.pitch_deg_s);
// ...the same three lines for roll...
vfl_pwr += pitch_pwr + roll_pwr;   vfr_pwr += pitch_pwr - roll_pwr;
vbl_pwr += -pitch_pwr + roll_pwr;  vbr_pwr += -pitch_pwr - roll_pwr;
```

Another option, not our default: NEPI could command pitch and roll *targets* through an unused input you agree on, on top of the same firmware change. LINK_PROTOCOL reserves `pitch` and `roll` fields for this. The Pi ignores them until the firmware supports them.

## 4. What We Build on the NEPI Side

Nothing here is built yet. Recommended build order:

1. **FastMJPG receive mode in `idx_net_stream`** (in `Drivers/idx_drivers`).
   - Today the driver accepts only `rtsp`, `rtsps`, `http` and `https` URLs (`idx_net_stream_driver.py:47`).
   - Add a URL form such as `fastmjpg://0.0.0.0:5600`, meaning "listen on UDP port 5600", plus a default port (`:48`). Discovery already launches one camera node per URL and names it from host, port and path (`idx_net_stream_discovery.py:180-232, 323-375`), so the new form fits as it is.
   - The new reader keeps the method set the node already calls: `startImageAcquisition`, `stopImageAcquisition`, `getImage`, `getFramerate`, `get_health` (`idx_net_stream_driver.py:190-367`). It reassembles packets as LINK_PROTOCOL describes: it drops bad packets instead of exiting, skips repeat rounds, and keeps only the newest frame, using the frame-counter pattern the driver already has (`:63-70, 307-330`).
   - The probe binds the port, waits for one complete frame, and releases the port before the node starts.
   - The header's capture time gives the latency, provided both clocks are synchronized.
   - The accepted-forms comment in `idx_net_stream_params.yaml:18-29` gets the new form.
2. **`nepi_link.py` and a fake Teensy for the bench** (Section 7). Both are Python and follow NEPI's naming rules.
3. **An RBX driver, `rbx_rov_link`** (in `Drivers/rbx_drivers`), started from `rbx_template_*` with `setup_new_driver.sh` (`DRIVER_NAME="rov_link"`).
   - Discovery listens on UDP 5700 for a valid status from the ROV's address, instead of probing serial ports (`rbx_template_discovery.py:106-121`). It stops listening before it launches the node.
   - The node registers `RBXRobotIF` (`rbx_template_node.py:165-201`) with:
     - empty state, mode and action lists, which the template allows (`:32-37`);
     - axis controls for x, y and yaw only;
     - a navpose built from telemetry depth, heading, pitch and roll;
     - the teleop pair `teleopControlsReadyFunction` (true while the link is up and assist is on) and `setTeleopVelocityFunction`.
   - The latter keeps the newest command with its time. A 10 Hz timer sends it as a demand: surge = linear.x, sway = -linear.y, yaw = -angular.z. linear.z is ignored. Any command older than 250 ms becomes zero.
   - Assist and the per-axis pilot flags are published on one driver topic, `link_status`, until `RBXRobotIF` has fields for them.
4. **Using `RBXRobotIF` as it is.** An earlier analysis expected a missing piece here: `goto_velocity` blocks, refuses new commands while one is active, and sleeps 0.5 s (nepi_engine repo, `nepi_api/src/nepi_api/device_if_rbx.py:1455, 1459, 1472`). But `RBXRobotIF` already has `set_teleop_velocity`. It is a stock `Twist` message with a queue of one, so the newest command wins, and it never blocks (`:644-657, 1278-1296`). So no interface change is needed, and the hold loop does not have to live inside the driver. Two small caveats:
   - The interface logs one info line per teleop message (`:1279`), which is noisy at 10 Hz. A throttled log would be a small engine change.
   - The interface has no teleop timeout. The driver owns the timeout, as `rbx_ardupilot` does with its own (nepi_drivers repo, `rbx_drivers/rbx_ardupilot_node.py:670`).
5. **The hold process**, a `ProcessIF` process (nepi_engine repo, `nepi_api/src/nepi_api/process_if.py:167`) in an app package that does not exist in this repo yet.
   - **Inputs.** It reads the camera image and the RBX navpose and `link_status`.
   - **Reference frame.** When assist turns on, it saves the current image as the reference.
   - **Registration.** For each new frame it estimates the image's shift and in-plane rotation relative to the reference. This is called *registration*. One way is OpenCV's `estimateAffinePartial2D` on matched features, which returns shift, rotation and scale together.
   - **Cross-check.** It compares the rotation with the change in IMU heading since the reference. If they disagree by more than a few degrees, it sends zero yaw and reports the disagreement.
   - **Output.** It turns the errors into surge, sway and yaw demands at up to 10 Hz, with a heading dead band of about 1 degree (Section 3). On an axis the pilot is overriding, it freezes that axis's correction.
   - **Re-grab.** Once the pilot has released all sticks and triggers for about 0.5 s, it takes a new reference, because the pilot moved the robot on purpose.
   - **Lost tracking.** If tracking is lost (too few matched features), it sends zeros and says so in its status.

## 5. Safety Rules

1. The pilot always wins on any axis where their stick or trigger is outside the dead zone.
2. NEPI never supplies the right stick or any button. Depth stays with the Teensy's depth hold.
3. A demand older than 250 ms counts as zero. If the link is down, NEPI's demand is zero.
4. The Pi caps every demand (`NEPI_MAX_SURGE`, `NEPI_MAX_SWAY`, `NEPI_MAX_YAW`). The first pool tests use low caps.
5. Assist starts off. It turns itself off after 2 s with no fresh demand, and only the pilot can turn it back on. NEPI never re-engages silently.
6. A `nepi_link` error can never stop `main.py`. On any error, the pilot's inputs pass through untouched.
7. An empty `NEPI_LAN_IP` restores today's behavior exactly. That is the off switch.
8. The link has no password. The Pi accepts demands only from NEPI's address, but anything on the switch could pretend to be NEPI. Keep the switch closed: no guest devices and no Wi-Fi bridge.
9. The pilot's video must not depend on NEPI. FastMJPG exits on any send error (`FastMJPG/src/VideoUDPSender.c:64-67`), and both outputs share one process. Whether Linux reports an error when NEPI is switched off is untested (a hypothesis), so Phase 1 tests it. If it does, the `Restart=always` unit or the laptop relay protects the pilot's video.
10. The first tests with a NEPI demand run with the propellers out of the water (Phase 3).
11. NEPI sends nothing to the laptop's video port 5600. FastMJPG's receiver exits on a malformed packet (`FastMJPG/src/VideoUDPReceiver.c:71-74, 87-90`).

## 6. Phased Rollout

Each phase must pass before the next starts. The numbers are starting targets for both teams to agree on, not measured results.

**Phase 0: Network.** Give NEPI its static address. Check NEPI's factory alias addresses (Section 2).
*Pass:* NEPI pings the ROV Pi and the laptop with no losses in 100 pings. The laptop opens NEPI's RUI. Control and video work exactly as before.

**Phase 1: Video into NEPI, with measured latency.** Add the second `send` output and a `fastmjpg://` URL in NEPI.
*Pass:*
- NEPI shows the camera at 25 frames per second or more.
- Latency is measured and recorded, either from the header capture time with synchronized clocks, or by filming a phone stopwatch and comparing it with the NEPI screen. Record the median and the worst of 20 readings. The hold design assumes under about 200 ms.
- The pilot's video frame rate is unchanged.
- The pilot's video keeps running when NEPI is switched off.
- Tether bandwidth is recorded with `ip -s link` on the Pi, read twice 10 s apart.

**Phase 2: Read-only telemetry.** `nepi_link` sends status; `rbx_rov_link` shows depth, heading, pitch and roll in NEPI. No demands are sent yet.
*Pass:*
- The telemetry rate is measured from `tn_count` and recorded.
- NEPI's values match the topside printout.
- Turning the robot by hand changes heading in NEPI.
- Status keeps arriving at 10 Hz with the Teensy unplugged, while `tn_age_ms` grows.

**Phase 3: Toggle and assist, propellers out of the water.** First confirm which firmware is installed (Section 8, note 1). Where thrusters are not driven, judge by the `*_pwr` telemetry values.
*Pass:*
- RB switches the topside display between `NEPI:ON` and `NEPI:OFF`. With NEPI off, the display shows `NEPI:WAIT`, then `OFF` after 2 s.
- **Signs.** A test surge demand of +0.1 drives the horizontal thrusters the same way as the pilot's "forward". Sway +0.1 matches "right". Yaw +0.1 matches clockwise. Set the `NEPI_*_SIGN` keys to match.
- **Timeout.** When NEPI stops sending, `out_ly` returns to the pilot's value within 0.35 s.
- **Override, per axis.** The pilot's LY replaces NEPI's surge but leaves NEPI's sway and yaw in place. Likewise for LX and for the triggers.
- Slow mode halves NEPI's surge power.
- Garbage sent to port 5701 changes nothing and raises `demand_bad`.
- The smallest demand that spins a propeller is recorded.
- With the triggers released, turning the robot by hand makes `yaw_rate_target_deg_s` oppose the turn.

**Phase 4: Pool test at a low cap.** Caps are 0.1 for surge, 0.1 for sway and 0.05 for yaw. The pilot keeps their hands on the sticks.
*Pass:*
- Over 60 s in calm water with assist on, the robot drifts clearly less than with assist off, judged in the video against a marker on the pool floor.
- Heading stays within 5 degrees of the start.
- Every pilot override works.
- No swinging back and forth (oscillation) grows over time.
- After that, raise the caps one step at a time.

**Phase 5: Pitch and roll, only if the team adds firmware leveling.**
*Pass:* while hovering for 60 s, pitch and roll stay within 3 degrees of level. After a push, the robot returns to level within about 2 s. NEPI shows pitch and roll in its navpose.

## 7. Bench Setup Without the Robot

All of Phases 0 to 3, except real thrust, can run without touching the robot:

- **ROV Pi stand-in.** A spare Raspberry Pi 5 with a USB webcam as `/dev/video0`, FastMJPG built, and the team's unmodified `rpi/modes/main.py`, later with the hook. It must use address 192.168.68.22, because the scripts have the addresses typed in.
- **A fake Teensy on a pseudo-terminal.** A pseudo-terminal is a pair of software serial ports wired to each other. The Linux tool `socat` creates one: `socat pty,raw,echo=0,link=/dev/ttyACM0 pty,raw,echo=0,link=/dev/ttyFAKE`, run as root, makes `main.py` open the fake port without changes (`rpi/modes/main.py:13`). A small Python script on `/dev/ttyFAKE`:
  - reads the 154-byte `$RPCTL` packets;
  - checks the checksum the way the Teensy does (`new_ctl.ino:306-317`);
  - decodes the 18 little-endian 64-bit integers;
  - turns heading at (LT minus RT) / 255 x 180 deg/s, holds depth;
  - prints 20-value `$TNCTL` lines every 250 ms.
- **Topside.** A laptop at 192.168.68.12 running the team's `topside/modes/main.py` with an Xbox controller, plus `quick_render.sh`.
- **NEPI.** The NEPI Pi at its chosen address, all on one switch.

The fake Teensy is ours to write and lives in this repo. It never goes on the robot.

## 8. Notes for the Team

These are questions, not corrections. You know your robot better than we do.

1. **Which code is on the robot?** In this commit `power_thrusters()` is commented out (`new_ctl.ino:640`), so this `new_ctl.ino` does not drive the thrusters. Is a different build installed? `tuning.ino` expects 20 tuning numbers before the controller inputs (`tuning.ino:127-165`), but `tuning.py` sends only the 18 inputs (`rpi/modes/tuning.py:112-113`). `tuning.ino` also reads triggers as 0 to 1023 (`tuning.ino:28`), where `new_ctl.ino` uses 0 to 255 (`new_ctl.ino:28`). Which range does your controller actually send? `topside/modes/gamepadtest.py` prints it, and the answer can depend on the laptop's operating system and the controller model.
2. **What happens if the link drops?**
   - If the tether or the topside script drops, the Pi keeps sending the last inputs it had (`rpi/modes/main.py:61-64, 84-88`).
   - The Teensy keeps its last packet forever, because only a new packet changes `input_data` (`new_ctl.ino:347`). A stick held at the moment of a drop would keep driving.
   - Have you thought about a timeout on the Teensy, for example zeroing sticks and triggers after 0.5 s with no valid packet? NEPI's link cannot add this for you, because the gap is downstream of our merge.
   - Related: the packet is copied into `input_data` before its checksum is checked (`:347` vs `:356`), so a packet that fails the check is still used. Reading into a temporary buffer and copying only after the check passes would close that.
3. **Is yaw rate on the right gyro axis?**
   - Yaw rate is read from `gyroscope.x` (`new_ctl.ino:387`), but heading is a rotation about the sensor's z axis (`:285`). If the IMU is mounted flat, yaw rate would be `gyroscope.z`.
   - The heading-hold error is also worth a bench check. With target 0, `yaw_rate_controller.compute(shortest_angle(yaw, yaw_target))` asks for a rate of yaw minus target (`:485-491`). If a positive rate means heading increasing, that pushes away from the target. If the hold works in the water, the sign is flipped somewhere else, which is fine.
   - Both matter more now, because assist hands heading back to your hold many times a minute.
4. **`$TNCTL` header.** `main.py` forwards the line with `$TNCTL` still inside (`rpi/modes/main.py:105`), while `tuning.py` strips it (`tuning.py:118-119`). Is that intended? `nepi_link` works either way.
5. **Telemetry rate.** We estimate about 4 Hz (Section 1). Is that enough for your own tuning? A smaller `SERIAL_TRANSMISSION_WRAP` (`new_ctl.ino:14`), such as 10 for about 20 Hz, would also give NEPI fresher depth and heading. USB serial carries that easily.

## 9. Open Questions for the Team

1. Which firmware and Pi script are installed on the robot today? (Section 8, note 1)
   - **Partly answered** (Victoria Zhang, 2026-10-16): the firmware is `mcu/modes/tuning/tuning.ino`. `power_thrusters()` is commented out in the repo because the public repo is an earlier version than what is on the Teensy.
   - Still open: which Pi and topside scripts run with it. No Pi script in the public repo sends the 313-byte frame the repo's `tuning.ino` reads. See [BENCH_SETUP_PLAN.md](BENCH_SETUP_PLAN.md), "What the Team's Code Says", D1 and V1.
2. Which free static address on 192.168.68.0/24, outside any DHCP range, can NEPI use?
   - **Answered** (Victoria Zhang, 2026-10-16): 192.168.68.23. See [BENCH_SETUP_PLAN.md](BENCH_SETUP_PLAN.md), P0 and P8.
3. Is RB free for assist, or would you prefer another button?
   - **Answered** (Victoria Zhang, 2026-10-16): RB is free. For testing stabilization she would prefer a hotkey on the computer itself, so the toggle generalizes to other systems. See [BENCH_SETUP_PLAN.md](BENCH_SETUP_PLAN.md), P9 and D3.
4. Which cameras exist, which way does each face (forward or down), and which `/dev/videoN` is each? Which should NEPI use?
   - **Partly answered** (Victoria Zhang, 2026-10-16): there are four cameras:
     - one faces down, on the claw;
     - one faces forward;
     - one is on the main gripper and sees the pool floor when the claw is rotated to face the bottom;
     - one faces down toward the back claw.
   - Still open: each camera's `/dev/videoN` and port, and which down camera is fixed to the hull. See [BENCH_SETUP_PLAN.md](BENCH_SETUP_PLAN.md), P3, D4, V3 and V5.
5. What is the tether's link type and speed? Our estimate is 22 to 58 Mbit/s per camera with both copies.
   - **Partly answered** (Victoria Zhang, 2026-10-16): camera streaming latency is about 60 ms from the control Pi to the topside computer.
   - Still open: the tether's link type and bandwidth. See [BENCH_SETUP_PLAN.md](BENCH_SETUP_PLAN.md), P3, P8 and V2.
6. Which operating system does the topside laptop run, which controller model do you use, and do the triggers report 0 to 255 or 0 to 1023?
7. Are you comfortable with the Pi change (`nepi_link.py` plus about five lines)? If not, we will plan the laptop-side merge instead.
8. Would you like the systemd units?
9. Does the robot tip in pitch or roll in the water?
10. Would you accept a smaller trigger dead zone, or a heading-target input?
11. Are you interested in Teensy leveling for pitch and roll, and later in NEPI pitch and roll targets?
12. How is the IMU mounted, and does the heading hold turn back toward its target today? (Section 8, note 3)
13. Would you consider a faster telemetry rate, and a comms-loss timeout on the Teensy?
14. When can we have pool time for Phases 3 and 4, and who will pilot?
15. Will assist be used in competition runs, or only in practice?

## References

1. Coral Crusaders ROV code, commit `1af4984`: github.com/CoralCrusaders/rov-26. Used for: every `file:line` citation without a repo name.
2. [LINK_PROTOCOL.md](LINK_PROTOCOL.md). Used for: ports, message fields, rates, timeouts, mappings, the video packet format.
3. [Adafruit BNO085 IMU](../Docs/npx/adafruit_bno085_imu.md). Used for: game rotation vector drift.
4. [Blue Robotics Basic ESC](../Docs/Motors/bluerobotics_basic_esc.md). Used for: the ±25 µs dead band around stop.
5. nepi_engine repo: `nepi_api/src/nepi_api/device_if_rbx.py`, `nepi_api/src/nepi_api/process_if.py`, `nepi_managers/scripts/network_mgr.py`. Used for: teleop and goto behavior, ProcessIF, the IP alias behavior.
6. This repo: `Drivers/idx_drivers/idx_net_stream_*`, `Drivers/rbx_drivers/rbx_template_*`, `setup_new_driver.sh`. Used for: the NEPI-side components.
