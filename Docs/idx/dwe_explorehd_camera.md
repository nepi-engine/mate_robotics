# DeepWater Exploration exploreHD (400 m) Camera

## Summary

The exploreHD is a 1080p, 30 fps underwater USB camera rated to 400 m. It is a standard USB 2.0 UVC device that streams H.264, MJPEG or YUY2 with no vendor driver [2]. On Linux each camera creates four `/dev/video` nodes: MJPEG and YUY2 come from the first node of the group and H.264 from the third [3].

The most important facts for a driver author:

- **Two pairs of USB IDs are in the field.** Match both [12].
- **H.264 encoder settings are vendor extension-unit controls**, not standard V4L2 controls. These are bitrate, GOP and CBR/VBR [12][17].
- **No source confirms a unique USB serial number per unit.** DWE's own software identifies each camera by its USB port path instead [8][12].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Interface | USB 2.0 High Speed, UVC compliant, V4L2 compatible | [2] |
| USB IDs | `0x0C45:0x6366` (marked "Legacy VID" in DWE code) or `0x3961:0x2100` | [12] |
| Formats | H.264 (Baseline), MJPEG, YUY2 | [2] |
| Max mode | 1920x1080 at 30 fps (H.264 or MJPEG); YUY2 limited to 5 fps at 1080p | [2][3] |
| Video nodes per camera | 4; MJPEG/YUY2 on the first, H.264 on the third | [3] |
| Supply | 5 V over USB; up to 260 mA (H.264 mode) | [2][4] |
| Sensor | 1/2.9 in Sony Exmor CMOS, 2.8 µm pixels, rolling shutter | [2] |
| Lens and field of view | 2.65 mm f/1.9 fisheye; 138° H / 70° V / 168° D in air; about 82° H in water | [2] |
| Depth rating | 400 m, with a 50% safety margin | [2] |
| Connector | 4-pin JST (5 V, D-, D+, GND) or 6-pin Cobalt, depending on variant | [1][2] |
| Cable | About 1 m by default; 5 m maximum for USB 2.0 | [2] |
| Operating temperature | -20 to 85 °C | [2] |

## Physical and Electrical Interface

**Housing.**

- Body: 6061-T6 aluminum with a hard-anodized coating [2].
- Mass: 84 g in air and 44 g in water [2].
- Ingress protection: "IP69K+ equivalent" [2].
- Depth rating: 400 m, with a 50% safety margin [2].

**Front window.** The two DWE sources disagree. The spec page lists "Standard Depth-Resistant Glass" [2]. The product page says the front element is quartz and "withstands pressure exceeding 1,700 PSI" [1]. Neither affects the driver.

**Connector and pinout.** The JST variant's 4 wires are [2][4]:

| Colour | Signal | Gauge |
|---|---|---|
| Red | 5 V | 24 AWG |
| White | D- | 28 AWG |
| Green | D+ | 28 AWG |
| Black | Ground | 24 AWG |

The 6-pin Cobalt cable (COB-1461) carries the same four signals on pins 1 to 4, ground, +5 V, D- and D+, and pins 5 and 6 are not connected [2].

**Wiring warnings** [4]:

- Swapping +5 V and ground "can permanently damage your device".
- The D+ and D- wires must be twisted together to prevent interference.

**What ships with the camera.** Epoxy and WetLink variants ship with a PAP-04V-S JST housing and a JST-to-USB Type-A adapter [1].

**Cable** [2]:

- Polyurethane jacket.
- 28 AWG twisted data pair (90 Ω ± 15%) and 24 AWG power conductors.
- Braided and foil shielding with a drain wire.
- About 1 m by default, 5 m maximum for USB 2.0.

**Power by mode** [2]:

| Mode | Typical current | Maximum current | Operating power |
|---|---|---|---|
| MJPEG | 180 mA | 200 mA | 0.9 W |
| H.264 | 250 mA | 260 mA | 1.2 W |

**How many cameras one host can power.** The sources disagree:

- A DWE staff post: a Raspberry Pi "can only supply enough power for 3 cameras from our testing" [24].
- Blue Robotics: "typically" up to 2 exploreHD cameras on a Raspberry Pi at one time [26].

**DWE 7-Port Camera Hub** (sold separately, optional):

- It is a USB 2.0 hub, so all cameras on it share one 480 Mbit/s link [10].
- Up to 3 cameras need no external power; more than 3 need an external 5 V supply [10].

## Communication Protocol

**USB and UVC.** The camera is a UVC-compliant USB 2.0 High Speed device. It needs no proprietary driver and uses the standard Linux UVC driver through V4L2 [2].

**USB identity.** DWE's current software (dweOS) lists both ID pairs as "exploreHD" and runs the same code for each [12]:

```
# Legacy VID
(0x0C45, 0x6366): DeviceMetadata("exploreHD", DeviceType.EXPLOREHD),
# exploreHD
(0x3961, 0x2100): DeviceMetadata("exploreHD", DeviceType.EXPLOREHD),
```

- The `0x3961:0x2100` pair was added in April 2026 [16].
- A dweOS issue confirms that cameras with the old IDs exist in the field and broke when only the new IDs were checked [14].
- In the Linux `usb.ids` database, `0c45:6366` is "Microdia Webcam Vitade AF", and vendor `0x3961` is not listed [20].
- So `lsusb` may name the camera as Microdia, or not name it at all. DWE's own detection one-liner greps for "explorehd", "microdia webcam vitade af" or "0c45" [4].

**Video nodes** [3]:

- The kernel "typically creates four device nodes (e.g., /dev/video0 through /dev/video3)".
- MJPEG and YUYV are "Usually found on the first node of the group".
- Hardware H.264 is "Usually found on the third node of the group".
- DWE does not say what the second and fourth nodes are. They are probably UVC metadata nodes, since the Linux UVC driver exposes metadata "through metadata video nodes" [22]. That is an inference.

**Formats, resolutions and frame rates** (exploreHD 3.0) [2]

| Resolution | MJPEG and H.264 (fps) | YUY2 (fps) |
|---|---|---|
| 1920x1080 | 10, 15, 20, 25, 30 | 5 |
| 1280x720 | 10, 15, 20, 25, 30 | 5, 10 |
| 800x600 | 10, 15, 20, 25, 30 | 5, 10, 15 |
| 640x480 | 10, 15, 20, 25, 30 | 5 to 30 (5, 10, 15, 20, 25, 30) |
| 640x360 | 10, 15, 20, 25, 30 | 5 to 30 (5, 10, 15, 20, 25, 30) |
| 352x288 | 10, 15, 20, 25, 30 | 5 to 30 (5, 10, 15, 20, 25, 30) |
| 320x240 | 10, 15, 20, 25, 30 | 5 to 30 (5, 10, 15, 20, 25, 30) |

- **Why YUY2 is slow.** YUY2 is limited by USB bandwidth: "At 1080p, YUYV is limited to 5 FPS" [3].
- **Stream sizes.** H.264 defaults to 10 Mbit/s; DWE lowered the default from 15 to 10 Mbit/s in March 2022 [9]. MJPEG can reach 60 Mbit/s per camera (January 2022 log entry) [9].
- **Selecting H.264 in a pipeline.** DWE's GStreamer examples read H.264 from the third node and parse it with `h264parse` [3].

**Set the pixel format before the resolution** on the first node, which defaults to YUYV. From DWE's OpenCV example: "MJPG needs to be set, before resolution. Pixel format is always selected first" [5].

**Standard controls.**

- The camera exposes ordinary UVC image controls through V4L2 [7]:
  - Brightness, contrast, saturation, hue and gamma.
  - Sharpness, gain and backlight compensation.
  - Automatic white balance and white balance temperature.
  - Power line frequency.
  - Auto exposure, absolute exposure time, and dynamic frame rate.
- Auto exposure offers only "Manual Mode" and "Aperture Priority Mode", because "All DWE cameras are fixed aperture" [7].
- DWE's auto white balance is its own algorithm, tuned for underwater color [7].
- Ranges, steps and defaults are not published; read them on the device (Open Question 1).
- A DWE developer says it is expected that writing a control another control has disabled returns "Permission denied" [15].

**H.264 extension-unit controls.**

- **These are not standard controls.** The H.264 encoder settings are not V4L2 controls. dweOS sets them through the Linux UVC `UVCIOC_CTRL_QUERY` call with a hard-coded unit ID and selector [12]. The kernel documents that such values "need to be hardcoded in the application or queried … by parsing the UVC descriptor" [21].
- **Exchange sequence.** Each is a two-step exchange on an 11-byte buffer [12][17]:
  - **Write:** SET_CUR `{0x9A, command, 0…}`, then SET_CUR `{value bytes…}`.
  - **Read:** SET_CUR `{0x9A, command, 0…}`, then GET_CUR.

| Name | Unit | Selector | Command | Value layout | Range used by dweOS | Source |
|---|---|---|---|---|---|---|
| Bitrate | 4 (user) | 0x02 (H.264) | 0x02 | uint32 big-endian, bits per second | 0.1 to 15 Mbit/s, default 10 | [12] |
| Group of pictures (GOP) | 4 | 0x02 | 0x03 | uint16 | 0 to 29, default 29 | [12] |
| Bitrate mode | 4 | 0x02 | 0x06 | uint8: 1 = constant, 2 = variable | Default variable | [12] |

- **Which node.** dweOS sends these to the third node of the group, the H.264 node [12].
- **GOP meaning.** DWE's legacy tool README: "A GOP of 0 indicates full MJPEG streaming; A GOP of 29 indicates full H.264 compression" [17].
- **Extension-unit GUIDs.** The legacy header defines two GUIDs [17][18]:
  - System unit: `{0x70,0x33,0xf0,0x28,0x11,0x63,0x2e,0x4a,0xba,0x2c,0x68,0x90,0xeb,0x33,0x40,0x16}`
  - User unit: `{0x94,0x73,0xDF,0xDD,0x3E,0x97,0x27,0x47,0xBE,0xD9,0x04,0xED,0x64,0x26,0xDC,0x67}`
- **No reported ranges.** The legacy control table declares the H.264 selector with SET_CUR and GET_CUR only, so the camera does not report minimum, maximum or default values. A driver must hard-code ranges [17].
- **DWE's own documentation disagrees with its code.** DWE's GStreamer guide shows a standard V4L2 control: `v4l2-ctl -d /dev/video2 --set-ctrl=video_bitrate=5000000` [3]. But DWE's spec page links its "Bitrate Control Code" to the extension-unit tool [2], dweOS uses the extension unit [12], and the mainline Linux UVC driver maps no bitrate control [23]. Treat the `video_bitrate` control as unverified (Open Question 1).

## Identification and Detection

**What DWE does.** dweOS finds cameras like this [12]:

1. Enumerate `/sys/class/video4linux`.
2. Query each node's capabilities.
3. Group the nodes by their USB `bus_info` path.
4. Read `idVendor` and `idProduct` from sysfs.
5. Match against the two ID pairs above.

**Cameras are keyed by bus path.** dweOS keys every camera, and every saved setting, by USB bus path. DWE describes this as "Bus ID tracking", so cameras "retain their specific settings even after a system reboot" [8][12].

**Chip check (optional).** dweOS reads an ASIC register through extension unit 3, selector 0x01, and expects chip ID `0x92` for the exploreHD family [12]. DWE's legacy code names that chip ID "SN9C292" [17]. That points to a Sonix SN9C292-series bridge chip; DWE's documents do not name the chip.

**Serial number: not confirmed.**

- No DWE documentation, code or issue found in this research mentions the USB serial-number string or its uniqueness [11][12][14].
- dweOS reads a 64-byte "USB string descriptor" from the camera's flash and shows it in its UI as "Firmware String". Nothing says whether that string is the USB serial number [12].
- DWE's legacy guide says the video node number "won't change as long as the USB device doesn't get unplugged" [19]. Node numbers are therefore not a stable identity across replugs.
- If two units report the same serial, they cannot be told apart after a replug except by which port they are on. A driver should key cameras by USB port path, as DWE does, until real units are checked (Open Question 2).

## Startup and Initialization

- **No setup needed to stream.** The camera is plug-and-play: "Because it is fully UVC compliant, it requires absolutely no proprietary drivers" [2].
- **DWE's suggested Linux checks.** `lsusb`, `ls -l /dev/video*`, and `sudo dmesg -w | grep usb`, watching for errors such as `device descriptor read/64, error` [4].
- **Format first.** Set the pixel format before the resolution [5].
- **Encoder settings may not persist.** DWE's legacy tool notes that the extension-unit settings "must be adjusting each time", and recommends a startup script [18]. dweOS re-applies saved settings to each camera at startup, keyed by bus path [12]. Whether current firmware keeps them across power cycles is not stated (Open Question 3).

## Data and Commands

| Item | Node | Values | Source |
|---|---|---|---|
| MJPEG or YUY2 video | First node of the group | See the format table | [2][3] |
| H.264 video (Baseline) | Third node of the group | See the format table | [2][3] |
| Standard image controls | Standard V4L2 controls | Ranges read from the device | [7] |
| H.264 bitrate | Extension unit 4, selector 0x02, command 0x02 | 0.1 to 15 Mbit/s (dweOS range) | [12] |
| H.264 GOP | Extension unit 4, selector 0x02, command 0x03 | 0 to 29 (dweOS range) | [12] |
| H.264 bitrate mode | Extension unit 4, selector 0x02, command 0x06 | 1 constant, 2 variable | [12] |

**Latency.** DWE's legacy documentation gives USB latency of 33 ms ± 3 for MJPEG or H.264 [19].

## Configuration and Calibration

**Firmware.**

- Camera firmware is flashed with DWE's Firmware Loader, which is Windows-only. DWE says a Linux tool is "coming soon" [6].
- "Flashing firmware for the wrong board can permanently brick your device." [6]
- Only one camera should be plugged in while flashing [6].

**Image and encoder settings.** No DWE source says these persist on the camera. dweOS stores them on the host [12]. Its "Reset Controls" restores the image settings [7].

**Optics, for calibration.** Focal length 2.65 mm and pixel pitch 2.8 µm [2]. The lens is a fisheye, so an ordinary pinhole calibration model will not fit well. DWE publishes no intrinsic calibration.

**Field of view.** In air: 138° horizontal, 70° vertical, 168° diagonal. In water: about 82° horizontal [2]. Because the in-water field of view is much narrower than in air, calibrate in water for underwater use.

## Failure Modes and Safety

- **Several cameras on one Linux USB controller.**
  - DWE documents a "nondeterministic issue related to bandwidth saturation" with "anywhere between 1-6 cameras". Compressed streams reported their bandwidth on a per-microsecond basis instead of per-second, so the kernel over-reserved USB bandwidth [12][13].
  - DWE says its firmware now reports the real data rate, and asks users to update firmware [12].
- **Power brownout.** Third-party reports show cameras dropping off the bus with `uvcvideo: Failed to resubmit video URB (-19)` errors. Blue Robotics staff attributed these to 5 V supply limits, fixed with a powered hub or dedicated supply [25].
- **Wiring damage.** Reversing +5 V and ground can permanently damage the camera. Untwisted D+/D- can cause "Unknown USB Device (Device Descriptor Request Failed)" [4].
- **Firmware fix (May 2023).** The update log lists a fix for artifacts in H.264 video at lower resolutions [9].
- **Hard-coded node index.** dweOS assumes the H.264 node is index 2 within each camera's group [12]. A system that creates a different number of nodes per camera would break that assumption.
- **Flashing risk.** A wrong or interrupted firmware flash can brick the camera or need Recovery Mode [6].

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| dweOS | DWE's current camera manager; device table, extension-unit control code, bus-path keying | Python, TypeScript, C | GPL-2.0 | [12] |
| DWE_OS_Legacy (archived) | DWE's "Bitrate Control Code"; extension-unit GUIDs, selectors and command bytes | Node.js, C | GPL-3.0 | [2][17] |
| exploreHD_Controls_Legacy | Older vendor-style extension-unit command-line tool | C | GPL version 2 or later (file headers; no repository license file) | [18] |
| DWE Firmware Loader | Firmware flashing | Windows application | Not stated | [6] |

Because dweOS is GPL-2.0 and DWE_OS_Legacy is GPL-3.0, a driver under a different license should treat them as protocol references rather than copy their code.

## Related Parts

- [Raspberry Pi 5](../tbd/raspberry_pi_5.md): the likely USB host; its USB controller layout and peripheral current limit decide how many cameras it can run.
- [PJRC Teensy 4.1](../tbd/pjrc_teensy_4_1.md): if it is also on the Pi's USB, it shares USB bandwidth and peripheral current with the cameras.

## Open Questions

1. **Exact formats, controls and ranges reported by a real unit.**
   - Why it matters: control ranges and defaults are not published, and DWE's documentation disagrees on whether a `video_bitrate` V4L2 control exists [3][12].
   - How to get it: on the target host with one camera connected, run:
     - `v4l2-ctl --list-devices`
     - `v4l2-ctl -d /dev/videoN --list-formats-ext`, for each of the camera's nodes
     - `v4l2-ctl -d /dev/videoN --list-ctrls-menus`, for the first and third nodes
     - `media-ctl -p -d /dev/mediaN`
2. **USB identity and serial uniqueness.**
   - Why it matters: if two cameras report the same serial number, they can only be told apart by port.
   - How to get it: for each camera, run `lsusb -d 0c45:6366 -v` or `lsusb -d 3961:2100 -v` and note `iSerial`, `iProduct` and `bcdDevice`. Also run `udevadm info -q property -n /dev/videoN | grep -E 'ID_SERIAL|ID_PATH|ID_VENDOR_ID|ID_MODEL_ID'`. Compare two cameras, then swap their ports, replug, and compare again.
3. **Persistence of encoder settings.**
   - Why it matters: the driver must know whether to re-apply bitrate, GOP and mode after every power-up.
   - How to get it: set the bitrate through the extension unit, power-cycle the camera, then read it back with GET_CUR.
4. **How many cameras does the vehicle carry?**
   - Why it matters: USB bandwidth and the host's 5 V budget limit the count, and the sources say 2 or 3 per Raspberry Pi [24][26].
   - How to get it: the vehicle design. Then a bench test with all cameras streaming H.264 at the planned resolution for 30 minutes, watching `dmesg` for USB errors.
5. **Which USB ID pair the units report.**
   - Why it matters: both must be matched [12]. Whether a firmware update can change a unit from one pair to the other is not documented.
   - How to get it: run `lsusb` on each unit, and record the firmware version shown by dweOS or the Firmware Loader.
6. **Exact image sensor.**
   - Why it matters: for calibration and low-light expectations. DWE says only "1/2.9 in Sony Exmor" [2]. A Blue Robotics forum post (third-party) once suggested an IMX323 for the original model [27], and a 2024 update changed imaging performance [9].
   - How to get it: ask DWE.

## Sources

1. "exploreHD (400m)", DeepWater Exploration. https://dwe.ai/products/explorehd. Used for: variants and box contents, quartz-window claim.
2. "exploreHD Technical Specifications" (exploreHD 3.0), DeepWater Exploration. https://docs.dwe.ai/exploreHD/specs/exploreHD. Used for: interface, formats and frame-rate table, sensor, lens, FOV, depth rating, housing, power by mode, cable, pinouts, temperature, Bitrate Control Code link.
3. "exploreHD GStreamer Guide", DeepWater Exploration. https://docs.dwe.ai/exploreHD/guides/gstreamer-exploreHD.md. Used for: four nodes and their roles, YUYV bandwidth limit, H.264 pipelines, `video_bitrate` claim.
4. "exploreHD Quick Start Guide", DeepWater Exploration. https://docs.dwe.ai/exploreHD/guides/exploreHD-quickstart.md. Used for: wiring and polarity warning, twisted pair, 260 mA, Linux checks, lsusb one-liner.
5. "exploreHD OpenCV Quickstart Guide", DeepWater Exploration. https://docs.dwe.ai/exploreHD/guides/opencv-exploreHD.md. Used for: default YUYV, format before resolution.
6. "exploreHD Firmware Update Guide", DeepWater Exploration. https://docs.dwe.ai/exploreHD/guides/exploreHD-firmware.md. Used for: Windows-only loader, bricking warning, recovery mode, one camera at a time.
7. "Camera Controls" (dweOS), DeepWater Exploration. https://docs.dwe.ai/dwe-os/pages/camera-controls.md. Used for: standard control list and meanings, fixed aperture, Reset Controls.
8. "dweOS overview", DeepWater Exploration. https://docs.dwe.ai/dwe-os/overview.md. Used for: Bus ID tracking.
9. "Product Update Log", DeepWater Exploration. https://docs.dwe.ai/product-update-log.md. Used for: H.264 default bitrate change, MJPEG bandwidth, firmware fixes, 2024 imaging update.
10. "7-Port Camera Hub", DeepWater Exploration. https://docs.dwe.ai/hardware/interface-modules/specs/7-port-camera-hub.md. Used for: USB 2.0 hub, power limits.
11. "DWE documentation full text", DeepWater Exploration. https://docs.dwe.ai/llms-full.txt. Used for: confirming no serial-number statement in DWE documentation.
12. "dweOS" source repository, DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/dweOS. Used for: USB ID table (`drivers/registry.py`), extension-unit protocol (`drivers/xu.py`, `drivers/ehd/options.py`), chip ID, enumeration by bus_info, settings keyed by bus path, kernel bandwidth note, GPL-2.0 license.
13. "kernel-uvc-issue.md" (dweOS v0.7.0), DeepWater Exploration (GitHub). https://raw.githubusercontent.com/DeepWaterExploration/dweOS/v0.7.0/docs/kernel-uvc-issue.md. Used for: multi-camera bandwidth problem description.
14. "dweOS issue #498", DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/dweOS/issues/498. Used for: old-ID cameras in the field.
15. "dweOS issue #519", DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/dweOS/issues/519. Used for: "Permission denied" on disabled controls is expected.
16. "dweOS pull request #494" (Add new VID and PID for camera), DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/dweOS/pull/494. Used for: date the new USB IDs were added.
17. "DWE_OS_Legacy" source repository (archived), DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/DWE_OS_Legacy. Used for: extension-unit GUIDs, selector and command table, GOP note, SN9C292 name, GPL-3.0 license.
18. "exploreHD_Controls_Legacy" source repository, DeepWater Exploration (GitHub). https://github.com/DeepWaterExploration/exploreHD_Controls_Legacy. Used for: settings must be re-applied, extension-unit GUIDs, file license headers.
19. "docs-legacy" (previous documentation source archive), DeepWater Exploration (GitHub). https://codeload.github.com/DeepwaterExploration/docs-legacy/tar.gz/refs/heads/main. Used for: node numbers change on replug, latency figures.
20. "usb.ids" (version 2026.06.26), linux-usb.org. http://www.linux-usb.org/usb.ids. Used for: `0c45:6366` vendor and product names; `0x3961` not listed.
21. "uvcvideo driver documentation", Linux kernel. https://docs.kernel.org/userspace-api/media/drivers/uvcvideo.html. Used for: extension-unit query needs a hard-coded unit and selector.
22. "UVC payload header metadata format", Linux kernel. https://docs.kernel.org/userspace-api/media/v4l/metafmt-uvc.html. Used for: metadata video nodes.
23. "uvc_ctrl.c", Linux kernel source (GitHub mirror). https://raw.githubusercontent.com/torvalds/linux/master/drivers/media/usb/uvc/uvc_ctrl.c. Used for: no bitrate control mapping in the standard UVC driver.
24. "Multiples USB Cameras", Blue Robotics Forum (post by DWE). https://discuss.bluerobotics.com/raw/14059. Used for: a Raspberry Pi powers about 3 cameras.
25. "exploreHD causes USB errors", Blue Robotics Forum (THIRD-PARTY reports, Blue Robotics staff replies). https://discuss.bluerobotics.com/raw/23430. Used for: brownout error signatures and power fix.
26. "Installing the DWE exploreHD on the BlueROV2", Blue Robotics (THIRD-PARTY). https://bluerobotics.com/learn/installing-the-dwe-explorehd-on-the-bluerov2/. Used for: typically 2 cameras per Raspberry Pi.
27. "A New High Quality Underwater USB Camera", Blue Robotics Forum (THIRD-PARTY, includes DWE posts). https://discuss.bluerobotics.com/raw/10279. Used for: third-party IMX323 sensor remark.
