# ROV camera displacement on NEPI

`scripts/optical_flow.py` estimates image displacement in pixels. The original
`tests/test_of.py` reads a saved video and opens plots; it is not a live NEPI
application. `scripts/live_udp_stabilization.py` runs the same tracker on the
ROV's FastMJPG UDP stream. It can run in the NEPI Docker container because that
container uses the host network. It also forwards camera packets to the
topside viewer.

## Camera stream after a Pi reboot

The RUI Devices page lists **Imaging** only after NEPI's IDX network stream
driver receives camera packets. The Pi can be reachable over SSH while no
`FastMJPG` sender is running. `systemd/rov-camera0-nepi.service` starts camera 0
at boot and sends it to topside UDP 5600 and NEPI UDP 5604. On the Pi, install
and enable it with:

```sh
sudo install -m 644 systemd/rov-camera0-nepi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rov-camera0-nepi.service
```

Check `systemctl status rov-camera0-nepi.service` if Devices again shows
"Waiting for Apps". Stop this service before starting `quick_stream.sh`
manually because both commands capture `/dev/video0`.

## One-camera live test

The example addresses are onboard Pi `192.168.68.22`, NEPI
`192.168.68.103`, and topside computer `192.168.68.12`. All three devices must
be connected to the Ethernet switch. NEPI must have its `192.168.68.103/24`
Ethernet alias, and the computer must have `192.168.68.12/24`.
On the computer, `ip route get 192.168.68.22` should use the Ethernet interface
and source address `192.168.68.12`. A NEPI connection profile with only a
`192.168.179.x` address can still reach NEPI while sending camera Pi traffic
over the wrong interface.

From the `mate_robotics` folder on the **computer**, run:

```sh
bash nepi_disp.sh
```

The script uses the same SSH destinations as the computer's `sshn` alias for
the NEPI container and `h` alias for the onboard Pi. It copies the processor to NEPI,
starts the NEPI processor, and opens the computer display. If camera 0 is free,
it starts onboard capture. If ROV quick-start already owns `/dev/video0`, it
copies the existing outgoing UDP camera packets to NEPI using a temporary
packet relay on the Pi. The relay needs passwordless `sudo` on the Pi and stops
with `nepi_disp.sh`; the existing four-camera stream keeps running. The script
opens one Tkinter window with the live camera, x(t) and y(t) Matplotlib plots,
and the parametric path (x(t), y(t)) overlaid across the camera image. The
orange dot stays at the center; the trail shows the most recent 10 seconds
relative to the current displacement. The computer calculates the green trail
from the same video frames it displays, so the overlay stays in step with the
camera feed. It excludes the camera-mounted foreground from feature matching
and tracks motion at the image center without temporal smoothing. One trail
pixel represents one corrected camera-frame pixel. NEPI separately sends
displacement samples for the side x/y plots, which keep positive Y downward in
camera coordinates. Those plots can update more slowly than the green trail.
NEPI sends displacement samples to computer UDP port 5601; the camera display
uses port 5600. Press Ctrl+C to stop the processes started by this
script. Stop an earlier one-off Python UDP relay or computer viewer if either
already owns its port. An active ROV quick-start onboard sender can stay running.

If the script reports that the computer needs `192.168.68.12`, switch to a
network profile that includes that address or add it to the Ethernet interface.
If SSH to `192.168.68.22` then times out, check that the separate onboard camera
Pi is powered on, connected to the same switch, still has `192.168.68.22`, and
has SSH running. From the topside computer, test with
`ssh -o ConnectTimeout=5 rishipi5@192.168.68.22 true` before retrying the script.
After a robot power cycle, the launcher waits up to 90 seconds for onboard SSH
to become ready.

The computer terminal shows NEPI's displacement in **pixels**, while the display
shows the camera and plots. No motor commands are produced. The camera must
be fixed to the robot, and pixel motion still needs camera calibration and
scale before it can be interpreted as robot motion in metres.

The `nepi_disp.sh` display shows the camera stream in its native orientation.
NEPI tracks that stream and maps its image motion to camera motion:
negative X is left and positive Y is down. The live test excludes the
camera-mounted orange claw and cable from feature matching by default: raw-frame
pixels `y=290..479` and `x=0..279, y=170..289` (scaled if the stream resolution
changes). This prevents stationary foreground features from hiding camera
movement. Use `./nepi_disp.sh --no-mask-foreground` to compare against the full
frame. The mask affects tracking only; the full video remains visible.

## Tune lens correction from a camera still

On the topside computer, run the graphical tuner:

```sh
python3 Apps/nepi_app_robot_stab/scripts/undistort_tuner.py
```

Choose a still from the **camera feed** when the file picker opens, or pass its
path as the first argument. If it is a screenshot with window borders, use
**Crop screenshot** to select only the full camera image. The window compares
the original and corrected images while you move the radial `k1` slider. Zero
leaves the image unchanged; negative values counteract barrel distortion.
Straight edges visible at several places in the scene are better guides than
the shape of a single object. The live optical-flow tracker now uses
`k1=-0.2100` by default, based on the visual fit. **Save preset** writes a JSON
file for the 640×480 live stream. To override the default with a new fit:

```sh
./nepi_disp.sh --undistort-config /absolute/path/to/camera_undistort.json
```

The topside video viewer and NEPI optical-flow tracker use the same `k1`;
the camera stream sent over the network remains unchanged. A preset with
`k1=0` disables the correction in both. This one-parameter
visual fit cannot recover scene depth, fix parallax, or replace a full camera calibration. If the
camera looks through an underwater housing, tune or calibrate it in its actual
operating setup.

## Optional Hailo-8 SuperPoint test

The NEPI container can access the Hailo-8 at `/dev/hailo0`. The Hailo backend
replaces ORB feature detection/description with SuperPoint inference on that
device. Feature matching and RANSAC still run on the NEPI CPU. The
normal `bash nepi_disp.sh` command remains the ORB comparison run.

To try the Hailo backend, first obtain a **Hailo-8** (not Hailo-8L or Hailo-10)
SuperPoint `.hef` on the computer. It must have one grayscale or RGB image
input and two *raw* outputs: 65 keypoint logits and 256 coarse descriptor
channels, both at one-eighth input resolution. A YOLO HEF or a model with
already decoded/keypoint-only outputs will be rejected. No compatible
SuperPoint HEF is included in this repository, and the computer currently has
no Hailo compiler installed.

From the `mate_robotics` folder on the **computer**:

```sh
./nepi_disp.sh --hailo-hef /absolute/path/to/superpoint_hailo8.hef
```

The script copies the model into the NEPI container, verifies its shapes and
that HailoRT can load it, then starts the normal stream/viewer. The model is
not transferred to the onboard Pi. `hailo_platform` must be importable by
NEPI's `python3`. Standard SuperPoint input is grayscale in `[0, 1]`; the
backend scales input pixels by `1/255` before HailoRT quantization. A HEF
compiled for a different input convention needs its scale adjusted in the
Python runner. Hailo-mode output includes processed fps and RANSAC inliers so
the same recorded scene can be compared with ORB. Model compatibility and
quality on ROV footage must be checked on the real device before using these
numbers for control.

This is a standalone live test, not an app registered with NEPI's App Manager
or RUI. The network stream driver in this repository can receive the ROV's
FastMJPG camera feed for RUI Imaging. The separate `nepi_app_optical_flow`
package registers optical-flow processing and its display with NEPI and RUI.
