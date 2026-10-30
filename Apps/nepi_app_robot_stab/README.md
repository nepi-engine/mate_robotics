# ROV camera displacement on NEPI

`scripts/optical_flow.py` estimates image displacement in pixels. The original
`tests/test_of.py` reads a saved video and opens plots; it is not a live NEPI
application. `scripts/live_udp_stabilization.py` runs the same tracker on the
ROV's FastMJPG UDP stream. It can run in the NEPI Docker container because that
container uses the host network. It also forwards camera packets to the
topside viewer.

## One-camera live test

The example addresses are onboard Pi `192.168.68.22`, NEPI
`192.168.68.103`, and topside computer `192.168.68.12`. All three devices must
be connected to the Ethernet switch. NEPI must have its `192.168.68.103/24`
Ethernet alias, and the computer must have `192.168.68.12/24`.

From the `mate_robotics` folder on the **computer**, run:

```sh
bash nepi_disp.sh
```

The script uses the same SSH destinations as the computer's `sshn` alias for
the NEPI container and `h` alias for the onboard Pi. It copies the two Python files to NEPI,
starts the NEPI processor, opens the computer viewer, then starts the onboard
camera. It also opens a Tkinter window on the computer with x(t), y(t), and
the parametric path (x(t), y(t)), all in pixels over the most recent 60 seconds.
NEPI sends displacement samples to computer UDP port 5601; the camera viewer
continues to use port 5600. Press Ctrl+C to stop the processes. Stop the earlier one-off Python UDP
relay, computer viewer, and onboard sender before running it so each camera
port has only one owner.

The computer terminal shows NEPI's displacement in **pixels**, while the video
window continues to show the camera. No motor commands are produced. The camera must
be fixed to the robot, and pixel motion still needs camera calibration and
scale before it can be interpreted as robot motion in metres.

## Optional Hailo-8 SuperPoint test

The NEPI container can access the Hailo-8 at `/dev/hailo0`. The Hailo backend
replaces ORB feature detection/description with SuperPoint inference on that
device. Feature matching, RANSAC, and smoothing still run on the Pi CPU. The
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
or RUI. The ROV FastMJPG stream is a custom UDP format; the network stream
driver in this repository accepts RTSP or HTTP and cannot ingest it directly.
