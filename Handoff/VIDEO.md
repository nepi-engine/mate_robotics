# Video: Two Ways to Get Camera Video Into NEPI

2026-11-02, from the NEPI team. There are two working methods: yours, which relays the pilot's video through NEPI, and ours, which sends NEPI a second copy. We list both so the second stays on record, not to push either. The trade-offs below are facts only.

## Terms

- **NEPI**: the edge-computing software on your NEPI computer. A NEPI **driver** connects one device to NEPI. **Discovery** is the driver step that finds a device and starts its node, one running program.
- **IDX**: NEPI's driver type for cameras. A stream that an IDX driver reads becomes a **NEPI camera**: the RUI shows it, and NEPI apps (object detection, recording, processing) can use it as an image source.
- **RBX**: NEPI's driver type for robots, here `rbx_nepi_connect` ("NEPI Connect" on NEPI's Drivers page). **NavPose**: NEPI's navigation and pose data.
- **RUI**: NEPI's browser-based user interface.
- **FastMJPG**: your video tool. It sends each frame as one JPEG cut into UDP packets in its own format, and each `send` output can repeat a frame several times (**send rounds**) to survive packet loss. A `send` output is written `send LOCAL_IP_ADDRESS LOCAL_PORT REMOTE_IP_ADDRESS REMOTE_PORT MAX_PACKET_LENGTH MAX_JPEG_LENGTH SEND_ROUNDS` (`rov-26 FastMJPG/README.md:142`; your `quick_stream.sh:1` is one).
- **Relay**: a program that receives packets and sends them on unchanged.
- **Holds a port**: a program that has bound a UDP port. Only one program on a computer can hold a given UDP port at a time.

## Your Method: The Relay Through NEPI

As your committed scripts implement it (`nepi_disp.sh`, [live_udp_stabilization.py](../Apps/nepi_app_robot_stab/scripts/live_udp_stabilization.py), [Apps/nepi_app_robot_stab/README.md](../Apps/nepi_app_robot_stab/README.md)):

```
 ROV PI 192.168.68.22                NEPI 192.168.68.103                       TOPSIDE 192.168.68.12
 FastMJPG capture /dev/video0        live_udp_stabilization.py
  640x480, 30 fps, 2 rounds           holds 192.168.68.103:5600,
  from :5600 ----------------------->  accepts only 192.168.68.22
                                       relays every packet as is ---------> :5600 FastMJPG receive + render
                                       reassembles frames, optical flow
                                       displacement (s, x px, y px) -------> :5601 tk_disp_plot.py
```

- **ROV Pi.** One output, to NEPI only: `send 192.168.68.22 5600 192.168.68.103 5600 1400 8000000 2` (`nepi_disp.sh`, "Starting onboard camera").
- **NEPI.** The relay runs in the NEPI container, which uses the host network, so it receives on NEPI's own address with no port mapping. It forwards each packet to the topside before it reassembles anything (`receive_frames`). Then it decodes the newest frame and runs `OpticalFlow` on it.
- **Topside.**
  - The pilot's viewer is `FastMJPG receive 192.168.68.12 5600 1400 8000000 640 480 1 30 render`.
  - The displacement plot listens on 192.168.68.12:5601 and accepts only NEPI.
- **Tested.** By you, as a one-camera test. We have not run it.

Your README says the network stream driver in this repo cannot read FastMJPG. That was true before commit `02b3ae7`, which added the `fastmjpg://` mode used below.

## Our Method: A Second Output to NEPI

From our [LINK_PROTOCOL.md](../Nepi_Connect/LINK_PROTOCOL.md) "Video to NEPI", which matches the code. FastMJPG accepts one input and any number of outputs. So each camera's command keeps the pilot's output exactly as it is, and gains a second `send` output aimed at NEPI:

```
 ROV PI 192.168.68.22                                                       TOPSIDE 192.168.68.12
 FastMJPG capture /dev/videoN
  pilot output, unchanged:  from :5600+N, 2 rounds ------------------------> :5600+N your viewer
  NEPI copy:                from :5610+N, 1 round  ---> NEPI 192.168.68.103 :5600+N
                                                        idx_net_stream camera node
                                                        URL fastmjpg://192.168.68.22:5600+N
```

For camera 0, as on our bench:

```bash
# [ROV Pi] camera 0: your pilot output first, then the NEPI copy
./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 \
  send 192.168.68.22 5600 192.168.68.12 5600 1400 1000000 2 \
  send 192.168.68.22 5610 192.168.68.103 5600 1400 1000000 1
```

### On NEPI: the idx_net_stream driver

The driver is in [Drivers/idx_drivers/](../Drivers/idx_drivers/): [idx_net_stream_params.yaml](../Drivers/idx_drivers/idx_net_stream_params.yaml), `_discovery.py`, `_node.py` and `_driver.py`. It installs the same way as the robot driver, with the `deploy_drivers.sh` in that folder ([NEPI_SETUP.md](NEPI_SETUP.md#installing-the-driver-files)). Then enable **Network Stream Camera** on the Drivers page.

| Discovery option | Default | What to enter |
|---|---|---|
| `stream_urls` | empty | A comma-separated list, one URL per camera, for example `fastmjpg://192.168.68.22:5600, fastmjpg://192.168.68.22:5601` |
| `data_products` | `color_image` | Leave as is |

How the driver reads a `fastmjpg://` URL (`NetStreamCamDriver.parseFastMJPGUrl` and `FastMJPGReceiver`):

- **Host.** The sender to accept packets from: your ROV Pi. `0.0.0.0` accepts any sender. Packets from any other address are dropped.
- **Port.** NEPI's own UDP port, the send command's REMOTE_PORT (default 5600). The camera node holds it on all of NEPI's addresses. Each stream needs its own port, and discovery rejects a second entry with the same port.
- **Options.** If the send command's MAX_PACKET_LENGTH or MAX_JPEG_LENGTH differ from 1400 and 1000000, add `?max_packet=N&max_jpeg=N`, for example `fastmjpg://192.168.68.22:5600?max_jpeg=8000000` for your 8000000 setting.
- **Probing.** Discovery waits until a URL delivers a frame before it starts that camera's node, and keeps re-probing a silent URL every 5 s (`PROBE_INTERVAL_SEC`). A camera started after NEPI therefore still appears.
- **What else it reads.** The same driver also reads RTSP and HTTP streams. It reads no other plain UDP format.

A vehicle file may also list cameras (`cameras`, [VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md) "cameras"). NEPI then only shows each camera's address on the robot driver's `link_status` topic, `fastmjpg://0.0.0.0:<port>` for a FastMJPG camera. It does not set up `idx_net_stream` from that list, so you still enter the URLs above by hand. Your [mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml) lists no cameras, and nothing in this file depends on it.

### Quick look without NEPI

[fastmjpg_viewer.py](../Nepi_Connect/tools/fastmjpg_viewer.py) receives the same stream on any computer with Python 3, OpenCV and numpy and a display. Its receiver class is the same one the driver uses. It prints the frame rate, lost frames and bad packets once a second.

```bash
# [topside] point the NEPI copy at this computer instead of NEPI for a moment, then:
python3 Nepi_Connect/tools/fastmjpg_viewer.py --port 5600 --from 192.168.68.22
```

`--max-packet` and `--max-jpeg` must match the send command, as in the URL options above.

### What we have tested

- The `fastmjpg://` mode worked under NEPI on our bench on 2026-10-19, with one camera.
- On a simulated link with 3% packet loss, 1 send round delivered 12 of 60 frames and 2 rounds delivered 58 of 60. On a lossy tether, give the NEPI copy 2 rounds, at twice that copy's bandwidth.
- From reading the FastMJPG source, not tested:
  - Each send output is its own socket, so NEPI being off should not stop the pilot's output.
  - A destination with no route, or a local address the Pi does not own, ends the whole FastMJPG process, pilot video included. Keep every `send` address on 192.168.68.0/24.

## The Trade-Offs, Side by Side

| Question | Your relay | Our second output |
|---|---|---|
| Does the pilot's video depend on NEPI being up? | Yes. It passes through the relay process on NEPI, so it stops when NEPI or the relay stops. | No. The pilot's output goes straight to the topside, as without NEPI. |
| Copies of each camera crossing the tether | One, to NEPI | Two: the pilot's and NEPI's |
| Ports held on the ROV Pi (local send ports) | 5600 | 5600+N for the pilot, 5610+N for NEPI |
| Ports held on NEPI | UDP 5600 on 192.168.68.103 | UDP 5600+N on all addresses, one per camera node |
| Ports held on the topside | 5600 for the viewer, 5601 for the plot | 5600+N for the viewers, as today |
| Can NEPI apps use the stream as a NEPI camera? | No. The relay is a standalone script, not a NEPI driver. | Yes. Each stream is a NEPI camera. |
| Overlap with your camera ports | The plot uses topside 5601, which is camera 1's port in `rov-26 config.yaml:5`. A camera 1 viewer and the plot cannot both hold it. If a FastMJPG receiver holds 5601, the plot's 16-byte packets reach it, and FastMJPG's receiver exits on any packet shorter than its 20-byte header (`rov-26 FastMJPG/src/VideoUDPReceiver.c:71-74`). | None on the topside |
| Change on the ROV Pi | The camera command sends to NEPI instead of the topside | One extra `send` output per camera |

The table makes no latency claim for either path, because we have not measured the relay's.

## Running Both at Once

Both can run together, because a NEPI port is just a number you choose. They clash only when two programs on NEPI try to hold the same port, as your relay (5600 on 192.168.68.103) and a camera node on 5600 would.

1. Keep your relay as it is, on NEPI port 5600.
2. Give NEPI's camera copy a different NEPI port, such as 5610. On the ROV Pi, add it to the same FastMJPG command as a second output:

```bash
# [ROV Pi] camera 0: your relay output, plus a NEPI camera copy on NEPI port 5610
./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 \
  send 192.168.68.22 5600 192.168.68.103 5600 1400 8000000 2 \
  send 192.168.68.22 5610 192.168.68.103 5610 1400 1000000 1
```

3. In `idx_net_stream`'s `stream_urls`, enter `fastmjpg://192.168.68.22:5610`.

This sends two copies of the camera to NEPI over the tether. If camera 1 also streams to the topside, first move the plot off 5601 to a port no camera uses: `--telemetry-port` on NEPI and `--port` on `tk_disp_plot.py`. Otherwise its packets would stop camera 1's viewer (table above).

Whichever method you choose, the stabilization process that uses the video talks to the ROV through the robot driver's topics ([NEPI_SETUP.md](NEPI_SETUP.md#topics-your-own-code-would-use)). Video and control are separate links.
