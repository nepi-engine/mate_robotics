# NEPI Optical Flow app

This Process app subscribes to a selectable `sensor_msgs/Image` topic, estimates
camera displacement with the ORB/RANSAC tracker from
`mate_robotics/Apps/nepi_app_robot_stab/scripts/optical_flow.py`, and publishes:

- `app_optical_flow/annotated_image`: corrected camera frame with a 10-second
  camera-center trail;
- `app_optical_flow/telemetry`: JSON in `std_msgs/String` with X/Y displacement
  in pixels, feature and inlier counts, frame rate, processing time, and camera
  options.

The RUI Process → Optical Flow page shows the video and two stacked displacement
plots. Below the plots, the ROV feature mask module previews the polygon on
live video. Click Redraw polygon, click points around the region to exclude,
then click Save polygon. The nearby toggle enables or disables that polygon
for tracking. Camera source, radial correction k1, and reference reset are live
controls. PID gain inputs are intentionally local to
the page: no stabilization actuator is wired yet.

Any NEPI camera that publishes `sensor_msgs/Image` can be chosen. The ROV UDP
camera becomes such a topic through the IDX network stream driver. The default
mask is the original ROV claw/cable area. Saving a polygon updates the app's
ROS parameter and asks NEPI's configuration manager to persist it, so the
same polygon is used after the app or device restarts. The mask is off by
default.

The app manifest is in `params/optical_flow_app_params.yaml`; the RUI class is
registered in `nepi_rui/src/rui_webserver/rui-app/src/Nepi_IF_Apps.js`.

## Offline mask coordinate tool

Run `python3 Apps/nepi_app_optical_flow/tools/draw_rov_mask.py` from the
`mate_robotics` directory. The editor opens the supplied ROV frame. Click around
the area to exclude, press Enter to close the polygon, then press S to save.
Backspace removes a point and R starts over. The tool writes
`tools/rov_mask_polygon.json` with the clicked image pixels and the corresponding
640×480 mask coordinates, plus `tools/rov_mask_polygon_preview.png` with a red
overlay. Use `--image PATH` for another frame and `--k1 VALUE` if the displayed
radial correction differs from the default −0.21. This offline JSON is for
inspection and is not loaded by the live app; use the page's mask module to
apply a polygon directly.
