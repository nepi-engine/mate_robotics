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
plots. Camera source, radial correction k1, the optional claw/cable mask, and
reference reset are live controls. PID gain inputs are intentionally local to
the page: no stabilization actuator is wired yet.

Any NEPI camera that publishes `sensor_msgs/Image` can be chosen. The ROV UDP
camera becomes such a topic through the IDX network stream driver. The mask is
specific to the original ROV camera and starts disabled for other cameras.

The app manifest is in `params/optical_flow_app_params.yaml`; the RUI class is
registered in `nepi_rui/src/rui_webserver/rui-app/src/Nepi_IF_Apps.js`.
