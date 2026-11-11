# This message details the AutoMove App Status Msg
#
# The app publishes this once per status cycle on <app>/status. Every command
# topic the app subscribes to hangs off that same <app> namespace, so a consumer
# that has this message has everything it needs to drive the app.

# Goto process states. The app reports exactly one of these in goto_state.
string GOTO_STATE_IDLE=idle
string GOTO_STATE_PLANNING=planning
string GOTO_STATE_MOVING=moving
string GOTO_STATE_COMPLETE=complete
string GOTO_STATE_CANCELLED=cancelled

#################
# Selected robot
#################
# Connect namespace of the app's ConnectRBXDeviceIF, FULLY QUALIFIED -- this is
# where the RUI's Nepi_IF_ConnectRBX mounts and where select_topic is published.
# Fully qualified because the RUI appends '/status' and hands the result to
# rosbridge, where a name with no leading slash resolves at the global root
# instead of under /<prefix>/<device_id>.
string rbx_connect_namespace
# Namespace of the RBX device currently selected on that connect, 'None' if none
string rbx_namespace
# True while that device is publishing DeviceRBXStatus
bool rbx_connected
# True when the selected device reports itself ready to accept a command
bool rbx_ready

#################
# Selected image source
#################
# Connect namespace of the app's ConnectImageIF, FULLY QUALIFIED, for the same
# reason as rbx_connect_namespace above
string image_connect_namespace
# Topic of the image currently selected on that connect, 'None' if none
string image_topic
# True while that image source is publishing ImageStatus
bool image_connected
# Size of the selected image as its ImageStatus reports it. The click pixel on
# this message is expressed in this raster.
int32 image_width_px
int32 image_height_px

#################
# Resolved companion topics
#################
# Each topic is derived from the selected image and reported with a found flag.
# A missing companion degrades the app rather than failing it: the flag reports
# false, the topic string is what the app looked for, and every feature that
# depends on it goes inert while the rest of the app keeps running.
#
# depth_map and depth_map_image are derived by NAMESPACE. Sibling data products
# share a parent namespace, so the depth map for <P>/color_image is <P>/depth_map
# and its colourized render is <P>/depth_map/depth_map_image. Note that
# DepthMapStatus.image_topic does NOT name the depth map image -- it names the
# sibling colour image -- so the image topic is never derived from that field.
string depth_map_topic
bool depth_map_found
string depth_map_image_topic
bool depth_map_image_found

# objects and targets are NOT namespace-derivable. An AI detector publishes them
# under its own node namespace and fans the same messages out on
# <base>/all/targets. Objects and targets are two readings of that one fan-out
# since the separate detections contract was removed, so both topics below
# report the same producer. The app subscribes to the
# fan-out and matches on the message's source_topic; the topic reported here is
# the producing detector's own topic, learned from the matching message's
# process_namespace, and is empty until one arrives.
string objects_topic
bool objects_found

string targets_topic
bool targets_found

# obstacles match on the resolved DEPTH MAP topic, not on the image topic -- an
# obstacles process consumes depth maps, so that is what its source_topic names.
# Empty and false when no obstacles process is running for this depth map, or
# when the obstacles message package is not installed on this device.
string obstacles_topic
bool obstacles_found

#################
# Image viewer wiring
#################
# Topic the RUI image viewer publishes nepi_interfaces/ImageMouseEvent clicks on.
# The RUI passes this to Nepi_IF_ImageViewer as its mouse_event_topic prop.
string mouse_event_topic
# Overlay image the app's image pub node publishes, the topic the RUI viewer
# displays. Empty until an image source is selected and the node is publishing.
string overlay_image_topic

#################
# Overlay controls
#################
# How transparent the depth map image render is drawn over the source frame,
# 0.0 fully opaque through 1.0 invisible. The image pub node blends at
# (1.0 - transparency), so this reads the way the RUI slider labels it.
# Meaningful only while show_depth_map_enabled is true.
float32 depth_map_transparency

# One enable flag per overlay layer. A disabled layer is skipped in the render
# rather than drawn transparent, so it costs nothing.
bool show_depth_map_enabled
bool show_objects_enabled
bool show_targets_enabled
bool show_obstacles_enabled
bool show_crosshair_enabled

#################
# Click to goto
#################
# Last clicked pixel, in the selected image's raster (image_width_px by
# image_height_px above). The ratios are the same point expressed 0.0-1.0, which
# is what the image pub node draws the crosshair from -- the overlay frame is
# the source frame's size, which need not match the viewed stream's size.
nepi_interfaces/ImagePixel last_click_pixel
float32 last_click_x_ratio
float32 last_click_y_ratio
# False until the first click of this session
bool has_click
# True when the last click landed on a pixel with a valid depth return. False
# leaves the goto values below unchanged from the previous click.
bool click_depth_valid

# Requested relative move in the robot body frame: x forward, y left, z up, in
# METERS -- the nepi_interfaces/GotoPosition convention. Depth map arrays carry
# MILLIMETRES; the app converts explicitly where it reads them.
float32 goto_x_m
float32 goto_y_m
float32 goto_z_m
# Largest forward distance a single click may request, in meters. A click
# farther than this is scaled back along the same bearing.
float32 max_move_m
# True when the last click was scaled back to max_move_m
bool goto_clamped
# Human readable account of the last click, e.g. why no goto values were set
string click_msg

#################
# Velocity mode
#################
# A second way to command the same click. Position mode sends a distance and the
# robot closes on it; velocity mode sends a body-frame velocity and a duration,
# open loop, and the robot stops when the duration elapses. The mode picks which
# one a Goto issues -- it does not change how the click itself is resolved.
bool goto_velocity_enabled

# A clicked distance is divided by a move speed to get a duration, so the speed
# is the one number that turns a position into a timed move. Which speed is used
# depends on whether the robot reports its own limit:
#
#   robot_max_speed_mps > 0   speed = robot_max_speed_mps * move_speed_ratio
#   robot_max_speed_mps == 0  speed = move_speed_mps, entered by hand
#
# The selected robot's max linear speed in METERS PER SECOND, as its
# DeviceRBXStatus.max_velocity_mps reports it. 0.0 means "not reported", never
# "may not move".
float32 robot_max_speed_mps
# Fraction of robot_max_speed_mps a velocity move runs at, 0.05-1.0.
float32 move_speed_ratio
# The hand entered fallback, in METERS PER SECOND. Unchecked against the robot:
# only the WPILib app's downstream clamp checks it.
#
# Both are persisted PER APP, not per robot. They do not follow a change of
# robot selection.
float32 move_speed_mps

# The velocity the last click resolved to, in the body frame of
# nepi_interfaces/GotoVelocity: x forward, y LEFT, z up, METERS PER SECOND.
float32 goto_x_mps
float32 goto_y_mps
float32 goto_z_mps
# Yaw rate in DEGREES PER SECOND, positive to port. Operator-facing units --
# the wire to the robot is rad/s, converted downstream. A click sets this to
# zero: a click gives a translation, not a rotation.
float32 goto_yaw_degps
# How long the robot holds that velocity, in seconds. Derived as the clicked
# distance divided by the move speed above, then clamped.
float32 goto_duration_s

# Whether the SELECTED ROBOT advertises timed velocity moves at all. False means
# the robot's RBX capabilities report has_goto_velocity false, and a velocity
# goto is refused rather than published into nothing.
bool rbx_has_goto_velocity

#################
# Goto process
#################
# One of the GOTO_STATE_* constants above
string goto_state
# Human readable account of the current or last goto process
string goto_msg
# Index of the move step being executed, and how many the plan holds
int32 goto_step
int32 goto_step_count

#################
# App controls
#################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message the app's ControlsIF publishes at
# <controls_namespace>/status, exactly as in ObstaclesStatus.msg.
#
# The namespace is FULLY QUALIFIED, for the same rosbridge reason as
# rbx_connect_namespace above.
string controls_namespace
# True once that ControlsIF has completed initialization
bool controls_ready
