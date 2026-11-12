# NEPI Connect service settings, read each time the service starts.
#
# nepi_connect_setup.sh copies this file to ~/nepi_connect/config/. Edit that
# copy, not this one, then restart the service (nepi_connect_stop, then
# nepi_connect_start). The service reads this file first and your copy second,
# so a setting your copy leaves out keeps the value here.
#
# The cameras to stream are not set here. They come from the robot's vehicle
# file, ~/nepi_connect/config/nepi_connect_vehicle.yaml by default
# (Nepi_Connect/VEHICLE_FILE.md "cameras"): one ustreamer per http camera.
# CAMERA_DEVICE, STREAM_RESOLUTION, STREAM_FPS and STREAM_PORT, which older
# copies of this file hold, are no longer read.

# How often the service checks on each stream, in seconds.
POLL_SECONDS=1

# How long the service waits before starting a stream that exited again.
RESTART_SECONDS=2

# A stream that exits QUICK_FAIL_LIMIT times in a row, each within
# QUICK_FAIL_SECONDS of starting, has a bad setting or a busy port, which
# retrying will not fix, so the service stops trying that camera. When every
# camera has stopped, the service stops.
QUICK_FAIL_SECONDS=10
QUICK_FAIL_LIMIT=3

# How long ustreamer gets to exit after a SIGTERM before a SIGKILL, in seconds.
STOP_GRACE_SECONDS=5

# How long nepi_connect_start waits for the streams' ports to open before it
# reports, in seconds.
STARTUP_WAIT_SECONDS=5
