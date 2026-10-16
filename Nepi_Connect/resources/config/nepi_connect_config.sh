# NEPI Connect service settings, read each time the service starts.
#
# nepi_connect_setup.sh copies this file to ~/nepi_connect/config/. Edit that
# copy, not this one, then restart the service (nepi_connect_stop, then
# nepi_connect_start). The service reads this file first and your copy second,
# so a setting your copy leaves out keeps the value here.

# The camera to stream. Empty means the first USB camera found. To pick one,
# set its /dev/v4l/by-path/...-video-index0 path (ls /dev/v4l/by-path). That
# path follows the USB port the camera is plugged into.
CAMERA_DEVICE=""

# One of the camera's MJPEG modes. The camera's own JPEG frames are sent as
# they are, with no re-encoding.
STREAM_RESOLUTION=1280x720
STREAM_FPS=30

# The stream is served at http://<this Pi's address>:<STREAM_PORT>/stream
STREAM_PORT=8080
