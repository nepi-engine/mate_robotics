#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The NEPI Connect service. Runs on the robot's Pi and, for now, does one job:
# it streams a USB camera with ustreamer, as MJPEG over HTTP, for NEPI's
# idx_net_stream camera driver to read. The camera's own JPEG frames go out as
# they are, and any number of viewers (NEPI, a browser) can watch at once.
# Later this service will run the team's FastMJPG video instead.
#
# nepi_connect_start.sh starts it in the background. nepi_connect_stop stops
# it with SIGTERM, and it stops the stream on the way out. If the stream
# exits, the service starts it again, but it gives up after QUICK_FAIL_LIMIT
# exits in a row that each came within QUICK_FAIL_SECONDS of starting: that is
# a bad setting or a busy port, which retrying will not fix.

CONFIG_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${CONFIG_FOLDER}")

POLL_SECONDS=1
RESTART_SECONDS=2
QUICK_FAIL_SECONDS=10
QUICK_FAIL_LIMIT=3
# How long ustreamer gets to exit after a SIGTERM before a SIGKILL.
STOP_GRACE_SECONDS=5

source "${RESOURCES_FOLDER}/bash/nepi_connect_bash_utils"
nepi_connect_load_config || exit 1

# Print the camera to stream, or nothing if there is none yet. A UVC webcam
# makes two device nodes: video-index0 captures, and video-index1 only carries
# metadata (ustreamer fails on it with "Can't set input channel"). by-path
# names follow the USB port, which keeps two identical cameras apart: two
# NexiGo N60s report the same serial number, so their by-id names collide.
function find_camera(){
    if [[ -n "$CAMERA_DEVICE" ]]; then
        [[ -e "$CAMERA_DEVICE" ]] && echo "$CAMERA_DEVICE"
        return 0
    fi
    local path
    for path in /dev/v4l/by-path/*usb*-video-index0 /dev/v4l/by-id/*-video-index0; do
        if [[ -e "$path" ]]; then
            echo "$path"
            return 0
        fi
    done
}

stream_pid=0
stream_started=0

function start_stream(){
    local device=$1
    echo "Starting stream: ${device} -> $(readlink -f "$device"), MJPEG ${STREAM_RESOLUTION} at ${STREAM_FPS} fps, port ${STREAM_PORT}"
    ustreamer --device="$device" --format=MJPEG --encoder=HW \
        --resolution="$STREAM_RESOLUTION" --desired-fps="$STREAM_FPS" \
        --host=0.0.0.0 --port="$STREAM_PORT" &
    stream_pid=$!
    stream_started=$SECONDS
}

# Stop the stream. No-op if it is not running.
function stop_stream(){
    [[ "$stream_pid" -ne 0 ]] || return 0
    echo "Stopping stream (pid ${stream_pid})"
    kill -TERM "$stream_pid" 2>/dev/null
    local tries
    for (( tries = 0; tries < STOP_GRACE_SECONDS * 5; tries++ )); do
        kill -0 "$stream_pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -KILL "$stream_pid" 2>/dev/null
    wait "$stream_pid" 2>/dev/null
    stream_pid=0
}

# nepi_connect_stop (nepi_connect_bash_utils) stops this service with a
# SIGTERM, and the stream goes down with it.
function stop_service(){
    trap '' TERM INT
    echo "Service stop requested -- stopping the stream"
    stop_stream
    echo "Service stopped"
    exit 0
}
trap stop_service TERM INT

echo ""
echo "##########################"
echo "*** STARTING NEPI CONNECT SERVICE ***"
echo "##########################"
echo ""
echo "Settings: camera ${CAMERA_DEVICE:-first USB camera found}, ${STREAM_RESOLUTION} at ${STREAM_FPS} fps, port ${STREAM_PORT}"
echo "Your settings file: ${NEPI_CONNECT_USER_CONFIG}"

quick_fails=0
waiting_for_camera=0

while true; do
    # Catch a stream that exited, and start it again below.
    if [[ "$stream_pid" -ne 0 ]] && ! kill -0 "$stream_pid" 2>/dev/null; then
        wait "$stream_pid"
        exit_code=$?
        ran_seconds=$((SECONDS - stream_started))
        stream_pid=0
        echo "Stream exited with code ${exit_code} after ${ran_seconds} s"
        if [[ $ran_seconds -lt $QUICK_FAIL_SECONDS ]]; then
            quick_fails=$((quick_fails + 1))
        else
            quick_fails=0
        fi
        if [[ $quick_fails -ge $QUICK_FAIL_LIMIT ]]; then
            echo "The stream exited ${quick_fails} times in a row right after starting, so the service is stopping."
            echo "Check the ustreamer messages above, fix the cause, then run nepi_connect_start again."
            exit 1
        fi
        sleep "$RESTART_SECONDS"
    fi

    if [[ "$stream_pid" -eq 0 ]]; then
        camera=$(find_camera)
        if [[ -n "$camera" ]]; then
            waiting_for_camera=0
            start_stream "$camera"
        elif [[ $waiting_for_camera -eq 0 ]]; then
            waiting_for_camera=1
            if [[ -n "$CAMERA_DEVICE" ]]; then
                echo "Waiting for camera ${CAMERA_DEVICE} (CAMERA_DEVICE in ${NEPI_CONNECT_USER_CONFIG})"
            else
                echo "Waiting for a USB camera (none in /dev/v4l/by-path or /dev/v4l/by-id yet)"
            fi
        fi
    fi

    sleep "$POLL_SECONDS"
done
