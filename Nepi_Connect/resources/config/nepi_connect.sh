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
# it streams each http camera the robot's vehicle file lists with ustreamer,
# as MJPEG over HTTP, for NEPI's idx_net_stream camera driver to read. The
# camera's own JPEG frames go out as they are, and any number of viewers
# (NEPI, a browser) can watch at once. fastmjpg cameras in the vehicle file
# are only announced to NEPI: the robot's own FastMJPG command sends them.
#
# The vehicle file is read through the nepi_connect package
# (nepi_connect_read_cameras in nepi_connect_bash_utils), never parsed here.
# With no vehicle file, without PyYAML, or with an invalid value, the service
# prints why and stops.
#
# nepi_connect_start.sh starts it in the background. nepi_connect_stop stops
# it with SIGTERM, and it stops every stream on the way out. If a stream
# exits, the service starts it again, but it gives up on that camera after
# QUICK_FAIL_LIMIT exits in a row that each came within QUICK_FAIL_SECONDS of
# starting: that is a bad setting or a busy port, which retrying will not fix.
# Once every camera has been given up on, the service stops. The timings are
# in nepi_connect_config.sh.

CONFIG_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${CONFIG_FOLDER}")

source "${RESOURCES_FOLDER}/bash/nepi_connect_bash_utils"
nepi_connect_load_config || exit 1
POLL_SECONDS=${POLL_SECONDS:-1}
RESTART_SECONDS=${RESTART_SECONDS:-2}
QUICK_FAIL_SECONDS=${QUICK_FAIL_SECONDS:-10}
QUICK_FAIL_LIMIT=${QUICK_FAIL_LIMIT:-3}
STOP_GRACE_SECONDS=${STOP_GRACE_SECONDS:-5}

if [[ -n "${CAMERA_DEVICE}${STREAM_RESOLUTION}${STREAM_FPS}${STREAM_PORT}" ]]; then
    echo "Note: CAMERA_DEVICE, STREAM_RESOLUTION, STREAM_FPS and STREAM_PORT in ${NEPI_CONNECT_USER_CONFIG}"
    echo "are no longer read. The cameras come from the vehicle file (Nepi_Connect/VEHICLE_FILE.md \"cameras\")."
fi

if ! nepi_connect_read_cameras; then
    echo "The service has no cameras to stream, so it is stopping. Fix the vehicle file, then run"
    echo "nepi_connect_start again."
    exit 1
fi
CAMERA_COUNT=${#CAMERA_NAMES[@]}
if [[ $CAMERA_COUNT -eq 0 ]]; then
    echo "The vehicle file lists no http cameras, so there is nothing to stream. The service is stopping."
    exit 0
fi

# Per camera, by index: the stream's pid (0 when not running), when it
# started, its quick exits in a row, whether it is waiting for its camera,
# whether the service gave up on it, and the device it streams.
stream_pid=()
stream_started=()
quick_fails=()
waiting_for_camera=()
given_up=()
stream_device=()
for (( i = 0; i < CAMERA_COUNT; i++ )); do
    stream_pid[$i]=0
    stream_started[$i]=0
    quick_fails[$i]=0
    waiting_for_camera[$i]=0
    given_up[$i]=0
    stream_device[$i]=""
done

# Print the device for camera $1, or nothing if there is none yet. A UVC
# webcam makes two device nodes: video-index0 captures, and video-index1 only
# carries metadata (ustreamer fails on it with "Can't set input channel").
# by-path names follow the USB port, which keeps two identical cameras apart:
# two NexiGo N60s report the same serial number, so their by-id names
# collide. A camera with no device takes the first USB camera that no other
# stream is using.
function find_camera(){
    local index=$1
    local device=${CAMERA_DEVICES[$index]}
    if [[ -n "$device" ]]; then
        [[ -e "$device" ]] && echo "$device"
        return 0
    fi
    local path other used j
    for path in /dev/v4l/by-path/*usb*-video-index0 /dev/v4l/by-id/*-video-index0; do
        [[ -e "$path" ]] || continue
        used=0
        for (( j = 0; j < CAMERA_COUNT; j++ )); do
            [[ $j -eq $index ]] && continue
            other=${stream_device[$j]}
            [[ -z "$other" ]] && other=${CAMERA_DEVICES[$j]}
            if [[ -n "$other" && "$(readlink -f "$other")" == "$(readlink -f "$path")" ]]; then
                used=1
            fi
        done
        if [[ $used -eq 0 ]]; then
            echo "$path"
            return 0
        fi
    done
}

function start_stream(){
    local index=$1
    local device=$2
    echo "Starting stream ${CAMERA_NAMES[$index]}: ${device} -> $(readlink -f "$device"), MJPEG" \
         "${CAMERA_RESOLUTIONS[$index]} at ${CAMERA_FPS[$index]} fps, port ${CAMERA_PORTS[$index]}"
    ustreamer --device="$device" --format=MJPEG --encoder=HW \
        --resolution="${CAMERA_RESOLUTIONS[$index]}" --desired-fps="${CAMERA_FPS[$index]}" \
        --host=0.0.0.0 --port="${CAMERA_PORTS[$index]}" &
    stream_pid[$index]=$!
    stream_started[$index]=$SECONDS
    stream_device[$index]=$device
}

# Stop camera $1's stream. No-op if it is not running.
function stop_stream(){
    local index=$1
    local pid=${stream_pid[$index]}
    [[ "$pid" -ne 0 ]] || return 0
    echo "Stopping stream ${CAMERA_NAMES[$index]} (pid ${pid})"
    kill -TERM "$pid" 2>/dev/null
    local tries
    for (( tries = 0; tries < STOP_GRACE_SECONDS * 5; tries++ )); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -KILL "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
    stream_pid[$index]=0
    stream_device[$index]=""
}

# nepi_connect_stop (nepi_connect_bash_utils) stops this service with a
# SIGTERM, and every stream goes down with it.
function stop_service(){
    trap '' TERM INT
    echo "Service stop requested -- stopping the streams"
    local i
    for (( i = 0; i < CAMERA_COUNT; i++ )); do
        stop_stream "$i"
    done
    echo "Service stopped"
    exit 0
}
trap stop_service TERM INT

echo ""
echo "##########################"
echo "*** STARTING NEPI CONNECT SERVICE ***"
echo "##########################"
echo ""
for (( i = 0; i < CAMERA_COUNT; i++ )); do
    echo "Camera ${CAMERA_NAMES[$i]}: ${CAMERA_DEVICES[$i]:-first free USB camera}, ${CAMERA_RESOLUTIONS[$i]} at" \
         "${CAMERA_FPS[$i]} fps, port ${CAMERA_PORTS[$i]}"
done
echo "Your settings file: ${NEPI_CONNECT_USER_CONFIG}"

while true; do
    for (( i = 0; i < CAMERA_COUNT; i++ )); do
        [[ ${given_up[$i]} -eq 0 ]] || continue
        # Catch a stream that exited, and start it again below.
        if [[ "${stream_pid[$i]}" -ne 0 ]] && ! kill -0 "${stream_pid[$i]}" 2>/dev/null; then
            wait "${stream_pid[$i]}"
            exit_code=$?
            ran_seconds=$((SECONDS - stream_started[$i]))
            stream_pid[$i]=0
            stream_device[$i]=""
            echo "Stream ${CAMERA_NAMES[$i]} exited with code ${exit_code} after ${ran_seconds} s"
            if [[ $ran_seconds -lt $QUICK_FAIL_SECONDS ]]; then
                quick_fails[$i]=$((quick_fails[$i] + 1))
            else
                quick_fails[$i]=0
            fi
            if [[ ${quick_fails[$i]} -ge $QUICK_FAIL_LIMIT ]]; then
                given_up[$i]=1
                echo "Stream ${CAMERA_NAMES[$i]} exited ${quick_fails[$i]} times in a row right after starting, so the"
                echo "service stops trying it. Check the ustreamer messages above, fix the cause, then run"
                echo "nepi_connect_stop and nepi_connect_start again."
                continue
            fi
            sleep "$RESTART_SECONDS"
        fi

        if [[ "${stream_pid[$i]}" -eq 0 ]]; then
            camera=$(find_camera "$i")
            if [[ -n "$camera" ]]; then
                waiting_for_camera[$i]=0
                start_stream "$i" "$camera"
            elif [[ ${waiting_for_camera[$i]} -eq 0 ]]; then
                waiting_for_camera[$i]=1
                if [[ -n "${CAMERA_DEVICES[$i]}" ]]; then
                    echo "Waiting for camera ${CAMERA_NAMES[$i]} at ${CAMERA_DEVICES[$i]} (device in the vehicle file)"
                else
                    echo "Waiting for a free USB camera for ${CAMERA_NAMES[$i]} (none in /dev/v4l/by-path or" \
                         "/dev/v4l/by-id yet)"
                fi
            fi
        fi
    done

    active=0
    for (( i = 0; i < CAMERA_COUNT; i++ )); do
        [[ ${given_up[$i]} -eq 0 ]] && active=$((active + 1))
    done
    if [[ $active -eq 0 ]]; then
        echo "The service gave up on every camera, so it is stopping."
        exit 1
    fi

    sleep "$POLL_SECONDS"
done
