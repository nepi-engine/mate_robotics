#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Entry point for the NEPI Connect service on the robot's Pi. Reads the
# cameras from the robot's vehicle file, starts nepi_connect.sh (the service)
# in the background, then reports whether each stream came up and where to
# find it. Normally run through the nepi_connect_start command that
# nepi_connect_setup.sh adds to the shell.
#
# Safe to re-run: if the service is already running, this only prints the
# streams' addresses.
#
# Usage: nepi_connect_start.sh

CONFIG_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${CONFIG_FOLDER}")

SERVICE=${CONFIG_FOLDER}/nepi_connect.sh
SERVICE_LOG=/tmp/nepi_connect_service.log

source "${RESOURCES_FOLDER}/bash/nepi_connect_bash_utils"
nepi_connect_load_config || exit 1
# How long to wait for the streams' ports to open before reporting
STARTUP_WAIT_SECONDS=${STARTUP_WAIT_SECONDS:-5}

function port_open(){
    ss -ltnH "sport = :$1" 2>/dev/null | grep -q .
}

function all_ports_open(){
    local port
    for port in "${CAMERA_PORTS[@]}"; do
        port_open "$port" || return 1
    done
}

function print_stream_urls(){
    local ip i
    echo "Stream addresses, for the stream_urls option of NEPI's idx_net_stream camera driver"
    echo "(NEPI's NEPI Connect driver also lists them on its link_status topic):"
    for (( i = 0; i < ${#CAMERA_NAMES[@]}; i++ )); do
        for ip in $(hostname -I 2>/dev/null); do
            [[ $ip == *:* ]] && continue   # skip IPv6
            echo "    ${CAMERA_NAMES[$i]}: http://${ip}:${CAMERA_PORTS[$i]}/stream"
        done
    done
    echo "To watch one in a browser, open the same address without /stream."
}

if [[ ! -f "$SERVICE" ]]; then
    echo "Not found: ${SERVICE}"
    exit 1
fi

if ! nepi_connect_read_cameras; then
    echo "The vehicle file could not be read, so the service was not started."
    echo "See Nepi_Connect/VEHICLE_FILE.md; nepi_connect_setup.sh stages a template."
    exit 1
fi
if [[ ${#CAMERA_NAMES[@]} -eq 0 ]]; then
    echo "The vehicle file lists no http cameras, so the service has nothing to stream and was not started."
    exit 0
fi

if pgrep -f "bash ${SERVICE}" >/dev/null 2>&1; then
    echo "nepi_connect service is already running (log: ${SERVICE_LOG})"
    print_stream_urls
    exit 0
fi

if ! command -v ustreamer >/dev/null 2>&1; then
    echo "ustreamer is not installed, so the service has nothing to stream with."
    echo "Install it with: sudo apt install -y ustreamer"
    exit 1
fi

# Only one program can stream from a USB camera at a time. A ustreamer left
# running from a manual start holds the camera, and usually the port as well.
running=$(pgrep -a -x ustreamer)
if [[ -n "$running" ]]; then
    echo "ustreamer is already running outside the nepi_connect service:"
    echo "$running"
    # Name the systemd service only when ustreamer is its main process: a
    # process started from a desktop terminal can report user@1000.service.
    ustreamer_pid=$(pgrep -o -x ustreamer)
    unit=$(ps -o unit= -p "$ustreamer_pid" 2>/dev/null | tr -d ' ')
    if [[ "$unit" == *.service && \
          "$(systemctl show -p MainPID --value "$unit" 2>/dev/null)" == "$ustreamer_pid" ]]; then
        echo "The ${unit} systemd service runs it and restarts it if it stops. Turn that off with:"
        echo "    sudo systemctl disable --now ${unit}"
    else
        echo "Stop it first (kill <pid>), then run nepi_connect_start again."
    fi
    exit 1
fi

for (( i = 0; i < ${#CAMERA_NAMES[@]}; i++ )); do
    if port_open "${CAMERA_PORTS[$i]}"; then
        echo "Port ${CAMERA_PORTS[$i]} (camera ${CAMERA_NAMES[$i]}) is already in use by another program. Stop that"
        echo "program, or give the camera a free port in the vehicle file."
        exit 1
    fi
done

echo "Starting nepi_connect service"
nohup bash "$SERVICE" >"$SERVICE_LOG" 2>&1 &
service_pid=$!

# Report as soon as every port opens. A service that has already exited is
# reported straight away, with the reason from its log.
tries=0
while ! all_ports_open && [[ $tries -lt $((STARTUP_WAIT_SECONDS * 2)) ]]; do
    if ! kill -0 "$service_pid" 2>/dev/null; then
        echo "nepi_connect service stopped right after starting. The end of ${SERVICE_LOG}:"
        tail -n 15 "$SERVICE_LOG"
        exit 1
    fi
    sleep 0.5
    tries=$((tries + 1))
done

echo "nepi_connect service started with PID ${service_pid} (log: ${SERVICE_LOG})"
if all_ports_open; then
    print_stream_urls
else
    echo "Not every stream is up yet. The end of ${SERVICE_LOG}:"
    tail -n 10 "$SERVICE_LOG"
fi
