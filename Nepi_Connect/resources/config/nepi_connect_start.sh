#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Entry point for the NEPI Connect service on the robot's Pi. Starts
# nepi_connect.sh (the service) in the background, then reports whether the
# stream came up and where to find it. Normally run through the
# nepi_connect_start command that nepi_connect_setup.sh adds to the shell.
#
# Safe to re-run: if the service is already running, this only prints the
# stream's address.
#
# Usage: nepi_connect_start.sh

CONFIG_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${CONFIG_FOLDER}")

SERVICE=${CONFIG_FOLDER}/nepi_connect.sh
SERVICE_LOG=/tmp/nepi_connect_service.log

# How long to wait for the stream's port to open before reporting.
STARTUP_WAIT_SECONDS=5

source "${RESOURCES_FOLDER}/bash/nepi_connect_bash_utils"
nepi_connect_load_config || exit 1

function stream_port_open(){
    ss -ltnH "sport = :${STREAM_PORT}" 2>/dev/null | grep -q .
}

function print_stream_urls(){
    local ip
    echo "Stream address, for the stream_urls option of NEPI's idx_net_stream camera driver:"
    for ip in $(hostname -I 2>/dev/null); do
        [[ $ip == *:* ]] && continue   # skip IPv6
        echo "    http://${ip}:${STREAM_PORT}/stream"
    done
    echo "To watch it in a browser, open the same address without /stream."
}

if [[ ! -f "$SERVICE" ]]; then
    echo "Not found: ${SERVICE}"
    exit 1
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

if stream_port_open; then
    echo "Port ${STREAM_PORT} is already in use by another program. Stop that program, or set"
    echo "STREAM_PORT to a free port in ${NEPI_CONNECT_USER_CONFIG}."
    exit 1
fi

echo "Starting nepi_connect service"
nohup bash "$SERVICE" >"$SERVICE_LOG" 2>&1 &
service_pid=$!

# Report as soon as the port opens. A service that has already exited is
# reported straight away, with the reason from its log.
tries=0
while ! stream_port_open && [[ $tries -lt $((STARTUP_WAIT_SECONDS * 2)) ]]; do
    if ! kill -0 "$service_pid" 2>/dev/null; then
        echo "nepi_connect service stopped right after starting. The end of ${SERVICE_LOG}:"
        tail -n 15 "$SERVICE_LOG"
        exit 1
    fi
    sleep 0.5
    tries=$((tries + 1))
done

echo "nepi_connect service started with PID ${service_pid} (log: ${SERVICE_LOG})"
if stream_port_open; then
    print_stream_urls
else
    echo "The stream is not up yet. The end of ${SERVICE_LOG}:"
    tail -n 10 "$SERVICE_LOG"
fi
