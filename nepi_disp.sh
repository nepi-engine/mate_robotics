#!/usr/bin/env bash
# Run one ROV camera through NEPI displacement tracking and show it here.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$ROOT_DIR/Apps/nepi_app_robot_stab/scripts"
PLOT="$SCRIPTS_DIR/tk_disp_plot.py"
ROV_DIR="${ROV_DIR:-$(cd "$ROOT_DIR/../rov-27" && pwd)}"
VIEWER="$ROV_DIR/FastMJPG/bin/FastMJPG"
HAILO_HEF=""
if [[ "${1:-}" == "--hailo-hef" && $# -eq 2 ]]; then
    HAILO_HEF="$2"
elif [[ $# -ne 0 ]]; then
    printf 'Usage: %s [--hailo-hef /path/to/superpoint_hailo8.hef]\n' "$0" >&2
    exit 2
fi
RUN_ID="${UID}-$$"
NEPI_PIDFILE="/tmp/nepi-disp-${RUN_ID}-nepi.pid"
ONBOARD_PIDFILE="/tmp/nepi-disp-${RUN_ID}-onboard.pid"
SSH_CONTROL_DIR="$(mktemp -d /tmp/nepi-disp-ssh.XXXXXX)"
NEPI_SSH_OPTIONS=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$SSH_CONTROL_DIR/nepi.sock"
    -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p 2222
    -i /home/victoria-zhang/.ssh/nepi_default_ssh_key)
ONBOARD_SSH_OPTIONS=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$SSH_CONTROL_DIR/onboard.sock")
NEPI_SSH_PID=""
ONBOARD_SSH_PID=""
VIEWER_PID=""
PLOT_PID=""

# Same SSH destinations as the interactive sshn and h aliases. Direct SSH
# avoids running an interactive shell as a background job, which can suspend
# this script when that shell tries to read the terminal.
run_nepi() { ssh "${NEPI_SSH_OPTIONS[@]}" nepi@nepi "$@"; }
run_onboard() { ssh "${ONBOARD_SSH_OPTIONS[@]}" rishipi5@192.168.68.22 "$@"; }
run_nepi_nostdin() { ssh -n "${NEPI_SSH_OPTIONS[@]}" nepi@nepi "$@"; }
run_onboard_nostdin() { ssh -n "${ONBOARD_SSH_OPTIONS[@]}" rishipi5@192.168.68.22 "$@"; }

cleanup() {
    trap - EXIT INT TERM
    printf '\nStopping NEPI displacement test...\n'
    if [[ -n "$ONBOARD_SSH_PID" ]]; then
        run_onboard "if [ -f '$ONBOARD_PIDFILE' ]; then kill -INT \$(cat '$ONBOARD_PIDFILE') 2>/dev/null || true; rm -f '$ONBOARD_PIDFILE'; fi" >/dev/null 2>&1 || true
    fi
    if [[ -n "$NEPI_SSH_PID" ]]; then
        run_nepi "if [ -f '$NEPI_PIDFILE' ]; then kill -INT \$(cat '$NEPI_PIDFILE') 2>/dev/null || true; rm -f '$NEPI_PIDFILE'; fi" >/dev/null 2>&1 || true
    fi
    if [[ -n "$VIEWER_PID" ]]; then
        kill -INT "$VIEWER_PID" 2>/dev/null || true
    fi
    if [[ -n "$PLOT_PID" ]]; then
        kill -TERM "$PLOT_PID" 2>/dev/null || true
    fi
    [[ -z "$ONBOARD_SSH_PID" ]] || kill "$ONBOARD_SSH_PID" 2>/dev/null || true
    [[ -z "$NEPI_SSH_PID" ]] || kill "$NEPI_SSH_PID" 2>/dev/null || true
    [[ -z "$VIEWER_PID" ]] || wait "$VIEWER_PID" 2>/dev/null || true
    [[ -z "$PLOT_PID" ]] || wait "$PLOT_PID" 2>/dev/null || true
    [[ -z "$ONBOARD_SSH_PID" ]] || wait "$ONBOARD_SSH_PID" 2>/dev/null || true
    [[ -z "$NEPI_SSH_PID" ]] || wait "$NEPI_SSH_PID" 2>/dev/null || true
    ssh -O exit -o "ControlPath=$SSH_CONTROL_DIR/nepi.sock" -p 2222 nepi@nepi >/dev/null 2>&1 || true
    ssh -O exit -o "ControlPath=$SSH_CONTROL_DIR/onboard.sock" rishipi5@192.168.68.22 >/dev/null 2>&1 || true
    rm -f "$SSH_CONTROL_DIR/nepi.sock" "$SSH_CONTROL_DIR/onboard.sock"
    rmdir "$SSH_CONTROL_DIR" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -x "$VIEWER" ]]; then
    printf 'Missing computer viewer: %s\n' "$VIEWER" >&2
    exit 1
fi
if [[ ! -f "$SCRIPTS_DIR/optical_flow.py" || ! -f "$SCRIPTS_DIR/live_udp_stabilization.py" || ! -f "$SCRIPTS_DIR/hailo_superpoint.py" || ! -f "$PLOT" ]]; then
    printf 'Missing NEPI stabilization scripts in %s\n' "$SCRIPTS_DIR" >&2
    exit 1
fi
python3 -c 'import tkinter as tk; from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg; root = tk.Tk(); root.withdraw(); root.destroy()' || {
    printf 'Matplotlib and Tkinter need to be installed with a graphical desktop on this computer.\n' >&2
    exit 1
}
if [[ -n "$HAILO_HEF" ]]; then
    if [[ ! -f "$HAILO_HEF" ]]; then
        printf 'Hailo mode needs a local SuperPoint HEF: %s\n' "$HAILO_HEF" >&2
        exit 1
    fi
fi

printf 'Checking sshn (NEPI container) and h (onboard Pi)...\n'
run_nepi 'python3 -c "import cv2, numpy"' || {
    printf 'NEPI Python needs cv2 and numpy.\n' >&2
    exit 1
}
if [[ -n "$HAILO_HEF" ]]; then
    run_nepi 'test -e /dev/hailo0 && python3 -c "import hailo_platform"' || {
        printf 'NEPI needs /dev/hailo0 and the hailo_platform Python package.\n' >&2
        exit 1
    }
fi
run_onboard 'test -x /home/rishipi5/rov-27/FastMJPG/bin/FastMJPG' || {
    printf 'Onboard FastMJPG is missing or not executable.\n' >&2
    exit 1
}
run_nepi "python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind((\"192.168.68.103\",5600))'" || {
    printf 'NEPI camera port 5600 is busy or its 192.168.68.103 address is missing. Stop the old relay first.\n' >&2
    exit 1
}
python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.68.12",5600))' || {
    printf 'Computer camera port 5600 is busy or its 192.168.68.12 address is missing. Stop the old viewer first.\n' >&2
    exit 1
}
python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.68.12",5601))' || {
    printf 'Computer displacement plot port 5601 is busy or its 192.168.68.12 address is missing.\n' >&2
    exit 1
}

printf 'Copying the live processor into the NEPI container...\n'
tar --mtime='@0' -C "$SCRIPTS_DIR" -cf - optical_flow.py live_udp_stabilization.py hailo_superpoint.py |
    run_nepi 'mkdir -p ~/robot-stab && tar -m -C ~/robot-stab -xf -'
NEPI_BACKEND_ARGS=""
if [[ -n "$HAILO_HEF" ]]; then
    printf 'Copying SuperPoint HEF to NEPI...\n'
    run_nepi 'cat > ~/robot-stab/superpoint.hef' < "$HAILO_HEF"
    printf 'Checking Hailo model on NEPI...\n'
    run_nepi "cd ~/robot-stab && python3 -c 'from hailo_superpoint import HailoSuperPoint; model = HailoSuperPoint(\"superpoint.hef\"); model.close()'" || {
        printf 'The HEF did not load as a raw SuperPoint model on NEPI.\n' >&2
        exit 1
    }
    NEPI_BACKEND_ARGS='--backend hailo --hef ~/robot-stab/superpoint.hef'
fi

printf 'Starting NEPI camera processing...\n'
run_nepi_nostdin "echo \$\$ > '$NEPI_PIDFILE'; exec python3 -u ~/robot-stab/live_udp_stabilization.py --forward-ip 192.168.68.12 --telemetry-ip 192.168.68.12 $NEPI_BACKEND_ARGS" &
NEPI_SSH_PID=$!
sleep 1
if ! kill -0 "$NEPI_SSH_PID" 2>/dev/null; then
    printf 'NEPI processor exited during startup.\n' >&2
    exit 1
fi

printf 'Starting computer video viewer...\n'
"$VIEWER" receive 192.168.68.12 5600 1400 8000000 640 480 1 30 render 640 480 &
VIEWER_PID=$!
sleep 1
if ! kill -0 "$VIEWER_PID" 2>/dev/null; then
    printf 'Computer viewer exited during startup.\n' >&2
    exit 1
fi

printf 'Starting Tkinter displacement plots...\n'
python3 "$PLOT" --bind-ip 192.168.68.12 --source-ip 192.168.68.103 &
PLOT_PID=$!
sleep 1
if ! kill -0 "$PLOT_PID" 2>/dev/null; then
    printf 'Tkinter plot exited during startup.\n' >&2
    exit 1
fi

printf 'Starting onboard camera...\n'
run_onboard_nostdin "cd /home/rishipi5/rov-27 && echo \$\$ > '$ONBOARD_PIDFILE' && exec ./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 send 192.168.68.22 5600 192.168.68.103 5600 1400 8000000 2" &
ONBOARD_SSH_PID=$!

printf 'Camera displacement is plotted locally and printed by NEPI. Press Ctrl+C to stop all processes.\n'
wait -n "$NEPI_SSH_PID" "$ONBOARD_SSH_PID" "$VIEWER_PID" "$PLOT_PID" || true
printf 'One process stopped; shutting down the test.\n'
