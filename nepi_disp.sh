#!/usr/bin/env bash
# Run one ROV camera through NEPI displacement tracking and show it here.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$ROOT_DIR/Apps/nepi_app_robot_stab/scripts"
PLOT="$SCRIPTS_DIR/tk_disp_plot.py"
RUI_CAMERA_PORT=5604
HAILO_HEF=""
MASK_FOREGROUND=1
UNDISTORT_CONFIG=""
usage() {
    printf 'Usage: %s [--no-mask-foreground] [--undistort-config /path/to/preset.json] [--hailo-hef /path/to/superpoint_hailo8.hef]\n' "$0" >&2
}
while (($#)); do
    case "$1" in
        --mask-foreground)
            MASK_FOREGROUND=1
            shift
            ;;
        --no-mask-foreground)
            MASK_FOREGROUND=0
            shift
            ;;
        --hailo-hef)
            if (($# < 2)) || [[ "$2" == --* ]]; then
                usage
                exit 2
            fi
            HAILO_HEF="$2"
            shift 2
            ;;
        --undistort-config)
            if (($# < 2)) || [[ "$2" == --* ]]; then
                usage
                exit 2
            fi
            UNDISTORT_CONFIG="$2"
            shift 2
            ;;
        *)
            usage
            exit 2
            ;;
    esac
done
RUN_ID="${UID}-$$"
NEPI_PIDFILE="/tmp/nepi-disp-${RUN_ID}-nepi.pid"
ONBOARD_PIDFILE="/tmp/nepi-disp-${RUN_ID}-onboard.pid"
SSH_CONTROL_DIR="$(mktemp -d /tmp/nepi-disp-ssh.XXXXXX)"
NEPI_SSH_OPTIONS=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$SSH_CONTROL_DIR/nepi.sock"
    -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p 2222
    -i /home/victoria-zhang/.ssh/nepi_default_ssh_key)
ONBOARD_SSH_OPTIONS=(-o ConnectTimeout=5 -o ControlMaster=auto -o ControlPersist=60
    -o "ControlPath=$SSH_CONTROL_DIR/onboard.sock")
NEPI_SSH_PID=""
ONBOARD_SSH_PID=""
PLOT_PID=""
ONBOARD_MODE="capture"
RUI_RELAY_PORT="$RUI_CAMERA_PORT"

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
        if [[ "$ONBOARD_MODE" == "reuse" ]]; then
            run_onboard "if [ -f '$ONBOARD_PIDFILE' ]; then sudo -n kill -INT \$(cat '$ONBOARD_PIDFILE') 2>/dev/null || true; sudo -n rm -f '$ONBOARD_PIDFILE'; fi" >/dev/null 2>&1 || true
        else
            run_onboard "if [ -f '$ONBOARD_PIDFILE' ]; then kill -INT \$(cat '$ONBOARD_PIDFILE') 2>/dev/null || true; rm -f '$ONBOARD_PIDFILE'; fi" >/dev/null 2>&1 || true
        fi
    fi
    if [[ -n "$NEPI_SSH_PID" ]]; then
        run_nepi "if [ -f '$NEPI_PIDFILE' ]; then kill -INT \$(cat '$NEPI_PIDFILE') 2>/dev/null || true; rm -f '$NEPI_PIDFILE'; fi" >/dev/null 2>&1 || true
    fi
    if [[ -n "$PLOT_PID" ]]; then
        kill -TERM "$PLOT_PID" 2>/dev/null || true
    fi
    [[ -z "$ONBOARD_SSH_PID" ]] || kill "$ONBOARD_SSH_PID" 2>/dev/null || true
    [[ -z "$NEPI_SSH_PID" ]] || kill "$NEPI_SSH_PID" 2>/dev/null || true
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

if [[ ! -f "$SCRIPTS_DIR/optical_flow.py" || ! -f "$SCRIPTS_DIR/live_udp_stabilization.py" || ! -f "$SCRIPTS_DIR/hailo_superpoint.py" || ! -f "$SCRIPTS_DIR/radial_undistort.py" || ! -f "$SCRIPTS_DIR/relay_existing_camera.py" || ! -f "$PLOT" ]]; then
    printf 'Missing NEPI stabilization scripts in %s\n' "$SCRIPTS_DIR" >&2
    exit 1
fi
python3 -c 'import cv2, numpy, tkinter as tk; from PIL import ImageTk; from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg; root = tk.Tk(); root.withdraw(); root.destroy()' || {
    printf 'OpenCV, NumPy, Pillow, Matplotlib, and Tkinter need to be installed with a graphical desktop on this computer.\n' >&2
    exit 1
}
if [[ -n "$HAILO_HEF" ]]; then
    if [[ ! -f "$HAILO_HEF" ]]; then
        printf 'Hailo mode needs a local SuperPoint HEF: %s\n' "$HAILO_HEF" >&2
        exit 1
    fi
fi
if [[ -n "$UNDISTORT_CONFIG" ]]; then
    if [[ ! -f "$UNDISTORT_CONFIG" ]]; then
        printf 'Undistortion preset is missing: %s\n' "$UNDISTORT_CONFIG" >&2
        exit 1
    fi
    VIEWER_UNDISTORT_K1="$(PYTHONPATH="$SCRIPTS_DIR" python3 -c 'import sys; from radial_undistort import load_preset; d=load_preset(sys.argv[1]); assert (d["width"], d["height"]) == (640, 480); print(d["k1"])' "$UNDISTORT_CONFIG")" || {
        printf 'Undistortion preset must be valid JSON for this 640x480 camera.\n' >&2
        exit 1
    }
else
    VIEWER_UNDISTORT_K1="$(PYTHONPATH="$SCRIPTS_DIR" python3 -c 'from radial_undistort import DEFAULT_K1; print(DEFAULT_K1)')"
fi

python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.68.12",0))' || {
    printf 'Computer needs 192.168.68.12 on the ROV Ethernet network. Check the active network profile before starting the test.\n' >&2
    exit 1
}
python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.68.12",5600))' || {
    printf 'Computer camera port 5600 is busy. Stop the old display first.\n' >&2
    exit 1
}
python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.68.12",5601))' || {
    printf 'Computer displacement plot port 5601 is busy.\n' >&2
    exit 1
}

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
if ! run_onboard 'true' >/dev/null 2>&1; then
    printf 'Waiting up to 90 seconds for onboard camera Pi SSH after power-up...\n'
    ONBOARD_SSH_DEADLINE=$((SECONDS + 90))
    until run_onboard 'true' >/dev/null 2>&1; do
        if (( SECONDS >= ONBOARD_SSH_DEADLINE )); then
            printf 'Cannot SSH to onboard camera Pi at 192.168.68.22:22 after 90 seconds. Check its power, Ethernet connection, IP address, and SSH service.\n' >&2
            exit 1
        fi
        sleep 3
    done
    printf 'Onboard camera Pi SSH is ready.\n'
fi
run_onboard 'test -x /home/rishipi5/rov-27/FastMJPG/bin/FastMJPG' || {
    printf 'Onboard FastMJPG is missing or not executable.\n' >&2
    exit 1
}
run_onboard 'command -v fuser >/dev/null' || {
    printf 'Onboard Pi needs fuser to check whether camera 0 is already streaming.\n' >&2
    exit 1
}
CAMERA_HOLDERS="$(run_onboard 'fuser /dev/video0 2>/dev/null || true')"
read -r -a CAMERA_HOLDER_PIDS <<< "$CAMERA_HOLDERS"
if ((${#CAMERA_HOLDER_PIDS[@]})); then
    if ((${#CAMERA_HOLDER_PIDS[@]} != 1)); then
        printf 'Onboard /dev/video0 is held by several processes: %s\n' "$CAMERA_HOLDERS" >&2
        exit 1
    fi
    CAMERA_COMMAND="$(run_onboard "ps -p '${CAMERA_HOLDER_PIDS[0]}' -o args=")"
    if [[ "$CAMERA_COMMAND" != *'FastMJPG capture /dev/video0 640 480 1 30 send 192.168.68.22 5600 192.168.68.12 5600'* ]]; then
        printf 'Onboard /dev/video0 is busy (PID %s): %s\n' "${CAMERA_HOLDER_PIDS[0]}" "$CAMERA_COMMAND" >&2
        printf 'Stop that camera process before running this test.\n' >&2
        exit 1
    fi
    run_onboard 'sudo -n python3 -c "import socket; s=socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3)); s.close()"' || {
        printf 'Reusing the active camera stream requires passwordless sudo for packet capture on the Pi.\n' >&2
        exit 1
    }
    ONBOARD_MODE="reuse"
    if [[ "$CAMERA_COMMAND" == *"192.168.68.103 $RUI_CAMERA_PORT"* ]]; then
        RUI_RELAY_PORT=0
    fi
    printf 'Onboard camera 0 is already streaming; reusing its UDP feed.\n'
fi
run_nepi "python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind((\"192.168.68.103\",0))'" || {
    printf 'NEPI needs 192.168.68.103 on its Ethernet interface. Check the saved NEPI wired aliases.\n' >&2
    exit 1
}
run_nepi "python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind((\"192.168.68.103\",5600))'" || {
    printf 'NEPI camera port 5600 is busy. Stop the old relay first.\n' >&2
    exit 1
}

printf 'Copying the live processor into the NEPI container...\n'
tar --mtime='@0' -C "$SCRIPTS_DIR" -cf - optical_flow.py live_udp_stabilization.py hailo_superpoint.py radial_undistort.py |
    run_nepi 'mkdir -p ~/robot-stab && tar -m -C ~/robot-stab -xf -'
if [[ "$ONBOARD_MODE" == "reuse" ]]; then
    run_onboard 'mkdir -p ~/robot-stab && cat > ~/robot-stab/relay_existing_camera.py' \
        < "$SCRIPTS_DIR/relay_existing_camera.py"
fi
NEPI_BACKEND_ARGS=""
NEPI_MASK_ARG=""
NEPI_UNDISTORT_ARGS=""
if (( MASK_FOREGROUND )); then
    NEPI_MASK_ARG='--mask-foreground'
fi
if [[ -n "$UNDISTORT_CONFIG" ]]; then
    run_nepi 'cat > ~/robot-stab/undistort.json' < "$UNDISTORT_CONFIG"
    NEPI_UNDISTORT_ARGS='--undistort-config ~/robot-stab/undistort.json'
fi
NEPI_FORWARD_ARGS='--forward-ip 192.168.68.12'
if [[ "$ONBOARD_MODE" == "reuse" ]]; then
    NEPI_FORWARD_ARGS=''
fi
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
run_nepi_nostdin "echo \$\$ > '$NEPI_PIDFILE'; exec python3 -u ~/robot-stab/live_udp_stabilization.py $NEPI_FORWARD_ARGS $NEPI_MASK_ARG $NEPI_UNDISTORT_ARGS --telemetry-ip 192.168.68.12 $NEPI_BACKEND_ARGS" &
NEPI_SSH_PID=$!
sleep 1
if ! kill -0 "$NEPI_SSH_PID" 2>/dev/null; then
    printf 'NEPI processor exited during startup.\n' >&2
    exit 1
fi

printf 'Starting camera display with displacement path overlay...\n'
python3 "$PLOT" --bind-ip 192.168.68.12 --source-ip 192.168.68.103 \
    --undistort-k1 "$VIEWER_UNDISTORT_K1" &
PLOT_PID=$!
sleep 1
if ! kill -0 "$PLOT_PID" 2>/dev/null; then
    printf 'Camera display exited during startup.\n' >&2
    exit 1
fi

if [[ "$ONBOARD_MODE" == "reuse" ]]; then
    printf 'Relaying the active onboard camera to NEPI...\n'
    run_onboard_nostdin "exec sudo -n python3 -u /home/rishipi5/robot-stab/relay_existing_camera.py --pid-file '$ONBOARD_PIDFILE' --rui-port '$RUI_RELAY_PORT'" &
else
    printf 'Starting onboard camera...\n'
    run_onboard_nostdin "cd /home/rishipi5/rov-27 && echo \$\$ > '$ONBOARD_PIDFILE' && exec ./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 send 192.168.68.22 5600 192.168.68.103 5600 1400 8000000 2 send 192.168.68.22 5614 192.168.68.103 $RUI_CAMERA_PORT 1400 8000000 1" &
fi
ONBOARD_SSH_PID=$!

printf 'Camera displacement is overlaid locally and printed by NEPI. Press Ctrl+C to stop all processes.\n'
wait -n "$NEPI_SSH_PID" "$ONBOARD_SSH_PID" "$PLOT_PID" || true
printf 'One process stopped; shutting down the test.\n'
