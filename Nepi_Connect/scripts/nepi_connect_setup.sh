#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Sets up the NEPI Connect service on the robot's Pi. It stages the service's
# settings in ~/nepi_connect and adds the nepi_connect_start and
# nepi_connect_stop commands to the user's shell. It installs nothing: the
# service streams with ustreamer, which must already be installed
# (sudo apt install -y ustreamer).
#
# Run it as the user who will start the service, without sudo:
#     bash <this checkout>/Nepi_Connect/scripts/nepi_connect_setup.sh
#
# Safe to re-run, and it keeps your edited settings. The commands run the
# scripts in this checkout, not a copy, so re-run this if the checkout moves.

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${SCRIPT_FOLDER}")/resources

if [[ $EUID -eq 0 ]]; then
    echo "Run this as your own user, without sudo: it sets up that user's shell."
    return 1 2>/dev/null || exit 1
fi

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_connect_bash_utils
if [[ ! -f "$NEPI_UTILS_SOURCE" ]]; then
    echo "NEPI Connect bash utils not found at: ${NEPI_UTILS_SOURCE}"
    return 1 2>/dev/null || exit 1
fi
# Sets NEPI_CONNECT_USER_CONFIG, the settings copy staged below.
source "$NEPI_UTILS_SOURCE"

echo ""
echo "########################"
echo "NEPI CONNECT SETUP"
echo "########################"
echo ""


####################################
# 1. Check what the service needs. Nothing is installed here.

if ! command -v ustreamer >/dev/null 2>&1; then
    echo "WARNING: ustreamer is not installed, so the service cannot stream."
    echo "WARNING: Install it with: sudo apt install -y ustreamer"
fi
if ! id -nG | grep -qw video; then
    echo "WARNING: ${USER} is not in the video group, so ustreamer cannot open the camera."
    echo "WARNING: Fix it with: sudo usermod -aG video ${USER}, then log out and back in."
fi


####################################
# 2. Stage the settings in ~/nepi_connect/config, the copy to edit. An
# existing copy is kept, so re-running this never undoes your changes.

mkdir -p "$(dirname "$NEPI_CONNECT_USER_CONFIG")"
if [[ -f "$NEPI_CONNECT_USER_CONFIG" ]]; then
    echo "Keeping your existing settings: ${NEPI_CONNECT_USER_CONFIG}"
else
    cp "${RESOURCES_FOLDER}/config/nepi_connect_config.sh" "$NEPI_CONNECT_USER_CONFIG"
    echo "Settings staged in: ${NEPI_CONNECT_USER_CONFIG}"
fi


####################################
# 3. Add nepi_connect_start and nepi_connect_stop to every new shell: a copy of
# the bash utils in the home folder, pinned to this checkout, sourced from
# .bashrc.

NEPI_UTILS_FILE_DEST=${HOME}/.nepi_connect_bash_utils
cp "$NEPI_UTILS_SOURCE" "$NEPI_UTILS_FILE_DEST"
echo "export NEPI_CONNECT_RESOURCES_FOLDER='${RESOURCES_FOLDER}'" >> "$NEPI_UTILS_FILE_DEST"
chmod 664 "$NEPI_UTILS_FILE_DEST"

BASHRC=${HOME}/.bashrc
# -x: a commented-out block must not count as already installed.
if ! grep -qxF "##### Source NEPI Connect Bash Utils #####" "$BASHRC" 2>/dev/null; then
    {
        echo ' '
        echo '##### Source NEPI Connect Bash Utils #####'
        echo 'NEPI_CONNECT_UTILS_FILE='${NEPI_UTILS_FILE_DEST}
        echo 'if [ -f ${NEPI_CONNECT_UTILS_FILE} ]; then'
        echo '    . ${NEPI_CONNECT_UTILS_FILE}'
        echo 'fi'
    } >> "$BASHRC"
fi
source "$NEPI_UTILS_FILE_DEST"


echo ""
echo "##################################"
echo "NEPI Connect Setup Complete"
echo "##################################"
echo ""
echo "Open a new terminal (or run: source ~/.bashrc), then:"
echo "    nepi_connect_start    start the service, which starts the camera stream"
echo "    nepi_connect_stop     stop the service and the stream"
echo ""
echo "Settings (camera, resolution, frame rate, port): ${NEPI_CONNECT_USER_CONFIG}"
echo "Service log: /tmp/nepi_connect_service.log"
echo ""
