#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

#######################################################################################################
# deploy_nepi_drivers.sh [cat ...]
#
# Deploys NEPI template driver files to the correct place in the target's
# nepi_engine_ws source tree. With no arguments it deploys ALL driver types;
# pass one or more categories to deploy just those, e.g.:
#     ./deploy_nepi_drivers.sh idx
#     ./deploy_nepi_drivers.sh idx ptx
#
# NEPI drivers are NOT per-driver folders on the target -- each category's
# .py and .yaml files must land FLAT inside:
#
#     <src>/nepi_engine_ws/src/nepi_drivers/<cat>_drivers/
#
# (drivers_mgr scans the installed copies of those folders; the build installs
#  <cat>_drivers/*.py and *.yaml wholesale -- see DRIVER_ARCHITECTURE.md Section 7)
#
# The script requires the following environment variable be set
#    NEPI_REMOTE_SETUP: Indicates whether running from development host or directly on target
#                      (1 = Dev. Host, 0 = From Target)
# In the case that NEPI_REMOTE_SETUP == 1, some further environment variables must be set
#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=${NEPI_IP} #/${NEPI_DEVICE_ID}
#    NEPI_TARGET_USERNAME: Target username
    nepihost=nepi
    if [[ "$NEPI_IN_CONTAINER" -eq 1 ]]; then
      nepihost=nepihost
    fi

     NEPI_TARGET_USERNAME=${nepihost}
#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
#    Honor an already-set NEPI_SSH_KEY / NEPI_SSH_KEY_PATH (the nepi env exports
#    the latter), otherwise fall back to the standard nepi key under ~/.ssh.
     NEPI_SSH_KEY=${NEPI_SSH_KEY:-${NEPI_SSH_KEY_PATH:-/home/${USER}/.ssh/nepi_default_ssh_key}}
#    NEPI_TARGET_SRC_DIR: Directory to deploy source code to
     NEPI_TARGET_SRC_DIR=/mnt/nepi_storage/nepi_src
#######################################################################################################

REPO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

ALL_DRIVER_CATS="idx lsx ptx npx rbx svx"

# Categories from args, or all of them
if [ "$#" -gt 0 ]; then
  DRIVER_CATS="$@"
else
  DRIVER_CATS=${ALL_DRIVER_CATS}
fi


# Set NEPI folder variables if not configured by nepi aliases bash script
if [[ ! -v NEPI_USER ]]; then
    NEPI_USER=nepi
fi
if [[ ! -v NEPI_HOME ]]; then
    NEPI_HOME=/home/${NEPI_USER}
fi
if [[ ! -v NEPI_STORAGE ]]; then
   NEPI_STORAGE=/mnt/nepi_storage
fi


if [[ -z "${NEPI_REMOTE_SETUP}" ]]; then
  echo "Must have environtment variable NEPI_REMOTE_SETUP set"
  exit 1
fi

if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    echo "Running in Local Mode"

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then

  if [[ -z "${NEPI_TARGET_IP}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_IP be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_TARGET_USERNAME}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_USERNAME be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi


## Sync update remote clock if needed
## sshnhc is a shell function from ~/.nepi_remote_aliases; it is NOT exported, so
## a child script only sees it if the caller has run `export -f sshnhc`. Guard the
## call so a missing helper doesn't abort/spam the deploy.
echo "Syncing remote clock if needed"
if [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  if command -v sshnhc >/dev/null 2>&1; then
    sshnhc
  else
    echo "  Skipping clock sync: 'sshnhc' not defined in this shell (source ~/.nepi_remote_aliases or 'export -f sshnhc' to enable)"
  fi
fi


# Only .py and .yaml belong in the driver folders (matches the CMake install pattern).
# NOTE: filters MUST be a quoted array -- an unquoted "*" in a string variable
# gets glob-expanded by the shell into extra rsync source arguments.
RSYNC_FILTERS=(--exclude '.git' --exclude '.gitmodules' --exclude '__pycache__'
               --include '*.py' --include '*.yaml' --exclude '*')
echo "Using rsync filters: ${RSYNC_FILTERS[*]}"

for DRIVER_CAT in ${DRIVER_CATS}; do

  DRIVER_SOURCE_PATH=${REPO_FOLDER}/${DRIVER_CAT}_drivers
  DRIVER_TARGET_PATH=${NEPI_TARGET_SRC_DIR}/nepi_engine_ws/src/nepi_drivers/${DRIVER_CAT}_drivers

  if [ ! -d "${DRIVER_SOURCE_PATH}" ]; then
    echo "Skipping ${DRIVER_CAT}: source folder not found: ${DRIVER_SOURCE_PATH}"
    continue
  fi

  echo ""
  echo "--------------------------------------------"
  echo "DEPLOYING BUILD UPDATES"
  echo ""
  echo "Syncing ${DRIVER_CAT} drivers from ${DRIVER_SOURCE_PATH} to ${DRIVER_TARGET_PATH}"

  if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    rsync -avrh "${RSYNC_FILTERS[@]}" "${DRIVER_SOURCE_PATH}/" "${DRIVER_TARGET_PATH}/"

  elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
    rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no" "${RSYNC_FILTERS[@]}" "${DRIVER_SOURCE_PATH}/" "${NEPI_TARGET_USERNAME}@${NEPI_TARGET_IP}:${DRIVER_TARGET_PATH}/"

  fi

SOURCE_PATH=${DRIVER_SOURCE_PATH}
DEST_PATH=/opt/nepi/nepi_engine/lib/nepi_drivers

RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules"
#echo "Excluding ${RSYNC_EXCLUDES}"

echo ""
echo "--------------------------------------------"
echo "DEPLOYING LIVE UPDATES"
echo ""
echo "Syncing App ${APP_NAME} from ${SOURCE_PATH} to NEPI Live Folders at:" 
echo "Destination Path ${DEST_PATH}"
echo ""
rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no -p 2222" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_live}@${NEPI_TARGET_IP}:${DEST_PATH}/ 2> /dev/null
echo ""
if [[ $? -ne 0 ]]; then
  if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    local_host_ip="localhost"
  elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
    local_host_ip=$NEPI_TARGET_IP
  fi
  echo "Failed connect to a running NEPI container on host: ${local_host_ip}"
  echo "Live Updates Failed"
else
  echo "Live Updates Deployed"
fi


done
