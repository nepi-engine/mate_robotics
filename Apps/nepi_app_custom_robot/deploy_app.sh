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
# deploy_app.sh -- deploy THIS app package to a NEPI target.
#
# The script below is byte-identical to the deploy_app.sh shipped with every
# app in nepi_engine_ws/src/nepi_apps. It does TWO different things, and the
# difference is the thing to understand before you use it:
#
#   1. BUILD UPDATES -- rsyncs the WHOLE package folder (scripts, api, params,
#      msg, srv, rui, CMakeLists.txt, package.xml) into the target's source
#      tree at
#          /mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_apps/<app>/
#      Nothing there is live. It takes effect only after a catkin build of the
#      workspace, and rui/ additionally needs a RUI rebuild
#      (build_nepi_rui.sh) because that is what injects your rui_main_class
#      into the RUI's appsClassMap.
#
#   2. LIVE UPDATES -- rsyncs ONLY scripts/ into the running container at
#          /opt/nepi/nepi_engine/lib/<app>/
#      That is the fast edit loop, and it covers the node script and nothing
#      else. A change under api/, params/, msg/ or rui/ that you have only
#      "deployed" is NOT running on the device. If you are debugging a change
#      to api/ and seeing old behaviour, this is why: check what is actually
#      installed before you go looking for the bug elsewhere.
#
# ORPHAN WARNING for api/
#   The workspace build installs this app's api/*.py OVER the shared nepi_api
#   package (see CMakeLists.txt, NEPI_API_FOLDER), and that install does NOT
#   use --delete. A file you REMOVE or RENAME under api/ therefore survives on
#   the device forever, and keeps shadowing whatever nepi_api ships under that
#   name -- through every later build, with no error. Delete the stale file on
#   the target by hand, and never name a file in api/ after something that
#   already exists in nepi_api.
#
# Set NEPI_REMOTE_SETUP (0 = running on the target, 1 = from a dev host) and,
# for remote mode, the NEPI_TARGET_IP just below.
#######################################################################################################

#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=192.168.179.103
     if [[ -n $NEPI_TARGET_IP ]]; then
        NEPI_TARGET_IP=${NEPI_IP}
     fi

    nepi_user_build=nepihost
    nepi_user_live=nepi

#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
     NEPI_SSH_KEY=/home/${USER}/.ssh/nepi_default_ssh_key

#######################################################################################################
# # Clear known hosts keys
# sudo rm /home/${USER}/.ssh/known*
########################################




# Set NEPI folder variables if not configured by nepi aliases bash script
if [[ ! -v NEPI_USER ]]; then
    NEPI_USER=nepi
fi
if [[ ! -v NEPI_HOME ]]; then
    NEPI_HOME=/home/${NEPI_USER}
fi
if [[ ! -v NEPI_DOCKER ]]; then
    NEPI_DOCKER=/mnt/nepi_docker
fi
if [[ ! -v NEPI_STORAGE ]]; then
   NEPI_STORAGE=/mnt/nepi_storage
fi
if [[ ! -v NEPI_CONFIG ]]; then
    NEPI_CONFIG=/mnt/nepi_config
fi
if [[ ! -v NEPI_BASE ]]; then
    NEPI_BASE=/opt/nepi
fi
if [[ ! -v NEPI_RUI ]]; then
    NEPI_RUI=${NEPI_BASE}/nepi_rui
fi
if [[ ! -v NEPI_ENGINE ]]; then
    NEPI_ENGINE=${NEPI_BASE}/nepi_engine
fi
if [[ ! -v NEPI_ETC ]]; then
    NEPI_ETC=${NEPI_BASE}/etc
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
 
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi


echo $(pwd)

###############################################

APP_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
APP_NAME=$(basename "$APP_FOLDER")

###############################################
# Deploy Nepi App
###############################################
SOURCE_PATH=$APP_FOLDER
DEST_PATH="/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_apps/${APP_NAME}"

RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules"
#echo "Excluding ${RSYNC_EXCLUDES}"


echo ""
echo "--------------------------------------------"
echo "DEPLOYING BUILD UPDATES"
echo ""
echo "Syncing App ${APP_NAME} from ${SOURCE_PATH} to NEPI Build Repo at:" 
echo "Destination Path ${DEST_PATH}"
echo ""
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  rsync -avrh  ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${DEST_PATH}/
  echo ""
  if [[ $? -ne 0 ]]; then
    echo "Failed connect to NEPI host at: ${DEST_PATH}"
  else
    echo "Build Updates Deployed"
  fi
elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no  -p 22" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_build}@${NEPI_TARGET_IP}:${DEST_PATH}/
  echo ""
  if [[ $? -ne 0 ]]; then
    echo "Failed connect to NEPI host: ${NEPI_TARGET_IP}"
  else
    echo "Build Updates Deployed"
  fi
fi


###############################################
# Deploy App Scripts Live
###############################################
SOURCE_PATH=${APP_FOLDER}/scripts
DEST_PATH=/opt/nepi/nepi_engine/lib/${APP_NAME}

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
