#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

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
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no  -p 22" --delete ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_build}@${NEPI_TARGET_IP}:${DEST_PATH}/
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


RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules"
#echo "Excluding ${RSYNC_EXCLUDES}"

echo ""
echo "--------------------------------------------"
echo "DEPLOYING LIVE UPDATES"
echo ""

SUCCESS=1


############################
# Deploy Live Scripts
SOURCE_PATH=${APP_FOLDER}/scripts
DEST_PATH=/opt/nepi/nepi_engine/lib/${APP_NAME}
if [[ $SUCCESS -eq 1 && -d $SOURCE_PATH ]]; then
  echo ""
  echo "Sending Live Updates from Source Path ${SOURCE_PATH}"
  echo "to Destination Path ${DEST_PATH}"
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no -p 2222" --delete ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_live}@${NEPI_TARGET_IP}:${DEST_PATH}/ 2> /dev/null
  if [[ $? -ne 0 ]]; then
    if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
      local_host_ip="localhost"
    elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
      local_host_ip=$NEPI_TARGET_IP
    fi
    echo "Failed connect to a running NEPI container on host: ${local_host_ip}"
    echo "Live Updates Failed"
    SUCCESS=0
  fi
fi

############################
# Deploy Live SDK
SOURCE_PATH=${APP_FOLDER}/sdk
DEST_PATH=/opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_sdk
if [[ $SUCCESS -eq 1 && -d $SOURCE_PATH ]]; then
  echo ""
  echo "Sending Live Updates from Source Path ${SOURCE_PATH}"
  echo "to Destination Path ${DEST_PATH}"
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no -p 2222" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_live}@${NEPI_TARGET_IP}:${DEST_PATH}/ 2> /dev/null
  if [[ $? -ne 0 ]]; then
    if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
      local_host_ip="localhost"
    elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
      local_host_ip=$NEPI_TARGET_IP
    fi
    echo "Failed connect to a running NEPI container on host: ${local_host_ip}"
    echo "Live Updates Failed"
    SUCCESS=0
  fi
fi

############################
# Deploy Live API
SOURCE_PATH=${APP_FOLDER}/api
DEST_PATH=/opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_api
if [[ $SUCCESS -eq 1 && -d $SOURCE_PATH ]]; then
  echo ""
  echo "Sending Live Updates from Source Path ${SOURCE_PATH}"
  echo "to Destination Path ${DEST_PATH}"
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no -p 2222" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_live}@${NEPI_TARGET_IP}:${DEST_PATH}/ 2> /dev/null
  if [[ $? -ne 0 ]]; then
    if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
      local_host_ip="localhost"
    elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
      local_host_ip=$NEPI_TARGET_IP
    fi
    echo "Failed connect to a running NEPI container on host: ${local_host_ip}"
    echo "Live Updates Failed"
    SUCCESS=0
  fi
fi


############################
# Deploy Live RUI
SOURCE_PATH=${APP_FOLDER}/rui
DEST_PATH="${NEPI_BASE}/nepi_rui/src/rui_webserver/rui-app/src"
if [[ $SUCCESS -eq 1 && -d $SOURCE_PATH ]]; then
  echo ""
  echo "Sending Live Updates from Source Path ${SOURCE_PATH}"
  echo "to Destination Path ${DEST_PATH}"
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no -p 2222" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/* ${nepi_user_live}@${NEPI_TARGET_IP}:${DEST_PATH}/ 2> /dev/null
  if [[ $? -ne 0 ]]; then
    if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
      local_host_ip="localhost"
    elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
      local_host_ip=$NEPI_TARGET_IP
    fi
    echo "Failed connect to a running NEPI container on host: ${local_host_ip}"
    echo "Live Updates Failed"
    SUCCESS=0
  fi
fi