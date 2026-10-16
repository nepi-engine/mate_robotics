#!/bin/bash
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine ws (${NEPI_REPO_NAME}) repo
# (see https://github.com/nepi-engine/${NEPI_REPO_NAME})
#
# License: NEPI Engine WS Tools and NEPI software deployed and/or compiled with these tools
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

#######################################################################################################
# Usage: $ ./deploy_nepi_engine_complete.sh
#
# This script copies the complete nepi_engine source code to proper filesystem locations on target
# hardware in preparation for building nepi-engine from source. 
#
#
# The script requires the following environment variable be set
#    NEPI_REMOTE_SETUP: Indicates whether running from development host or directly on target 
#                      (1 = Dev. Host, 0 = From Target)
# In the case that NEPI_REMOTE_SETUP == 1, some further environment variables must be set
#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=${NEPI_IP} #/${NEPI_DEVICE_ID}
     echo "Using target IP: ${NEPI_TARGET_IP}"
#    NEPI_DEPLOY_USERNAME: Target username

     NEPI_DEPLOY_USERNAME=nepihost
     NEPI_SSH_PORT=22
#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
     NEPI_SSH_KEY=/home/${USER}/.ssh/nepi_default_ssh_key
#    NEPI_TARGET_SRC_DIR: Directory to deploy source code to
     NEPI_TARGET_SRC_DIR=/mnt/nepi_storage/nepi_src
#    NEPI_SETUP_SRC_DIR: Directory to deploy setup source to
     NEPI_SETUP_SRC_DIR=/home/nepihost
#######################################################################################################
# # Clear known hosts keys
# sudo rm /home/${USER}/.ssh/known*
########################################
# sudo -v





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
  echo "Running in Remote Mode"
  if [[ -z "${NEPI_TARGET_IP}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_IP be assigned"
    exit 1
  fi

  if [[ -z "${NEPI_DEPLOY_USERNAME}" ]]; then
    echo "Remote setup requires env. variable NEPI_DEPLOY_USERNAME be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi

echo "Deploying to NEPI target IP: ${NEPI_TARGET_IP} (port ${NEPI_SSH_PORT})"
# ## Synce update remote clock if needed
# echo "Syncing remote clock if needed"
# if [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
#   sshnhc
# fi
source /home/${USER}/.bashrc

reset_path=$(pwd)

###############################################
# Deploy Gazebo Environment Config
###############################################

SCRIPTS_SOURCE_PATH=$(pwd)/ENVIRONMENT_CONFIGS
SCRIPTS_DEST_PATH=/mnt/nepi_storage/databases/sims/gazebo/ENVIRONMENT_CONFIGS
echo ""
echo "Syncing NEPI ENVIRONMENT_CONFIGS from ${SCRIPTS_SOURCE_PATH} to ${SCRIPTS_DEST_PATH}"
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  rsync -arh ${SCRIPTS_SOURCE_PATH}/ ${SCRIPTS_DEST_PATH}/

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  rsync -azhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" ${SCRIPTS_SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SCRIPTS_DEST_PATH}/

fi

SCRIPTS_SOURCE_PATH=$(pwd)/ROBOT_CONFIGS
SCRIPTS_DEST_PATH=/mnt/nepi_storage/databases/sims/gazebo/ROBOT_CONFIGS
echo ""
echo "Syncing NEPI ROBOT_CONFIGS from ${SCRIPTS_SOURCE_PATH} to ${SCRIPTS_DEST_PATH}"
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  rsync -arh ${SCRIPTS_SOURCE_PATH}/ ${SCRIPTS_DEST_PATH}/

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  rsync -azhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" ${SCRIPTS_SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SCRIPTS_DEST_PATH}/

fi

cd $reset_path