#[[
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
]]
#
# Install rules below are the ones every app in nepi_engine_ws/src/nepi_apps
# uses. Four of them put files OUTSIDE this package, and it is worth knowing
# where before you add or remove a file:
#
#   scripts/*.py      -> lib/<pkg_name>/        (catkin bin, as listed below)
#   params/           -> ${NEPI_ENGINE}/share/nepi_apps/params   (apps_mgr)
#                    AND ${NEPI_RUI}/.../src/apps                (RUI codegen)
#   api/*.py          -> ${NEPI_ENGINE}/.../dist-packages/nepi_api
#   rui/              -> ${NEPI_RUI}/.../rui-app/src             (FLAT)
#   msg/*.msg         -> generated into the <pkg_name>.msg Python module
#   etc/              -> catkin global etc
#
# TWO CONSEQUENCES:
#   * api/ and rui/ install INTO shared folders, and neither install uses
#     --delete. A file you remove or rename here survives on the device and
#     keeps shadowing whatever nepi_api / the RUI ships under that name.
#     Delete the stale file on the target by hand.
#   * rui/ is flattened into ONE folder shared by every app, so a .js basename
#     is globally unique or it silently overwrites another app's component.
#
cmake_minimum_required(VERSION 3.5)
project(nepi_app_custom_robot)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  NepiAppCustomRobotStatus.msg
)

## Generate services in the 'srv' folder.
## Services ARE supported and used -- nepi_app_onvif_mgr ships four .srv files
## and registers them through NodeClassIF's services_dict. Uncomment and list
## yours here if you need them; a services_dict entry takes
## {'namespace', 'topic', 'srv', 'req', 'resp', 'callback'}.
# add_service_files(
#   FILES
#   ServiceName.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## custom_robot_rbx_if.py is imported by the node as a plain sibling module,
## so it must be installed into the same lib/<pkg_name> directory. A catkin
## install installs only what is listed here; deploy_app.sh's live path works
## either way because it rsyncs the whole scripts/ directory.
catkin_install_python(PROGRAMS
  scripts/custom_robot_app_node.py
  scripts/custom_robot_rbx_if.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY etc/
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(NOT NEPI_BASE)
  set(NEPI_BASE /opt/nepi)
endif()

if(NOT NEPI_RUI)
  set(NEPI_RUI ${NEPI_BASE}/nepi_rui)
endif()

if(NOT NEPI_ENGINE)
  set(NEPI_ENGINE ${NEPI_BASE}/nepi_engine)
endif()

## Install NEPI Engine App Files
set(NEPI_APP_PARAMS_FOLDER
  ${NEPI_ENGINE}/share/nepi_apps/params
)

install(DIRECTORY params/
  DESTINATION ${NEPI_APP_PARAMS_FOLDER}
)

set(NEPI_API_FOLDER
  ${NEPI_ENGINE}/lib/python3/dist-packages/nepi_api
)

# NOTE: no --delete. See the orphan warning at the top of this file.
install(DIRECTORY api/
  DESTINATION ${NEPI_API_FOLDER}
  FILES_MATCHING PATTERN "*.py"
)

## Install NEPI RUI Files
set(NEPI_RUI_PARAMS_FOLDER
  ${NEPI_RUI}/src/rui_webserver/rui-app/src/apps
)

install(DIRECTORY params/
  DESTINATION ${NEPI_RUI_PARAMS_FOLDER}
)

set(NEPI_RUI_SOURCE_FOLDER
  ${NEPI_RUI}/src/rui_webserver/rui-app/src
)

# Flat install into the shared RUI source folder: basenames are globally
# unique across all app packages. Keep non-source files (editor workspaces,
# notes) OUT of rui/ -- everything in here lands in the RUI source tree.
install(DIRECTORY rui/
  DESTINATION ${NEPI_RUI_SOURCE_FOLDER}
)
