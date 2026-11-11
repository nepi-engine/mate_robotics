#[[
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
]]
cmake_minimum_required(VERSION 3.5)
project(nepi_app_auto_move)

# sensor_msgs, geometry_msgs, and nav_msgs are not used by this package's own
# messages. They are nepi_interfaces' MSG_DEP_SET, and genmsg resolves a message
# dependency's full dep set through this find_package, so listing nepi_interfaces
# in generate_messages() below without them fails the build with
# "Messages depends on unknown pkg: sensor_msgs". Same pattern as
# nepi_app_obstacles and nepi_app_sim_connector.
find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
nepi_interfaces
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  NepiAppAutoMoveStatus.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   ServiceName.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nepi_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  nepi_interfaces
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/auto_move_app_node.py
  scripts/auto_move_app_img_pub_node.py
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

set(NEPI_SDK_FOLDER
  ${NEPI_ENGINE}/lib/python3/dist-packages/nepi_sdk
)

install(DIRECTORY sdk/
  DESTINATION ${NEPI_SDK_FOLDER}
  FILES_MATCHING PATTERN "*.py"
)

set(NEPI_API_FOLDER
  ${NEPI_ENGINE}/lib/python3/dist-packages/nepi_api
)

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

install(DIRECTORY rui/
  DESTINATION ${NEPI_RUI_SOURCE_FOLDER}
)
