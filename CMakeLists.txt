cmake_minimum_required(VERSION 2.8.3)
project(photonfocus_camera)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  dynamic_reconfigure
  camera_info_manager
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# EBUS LIBRARY (it is necessary to source the set_puregev_env file inside the ebus sdk bin/ directory)
set(EBUS_INCLUDE_DIRS $ENV{PUREGEV_ROOT}/include)

find_library(EBUS_PvBase_LIB PvBase $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvDevice_LIB PvDevice $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvPersistence_LIB PvPersistence $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvGenICam_LIB PvGenICam $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvStream_LIB PvStream $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvTransmitter_LIB PvTransmitter $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvVirtualDevice_LIB PvVirtualDevice $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvAppUtils_LIB PvAppUtils $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvSerial_LIB PvSerial $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvGUI_LIB PvGUI $ENV{PUREGEV_ROOT}/lib)
find_library(EBUS_PvBuffer_LIB PvBuffer $ENV{PUREGEV_ROOT}/lib)

set(EBUS_LIBRARIES
    ${EBUS_PvBase_LIB}
    ${EBUS_PvDevice_LIB}
    ${EBUS_PvPersistence_LIB}
    ${EBUS_PvGenICam_LIB}
    ${EBUS_PvStream_LIB}
    ${EBUS_PvTransmitter_LIB}
    ${EBUS_PvVirtualDevice_LIB}
    ${EBUS_PvAppUtils_LIB}
    ${EBUS_PvSerial_LIB}
    ${EBUS_PvGUI_LIB}
    ${EBUS_PvBuffer_LIB}
)

add_definitions(-D_UNIX_ -D_LINUX_)
# END EBUS LIBRARY

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

generate_dynamic_reconfigure_options(
  cfg/photonfocus.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES photonfocus_camera
#  CATKIN_DEPENDS cv_bridge image_transport roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EBUS_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(photonfocus_camera
#   src/${PROJECT_NAME}/photonfocus_camera.cpp
# )
add_library(photonfocuslib src/photonfocuscamera.cpp)

## Declare a cpp executable
# add_executable(photonfocus_camera_node src/photonfocus_camera_node.cpp)

# DRIVER
add_executable(photonfocus_node src/photonfocus_node.cpp)

# UTILITIES
add_executable(set_ip src/utilities/set_ip.cpp)
add_executable(find_camera src/utilities/find_camera.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(photonfocus_camera_node photonfocus_camera_generate_messages_cpp)
add_dependencies(photonfocus_node ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
# target_link_libraries(photonfocus_camera_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(photonfocuslib
    ${EBUS_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(photonfocus_node
    photonfocuslib
    ${EBUS_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(set_ip
    ${EBUS_LIBRARIES}
)

target_link_libraries(find_camera
    ${EBUS_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS photonfocus_camera photonfocus_camera_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_photonfocus_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
