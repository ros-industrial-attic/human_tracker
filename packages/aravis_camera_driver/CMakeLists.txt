cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)y

rosbuild_init()

# For whatever reason the Ubuntu cmake-data package does not contain FindGLIB2.cmake
# Therefore the following line does not find libglib2 as intended and explicit commands 
# will be used instead.
#
# find_package(GLIB2 REQUIRED)
FIND_PATH(GLIB_CONFIG_DIR NAMES glibconfig.h PATHS /usr/lib /usr/lib/i386-linux-gnu /usr/lib/x86_64-linux-gnu ${CMAKE_LIBRARY_PATH} PATH_SUFFIXES glib-2.0/include)
FIND_PATH(GLIB2_DIR NAMES glib.h PATHS /usr/include PATH_SUFFIXES glib-2.0)
SET ( GLIB2_INCLUDE_DIRS ${GLIB_CONFIG_DIR} ${GLIB2_DIR} )
FIND_LIBRARY(GLIB2_LIBRARIES NAMES glib-2.0 glib PATHS /usr/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${GLIB2_INCLUDE_DIRS})

# Dynamic reconfigure support
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Build library
rosbuild_add_library(${PROJECT_NAME} src/camera.cpp src/basler_camera.cpp)
target_link_libraries(${PROJECT_NAME} ${GLIB2_LIBRARIES} yaml-cpp)

rosbuild_add_executable(basler_multi_node src/nodes/basler_multi_node.cpp)
target_link_libraries(basler_multi_node ${PROJECT_NAME} boost_system boost_filesystem)
