#
# The find_package macro for Orocos-RTT requires
# cmake >= 2.8
#
cmake_minimum_required(VERSION 2.8)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(giel_component)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# find dependencies
find_package( catkin REQUIRED COMPONENTS cmake_modules)
find_package( roscpp REQUIRED)
find_package( rosbag REQUIRED)
find_package( Eigen REQUIRED)

include_directories(include
	${catkin_INCLUDE_DIRS} 
	${Eigen_INCLUDE_DIRS} 
	${roscpp_INCLUDE_DIRS} 
	${rosbag_INCLUDE_DIRS}
)

#catkin_package(
# INCLUDE_DIRS include
#  CATKIN_DEPENDS
#    rosbag
#    roscpp
#)

#add_library(${giel_component} ${${giel_component}_SRCS})
#add_library(giel_comp src/giel_component.cpp)
#target_link_libraries(giel_component ${rosbag_LIBRARIES})

# Rest of the configuration is in src/
add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
