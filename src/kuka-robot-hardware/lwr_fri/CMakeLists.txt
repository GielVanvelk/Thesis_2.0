#
# The find_package macro for Orocos-RTT requires
# cmake >= 2.8
#
cmake_minimum_required(VERSION 2.8)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(lwr_fri)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(orocos_kdl)
find_package(tf_conversions)
include_directories(${orocos_kdl_INCLUDE_DIRS} ${tf_conversions_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS} ${tf_conversions_LIBRARY_DIRS})

find_package(kdl_parser)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS} include)

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library liblwr_fri-types-<target>.so
# and installs in the directory lib/orocos/@target@/lwr_fri/types/
#
# The header will go in include/orocos/lwr_fri/types/lwr_fri/lwr_fri-types.hpp
# So you can #include <lwr_fri/lwr_fri-types.hpp>
#
# orocos_typegen_headers(include/lwr_fri/lwr_fri-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
