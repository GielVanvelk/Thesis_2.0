cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# compile xacro files :
file(GLOB def_files 
                    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/tool.xacro 
    )

file(GLOB xacros
                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/pma-lwr.urdf.xacro
		                ${CMAKE_CURRENT_SOURCE_DIR}/robots/pma-lwr-simple.urdf.xacro
		                ${CMAKE_CURRENT_SOURCE_DIR}/robots/pma-lwr1.urdf.xacro
                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/pma-lwr-with-cyl.urdf.xacro
                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/pma-lwr-with-sch.urdf.xacro
    )

rosbuild_find_ros_package(xacro)

set(outfiles "")

foreach( _xacro ${xacros} )
  string(REPLACE ".urdf.xacro" ".urdf" _outfile ${_xacro})
  add_custom_command(
    OUTPUT ${_outfile}
    COMMAND ${xacro_PACKAGE_PATH}/xacro.py
    ARGS ${_xacro} > ${_outfile}
    DEPENDS ${_xacro} ${def_files})
  list(APPEND outfiles ${_outfile})
endforeach( _xacro)

add_custom_target( expanded_defs ALL DEPENDS ${outfiles} )
