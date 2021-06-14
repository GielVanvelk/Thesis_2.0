
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library liblwr_fri-<target>.so
# and installs in the directory lib/orocos/lwr_fri/
#
orocos_component(lwr_fri FRIComponent.hpp FRIComponent.cpp)
target_link_libraries(lwr_fri ${kdl_parser_LIBRARIES}  ${orocos_kdl_LIBRARIES} ${tf_conversions_LIBRARIES})

# ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/lwr_fri/ directory
#
orocos_install_headers(FRIComponent.hpp)
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library liblwr_fri-plugin-<target>.so
# and installs in the directory lib/orocos/lwr_fri/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
# orocos_plugin(lwr_fri-plugin lwr_fri-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library liblwr_fri-service-<target>.so
# and installs in the directory lib/orocos/lwr_fri/plugins/
#
# orocos_service(lwr_fri-service lwr_fri-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library liblwr_fri-support-<target>.so and installs it in
# lib/
#
# orocos_library(lwr_fri-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


