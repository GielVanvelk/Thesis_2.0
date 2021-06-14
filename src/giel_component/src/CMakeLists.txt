
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libgui_connector-<target>.so
# and installs in the directory lib/orocos/gui_connector/
#
orocos_component(giel_component giel_component.hpp giel_component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/gui_connector/ directory
#
orocos_install_headers(giel_component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

