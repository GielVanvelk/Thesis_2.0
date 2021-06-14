
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libati_iface-<target>.so
# and installs in the directory lib/orocos/ati_iface/
#
orocos_component(ati_iface ati_iface-component.hpp ati_iface-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/ati_iface/ directory
#
orocos_install_headers(ati_iface-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
