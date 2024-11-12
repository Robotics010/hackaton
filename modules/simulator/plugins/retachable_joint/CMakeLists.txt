# gz_add_system(detachable-joint
# SOURCES
# DetachableJoint.cc
# PUBLIC_LINK_LIBS
#   ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
# )

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)
# find_package(ignition-common4 REQUIRED)

project(detachable_joint)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)

add_library(ignition-gazebo-retachable-joint-system SHARED RetachableJoint.cc)
set_property(TARGET ignition-gazebo-retachable-joint-system PROPERTY CXX_STANDARD 17)

#   PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
target_link_libraries(ignition-gazebo-retachable-joint-system
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6)
