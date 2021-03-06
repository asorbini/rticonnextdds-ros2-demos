# (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
cmake_minimum_required(VERSION 3.5)
project(connext_nodes_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# find basic ROS 2 dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

################################################################################
# Load package `connext_node_helpers` which provides some CMake and C++
# utilities, and it also loads RTI Connext DDS 6.x using package
# `rti_connext_dds_cmake_module`.
################################################################################
find_package(connext_node_helpers REQUIRED)

################################################################################
# Generate type supports with `rtiddsgen` from IDL files generated by ROS 2
# and build a library with all type supports.
# We also generate some type support from "standard" IDL files. In this case we
# use the syntax `<file>[@<prefix>]` to specify an optional prefix for the
# location where files will be generated (and that will be used to `#include`
# them in code, e.g. `#include "my/custom/prefix/Foo.hpp"`). This option is
# useful to use an "include policy" for type definition files that is consistent
# with the one used by ROS 2 in the automatically generated IDL files.
################################################################################
connext_generate_typesupport_library(connext_nodes_cpp_types
  MESSAGES
    builtin_interfaces/Time
    std_msgs/String
    std_msgs/Header
    sensor_msgs/PointField
    sensor_msgs/PointCloud2
  IDLS
    idl/ShapeType.idl
    idl/camera/CameraCommon.idl@camera
    idl/camera/CameraCommonFlat.idl@camera
    idl/camera/CameraImage.idl@camera
    idl/camera/CameraImageFlat.idl@camera
    idl/camera/CameraImageFlatZc.idl@camera
    idl/camera/CameraImageZc.idl@camera
  ZEROCOPY)

################################################################################
# Build example stand-alone applications
################################################################################
# Generate a separate type support for these executables, just as an example.
# This function will store a list of generated files in variable
# ${pkg}_${type}_FILES
connext_generate_message_typesupport_cpp(String
  PACKAGE std_msgs
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/chatter
  INSTALL_PREFIX include/chatter)
connext_add_executable(
  NAME talker_main
  SOURCES
    src/chatter/talker_main.cpp
    ${std_msgs_String_FILES}
  INCLUDES
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/chatter)
connext_add_executable(
  NAME listener_main
  SOURCES
    src/chatter/listener_main.cpp
    ${std_msgs_String_FILES}
  INCLUDES
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/chatter)

################################################################################
# Create a library for the "component" Nodes
################################################################################
add_library(connext_node_components SHARED
  src/chatter/talker.cpp
  src/chatter/listener.cpp)
ament_target_dependencies(connext_node_components
  rclcpp
  rclcpp_components
  std_msgs)
target_compile_definitions(connext_node_components
  PRIVATE "CONNEXT_NODES_CPP_BUILDING_DLL")
target_include_directories(connext_node_components
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(connext_node_components
  connext_nodes_cpp_types
  RTIConnextDDS::cpp2_api)
set_target_properties(connext_node_components PROPERTIES ENABLE_EXPORTS true)
install(TARGETS
  connext_node_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

################################################################################
# Build executables for each component using a custom (but similar) macro
################################################################################
connext_components_register_node(connext_node_components
  PLUGIN "rti::connext_nodes_cpp::DdsTalker"
  EXECUTABLE talker)
connext_components_register_node(connext_node_components
  PLUGIN "rti::connext_nodes_cpp::DdsListener"
  EXECUTABLE listener)

################################################################################
# Build applications based on standard IDL files
################################################################################
# Camera example is built as a component library
add_library(camera_components SHARED
  src/camera/camera_pub_flat.cpp
  src/camera/camera_pub_flat_zc.cpp
  src/camera/camera_pub_plain.cpp
  src/camera/camera_pub_zc.cpp
  src/camera/camera_sub_flat.cpp
  src/camera/camera_sub_flat_zc.cpp
  src/camera/camera_sub_plain.cpp
  src/camera/camera_sub_zc.cpp
  include/connext_nodes/camera/CameraImagePublisher.hpp
  include/connext_nodes/camera/CameraImageSubscriber.hpp)
ament_target_dependencies(camera_components
  rclcpp
  rclcpp_components
  connext_node_helpers)
target_compile_definitions(camera_components
  PRIVATE "CONNEXT_NODES_CPP_BUILDING_DLL")
target_include_directories(camera_components
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(camera_components
  connext_nodes_cpp_types
  RTIConnextDDS::cpp2_api)
set_target_properties(camera_components PROPERTIES ENABLE_EXPORTS true)
install(TARGETS
  camera_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImagePublisherPlain"
  EXECUTABLE camera_pub_plain)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImagePublisherFlat"
  EXECUTABLE camera_pub_flat)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImagePublisherFlatZc"
  EXECUTABLE camera_pub_flat_zc)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImagePublisherZc"
  EXECUTABLE camera_pub_zc)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImageSubscriberPlain"
  EXECUTABLE camera_sub_plain)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImageSubscriberFlat"
  EXECUTABLE camera_sub_flat)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImageSubscriberFlatZc"
  EXECUTABLE camera_sub_flat_zc)

connext_components_register_node(camera_components
  PLUGIN "rti::connext_nodes_cpp::camera::CameraImageSubscriberZc"
  EXECUTABLE camera_sub_zc)

# Build equivalent DDS-only applications
connext_add_executable(
  NAME dds_camera_pub
  SOURCES
    src/camera/dds/dds_camera_common.hpp
    src/camera/dds/dds_camera_pub_main.cpp
    src/camera/dds/dds_camera_pub.hpp
    src/camera/dds/dds_camera_pub_plain.cpp
    src/camera/dds/dds_camera_pub_flat.cpp
    src/camera/dds/dds_camera_pub_flat_zc.cpp
    src/camera/dds/dds_camera_pub_zc.cpp
  LIBRARIES
    connext_nodes_cpp_types
  STANDALONE)

connext_add_executable(
  NAME dds_camera_sub
  SOURCES
    src/camera/dds/dds_camera_common.hpp
    src/camera/dds/dds_camera_sub_main.cpp
    src/camera/dds/dds_camera_sub.hpp
    src/camera/dds/dds_camera_sub_plain.cpp
    src/camera/dds/dds_camera_sub_flat.cpp
    src/camera/dds/dds_camera_sub_flat_zc.cpp
    src/camera/dds/dds_camera_sub_zc.cpp
  LIBRARIES
    connext_nodes_cpp_types
  STANDALONE)

################################################################################
# Register ROS 2 package
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
