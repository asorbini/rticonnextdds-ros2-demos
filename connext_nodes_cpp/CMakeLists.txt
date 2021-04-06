################################################################################
# (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################
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

# find basic ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# rclcpp's dependency chain will already include `rti_connext_dds_cmake_module`
# if environment variable RMW_IMPLEMENTATION is set to `rmw_connextdds` when the
# project is configured with cmake. Since that might not always be the case,
# we load the package manually to make sure that we use the same RTI Connext DDS
# installation and the exact same architecture as `rmw_connextdds`.
# This is required for things to work properly.
find_package(rti_connext_dds_cmake_module REQUIRED)
list(INSERT CMAKE_MODULE_PATH 0 "${CONNEXTDDS_DIR}/resource/cmake")
message(STATUS "loading RTI Connext DDS with `rti_connext_cmake_module`")
find_package(RTIConnextDDS REQUIRED)
if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS (${CONNEXTDDS_ARCH}) not found in "
    "'${CONNEXTDDS_DIR}'")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "RTI Connext DDS 6.x required. "
    "Found: ${RTICONNEXTDDS_VERSION}")
endif()

# We reuse the IDL files that are automatically generated from .msg files.
# These are installed in <package>/msg, while variable ${<package>_DIR} points
# to <package>/cmake. We also need to cache the root installation directory 
# so that we can add it to rtiddsgen's include path.
set(std_msgs_IDL_DIR ${std_msgs_DIR}/../msg)
set(ROOT_IDL_DIR    ${std_msgs_DIR}/../../)
set(BUILD_IDL_DIR   ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen)

# Helper function to build an example application
# Usage: build_exec(NAME <executable_name> SOURCES <source_files> LIBRARIES <libs>)
function(build_exec)
  cmake_parse_arguments(_exec
    "" # boolean arguments
    "NAME" # single value arguments
    "SOURCES;LIBRARIES;DEFINES" # multi-value arguments
    ${ARGN} # current function arguments
  )

  add_executable(${_exec_NAME} ${_exec_SOURCES})
  ament_target_dependencies(${_exec_NAME} rclcpp)
  # Link RTI Connext DDS' "modern C++" API
  target_link_libraries(${_exec_NAME}
    RTIConnextDDS::cpp2_api ${_exec_LIBRARIES})
  # Set property ENABLE_EXPORTS to link the library with `-rdynamic` and
  # enable sharing of static variables with dynamic libraries.
  set_target_properties(${_exec_NAME} PROPERTIES ENABLE_EXPORTS true)
  target_include_directories(${_exec_NAME}
    PRIVATE
      ${BUILD_IDL_DIR}
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  if(_exec_DEFINES)
    target_compile_definitions(${_exec_NAME} PRIVATE ${_exec_DEFINES})
  endif()
  install(
    TARGETS ${_exec_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

# Helper function define a custom to generate type support code with rtiddsgen
# from a given IDL file. The function returns the list of generated files as a
# variable in the caller's scope so that it may be passed to add_executable().
function(idl_generate idl_file files_out)
  get_filename_component(idl_filename "${idl_file}" NAME)
  get_filename_component(idl_dir "${idl_file}" DIRECTORY)
  string(REGEX REPLACE "\.idl$" "" idl_base "${idl_filename}")
  set(generated_files
    ${BUILD_IDL_DIR}/${idl_base}.cxx
    ${BUILD_IDL_DIR}/${idl_base}.hpp
    ${BUILD_IDL_DIR}/${idl_base}Plugin.cxx
    ${BUILD_IDL_DIR}/${idl_base}Plugin.hpp)
  file(MAKE_DIRECTORY ${BUILD_IDL_DIR})
  message(STATUS "GENERATE FILES from ${idl_file}: ${generated_files}")
  add_custom_command(OUTPUT ${generated_files}
    COMMAND ${CONNEXTDDS_DIR}/bin/rtiddsgen -language C++11
                                            -d ${BUILD_IDL_DIR}
                                            -replace
                                            -unboundedSupport
                                            -I ${ROOT_IDL_DIR}
                                            ${idl_file}
    DEPENDS ${idl_file}
    VERBATIM)
  set(${files_out} ${generated_files} PARENT_SCOPE)
endfunction()

# Generate type support for std_msgs::msg::String
idl_generate(${std_msgs_IDL_DIR}/String.idl String_IDL_FILES)

# Build a library with all type supports
set(types_lib connext_nodes_cpp_types)
add_library(${types_lib} SHARED ${String_IDL_FILES})
target_link_libraries(${types_lib} RTIConnextDDS::cpp2_api)
install(
  TARGETS ${types_lib}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build example applications
build_exec(
  NAME pub
  SOURCES src/min_pub.cpp
  LIBRARIES ${types_lib})
build_exec(
  NAME sub
  SOURCES src/min_sub.cpp
  LIBRARIES ${types_lib})

set(processor_chatter_src
  src/processor/processor_chatter.cpp
  include/connext_nodes/processor.hpp
  include/connext_nodes/processor_chatter.hpp)
build_exec(
  NAME processor_chatter_ros
  SOURCES
    ${processor_chatter_src}
    include/connext_nodes/processor_chatter_ros.hpp
  LIBRARIES ${types_lib}
  DEFINES CHATTER_USE_DDS=0)
build_exec(
  NAME processor_chatter_dds
  SOURCES
    ${processor_chatter_src}
    include/connext_nodes/processor_chatter_dds.hpp
  LIBRARIES ${types_lib}
  DEFINES CHATTER_USE_DDS=1)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
