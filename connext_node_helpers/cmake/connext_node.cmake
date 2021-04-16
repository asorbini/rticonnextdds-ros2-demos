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
# Similar to `rclcpp_components_register_node()` but links Connext DDS directly
################################################################################
function(connext_components_register_node target)
  cmake_parse_arguments(_component
    "" # boolean arguments
    "PLUGIN;EXECUTABLE;RESOURCE_INDEX" # single value arguments
    "" # multi-value arguments
    ${ARGN} # current function arguments
    )

  # use a custom template to generate each node's main, because we must make
  # sure that the DomainParticipantFactory's global static variable is linked
  # into the process. We do this by calling some DDS API that accesses it.
  set(rclcpp_components_NODE_TEMPLATE ${connext_node_helpers_NODE_TEMPLATE})

  rclcpp_components_register_node(${target}
    PLUGIN ${_component_PLUGIN}
    EXECUTABLE ${_component_EXECUTABLE}
    RESOURCE_INDEX ${_component_RESOURCE_INDEX})

  set_target_properties(${_component_EXECUTABLE} PROPERTIES ENABLE_EXPORTS true)
  target_link_libraries(${_component_EXECUTABLE} RTIConnextDDS::cpp2_api)
endfunction()

################################################################################
# Helper function define a custom to generate type support code with rtiddsgen
# from a given IDL file. The function returns the list of generated files as a
# variable in the caller's scope so that it may be passed to add_executable().
################################################################################
function(connext_generate_message_typesupport_cpp type)
  cmake_parse_arguments(_idl
  "" # boolean arguments
  "PACKAGE;OUTPUT_DIR;INSTALL_PREFIX" # single value arguments
  "INCLUDES;DEPENDS" # multi-value arguments
  ${ARGN} # current function arguments
  )

  if(NOT _idl_PACKAGE)
    message(FATAL_ERROR "connext_generate_message_typesupport_cpp passed an "
      "invalid PACKAGE: '${_idl_PACKAGE}'")
  endif()

  if("${_idl_INSTALL_PREFIX}" STREQUAL "")
    set(_idl_INSTALL_PREFIX include)
  endif()

  find_package(${_idl_PACKAGE} REQUIRED)
  set(_pkg_dir "${${_idl_PACKAGE}_DIR}")
  get_filename_component(_pkg_include_dir "${_pkg_dir}/../.." REALPATH)
  get_filename_component(_idl_FILE "${_pkg_dir}/../msg/${type}.idl" REALPATH)
  set(_idl_NS "${_idl_PACKAGE}/msg")
  
  if("${_idl_OUTPUT_DIR}" STREQUAL "")
    set(_idl_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen")
  endif()

  get_filename_component(idl_filename "${_idl_FILE}" NAME)
  get_filename_component(idl_dir "${_idl_FILE}" DIRECTORY)
  string(REGEX REPLACE "\.idl$" "" idl_base "${idl_filename}")
  set(generated_files
    "${_idl_OUTPUT_DIR}/${_idl_NS}/${idl_base}.cxx"
    "${_idl_OUTPUT_DIR}/${_idl_NS}/${idl_base}.hpp"
    "${_idl_OUTPUT_DIR}/${_idl_NS}/${idl_base}Plugin.cxx"
    "${_idl_OUTPUT_DIR}/${_idl_NS}/${idl_base}Plugin.hpp")

  set(pkg_include_dirs "${_pkg_include_dir}")
  foreach(_msg ${_idl_INCLUDES})
    string(REGEX REPLACE "/[^/]*$" "" _msg_pkg "${_msg}")
    get_filename_component(_msg_include "${${_msg_pkg}_DIR}/../.." REALPATH)
    list(APPEND pkg_include_dirs "${_msg_include}")
  endforeach()
  list(REMOVE_DUPLICATES pkg_include_dirs)

  set(pkg_includes)
  foreach(p ${pkg_include_dirs})
    list(APPEND pkg_includes "-I" "${p}")
  endforeach()

  set(_idl_HEADERS ${generated_files})
  list(FILTER _idl_HEADERS INCLUDE REGEX ".*hpp$")

  set(_idl_OUTPUT_VAR ${_idl_PACKAGE}_${type}_FILES)
  
  file(MAKE_DIRECTORY "${_idl_OUTPUT_DIR}/${_idl_NS}")

  set(_idl_CMD)
  list(APPEND _idl_CMD
    "${CONNEXTDDS_DIR}/bin/rtiddsgen"
    "-language"
    "C++11"
    "-d" "${_idl_OUTPUT_DIR}/${_idl_NS}"
    "-replace"
    "-unboundedSupport"
    ${pkg_includes}
    "${_idl_FILE}")

  add_custom_command(OUTPUT ${generated_files}
    COMMAND ${_idl_CMD}
    DEPENDS ${_idl_FILE} ${_idl_DEPENDS})

  set(${_idl_OUTPUT_VAR} ${generated_files} PARENT_SCOPE)

  # Install header files for public consumption
  install(
    FILES ${_idl_HEADERS}
    DESTINATION "${_idl_INSTALL_PREFIX}/${_idl_NS}")
endfunction()


################################################################################
# Helper function to generate a shared library containing type support code
# generated with rtiddsgen from ROS 2 .idl files
################################################################################
function(connext_generate_typesupport_library lib)
  cmake_parse_arguments(_tslib
    "" # boolean arguments
    "INSTALL_PREFIX" # single value arguments
    "MESSAGES;SERVICES;DEPENDS" # multi-value arguments
    ${ARGN} # current function arguments
    )
  
  if("${_tslib_INSTALL_PREFIX}" STREQUAL "")
    set(_tslib_INSTALL_PREFIX include)
  endif()
  
  # Collect list of all included packages
  set(_tslib_PACKAGES)
  foreach(_msg ${_tslib_MESSAGES} ${_tslib_SERVICES})
    string(REGEX REPLACE "/[^/]*$" "" _msg_pkg "${_msg}")
    if(NOT _msg_pkg)
      message(FATAL_ERROR "Invalid package detected for message "
        "'${__msg}': '$(_msg_pkg)'")
    endif()
    list(APPEND _tslib_PACKAGES "${_msg_pkg}")
    find_package(${_msg_pkg} REQUIRED)
  endforeach()

  set(_tslib_OUTPUT_DIR  "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/${lib}")

  # Generate type supports for all types
  set(_tslib_GENERATED_FILES)
  foreach(_msg ${_tslib_MESSAGES})
    string(REGEX REPLACE "/[^/]*$" "" _msg_pkg "${_msg}")
    string(REGEX REPLACE "^${_msg_pkg}/" "" _msg_name "${_msg}")
    set(_msg_includes ${_tslib_MESSAGES})
    list(REMOVE_ITEM _msg_includes "${_msg_pkg}")
    
    connext_generate_message_typesupport_cpp(${_msg_name}
      PACKAGE ${_msg_pkg}
      INCLUDES ${_msg_includes}
      OUTPUT_DIR ${_tslib_OUTPUT_DIR}
      INSTALL_PREFIX ${_tslib_INSTALL_PREFIX}
      DEPENDS ${_tslib_DEPENDS})
    list(APPEND _tslib_GENERATED_FILES ${${_msg_pkg}_${_msg_name}_FILES})
  endforeach()

  # Define library target to build all generated files into shared library
  add_library(${lib} SHARED
    ${_tslib_GENERATED_FILES})
  target_link_libraries(${lib}
    RTIConnextDDS::cpp2_api)
  target_include_directories(${lib}
    PUBLIC
      "$<BUILD_INTERFACE:${_tslib_OUTPUT_DIR}>"
      "$<INSTALL_INTERFACE:include")
  install(
    TARGETS ${lib}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endfunction()

################################################################################
# Helper function to build simple C++ DDS/ROS 2 applications
# Usage: build_exec(NAME <executable_name> SOURCES <source_files> LIBRARIES <libs>)
################################################################################
function(connext_add_executable)
  cmake_parse_arguments(_exec
    "" # boolean arguments
    "NAME" # single value arguments
    "SOURCES;LIBRARIES;DEFINES;PKG_DEPS;INCLUDES" # multi-value arguments
    ${ARGN} # current function arguments
  )

  add_executable(${_exec_NAME} ${_exec_SOURCES})
  ament_target_dependencies(${_exec_NAME}
    rclcpp connext_node_helpers ${_exec_PKG_DEPS})
  # Link RTI Connext DDS' "modern C++" API
  target_link_libraries(${_exec_NAME}
    RTIConnextDDS::cpp2_api ${_exec_LIBRARIES})
  # Set property ENABLE_EXPORTS to link the library with `-rdynamic` and
  # enable sharing of static variables with dynamic libraries.
  set_target_properties(${_exec_NAME} PROPERTIES ENABLE_EXPORTS true)
  target_include_directories(${_exec_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${_exec_INCLUDES})
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