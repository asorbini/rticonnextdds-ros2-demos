# (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.

################################################################################
# Helper function to generate type support code with rtiddsgen from a ROS 2
# message definition.
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
