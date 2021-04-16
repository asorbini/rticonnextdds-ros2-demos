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
# Helper function to generate a shared library containing DDS type support code
# from a list of ROS 2 messages and (soon) services.
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
