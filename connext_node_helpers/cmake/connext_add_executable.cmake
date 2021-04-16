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
# Helper function to build simple C++ DDS/ROS 2 applications
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
