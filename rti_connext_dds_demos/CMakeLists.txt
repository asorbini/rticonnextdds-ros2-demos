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
project(rti_connext_dds_demos)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT NDDSHOME)
file(TO_CMAKE_PATH "$ENV{NDDSHOME}" NDDSHOME)
endif()
list(APPEND CMAKE_MODULE_PATH "${NDDSHOME}/resource/cmake")
find_package(RTIConnextDDS REQUIRED)

if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS is required to build this package. "
    "Make sure to specify a valid installation via NDDSHOME.")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "This examples require RTI Connext DDS 6.x")
endif()

# Build an example application
function(build_exec exec)
  add_executable(${exec} ${ARGN})
  ament_target_dependencies(${exec} rclcpp std_msgs)
  target_link_libraries(${exec} RTIConnextDDS::cpp2_api)
  set_target_properties(${exec} PROPERTIES ENABLE_EXPORTS true)
  install(
    TARGETS ${exec}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

build_exec(pub src/min_pub.cpp)
build_exec(sub src/min_sub.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
