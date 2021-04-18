cmake_minimum_required(VERSION 3.8)
project(connext_msgs)

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
find_package(connext_node_helpers REQUIRED)

# Load all message packages
set(MESSAGE_BROKEN
  actionlib_msgs/GoalStatusArray
  map_msgs/ProjectedMap
  nav_msgs/OccupancyGrid
  nav_msgs/Odometry
  sensor_msgs/MultiDOFJointState
  test_msgs/Arrays
  test_msgs/BoundedSequences
  test_msgs/Defaults
  test_msgs/MultiNested
  test_msgs/UnboundedSequences
  trajectory_msgs/MultiDOFJointTrajectory
  trajectory_msgs/MultiDOFJointTrajectoryPoint
  visualization_msgs/InteractiveMarker
  visualization_msgs/InteractiveMarkerControl
  visualization_msgs/InteractiveMarkerFeedback
  visualization_msgs/InteractiveMarkerInit
  visualization_msgs/InteractiveMarkerUpdate
  visualization_msgs/Marker
  visualization_msgs/MarkerArray
  )

get_filename_component(pkg_msg_dir "${${pkg}_DIR}/../msg" REALPATH)
file(GLOB_RECURSE idl_files "${CMAKE_CURRENT_SOURCE_DIR}/idl/*.idl")

set(idl_input)
foreach(f ${idl_files})
  string(REGEX REPLACE "^${CMAKE_CURRENT_SOURCE_DIR}/idl/" "" idl_rel "${f}")
  get_filename_component(idl_pkg_msg "${idl_rel}" DIRECTORY)
  string(REGEX REPLACE "/msg$" "" idl_pkg "${idl_pkg_msg}")
  get_filename_component(idl_name_ext "${idl_rel}" NAME)
  string(REGEX REPLACE "[.]idl$" "" idl_name "${idl_name_ext}")

  list(FIND MESSAGE_BROKEN "${idl_pkg}/${idl_name}" idl_skip)

  if(idl_skip GREATER -1)
    message(STATUS "skipped: ${idl_pkg}/${idl_name}")
    continue()
  endif()
  list(APPEND idl_input "${f}@${idl_pkg_msg}")
endforeach()

connext_generate_typesupport_library(${PROJECT_NAME}
  IDLS ${idl_input}
  INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/idl
  ZEROCOPY)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
