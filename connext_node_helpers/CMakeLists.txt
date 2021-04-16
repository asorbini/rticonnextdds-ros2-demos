cmake_minimum_required(VERSION 3.8)
project(connext_node_helpers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)
find_package(rmw_connextdds REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CONNEXTDDS_DIR}/resource/cmake")
message(STATUS "loading RTI Connext DDS with `rti_connext_cmake_module`")
find_package(RTIConnextDDS REQUIRED)
if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS (${CONNEXTDDS_ARCH}) not found in "
    "'${CONNEXTDDS_DIR}'")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "RTI Connext DDS 6.x required. "
    "Found: ${RTICONNEXTDDS_VERSION}")
  set(RTIConnextDDS_FOUND false)
endif()

add_library(${PROJECT_NAME} SHARED
  src/processor/processor.cpp
  include/connext_node_helpers/processor.hpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp)
ament_export_dependencies(
  rclcpp
  rti_connext_dds_cmake_module
  rmw_connextdds)
ament_export_libraries(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} RTIConnextDDS::cpp2_api)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
    DIRECTORY include/
    DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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

# Install CMake helpers
install(
    FILES
      cmake/connext_node.cmake
      cmake/node_main.cpp.in
    DESTINATION "share/${PROJECT_NAME}/cmake")

ament_package(
  CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
