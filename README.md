# Example ROS2 Applications using RTI Connext DDS

## Build dependencies

All example application require RTI Connext DDS 6.x and can only be run using
[`rmw_connextdds`](https://github.com/ros2/rmw_connextdds).

Once RTI Connext DDS 6.x has been installed on the build host, build `rmw_connextdds`
in a dedicated workspace:

```sh
# Load your ROS installation, e.g. Foxy.
source /opt/ros/foxy/setup.bash

# Load RTI Connext DDS 6.x
# Replace <NDDSHOME> with your installation path, e.g. ${HOME}/rti_connext_dds-6.0.1
# Replace <ARCH> with the name of the installed target libraries, e.g. x64Linux4gcc7.3.0
source <NDDSHOME>/resource/scripts/rtisetenv_<ARCH>.bash

# Create a workspace to build the RMW and enter it
mkdir -p ws-connext-demos-rmw/src/ros2

cd ws-connext-demos-rmw

# Clone and build rmw_connextdds (use `-b <branch>` to clone a branch for a
# specific release, or leave out to target Rolling)
git clone -b foxy https://github.com/ros2/rmw_connextdds src/ros2/rmw_connextdds

# If you have multiple target libraries installed you might need to select the
# desired one with `--cmake-args -DCONNEXTDDS_ARCH=<ARCH>`
colcon build --symlink-install
```

If your installation contains a binary version of `rmw_connextdds` built with
Connext 5.3.1, you should `unset` variables `CONNEXTDDS_DIR`, `CONNEXTDDS_ARCH`,
and `NDDSHOME` before loading the Connext 6.x installation, to make sure that
the older version will not be picked up by `colcon build`, e.g. if using a 
binary Rolling installation with `rmw_connextdds` already installed:

```sh
source /opt/ros/rolling/setup.bash

unset CONNEXTDDS_DIR \
      CONNEXTDDS_ARCH \
      NDDSHOME

source <NDDSHOME>/resource/scripts/rtisetenv_<ARCH>.bash

...
```

## Build examples

After `rmw_connextdds` has been built with RTI Connext DDS 6.x, load it in the
environment, and build the examples repository:

```sh
source ws-connext-demos-rmw/install/setup.bash

mkdir -p ws-connext-demos/src/rti

cd ws-connext-demos

git clone https://github.com/asorbini/rticonnextdds-ros2-demos src/rti/rticonnextdds-ros2-demos

colcon build --symlink-install
```

## Run examples

Some examples are built as stand-alone executable, which must be run directly,
while a few examples are provided in the form of registered components that
can be run with `ros2 run`. Regardless, all examples must be run with
`rmw_connextdds` as the RMW implementation:

```sh
source ws-connext-demos/install/setup.bash

export RMW_IMPLEMENTATION=rmw_connextdds

# Run a component-ized example
ros2 run connext_nodes_cpp listener

# Run a stand-alone example
./ws-connext-demos/install/connext_nodes_cpp/bin/talker_main
```

## connext_nodes_cpp

This package contains several example ROS 2/Connext hybrid applications.

## connext_node_helpers

This package provides CMake and C++ helpers to facilitate the implementation of
ROS 2 packages based on RTI Connext DDS.

Add this package to your `package.xml`'s dependencies and then load it in your
`CMakeLists.txt`:

- `package.xml`:

  ```xml
  <package format="3">
    <name>my_package</name>
    
    <!-- ... -->

    <depend>connext_node_helpers</depend>
  
    <!-- ... -->
  </package>
  ```

- `CMakeLists.txt`

  ```cmake
  cmake_minimum_required(VERSION 3.5)
  project(my_package)

  # ...

  find_package(connext_node_helpers REQUIRED)

  # ...
  ```

### CMake Helpers

Once loaded in a `CMakeList.txt`, the module offers several CMake functions that
can be used to facilitate some common build task for ROS 2 applications that
want to use RTI Connext DDS.

#### connext_generate_typesupport_library

Generates a shared library containing DDS type support code from a list of ROS 2
messages and (soon) services.

This function takes a list of ROS 2 types and will generate a shared library
after defining appropriate code generation targets for each type using
[`connext_generate_message_typesupport_cpp`](#connext_generate_message_typesupport_cpp).

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

connext_generate_typesupport_library(my_connext_types_lib
  MESSAGES
    std_msgs/String
    std_msgs/Header
    builtin_interfaces/Time
    sensor_msgs/PointField
    sensor_msgs/PointCloud2)
```

#### connext_generate_message_typesupport_cpp

Generates type support code with `rtiddsgen` from a ROS 2 message definition.

This is a "lower-level" helper which only defines a code generation target, and
it will not actually build the generated files.

The list of generated files is returned in an output variable `<pkg>_<type>_FILES`.
Tt is up to the caller to consume it appropriately as part of an `add_executable()`
or `add_library()` command.

The compilation target will also need to be configured with the appropriate
include directories. By default, all files will be generated in
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/<pkg>`, and it is sufficient to add
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen` to the include path (since all files
must always be included as `#include "<pkg>/<type>.idl"`).

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)
find_package(std_msgs REQUIRED)

connext_generate_message_typesupport_cpp(String PACKAGE std_msgs)

add_executable(my_app
  main.c
  ${std_msgs_String_FILES})

target_include_directories(my_app
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen)
```

#### connext_components_register_node

Similar to `rclcpp_components_register_node()`, registers a node from a component
library, and generates an executable to spin it in a `main()` function.

This function differs from `rclcpp_components_register_node()` in that the
generate executable will link RTI Connext DDS directly, and it will also be
properly linked in order to allow sharing of the DomainParticipantFactory
between the node component and the RMW.

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)

# First create a library with node components
add_library(my_node_components SHARED
  my_node.cpp)
ament_target_dependencies(my_node_components
  rclcpp
  rclcpp_components)
target_link_libraries(my_node_components
  RTIConnextDDS::cpp2_api)

# Then register each component into its own executable
connext_components_register_node(my_node_components
  PLUGIN "my_node::MyNode"
  EXECUTABLE my_node)
```

#### connext_add_executable

Build a ROS 2/DDS C++ application.

This function can be used to simplify the definition of a build target for a
ROS 2 C++ application that requires direct access to RTI Connext DDS. The
function will take care of most boiler plate commands required to configure the
target with the required dependencies and the appropriate linkage.

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)
find_package(std_msgs REQUIRED)

# Generate type support files
connext_generate_message_typesupport_cpp(String
  PACKAGE std_msgs)

# Build application and generated files
connext_add_executable(
  NAME my_app
  SOURCES
    my_app.cpp
    ${std_msgs_String_FILES}
  INCLUDES
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen)
```
