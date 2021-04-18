# Example ROS2 Applications using RTI Connext DDS

This repository contains a collection of example ROS 2/Connext hybrid applications
(`connext_cpp_nodes`), and some helper resources to simplify the implementation of
this class of ROS 2 applications (`connext_node_helpers`).

The repository also contains a helper library (`connext_msgs`) which contains
RTI Connext DDS type support for most types included in the ROS 2 distribution.

- [Build repository](#build-repository)
- [Run examples](#run-examples)
- [Package `connext_nodes_cpp`](#package-connext_nodes_cpp)
  - [camera](#camera)
  - [processor_chatter](#processor_chatter)
  - [talker/listener](#talkerlistener)
- [Package `connext_node_helpers`](#package-connext_node_helpers)
  - [CMake Helpers](#cmake-helpers)
    - [connext_generate_typesupport_library](#connext_generate_typesupport_library)
    - [connext_generate_message_typesupport_cpp](#connext_generate_message_typesupport_cpp)
    - [connext_components_register_node](#connext_components_register_node)
    - [connext_add_executable](#connext_add_executable)
- [Package `connext_msgs`](#package-connext_msgs)
  - [Use DDS types in a ROS 2 application](#use-dds-types-in-a-ros-2-application)
  - [Included packages](#included-packages)
  - [Unsupported types](#unsupported-types)
- [License](#license)

## Build repository

All included packages require RTI Connext DDS 6.x, and generated applications
can only be run using [`rmw_connextdds`](https://github.com/ros2/rmw_connextdds).

Once RTI Connext DDS 6.x is installed, you can clone an build all required
packages in a single workspace:

```sh
# Load your ROS installation, e.g. Foxy.
source /opt/ros/foxy/setup.bash

# Load RTI Connext DDS 6.x
# Replace <NDDSHOME> with your installation path, e.g. ${HOME}/rti_connext_dds-6.0.1
# Replace <ARCH> with the name of the installed target libraries, e.g. x64Linux4gcc7.3.0
source <NDDSHOME>/resource/scripts/rtisetenv_<ARCH>.bash

# Create a workspace and enter it
mkdir -p ws-connext/src

cd ws-connext

# Clone rmw_connextdds (use `-b <branch>` to clone a branch for a specific
# release, or leave it out to target Rolling)
git clone -b foxy https://github.com/ros2/rmw_connextdds src/rmw_connextdds

# Clone the examples repository
git clone https://github.com/asorbini/rticonnextdds-ros2-demos src/rticonnextdds-ros2-demos

# If you have multiple target libraries installed you might need to select the
# desired one with `--cmake-args -DCONNEXTDDS_ARCH=<ARCH>`.
# Omit the `--package-skip` argument if you plan on using package `connext_msgs`.
colcon build --symlink-install --packages-skip connext_msgs
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

## Run examples

Some examples are built as stand-alone executables, and they must be run directly,
while other examples are provided as `rclcpp` components that can be run with
`ros2 run`. All examples must be run with `rmw_connextdds` as the RMW implementation:

```sh
source ws-connext/install/setup.bash

export RMW_IMPLEMENTATION=rmw_connextdds

# Run a component-ized example
ros2 run connext_nodes_cpp listener

# Run a stand-alone example
./ws-connext/install/connext_nodes_cpp/bin/talker_main
```

## Package `connext_nodes_cpp`

This package contains several example ROS 2/Connext hybrid applications.

Most examples are available both as a stand-alone executable and as an `rclcpp`
component. When built as a stand-alone executable, the generated binary will
use the `_main` suffix to differentiate it from the "component-ized" version.

### camera

[Source code](connext_nodes_cpp/src/camera)

This example demonstrates the benefits of *Zero Copy Transfer over Shared Memory*
(Zero-Copy), and *FlatData Language Binding* (Flat-Data) features provided by
RTI Connext DDS.

The examples perform a simple latency test between a publisher and a subscriber
applications using large data samples exchanged in a "ping/pong" fashion.
The computed roundtrip time is halved and a running average will be printed
periodically while the test is being performed.

The testers applications are provided in different variants, each one using a
different memory binding (plain, or Flat-Data), and a different transfer method
(default, or Zero-Copy). Each version can communicate with all the others,
regardless of enabled features, but the resulting performance will depend on
runtime negotiation, and compatibility of available features.

For example, a Zero-Copy subscriber will be able to receive samples from a
non Zero-Copy writer, but communication will be carried out using the default
transfer method. Similarly, a Flat-Data/Zero-Copy endpoint may communicate with a
Zero-Copy endpoint, but the different memory representations will prevent
some optimizations from being enabled.

These examples are based on the
[`flat_data_latency`](https://github.com/rticommunity/rticonnextdds-examples/tree/master/examples/connext_dds/flat_data_latency/)
example from the [rticommunity/rticonnextdds-examples](https://github.com/rticommunity/rticonnextdds-examples)
repository.

| Example | Description |
|---------|-------------|
|[camera_pub_plain.cpp](connext_nodes_cpp/src/camera/camera_pub_plain.cpp) | Publisher using plain memory representation, and default transport |
|[camera_pub_flat.cpp](connext_nodes_cpp/src/camera/camera_pub_flat.cpp) | Publisher using Flat-Data memory representation, and default transport |
|[camera_pub_flat_zc.cpp](connext_nodes_cpp/src/camera/camera_pub_flat_zc.cpp) | Publisher using Flat-Data memory representation, and Zero-Copy transport |
|[camera_pub_zc.cpp](connext_nodes_cpp/src/camera/camera_pub_zc.cpp) | Publisher using plain memory representation, and Zero-Copy transport |
|[camera_sub_plain.cpp](connext_nodes_cpp/src/camera/camera_sub_plain.cpp) | Subscriber using plain memory representation, and default transport |
|[camera_sub_flat.cpp](connext_nodes_cpp/src/camera/camera_sub_flat.cpp) | Subscriber using Flat-Data memory representation, and default transport |
|[camera_sub_flat_zc.cpp](connext_nodes_cpp/src/camera/camera_sub_flat_zc.cpp) | Subscriber using Flat-Data memory representation, and Zero-Copy transport |
|[camera_sub_zc.cpp](connext_nodes_cpp/src/camera/camera_sub_zc.cpp) | Subscriber using plain memory representation, and Zero-Copy transport |

*Example usage:*

- Slow: [Plain, Default] to [Plain, Default]

```sh
ros2 run connext_nodes_cpp camera_pub_plain

ros2 run connext_nodes_cpp camera_sub_plain
```

- Slow: [Flat-Data, Default] to [Flat-Data, Default]

```sh
ros2 run connext_nodes_cpp camera_pub_flat

ros2 run connext_nodes_cpp camera_sub_flat
```

- Fast: [Plain, Zero-Copy] to [Plain, Zero-Copy]

```sh
ros2 run connext_nodes_cpp camera_pub_zc

ros2 run connext_nodes_cpp camera_sub_zc
```

- Fast: [Flat-Data, Zero-Copy] to [Flat-Data, Zero-Copy]

```sh
ros2 run connext_nodes_cpp camera_pub_flat_zc

ros2 run connext_nodes_cpp camera_sub_flat_zc
```

### processor_chatter

[Source code](connext_nodes_cpp/src/processor)

This example uses the interfaces offered by [connext_nodes/processor.hpp](connext_node_helpers/include/connext_nodes/processor.hpp)
to implement a simple "processor" node for topic `"chatter"`. A "processor" in this context
is a type of node which consumes messages from topic an input topic,
manipulates them, and redistributes the results over an output topic.

The message processing logic is implemented in a "middleware-agnostic" fashion,
so that it may be encapsulated in a class which doesn't directly depend on neither
the ROS 2 nor the RTI Connext DDS APIs (except for their respective data bindings).
Thanks to the available node templates, two executables are built from the same
`main()`: one which runs a DDS-based version of the node (implemented by deriving
from template class `rti::ros2::DdsProcessorNode`), and one which uses
"classic" ROS 2 APIs (implemented by deriving from template class `rti::ros2::RosProcessorNode`).

| Example | Description   |
|---------|---------------|
|[processor_chatter_dds.hpp](connext_nodes_cpp/src/processor/processor_chatter_dds.hpp)|Instantiation of `rti::ros2::DdsProcessorNode<std_msgs::msg::String, std_msgs::msg::String>` for topics `"chatter"`/`"chatter/processed"`.|
|[processor_chatter_ros.hpp](connext_nodes_cpp/src/processor/processor_chatter_ros.hpp)|Instantiation of `rti::ros2::RosProcessorNode<std_msgs::msg::String, std_msgs::msg::String>` for topics `"chatter"`/`"chatter/processed"`.|
|[processor_chatter.hpp](connext_nodes_cpp/src/processor/processor_chatter.hpp)|Middleware-agnostic message processing logic.|
|[processor_chatter.cpp](connext_nodes_cpp/src/processor/processor_chatter.cpp)|`main()` entry point for the generated executables.|

*Example usage:*

```sh
# Use rtiddsspy to print processed samples
rtiddspy -printSample

# Start a talker on topic "rt/chatter"
ros2 run connext_nodes_cpp talker

# Process samples using the DDS-based version
./install/connext_nodes_cpp/bin/processor_chatter_dds
```

### talker/listener

[Source code](connext_nodes_cpp/src/chatter)

These examples mimic the `talker` and `listener` applications included in package
`demo_nodes_cpp`, but they use the RTI Connext DDS C++11 API to create DDS endpoint
that can interoperate over the ROS 2 topic `"chatter"`.

The endpoints are created reusing the type definition of `std_msgs::msg::String``
automatically converted from ROS IDL to OMG IDL by the ROS 2 build process.

| Example | Description   |
|---------|---------------|
|[talker.cpp](connext_nodes_cpp/src/chatter/talker.cpp)| Connext-based version of [demo_nodes_cpp/src/topics/talker.cpp](https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/talker.cpp)|
|[listener.cpp](connext_nodes_cpp/src/chatter/listener.cpp)| Connext-based version of [demo_nodes_cpp/src/topics/listener.cpp](https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/listener.cpp)|
|[talker_main.cpp](connext_nodes_cpp/src/standalone/talker_main.cpp)| Stand-alone version of [talker.cpp](connext_nodes_cpp/src/chatter/talker.cpp)|
|[listener_main.cpp](connext_nodes_cpp/src/standalone/listener_main.cpp)| Stand-alone version of [listener.cpp](connext_nodes_cpp/src/chatter/listener.cpp)|

*Example usage:*

```sh
# Start a DDS talker on topic "chatter"
ros2 run connext_nodes_cpp talker

# Consume data with a ROS 2 listener
ros2 run demo_nodes_cpp listener
```

## Package `connext_node_helpers`

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
messages and (soon) services, but also regular Connext IDL files.

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
    sensor_msgs/PointCloud2
  IDLS
    idl/my/custom/ns/MyType.idl@my/custom/ns
    idl/SomeTypesWithoutNamespace.idl
  ZEROCOPY)
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

# Generate type support for type `std_msgs::msg::String`.
# When using this syntax, the first argument is the "base name" of the type
# and the PACKAGE argument must always be specified to qualify the type.
# The list of generated files will be stored as `${std_msgs_String_FILES}`.
# The generated code must be included with `#include "std_msgs/msg/String.hpp"`.
find_package(std_msgs REQUIRED)
connext_generate_message_typesupport_cpp(String PACKAGE std_msgs)

# Generate type support from a typical IDL file for Connext.
# In this case, the input is the path to the input file, and the
# PACKAGE argument can be used to specify an optional "include prefix".

# Generated files will be available as `${my_custom_ns_MyType_FILES}`
# The generated code must be included with `#include "my/custom/ns/MyType.hpp"`.
connext_generate_message_typesupport_cpp(idl/my/custom/ns/MyType.idl
  PACKAGE my/custom/ns)

# Generated files will be available as `${SomeTypesWithoutNamespace_FILES}`
# The generated code must be included with `#include "SomeTypesWithoutNamespace.hpp"`.
connext_generate_message_typesupport_cpp(idl/SomeTypesWithoutNamespace.idl)

add_executable(my_app
  main.c
  ${std_msgs_String_FILES}
  ${my_custom_ns_MyType_FILES}
  ${SomeTypesWithoutNamespace_FILES})

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

## Package `connext_msgs`

Package `connext_msg` contains a collection of IDL files extracted from the
ROS 2 Rolling distribution, and modified to compile with `rtiddsgen` so that
they may be compiled into a single, ready-to-use shared library.

Only IDL files for message types are included.

The IDL files can be updated using script `copy_idls.sh`.

The script will scan a ROS 2 installation, and it will copy all IDL files
to the `./idl` directory. It will also perform some lightweight processing on
the files to remove some incompatibilities.

This package takes inspiration from repository [`rticommunity/ros-data-types`](https://github.com/rticommunity/ros-data-types),
which offers a similar library that provides easy access to ROS 2 data types to
any Connext application.

### Use DDS types in a ROS 2 application

Since `connext_msgs` is a regular ROS 2 package, you can just add it to your
`package.xml` dependencies, and load it in your `CMakeLists.txt` like any other
package and library:

- `package.xml`:

  ```xml
  <package format="3">
    <name>my_package</name>
    
    <!-- ... -->

    <depend>connext_msgs</depend>
  
    <!-- ... -->
  </package>
  ```

- `CMakeLists.txt`

  ```cmake
  cmake_minimum_required(VERSION 3.5)
  project(my_package)

  # ...

  # Load package
  find_package(connext_msgs REQUIRED)

  # ...

  # Add dependency to your targets
  ament_target_dependencies(my_target  connext_msgs)
  ament_export_dependencies(connext_msgs)

  ```

In oder to you the types, you must `#include` the appropriate file in your C++
code. The path is slightly different from the one used for standard ROS 2 messages,
in that the file name is not converted to all lowercase and "snake case" format,
instead retaining the same name as the input `.idl`.

For example, in order to use type `sensor_msgs::msg::PointCloud`:

```cpp
// typical ROS 2 include
#include "sensor_msgs/msg/point_cloud.hpp"

// DDS type include
#include "sensor_msgs/msg/PointCloud.hpp"
```

### Included packages

Types from the following packages are currently included in the library:

```txt
actionlib_msgs           nav_msgs         std_msgs
action_msgs              pcl_msgs         stereo_msgs
builtin_interfaces       pendulum_msgs    test_msgs
diagnostic_msgs          rcl_interfaces   tf2_msgs
example_interfaces       rmw_dds_common   trajectory_msgs
geometry_msgs            rosgraph_msgs    turtlesim
libstatistics_collector  sensor_msgs      unique_identifier_msgs
lifecycle_msgs           shape_msgs       visualization_msgs
map_msgs                 statistics_msgs
```

### Unsupported types

The following types are currently unsupported, typically because their IDL files
contain multiple nested `#include`'s of the same file:

```txt
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
```

## License

RTI grants Licensee a license to use, modify, compile, and create derivative
works of the Software solely in combination with RTI Connext DDS. Licensee
may redistribute copies of the Software provided that all such copies are
subject to this License. The Software is provided "as is", with no warranty
of any type, including any warranty for fitness for any purpose. RTI is
under no obligation to maintain or support the Software. RTI shall not be
liable for any incidental or consequential damages arising out of the use or
inability to use the Software. For purposes of clarity, nothing in this
License prevents Licensee from using alternate versions of DDS, provided
that Licensee may not combine or link such alternate versions of DDS with
the Software.

```text
(c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
```
