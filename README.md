# Example ROS2 Applications using RTI Connext DDS

This repository contains a collection of  ROS 2 applications built using the
RTI Connext DDS APIs.

- [Build repository](#build-repository)
- [Run examples](#run-examples)
- [Included examples](#included-examples)
  - [camera](#camera)
  - [processor_chatter](#processor_chatter)
  - [talker/listener](#talkerlistener)
- [Other useful resources](#other-useful-resources)
- [License](#license)

## Build repository

The included applications require RTI Connext DDS 6.x, and they can only be
run using [`rmw_connextdds`](https://github.com/ros2/rmw_connextdds) (with a
few exceptions).

Once RTI Connext DDS 6.x is installed, you can clone an build all required
packages in a single workspace using one of the provided `.repos` files:

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

# Clone all required repositories using the included repos file
wget https://raw.githubusercontent.com/asorbini/rticonnextdds-ros2-demos/master/foxy.repos
vcs import src < foxy.repos

# If you have multiple target libraries installed you might need to select the
# desired one with `--cmake-args -DCONNEXTDDS_ARCH=<ARCH>`.
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

## Included examples

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

## Other useful resources

- [`rticonnextdds-ros2-helpers`](https://github.com/asorbini/rticonnextdds-ros2-helpers)
  - Collection of utilities to built ROS 2 applications with RTI Connext DDS.
    Used extensively by the demos in this repository.
- [`rticonnextdds-ros2-msgs`](https://github.com/asorbini/rticonnextdds-ros2-msgs)
  - Helper library containing C++11 message type supports generated with
   `rtiddsgen` for almost every type include in ROS 2.

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
