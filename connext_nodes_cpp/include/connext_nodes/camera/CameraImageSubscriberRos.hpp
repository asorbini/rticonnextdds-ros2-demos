// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImageSubscriber_hpp_
#define CameraImageSubscriber_hpp_

#include <rti/ros2/ping/subscriber_ros.hpp>

#include "connext_node_msgs/msg/camera_common.hpp"
#include "connext_node_msgs/msg/resolution.hpp"
#include "connext_node_msgs/msg/camera_image.hpp"

namespace rti { namespace connext_nodes_cpp { namespace camera {

class CameraImageSubscriberRos : public rti::ros2::ping::RosPingPongSubscriber<connext_node_msgs::msg::CameraImage>
{
public:
  CameraImageSubscriberRos(const rclcpp::NodeOptions & options)
  : rti::ros2::ping::RosPingPongSubscriber<connext_node_msgs::msg::CameraImage>(
      "camera_sub_ros", options)
  {
    this->init_test();
  }

protected:
  virtual void prepare_pong(
    connext_node_msgs::msg::CameraImage & pong, const uint64_t ping_ts)
  {
    pong.timestamp = ping_ts;
    if (pong.data.size() < connext_node_msgs::msg::CameraCommon::IMAGE_SIZE)
    {
      pong.data.resize(connext_node_msgs::msg::CameraCommon::IMAGE_SIZE);
      std::fill(pong.data.begin(), pong.data.end(), 0);
    }
  }

  virtual void process_ping(
    connext_node_msgs::msg::CameraImage & ping, uint64_t & ping_timestamp)
  {
    ping_timestamp = ping.timestamp;
  }

  virtual void dump_ping(
    connext_node_msgs::msg::CameraImage & ping, std::ostringstream & msg)
  {
    msg << "[" << ping.timestamp << "] " <<  ping.format;

    for (int i = 0; i < 4; i++) {
        msg << "0x" << 
          std::hex << std::uppercase <<
          std::setfill('0') << std::setw(2) <<
          (int) ping.data[i] <<
          std::nouppercase << std::dec <<
          " ";
    }
  }
};

}  // namespace camera
}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageSubscriber_hpp_
