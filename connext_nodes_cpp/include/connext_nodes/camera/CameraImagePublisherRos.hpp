// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImagePublisherRos_hpp_
#define CameraImagePublisherRos_hpp_

#include <rti/ros2/ping/ros_publisher.hpp>

#include "connext_node_msgs/msg/camera_common.hpp"
#include "connext_node_msgs/msg/resolution.hpp"
#include "connext_node_msgs/msg/camera_image.hpp"

namespace rti { namespace connext_nodes_cpp { namespace camera {

class CameraImagePublisherRos : public rti::ros2::ping::RosPingPongPublisher<connext_node_msgs::msg::CameraImage>
{
public:
  CameraImagePublisherRos(const rclcpp::NodeOptions & options)
  : rti::ros2::ping::RosPingPongPublisher<connext_node_msgs::msg::CameraImage>(
      "camera_pub_ros", options)
  {
    this->init_test();
  }

protected:
  virtual void prepare_ping(
    connext_node_msgs::msg::CameraImage & ping, const bool final)
  {
    using namespace connext_node_msgs::msg;

    if (final) {
      ping.timestamp = 0;
      return;
    }
    
    ping.format = CameraCommon::FORMAT_RGB;
    ping.resolution.height = CameraCommon::CAMERA_HEIGHT_DEFAULT;
    ping.resolution.width = CameraCommon::CAMERA_WIDTH_DEFAULT;

    if (ping.data.size() < connext_node_msgs::msg::CameraCommon::IMAGE_SIZE)
    {
      ping.data.resize(connext_node_msgs::msg::CameraCommon::IMAGE_SIZE);
      std::fill(ping.data.begin(), ping.data.end(), 0);
    }
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + this->count_) % 124;
      ping.data[i] = image_value;
    }
    
    // Update timestamp
    ping.timestamp = this->ts_now();
  }

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    connext_node_msgs::msg::CameraImage & pong,
    uint64_t & pong_timestamp)
  {
    pong_timestamp = pong.timestamp;
  }
};

}  // namespace camera
}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImagePublisherRos_hpp_
