// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#include "connext_nodes/visibility_control.h"

#include "rti/ros2/ping/subscriber.hpp"

#include "camera/CameraImage.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace rti::camera::plain;

namespace rti { namespace connext_nodes_cpp { namespace camera {

class CameraImageSubscriberPlain :
  public rti::ros2::ping::PingPongSubscriber<CameraImage>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  explicit CameraImageSubscriberPlain(const rclcpp::NodeOptions & options)
  : PingPongSubscriber("camera_sub_plain", options)
  {
    this->init_test();
  }

protected:
  virtual CameraImage * alloc_sample()
  {
    return &ping_msg_;
  }

  virtual void prepare_pong(CameraImage * const pong, const uint64_t ping_ts)
  {
    pong->timestamp(ping_ts);
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = ping_samples[0].data().timestamp();
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    std::ostringstream & msg)
  {
    auto & sample = ping_samples[0].data();

    msg << "[" << sample.timestamp() << "] " <<  sample.format();

    for (int i = 0; i < 4; i++) {
        msg << "0x" << 
          std::hex << std::uppercase <<
          std::setfill('0') << std::setw(2) <<
          (int) sample.data()[i] <<
          std::nouppercase << std::dec <<
          " ";
    }
  }

  CameraImage ping_msg_;
};

}  // namespace camera
}  // namespace connext_nodes_cpp
}  // namespace rti

RCLCPP_COMPONENTS_REGISTER_NODE(rti::connext_nodes_cpp::camera::CameraImageSubscriberPlain)
