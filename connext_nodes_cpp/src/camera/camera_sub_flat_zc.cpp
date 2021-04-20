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

#include "camera/CameraImageFlatZc.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace rti::camera::flat_zc;

namespace rti { namespace connext_nodes_cpp { namespace camera {

class CameraImageSubscriberFlatZc :
  public rti::ros2::ping::PingPongSubscriber<CameraImage>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  explicit CameraImageSubscriberFlatZc(const rclcpp::NodeOptions & options)
  : PingPongSubscriber("camera_sub_flat_zc", options)
  {
    this->init_test();
  }

protected:
  virtual CameraImage * alloc_sample()
  {
    return writer_.extensions().get_loan();
  }

  virtual void prepare_pong(CameraImage * const pong, const uint64_t ping_ts)
  {
    pong->root().timestamp(ping_ts);
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = ping_samples[0].data().root().timestamp();
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    std::ostringstream & msg)
  {
    auto sample = ping_samples[0].data().root();

    msg << "[" << sample.timestamp() << "] " <<  sample.format();

    for (int i = 0; i < 4; i++) {
        msg << "0x" << 
          std::hex << std::uppercase <<
          std::setfill('0') << std::setw(2) <<
          (int) sample.data().get_elements()[i] <<
          std::nouppercase << std::dec <<
          " ";
    }
  }
};

}  // namespace camera
}  // namespace connext_nodes_cpp
}  // namespace rti

RCLCPP_COMPONENTS_REGISTER_NODE(rti::connext_nodes_cpp::camera::CameraImageSubscriberFlatZc)
