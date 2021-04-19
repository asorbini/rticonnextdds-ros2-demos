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

#include "connext_nodes/camera/CameraImageSubscriber.hpp"

#include "camera/CameraImageFlat.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace rti { namespace connext_nodes_cpp { namespace camera {

class CameraImageSubscriberFlat :
  public BaseCameraImageSubscriberFlat<rti::camera::flat::CameraImage>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  explicit CameraImageSubscriberFlat(const rclcpp::NodeOptions & options)
  : CameraImageSubscriber("camera_sub_flat", options)
  {}
};

}  // namespace camera
}  // namespace connext_nodes_cpp
}  // namespace rti

RCLCPP_COMPONENTS_REGISTER_NODE(rti::connext_nodes_cpp::camera::CameraImageSubscriberFlat)
