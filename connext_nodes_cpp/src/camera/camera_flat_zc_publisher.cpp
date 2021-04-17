// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#include "CameraImagePublisher.hpp"

#include "camera/CameraImageFlatZc.hpp"

namespace rti { namespace connext_nodes_cpp {

class CameraImagePublisherFlatZc :
  public BaseCameraImagePublisherFlatZc<camera::flat_zc::CameraImage>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  explicit CameraImagePublisherFlatZc(const rclcpp::NodeOptions & options)
  : CameraImagePublisher(options)
  {}
};

}  // namespace connext_nodes_cpp
}  // namespace rti

RCLCPP_COMPONENTS_REGISTER_NODE(rti::connext_nodes_cpp::CameraImagePublisherFlatZc)
