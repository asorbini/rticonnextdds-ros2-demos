// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#if USE_DDS
#include "processor_chatter_dds.hpp"
#define NODE_CLASS      rti::connext_nodes_cpp::DdsChatterProcessorNode
#else
#include "processor_chatter_ros.hpp"
#define NODE_CLASS      rti::connext_nodes_cpp::Ros2ChatterProcessorNode
#endif  // USE_DDS

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NODE_CLASS>());
  rclcpp::shutdown();
  return 0;
}
