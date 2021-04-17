// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImageTestOptions_hpp_
#define CameraImageTestOptions_hpp_

#include <dds/dds.hpp>

#include "rclcpp/rclcpp.hpp"

#include "connext_nodes/visibility_control.h"

namespace rti { namespace connext_nodes_cpp {

struct CameraImageTestOptions
{
  int32_t domain_id;
  uint64_t max_samples;
  uint64_t max_execution_time;
  uint64_t ignored_initial_samples;
  dds::core::Duration wait_timeout;
  uint64_t print_interval;
  const char * topic_name_ping;
  const char * topic_name_pong;
  const char * type_name;
  const char * qos_profile;
  bool display_samples_recvd;

  CONNEXT_NODES_CPP_PUBLIC
  static CameraImageTestOptions defaults() {
    return {
      0,  // domain_id
      100,  // max_samples
      0,  // max_execution_time
      3,  // ignored_initial_samples
      dds::core::Duration(10),  // wait_timeout
      1000000,  // print_interval (1s)
      "CameraImagePing",  // topic_name_ping
      "CameraImagePong",  // topic_name_pong
      "CameraImage",  // type_name
      "BuiltinQosLibExp::Generic.StrictReliable.LargeData",  // qos_profile
      true  // display_samples_recvd
    };
  }
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageTestOptions_hpp_
