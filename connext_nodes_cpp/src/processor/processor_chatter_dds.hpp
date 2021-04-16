// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef PROCESSOR_CHATTER_DDS_HPP
#define PROCESSOR_CHATTER_DDS_HPP

#include "connext_node_helpers/processor.hpp"

#include "processor_chatter.hpp"

#include "std_msgs/msg/String.hpp"

namespace rti { namespace connext_nodes_cpp {

/******************************************************************************
 * DDS Node interface
 ******************************************************************************/
class DdsChatterProcessorNode :
  public rti::ros2::DdsProcessorNode<std_msgs::msg::String, std_msgs::msg::String>
{
public:
  DdsChatterProcessorNode()
  : DdsProcessorNode<std_msgs::msg::String, std_msgs::msg::String>(
      "chatter_processor",
      "rt/" CHATTER_TOPIC_IN,
      "std_msgs::msg::dds_::String_",
      "rt/" CHATTER_TOPIC_OUT,
      "std_msgs::msg::dds_::String_")
  {
    this->create_entities();
  }

protected:
  virtual std_msgs::msg::String & result_message()
  {
    return output_msg_;
  }

  virtual void process_message(
    const std_msgs::msg::String & msg_in, std_msgs::msg::String & msg_out)
  {
    processor_.process_data(msg_in.data(), msg_out.data());
  }

  std_msgs::msg::String output_msg_;
  ChatterProcessor processor_;
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // PROCESSOR_CHATTER_DDS_HPP
