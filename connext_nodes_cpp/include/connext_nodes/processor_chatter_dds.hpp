// Copyright 2021 Real-Time Innovations, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONNEXT_NODES_CPP__PROCESSOR_CHATTER_DDS_HPP
#define CONNEXT_NODES_CPP__PROCESSOR_CHATTER_DDS_HPP

#include "connext_nodes/processor.hpp"
#include "connext_nodes/processor_chatter.hpp"

#include "String.hpp"

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

#endif  // CONNEXT_NODES_CPP__PROCESSOR_CHATTER_DDS_HPP
