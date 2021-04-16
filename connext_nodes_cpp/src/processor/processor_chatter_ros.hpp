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

#ifndef PROCESSOR_CHATTER_ROS_HPP
#define PROCESSOR_CHATTER_ROS_HPP

#include "connext_node_helpers/processor.hpp"

#include "processor_chatter.hpp"

#include "std_msgs/msg/string.hpp"

namespace rti { namespace connext_nodes_cpp {

/******************************************************************************
 * Traditional ROS 2 Node
 ******************************************************************************/
class Ros2ChatterProcessorNode :
  public rti::ros2::Ros2ProcessorNode<std_msgs::msg::String, std_msgs::msg::String>
{
public:
  Ros2ChatterProcessorNode()
  : Ros2ProcessorNode(
      "chatter_processor",
      CHATTER_TOPIC_IN,
      CHATTER_TOPIC_OUT)
  {
    output_msg_ = std::make_unique<std_msgs::msg::String>();
    this->create_entities();
  }

protected:
  virtual std_msgs::msg::String & result_message()
  {
    return *output_msg_;
  }

  virtual void create_output()
  {
    rclcpp::QoS qos(rclcpp::KeepLast(this->history_depth_));
    pub_ = this->create_publisher<std_msgs::msg::String>(this->topic_out_, qos);
  }

  virtual void create_input()
  {
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        this->on_data(*msg);
      };


    sub_ = this->create_subscription<std_msgs::msg::String>(
      this->topic_in_, this->history_depth_, callback);
  }

  virtual void process_message(
    const std_msgs::msg::String & msg_in, std_msgs::msg::String & msg_out)
  {
    processor_.process_data(msg_in.data, msg_out.data);
  }

  std::unique_ptr<std_msgs::msg::String> output_msg_;
  ChatterProcessor processor_;
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CONNEXT_NODES_CPP__PROCESSOR_CHATTER_ROS_HPP