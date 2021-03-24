// Copyright 2021 Real-Time Innovations, Inc.
// Copyright 2016 Open Source Robotics Foundation, Inc.
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

// Include RTI Connext DDS "modern C++" API
#include <dds/domain/ddsdomain.hpp>
#include <dds/domain/find.hpp>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"),
    count_(0),
    participant_(dds::core::null)
  {
    auto opts = rclcpp::contexts::get_global_default_context()->get_init_options();
    participant_ = dds::domain::find(opts.get_domain_id());
    if (dds::core::null == participant_) {
      throw new std::runtime_error(
              "failed to look up DomainParticipant. Is the application using rmw_connextdds?");
    }

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {}
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  dds::domain::DomainParticipant participant_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
