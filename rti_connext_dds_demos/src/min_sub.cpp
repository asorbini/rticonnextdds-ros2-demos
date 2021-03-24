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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// Include RTI Connext DDS "modern C++" API
#include <dds/domain/ddsdomain.hpp>
#include <dds/domain/find.hpp>

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"),
      participant_(dds::core::null)
    {
      auto opts = rclcpp::contexts::get_global_default_context()->get_init_options();
      participant_ = dds::domain::find(opts.get_domain_id());
      if (dds::core::null == participant_) {
        throw new std::runtime_error(
          "failed to look up DomainParticipant. Is the application using rmw_connextdds?");
      }
    }

  private:
    dds::domain::DomainParticipant participant_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
