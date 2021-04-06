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

#ifndef CONNEXT_NODES_CPP__PROCESSOR_CHATTER_HPP
#define CONNEXT_NODES_CPP__PROCESSOR_CHATTER_HPP

#include <string>

#define CHATTER_TOPIC_IN        "chatter"
#define CHATTER_TOPIC_OUT       "chatter/processed"

/******************************************************************************
 * Shared "processor" logic
 ******************************************************************************/
class ChatterProcessor
{
public:
  virtual void process_data(
    const std::string & msg_in, std::string & msg_out)
  {
    received_count_ += 1;
    printf("* Received data %lu: %s\n", received_count_, msg_in.c_str());
    msg_out = "Processed samples: " + std::to_string(received_count_);
  }

private:
  uint64_t received_count_{0};
};

#endif  // CONNEXT_NODES_CPP__PROCESSOR_CHATTER_HPP