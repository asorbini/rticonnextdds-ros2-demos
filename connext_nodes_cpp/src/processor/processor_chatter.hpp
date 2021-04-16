// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef PROCESSOR_CHATTER_HPP
#define PROCESSOR_CHATTER_HPP

#include <string>

#define CHATTER_TOPIC_IN        "chatter"
#define CHATTER_TOPIC_OUT       "chatter/processed"

namespace rti { namespace connext_nodes_cpp {

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

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // PROCESSOR_CHATTER_HPP