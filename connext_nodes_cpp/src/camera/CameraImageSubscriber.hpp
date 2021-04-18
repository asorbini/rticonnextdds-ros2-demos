// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImageSubscriber_hpp_
#define CameraImageSubscriber_hpp_

#include "PingPongTester.hpp"

#include "CameraImageManipulator.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImageSubscriber : public PingPongTester<T, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImageSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongTester<T, A>(name, options, false /* pong */)
  {
    RCLCPP_INFO(this->get_logger(),
      "camera subscriber ready, waiting for publisher...");
  }

protected:
  // Overload `on_data_available()` to propagate ping sample to the pong topic.
  virtual void on_data_available()
  {
    // Read pong sample and calculate latency
    auto ping_samples = this->reader_.take();

    const bool has_data = ping_samples.length() > 0;
    if (has_data && ping_samples[0].info().valid()) {
      auto sample = ping_samples[0];
      auto ping_ts = M::timestamp(sample.data());

      if (ping_ts == 0) {
        RCLCPP_INFO(this->get_logger(), "received end ping, exiting");
        this->shutdown();
        return;
      }

      if (this->test_options_.display_received) {
        std::ostringstream msg;
        msg << "[CameraImage] " <<
          "[" << M::timestamp(sample.data()) << "] " << 
          "" << M::format(sample.data());

        for (int i = 0; i < 4; i++) {
            msg << "0x" << 
              std::hex << std::uppercase <<
              std::setfill('0') << std::setw(2) <<
              (int) M::data(sample.data(), i) <<
              std::nouppercase << std::dec <<
              " ";
        }

        RCLCPP_INFO(this->get_logger(), msg.str().c_str());
      }

      // Send back the timestamp to the writer.
      auto pong = this->alloc_sample();
      M::timestamp(*pong, ping_ts);
      this->writer_.write(*pong);
    } else if (has_data && !ping_samples[0].info().valid()) {
      RCLCPP_ERROR(this->get_logger(), "lost pong writer before end of test");
      this->shutdown();
    }
  }
};

template<typename T>
using BaseCameraImageSubscriberPlain = CameraImageSubscriber<T, CameraImageManipulatorPlain<T>, DataAllocatorDynamic<T>>;

template<typename T>
using BaseCameraImageSubscriberFlat = CameraImageSubscriber<T, CameraImageManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImageSubscriberFlatZc = CameraImageSubscriber<T, CameraImageManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImageSubscriberZc = CameraImageSubscriber<T, CameraImageManipulatorPlain<T>, DataAllocatorLoan<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageSubscriber_hpp_
