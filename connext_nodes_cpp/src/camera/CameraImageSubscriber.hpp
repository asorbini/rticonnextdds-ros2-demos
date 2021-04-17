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

#include "CameraImageTester.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImageSubscriber : public CameraImageTester<T, M, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImageSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options,
    const CameraImageTestOptions & test_options = CameraImageTestOptions::defaults())
  : CameraImageTester<T, M, A>(name, options, test_options)
  {
    this->init_test(
      // pong writer configuration
      this->test_options_.topic_name_pong,
      this->test_options_.type_name,
      this->test_options_.qos_profile,
      // ping reader configuration
      this->test_options_.topic_name_ping,
      this->test_options_.type_name,
      this->test_options_.qos_profile);
    
    RCLCPP_INFO(this->get_logger(),
      "camera subscriber ready, waiting for publisher...");
  }

protected:
  virtual void on_test_state_changed(
    const size_t pub_match_count,
    const size_t sub_match_count,
    const bool was_active,
    const bool is_ready,
    bool & is_active)
  {
    if (is_ready) {
      if (!was_active) {
        RCLCPP_INFO(this->get_logger(), "all endpoints matched, beginning test");
        is_active = true;
      }
    } else {
      if (!was_active) {
        RCLCPP_INFO(this->get_logger(),
          "match event detected: ping_reader=%lu, pong_writer=%lu",
          pub_match_count, sub_match_count);
      } else {
        RCLCPP_ERROR(this->get_logger(), "lost matches, shutting down");
        is_active = false;
      }
    }
  }

  virtual void on_data()
  {
    // Read pong sample and calculate latency
    auto ping_samples = this->reader_.take();

    const bool has_data = ping_samples.length() > 0;
    if (has_data && ping_samples[0].info().valid()) {
      auto sample = ping_samples[0];
      if (!this->test_active_) {
        RCLCPP_WARN(this->get_logger(),
          "ping received while test not yet active");
      }

      auto ping_ts = M::timestamp(sample.data());

      if (ping_ts == 0) {
        RCLCPP_INFO(this->get_logger(), "received end ping, exiting");
        this->shutdown();
        return;
      }

      if (this->test_options_.display_samples_recvd) {
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

      auto pong = A::alloc(this->writer_, this->cached_sample_);
      M::timestamp(*pong, ping_ts);
      this->writer_.write(*pong);
    } else if (has_data && !ping_samples[0].info().valid()) {
      RCLCPP_ERROR(this->get_logger(), "lost pong writer before end of test");
      this->shutdown();
    }
  }
};

template<typename T>
using BaseCameraImageSubscriberPlain = CameraImageSubscriber<T, CameraImageManipulatorPlain<T>, CameraImageAllocatorDynamic<T>>;

template<typename T>
using BaseCameraImageSubscriberFlat = CameraImageSubscriber<T, CameraImageManipulatorFlat<T>, CameraImageAllocatorWriter<T>>;

template<typename T>
using BaseCameraImageSubscriberFlatZc = CameraImageSubscriber<T, CameraImageManipulatorFlat<T>, CameraImageAllocatorWriter<T>>;

template<typename T>
using BaseCameraImageSubscriberZc = CameraImageSubscriber<T, CameraImageManipulatorPlain<T>, CameraImageAllocatorWriter<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageSubscriber_hpp_
