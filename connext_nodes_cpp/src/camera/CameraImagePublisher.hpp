// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImagePublisher_hpp_
#define CameraImagePublisher_hpp_

#include "CameraImageTester.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImagePublisher : public CameraImageTester<T, M, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImagePublisher(
    const char * const name,
    const rclcpp::NodeOptions & options,
    const CameraImageTestOptions & test_options = CameraImageTestOptions::defaults())
  : CameraImageTester<T, M, A>(name, options, test_options)
  {
    this->init_test(
      // ping writer configuration
      this->test_options_.topic_name_ping,
      this->test_options_.type_name,
      this->test_options_.qos_profile,
      // pong reader configuration
      this->test_options_.topic_name_pong,
      this->test_options_.type_name,
      this->test_options_.qos_profile);
    
    RCLCPP_INFO(this->get_logger(),
      "camera publisher ready, waiting for subscriber...");
  }

protected:
  virtual void print_latency()
  {
    std::ostringstream msg;

    if (count_ > 0) {
      const uint64_t avg_us = total_latency_ / (count_ * 2);
      const double avg_ms = static_cast<double>(avg_us) / 1000.0;
      msg << "Avg-Latency[" << count_ << "/" <<
        this->test_options_.max_samples << "]: " <<
        std::fixed << std::setprecision(3) << avg_ms << " ms";
    } else {
      msg << "Avg-Latency: no samples yet";
    }

    RCLCPP_INFO(this->get_logger(), msg.str().c_str());
  }

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
        test_complete_ = false;
        count_ignored_ = 0;
        count_ = 0;
        start_ts_ = this->participant_->current_time().to_microsecs();
        ping();
      }
    } else {
      if (!was_active) {
        RCLCPP_INFO(this->get_logger(),
          "match event detected: pong_reader=%lu, ping_writer=%lu",
          pub_match_count, sub_match_count);
      } else {
        is_active = false;
        if (test_complete_) {
          RCLCPP_INFO(this->get_logger(), "test completed, shutting down");
        } else {
          RCLCPP_ERROR(this->get_logger(), "lost matches, shutting down");
        }
      }
    }
  }

  virtual void ping()
  {
    ping_ts_ = this->participant_->current_time().to_microsecs();

    auto ping = A::alloc(this->writer_, this->cached_sample_);

    if ((this->test_options_.max_samples > 0 && count_ >=
            this->test_options_.max_samples) ||
      (this->test_options_.max_execution_time > 0 && ping_ts_ >
        this->test_options_.max_execution_time - start_ts_))
    {
      RCLCPP_INFO(this->get_logger(), "all samples published or max time expired: %lu/%lu",
        count_, this->test_options_.max_samples);

      M::timestamp(*ping, 0);
      this->writer_->write(*ping);

      print_latency();

      test_complete_ = true;
      return;
    }

    // Populate the sample
    M::format(*ping, camera::common::Format::RGB);
    M::resolution_height(*ping, camera::common::CAMERA_HEIGHT_DEFAULT);
    M::resolution_width(*ping, camera::common::CAMERA_WIDTH_DEFAULT);

    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + count_) % 124;
      M::data(*ping, i, image_value);
    }

    // Write the ping sample:
    M::timestamp(*ping, ping_ts_);
    this->writer_.write(*ping);
  }

  virtual void on_data()
  {
    // Read pong sample and calculate latency. Always peform a take so the
    // listener is not called repeatedly.
    auto pong_samples = this->reader_.take();
    const bool has_data = pong_samples.length() > 0;
    if (has_data && pong_samples[0].info().valid()) {
      if (!this->test_active_) {
        RCLCPP_ERROR(this->get_logger(), "pong received while test inactive");
        this->shutdown();
        return;
      }

      auto pong_ts = M::timestamp(pong_samples[0].data());
      uint64_t receive_ts = this->participant_->current_time().to_microsecs();
      // this test will not cope well with a drifting clock
      assert(receive_ts >= pong_ts);
      uint64_t latency = (receive_ts - pong_ts) / 2;

      // we ignore the first few samples to let things ramp up to steady state
      const bool ignored =
        this->test_options_.ignored_initial_samples > count_ignored_;
      if (!ignored) {
        count_ += 1;
        total_latency_ += latency;

        if (last_print_ == 0 ||
          receive_ts - this->test_options_.print_interval > last_print_) {
          print_latency();
          last_print_ = receive_ts;
        }
      } else {
        count_ignored_ += 1;
      }

      ping();
    } else if (has_data && !pong_samples[0].info().valid()) {
      if (!test_complete_) {
        RCLCPP_ERROR(this->get_logger(), "lost pong writer before end of test");
        this->shutdown();
      }
    } else if (!has_data) {
      RCLCPP_ERROR(this->get_logger(), "woke up without data");
      this->shutdown();
    }
  }

  bool test_complete_{false};
  uint64_t count_ = 0;
  uint64_t count_ignored_ = 0;
  uint64_t total_latency_ = 0;
  uint64_t last_print_ = 0;
  uint64_t start_ts_ = 0;
  uint64_t ping_ts_ = 0;
};

template<typename T>
using BaseCameraImagePublisherPlain = CameraImagePublisher<T, CameraImageManipulatorPlain<T>, CameraImageAllocatorDynamic<T>>;

template<typename T>
using BaseCameraImagePublisherFlat = CameraImagePublisher<T, CameraImageManipulatorFlat<T>, CameraImageAllocatorWriter<T>>;

template<typename T>
using BaseCameraImagePublisherFlatZc = CameraImagePublisher<T, CameraImageManipulatorFlat<T>, CameraImageAllocatorWriter<T>>;

template<typename T>
using BaseCameraImagePublisherZc = CameraImagePublisher<T, CameraImageManipulatorPlain<T>, CameraImageAllocatorWriter<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImagePublisher_hpp_
