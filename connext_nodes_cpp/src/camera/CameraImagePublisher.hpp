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

#include "PingPongTester.hpp"

#include "CameraImageManipulator.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImagePublisher : public PingPongTester<T, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImagePublisher(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongTester<T, A>(name, options, true /* ping */)
  {
    // Start timer to periodically check exit conditions
    this->start_exit_timer();
    
    RCLCPP_INFO(this->get_logger(),
      "camera publisher ready, waiting for subscriber...");
  }

protected:
  // Overload `test_start()` to initialize test state and send an initial ping.
  virtual void test_start()
  {
    PingPongTester<T, A>::test_start();
    ping();
  }

  // Overload `test_stop()` to print the final computed latency
  virtual void test_stop()
  {
    if (this->test_complete_) {
      print_latency(true /* final */);
    }
    PingPongTester<T, A>::test_stop();
  }

  // Once the test is complete, we send a final ping with timestamp=0 to notify
  // that reader that the test is done. Once the reader is detected as offline,
  // the writer will also exit.
  virtual void test_complete()
  {
    PingPongTester<T, A>::test_complete();

    auto sample = this->alloc_sample();
    M::timestamp(*sample, 0);
    this->writer_->write(*sample);
  }

  // Compute a running average of the latency measurements and log it to stdout.
  virtual void print_latency(const bool final_log = false)
  {
    std::ostringstream msg;

    if (this->count_ > 0) {
      const uint64_t avg_us = total_latency_ / (this->count_ * 2);
      const double avg_ms = static_cast<double>(avg_us) / 1000.0;
      msg << "Avg-Latency [" << this->count_ << "/" <<
        this->test_options_.max_samples << "] " <<
        std::fixed << std::setprecision(3) << avg_ms << " ms";
      if (final_log) {
        msg << " [FINAL]";
      }
    } else {
      msg << "Avg-Latency: no samples yet";
    }

    RCLCPP_INFO(this->get_logger(), msg.str().c_str());
  }

  // Send a ping message unless the test has already completed.
  virtual void ping()
  {
    if (this->check_test_complete())
    {
      return;
    }

    // Allocate a sample and prepare it for publishing
    auto sample = this->alloc_sample();

    M::format(*sample, camera::common::Format::RGB);
    M::resolution_height(*sample, camera::common::CAMERA_HEIGHT_DEFAULT);
    M::resolution_width(*sample, camera::common::CAMERA_WIDTH_DEFAULT);

    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + this->count_) % 124;
      M::data(*sample, i, image_value);
    }

    // Update timestamp
    M::timestamp(*sample, this->participant_->current_time().to_microsecs());

    // Write the sample out
    this->writer_.write(*sample);
  }

  // Overload on_data_available() to process the pong sample and compute the
  // round-trip latency. We store the value halved to compute a running average
  // of one-way latency.
  virtual void on_data_available()
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
        static_cast<uint64_t>(this->test_options_.ignored_initial_samples) > this->count_ignored_;
      if (!ignored) {
        this->count_ += 1;
        total_latency_ += latency;

        if (last_print_ == 0 ||
          receive_ts - static_cast<uint64_t>(this->test_options_.print_interval) > last_print_) {
          print_latency();
          last_print_ = receive_ts;
        }
      } else {
        this->count_ignored_ += 1;
      }

      ping();
    } else if (has_data && !pong_samples[0].info().valid()) {
      // A "invalid" sample generally indicates a state transition from an
      // unmatched/not-alive remote writer. Check this is expected (e.g if the
      // test has already been completed), or terminate the test early otherwise.
      if (!this->check_test_complete()) {
        RCLCPP_ERROR(this->get_logger(), "lost pong writer before end of test");
        this->test_stop();
      }
    } else if (!has_data) {
      // This should never happen, but just in case, print an error and exit.
      RCLCPP_ERROR(this->get_logger(), "woke up without data");
      this->test_stop();
    }
  }

  uint64_t total_latency_ = 0;
  uint64_t last_print_ = 0;
  uint64_t ping_ts_ = 0;
};

template<typename T>
using BaseCameraImagePublisherPlain = CameraImagePublisher<T, CameraImageManipulatorPlain<T>, DataAllocatorDynamic<T>>;

template<typename T>
using BaseCameraImagePublisherFlat = CameraImagePublisher<T, CameraImageManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImagePublisherFlatZc = CameraImagePublisher<T, CameraImageManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImagePublisherZc = CameraImagePublisher<T, CameraImageManipulatorPlain<T>, DataAllocatorLoan<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImagePublisher_hpp_
