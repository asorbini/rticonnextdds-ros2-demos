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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include <dds/dds.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "connext_nodes/visibility_control.h"

#include "CameraImageManipulator.hpp"

namespace rti { namespace connext_nodes_cpp {

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
// Use Connext's Modern C++ API to create a DataWriter to publish messages.
template<typename CameraImageType, typename Manipulator>
class CameraImagePublisher : public rclcpp::Node
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImagePublisher(
    const rclcpp::NodeOptions & options,
    const uint64_t max_samples = 1000000,
    const uint64_t max_execution_time = 0,
    const dds::core::Duration wait_timeout = dds::core::Duration(10),
    const uint64_t print_interval = 4000000,
    const char * const topic_name_ping = "CameraImagePing",
    const char * const topic_name_pong = "CameraImagePong",
    const char * const type_name = "CameraImage")
  : Node("camera_plain", options),
    max_samples_(max_samples),
    max_execution_time_(max_execution_time),
    wait_timeout_(wait_timeout),
    print_interval_(print_interval)
  {
    using namespace std::chrono_literals;
    using namespace dds::core;

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // The DomainParticipant is created on domain 0 by default
    participant_ = dds::domain::find(0);
    assert(null != participant_);
    // Create the ping DataWriter
    dds::pub::Publisher publisher(participant_);
    dds::topic::Topic<CameraImageType> topic_ping(participant_,
      topic_name_ping, type_name);
    auto writer_qos = publisher.default_datawriter_qos(); 
    writer_qos << policy::Reliability::Reliable();
    writer_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
    writer_ = dds::pub::DataWriter<CameraImageType>(
      publisher, topic_ping, writer_qos);

    // Create the pong DataReader
    dds::sub::Subscriber subscriber(participant_);
    dds::topic::Topic<CameraImageType> topic_pong(participant_,
      topic_name_pong, type_name);
    auto reader_qos = subscriber.default_datareader_qos(); 
    reader_qos << policy::Reliability::Reliable();
    reader_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
    reader_ = dds::sub::DataReader<CameraImageType>(
      subscriber, topic_pong, reader_qos);
    
    // Create a ReadCondition for any data on the pong reader, and attach it to
    // a Waitset
    read_condition_ = dds::sub::cond::ReadCondition(
      reader_, dds::sub::status::DataState::any());

    waitset_ += read_condition_;

    ping_sample_ = Manipulator::prealloc(writer_);

    auto ping_pong_cb = [this]() {
      ping_pong();
    };

    // Use a fast timer to schedule the ping_pong() function as quickly as possible
    // The function itself will block the calling timer to wait for the pong.
    timer_ = this->create_wall_timer(1ns, ping_pong_cb);
  }

  // Wait for discovery
  void wait_for_reader(bool match = true)
  {
    if (match) {
      RCLCPP_INFO(this->get_logger(),
        "Waiting for the subscriber application to match ping writer...");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Waiting for the subscriber application to unmatch ping writer...");
    }

    while (((match && dds::pub::matched_subscriptions(writer_).empty())
            || (!match && !dds::pub::matched_subscriptions(writer_).empty())) &&
            rclcpp::ok()) {
      rti::util::sleep(
        dds::core::Duration::from_millisecs(
          camera::common::CAMERA_TIMEOUT_DEFAULT));
    }

    if (dds::pub::matched_subscriptions(writer_).empty()) {
      RCLCPP_ERROR(this->get_logger(), "shutdown before ping writer match");
      throw std::runtime_error("shutdown before ping writer match");
    }

    RCLCPP_INFO(this->get_logger(), "Ping writer matched.");
  }

  void wait_for_writer()
  {
    RCLCPP_INFO(this->get_logger(),
      "Waiting for the subscriber application to match pong reader...");

    while (dds::sub::matched_publications(reader_).empty() && rclcpp::ok()) {
      rti::util::sleep(
        dds::core::Duration::from_millisecs(
          camera::common::CAMERA_TIMEOUT_DEFAULT));
    }

    if (dds::sub::matched_publications(reader_).empty()) {
      RCLCPP_ERROR(this->get_logger(), "shutdown before pong reader match");
      throw std::runtime_error("shutdown before pong reader match");
    }

    RCLCPP_INFO(this->get_logger(), "Pong reader matched.");
  }

  void print_latency(int total_latency, int count)
  {
    std::ostringstream msg;

    if (count > 0) {
      msg << "Avg-Latency: " << total_latency / (count * 2) << "ms";
    } else {
      msg << "Avg-Latency: no samples yet";
    }

    RCLCPP_INFO(this->get_logger(), msg.str().c_str());
  }

  uint64_t
  publish_begin()
  {
    wait_for_reader();

    wait_for_writer();

    RCLCPP_INFO(this->get_logger(),
      "Discovery complete! Starting publishing: max=%lu", max_samples_);

    return participant_->current_time().to_microsecs();
  }

  void
  publish_complete()
  {
    RCLCPP_INFO(this->get_logger(), "All samples published: %lu/%lu",
      count_, max_samples_);

    Manipulator::timestamp(*ping_sample_, 0);
    writer_->write(*ping_sample_);

    // Cancel scheduling timer
    timer_->cancel();

    print_latency(total_latency_, count_);

    RCLCPP_INFO(this->get_logger(), "press CTRL+C to exit...");
  }

  bool
  is_publish_complete()
  {
    assert(send_ts_ > 0);
    assert(start_ts_ > 0);
  
    return ((max_samples_ > 0 && count_ >= max_samples_) ||
      (max_execution_time_ > 0 && send_ts_ > max_execution_time_ - start_ts_));
  }

  void
  ping_pong()
  {
    if (count_ == 0) {
      start_ts_ = publish_begin();
    }
    send_ts_ = participant_->current_time().to_microsecs();

    if (is_publish_complete()) {
      publish_complete();
      return;
    }

    auto ping_sample = Manipulator::alloc(writer_, ping_sample_);

    // Populate the sample
    Manipulator::populate(*ping_sample, count_);

    // Write the ping sample:
    Manipulator::timestamp(*ping_sample, send_ts_);
    writer_.write(*ping_sample);
    if (last_print_interval_ == 0) {
      last_print_interval_ = send_ts_;
    }
    
    // Wait for the pong
    auto conditions = waitset_.wait(wait_timeout_);
    if (conditions.empty()) {
      RCLCPP_ERROR(this->get_logger(), "timeout while waiting for pong");
      throw std::runtime_error("timeout while waiting for pong");
    }

    // Read pong sample and calculate latency
    auto pong_samples = reader_.take();
    const bool has_data = pong_samples.length() > 0;
    if (has_data && pong_samples[0].info().valid()) {
      count_ += 1;
      auto pong_ts = Manipulator::timestamp(pong_samples[0].data());
      uint64_t receive_ts = participant_->current_time().to_microsecs();
      // this test will not cope well with a drifting clock
      assert(receive_ts >= pong_ts);
      uint64_t latency = receive_ts - pong_ts;
      total_latency_ += latency;
      if (receive_ts > last_print_interval_ -print_interval_) {
          print_latency(total_latency_, count_);
          last_print_interval_ = 0;
      }
    } else if (has_data && !pong_samples[0].info().valid()) {
      RCLCPP_ERROR(this->get_logger(), "lost pong writer before end of test");
      throw std::runtime_error("lost pong writer before end of test");
    }
  }

private:
  const uint64_t max_samples_;
  const uint64_t max_execution_time_;
  const dds::core::Duration wait_timeout_;
  const uint64_t print_interval_;
  uint64_t count_ = 0;
  uint64_t total_latency_ = 0;
  uint64_t last_print_interval_ = 0;
  uint64_t start_ts_ = 0;
  uint64_t send_ts_ = 0;
  dds::domain::DomainParticipant participant_{nullptr};
  dds::pub::DataWriter<CameraImageType> writer_{nullptr};
  dds::sub::DataReader<CameraImageType> reader_{nullptr};
  dds::sub::cond::ReadCondition read_condition_{nullptr};
  CameraImageType * ping_sample_{nullptr};
  dds::core::cond::WaitSet waitset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

template<typename T>
using BaseCameraImagePublisherPlain = CameraImagePublisher<T, CameraImageManipulator<T>>;

template<typename T>
using BaseCameraImagePublisherFlat = CameraImagePublisher<T, CameraImageManipulatorFlat<T>>;

template<typename T>
using BaseCameraImagePublisherFlatZc = CameraImagePublisher<T, CameraImageManipulatorFlat<T>>;

template<typename T>
using BaseCameraImagePublisherZc = CameraImagePublisher<T, CameraImageManipulatorZc<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImagePublisher_hpp_
