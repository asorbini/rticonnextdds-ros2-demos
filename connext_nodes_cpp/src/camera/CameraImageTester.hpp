// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CameraImageTester_hpp_
#define CameraImageTester_hpp_

#include <dds/dds.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>

#include "rclcpp/rclcpp.hpp"

#include "connext_nodes/visibility_control.h"

#include "CameraImageManipulator.hpp"
#include "CameraImageTestOptions.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImageTester : public rclcpp::Node
{
protected:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImageTester(
    const char * const name,
    const rclcpp::NodeOptions & options,
    const CameraImageTestOptions & test_options = CameraImageTestOptions::defaults())
  : Node(name, options),
    test_options_(test_options)
  {}

  virtual ~CameraImageTester()
  {
    // stop waitset thread pool if created
    if (nullptr != awaitset_) {
      awaitset_->stop();
    }
  }

  virtual void init_test(
    const char * const writer_topic_name,
    const char * const writer_type_name,
    const char * const writer_qos_profile,
    const char * const reader_topic_name,
    const char * const reader_type_name,
    const char * const reader_qos_profile)
  {
    // look up DomainParticipant
    participant_ = dds::domain::find(test_options_.domain_id);
    if (dds::core::null == participant_) {
      RCLCPP_ERROR(this->get_logger(),
        "failed to lookup DomainParticipant. Is the application running on rmw_connextdds?");
      throw std::runtime_error("failed to lookup DomainParticipant");
    }

    // Create a custom Publisher and Subscriber
    publisher_ = dds::pub::Publisher(participant_);
    subscriber_ = dds::sub::Subscriber(participant_);

    writer_ = create_writer(
      writer_topic_name, writer_type_name, writer_qos_profile);

    reader_ = create_reader(
      reader_topic_name, reader_type_name, reader_qos_profile);

    // Ask sample allocator to preallocate a sample if needed.
    cached_sample_ = A::prealloc(writer_);

    initialize_waitset();
  }

  virtual void shutdown()
  {
    using namespace dds::core::status;

    test_active_ = false;

    // Disable all events on status conditions
    reader_condition_.enabled_statuses(StatusMask::none());
    writer_condition_.enabled_statuses(StatusMask::none());
    rclcpp::shutdown();
  }

  virtual dds::pub::DataWriter<T> create_writer(
    const char * const topic_name,
    const char * const type_name,
    const char * const qos_profile)
  {
    using namespace dds::core;

    dds::topic::Topic<T> topic_ping(participant_, topic_name, type_name);
    
    // Customize the writer's QoS using one of the built-in profiles tuned for
    // large data.
    auto qos_provider = QosProvider::Default();
    auto writer_qos = qos_provider.datawriter_qos(qos_profile);
    rti::core::policy::Property props;
    // Optional optimization: prevent dynamic allocation of serialization buffer
    props.set({
      "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
      "LENGTH_UNLIMITED"
    }, false);
    writer_qos << props;
    writer_qos << policy::Durability::TransientLocal();

    return dds::pub::DataWriter<T>(publisher_, topic_ping, writer_qos);
  }

  virtual dds::sub::DataReader<T> create_reader(
    const char * const topic_name,
    const char * const type_name,
    const char * const qos_profile)
  {
    using namespace dds::core;

    dds::topic::Topic<T> topic_pong(participant_, topic_name, type_name);
    
    // Customize the reader's QoS using one of the built-in profiles tuned for
    // large data.
    auto qos_provider = QosProvider::Default();
    auto reader_qos = qos_provider.datareader_qos(qos_profile);
    // Optional optimization: prevent dynamic allocation of received fragments
    rti::core::policy::DataReaderResourceLimits dr_limits;
    dr_limits.dynamically_allocate_fragmented_samples(false);
    reader_qos << dr_limits;
    reader_qos << policy::Durability::TransientLocal();

    return dds::sub::DataReader<T>(subscriber_, topic_pong, reader_qos);
  }

  virtual void initialize_waitset()
  {
    using namespace dds::core;

    // Create an AsyncWaitSet to call our listeners on dedicated user threads,
    // instead of calling them from the middleware's transport receive threads.
    rti::core::cond::AsyncWaitSetProperty ws_props;
    ws_props.thread_pool_size(1);
    awaitset_ = std::make_unique<rti::core::cond::AsyncWaitSet>(ws_props);

    // Attach DataReader's StatusCondition to detect "subscription matched"
    // and "data available".
    reader_condition_ = cond::StatusCondition(reader_);
    reader_condition_.enabled_statuses(
      status::StatusMask::subscription_matched() |
      status::StatusMask::data_available());
    reader_condition_->handler([this](){
      on_reader_active();
    });
    awaitset_->attach_condition(reader_condition_);

    // Attach DataWriter's StatusCondition to detect "publication matched"
    writer_condition_ = cond::StatusCondition(writer_);
    writer_condition_.enabled_statuses(
      status::StatusMask::publication_matched());
    writer_condition_->handler([this](){
      // no need to differentiate, since we only enabled one status.
      on_matched();
    });
    awaitset_->attach_condition(writer_condition_);

    // Start waitset thread(s)
    awaitset_->start();
  }

  virtual void on_test_state_changed(
    const size_t pub_match_count,
    const size_t sub_match_count,
    const bool was_active,
    const bool is_ready,
    bool & is_active) = 0;

  virtual void on_matched()
  {
    using namespace dds::core::status;

    uint64_t pub_match_count = 0,
      sub_match_count = 0;

    if ((reader_.status_changes() & StatusMask::subscription_matched()).any()) {
      // Access status structures so event flags are reset.
      auto pub_matched_status = reader_.subscription_matched_status();
      (void)pub_matched_status;
      pub_match_count = pub_matched_status.current_count();
      // auto pub_match_count = dds::sub::matched_publications(reader_).size();
    }
    if ((writer_.status_changes() & StatusMask::publication_matched()).any()) {
      auto sub_matched_status = writer_.publication_matched_status();
      (void)sub_matched_status;
      sub_match_count = sub_matched_status.current_count();
      // auto sub_match_count = dds::pub::matched_subscriptions(writer_).size();
    }

    const bool is_ready = pub_match_count > 0 && sub_match_count > 0;
    const bool was_active = test_active_;

    RCLCPP_INFO(this->get_logger(),
      "on_test_state_changed: pubs=%lu, subs=%lu, was_active=%d, is_ready=%d",
      pub_match_count, sub_match_count, was_active, is_ready);

    on_test_state_changed(
      pub_match_count, sub_match_count, was_active, is_ready, test_active_);
    
    RCLCPP_INFO(this->get_logger(),
      "on_test_state_changed(RESULT): is_active=%d", test_active_);
    
    if (!was_active && test_active_) {
      // Test started  
      // Enable "data available" on pong reader's status condition
      reader_condition_.enabled_statuses(
        reader_condition_.enabled_statuses() |
        dds::core::status::StatusMask::data_available());
    } else if (was_active && !test_active_) {
      // Test stopped
      shutdown();
    }
  }

  virtual void on_data() = 0;

  virtual void on_reader_active()
  {
    using namespace dds::core::status;

    if ((reader_.status_changes() & StatusMask::subscription_matched()).any())
    {
      on_matched();
    }
    if ((reader_.status_changes() & StatusMask::data_available()).any())
    {
      on_data();
    }
  }

  virtual void on_writer_active()
  {
    // no need to differentiate, since we only enabled one status.
    on_matched();
  }

  const CameraImageTestOptions test_options_;
  bool test_active_{false};
  dds::domain::DomainParticipant participant_{nullptr};
  dds::pub::Publisher publisher_{nullptr};
  dds::sub::Subscriber subscriber_{nullptr};
  dds::pub::DataWriter<T> writer_{nullptr};
  dds::sub::DataReader<T> reader_{nullptr};
  dds::core::cond::StatusCondition reader_condition_{nullptr};
  dds::core::cond::StatusCondition writer_condition_{nullptr};
  T * cached_sample_{nullptr};
  std::unique_ptr<rti::core::cond::AsyncWaitSet> awaitset_;
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageTester_hpp_
