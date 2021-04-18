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

#include "PingPongPublisher.hpp"

#include "CameraImageManipulator.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImagePublisher : public PingPongPublisher<T, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImagePublisher(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongPublisher<T, A>(name, options)
  {}

protected:
  virtual void prepare_ping(T * const sample, const bool final)
  {
    if (final) {
      M::timestamp(*sample, 0);
      return;
    }

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
  }

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    dds::sub::LoanedSamples<T> & pong_samples,
    uint64_t & pong_timestamp)
  {
    pong_timestamp = M::timestamp(pong_samples[0].data());
  }
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
