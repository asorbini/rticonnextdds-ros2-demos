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

#include "PingPongSubscriber.hpp"

#include "DataManipulator.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T, typename M, typename A>
class CameraImageSubscriber : public PingPongSubscriber<T, A>
{
public:
  CONNEXT_NODES_CPP_PUBLIC
  CameraImageSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongSubscriber<T, A>(name, options)
  {
    this->init_test();
  }

protected:
  virtual void prepare_pong(T * const pong, const uint64_t ping_ts)
  {
    M::get(*pong).timestamp(ping_ts);
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = M::get(ping_samples[0].data()).timestamp();
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    std::ostringstream & msg)
  {
    auto & sample = ping_samples[0].data();

    msg << "[" << M::get(sample).timestamp() << "] " <<  M::get(sample).format();

    for (int i = 0; i < 4; i++) {
        const uint8_t * el;
        M::array::ref(M::get(sample).data(), i, el);

        msg << "0x" << 
          std::hex << std::uppercase <<
          std::setfill('0') << std::setw(2) <<
          (int) *el <<
          std::nouppercase << std::dec <<
          " ";
    }
  }
};

template<typename T>
using BaseCameraImageSubscriberPlain = CameraImageSubscriber<T, DataManipulatorPlain<T>, DataAllocatorDynamic<T>>;

template<typename T>
using BaseCameraImageSubscriberFlat = CameraImageSubscriber<T, DataManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImageSubscriberFlatZc = CameraImageSubscriber<T, DataManipulatorFlat<T>, DataAllocatorLoan<T>>;

template<typename T>
using BaseCameraImageSubscriberZc = CameraImageSubscriber<T, DataManipulatorPlain<T>, DataAllocatorLoan<T>>;

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageSubscriber_hpp_
