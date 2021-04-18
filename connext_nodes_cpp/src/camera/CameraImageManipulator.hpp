// (c) 2019-2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

// Utilities used by CameraImage_publisher.cxx and CameraImage_subscriber.cxx.

#ifndef CameraImageManipulator_hpp_
#define CameraImageManipulator_hpp_

#include <dds/dds.hpp>

#include "camera/CameraCommon.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T>
class CameraImageManipulatorPlain
{
public:
  static uint64_t timestamp(const T & sample)
  {
    return sample.timestamp();
  }

  static void timestamp(T & sample, const uint64_t val)
  {
    sample.timestamp(val);
  }

  static uint8_t data(const T & sample, const size_t i)
  {
    return sample.data()[i];
  }

  static void data(
    T & sample, const size_t i, const uint8_t val)
  {
    sample.data()[i] = val;
  }

  static camera::common::Format format(const T & sample)
  {
    return sample.format();
  }

  static void format(T & sample, const camera::common::Format & val)
  {
    sample.format(val);
  }

  static int32_t resolution_height(const T & sample)
  {
    return sample.resolution().height();
  }

  static void resolution_height(T & sample, const int32_t & val)
  {
    sample.resolution().height(val);
  }

  static int32_t resolution_width(const T & sample)
  {
    return sample.resolution().width();
  }

  static void resolution_width(T & sample, const int32_t & val)
  {
    sample.resolution().width(val);
  }
};

template<typename T>
class CameraImageManipulatorFlat
{
public:
  static uint64_t timestamp(const T & sample)
  {
    return sample.root().timestamp();
  }

  static void timestamp(T & sample, const uint64_t ts)
  {
    sample.root().timestamp(ts);
  }

  static uint8_t data(const T & sample, const size_t i)
  {
    return sample.root().data().get_elements()[i];
  }

  static void data(
    T & sample, const size_t i, const uint8_t val)
  {
    sample.root().data().set_element(i, val);
  }

  static camera::common::Format format(const T & sample)
  {
    return sample.root().format();
  }

  static void format(T & sample, const camera::common::Format & val)
  {
    sample.root().format(val);
  }

  static int32_t resolution_height(const T & sample)
  {
    return sample.root().resolution().height();
  }

  static void resolution_height(T & sample, const int32_t & val)
  {
    sample.root().resolution().height(val);
  }

  static int32_t resolution_width(const T & sample)
  {
    return sample.root().resolution().width();
  }

  static void resolution_width(T & sample, const int32_t & val)
  {
    sample.root().resolution().width(val);
  }
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageManipulator_hpp_
