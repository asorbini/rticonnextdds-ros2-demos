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

#include <string>

#include <dds/dds.hpp>

#include "rclcpp/rclcpp.hpp"

#include "camera/CameraCommon.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename CameraImageType>
class CameraImageAllocatorDynamic
{
public:
  // Dynamically pre-allocate a sample.
  static CameraImageType * prealloc(
    dds::pub::DataWriter<CameraImageType> & writer)
  {
    (void)writer;
    
    return new CameraImageType();
  }
  
  static CameraImageType * alloc(
    dds::pub::DataWriter<CameraImageType> & writer,
    CameraImageType * const preallocd_sample)
  {
    (void)writer;
    assert(preallocd_sample != nullptr);

    return preallocd_sample;
  }
};


template<typename CameraImageType>
class CameraImageAllocatorWriter
{
public:
  // No pre-allocated samples, since we are going to loan them from the writer
  static CameraImageType * prealloc(
    dds::pub::DataWriter<CameraImageType> & writer)
  {
    (void)writer;
    
    return nullptr;
  }
  
  // Return a sample loaned from the DataWriter
  static CameraImageType * alloc(
    dds::pub::DataWriter<CameraImageType> & writer,
    CameraImageType * const preallocd_sample)
  {
    (void)preallocd_sample;
    assert(preallocd_sample == nullptr);

    return writer.extensions().get_loan();
  }
};

template<typename CameraImageType>
class CameraImageManipulatorPlain
{
public:
  static uint64_t timestamp(const CameraImageType & sample)
  {
    return sample.timestamp();
  }

  static void timestamp(CameraImageType & sample, const uint64_t val)
  {
    sample.timestamp(val);
  }

  static uint8_t data(const CameraImageType & sample, const size_t i)
  {
    return sample.data()[i];
  }

  static void data(
    CameraImageType & sample, const size_t i, const uint8_t val)
  {
    sample.data()[i] = val;
  }

  static camera::common::Format format(const CameraImageType & sample)
  {
    return sample.format();
  }

  static void format(CameraImageType & sample, const camera::common::Format & val)
  {
    sample.format(val);
  }

  static int32_t resolution_height(const CameraImageType & sample)
  {
    return sample.resolution().height();
  }

  static void resolution_height(CameraImageType & sample, const int32_t & val)
  {
    sample.resolution().height(val);
  }

  static int32_t resolution_width(const CameraImageType & sample)
  {
    return sample.resolution().width();
  }

  static void resolution_width(CameraImageType & sample, const int32_t & val)
  {
    sample.resolution().width(val);
  }
};

template<typename CameraImageType>
class CameraImageManipulatorFlat
{
public:
  // No pre-allocated samples, since we are going to loan them from the writer
  static CameraImageType * prealloc(
    dds::pub::DataWriter<CameraImageType> & writer)
  {
    (void)writer;
    
    return nullptr;
  }

  static uint64_t timestamp(const CameraImageType & sample)
  {
    return sample.root().timestamp();
  }

  static void timestamp(CameraImageType & sample, const uint64_t ts)
  {
    sample.root().timestamp(ts);
  }

  static uint8_t data(const CameraImageType & sample, const size_t i)
  {
    return sample.root().data().get_elements()[i];
  }

  static void data(
    CameraImageType & sample, const size_t i, const uint8_t val)
  {
    sample.root().data().set_element(i, val);
  }

  static camera::common::Format format(const CameraImageType & sample)
  {
    return sample.root().format();
  }

  static void format(CameraImageType & sample, const camera::common::Format & val)
  {
    sample.root().format(val);
  }

  static int32_t resolution_height(const CameraImageType & sample)
  {
    return sample.root().resolution().height();
  }

  static void resolution_height(CameraImageType & sample, const int32_t & val)
  {
    sample.root().resolution().height(val);
  }

  static int32_t resolution_width(const CameraImageType & sample)
  {
    return sample.root().resolution().width();
  }

  static void resolution_width(CameraImageType & sample, const int32_t & val)
  {
    sample.root().resolution().width(val);
  }
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageManipulator_hpp_
