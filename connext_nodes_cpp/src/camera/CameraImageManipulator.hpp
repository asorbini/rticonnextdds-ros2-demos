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
class CameraImageManipulator
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

  static void populate(CameraImageType &sample, int count)
  {
    sample.format(camera::common::Format::RGB);
    sample.resolution().height(camera::common::CAMERA_HEIGHT_DEFAULT);
    sample.resolution().width(camera::common::CAMERA_WIDTH_DEFAULT);

    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + count) % 124;
      sample.data()[i] = image_value;
    }
  }

  static void display(rclcpp::Logger logger, const CameraImageType &sample)
  {
    std::ostringstream msg;
    msg << "CameraImage[plain]: ts=" << sample.timestamp() <<
      ", fmt=" << sample.format() << ", data[0..3]={ ";

    for (int i = 0; i < 4; i++) {
        msg << sample.data()[i] << " ";
    }
    msg << "}";

    RCLCPP_INFO(logger, msg.str().c_str());
  }

  static uint64_t timestamp(const CameraImageType & sample)
  {
    return sample.timestamp();
  }

  static void timestamp(CameraImageType & sample, const uint64_t ts)
  {
    sample.timestamp(ts);
  }
};

template<typename CameraImageType>
class CameraImageManipulatorZc
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

  static void populate(CameraImageType &sample, int count)
  {
    CameraImageManipulator<CameraImageType>::populate(sample, count);
  }

  static void display(rclcpp::Logger logger, const CameraImageType &sample)
  {
    CameraImageManipulator<CameraImageType>::display(logger, sample);
  }

  static uint64_t timestamp(const CameraImageType & sample)
  {
    return CameraImageManipulator<CameraImageType>::timestamp(sample);
  }

  static void timestamp(CameraImageType & sample, const uint64_t ts)
  {
    CameraImageManipulator<CameraImageType>::timestamp(sample, ts);
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
  
  // Return a sample loaned from the DataWriter
  static CameraImageType * alloc(
    dds::pub::DataWriter<CameraImageType> & writer,
    CameraImageType * const preallocd_sample)
  {
    (void)preallocd_sample;
    assert(preallocd_sample == nullptr);

    return writer.extensions().get_loan();
  }

  static void populate(CameraImageType &sample, int count)
  {
    auto image = sample.root();
    image.format(camera::common::Format::RGB);
    image.resolution().height(camera::common::CAMERA_HEIGHT_DEFAULT);
    image.resolution().width(camera::common::CAMERA_WIDTH_DEFAULT);

    auto image_data = image.data();
    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + count) % 124;
      image_data.set_element(i, image_value);
    }
  }

  void display(rclcpp::Logger logger, const CameraImageType &sample)
  {
    auto image = sample.root();

    std::ostringstream msg;
    msg << "CameraImage[flat]: ts=" << image.timestamp() <<
      ", fmt=" << image.format() << ", data[0..3]={ ";

    const uint8_t *image_data = image.data().get_elements();
    for (int i = 0; i < 4; i++) {
        msg << image_data[i] << " ";
    }
    msg << "}";

    RCLCPP_INFO(logger, msg.str().c_str());
  }

  static uint64_t timestamp(const CameraImageType & sample)
  {
    return sample.root().timestamp();
  }

  static void timestamp(CameraImageType & sample, const uint64_t ts)
  {
    sample.root().timestamp(ts);
  }
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // CameraImageManipulator_hpp_
