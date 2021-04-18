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

#ifndef DataAllocator_hpp_
#define DataAllocator_hpp_

#include <dds/dds.hpp>

namespace rti { namespace connext_nodes_cpp {

template<typename T>
class DataAllocatorDynamic
{
public:
  // Dynamically pre-allocate a sample.
  static T * prealloc(
    dds::pub::DataWriter<T> & writer)
  {
    (void)writer;
    
    return new T();
  }
  
  static T * alloc(
    dds::pub::DataWriter<T> & writer,
    T * const preallocd_sample)
  {
    (void)writer;
    assert(preallocd_sample != nullptr);

    return preallocd_sample;
  }
};

template<typename T>
class DataAllocatorLoan
{
public:
  // No pre-allocated samples, since we are going to loan them from the writer
  static T * prealloc(
    dds::pub::DataWriter<T> & writer)
  {
    (void)writer;
    
    return nullptr;
  }
  
  // Return a sample loaned from the DataWriter
  static T * alloc(
    dds::pub::DataWriter<T> & writer,
    T * const preallocd_sample)
  {
    (void)preallocd_sample;
    assert(preallocd_sample == nullptr);

    return writer.extensions().get_loan();
  }
};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // DataAllocator_hpp_
