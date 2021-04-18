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

#ifndef DataManipulator_hpp_
#define DataManipulator_hpp_

#include <dds/dds.hpp>

#include "camera/CameraCommon.hpp"

#include "rti/topic/flat/FlatData.hpp"

namespace rti { namespace connext_nodes_cpp {

template<typename T>
class DataManipulatorPlain
{
public:

  static T & get(T & sample) {
    return sample;
  }

  static const T & get(const T & sample) {
    return sample;
  }

  class array {
  public:
    template<typename A, typename E>
    static void ref(A & array, const size_t i, E & val)
    {
        val = array.data() + i;
    }

    template<typename A, typename E>
    static void set(A & array, const size_t i, E & val)
    {
        array[i] = val;
    }
  };
};

template<typename T>
class DataManipulatorFlat
{
public:

  static typename rti::flat::flat_type_traits<T>::offset get(T & sample) {
    return sample.root();
  }

  static typename rti::flat::flat_type_traits<T>::offset::ConstOffset get(const T & sample) {
    return sample.root();
  }

  class array {
  public:
    template<typename A, typename E>
    static void ref(A & array, const size_t i, E & val)
    {
        val = array.get_elements() + i;
    }

    template<typename A, typename E>
    static void set(A array, const size_t i, E & val)
    {
        array.set_element(i, val);
    }
  };

};

}  // namespace connext_nodes_cpp
}  // namespace rti

#endif  // DataManipulator_hpp_
