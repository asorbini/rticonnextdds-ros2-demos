/*
 * (c) 2019 Copyright, Real-Time Innovations, Inc.  All rights reserved.
 *
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the Software.  Licensee has the right to distribute object form
 * only for use with RTI products.  The Software is provided "as is", with no
 * warranty of any type, including any warranty for fitness for any purpose.
 * RTI is under no obligation to maintain or support the Software.  RTI shall
 * not be liable for any incidental or consequential damages arising out of the
 * use or inability to use the software.
 */

#ifndef dds_camera_pub_hpp
#define dds_camera_pub_hpp

#include "dds_camera_common.hpp"

#include <iostream>
#include <memory>  // for shared_ptr and unique_ptr
#include <signal.h>

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include <rti/config/Logger.hpp>
#include <rti/core/ListenerBinder.hpp>
#include <rti/util/util.hpp>  // for sleep()

#include <rti/zcopy/pub/ZcopyDataWriter.hpp>
#include <rti/zcopy/sub/ZcopyDataReader.hpp>

void publisher_flat(const ApplicationOptions &options);
void publisher_flat_zero_copy(const ApplicationOptions &options);
void publisher_zero_copy(const ApplicationOptions &options);
void publisher_plain(const ApplicationOptions &options);

#endif  // dds_camera_pub_hpp

