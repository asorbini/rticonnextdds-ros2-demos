// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONNEXT_NODES_CPP__VISIBILITY_CONTROL_H_
#define CONNEXT_NODES_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONNEXT_NODES_CPP_EXPORT __attribute__ ((dllexport))
    #define CONNEXT_NODES_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CONNEXT_NODES_CPP_EXPORT __declspec(dllexport)
    #define CONNEXT_NODES_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONNEXT_NODES_CPP_BUILDING_DLL
    #define CONNEXT_NODES_CPP_PUBLIC CONNEXT_NODES_CPP_EXPORT
  #else
    #define CONNEXT_NODES_CPP_PUBLIC CONNEXT_NODES_CPP_IMPORT
  #endif
  #define CONNEXT_NODES_CPP_PUBLIC_TYPE CONNEXT_NODES_CPP_PUBLIC
  #define CONNEXT_NODES_CPP_LOCAL
#else
  #define CONNEXT_NODES_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CONNEXT_NODES_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CONNEXT_NODES_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CONNEXT_NODES_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONNEXT_NODES_CPP_PUBLIC
    #define CONNEXT_NODES_CPP_LOCAL
  #endif
  #define CONNEXT_NODES_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONNEXT_NODES_CPP__VISIBILITY_CONTROL_H_
