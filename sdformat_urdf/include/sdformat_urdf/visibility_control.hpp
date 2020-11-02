// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef SDFORMAT_URDF__VISIBILITY_CONTROL_HPP_
#define SDFORMAT_URDF__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SDFORMAT_URDF_EXPORT __attribute__ ((dllexport))
    #define SDFORMAT_URDF_IMPORT __attribute__ ((dllimport))
  #else
    #define SDFORMAT_URDF_EXPORT __declspec(dllexport)
    #define SDFORMAT_URDF_IMPORT __declspec(dllimport)
  #endif
  #ifdef SDFORMAT_URDF_BUILDING_DLL
    #define SDFORMAT_URDF_PUBLIC SDFORMAT_URDF_EXPORT
  #else
    #define SDFORMAT_URDF_PUBLIC SDFORMAT_URDF_IMPORT
  #endif
  #define SDFORMAT_URDF_PUBLIC_TYPE SDFORMAT_URDF_PUBLIC
  #define SDFORMAT_URDF_LOCAL
#else
  #define SDFORMAT_URDF_EXPORT __attribute__ ((visibility("default")))
  #define SDFORMAT_URDF_IMPORT
  #if __GNUC__ >= 4
    #define SDFORMAT_URDF_PUBLIC __attribute__ ((visibility("default")))
    #define SDFORMAT_URDF_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SDFORMAT_URDF_PUBLIC
    #define SDFORMAT_URDF_LOCAL
  #endif
  #define SDFORMAT_URDF_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SDFORMAT_URDF__VISIBILITY_CONTROL_HPP_
