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

#ifndef TEST_TOOLS_HPP_
#define TEST_TOOLS_HPP_

#include <sdf/sdf.hh>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

inline
std::string
get_file(const char * path)
{
  std::ifstream file_reader(path, std::ifstream::in);
  std::stringstream buffer;
  buffer << file_reader.rdbuf();
  return buffer.str();
}

#define EXPECT_POSE(expected_ign, actual_urdf) \
  do { \
    const auto actual_ign = ignition::math::Pose3d{ \
      actual_urdf.position.x, \
      actual_urdf.position.y, \
      actual_urdf.position.z, \
      actual_urdf.rotation.w, \
      actual_urdf.rotation.x, \
      actual_urdf.rotation.y, \
      actual_urdf.rotation.z}; \
    EXPECT_EQ(expected_ign, actual_ign); \
  } while (false)

std::ostream & operator<<(std::ostream & os, const sdf::Errors & errors)
{
  for (const auto & error : errors) {
    os << error;
  }
  return os;
}

#endif  // TEST_TOOLS_HPP_
