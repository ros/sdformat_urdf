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


#include <gtest/gtest.h>
#include <sdf/sdf.hh>
#include <sdformat_urdf/sdformat_urdf.hpp>
#include <urdf_model/model.h>
#include <urdf_model/types.h>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Geometry, geometry_box)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_BOX), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_box", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);

  ASSERT_EQ(urdf::Geometry::BOX, link->visual->geometry->type);
  {
    urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<urdf::Box>(link->visual->geometry);
    EXPECT_DOUBLE_EQ(0.1, box->dim.x);
    EXPECT_DOUBLE_EQ(0.2, box->dim.y);
    EXPECT_DOUBLE_EQ(0.4, box->dim.z);
  }

  ASSERT_EQ(urdf::Geometry::BOX, link->collision->geometry->type);
  {
    urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
    EXPECT_DOUBLE_EQ(0.1, box->dim.x);
    EXPECT_DOUBLE_EQ(0.2, box->dim.y);
    EXPECT_DOUBLE_EQ(0.4, box->dim.z);
  }
}

TEST(Geometry, geometry_plane)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_PLANE), errors);
  EXPECT_FALSE(errors.empty());
  ASSERT_FALSE(model);
}
