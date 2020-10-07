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

TEST(Link, link_inertia)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_LINK_INERTIA), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("link_inertia", model->getName());

  urdf::LinkConstSharedPtr link = model->getLink("link");

  EXPECT_EQ("link", link->name);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(116.0, link->inertial->mass);
  EXPECT_DOUBLE_EQ(5.652232699207, link->inertial->ixx);
  EXPECT_DOUBLE_EQ(-0.009719934438, link->inertial->ixy);
  EXPECT_DOUBLE_EQ(1.293988226423, link->inertial->ixz);
  EXPECT_DOUBLE_EQ(5.669473158652, link->inertial->iyy);
  EXPECT_DOUBLE_EQ(-0.007379583694, link->inertial->iyz);
  EXPECT_DOUBLE_EQ(3.683196351726, link->inertial->izz);
}
