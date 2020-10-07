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

TEST(Joint, joint_ball)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_BALL), errors);
  EXPECT_FALSE(errors.empty());
  ASSERT_FALSE(model);
}

TEST(Joint, joint_continuous)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_CONTINUOUS), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_continuous", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_continuous");

  EXPECT_EQ("joint_continuous", joint->name);
  EXPECT_EQ(urdf::Joint::CONTINUOUS, joint->type);
  ASSERT_NE(nullptr, joint->dynamics);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->damping);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->friction);
  ASSERT_EQ(nullptr, joint->limits);
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_fixed)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_FIXED), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_fixed", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_fixed");

  EXPECT_EQ("joint_fixed", joint->name);
  EXPECT_EQ(urdf::Joint::FIXED, joint->type);
  ASSERT_EQ(nullptr, joint->dynamics);
  ASSERT_EQ(nullptr, joint->limits);
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_prismatic)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_PRISMATIC), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_prismatic", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_prismatic");

  EXPECT_EQ("joint_prismatic", joint->name);
  EXPECT_EQ(urdf::Joint::PRISMATIC, joint->type);
  ASSERT_NE(nullptr, joint->dynamics);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->damping);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->friction);
  ASSERT_NE(nullptr, joint->limits);
  EXPECT_DOUBLE_EQ(-0.2, joint->limits->lower);
  EXPECT_DOUBLE_EQ(0.2, joint->limits->upper);
  EXPECT_DOUBLE_EQ(-1, joint->limits->effort);  // SDFormat default
  EXPECT_DOUBLE_EQ(-1, joint->limits->velocity);  // SDFormat default
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}
