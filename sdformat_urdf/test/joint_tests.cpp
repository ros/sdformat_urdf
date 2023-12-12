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
#include <urdf_model/model.h>
#include <urdf_model/types.h>
#include <sdformat_urdf/sdformat_urdf.hpp>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Types.hh>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Joint, joint_ball)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_BALL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_ball", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_ball");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_ball", joint->name);
  EXPECT_EQ(urdf::Joint::FLOATING, joint->type);
  EXPECT_EQ(nullptr, joint->dynamics);
  EXPECT_EQ(nullptr, joint->limits);
  EXPECT_EQ(nullptr, joint->safety);
  EXPECT_EQ(nullptr, joint->calibration);
  EXPECT_EQ(nullptr, joint->mimic);
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
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_continuous", joint->name);
  EXPECT_EQ(urdf::Joint::CONTINUOUS, joint->type);
  EXPECT_DOUBLE_EQ(1, joint->axis.x);
  EXPECT_DOUBLE_EQ(0, joint->axis.y);
  EXPECT_DOUBLE_EQ(0, joint->axis.z);
  ASSERT_NE(nullptr, joint->dynamics);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->damping);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->friction);
  EXPECT_EQ(nullptr, joint->limits);
  EXPECT_EQ(nullptr, joint->safety);
  EXPECT_EQ(nullptr, joint->calibration);
  EXPECT_EQ(nullptr, joint->mimic);
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
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_fixed", joint->name);
  EXPECT_EQ(urdf::Joint::FIXED, joint->type);
  EXPECT_EQ(nullptr, joint->dynamics);
  EXPECT_EQ(nullptr, joint->limits);
  EXPECT_EQ(nullptr, joint->safety);
  EXPECT_EQ(nullptr, joint->calibration);
  EXPECT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_gearbox)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_GEARBOX), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
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
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_prismatic", joint->name);
  EXPECT_EQ(urdf::Joint::PRISMATIC, joint->type);
  EXPECT_DOUBLE_EQ(0, joint->axis.x);
  EXPECT_DOUBLE_EQ(1, joint->axis.y);
  EXPECT_DOUBLE_EQ(0, joint->axis.z);
  ASSERT_NE(nullptr, joint->dynamics);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->damping);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->friction);
  ASSERT_NE(nullptr, joint->limits);
  EXPECT_DOUBLE_EQ(-0.2, joint->limits->lower);
  EXPECT_DOUBLE_EQ(0.2, joint->limits->upper);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->effort);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->velocity);
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_prismatic_no_axis)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_PRISMATIC_NO_AXIS), errors);
  EXPECT_FALSE(errors.empty());
  ASSERT_FALSE(model);
}

TEST(Joint, joint_revolute)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_revolute", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_revolute");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_revolute", joint->name);
  EXPECT_EQ(urdf::Joint::REVOLUTE, joint->type);
  ASSERT_NE(nullptr, joint->dynamics);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->damping);
  EXPECT_DOUBLE_EQ(0, joint->dynamics->friction);
  ASSERT_NE(nullptr, joint->limits);
  EXPECT_DOUBLE_EQ(-1.5, joint->limits->lower);
  EXPECT_DOUBLE_EQ(1.5, joint->limits->upper);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->effort);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->velocity);
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_revolute2)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE2), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_revolute2", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_revolute2");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_revolute2", joint->name);
  EXPECT_EQ(urdf::Joint::FLOATING, joint->type);
  ASSERT_EQ(nullptr, joint->dynamics);
  ASSERT_EQ(nullptr, joint->limits);
  ASSERT_EQ(nullptr, joint->safety);
  ASSERT_EQ(nullptr, joint->calibration);
  ASSERT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_revolute_axis)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE_AXIS), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_revolute_axis", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_revolute");
  ASSERT_NE(nullptr, joint);

  const gz::math::Vector3d expected_axis{0.1, 1.23, 4.567};
  const gz::math::Vector3d actual_axis{joint->axis.x, joint->axis.y, joint->axis.z};

  EXPECT_EQ("joint_revolute", joint->name);
  EXPECT_EQ(urdf::Joint::REVOLUTE, joint->type);
  EXPECT_EQ(expected_axis.Normalized(), actual_axis.Normalized());
}

TEST(Joint, joint_revolute_axis_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE_AXIS_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_revolute_axis_in_frame", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_revolute");
  ASSERT_NE(nullptr, joint);

  const gz::math::Pose3d model_to_frame_in_model{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};
  const gz::math::Pose3d model_to_child_in_model{0.1, 0, 0.1, 0, 0, 0};
  const gz::math::Pose3d frame_to_child_in_frame =
    model_to_frame_in_model.Inverse() * model_to_child_in_model;
  const gz::math::Pose3d child_to_joint_in_child{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d frame_to_joint_in_frame =
    frame_to_child_in_frame * child_to_joint_in_child;

  const gz::math::Vector3d axis_in_frame{0.1, 1.23, 4.567};
  const gz::math::Vector3d axis_in_joint =
    frame_to_joint_in_frame.Inverse().Rot().RotateVector(axis_in_frame);

  const gz::math::Vector3d actual_axis{joint->axis.x, joint->axis.y, joint->axis.z};

  EXPECT_EQ("joint_revolute", joint->name);
  EXPECT_EQ(urdf::Joint::REVOLUTE, joint->type);
  EXPECT_EQ(axis_in_joint.Normalized(), actual_axis.Normalized());
}

TEST(Joint, joint_revolute_default_limits)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE_DEFAULT_LIMITS), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_revolute_default_limits", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_revolute");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_revolute", joint->name);
  EXPECT_EQ(urdf::Joint::REVOLUTE, joint->type);
  ASSERT_NE(nullptr, joint->limits);
  EXPECT_DOUBLE_EQ(-1e16, joint->limits->lower);  // SDFormat default
  EXPECT_DOUBLE_EQ(1e16, joint->limits->upper);  // SDFormat default
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->effort);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), joint->limits->velocity);
}

TEST(Joint, joint_revolute_two_joints_two_links)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_REVOLUTE_TWO_JOINTS_TWO_LINKS), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Joint, joint_screw)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_SCREW), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_screw", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_screw");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_screw", joint->name);
  EXPECT_EQ(urdf::Joint::FLOATING, joint->type);
  ASSERT_EQ(nullptr, joint->dynamics);
  ASSERT_EQ(nullptr, joint->limits);
  EXPECT_EQ(nullptr, joint->safety);
  EXPECT_EQ(nullptr, joint->calibration);
  EXPECT_EQ(nullptr, joint->mimic);
}

TEST(Joint, joint_universal)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_JOINT_UNIVERSAL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_TRUE(model);
  ASSERT_EQ("joint_universal", model->getName());

  urdf::JointConstSharedPtr joint = model->getJoint("joint_universal");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("joint_universal", joint->name);
  EXPECT_EQ(urdf::Joint::FLOATING, joint->type);
  EXPECT_EQ(nullptr, joint->dynamics);
  EXPECT_EQ(nullptr, joint->limits);
  EXPECT_EQ(nullptr, joint->safety);
  EXPECT_EQ(nullptr, joint->calibration);
  EXPECT_EQ(nullptr, joint->mimic);
}
