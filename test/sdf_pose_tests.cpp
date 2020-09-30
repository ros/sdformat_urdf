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
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include <sdformat_urdf/sdformat_urdf.hpp>
#include <urdf_model/model.h>
#include <urdf_model/types.h>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Pose, pose_collision)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_COLLISION), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_collision", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d expected_collision_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_other_pose(0, 0, 0, 0, 0, 0);

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_other_pose, link->visual->origin);
  EXPECT_POSE(expected_collision_pose, link->collision->origin);
}

TEST(Pose, pose_collision_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_COLLISION_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_collision_in_frame", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d frame_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_collision_pose =
    ignition::math::Pose3d{0.2, 0.4, 0.8, 0.2, 0.3, 0.4} + frame_pose;
  const ignition::math::Pose3d expected_other_pose{0, 0, 0, 0, 0, 0};

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_other_pose, link->visual->origin);
  EXPECT_POSE(expected_collision_pose, link->collision->origin);
}

TEST(Pose, pose_inertial)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_INERTIAL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_inertial", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d expected_inertial_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_other_pose(0, 0, 0, 0, 0, 0);

  EXPECT_POSE(expected_inertial_pose, link->inertial->origin);
  EXPECT_POSE(expected_other_pose, link->visual->origin);
  EXPECT_POSE(expected_other_pose, link->collision->origin);
}

TEST(Pose, pose_inertial_in_frame)
{
  GTEST_SKIP() << "https://github.com/osrf/sdformat/issues/380";

  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_INERTIAL_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_inertial_in_frame", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d frame_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_inertial_pose =
    ignition::math::Pose3d{0.2, 0.4, 0.8, 0.2, 0.3, 0.4} + frame_pose;
  const ignition::math::Pose3d expected_other_pose{0, 0, 0, 0, 0, 0};

  EXPECT_POSE(expected_inertial_pose, link->inertial->origin);
  EXPECT_POSE(expected_other_pose, link->visual->origin);
  EXPECT_POSE(expected_other_pose, link->collision->origin);
}

TEST(Pose, pose_joint)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_JOINT), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_joint", model->getName());

  ASSERT_EQ(2u, model->links_.size());
  ASSERT_EQ(1u, model->joints_.size());
  urdf::JointConstSharedPtr joint = model->joints_.begin()->second;
  ASSERT_NE(nullptr, joint);
  urdf::LinkConstSharedPtr child_link = model->getLink(joint->child_link_name);
  ASSERT_NE(nullptr, child_link);

  // In URDF joint is in parent link frame
  // The child link in URDF lives in the joint frame
  const ignition::math::Pose3d model_to_parent_in_model(0, 0, 0, 0, 0, 0);
  const ignition::math::Pose3d model_to_child_in_model(0, 0, 0, 0, 0, 0);
  const ignition::math::Pose3d model_to_joint_in_model(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);

  const ignition::math::Pose3d parent_to_joint_in_parent =
    model_to_joint_in_model - model_to_parent_in_model;
  const ignition::math::Pose3d joint_to_child_in_joint =
    model_to_child_in_model - model_to_joint_in_model;

  EXPECT_POSE(parent_to_joint_in_parent, joint->parent_to_joint_origin_transform);

  // URDF link C++ structure does not have an origin - instead the pose of the
  // link should be added to the visual, collision, and inertial members.
  EXPECT_POSE(joint_to_child_in_joint, child_link->inertial->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->visual->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->collision->origin);
}

TEST(Pose, pose_joint_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_JOINT_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_joint_in_frame", model->getName());

  ASSERT_EQ(2u, model->links_.size());
  ASSERT_EQ(1u, model->joints_.size());
  urdf::JointConstSharedPtr joint = model->joints_.begin()->second;
  ASSERT_NE(nullptr, joint);
  urdf::LinkConstSharedPtr child_link = model->getLink(joint->child_link_name);
  ASSERT_NE(nullptr, child_link);

  // In URDF joint is in parent link frame
  // The child link in URDF lives in the joint frame
  const ignition::math::Pose3d model_to_parent_in_model(0, 0, 0, 0, 0, 0);
  const ignition::math::Pose3d model_to_child_in_model(0, 0, 0, 0, 0, 0);
  const ignition::math::Pose3d model_to_frame_in_model(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d frame_to_joint_in_frame(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);

  const ignition::math::Pose3d model_to_joint_in_model =
    frame_to_joint_in_frame + model_to_frame_in_model;
  const ignition::math::Pose3d parent_to_joint_in_parent =
    model_to_joint_in_model - model_to_parent_in_model;
  const ignition::math::Pose3d joint_to_child_in_joint =
    model_to_child_in_model - model_to_joint_in_model;

  EXPECT_POSE(parent_to_joint_in_parent, joint->parent_to_joint_origin_transform);

  // URDF link C++ structure does not have an origin - instead the pose of the
  // link should be added to the visual, collision, and inertial members.
  EXPECT_POSE(joint_to_child_in_joint, child_link->inertial->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->visual->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->collision->origin);
}

TEST(Pose, pose_link)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_LINK), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_link", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  // URDF link C++ structure does not have an origin - instead the pose of the
  // link should be added to the visual, collision, and inertial members.
  const ignition::math::Pose3d expected_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);

  EXPECT_POSE(expected_pose, link->inertial->origin);
  EXPECT_POSE(expected_pose, link->visual->origin);
  EXPECT_POSE(expected_pose, link->collision->origin);
}

TEST(Pose, pose_link_all)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_LINK_ALL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_link_all", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  // URDF link C++ structure does not have an origin - instead the pose of the
  // link should be added to the visual, collision, and inertial members.
  const ignition::math::Pose3d link_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_inertial_pose =
    link_pose + ignition::math::Pose3d{0.05, 0.1, 0.2, 0.4, 0.5, 0.6};
  const ignition::math::Pose3d expected_collision_pose =
    link_pose + ignition::math::Pose3d{0.04, 0.8, 0.16, 0.3, 0.4, 0.5};
  const ignition::math::Pose3d expected_visual_pose =
    link_pose + ignition::math::Pose3d{0.03, 0.6, 0.12, 0.2, 0.3, 0.4};

  EXPECT_POSE(expected_inertial_pose, link->inertial->origin);
  EXPECT_POSE(expected_visual_pose, link->visual->origin);
  EXPECT_POSE(expected_collision_pose, link->collision->origin);
}

TEST(Pose, pose_link_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_LINK_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_link_in_frame", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d frame_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_pose =
    ignition::math::Pose3d{0.2, 0.4, 0.8, 0.2, 0.3, 0.4} + frame_pose;
  EXPECT_POSE(expected_pose, link->inertial->origin);
  EXPECT_POSE(expected_pose, link->visual->origin);
  EXPECT_POSE(expected_pose, link->collision->origin);
}

TEST(Pose, pose_model)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_MODEL), errors);
  EXPECT_FALSE(errors.empty());
  ASSERT_FALSE(model);
}

TEST(Pose, pose_visual)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_VISUAL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_visual", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d expected_visual_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_other_pose(0, 0, 0, 0, 0, 0);

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_visual_pose, link->visual->origin);
  EXPECT_POSE(expected_other_pose, link->collision->origin);
}

TEST(Pose, pose_visual_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_VISUAL_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_visual_in_frame", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d frame_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_visual_pose =
    ignition::math::Pose3d{0.2, 0.4, 0.8, 0.2, 0.3, 0.4} + frame_pose;
  const ignition::math::Pose3d expected_other_pose{0, 0, 0, 0, 0, 0};

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_visual_pose, link->visual->origin);
  EXPECT_POSE(expected_other_pose, link->collision->origin);
}
