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
#include <sdf/Types.hh>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Pose, pose_chain)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_CHAIN), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_chain", model->getName());

  ASSERT_EQ(4u, model->links_.size());
  ASSERT_EQ(3u, model->joints_.size());

  urdf::LinkConstSharedPtr link_1 = model->getLink("link_1");
  ASSERT_NE(nullptr, link_1);
  urdf::LinkConstSharedPtr link_2 = model->getLink("link_2");
  ASSERT_NE(nullptr, link_2);
  urdf::LinkConstSharedPtr link_3 = model->getLink("link_3");
  ASSERT_NE(nullptr, link_3);
  urdf::LinkConstSharedPtr link_4 = model->getLink("link_4");
  ASSERT_NE(nullptr, link_4);

  urdf::JointConstSharedPtr joint_1 = model->getJoint("joint_1");
  ASSERT_NE(nullptr, joint_1);
  urdf::JointConstSharedPtr joint_2 = model->getJoint("joint_2");
  ASSERT_NE(nullptr, joint_2);
  urdf::JointConstSharedPtr joint_3 = model->getJoint("joint_3");
  ASSERT_NE(nullptr, joint_3);

  const gz::math::Pose3d model_to_link_1_in_model{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const gz::math::Pose3d model_to_link_2_in_model{0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const gz::math::Pose3d model_to_link_3_in_model{0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  const gz::math::Pose3d model_to_link_4_in_model{0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  const gz::math::Pose3d link_2_to_joint_1_in_link_2 {0.9, 0.8, 0.7, 0.6, 0.5, 0.4};
  const gz::math::Pose3d link_3_to_joint_2_in_link_3{0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
  const gz::math::Pose3d link_4_to_joint_3_in_link_4{0.7, 0.6, 0.5, 0.4, 0.3, 0.2};

  const gz::math::Pose3d model_to_joint_1_in_model =
    model_to_link_2_in_model * link_2_to_joint_1_in_link_2;
  const gz::math::Pose3d model_to_joint_2_in_model =
    model_to_link_3_in_model * link_3_to_joint_2_in_link_3;
  const gz::math::Pose3d model_to_joint_3_in_model =
    model_to_link_4_in_model * link_4_to_joint_3_in_link_4;

  const gz::math::Pose3d link_1_to_joint_1_in_link_1 =
    model_to_link_1_in_model.Inverse() * model_to_joint_1_in_model;
  const gz::math::Pose3d joint_1_to_link_2_in_joint_1 =
    model_to_joint_1_in_model.Inverse() * model_to_link_2_in_model;
  const gz::math::Pose3d joint_2_to_link_3_in_joint_2 =
    model_to_joint_2_in_model.Inverse() * model_to_link_3_in_model;
  const gz::math::Pose3d joint_3_to_link_4_in_joint_3 =
    model_to_joint_3_in_model.Inverse() * model_to_link_4_in_model;

  const gz::math::Pose3d joint_1_to_joint_2_in_joint_1 =
    model_to_joint_1_in_model.Inverse() * model_to_joint_2_in_model;
  const gz::math::Pose3d joint_2_to_joint_3_in_joint_2 =
    model_to_joint_2_in_model.Inverse() * model_to_joint_3_in_model;

  EXPECT_POSE(gz::math::Pose3d::Zero, link_1->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link_1->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link_1->collision->origin);

  EXPECT_POSE(link_1_to_joint_1_in_link_1, joint_1->parent_to_joint_origin_transform);

  EXPECT_POSE(joint_1_to_link_2_in_joint_1, link_2->inertial->origin);
  EXPECT_POSE(joint_1_to_link_2_in_joint_1, link_2->visual->origin);
  EXPECT_POSE(joint_1_to_link_2_in_joint_1, link_2->collision->origin);

  EXPECT_POSE(joint_1_to_joint_2_in_joint_1, joint_2->parent_to_joint_origin_transform);

  EXPECT_POSE(joint_2_to_link_3_in_joint_2, link_3->inertial->origin);
  EXPECT_POSE(joint_2_to_link_3_in_joint_2, link_3->visual->origin);
  EXPECT_POSE(joint_2_to_link_3_in_joint_2, link_3->collision->origin);

  EXPECT_POSE(joint_2_to_joint_3_in_joint_2, joint_3->parent_to_joint_origin_transform);

  EXPECT_POSE(joint_3_to_link_4_in_joint_3, link_4->inertial->origin);
  EXPECT_POSE(joint_3_to_link_4_in_joint_3, link_4->visual->origin);
  EXPECT_POSE(joint_3_to_link_4_in_joint_3, link_4->collision->origin);
}


TEST(Pose, pose_collision)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_COLLISION), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_collision", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d expected_collision_pose{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};

  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
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

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d model_to_frame_in_model{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};
  const gz::math::Pose3d model_to_link_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d frame_to_link_in_frame =
    model_to_frame_in_model.Inverse() * model_to_link_in_model;
  const gz::math::Pose3d frame_to_collision_in_frame{0.2, 0.4, 0.8, 0.2, 0.3, 0.4};
  const gz::math::Pose3d link_to_collision_in_link =
    frame_to_link_in_frame.Inverse() * frame_to_collision_in_frame;

  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
  EXPECT_POSE(link_to_collision_in_link, link->collision->origin);
}

TEST(Pose, pose_inertial)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_INERTIAL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_inertial", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d expected_inertial_pose{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};

  EXPECT_POSE(expected_inertial_pose, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
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

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d model_to_frame_in_model{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};
  const gz::math::Pose3d model_to_link_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d frame_to_link_in_frame =
    model_to_frame_in_model.Inverse() * model_to_link_in_model;
  const gz::math::Pose3d frame_to_inertial_in_frame{0.2, 0.4, 0.8, 0.2, 0.3, 0.4};
  const gz::math::Pose3d link_to_inertial_in_link =
    frame_to_link_in_frame.Inverse() * frame_to_inertial_in_frame;

  EXPECT_POSE(link_to_inertial_in_link, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
}

TEST(Pose, pose_joint)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_JOINT), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_joint", model->getName());

  urdf::JointConstSharedPtr joint = model->joints_.begin()->second;
  ASSERT_NE(nullptr, joint);
  urdf::LinkConstSharedPtr child_link = model->getLink(joint->child_link_name);
  ASSERT_NE(nullptr, child_link);

  // In URDF joint is in parent link frame
  // The child link in URDF lives in the joint frame
  const gz::math::Pose3d model_to_parent_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d model_to_child_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d child_to_joint_in_child{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};

  const gz::math::Pose3d parent_to_child_in_parent =
    model_to_parent_in_model.Inverse() * model_to_child_in_model;
  const gz::math::Pose3d parent_to_joint_in_parent =
    parent_to_child_in_parent * child_to_joint_in_child;
  const gz::math::Pose3d joint_to_child_in_joint = child_to_joint_in_child.Inverse();

  EXPECT_POSE(parent_to_joint_in_parent, joint->parent_to_joint_origin_transform);

  // URDF link C++ structure does not have an origin - instead the pose of the
  // link should be added to the visual, collision, and inertial members.
  EXPECT_POSE(joint_to_child_in_joint, child_link->inertial->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->visual->origin);
  EXPECT_POSE(joint_to_child_in_joint, child_link->collision->origin);
}

TEST(Pose, pose_joint_all)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_JOINT_ALL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_joint_all", model->getName());

  ASSERT_EQ(2u, model->links_.size());
  ASSERT_EQ(1u, model->joints_.size());

  urdf::LinkConstSharedPtr link_1 = model->getLink("link_1");
  ASSERT_NE(nullptr, link_1);
  urdf::LinkConstSharedPtr link_2 = model->getLink("link_2");
  ASSERT_NE(nullptr, link_2);

  urdf::JointConstSharedPtr joint = model->getJoint("joint");
  ASSERT_NE(nullptr, joint);

  const gz::math::Pose3d model_to_link_1_in_model{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const gz::math::Pose3d model_to_link_2_in_model{0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  const gz::math::Pose3d link_2_to_joint_in_link_2{0.9, 1.0, 1.1, 1.2, 1.3, 1.4};

  const gz::math::Pose3d model_to_joint_in_model =
    model_to_link_2_in_model * link_2_to_joint_in_link_2;

  const gz::math::Pose3d link_1_to_joint_in_link_1 =
    model_to_link_1_in_model.Inverse() * model_to_joint_in_model;
  const gz::math::Pose3d joint_to_link_2_in_joint =
    model_to_joint_in_model.Inverse() * model_to_link_2_in_model;

  const gz::math::Pose3d link_1_to_visual_in_link_1{0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const gz::math::Pose3d link_1_to_collision_in_link_1{0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  const gz::math::Pose3d link_1_to_inertial_in_link_1{0.4, 0.5, 0.6, 0.7, 0.8, 0.9};

  const gz::math::Pose3d link_2_to_visual_in_link_2{0.6, 0.7, 0.8, 0.9, 1.0, 1.1};
  const gz::math::Pose3d link_2_to_collision_in_link_2{0.7, 0.8, 0.9, 1.0, 1.1, 1.2};
  const gz::math::Pose3d link_2_to_inertial_in_link_2{0.8, 0.9, 1.0, 1.1, 1.2, 1.3};

  const gz::math::Pose3d joint_to_visual_in_joint =
    joint_to_link_2_in_joint * link_2_to_visual_in_link_2;
  const gz::math::Pose3d joint_to_collision_in_joint =
    joint_to_link_2_in_joint * link_2_to_collision_in_link_2;
  const gz::math::Pose3d joint_to_inertial_in_joint =
    joint_to_link_2_in_joint * link_2_to_inertial_in_link_2;

  EXPECT_POSE(link_1_to_visual_in_link_1, link_1->visual->origin);
  EXPECT_POSE(link_1_to_collision_in_link_1, link_1->collision->origin);
  EXPECT_POSE(link_1_to_inertial_in_link_1, link_1->inertial->origin);

  EXPECT_POSE(link_1_to_joint_in_link_1, joint->parent_to_joint_origin_transform);

  EXPECT_POSE(joint_to_visual_in_joint, link_2->visual->origin);
  EXPECT_POSE(joint_to_collision_in_joint, link_2->collision->origin);
  EXPECT_POSE(joint_to_inertial_in_joint, link_2->inertial->origin);
}

TEST(Pose, pose_joint_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_JOINT_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_joint_in_frame", model->getName());

  urdf::JointConstSharedPtr joint = model->joints_.begin()->second;
  ASSERT_NE(nullptr, joint);
  urdf::LinkConstSharedPtr child_link = model->getLink(joint->child_link_name);
  ASSERT_NE(nullptr, child_link);

  // In URDF joint is in parent link frame
  // The child link in URDF lives in the joint frame
  const gz::math::Pose3d model_to_parent_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d model_to_child_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d model_to_frame_in_model{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};
  const gz::math::Pose3d frame_to_joint_in_frame{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};

  const gz::math::Pose3d model_to_joint_in_model =
    model_to_frame_in_model * frame_to_joint_in_frame;
  const gz::math::Pose3d parent_to_joint_in_parent =
    model_to_parent_in_model.Inverse() * model_to_joint_in_model;
  const gz::math::Pose3d joint_to_child_in_joint =
    model_to_joint_in_model.Inverse() * model_to_child_in_model;

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

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  // URDF link C++ structure does not have an origin - root link members should be unaffected
  // by root link pose
  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
}

TEST(Pose, pose_link_all)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_LINK_ALL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_link_all", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  // URDF link C++ structure does not have an origin - root link members should be unaffected
  // by root link pose
  const gz::math::Pose3d link_to_inertial_in_link{0.05, 0.1, 0.2, 0.4, 0.5, 0.6};
  const gz::math::Pose3d link_to_collision_in_link{0.04, 0.8, 0.16, 0.3, 0.4, 0.5};
  const gz::math::Pose3d link_to_visual_in_link{0.03, 0.6, 0.12, 0.2, 0.3, 0.4};

  EXPECT_POSE(link_to_inertial_in_link, link->inertial->origin);
  EXPECT_POSE(link_to_visual_in_link, link->visual->origin);
  EXPECT_POSE(link_to_collision_in_link, link->collision->origin);
}

TEST(Pose, pose_link_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_LINK_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_link_in_frame", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  // URDF link C++ structure does not have an origin - root link members should be unaffected
  // by root link pose
  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
}

TEST(Pose, pose_model)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_MODEL), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
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

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d expected_visual_pose{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};

  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(expected_visual_pose, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
}

TEST(Pose, pose_visual_in_frame)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_POSE_VISUAL_IN_FRAME), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("pose_visual_in_frame", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const gz::math::Pose3d model_to_frame_in_model{0.05, 0.1, 0.2, 0.1, 0.2, 0.3};
  const gz::math::Pose3d model_to_link_in_model{0, 0, 0, 0, 0, 0};
  const gz::math::Pose3d frame_to_link_in_frame =
    model_to_frame_in_model.Inverse() * model_to_link_in_model;
  const gz::math::Pose3d frame_to_visual_in_frame{0.2, 0.4, 0.8, 0.2, 0.3, 0.4};
  const gz::math::Pose3d link_to_visual_in_link =
    frame_to_link_in_frame.Inverse() * frame_to_visual_in_frame;

  EXPECT_POSE(gz::math::Pose3d::Zero, link->inertial->origin);
  EXPECT_POSE(link_to_visual_in_link, link->visual->origin);
  EXPECT_POSE(gz::math::Pose3d::Zero, link->collision->origin);
}
