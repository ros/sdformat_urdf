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

TEST(Pose, pose_collision)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(POSE_COLLISION_PATH_TO_SDF), errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_TRUE(model);
  EXPECT_EQ("pose_collision", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d expected_collision_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_other_pose(0, 0, 0, 0, 0, 0);

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_other_pose, link->visual->origin);
  EXPECT_POSE(expected_collision_pose, link->collision->origin);
}

TEST(Pose, pose_link)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(POSE_LINK_PATH_TO_SDF), errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_TRUE(model);
  EXPECT_EQ("pose_link", model->getName());

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

TEST(Pose, pose_visual)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(POSE_VISUAL_PATH_TO_SDF), errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_TRUE(model);
  EXPECT_EQ("pose_visual", model->getName());

  ASSERT_EQ(1u, model->links_.size());
  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);

  const ignition::math::Pose3d expected_visual_pose(0.05, 0.1, 0.2, 0.1, 0.2, 0.3);
  const ignition::math::Pose3d expected_other_pose(0, 0, 0, 0, 0, 0);

  EXPECT_POSE(expected_other_pose, link->inertial->origin);
  EXPECT_POSE(expected_visual_pose, link->visual->origin);
  EXPECT_POSE(expected_other_pose, link->collision->origin);
}
