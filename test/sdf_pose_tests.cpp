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
#include <sdf/sdf.hh>
#include <sdformat_urdf/sdformat_urdf.hpp>

#include "sdf_paths.hpp"


TEST(Pose, pose_link)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(POSE_LINK_PATH_TO_SDF), errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_TRUE(model);

  EXPECT_EQ(1u, model->links_.size());
  EXPECT_EQ(0u, model->joints_.size());
  EXPECT_EQ(0u, model->materials_.size());
  EXPECT_EQ("pose_link", model->getName());
  EXPECT_EQ(model->getRoot(), model->getLink("pose_link_link"));
}
