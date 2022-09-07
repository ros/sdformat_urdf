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

#include <gz/math/Vector4.hh>
#include <sdf/Types.hh>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Material, material_blinn_phong)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_MATERIAL_BLINN_PHONG), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("material_blinn_phong", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  urdf::VisualConstSharedPtr visual = link->visual;
  ASSERT_NE(nullptr, visual);

  const gz::math::Vector4d ambient{0.3, 0, 0, 1};
  const gz::math::Vector4d diffuse{0, 0.3, 0, 1};
  const gz::math::Vector4d expected_color =
    0.4 * ambient + 0.8 * diffuse;

  EXPECT_EQ(link->name + visual->name, visual->material->name);
  EXPECT_EQ("", visual->material->texture_filename);
  EXPECT_FLOAT_EQ(expected_color[0], visual->material->color.r);
  EXPECT_FLOAT_EQ(expected_color[1], visual->material->color.g);
  EXPECT_FLOAT_EQ(expected_color[2], visual->material->color.b);
  EXPECT_FLOAT_EQ(expected_color[3], visual->material->color.a);
}

TEST(Material, material_dynamic_lights)
{
  GTEST_SKIP() << "https://github.com/osrf/sdformat/issues/384";

  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_MATERIAL_DYNAMIC_LIGHTS), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("material_dynamic_lights", model->getName());

  // URDF doesn't support toggling dynamic lights, so a warning is omitted to the console
}
