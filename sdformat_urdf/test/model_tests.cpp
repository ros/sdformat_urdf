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

#include <sdf/Types.hh>

#include "sdf_paths.hpp"
#include "test_tools.hpp"

TEST(Model, model_two_models)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_MODEL_TWO_MODELS), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Model, model_zero_models)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_MODEL_ZERO_MODELS), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}
