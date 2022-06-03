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

#ifndef SDFORMAT_URDF__SDFORMAT_URDF_HPP_
#define SDFORMAT_URDF__SDFORMAT_URDF_HPP_

#include <urdf_world/types.h>
#include <urdf_model/types.h>

#include <string>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Types.hh>

#include "sdformat_urdf/visibility_control.hpp"

namespace sdformat_urdf
{
/// \brief Parse an SDFormat XML string and return URDF C++ structures
SDFORMAT_URDF_PUBLIC
urdf::ModelInterfaceSharedPtr
parse(const std::string & data, sdf::Errors & errors);

/// \brief Convert SDFormat C++ structures to URDF C++ structures
SDFORMAT_URDF_PUBLIC
urdf::ModelInterfaceSharedPtr
sdf_to_urdf(const sdf::Root & sdf_dom, sdf::Errors & errors);

/// \brief Convert SDFormat Model to URDF Model
SDFORMAT_URDF_PUBLIC
urdf::ModelInterfaceSharedPtr
convert_model(const sdf::Model & sdf_model, sdf::Errors & errors);
}  // namespace sdformat_urdf

#endif  // SDFORMAT_URDF__SDFORMAT_URDF_HPP_
