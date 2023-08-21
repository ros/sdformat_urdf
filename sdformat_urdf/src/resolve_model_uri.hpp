// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RESOLVE_MODEL_URI_HPP_
#define RESOLVE_MODEL_URI_HPP_

#include <string.h>
#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
#include <iostream>
#include "sdformat_urdf/visibility_control.hpp"

namespace sdformat_urdf
{

namespace
{
/// \brief Get the list of available models
SDFORMAT_URDF_LOCAL
std::unordered_map<std::string, std::string> gz_models()
{
  namespace fs = std::filesystem;
  std::unordered_map<std::string, std::string> models;

  // there seem to be many possible environment variables to get models?
  // https://github.com/gazebosim/gz-sim/pull/172
  // no idea how they should be ordered
  for (auto env : {
        "IGN_GAZEBO_RESOURCE_PATH",
        "GZ_SIM_RESOURCE_PATH",
        "GAZEBO_MODEL_PATH",
        "SDF_PATH"})
  {
    const std::string paths{std::getenv(env)};
    if(paths.empty())
      continue;

    std::istringstream iss{paths};
    std::string path;

    while (std::getline(iss, path, ':')) {
      const fs::path modelDirectory(path);
      if (fs::exists(modelDirectory)) {
        for (const auto & model : fs::directory_iterator(modelDirectory)) {
          if (model.is_directory() && fs::exists(model.path() / "model.sdf")) {
            models[model.path().filename()] = model.path();
          }
        }
      }
    }
  }
  return models;
}
}  // namespace


/// \brief Get a SDF-formatted mesh URI and returns the absolute path to it
SDFORMAT_URDF_PUBLIC
std::string
resolveURI(const std::string &uri)
{
  // URDF is fine with package:// or file://
  const auto sep{uri.find("://")};
  if(uri.substr(0, sep) != "model")
    return uri;

  const auto models{gz_models()};
  const auto model_path{uri.substr(sep+3, uri.npos)};
  const auto slash{model_path.find('/')};
  if (slash == model_path.npos) {
    return uri;
  }
  const auto path{models.find(model_path.substr(0, slash))};
  if(path == models.end())
    return uri;

  const auto relativePath{model_path.substr(slash, model_path.npos)};

  return "file://" + path->second + relativePath;
}
}  // namespace sdformat_urdf


#endif  // RESOLVE_MODEL_URI_HPP_
