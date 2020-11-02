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

#include <rcutils/logging_macros.h>
#include <tinyxml2.h>
#include <urdf_parser_plugin/parser.h>

#include <limits>
#include <string>

#include "sdformat_urdf/sdformat_urdf.hpp"

namespace sdformat_urdf
{
class SDFormatURDFParser final : public urdf::URDFParser
{
public:
  SDFormatURDFParser() = default;
  ~SDFormatURDFParser() = default;

  urdf::ModelInterfaceSharedPtr
  parse(const std::string & data) override
  {
    sdf::Errors errors;
    urdf::ModelInterfaceSharedPtr urdf_cpp = sdformat_urdf::parse(data, errors);

    if (urdf_cpp) {
      return urdf_cpp;
    }

    for (const sdf::Error & error : errors) {
      RCUTILS_LOG_ERROR_NAMED("sdformat_urdf", "%s", error.Message().c_str());
    }
    if (errors.empty()) {
      RCUTILS_LOG_ERROR_NAMED("sdformat_urdf", "Failed to parse but no errors reported");
    }
    return nullptr;
  }

  size_t
  might_handle(const std::string & data) override
  {
    tinyxml2::XMLDocument doc;
    const tinyxml2::XMLError error = doc.Parse(data.c_str());
    if (error == tinyxml2::XML_SUCCESS) {
      // Since it's an XML document it must have `<sdf>` as the first tag
      const tinyxml2::XMLElement * root = doc.RootElement();
      if (std::string("sdf") != root->Name()) {
        return std::numeric_limits<size_t>::max();
      }
    }

    // Possiblities:
    //  1) It is not an XML based robot description
    //  2) It is an XML based robot description, but there's an XML syntax error
    //  3) It is an SDFormat XML with correct XML syntax
    return data.find("<sdf");
  }
};
}  // namespace sdformat_urdf

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(sdformat_urdf::SDFormatURDFParser, urdf::URDFParser)
