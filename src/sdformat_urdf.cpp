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

#include "sdformat_urdf/sdformat_urdf.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <sdf/sdf.hh>
#include <urdf_world/types.h>
#include <urdf_model/model.h>

namespace sdformat_urdf {
/// \brief Convert SDFormat Link to URDF Link
urdf::LinkSharedPtr
convert_link(const sdf::Link & sdf_link, sdf::Errors & errors);

/// \brief Convert SDFormat Joint to URDF Joint
urdf::JointSharedPtr
convert_joint(const sdf::Joint & sdf_joint, sdf::Errors & errors);

urdf::Pose
convert_pose(const ignition::math::Pose3d & sdf_pose);
}  // namespace sdformat_urdf

urdf::ModelInterfaceSharedPtr
sdformat_urdf::parse(const std::string & data, sdf::Errors & errors)
{
  auto sdf_dom = std::make_shared<sdf::Root>();
  errors = sdf_dom->LoadSdfString(data);
  if (sdf_dom) {
    return sdformat_urdf::sdf_to_urdf(*sdf_dom, errors);
  }
  return nullptr;
}


urdf::ModelInterfaceSharedPtr
sdformat_urdf::sdf_to_urdf(const sdf::Root & sdf_dom, sdf::Errors & errors)
{
  if (sdf_dom.WorldCount()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "SDFormat xml has a world; but only a single model is supported");
    return nullptr;
  }
  if (1u != sdf_dom.ModelCount()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "SDFormat xml has multiple models; but only a single model is supported");
    return nullptr;
  }

  return convert_model(*sdf_dom.ModelByIndex(0), errors);
}

urdf::ModelInterfaceSharedPtr
sdformat_urdf::convert_model(const sdf::Model & sdf_model, sdf::Errors & errors)
{
  urdf::ModelInterfaceSharedPtr urdf_model = std::make_shared<urdf::ModelInterface>();

  // copy name
  urdf_model->name_ = sdf_model.Name();

  // create matching links
  for (uint64_t l = 0; l < sdf_model.LinkCount(); ++l) {
    const sdf::Link * sdf_link = sdf_model.LinkByIndex(l);

    if (!sdf_link) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get sdf link");
      return nullptr;
    }

    auto pair = urdf_model->links_.emplace(sdf_link->Name(), convert_link(*sdf_link, errors));

    if (!pair.second) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to create element in links map");
      return nullptr;
    }

    const std::shared_ptr<urdf::Link> & urdf_link = pair.first->second;

    if (!urdf_link) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to convert sdf::link [" + sdf_link->Name() + "] to urdf::Link");
      return nullptr;
    }
  }

  // sdf canonical link -> urdf root link
  const sdf::Link * sdf_canonical_link = sdf_model.CanonicalLink();
  if (!sdf_canonical_link) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Failed to get sdf canonical link");
    return nullptr;
  }
  auto iter = urdf_model->links_.find(sdf_canonical_link->Name());
  if (iter == urdf_model->links_.end()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Failed to find sdf canonical link [" + sdf_canonical_link->Name() + "]");
    return nullptr;
  }
  urdf_model->root_link_ = iter->second;

  // create matching joints
  for (uint64_t j = 0; j < sdf_model.JointCount(); ++j) {
    const sdf::Joint * sdf_joint = sdf_model.JointByIndex(j);

    if (!sdf_joint) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get sdf joint");
      return nullptr;
    }

    auto pair = urdf_model->joints_.emplace(sdf_joint->Name(), convert_joint(*sdf_joint, errors));

    if (!pair.second) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to create element in joints map");
      return nullptr;
    }

    const std::shared_ptr<urdf::Joint> & urdf_joint = pair.first->second;
    if (!urdf_joint) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to convert sdf::joint [" + sdf_joint->Name() + "] to urdf::Joint");
      return nullptr;
    }
  }

  // Start with root link (sdf canonical frame) and recurse this function:
  //  
  // TODO what about coordinate frames?
  // Link pose in URDF is relative to link reference frame
  //  Link reference frame is parent joint - joint in which this frame is the child link
  //  Or if root, link reference frame is the model, where the model itself in sdf may be offset
  // Seems like need to build tree of joints and links starting with root link

  std::vector<const sdf::Link *> visited_links_;
  std::vector<const sdf::Link *> link_stack{sdf_canonical_link};
  while (!link_stack.empty()) {
    const sdf::Link * sdf_parent_link = link_stack.back();
    urdf::LinkSharedPtr & urdf_parent_link = urdf_model->links_.at(sdf_parent_link->Name());
    link_stack.pop_back();

    // Check if there is a kinematic loop
    if (visited_links_.end() != std::find(visited_links_.begin(), visited_links_.end(), sdf_parent_link) ) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Not a tree because link [" + sdf_parent_link->Name() + "] was visited twice");
      return nullptr;
    }
    visited_links_.emplace_back(sdf_parent_link);

    sdf::Errors pose_errors;

    // Look for joints attached to this link
    for (uint64_t j = 0; j < sdf_model.JointCount(); ++j) {
      const sdf::Joint * sdf_joint = sdf_model.JointByIndex(j);

      if (!sdf_joint) {
        errors.emplace_back(
          sdf::ErrorCode::STRING_READ,
          "Failed to get sdf joint");
        return nullptr;
      }

      if (sdf_joint->ParentLinkName() == sdf_parent_link->Name()) {
        urdf::JointSharedPtr & urdf_joint = urdf_model->joints_.at(sdf_joint->Name());
        // Append to child joints
        urdf_parent_link->child_joints.push_back(urdf_joint);

        // Set the joint origin to the transform from the joint to the parent link
        ignition::math::Pose3d joint_pose;
        pose_errors = sdf_joint->SemanticPose().Resolve(joint_pose, sdf_parent_link->Name());
        if (!pose_errors.empty()) {
          errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Failed to get transfrom from joint [" + sdf_joint->Name()
            + "] to link [" + sdf_parent_link->Name() + "]");
          return nullptr;
        }
        urdf_joint->parent_to_joint_origin_transform = convert_pose(joint_pose);

        // get child link
        const std::string & child_link_name = urdf_joint->child_link_name;
        const sdf::Link * sdf_child_link = sdf_model.LinkByName(child_link_name);
        if (!sdf_child_link) {
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Failed to get child link [" + child_link_name + "]");
          return nullptr;
        }
        urdf::LinkSharedPtr & urdf_child_link = urdf_model->links_.at(child_link_name);

        // child link is attached to parent joint
        urdf_child_link->parent_joint = urdf_joint;

        // parent link has reference to child link
        urdf_parent_link->child_links.push_back(urdf_child_link);

        // Explore this child link later
        link_stack.push_back(sdf_child_link);
      }
    }
  }

  return urdf_model;
}

urdf::LinkSharedPtr
sdformat_urdf::convert_link(const sdf::Link & sdf_link, sdf::Errors & errors)
{
  urdf::LinkSharedPtr urdf_link = std::make_shared<urdf::Link>();

  urdf_link->name = sdf_link.Name();

  urdf_link->inertial = std::make_shared<urdf::Inertial>();
  if (!urdf_link->inertial) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Failed to create inertial for link [" + sdf_link.Name() + "]");
    return nullptr;
  }
  urdf_link->inertial->mass = sdf_link.Inertial().MassMatrix().Mass();

  // TODO(sloretz) inertial pose
  // TODO(sloretz) ixx, ixy, ixz, iyy, iyz, izz

  // TODO(sloretz) visual

  // TODO(sloretz) collision

  return urdf_link;
}

urdf::JointSharedPtr
sdformat_urdf::convert_joint(const sdf::Joint & sdf_joint, sdf::Errors & errors)
{
  urdf::JointSharedPtr urdf_joint = std::make_shared<urdf::Joint>();

  urdf_joint->name = sdf_joint.Name();

  switch (sdf_joint.Type()) {
    case sdf::JointType::CONTINUOUS:
      urdf_joint->type = urdf::Joint::CONTINUOUS;
      break;
    case sdf::JointType::REVOLUTE:
      urdf_joint->type = urdf::Joint::REVOLUTE;
      break;
    case sdf::JointType::FIXED:
      urdf_joint->type = urdf::Joint::FIXED;
      break;
    case sdf::JointType::PRISMATIC:
      urdf_joint->type = urdf::Joint::PRISMATIC;
      break;
    case sdf::JointType::INVALID:
      // fall through
    case sdf::JointType::BALL:
      // fall through
    case sdf::JointType::GEARBOX:
      // fall through
    case sdf::JointType::REVOLUTE2:
      // fall through
    case sdf::JointType::SCREW:
      // fall through
    case sdf::JointType::UNIVERSAL:
      // fall through
    default:
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Unsupported joint type on joint [" + sdf_joint.Name() + "]");
      return nullptr;
  };

  const sdf::JointAxis * sdf_axis = sdf_joint.Axis(0);
  urdf_joint->axis.x = sdf_axis->Xyz().X();
  urdf_joint->axis.y = sdf_axis->Xyz().Y();
  urdf_joint->axis.z = sdf_axis->Xyz().Z();

  urdf_joint->child_link_name = sdf_joint.ChildLinkName();
  urdf_joint->parent_link_name = sdf_joint.ParentLinkName();

  return urdf_joint;
}

urdf::Pose
sdformat_urdf::convert_pose(const ignition::math::Pose3d & sdf_pose)
{
  urdf::Pose pose;
  pose.position.x = sdf_pose.Pos().X();
  pose.position.y = sdf_pose.Pos().Y();
  pose.position.z = sdf_pose.Pos().Z();

  pose.rotation.x = sdf_pose.Rot().X();
  pose.rotation.y = sdf_pose.Rot().Y();
  pose.rotation.z = sdf_pose.Rot().Z();
  pose.rotation.w = sdf_pose.Rot().W();

  return pose;
}
