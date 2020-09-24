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

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include <urdf_world/types.h>
#include <urdf_model/model.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "sdformat_urdf/sdformat_urdf.hpp"

namespace sdformat_urdf
{
/// \brief Convert SDFormat Link to URDF Link
urdf::LinkSharedPtr
convert_link(const sdf::Link & sdf_link, sdf::Errors & errors);

/// \brief Convert SDFormat Joint to URDF Joint
urdf::JointSharedPtr
convert_joint(const sdf::Joint & sdf_joint, sdf::Errors & errors);

urdf::Pose
convert_pose(const ignition::math::Pose3d & sdf_pose);

urdf::GeometrySharedPtr
convert_geometry(const sdf::Geometry & sdf_geometry, sdf::Errors & errors);
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

  // Assume sdf canonical link is urdf root link
  // TODO(osrf/sdformat#273) In future use API for getting kinematic root link
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

  // Start with root link and resolve the poses one joint at a time depth-first
  std::vector<const sdf::Link *> visited_links_;
  std::vector<const sdf::Link *> link_stack{sdf_canonical_link};
  while (!link_stack.empty()) {
    const sdf::Link * sdf_parent_link = link_stack.back();
    urdf::LinkSharedPtr & urdf_parent_link = urdf_model->links_.at(sdf_parent_link->Name());
    link_stack.pop_back();

    // Check if there is a kinematic loop
    if (visited_links_.end() !=
      std::find(visited_links_.begin(), visited_links_.end(), sdf_parent_link))
    {
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
            "Failed to get transfrom from joint [" + sdf_joint->Name() +
            "] to link [" + sdf_parent_link->Name() + "]");
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

  // Link pose is by default relative to the model
  ignition::math::Pose3d link_pose;
  {
    sdf::Errors pose_errors = sdf_link.SemanticPose().Resolve(link_pose);
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transform of link [" + sdf_link.Name() + "]");
      return nullptr;
    }
  }

  const ignition::math::Inertiald sdf_inertia = sdf_link.Inertial();
  urdf_link->inertial = std::make_shared<urdf::Inertial>();
  if (!urdf_link->inertial) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Failed to create inertial for link [" + sdf_link.Name() + "]");
    return nullptr;
  }
  urdf_link->inertial->mass = sdf_inertia.MassMatrix().Mass();
  // URDF doesn't have link pose concept, so add SDF link pose to inertial
  urdf_link->inertial->origin = convert_pose(link_pose + sdf_inertia.Pose());
  urdf_link->inertial->ixx = sdf_inertia.MassMatrix().Ixx();
  urdf_link->inertial->ixy = sdf_inertia.MassMatrix().Ixy();
  urdf_link->inertial->ixz = sdf_inertia.MassMatrix().Ixz();
  urdf_link->inertial->iyy = sdf_inertia.MassMatrix().Iyy();
  urdf_link->inertial->iyz = sdf_inertia.MassMatrix().Iyz();
  urdf_link->inertial->izz = sdf_inertia.MassMatrix().Izz();

  for (uint64_t vi = 0; vi < sdf_link.VisualCount(); ++vi) {
    const sdf::Visual * sdf_visual = sdf_link.VisualByIndex(vi);
    if (!sdf_visual) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get visual on lilnk [" + sdf_link.Name() + "]");
      return nullptr;
    }

    auto urdf_visual = std::make_shared<urdf::Visual>();

    urdf_visual->name = sdf_visual->Name();

    // URDF visual is relative to link origin
    ignition::math::Pose3d visual_pose;
    sdf::Errors pose_errors = sdf_visual->SemanticPose().Resolve(visual_pose, sdf_link.Name());
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transfrom from visual [" + sdf_visual->Name() +
        "] to link [" + sdf_link.Name() + "]");
      return nullptr;
    }
    // URDF doesn't have link pose concept, so add SDF link pose to visual
    urdf_visual->origin = convert_pose(link_pose + visual_pose);

    urdf_visual->geometry = convert_geometry(*sdf_visual->Geom(), errors);
    if (!urdf_visual->geometry) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to convert geometry on visual [" + sdf_visual->Name() + "]");
      return nullptr;
    }

    const sdf::Material * sdf_material = sdf_visual->Material();
    if (sdf_material) {
      // TODO(sloretz) textures
      // TODO(sloretz) error if any file names we can't resolve are given
      auto urdf_material = std::make_shared<urdf::Material>();
      // sdf materials don't have names, so assign it the visual's name and hope that's unique
      urdf_material->name = sdf_visual->Name();
      // Color support is pretty limited in urdf, just take the ambient (color with no light)
      urdf_material->color.r = sdf_material->Ambient().R();
      urdf_material->color.g = sdf_material->Ambient().G();
      urdf_material->color.b = sdf_material->Ambient().B();
      urdf_material->color.a = sdf_material->Ambient().A();

      urdf_visual->material = urdf_material;
    }

    if (0u == vi) {
      urdf_link->visual = urdf_visual;
    }
    urdf_link->visual_array.push_back(urdf_visual);
  }

  for (uint64_t vi = 0; vi < sdf_link.CollisionCount(); ++vi) {
    const sdf::Collision * sdf_collision = sdf_link.CollisionByIndex(vi);
    if (!sdf_collision) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get collision on lilnk [" + sdf_link.Name() + "]");
      return nullptr;
    }

    auto urdf_collision = std::make_shared<urdf::Collision>();

    urdf_collision->name = sdf_collision->Name();

    // URDF collision is relative to link origin
    ignition::math::Pose3d collision_pose;
    sdf::Errors pose_errors =
      sdf_collision->SemanticPose().Resolve(collision_pose, sdf_link.Name());
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transfrom from collision [" + sdf_collision->Name() +
        "] to link [" + sdf_link.Name() + "]");
      return nullptr;
    }
    // URDF doesn't have link pose concept, so add SDF link pose to collision
    urdf_collision->origin = convert_pose(link_pose + collision_pose);

    urdf_collision->geometry = convert_geometry(*sdf_collision->Geom(), errors);
    if (!urdf_collision->geometry) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to convert geometry on collision [" + sdf_collision->Name() + "]");
      return nullptr;
    }

    if (0u == vi) {
      urdf_link->collision = urdf_collision;
    }
    urdf_link->collision_array.push_back(urdf_collision);
  }

  return urdf_link;
}

urdf::JointSharedPtr
sdformat_urdf::convert_joint(const sdf::Joint & sdf_joint, sdf::Errors & errors)
{
  urdf::JointSharedPtr urdf_joint = std::make_shared<urdf::Joint>();

  urdf_joint->name = sdf_joint.Name();

  size_t num_axes = 0;

  switch (sdf_joint.Type()) {
    case sdf::JointType::CONTINUOUS:
      urdf_joint->type = urdf::Joint::CONTINUOUS;
      num_axes = 1;
      break;
    case sdf::JointType::REVOLUTE:
      urdf_joint->type = urdf::Joint::REVOLUTE;
      num_axes = 1;
      break;
    case sdf::JointType::FIXED:
      urdf_joint->type = urdf::Joint::FIXED;
      num_axes = 0;
      break;
    case sdf::JointType::PRISMATIC:
      urdf_joint->type = urdf::Joint::PRISMATIC;
      num_axes = 1;
      break;
    case sdf::JointType::INVALID:     // Unsupported: fall through to default
    case sdf::JointType::BALL:        //  |
    case sdf::JointType::GEARBOX:     //  |
    case sdf::JointType::REVOLUTE2:   //  |
    case sdf::JointType::SCREW:       //  |
    case sdf::JointType::UNIVERSAL:   //  V
    default:
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Unsupported joint type on joint [" + sdf_joint.Name() + "]");
      return nullptr;
  }

  // Supported joints have at most one axis
  if (1 == num_axes) {
    const sdf::JointAxis * sdf_axis = sdf_joint.Axis(0);

    // URDF expects axis to be expressed in the joint frame
    ignition::math::Vector3d axis_xyz;
    sdf::Errors axis_errors = sdf_axis->ResolveXyz(axis_xyz, sdf_joint.Name());
    if (!axis_errors.empty()) {
      errors.insert(errors.end(), axis_errors.begin(), axis_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transfrom of joint axis in frame [" + sdf_axis->XyzExpressedIn() +
        "] to joint [" + sdf_joint.Name() + "]");
      return nullptr;
    }

    urdf_joint->axis.x = axis_xyz.X();
    urdf_joint->axis.y = axis_xyz.Y();
    urdf_joint->axis.z = axis_xyz.Z();
  }

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

urdf::GeometrySharedPtr
sdformat_urdf::convert_geometry(const sdf::Geometry & sdf_geometry, sdf::Errors & errors)
{
  if (sdf_geometry.BoxShape()) {
    const sdf::Box * box = sdf_geometry.BoxShape();
    auto urdf_box = std::make_shared<urdf::Box>();
    urdf_box->dim.x = box->Size().X();
    urdf_box->dim.y = box->Size().Y();
    urdf_box->dim.z = box->Size().Z();
    return urdf_box;
  } else if (sdf_geometry.CylinderShape()) {
    const sdf::Cylinder * cylinder = sdf_geometry.CylinderShape();
    auto urdf_cylinder = std::make_shared<urdf::Cylinder>();
    urdf_cylinder->length = cylinder->Length();
    urdf_cylinder->radius = cylinder->Radius();
    return urdf_cylinder;
  } else if (sdf_geometry.SphereShape()) {
    const sdf::Sphere * sphere = sdf_geometry.SphereShape();
    auto urdf_sphere = std::make_shared<urdf::Sphere>();
    urdf_sphere->radius = sphere->Radius();
    return urdf_sphere;
  } else if (sdf_geometry.PlaneShape()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Plane geometry cannot be converted to urdf C++ structures");
    return nullptr;
  } else if (sdf_geometry.MeshShape()) {
    const std::string & uri = sdf_geometry.MeshShape()->Uri();
    auto urdf_mesh = std::make_shared<urdf::Mesh>();
    // The only example in ROS that I've found using urdf_mesh->filename is
    // the RobotModel plugin in RViz. This plugin uses resource retriever to
    // resolve the filename - which may be a URI - to the mesh resource.
    // Pass it here unmodified, ignoring that SDFormat relative paths may not
    // be resolvable this way.
    urdf_mesh->filename = uri;

    urdf_mesh->scale.x = sdf_geometry.MeshShape()->Scale().X();
    urdf_mesh->scale.y = sdf_geometry.MeshShape()->Scale().Y();
    urdf_mesh->scale.z = sdf_geometry.MeshShape()->Scale().Z();
    return urdf_mesh;
  }

  errors.emplace_back(
    sdf::ErrorCode::STRING_READ,
    "Unknown geometry shape");
  return nullptr;
}
