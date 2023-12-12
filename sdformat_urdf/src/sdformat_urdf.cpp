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
#include <urdf_world/types.h>
#include <urdf_model/model.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <sdf/Error.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Visual.hh>

#include "sdformat_urdf/sdformat_urdf.hpp"

namespace sdformat_urdf
{
/// \brief Convert SDFormat Link to URDF Link
/// \param[in] sdf_link the SDFormat link instance to convert
/// \param[in] joint_frame the name of the only joint who has this link as a child, or empty string
///   if no such joint exists.
/// \param[out] errors any errors encountered while trying to convert the link
urdf::LinkSharedPtr
convert_link(const sdf::Link & sdf_link, const std::string & joint_frame, sdf::Errors & errors);

/// \brief Convert SDFormat Joint to URDF Joint
urdf::JointSharedPtr
convert_joint(const sdf::Joint & sdf_joint, sdf::Errors & errors);

urdf::Pose
convert_pose(const gz::math::Pose3d & sdf_pose);

urdf::GeometrySharedPtr
convert_geometry(const sdf::Geometry & sdf_geometry, sdf::Errors & errors);
}  // namespace sdformat_urdf

urdf::ModelInterfaceSharedPtr
sdformat_urdf::parse(const std::string & data, sdf::Errors & errors)
{
  auto sdf_dom = std::make_shared<sdf::Root>();
  errors = sdf_dom->LoadSdfString(data);
  if (errors.empty()) {
    return sdformat_urdf::sdf_to_urdf(*sdf_dom, errors);
  }
  return nullptr;
}

urdf::ModelInterfaceSharedPtr
sdformat_urdf::sdf_to_urdf(const sdf::Root & sdf_dom, sdf::Errors & errors)
{
  if (sdf_dom.WorldCount() > 0u) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "SDFormat xml has a world; but only a single model is supported");
    return nullptr;
  }
  if (nullptr == sdf_dom.Model()) {
    errors.emplace_back(
      sdf::ErrorCode::ELEMENT_MISSING,
      "SDFormat xml has no models; need at least one");
    return nullptr;
  }
  return convert_model(*sdf_dom.Model(), errors);
}

urdf::ModelInterfaceSharedPtr
sdformat_urdf::convert_model(const sdf::Model & sdf_model, sdf::Errors & errors)
{
  urdf::ModelInterfaceSharedPtr urdf_model = std::make_shared<urdf::ModelInterface>();

  // copy name
  urdf_model->name_ = sdf_model.Name();

  // A model's pose is the location of the model in a larger context, like a world or parent model
  // It doesn't make sense in the context of a robot description.
  if ("" != sdf_model.PoseRelativeTo() || gz::math::Pose3d{} != sdf_model.RawPose()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "<model> tags with <pose> are not currently supported by sdformat_urdf");
    return nullptr;
  }

  // create matching links
  for (uint64_t l = 0; l < sdf_model.LinkCount(); ++l) {
    const sdf::Link * sdf_link = sdf_model.LinkByIndex(l);

    if (!sdf_link) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get sdf link");
      return nullptr;
    }

    // URDF link pose is either relative to  __model__ or to a joint
    std::string relative_joint_name{""};
    for (uint64_t j = 0; j < sdf_model.JointCount(); ++j) {
      const sdf::Joint * sdf_joint = sdf_model.JointByIndex(j);
      if (sdf_joint && sdf_joint->ChildLinkName() == sdf_link->Name()) {
        relative_joint_name = sdf_joint->Name();
        break;
      }
    }

    auto pair = urdf_model->links_.emplace(
      sdf_link->Name(),
      convert_link(*sdf_link, relative_joint_name, errors));

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
  std::vector<const sdf::Link *> visited_links;
  std::vector<const sdf::Link *> link_stack{sdf_canonical_link};
  std::vector<const sdf::Joint *> joints_to_visit(sdf_model.JointCount(), nullptr);
  for (size_t j = 0; j < joints_to_visit.size(); ++j) {
    joints_to_visit[j] = sdf_model.JointByIndex(j);
    if (!joints_to_visit[j]) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get sdf joint");
      return nullptr;
    }
  }
  while (!link_stack.empty()) {
    const sdf::Link * sdf_parent_link = link_stack.back();
    urdf::LinkSharedPtr & urdf_parent_link = urdf_model->links_.at(sdf_parent_link->Name());
    link_stack.pop_back();
    visited_links.emplace_back(sdf_parent_link);

    auto joint_iter = joints_to_visit.begin();
    // Fix poses and check for tree structure issues
    while (joint_iter != joints_to_visit.end()) {
      const sdf::Joint * sdf_joint = *joint_iter;
      if (sdf_joint->ParentLinkName() == sdf_parent_link->Name()) {
        // Visited parent link of this joint - don't look at it again
        joint_iter = joints_to_visit.erase(joint_iter);

        urdf::JointSharedPtr & urdf_joint = urdf_model->joints_.at(sdf_joint->Name());
        urdf_parent_link->child_joints.push_back(urdf_joint);

        // SDFormat joint pose is relative to the child sdformat link
        // URDF joint pose is relative to the parent urdf link, which is the frame of the
        // previous urdf joint
        std::string parent_frame_name{sdf_parent_link->Name()};
        if (urdf_parent_link->parent_joint) {
          parent_frame_name = urdf_parent_link->parent_joint->name;
        }

        gz::math::Pose3d joint_pose;
        sdf::Errors pose_errors = sdf_joint->SemanticPose().Resolve(joint_pose, parent_frame_name);
        if (!pose_errors.empty()) {
          errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Failed to get transform from joint [" + sdf_joint->Name() +
            "] to link [" + sdf_parent_link->Name() + "]");
          return nullptr;
        }
        urdf_joint->parent_to_joint_origin_transform = convert_pose(joint_pose);

        const std::string & child_link_name = urdf_joint->child_link_name;
        const sdf::Link * sdf_child_link = sdf_model.LinkByName(child_link_name);
        if (!sdf_child_link) {
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Failed to get child link [" + child_link_name + "]");
          return nullptr;
        }

        // Check for kinematic loops and redundant joints between two links
        if (link_stack.end() != std::find(link_stack.begin(), link_stack.end(), sdf_child_link)) {
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Found kinematic loop at joint [" + sdf_joint->Name() + "]");
          return nullptr;
        }

        urdf::LinkSharedPtr & urdf_child_link = urdf_model->links_.at(child_link_name);

        // child link is attached to parent joint
        urdf_child_link->parent_joint = urdf_joint;

        // Child link keeps weak reference to parent link
        urdf_child_link->setParent(urdf_parent_link);

        // parent link has reference to child link
        urdf_parent_link->child_links.push_back(urdf_child_link);

        // Explore this child link later
        link_stack.push_back(sdf_child_link);
      } else if (sdf_joint->ChildLinkName() == sdf_parent_link->Name()) {
        // Something is wrong here
        if (sdf_parent_link == sdf_canonical_link) {
          // The canonical link can't be a child of a joint
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Canonical link must not be a child of a joint [" + sdf_parent_link->Name() + "]");
        } else {
          // This link must be a child of two joints - kinematic loop :(
          errors.emplace_back(
            sdf::ErrorCode::STRING_READ,
            "Link [" + sdf_parent_link->Name() + "] must only be a child of one joint");
        }
        return nullptr;
      } else {
        // Not interested in this joint yet, look at the next one
        ++joint_iter;
      }
    }
  }
  if (visited_links.size() < sdf_model.LinkCount()) {
    // Must be multiple roots
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Found multiple root links - must only have one link not a child of any joint");
    return nullptr;
  } else if (visited_links.size() > sdf_model.LinkCount()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Algorithm error - visited more links than exist, please file a bug report");
    return nullptr;
  }
  if (!joints_to_visit.empty()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Algorithm error - did not visit all joints, please file a bug report");
    return nullptr;
  }

  return urdf_model;
}

urdf::LinkSharedPtr
sdformat_urdf::convert_link(
  const sdf::Link & sdf_link, const std::string & relative_joint_name, sdf::Errors & errors)
{
  urdf::LinkSharedPtr urdf_link = std::make_shared<urdf::Link>();

  urdf_link->name = sdf_link.Name();

  // joint to link in joint if this is not the root link, else identity
  // The pose of the root link does not matter because there is no equivalent in URDF
  gz::math::Pose3d link_pose;
  if (!relative_joint_name.empty()) {
    // urdf link pose is the location of the joint having it as a child
    sdf::Errors pose_errors = sdf_link.SemanticPose().Resolve(link_pose, relative_joint_name);
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transform of link [" + sdf_link.Name() + "]");
      return nullptr;
    }
  }

  const gz::math::Inertiald sdf_inertia = sdf_link.Inertial();
  urdf_link->inertial = std::make_shared<urdf::Inertial>();
  urdf_link->inertial->mass = sdf_inertia.MassMatrix().Mass();
  // URDF doesn't have link pose concept, so add SDF link pose to inertial
  urdf_link->inertial->origin = convert_pose(link_pose * sdf_inertia.Pose());
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
        "Failed to get visual on link [" + sdf_link.Name() + "]");
      return nullptr;
    }

    auto urdf_visual = std::make_shared<urdf::Visual>();

    urdf_visual->name = sdf_visual->Name();

    // URDF visual is relative to link origin
    gz::math::Pose3d visual_pose;
    sdf::Errors pose_errors = sdf_visual->SemanticPose().Resolve(visual_pose, sdf_link.Name());
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transform from visual [" + sdf_visual->Name() +
        "] to link [" + sdf_link.Name() + "]");
      return nullptr;
    }
    // URDF doesn't have link pose concept, so add SDF link pose to visual
    urdf_visual->origin = convert_pose(link_pose * visual_pose);

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
      // sdf materials don't have names, so assign link's + visual's name and hope it's unique
      urdf_material->name = sdf_link.Name() + sdf_visual->Name();
      // Render to color same as Gazebo's default world ignoring specular
      // color = 0.4 * ambient + 0.8 * specular
      // Color support is pretty limited in urdf, just take the ambient (color with no light)
      urdf_material->color.r =
        0.4 * sdf_material->Ambient().R() + 0.8 * sdf_material->Diffuse().R();
      urdf_material->color.g =
        0.4 * sdf_material->Ambient().G() + 0.8 * sdf_material->Diffuse().G();
      urdf_material->color.b =
        0.4 * sdf_material->Ambient().B() + 0.8 * sdf_material->Diffuse().B();
      urdf_material->color.a =
        0.4 * sdf_material->Ambient().A() + 0.8 * sdf_material->Diffuse().A();

      urdf_visual->material = urdf_material;

      // Warn about unsupported <material> features
      if (!sdf_material->Lighting()) {
        RCUTILS_LOG_WARN_NAMED(
          "sdformat_urdf", "SDFormat visual [%s] has <material><lighting>,"
          " but URDF does not support this", sdf_visual->Name().c_str());
      }
      // TODO(sloretz) warn about materials with ogre scripts, shaders, and pbr
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
        "Failed to get collision on link [" + sdf_link.Name() + "]");
      return nullptr;
    }

    auto urdf_collision = std::make_shared<urdf::Collision>();

    urdf_collision->name = sdf_collision->Name();

    // URDF collision is relative to link origin
    gz::math::Pose3d collision_pose;
    sdf::Errors pose_errors =
      sdf_collision->SemanticPose().Resolve(collision_pose, sdf_link.Name());
    if (!pose_errors.empty()) {
      errors.insert(errors.end(), pose_errors.begin(), pose_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transform from collision [" + sdf_collision->Name() +
        "] to link [" + sdf_link.Name() + "]");
      return nullptr;
    }
    // URDF doesn't have link pose concept, so add SDF link pose to collision
    urdf_collision->origin = convert_pose(link_pose * collision_pose);

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

  // Warn about unsupported <link> features
  if (0u != sdf_link.SensorCount()) {
    RCUTILS_LOG_WARN_NAMED(
      "sdformat_urdf", "SDFormat link [%s] has a <sensor>,"
      " but URDF does not support this", sdf_link.Name().c_str());
  }
  if (0u != sdf_link.LightCount()) {
    RCUTILS_LOG_WARN_NAMED(
      "sdformat_urdf", "SDFormat link [%s] has a <light>,"
      " but URDF does not support this", sdf_link.Name().c_str());
  }

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
    case sdf::JointType::UNIVERSAL:   // Unsupported: fall through to floating
    case sdf::JointType::BALL:        //  Will require custom TF publisher
    case sdf::JointType::GEARBOX:     //  |
    case sdf::JointType::REVOLUTE2:   //  |
    case sdf::JointType::SCREW:       //  V
      urdf_joint->type = urdf::Joint::FLOATING;
      break;
    case sdf::JointType::INVALID:
    default:
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Unsupported joint type on joint [" + sdf_joint.Name() + "]");
      return nullptr;
  }

  if ((urdf::Joint::FIXED != urdf_joint->type) && (urdf::Joint::FLOATING != urdf_joint->type)) {
    // Add axis info for non-fixed and non-floating joints
    const sdf::JointAxis * sdf_axis = sdf_joint.Axis(0);
    if (nullptr == sdf_axis) {
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Axes missing for joint [" + sdf_joint.Name() + "]");
      return nullptr;
    }

    // URDF expects axis to be expressed in the joint frame
    gz::math::Vector3d axis_xyz;
    sdf::Errors axis_errors = sdf_axis->ResolveXyz(axis_xyz, sdf_joint.Name());
    if (!axis_errors.empty()) {
      errors.insert(errors.end(), axis_errors.begin(), axis_errors.end());
      errors.emplace_back(
        sdf::ErrorCode::STRING_READ,
        "Failed to get transform of joint axis in frame [" + sdf_axis->XyzExpressedIn() +
        "] to joint [" + sdf_joint.Name() + "]");
      return nullptr;
    }
    urdf_joint->axis.x = axis_xyz.X();
    urdf_joint->axis.y = axis_xyz.Y();
    urdf_joint->axis.z = axis_xyz.Z();

    // Add dynamics info for non-fixed joints
    urdf_joint->dynamics = std::make_shared<urdf::JointDynamics>();
    urdf_joint->dynamics->damping = sdf_axis->Damping();
    urdf_joint->dynamics->friction = sdf_axis->Friction();

    // Warn about non-default values on unsupported <dynamics> tags
    if (0.0 != sdf_axis->SpringReference()) {
      RCUTILS_LOG_WARN_NAMED(
        "sdformat_urdf", "SDFormat Joint [%s] given non-default value for <spring_reference>,"
        " but URDF does not support this", sdf_joint.Name().c_str());
    }
    if (0.0 != sdf_axis->SpringStiffness()) {
      RCUTILS_LOG_WARN_NAMED(
        "sdformat_urdf", "SDFormat Joint [%s] given non-default value for <spring_stiffness>,"
        " but URDF does not support this", sdf_joint.Name().c_str());
    }

    // Add limits info for non-fixed non-continuous joints
    if (urdf::Joint::CONTINUOUS != urdf_joint->type) {
      urdf_joint->limits = std::make_shared<urdf::JointLimits>();
      urdf_joint->limits->lower = sdf_axis->Lower();
      urdf_joint->limits->upper = sdf_axis->Upper();
      urdf_joint->limits->effort = sdf_axis->Effort();
      urdf_joint->limits->velocity = sdf_axis->MaxVelocity();

      // Warn about non-default values on unsupported <limit> tags
      if (1.0 != sdf_axis->Dissipation()) {
        RCUTILS_LOG_WARN_NAMED(
          "sdformat_urdf", "SDFormat Joint [%s] given non-default value for <dissipation>,"
          " but URDF does not support this", sdf_joint.Name().c_str());
      }
      if (1e8 != sdf_axis->Stiffness()) {
        RCUTILS_LOG_WARN_NAMED(
          "sdformat_urdf", "SDFormat Joint [%s] given non-default value for <stiffness>,"
          " but URDF does not support this", sdf_joint.Name().c_str());
      }
    }
  }

  urdf_joint->child_link_name = sdf_joint.ChildLinkName();
  urdf_joint->parent_link_name = sdf_joint.ParentLinkName();

  return urdf_joint;
}

urdf::Pose
sdformat_urdf::convert_pose(const gz::math::Pose3d & sdf_pose)
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
  } else if (sdf_geometry.PlaneShape()) {
    errors.emplace_back(
      sdf::ErrorCode::STRING_READ,
      "Plane geometry cannot be converted to urdf C++ structures");
    return nullptr;
  }

  errors.emplace_back(
    sdf::ErrorCode::STRING_READ,
    "Unknown geometry shape");
  return nullptr;
}
