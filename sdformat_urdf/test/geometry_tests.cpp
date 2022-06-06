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

TEST(Geometry, geometry_box)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_BOX), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_box", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::BOX, link->visual->geometry->type);
  {
    urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<urdf::Box>(link->visual->geometry);
    ASSERT_NE(nullptr, box);
    EXPECT_DOUBLE_EQ(0.1, box->dim.x);
    EXPECT_DOUBLE_EQ(0.2, box->dim.y);
    EXPECT_DOUBLE_EQ(0.4, box->dim.z);
  }

  ASSERT_EQ(urdf::Geometry::BOX, link->collision->geometry->type);
  {
    urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
    ASSERT_NE(nullptr, box);
    EXPECT_DOUBLE_EQ(0.1, box->dim.x);
    EXPECT_DOUBLE_EQ(0.2, box->dim.y);
    EXPECT_DOUBLE_EQ(0.4, box->dim.z);
  }
}

TEST(Geometry, geometry_cylinder)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_CYLINDER), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_cylinder", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::CYLINDER, link->visual->geometry->type);
  {
    urdf::CylinderConstSharedPtr cylinder =
      std::dynamic_pointer_cast<urdf::Cylinder>(link->visual->geometry);
    ASSERT_NE(nullptr, cylinder);
    EXPECT_DOUBLE_EQ(0.2, cylinder->length);
    EXPECT_DOUBLE_EQ(0.125, cylinder->radius);
  }

  ASSERT_EQ(urdf::Geometry::CYLINDER, link->collision->geometry->type);
  {
    urdf::CylinderConstSharedPtr cylinder =
      std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
    ASSERT_NE(nullptr, cylinder);
    EXPECT_DOUBLE_EQ(0.2, cylinder->length);
    EXPECT_DOUBLE_EQ(0.125, cylinder->radius);
  }
}

TEST(Geometry, geometry_heightmap)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_HEIGHTMAP), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Geometry, geometry_mesh_collada)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_MESH_COLLADA), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_mesh_collada", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::MESH, link->visual->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_collada/torus.dae", mesh->filename);
  }

  ASSERT_EQ(urdf::Geometry::MESH, link->collision->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_collada/torus.dae", mesh->filename);
  }
}

TEST(Geometry, geometry_mesh_obj)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_MESH_OBJ), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_mesh_obj", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::MESH, link->visual->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_obj/torus.obj", mesh->filename);
  }

  ASSERT_EQ(urdf::Geometry::MESH, link->collision->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_obj/torus.obj", mesh->filename);
  }
}

TEST(Geometry, geometry_mesh_scaled)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_MESH_SCALED), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_mesh_scaled", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::MESH, link->visual->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_scaled/torus.stl", mesh->filename);
    EXPECT_DOUBLE_EQ(0.1, mesh->scale.x);
    EXPECT_DOUBLE_EQ(0.2, mesh->scale.y);
    EXPECT_DOUBLE_EQ(0.4, mesh->scale.z);
  }

  ASSERT_EQ(urdf::Geometry::MESH, link->collision->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_scaled/torus.stl", mesh->filename);
    EXPECT_DOUBLE_EQ(0.1, mesh->scale.x);
    EXPECT_DOUBLE_EQ(0.2, mesh->scale.y);
    EXPECT_DOUBLE_EQ(0.4, mesh->scale.z);
  }
}

TEST(Geometry, geometry_mesh_stl)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_MESH_STL), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_mesh_stl", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::MESH, link->visual->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_stl/torus.stl", mesh->filename);
  }

  ASSERT_EQ(urdf::Geometry::MESH, link->collision->geometry->type);
  {
    urdf::MeshConstSharedPtr mesh =
      std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
    ASSERT_NE(nullptr, mesh);
    EXPECT_EQ("model://geometry_mesh_stl/torus.stl", mesh->filename);
  }
}

TEST(Geometry, geometry_plane)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_PLANE), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Geometry, geometry_sphere)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GEOMETRY_SPHERE), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("geometry_sphere", model->getName());

  urdf::LinkConstSharedPtr link = model->getRoot();
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->visual);
  ASSERT_NE(nullptr, link->collision);
  ASSERT_NE(nullptr, link->visual->geometry);
  ASSERT_NE(nullptr, link->collision->geometry);

  ASSERT_EQ(urdf::Geometry::SPHERE, link->visual->geometry->type);
  {
    urdf::SphereConstSharedPtr sphere =
      std::dynamic_pointer_cast<urdf::Sphere>(link->visual->geometry);
    ASSERT_NE(nullptr, sphere);
    EXPECT_DOUBLE_EQ(0.125, sphere->radius);
  }

  ASSERT_EQ(urdf::Geometry::SPHERE, link->collision->geometry->type);
  {
    urdf::SphereConstSharedPtr sphere =
      std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);
    ASSERT_NE(nullptr, sphere);
    EXPECT_DOUBLE_EQ(0.125, sphere->radius);
  }
}
