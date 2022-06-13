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

TEST(Graph, graph_chain)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_CHAIN), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("graph_chain", model->getName());

  EXPECT_EQ(3u, model->links_.size());
  EXPECT_EQ(2u, model->joints_.size());

  ASSERT_NE(nullptr, model->getRoot());
  EXPECT_EQ("link_1", model->getRoot()->name);

  { // link_1
    urdf::LinkConstSharedPtr link = model->getLink("link_1");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(nullptr, link->parent_joint);
    EXPECT_EQ(nullptr, link->getParent());
    EXPECT_NAMES(link->child_joints, "joint_1");
    EXPECT_NAMES(link->child_links, "link_2");
  }

  { // link_2
    urdf::LinkConstSharedPtr link = model->getLink("link_2");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_1"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_1"), link->getParent());
    EXPECT_NAMES(link->child_joints, "joint_2");
    EXPECT_NAMES(link->child_links, "link_3");
  }

  { // link_3
    urdf::LinkConstSharedPtr link = model->getLink("link_3");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_2"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_2"), link->getParent());
    EXPECT_TRUE(link->child_joints.empty());
    EXPECT_TRUE(link->child_links.empty());
  }

  { // joint_1
    urdf::JointConstSharedPtr joint = model->getJoint("joint_1");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_1", joint->parent_link_name);
    EXPECT_EQ("link_2", joint->child_link_name);
  }

  { // joint_2
    urdf::JointConstSharedPtr joint = model->getJoint("joint_2");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_2", joint->parent_link_name);
    EXPECT_EQ("link_3", joint->child_link_name);
  }
}

TEST(Graph, graph_chain_non_canonical_root)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_CHAIN_NON_CANONICAL_ROOT), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Graph, graph_four_bar)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_FOUR_BAR), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Graph, graph_loop)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_LOOP), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}

TEST(Graph, graph_tree)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_TREE), errors);
  EXPECT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(model);
  ASSERT_EQ("graph_tree", model->getName());

  EXPECT_EQ(6u, model->links_.size());
  EXPECT_EQ(5u, model->joints_.size());

  ASSERT_NE(nullptr, model->getRoot());
  EXPECT_EQ("link_1", model->getRoot()->name);

  { // link_1
    urdf::LinkConstSharedPtr link = model->getLink("link_1");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(nullptr, link->parent_joint);
    EXPECT_EQ(nullptr, link->getParent());
    EXPECT_NAMES(link->child_joints, "joint_1", "joint_2");
    EXPECT_NAMES(link->child_links, "link_2", "link_3");
  }

  { // link_2
    urdf::LinkConstSharedPtr link = model->getLink("link_2");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_1"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_1"), link->getParent());
    EXPECT_NAMES(link->child_joints, "joint_3");
    EXPECT_NAMES(link->child_links, "link_4");
  }

  { // link_3
    urdf::LinkConstSharedPtr link = model->getLink("link_3");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_2"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_1"), link->getParent());
    EXPECT_NAMES(link->child_joints, "joint_4", "joint_5");
    EXPECT_NAMES(link->child_links, "link_5", "link_6");
  }

  { // link_4
    urdf::LinkConstSharedPtr link = model->getLink("link_4");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_3"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_2"), link->getParent());
    EXPECT_TRUE(link->child_joints.empty());
    EXPECT_TRUE(link->child_links.empty());
  }

  { // link_5
    urdf::LinkConstSharedPtr link = model->getLink("link_5");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_4"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_3"), link->getParent());
    EXPECT_TRUE(link->child_joints.empty());
    EXPECT_TRUE(link->child_links.empty());
  }

  { // link_6
    urdf::LinkConstSharedPtr link = model->getLink("link_6");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(model->getJoint("joint_5"), link->parent_joint);
    EXPECT_EQ(model->getLink("link_3"), link->getParent());
    EXPECT_TRUE(link->child_joints.empty());
    EXPECT_TRUE(link->child_links.empty());
  }

  { // joint_1
    urdf::JointConstSharedPtr joint = model->getJoint("joint_1");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_1", joint->parent_link_name);
    EXPECT_EQ("link_2", joint->child_link_name);
  }

  { // joint_2
    urdf::JointConstSharedPtr joint = model->getJoint("joint_2");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_1", joint->parent_link_name);
    EXPECT_EQ("link_3", joint->child_link_name);
  }

  { // joint_3
    urdf::JointConstSharedPtr joint = model->getJoint("joint_3");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_2", joint->parent_link_name);
    EXPECT_EQ("link_4", joint->child_link_name);
  }

  { // joint_4
    urdf::JointConstSharedPtr joint = model->getJoint("joint_4");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_3", joint->parent_link_name);
    EXPECT_EQ("link_5", joint->child_link_name);
  }

  { // joint_5
    urdf::JointConstSharedPtr joint = model->getJoint("joint_5");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ("link_3", joint->parent_link_name);
    EXPECT_EQ("link_6", joint->child_link_name);
  }
}

TEST(Graph, graph_tree_non_canonical_root)
{
  sdf::Errors errors;
  urdf::ModelInterfaceSharedPtr model = sdformat_urdf::parse(
    get_file(PATH_TO_SDF_GRAPH_TREE_NON_CANONICAL_ROOT), errors);
  EXPECT_FALSE(errors.empty());
  EXPECT_NO_ALGORITHM_ERRORS(errors);
  ASSERT_FALSE(model);
}
