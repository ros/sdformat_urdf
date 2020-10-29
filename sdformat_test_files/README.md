# SDFormat Test files

This package contains a list of SDFormat files for testing tools that work with SDFormat XML.

## Models

### Geometry

* `geometry_box`
  * A single-link model using the box geometry type for both the visual and collision.
* `geometry_cylinder`
  * A single-link model using the cylinder geometry type for both the visual and collision.
* `geometry_heightmap`
  * A single-link model using heightmap geometry for both the visual and collision.
* `geometry_mesh_collada`
  * A single link using mesh geometry with a COLLADA mesh.
* `geometry_mesh_obj`
  * A single link using a Wavefront OBJ mesh.
* `geometry_mesh_scaled`
  * A single-link model using a mesh scaled differently on x, y, and z axes for both the visual and collision.
* `geometry_mesh_stl`
  * A single-link model using an STL mesh.
* `geometry_plane`
  * A single-link model using the plane geometry type for both the visual and collision.
* `geometry_sphere`
  * A single-link model using the sphere geometry type for both the visual and collision.

### Materials

* `material_blinn_phong`
  * A single link with a material that uses ambient/diffuse/specular/emissive components to color it.
* `material_dynamic_lights`
  * A link with two visuals: one with dynamic lights on, and the other with dynamic lights off.

### Joints

* `joint_ball`
  * A model with two links connected by a ball joint.
* `joint_continuous`
  * A model with two links connected by a continuous joint.
* `joint_fixed`
  * A model with two links connected by a fixed joint.
* `joint_gearbox`
  * A model with 3 links total. Two links are connected to a reference link with revolute joints and a gearbox joint enforces that one revolute joint rotates faster than the other.
* `joint_prismatic`
  * A model with two links connected by a prismatic joint.
* `joint_revolute`
  * A model with two links connected by a revolute joint.
* `joint_revolute2`
  * A model with two links connected by a revolute2 joint.
* `joint_revolute_axis`
  * A model with two links connected by a revolute joint with an axis having different values for x, y, and z.
* `joint_revolute_axis_in_frame`
  * A model with two links connected by a revolute joint with an axis having different values for x, y, and z, and specified in a frame on the model.
* `joint_revolute_default_limits`
  * A model with two links connected by a revolute joint, having no joint limits specified on its axis.
* `joint_revolute_two_joints_two_links`
  * A model with two links connected by two revolute joints, effectively rigidly connecting the two.
* `joint_screw`
  * A model with two links connected by a screw joint.
* `joint_universal`
  * A model with two links connected by a universal joint.

### Links

* `link_inertia`
  * A link having an inerta with a different value for each of it's 6 components.
* `link_light_point`
  * A model with a single link having a point light attached to it.
* `link_multiple_collisions`
  * A model with a single link having multiple collision elements on it.
* `link_multiple_visuals`
  * A model with a single link having multile visual elements on it.
* `link_sensor_imu`
  * A model with a single link having an IMU sensor attached to it.

### Kinematic structures

* `graph_chain`
  * A model having a chain of 3 links connected in series with revolute joints.
* `graph_chain_non_canonical_root`
  * A model having a chain of 3 links connected in series with revolute joints, but the canonical link is not the root of the chain.
* `graph_four_bar`
  * A four-bar linkage made with four links connected by 4 revolute joints.
* `graph_loop`
  * A model having three links connected by 3 joints to form a triangle.
* `graph_tree`
  * A model with multiple links connected by joints forming a tree.
* `graph_tree_non_canonical_root`
  * A model with multiple links connected by joints forming a tree, but the canonical link is not the root of the tree.

### Poses and Frames

* `pose_chain`
  * A chain of links connected by joints, where every link and joint in the chain has a non-zero pose.
* `pose_collision`
  * A single-link model where only the collision has a non-zero pose.
* `pose_collision_in_frame`
  * A single-link model where only the collision has a non-zero pose, and that pose is given in a frame on the model.
* `pose_inertial`
  * A single-link model where only the inertial has a non-zero pose.
* `pose_inertial_in_frame`
  * A single-link model where only the inertial has a non-zero pose, and that pose is given in a frame on the model.
* `pose_joint`
  * A model having two links and a revolute joint, where only the joint has a non-zero pose.
* `pose_joint_all`
  * A model having two links and a revolute joint, with non-zero poses on all including the inertials, visuals, and collisions.
* `pose_joint_in_frame`
  * A model having two links and a revolute joint, where only the joint has a non-zero pose, and that pose is given in a frame on the model.
* `pose_link`
  * A single-link model where only the link has a non-zero pose.
* `pose_link_all`
  * A single-link model where the link, visual, collision, and inertial elements all have poses.
* `pose_link_in_frame`
  * A single-link model where only the link has a non-zero pose, and that pose is given in a frame on the model.
* `pose_model`
  * A single-link model where the model itself has a non-zero pose.
* `pose_visual`
  * A single-link model where only the visual has a non-zero pose.
* `pose_visual_in_frame`
  * A single-link model where only the visual has a non-zero pose, and that pose is given in a frame on the model.

### Models

* `model_two_models`
  * An SDFormat XML file having two models in it.
* `model_zero_models`
  * An SDFormat XML file that does not have a model in it.

