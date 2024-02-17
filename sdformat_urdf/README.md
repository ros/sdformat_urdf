# SDFormat URDF

This package contains a C++ library and `urdf_parser_plugin` for converting SDFormat XML into URDF C++ structures.
Installing it allows one to use SDFormat XML instead of URDF XML as a robot description.

## Supported platforms

The package has been tested with ROS Rolling Ridley on Ubuntu Focal.

## Limitations and Considerations

In general, URDF C++ structures cannot represent every possible SDFormat XML file.
SDFormat XML used with this plugin must be constructed with the following limitations.

### Limitations that result in Errors

This package will error and refuse to convert any SDFormat XML that violates these constraints.

#### Model limitations
* The XML file must contain a single `<model>` not in a `<world>`
* The `<model>` tag must not have a `<pose>`
* There must not be any nested `<model>`

#### Geometry limitations
* The only supported geometry types are `<box>`, `<cylinder>`, `<mesh>`, and `<sphere>`
* The model must not use `<plane>` or `<heightmap>`

#### Joint limitations
* The only supported joint types are `continuous`, `fixed`, `prismatic`, and `revolute`
* The model must not use `universal`, `screw`, `revolute2`, `gearbox`, or `ball` joints

#### Kinematic limitations
*  Starting from the canonical link, the links and joints must form a tree
*  No link may be a child of more than 1 joint
*  The canonical link cannot be a child of any joint

### Limitations that may result in Warnings

If any of these constraints are violated then the library may issue a console warning, but the model is still converted to URDF c++ structures.
The warning is issued using an `rcutils` logger with the name `sdformat_urdf`.

#### Joint limitations
* `<axis>` should not use `<initial_position>`
* `<dynamics>` should not use `<spring_reference>` or `<spring_stiffness>`
* `<limit>` should not use `<dissipation>` or `<stiffness>`
* `<joint>` should not use `<sensor>` or `<physics>`

#### Link limitations
* `<link>` should not have any `<light>` or `<sensor>`

#### Material limitations
* Only solid color materials are supported
  * The color only uses `<ambient>` and `<diffuse>` tags of the material
  * Color is calculated as `0.4 * ambient + 0.8 * diffuse`
* The model should not disable dynamic lighting using the `<lighting>`
* The model should not use materials with `<script>`, `<shader>`, or `<pbr>`
* The URDF material name assigned the name of the SDFormat `<visual>` containing the material, so `<visual>` names should be unique within the model

### Considerations for Mesh URIs

The `model://` prefix is commonly used in SDFormat XML files to set URIs for meshes, e.g. `model://my_model/meshes/mesh1.obj`.
However, ROS tools likes `RViz` which rely on [resource_retriever](https://github.com/ros/resource_retriever/tree/ros2) will throw errors when attempting to load
a mesh from such an URI since `resource_retriever` does not resolve `model://` prefixes to absolute filepaths.
This may happen when an SDFormat XML file is parsed into an URDF XML by a `robot_state_publisher` node and published as a serialized
string over `/robot_description` which is subscribed to by `RViz`.
To allow both `Gazebo` and `RViz` to resolve URIs to absolute paths and load meshes properly, the following pattern may be adopted.
- Move the `my_model` folder containing the meshes, `model.sdf` and `model.config` files into a ROS package, say `my_model_description` which installs the model.
- Set the mesh URIs using the `package://` prefix, e.g. `package://my_model_description/models/my_model/meshes/mesh1.obj`
- Set the `GZ_SIM_RESOURCE_PATH` environment variable to `<install_prefix>/my_model_description/share/`.

> Note `<install_prefix>/my_model_description/share/` above is the output of `os.path.join(get_package_prefix('my_model_description'), 'share')` or `"$(ros2 pkg prefix my_model_description)/share"` and not `ros2 pkg prefix --share my_model_description` since the latter includes an extra package name at the end. See https://github.com/ros/sdformat_urdf/pull/20#issuecomment-1944430224 for more details.
