# SDFormat XML Robot Descriptions

This repo enables using SDFormat XML as a robot description format instead of URDF XML.
It does this by providing a `urdf_parser_plugin` for SDFormat that reads SDFormat and outputs URDF C++ DOM structures.
To use it, install `sdformat_urdf` and use a valid SDFormat XML file (with some limitations) for your robot description.
See the [README in the `sdformat_urdf` package](./sdformat_urdf/README.md) for more info on the limitations.

## Packages

* [`sdformat_urdf`](./sdformat_urdf/README.md)
  * provides a library and a `urdf_parser_plugin` using that library to convert SDFormat XML to URDF C++ DOM structures
* [`sdformat_test_files`](./sdformat_test_files/README.md)
  * provides SDFormat models using different parts of the SDFormat XML specification for testing

## Version combinations

This package can be compiled against versions of libSDFormat.

Set the `GZ_VERSION` environment variable to match the libSDFormat version you'd like to compile against.
For example:

    export GZ_VERSION=fortress

You only need to set this variable when compiling, not when running.
ROS version | Gazebo version | libSDFormat version | Branch | Binaries hosted at
-- | -- | -- | -- | --
Galactic | Citadel | 9.x | [galactic](https://github.com/ros/ros_ign/tree/galactic) | https://packages.ros.org
Galactic | Edifice | 11.x | [galactic](https://github.com/ros/ros_ign/tree/galactic) | only from source
Galactic | Fortress | 12.x | [galactic](https://github.com/ros/ros_ign/tree/galactic) | only from source
Humble | Fortress | 12.x | [humble](https://github.com/ros/sdformat_urdf/tree/humble) | https://packages.ros.org
Humble | Garden | 13.x | [humble](https://github.com/ros/sdformat_urdf/tree/humble) | only from source
Humble | Harmonic | 14.x | [humble](https://github.com/ros/sdformat_urdf/tree/humble) | only from source
Iron | Fortress | 12.x | [iron](https://github.com/ros/sdformat_urdf/tree/iron) | https://packages.ros.org
Iron | Garden | 13.x | [iron](https://github.com/ros/sdformat_urdf/tree/iron) | only from source
Iron | Harmonic | 14.x | [iron](https://github.com/ros/sdformat_urdf/tree/iron) | only from source
Jazzy | Harmonic | 14.x | [jazzy](https://github.com/ros/sdformat_urdf/tree/jazzy) | https://packages.ros.org
Rolling | Harmonic | 14.x | [rolling](https://github.com/ros/sdformat_urdf/tree/rolling) | https://packages.ros.org
