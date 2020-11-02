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
