cmake_minimum_required(VERSION 3.5)

project(sdformat_urdf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(sdformat9 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdf_parser_plugin REQUIRED)

# Add sdformat_urdf shared library
add_library(sdformat_urdf SHARED
  src/sdformat_urdf.cpp
)
target_link_libraries(sdformat_urdf
  PUBLIC
    sdformat9::sdformat9
)
target_include_directories(sdformat_urdf
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${urdfdom_headers_INCLUDE_DIRS}
)

# Add sdformat_urdf_plugin module library
add_library(sdformat_urdf_plugin MODULE
  src/sdformat_urdf_plugin.cpp
)
target_link_libraries(sdformat_urdf_plugin PUBLIC
  sdformat_urdf
)
ament_target_dependencies(sdformat_urdf_plugin PUBLIC
  "pluginlib"
  "rcutils"
  "urdf_parser_plugin"
)

ament_export_dependencies(urdfdom_headers)
ament_export_dependencies(sdformat9)

install(TARGETS sdformat_urdf sdformat_urdf_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS sdformat_urdf EXPORT sdformat_urdf-export)
ament_export_targets(sdformat_urdf-export)

pluginlib_export_plugin_description_file(urdf_parser_plugin "sdformat_urdf_plugin.xml")

ament_package()
