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
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

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
  "TinyXML2"
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(sdformat_test_files 0.1 REQUIRED)
  ament_lint_auto_find_test_dependencies()

  sdformat_test_files_get_model_sdf("pose_collision_path_to_sdf" "pose_collision")
  sdformat_test_files_get_model_sdf("pose_inertial_path_to_sdf" "pose_inertial")
  sdformat_test_files_get_model_sdf("pose_link_all_path_to_sdf" "pose_link_all")
  sdformat_test_files_get_model_sdf("pose_link_in_frame_path_to_sdf" "pose_link_in_frame")
  sdformat_test_files_get_model_sdf("pose_link_path_to_sdf" "pose_link")
  sdformat_test_files_get_model_sdf("pose_model_path_to_sdf" "pose_model")
  sdformat_test_files_get_model_sdf("pose_visual_path_to_sdf" "pose_visual")
  configure_file("test/sdf_paths.hpp.in" "${CMAKE_CURRENT_BINARY_DIR}/generated_includes/sdf_paths.hpp" @ONLY)

  ament_add_gtest(sdf_pose_tests "test/sdf_pose_tests.cpp")
  target_link_libraries(sdf_pose_tests sdformat_urdf)
  target_include_directories(sdf_pose_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

endif()

ament_package()
