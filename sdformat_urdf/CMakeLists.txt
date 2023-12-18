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
find_package(urdfdom_headers 1.0.6 REQUIRED)
find_package(urdf_parser_plugin REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Default to Fortress
set(SDF_VER 12)

# If the user didn't specify a GZ distribution, pick the one matching the ROS distribution according to REP 2000
if(NOT DEFINED ENV{GZ_VERSION} AND DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ENV{GZ_VERSION} "fortress")
  endif()
endif()

# Find libsdformat matching the picked GZ distribution
if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(sdformat12 REQUIRED)
  set(SDF_VER ${sdformat12_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Fortress (libSDFormat 12)")
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden (libSDFormat 13)")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(sdformat14 REQUIRED)
  set(SDF_VER ${sdformat14_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic (libSDFormat 14)")
# No GZ distribution specified, find any version of libsdformat we can
else()
  foreach(major RANGE 13 9)
    find_package(sdformat${major} QUIET)
    if(sdformat${major}_FOUND)
      # Next `find_package` call will be a noop
      set(SDF_VER ${major})
      message(STATUS "Compiling against libSDFormat ${major}")
      break()
    endif()
  endforeach()
endif()

# Add sdformat_urdf shared library
add_library(sdformat_urdf SHARED
  src/sdformat_urdf.cpp
)
target_link_libraries(sdformat_urdf PUBLIC
    sdformat${SDF_VER}::sdformat${SDF_VER}
  urdfdom_headers::urdfdom_headers
)
target_link_libraries(sdformat_urdf PRIVATE
  rcutils::rcutils
)
target_include_directories(sdformat_urdf
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# Add sdformat_urdf_plugin module library
# not actually a MODULE as workaround for
# https://github.com/ros/pluginlib/issues/200
add_library(sdformat_urdf_plugin SHARED
  src/sdformat_urdf_plugin.cpp
)
target_link_libraries(sdformat_urdf_plugin PRIVATE
  pluginlib::pluginlib
  rcutils::rcutils
  sdformat_urdf
  tinyxml2::tinyxml2
  urdf_parser_plugin::urdf_parser_plugin
)

ament_export_dependencies(pluginlib)
ament_export_dependencies(rcutils)
ament_export_dependencies(sdformat${SDF_VER})
ament_export_dependencies(tinyxml2)
ament_export_dependencies(urdf_parser_plugin)
ament_export_dependencies(urdfdom_headers)

install(TARGETS sdformat_urdf EXPORT sdformat_urdf-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sdformat_urdf_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(sdformat_urdf-export HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(urdf_parser_plugin "sdformat_urdf_plugin.xml")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(sdformat_test_files REQUIRED)
  ament_lint_auto_find_test_dependencies()

  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_box" "geometry_box")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_cylinder" "geometry_cylinder")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_heightmap" "geometry_heightmap")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_mesh_collada" "geometry_mesh_collada")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_mesh_obj" "geometry_mesh_obj")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_mesh_scaled" "geometry_mesh_scaled")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_mesh_stl" "geometry_mesh_stl")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_plane" "geometry_plane")
  sdformat_test_files_get_model_sdf("path_to_sdf_geometry_sphere" "geometry_sphere")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_chain" "graph_chain")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_chain_non_canonical_root" "graph_chain_non_canonical_root")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_four_bar" "graph_four_bar")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_loop" "graph_loop")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_tree" "graph_tree")
  sdformat_test_files_get_model_sdf("path_to_sdf_graph_tree_non_canonical_root" "graph_tree_non_canonical_root")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_ball" "joint_ball")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_continuous" "joint_continuous")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_fixed" "joint_fixed")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_gearbox" "joint_gearbox")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_prismatic" "joint_prismatic")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_prismatic_no_axis" "joint_prismatic_no_axis")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute" "joint_revolute")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute2" "joint_revolute2")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute_axis" "joint_revolute_axis")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute_axis_in_frame" "joint_revolute_axis_in_frame")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute_default_limits" "joint_revolute_default_limits")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_revolute_two_joints_two_links" "joint_revolute_two_joints_two_links")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_screw" "joint_screw")
  sdformat_test_files_get_model_sdf("path_to_sdf_joint_universal" "joint_universal")
  sdformat_test_files_get_model_sdf("path_to_sdf_link_inertia" "link_inertia")
  sdformat_test_files_get_model_sdf("path_to_sdf_link_light_point" "link_light_point")
  sdformat_test_files_get_model_sdf("path_to_sdf_link_multiple_collisions" "link_multiple_collisions")
  sdformat_test_files_get_model_sdf("path_to_sdf_link_multiple_visuals" "link_multiple_visuals")
  sdformat_test_files_get_model_sdf("path_to_sdf_link_sensor_imu" "link_sensor_imu")
  sdformat_test_files_get_model_sdf("path_to_sdf_material_blinn_phong" "material_blinn_phong")
  sdformat_test_files_get_model_sdf("path_to_sdf_material_dynamic_lights" "material_dynamic_lights")
  sdformat_test_files_get_model_sdf("path_to_sdf_model_two_models" "model_two_models")
  sdformat_test_files_get_model_sdf("path_to_sdf_model_zero_models" "model_zero_models")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_chain" "pose_chain")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_collision" "pose_collision")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_collision_in_frame" "pose_collision_in_frame")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_inertial" "pose_inertial")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_inertial_in_frame" "pose_inertial_in_frame")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_joint" "pose_joint")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_joint_all" "pose_joint_all")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_joint_in_frame" "pose_joint_in_frame")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_link" "pose_link")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_link_all" "pose_link_all")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_link_in_frame" "pose_link_in_frame")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_model" "pose_model")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_visual" "pose_visual")
  sdformat_test_files_get_model_sdf("path_to_sdf_pose_visual_in_frame" "pose_visual_in_frame")
  configure_file("test/sdf_paths.hpp.in" "${CMAKE_CURRENT_BINARY_DIR}/generated_includes/sdf_paths.hpp" @ONLY)

  ament_add_gtest(sdf_geometry_tests "test/geometry_tests.cpp")
  target_link_libraries(sdf_geometry_tests sdformat_urdf)
  target_include_directories(sdf_geometry_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_graph_tests "test/graph_tests.cpp")
  target_link_libraries(sdf_graph_tests sdformat_urdf)
  target_include_directories(sdf_graph_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_joint_tests "test/joint_tests.cpp")
  target_link_libraries(sdf_joint_tests sdformat_urdf)
  target_include_directories(sdf_joint_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_link_tests "test/link_tests.cpp")
  target_link_libraries(sdf_link_tests sdformat_urdf)
  target_include_directories(sdf_link_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_material_tests "test/material_tests.cpp")
  target_link_libraries(sdf_material_tests sdformat_urdf)
  target_include_directories(sdf_material_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_model_tests "test/model_tests.cpp")
  target_link_libraries(sdf_model_tests sdformat_urdf)
  target_include_directories(sdf_model_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

  ament_add_gtest(sdf_pose_tests "test/pose_tests.cpp")
  target_link_libraries(sdf_pose_tests sdformat_urdf)
  target_include_directories(sdf_pose_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

endif()

ament_package()
