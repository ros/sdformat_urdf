cmake_minimum_required(VERSION 3.5)
project(sdformat_test_files)

set(sdformat_test_files_VERSION 1.0.1)

# Decide where to install stuff
include(GNUInstallDirs)

set(INSTALL_SHARE_DIR "${CMAKE_INSTALL_DATAROOTDIR}/sdformat_test_files" CACHE PATH
  "Installation directory for arch independent data files")

set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATAROOTDIR}/sdformat_test_files/cmake" CACHE PATH
  "Installation directory for CMake files")

# Install models
set(model_names
  "geometry_box"
  "geometry_cylinder"
  "geometry_heightmap"
  "geometry_mesh_collada"
  "geometry_mesh_obj"
  "geometry_mesh_scaled"
  "geometry_mesh_stl"
  "geometry_plane"
  "geometry_sphere"
  "graph_chain"
  "graph_chain_non_canonical_root"
  "graph_four_bar"
  "graph_loop"
  "graph_tree"
  "graph_tree_non_canonical_root"
  "joint_ball"
  "joint_continuous"
  "joint_fixed"
  "joint_gearbox"
  "joint_prismatic"
  "joint_prismatic_no_axis"
  "joint_revolute"
  "joint_revolute2"
  "joint_revolute_axis"
  "joint_revolute_axis_in_frame"
  "joint_revolute_default_limits"
  "joint_revolute_two_joints_two_links"
  "joint_screw"
  "joint_universal"
  "link_inertia"
  "link_light_point"
  "link_multiple_collisions"
  "link_multiple_visuals"
  "link_sensor_imu"
  "material_blinn_phong"
  "material_dynamic_lights"
  "model_two_models"
  "model_zero_models"
  "pose_chain"
  "pose_collision"
  "pose_collision_in_frame"
  "pose_inertial"
  "pose_inertial_in_frame"
  "pose_joint"
  "pose_joint_all"
  "pose_joint_in_frame"
  "pose_link"
  "pose_link_all"
  "pose_link_in_frame"
  "pose_model"
  "pose_visual"
  "pose_visual_in_frame"
)

foreach(model ${model_names})
  # Install models to share/project-name/model-name
  install(DIRECTORY "models/${model}" DESTINATION "${INSTALL_SHARE_DIR}/models")
endforeach()

set(_arch_independent "")
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.14.0")
  set(_arch_independent "ARCH_INDEPENDENT")
endif()

# Create projectConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_filesConfigVersion.cmake"
  VERSION "${sdformat_test_files_VERSION}"
  COMPATIBILITY SameMajorVersion
  ${_arch_independent})

# Create projectConfig.cmake
configure_package_config_file("sdformat_test_filesConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_filesConfig.cmake"
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
  PATH_VARS "INSTALL_CMAKE_DIR" "INSTALL_SHARE_DIR")

# Create file with CMake functions
configure_file("sdformat_test_files_functions.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_files_functions.cmake" @ONLY)

# Install cmake files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_filesConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_filesConfigVersion.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/sdformat_test_files/sdformat_test_files_functions.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")
