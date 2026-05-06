cmake_minimum_required(VERSION 3.10)
project(trajectory_optimizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bspline_path_optimizer.cpp
  src/trajectory_optimizer_node.cpp
  src/nav2_bspline_smoother.cpp
  src/trajectory_speed_governor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN trajectory_optimizer::TrajectoryOptimizerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN trajectory_optimizer::TrajectorySpeedGovernor
  EXECUTABLE trajectory_speed_governor_node
)

pluginlib_export_plugin_description_file(nav2_core trajectory_optimizer_plugins.xml)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  trajectory_optimizer_plugins.xml
)
