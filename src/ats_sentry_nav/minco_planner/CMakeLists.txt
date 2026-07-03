cmake_minimum_required(VERSION 3.10)
project(minco_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug/planner_debug_visualizer.cpp
  src/nodes/minco_planner_node.cpp
  src/planning/grid_astar.cpp
  src/safety/footprint_safety_checker.cpp
  src/safety/local_collision_repair.cpp
  src/trajectory/minco_trajectory_optimizer.cpp
  src/trajectory/yaw_spline_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN minco_planner::MincoPlannerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
