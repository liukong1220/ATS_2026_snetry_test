cmake_minimum_required(VERSION 3.8)
project(ats_sentry_bringup)

# 未指定 CMAKE_BUILD_TYPE 时 colcon 不传 -O，整包以 -O0 编译（报告 5.1.6 H）；默认取 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ros_domain_id.dsv.in")

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  params
  pcd
  rviz
)

install(
  PROGRAMS
  scripts/fake_decision_sim_inputs.py
  scripts/debug_path_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)
