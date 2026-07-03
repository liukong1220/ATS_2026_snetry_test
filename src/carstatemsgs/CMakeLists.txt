cmake_minimum_required(VERSION 3.8)
project(carstatemsgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarState.msg"
  "msg/KinematicState.msg"
  "msg/CarControl.msg"
  "msg/SimulatedCarState.msg"
  "msg/Polynome.msg"
  "msg/TrajectoryPoint.msg"
  "msg/TrajectorySegment.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_package()
