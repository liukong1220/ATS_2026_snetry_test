cmake_minimum_required(VERSION 3.8)
project(manda_can_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryFb.msg"
  "msg/FaultFb.msg"
  "msg/MotionCtrl.msg"
  "msg/MotionFb.msg"
  "msg/SpeedCtrl.msg"
  "msg/SpeedFb.msg"
  "msg/SteerCtrl.msg"
  "msg/SteerFb.msg"
  "msg/SystemstateFb.msg"
  "srv/ControlMode.srv"
  "srv/DoCtrl.srv"
  "srv/MotionMode.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
