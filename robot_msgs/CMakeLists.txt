cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/quest/Quest2Button.msg"
  "msg/yam/YamCmd.msg"
  "msg/yam/YamStatus.msg"
  "msg/arx/RobotCmd.msg"
  "msg/arx/RobotStatus.msg"
  "msg/arx/PosCmd.msg"
  "msg/galaxea/MotorControl.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
