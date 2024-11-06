cmake_minimum_required(VERSION 3.8)
project(project_tsb_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoatPosition.msg"
  "msg/BoatReference.msg"
  "msg/ControlForces.msg"
  "msg/ControlCurrents.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
