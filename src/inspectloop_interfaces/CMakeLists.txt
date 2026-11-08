cmake_minimum_required(VERSION 3.8)
project(inspectloop_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaugeReading.msg"
  "msg/GaugeDecision.msg"
  "msg/MissionStatus.msg"
  "srv/ReadGauge.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
