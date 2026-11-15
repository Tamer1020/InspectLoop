cmake_minimum_required(VERSION 3.8)
project(inspectloop_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-independent safety logic (unit tested)
add_library(${PROJECT_NAME}_core
  src/gate_supervisor.cpp
  src/topic_monitor.cpp
  src/velocity_gate.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(cmd_vel_gate src/cmd_vel_gate_node.cpp)
target_link_libraries(cmd_vel_gate ${PROJECT_NAME}_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})

add_executable(health_monitor src/health_monitor_node.cpp)
target_link_libraries(health_monitor ${PROJECT_NAME}_core
  rclcpp::rclcpp ${diagnostic_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros)

install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS cmd_vel_gate health_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topic_monitor test/test_topic_monitor.cpp)
  target_link_libraries(test_topic_monitor ${PROJECT_NAME}_core)
  ament_add_gtest(test_velocity_gate test/test_velocity_gate.cpp)
  target_link_libraries(test_velocity_gate ${PROJECT_NAME}_core)
  ament_add_gtest(test_astra_safety_existing_api test/test_astra_safety_existing_api.cpp)
  target_link_libraries(test_astra_safety_existing_api ${PROJECT_NAME}_core)
  ament_add_gtest(test_scan_validation test/test_scan_validation.cpp)
  target_link_libraries(test_scan_validation ${PROJECT_NAME}_core)
  ament_add_gtest(test_v3_sector_blindness_existing_api test/test_v3_sector_blindness_existing_api.cpp)
  target_link_libraries(test_v3_sector_blindness_existing_api ${PROJECT_NAME}_core)
  ament_add_gtest(test_v3_motion_coverage test/test_v3_motion_coverage.cpp)
  target_link_libraries(test_v3_motion_coverage ${PROJECT_NAME}_core)
  ament_add_gtest(test_v3_1_scan_workload test/test_v3_1_scan_workload.cpp TIMEOUT 60)
  target_link_libraries(test_v3_1_scan_workload ${PROJECT_NAME}_core)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
