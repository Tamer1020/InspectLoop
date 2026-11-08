cmake_minimum_required(VERSION 3.8)
project(inspectloop_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_warehouse.py scripts/gz_sim_supervised.sh
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gz_sim_supervised.sh DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world_generation test/test_world_generation.py)
endif()

ament_package()
