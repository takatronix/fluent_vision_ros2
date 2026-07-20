cmake_minimum_required(VERSION 3.8)
project(fv_tts)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fluent_dialogue_dora_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(fv_tts_py PACKAGE_DIR fv_tts_py)

install(PROGRAMS scripts/fv_tts_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config resource DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fv_tts test/test_fv_tts.py)
endif()

ament_package()
