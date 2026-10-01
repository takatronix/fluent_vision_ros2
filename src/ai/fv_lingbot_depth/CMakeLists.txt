cmake_minimum_required(VERSION 3.8)
project(fv_lingbot_depth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(fv_lingbot_depth_py PACKAGE_DIR fv_lingbot_depth_py)

install(PROGRAMS
  scripts/fv_lingbot_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_input_demand test/test_input_demand.py TIMEOUT 30
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
endif()

ament_package()
