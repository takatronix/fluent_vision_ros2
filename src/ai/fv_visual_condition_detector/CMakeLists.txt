cmake_minimum_required(VERSION 3.8)
project(fv_visual_condition_detector)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  fv_visual_condition_detector_py
  PACKAGE_DIR fv_visual_condition_detector_py
)

install(PROGRAMS
  scripts/fv_visual_condition_detector_node
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
  ament_add_pytest_test(test_vad_state test/test_vad_state.py)
endif()

ament_package()
