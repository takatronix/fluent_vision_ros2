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
  # 需要駆動ゲートの単体テスト。ROS には触らないので実機でも安全に回せる
  # (ROSノードを構築するテストは 2026-08-23 に 52GB 食ってスタックを落とした)。
  ament_add_pytest_test(test_demand_gate test/test_demand_gate.py)
endif()

ament_package()
