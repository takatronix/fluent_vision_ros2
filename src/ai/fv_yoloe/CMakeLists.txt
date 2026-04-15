cmake_minimum_required(VERSION 3.8)
project(fv_yoloe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(fv_yoloe_py PACKAGE_DIR fv_yoloe_py)

install(PROGRAMS
  scripts/fv_yoloe_node
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

ament_package()
