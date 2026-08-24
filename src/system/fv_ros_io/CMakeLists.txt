cmake_minimum_required(VERSION 3.8)
project(fv_ros_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cpu_contract.cpp
  src/thread_affinity.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/setup_ros_io_cpu.sh
  DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cpu_contract
    test/test_cpu_contract.cpp
    src/cpu_contract.cpp
  )
  target_compile_features(test_cpu_contract PRIVATE cxx_std_17)
  target_include_directories(test_cpu_contract PRIVATE src)
  target_link_libraries(test_cpu_contract Threads::Threads)

  ament_add_gtest(test_thread_affinity
    test/test_thread_affinity.cpp
    src/cpu_contract.cpp
  )
  target_compile_features(test_thread_affinity PRIVATE cxx_std_17)
  target_include_directories(test_thread_affinity PRIVATE src)
  target_link_libraries(test_thread_affinity Threads::Threads)

  add_test(
    NAME test_setup_ros_io_cpu
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_setup_ros_io_cpu.sh
  )
endif()

ament_package()
