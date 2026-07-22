cmake_minimum_required(VERSION 3.8)
project(fv_tts)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aspa_audio_interfaces REQUIRED)
find_package(fv_speech_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# voicevox_core is an official, pinned external binary installed by
# scripts/setup-voicevox-core.  An explicit cache value wins, followed by the
# setup script's environment/default locations and then normal system paths.
set(VOICEVOX_CORE_ROOT "" CACHE PATH "Root containing voicevox_core include/ and lib/")
set(_voicevox_core_hints "")
if(VOICEVOX_CORE_ROOT)
  list(APPEND _voicevox_core_hints "${VOICEVOX_CORE_ROOT}")
endif()
if(DEFINED ENV{VOICEVOX_CORE_ROOT} AND NOT "$ENV{VOICEVOX_CORE_ROOT}" STREQUAL "")
  list(APPEND _voicevox_core_hints "$ENV{VOICEVOX_CORE_ROOT}")
endif()
if(DEFINED ENV{VOICEVOX_CORE_INSTALL_ROOT} AND NOT "$ENV{VOICEVOX_CORE_INSTALL_ROOT}" STREQUAL "")
  list(APPEND _voicevox_core_hints "$ENV{VOICEVOX_CORE_INSTALL_ROOT}/current/core")
endif()
if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
  list(APPEND _voicevox_core_hints "$ENV{HOME}/.aspa/voicevox_core/current/core")
endif()

find_path(VOICEVOX_CORE_INCLUDE_DIR
  NAMES voicevox_core.h
  HINTS ${_voicevox_core_hints}
  PATH_SUFFIXES include
)
find_library(VOICEVOX_CORE_LIBRARY
  NAMES voicevox_core
  HINTS ${_voicevox_core_hints}
  PATH_SUFFIXES lib
)
if(NOT VOICEVOX_CORE_INCLUDE_DIR OR NOT VOICEVOX_CORE_LIBRARY)
  message(FATAL_ERROR
    "voicevox_core C API was not found. Run scripts/setup-voicevox-core, or "
    "configure with -DVOICEVOX_CORE_ROOT=/path/to/voicevox_core.")
endif()
get_filename_component(VOICEVOX_CORE_LIBRARY_DIR "${VOICEVOX_CORE_LIBRARY}" DIRECTORY)

add_library(fv_tts_core STATIC
  src/contracts.cpp
  src/synthesis_scheduler.cpp
  src/synthesis_watchdog.cpp
  src/voicevox_backend.cpp
)
target_include_directories(fv_tts_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${VOICEVOX_CORE_INCLUDE_DIR}"
)
target_link_libraries(fv_tts_core
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE "${VOICEVOX_CORE_LIBRARY}"
)
target_compile_options(fv_tts_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(fv_tts_node src/tts_node.cpp)
target_link_libraries(fv_tts_node fv_tts_core "${VOICEVOX_CORE_LIBRARY}")
ament_target_dependencies(fv_tts_node
  aspa_audio_interfaces
  fv_speech_interfaces
  rclcpp
  std_msgs
  std_srvs
)
target_compile_options(fv_tts_node PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(fv_tts_node PROPERTIES
  BUILD_RPATH "${VOICEVOX_CORE_LIBRARY_DIR}"
  INSTALL_RPATH "${VOICEVOX_CORE_LIBRARY_DIR}"
)

install(TARGETS fv_tts_core fv_tts_node
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config resource DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_fv_tts_core
    test/test_contracts.cpp
    test/test_synthesis_scheduler.cpp
    test/test_synthesis_watchdog.cpp
  )
  if(TARGET test_fv_tts_core)
    target_link_libraries(test_fv_tts_core fv_tts_core "${VOICEVOX_CORE_LIBRARY}")
    set_target_properties(test_fv_tts_core PROPERTIES
      BUILD_RPATH "${VOICEVOX_CORE_LIBRARY_DIR}"
    )
  endif()
  ament_add_pytest_test(test_fv_tts_launch test/test_launch_contract.py)
endif()

ament_export_include_directories(include)
ament_package()
