cmake_minimum_required(VERSION 3.8)
project(aspa_audio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/OutputDrained.msg
  msg/PlaybackControl.msg
  msg/PlaybackEvent.msg
  msg/PlaybackFrame.msg
  msg/SpeechMark.msg
  msg/SynthesizedSpeech.msg
  srv/FlushAudio.srv
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interfaces test/test_interfaces.py)
endif()

ament_package()
