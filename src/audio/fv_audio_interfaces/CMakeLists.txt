cmake_minimum_required(VERSION 3.8)
project(fv_audio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AudioDevice.msg
  msg/OutputDrained.msg
  msg/PlaybackControl.msg
  msg/PlaybackEvent.msg
  msg/PlaybackFrame.msg
  msg/SynthesizedSpeechChunk.msg
  msg/TtsAdmissionState.msg
  msg/TtsAlignmentEvent.msg
  msg/TtsRequest.msg
  msg/TtsResult.msg
  msg/TtsRuntimeState.msg
  msg/TtsSay.msg
  msg/TtsSourceProgress.msg
  msg/TtsTimingEvent.msg
  msg/TtsTimingReceipt.msg
  msg/VoiceMaintenanceState.msg
  srv/FlushAudio.srv
  srv/ListAudioDevices.srv
  srv/QuiesceTts.srv
  srv/SetTtsAdmission.srv
  srv/SetVoiceMaintenance.srv
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interfaces test/test_interfaces.py)
endif()

ament_package()
