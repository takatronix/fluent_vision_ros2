# All timestamps use Linux CLOCK_MONOTONIC nanoseconds. They are comparable
# across processes on one host and are never converted to wall-clock time.
uint32 TTS_TIMING_SCHEMA_VERSION=2

uint8 AGENT=1
uint8 SYSTEM=2

uint8 REQUEST_ACCEPTED=1
uint8 FRONTEND_COMPLETED=2
uint8 NATIVE_REQUEST_STARTED=3
uint8 FIRST_NATIVE_AUDIO=4
uint8 FIRST_ROS_AUDIO_PUBLISHED=5
uint8 FIRST_PLAYBACK_FRAME_PUBLISHED=6
uint8 PHYSICAL_PLAYBACK_STARTED=7
uint8 PHYSICAL_PLAYBACK_ENDED=8
uint8 PLAYBACK_UNDERRUN=9

uint32 version
uint8 stage
uint8 kind
string request_id
string generation_id
string utterance_id
uint64 monotonic_time_ns

# Zero for every required stage. PLAYBACK_UNDERRUN is a repeatable side-band
# event and reports the exact number of output frames filled with silence while
# this non-final TTS utterance was still active.
uint64 underrun_frames
