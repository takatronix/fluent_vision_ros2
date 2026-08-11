# One immutable, complete timing chain. Partial chains are never published.
uint32 TTS_TIMING_RECEIPT_SCHEMA_VERSION=1

uint8 AGENT=1
uint8 SYSTEM=2

uint32 version
uint8 kind
string request_id
string generation_id
string utterance_id
uint64 request_accepted_ns
uint64 frontend_completed_ns
uint64 native_request_started_ns
uint64 first_native_audio_ns
uint64 first_ros_audio_published_ns
uint64 first_playback_frame_published_ns
uint64 physical_playback_started_ns
uint64 physical_playback_ended_ns
uint64 total_underrun_frames
