# Zero is invalid so an uninitialized kind cannot become audible.
uint8 AGENT=1
uint8 SYSTEM=2
uint8 CUE=3

uint8 kind
# AGENT and SYSTEM frames carry the exact originating TTS identity. CUE frames
# carry empty request_id and generation_id because they are not synthesized.
string request_id
string generation_id
string utterance_id
uint64 seq
uint64 sample_index
uint32 frame_count
uint32 sample_rate_hz
uint32 channels
uint8[] pcm_s16le
bool final
uint64 playout_generation
