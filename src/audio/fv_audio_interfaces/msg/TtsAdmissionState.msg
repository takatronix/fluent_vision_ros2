# Dialogue's fail-closed admission projection for one TTS runtime generation.
uint32 TTS_ADMISSION_SCHEMA_VERSION=1

uint32 version
string controller_generation_id
uint64 heartbeat_sequence
bool generation_valid
string generation_id
bool admission_open

# Empty exactly when admission_open=true. A closed projection always carries
# the reason that prevents new TTS requests from entering the runtime.
string reason
