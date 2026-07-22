# Kind values match PlaybackFrame.
uint8 AGENT=1
uint8 SYSTEM=2
uint8 CUE=3

uint8 AGENT_STARTED=1
uint8 SYSTEM_STARTED=2
uint8 CUE_STARTED=3
uint8 AGENT_PAUSED=4
uint8 AGENT_RESUMED=5
uint8 AGENT_DISCARDED=6
uint8 AGENT_COMPLETED=7
uint8 SYSTEM_COMPLETED=8
uint8 CUE_COMPLETED=9
uint8 SYSTEM_ABORTED=10

uint8 event
uint8 kind
string utterance_id
uint64 played_frames
uint64 total_frames
uint64 playout_generation
