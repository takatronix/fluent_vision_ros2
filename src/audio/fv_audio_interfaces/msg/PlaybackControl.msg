uint8 PAUSE=1
uint8 RESUME=2
uint8 DISCARD=3
uint8 ABORT_SYSTEM=4
uint8 ABORT_AGENT=5

uint8 NONE=0
uint8 USER=1
uint8 SYSTEM=2

uint8 action
# PAUSE requires one canonical lowercase UUID. Every other action requires empty.
string pause_id
uint8 release_hold
uint64 minimum_agent_epoch
string utterance_id
