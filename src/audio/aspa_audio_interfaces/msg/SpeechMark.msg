# Source character offsets are Unicode scalar-value indexes into the exact TTS
# request text. Frame offsets refer to the synthesized PCM before playback
# conversion. A mark becomes heard only after end_frame has physically drained.
uint64 source_start
uint64 source_end
string surface
string pronunciation
uint64 mora_start
uint64 mora_end
uint64 start_frame
uint64 end_frame
