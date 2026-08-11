uint8 AGENT=1
uint8 SYSTEM=2

uint8 AUDIO=1
uint8 COMPLETE=2
uint8 ABORT=3

uint8 kind
uint8 event
string request_id
string generation_id
string utterance_id

# AUDIO messages use a zero-based contiguous sequence and absolute sample
# index. COMPLETE and ABORT carry the next expected values.
uint64 sequence
uint64 first_sample_index
uint32 sample_rate_hz
uint32 channels
float32[] pcm_f32
# True only on final AUDIO. A non-first final AUDIO may contain zero frames as
# a control marker; it advances sequence but not sample or token progress.
# COMPLETE and ABORT carry false. This remains explicit because a zero- or
# eight-frame final cannot be inferred from ordinary PCM chunk sizing.
bool final_chunk

# Native alignment for this AUDIO payload. COMPLETE and ABORT carry zero and
# an empty list.
uint64 committed_text_tokens
fv_audio_interfaces/TtsAlignmentEvent[] alignment_events

# Present only on AUDIO sequence zero. The progress table has one entry for
# every end-exclusive prepared-token position, beginning at position zero.
string source_text
fv_audio_interfaces/TtsSourceProgress[] source_progress
