# Utterance-global, end-exclusive boundary in the native TTS PCM stream.
# Text progress becomes audible only after every sample before sample_index has
# physically drained.
uint64 sample_index
uint64 committed_text_tokens
