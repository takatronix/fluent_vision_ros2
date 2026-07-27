pub mod manifest;
pub mod segmenter;
pub mod spotter;

pub const INPUT_SOURCE_ID: &str = "fv_audio_aec";
pub const INPUT_STREAM_ID: &str = "audio/mic/main";
pub const ACTIVITY_SOURCE_ID: &str = "aspa_silero_vad";
pub const ACTIVITY_STREAM_ID: &str = "dialogue/vad/main";
pub const KWS_SOURCE_ID: &str = "fv_kws";
pub const KWS_STREAM_ID: &str = "dialogue/kws/main";
pub const WAKE_KEYWORD: &str = "アスパ";
pub const VOSK_GRAMMAR: [&str; 2] = ["アス パ", "[unk]"];
