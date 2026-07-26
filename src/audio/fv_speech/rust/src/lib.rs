pub mod asr;
pub mod audio;
pub mod inference;
pub mod manifest;
#[cfg(feature = "ros2")]
pub mod ros;
pub mod turn;
pub mod vad;

pub const INPUT_SOURCE_ID: &str = "aspa_audio_aec";
pub const INPUT_STREAM_ID: &str = "audio/mic/main";
pub const VAD_SOURCE_ID: &str = "aspa_silero_vad";
pub const VAD_STREAM_ID: &str = "dialogue/vad/main";
pub const TURN_STREAM_ID: &str = "dialogue/turn/main";
pub const TRANSCRIPT_STREAM_ID: &str = "dialogue/asr/main";
pub const SESSION_ID: &str = "aspa-voice";
