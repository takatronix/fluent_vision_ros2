use thiserror::Error;
use vosk::{CompleteResult, Model, Recognizer};

use crate::VOSK_GRAMMAR;
use crate::manifest::RuntimeManifest;

const SAMPLE_RATE_HZ: f32 = 16_000.0;

#[derive(Debug, Error)]
pub enum SpotterError {
    #[error("KWS model path is not valid UTF-8: {0}")]
    PathEncoding(String),
    #[error("Vosk failed to load the verified Japanese model")]
    Model,
    #[error("Vosk failed to create the pinned grammar recognizer")]
    Recognizer,
    #[error("Vosk failed to accept PCM: {0}")]
    Accept(#[from] vosk::AcceptWaveformError),
}

pub struct VoskSpotter {
    recognizer: Recognizer,
    _model: Model,
}

impl VoskSpotter {
    pub fn load(manifest: &RuntimeManifest) -> Result<Self, SpotterError> {
        vosk::set_log_level(vosk::LogLevel::Error);
        let path = manifest
            .model_path
            .to_str()
            .ok_or_else(|| SpotterError::PathEncoding(manifest.model_path.display().to_string()))?;
        let model = Model::new(path.to_owned()).ok_or(SpotterError::Model)?;
        let recognizer = Recognizer::new_with_grammar(&model, SAMPLE_RATE_HZ, &VOSK_GRAMMAR)
            .ok_or(SpotterError::Recognizer)?;
        Ok(Self {
            recognizer,
            _model: model,
        })
    }

    pub fn accept(&mut self, samples: &[i16]) -> Result<(), SpotterError> {
        self.recognizer.accept_waveform(samples)?;
        Ok(())
    }

    pub fn finalize_text(&mut self) -> String {
        match self.recognizer.final_result() {
            CompleteResult::Single(result) => result.text.to_owned(),
            CompleteResult::Multiple(_) => {
                unreachable!("max alternatives is never enabled for KWS")
            }
        }
    }

    pub fn reset(&mut self) {
        self.recognizer.reset();
    }
}

pub fn contains_wake_phrase(text: &str) -> bool {
    let tokens = text.split_whitespace().collect::<Vec<_>>();
    tokens.windows(2).any(|pair| pair == ["アス", "パ"])
}

#[cfg(test)]
mod tests {
    use super::contains_wake_phrase;

    #[test]
    fn accepts_the_evaluated_vosk_token_pair_with_leading_acoustic_unknowns() {
        assert!(contains_wake_phrase("アス パ"));
        assert!(contains_wake_phrase("[unk] アス パ"));
        assert!(contains_wake_phrase("[unk] パ アス パ"));
    }

    #[test]
    fn rejects_partial_or_different_words() {
        assert!(!contains_wake_phrase(""));
        assert!(!contains_wake_phrase("[unk]"));
        assert!(!contains_wake_phrase("アス"));
        assert!(!contains_wake_phrase("パ"));
        assert!(!contains_wake_phrase("アス パラ"));
    }
}
