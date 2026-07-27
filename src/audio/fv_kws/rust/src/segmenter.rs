use std::collections::VecDeque;

use thiserror::Error;

pub const VAD_WINDOW_SAMPLES: usize = 512;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SegmenterConfig {
    pub end_silence_samples: usize,
    pub pre_roll_samples: usize,
    pub max_candidate_samples: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Activity {
    Speech,
    Silence,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SegmentAction {
    Buffering,
    Feed(Vec<i16>),
    Finalize { detected_sample_index: u64 },
    RejectOverlong,
}

#[derive(Debug, Error)]
pub enum SegmenterError {
    #[error("{name} must be a positive multiple of {VAD_WINDOW_SAMPLES}")]
    InvalidWindowMultiple { name: &'static str },
    #[error("pre_roll_samples must be smaller than max_candidate_samples")]
    InvalidPreRoll,
    #[error("KWS VAD frame must contain exactly {VAD_WINDOW_SAMPLES} samples")]
    InvalidFrameSize,
    #[error("sample index overflow")]
    SampleIndexOverflow,
}

pub struct CandidateSegmenter {
    config: SegmenterConfig,
    pre_roll: VecDeque<Vec<i16>>,
    active: bool,
    candidate_samples: usize,
    silence_samples: usize,
    last_speech_end: u64,
}

impl CandidateSegmenter {
    pub fn new(config: SegmenterConfig) -> Result<Self, SegmenterError> {
        for (name, value) in [
            ("end_silence_samples", config.end_silence_samples),
            ("pre_roll_samples", config.pre_roll_samples),
            ("max_candidate_samples", config.max_candidate_samples),
        ] {
            if value == 0 || value % VAD_WINDOW_SAMPLES != 0 {
                return Err(SegmenterError::InvalidWindowMultiple { name });
            }
        }
        if config.pre_roll_samples >= config.max_candidate_samples {
            return Err(SegmenterError::InvalidPreRoll);
        }
        Ok(Self {
            config,
            pre_roll: VecDeque::new(),
            active: false,
            candidate_samples: 0,
            silence_samples: 0,
            last_speech_end: 0,
        })
    }

    pub fn push(
        &mut self,
        activity: Activity,
        sample_index: u64,
        samples: Vec<i16>,
    ) -> Result<SegmentAction, SegmenterError> {
        if samples.len() != VAD_WINDOW_SAMPLES {
            return Err(SegmenterError::InvalidFrameSize);
        }
        let frame_end = sample_index
            .checked_add(VAD_WINDOW_SAMPLES as u64)
            .ok_or(SegmenterError::SampleIndexOverflow)?;
        if !self.active {
            if activity == Activity::Silence {
                self.push_pre_roll(samples);
                return Ok(SegmentAction::Buffering);
            }
            self.active = true;
            self.silence_samples = 0;
            self.last_speech_end = frame_end;
            let mut initial = Vec::with_capacity(self.config.pre_roll_samples + samples.len());
            for frame in self.pre_roll.drain(..) {
                initial.extend(frame);
            }
            initial.extend(samples);
            self.candidate_samples = initial.len();
            return Ok(SegmentAction::Feed(initial));
        }

        self.candidate_samples += samples.len();
        if self.candidate_samples > self.config.max_candidate_samples {
            self.reset();
            return Ok(SegmentAction::RejectOverlong);
        }
        match activity {
            Activity::Speech => {
                self.silence_samples = 0;
                self.last_speech_end = frame_end;
                Ok(SegmentAction::Feed(samples))
            }
            Activity::Silence => {
                self.silence_samples += samples.len();
                if self.silence_samples >= self.config.end_silence_samples {
                    let detected_sample_index = self.last_speech_end;
                    self.reset();
                    Ok(SegmentAction::Finalize {
                        detected_sample_index,
                    })
                } else {
                    Ok(SegmentAction::Feed(samples))
                }
            }
        }
    }

    pub fn reset(&mut self) {
        self.pre_roll.clear();
        self.active = false;
        self.candidate_samples = 0;
        self.silence_samples = 0;
        self.last_speech_end = 0;
    }

    fn push_pre_roll(&mut self, samples: Vec<i16>) {
        self.pre_roll.push_back(samples);
        while self.pre_roll.len() * VAD_WINDOW_SAMPLES > self.config.pre_roll_samples {
            self.pre_roll.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> SegmenterConfig {
        SegmenterConfig {
            end_silence_samples: 3 * VAD_WINDOW_SAMPLES,
            pre_roll_samples: 2 * VAD_WINDOW_SAMPLES,
            max_candidate_samples: 10 * VAD_WINDOW_SAMPLES,
        }
    }

    fn frame(value: i16) -> Vec<i16> {
        vec![value; VAD_WINDOW_SAMPLES]
    }

    #[test]
    fn includes_bounded_preroll_and_finalizes_at_independent_silence_threshold() {
        let mut segmenter = CandidateSegmenter::new(config()).unwrap();
        assert_eq!(
            segmenter.push(Activity::Silence, 0, frame(1)).unwrap(),
            SegmentAction::Buffering
        );
        assert_eq!(
            segmenter
                .push(Activity::Silence, VAD_WINDOW_SAMPLES as u64, frame(2))
                .unwrap(),
            SegmentAction::Buffering
        );
        let action = segmenter
            .push(Activity::Speech, (2 * VAD_WINDOW_SAMPLES) as u64, frame(3))
            .unwrap();
        let SegmentAction::Feed(samples) = action else {
            panic!("speech must start a candidate");
        };
        assert_eq!(samples.len(), 3 * VAD_WINDOW_SAMPLES);
        assert_eq!(samples[0], 1);
        assert_eq!(samples[VAD_WINDOW_SAMPLES], 2);
        assert_eq!(samples[2 * VAD_WINDOW_SAMPLES], 3);

        for window in 3..5 {
            assert!(matches!(
                segmenter
                    .push(
                        Activity::Silence,
                        (window * VAD_WINDOW_SAMPLES) as u64,
                        frame(0),
                    )
                    .unwrap(),
                SegmentAction::Feed(_)
            ));
        }
        assert_eq!(
            segmenter
                .push(Activity::Silence, (5 * VAD_WINDOW_SAMPLES) as u64, frame(0),)
                .unwrap(),
            SegmentAction::Finalize {
                detected_sample_index: (3 * VAD_WINDOW_SAMPLES) as u64
            }
        );
    }

    #[test]
    fn rejects_overlong_candidate_and_returns_to_idle() {
        let mut segmenter = CandidateSegmenter::new(config()).unwrap();
        for window in 0..=10 {
            let action = segmenter
                .push(
                    Activity::Speech,
                    (window * VAD_WINDOW_SAMPLES) as u64,
                    frame(1),
                )
                .unwrap();
            if action == SegmentAction::RejectOverlong {
                assert_eq!(window, 10);
                return;
            }
        }
        panic!("overlong candidate was not rejected");
    }
}
