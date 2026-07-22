use thiserror::Error;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Activity {
    pub seq: u64,
    pub sample_index: u64,
    pub frame_count: u32,
    pub state: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Turn {
    pub session_id: String,
    pub user_turn_id: String,
    pub stream_id: String,
    pub seq: u64,
    pub sample_index: u64,
    pub state: &'static str,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum TurnError {
    #[error("session, stream, turn prefix, and silence threshold must be valid")]
    Config,
    #[error("activity frame count and state are invalid")]
    Activity,
    #[error("input seq discontinuity: expected {expected}, got {actual}")]
    Sequence { expected: u64, actual: u64 },
    #[error("input sample discontinuity: expected {expected}, got {actual}")]
    Sample { expected: u64, actual: u64 },
    #[error("final marker is inconsistent with the activity stream")]
    Final,
}

pub struct TurnDetector {
    session_id: String,
    stream_id: String,
    end_silence_frames: u64,
    turn_id_prefix: String,
    next_output_seq: u64,
    next_turn_number: u64,
    previous_input_seq: Option<u64>,
    previous_input_start: Option<u64>,
    previous_input_end: Option<u64>,
    active_turn_id: Option<String>,
    last_speech_end: Option<u64>,
    silence_frames: u64,
}

impl TurnDetector {
    pub fn new(
        session_id: String,
        stream_id: String,
        end_silence_frames: u64,
        turn_id_prefix: String,
    ) -> Result<Self, TurnError> {
        if session_id.is_empty()
            || stream_id.is_empty()
            || turn_id_prefix.is_empty()
            || end_silence_frames == 0
        {
            return Err(TurnError::Config);
        }
        Ok(Self {
            session_id,
            stream_id,
            end_silence_frames,
            turn_id_prefix,
            next_output_seq: 0,
            next_turn_number: 1,
            previous_input_seq: None,
            previous_input_start: None,
            previous_input_end: None,
            active_turn_id: None,
            last_speech_end: None,
            silence_frames: 0,
        })
    }

    pub fn next_output_seq(&self) -> u64 {
        self.next_output_seq
    }

    pub fn push(&mut self, activity: Activity) -> Result<Option<Turn>, TurnError> {
        if activity.frame_count == 0 || !matches!(activity.state.as_str(), "speech" | "silence") {
            return Err(TurnError::Activity);
        }
        if let Some(previous) = self.previous_input_seq
            && activity.seq != previous + 1
        {
            return Err(TurnError::Sequence {
                expected: previous + 1,
                actual: activity.seq,
            });
        }
        if let Some(previous) = self.previous_input_end
            && activity.sample_index != previous
        {
            return Err(TurnError::Sample {
                expected: previous,
                actual: activity.sample_index,
            });
        }
        let end = activity.sample_index + u64::from(activity.frame_count);
        self.previous_input_seq = Some(activity.seq);
        self.previous_input_start = Some(activity.sample_index);
        self.previous_input_end = Some(end);

        if activity.state == "speech" {
            self.silence_frames = 0;
            self.last_speech_end = Some(end);
            let state = if self.active_turn_id.is_none() {
                self.active_turn_id = Some(format!(
                    "{}-{:06}",
                    self.turn_id_prefix, self.next_turn_number
                ));
                self.next_turn_number += 1;
                "started"
            } else {
                "active"
            };
            return Ok(Some(self.event(activity.sample_index, state)));
        }
        if self.active_turn_id.is_none() {
            return Ok(None);
        }
        self.silence_frames += u64::from(activity.frame_count);
        if self.silence_frames < self.end_silence_frames {
            return Ok(None);
        }
        Ok(Some(self.end_turn()))
    }

    pub fn finish(&mut self, sample_index: u64, seq: u64) -> Result<Option<Turn>, TurnError> {
        let Some(previous_seq) = self.previous_input_seq else {
            return Err(TurnError::Final);
        };
        if seq != previous_seq + 1
            || !matches!(
                (self.previous_input_start, self.previous_input_end),
                (Some(start), Some(end)) if start < sample_index && sample_index <= end
            )
        {
            return Err(TurnError::Final);
        }
        if self.active_turn_id.is_none() {
            return Ok(None);
        }
        if let Some(end) = self.last_speech_end.as_mut() {
            *end = (*end).min(sample_index);
        }
        Ok(Some(self.end_turn()))
    }

    fn end_turn(&mut self) -> Turn {
        let sample_index = self
            .last_speech_end
            .expect("active turn must contain speech");
        let event = self.event(sample_index, "ended");
        self.active_turn_id = None;
        self.last_speech_end = None;
        self.silence_frames = 0;
        event
    }

    fn event(&mut self, sample_index: u64, state: &'static str) -> Turn {
        let event = Turn {
            session_id: self.session_id.clone(),
            user_turn_id: self
                .active_turn_id
                .clone()
                .expect("turn event requires active turn"),
            stream_id: self.stream_id.clone(),
            seq: self.next_output_seq,
            sample_index,
            state,
        };
        self.next_output_seq += 1;
        event
    }
}

#[cfg(test)]
mod tests {
    use super::{Activity, TurnDetector};

    fn activity(seq: u64, sample_index: u64, frames: u32, state: &str) -> Activity {
        Activity {
            seq,
            sample_index,
            frame_count: frames,
            state: state.to_owned(),
        }
    }

    #[test]
    fn emits_started_active_and_ends_at_last_speech_sample() {
        let mut detector =
            TurnDetector::new("session".into(), "turns".into(), 1024, "turn".into()).unwrap();
        assert_eq!(detector.push(activity(0, 0, 512, "silence")).unwrap(), None);
        assert_eq!(
            detector
                .push(activity(1, 512, 512, "speech"))
                .unwrap()
                .unwrap()
                .state,
            "started"
        );
        assert_eq!(
            detector
                .push(activity(2, 1024, 512, "speech"))
                .unwrap()
                .unwrap()
                .state,
            "active"
        );
        assert_eq!(
            detector.push(activity(3, 1536, 512, "silence")).unwrap(),
            None
        );
        let ended = detector
            .push(activity(4, 2048, 512, "silence"))
            .unwrap()
            .unwrap();
        assert_eq!(ended.state, "ended");
        assert_eq!(ended.sample_index, 1536);
    }
}
