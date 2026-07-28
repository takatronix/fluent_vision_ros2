use std::collections::VecDeque;

use thiserror::Error;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InferenceCommand {
    Start {
        session_id: String,
        user_turn_id: String,
        stream_id: String,
        start_sample_index: u64,
    },
    Audio(Vec<i16>),
    Finish {
        end_sample_index: u64,
        valid_samples: usize,
    },
    Cancel,
    Shutdown,
}

#[derive(Clone, Debug)]
pub struct BufferedAudio {
    pub sample_index: u64,
    pub samples: Vec<i16>,
}

impl BufferedAudio {
    fn end_sample_index(&self) -> u64 {
        self.sample_index + self.samples.len() as u64
    }
}

#[derive(Clone, Debug)]
struct ActiveTurn {
    session_id: String,
    user_turn_id: String,
    stream_id: String,
    start_sample_index: u64,
    fed_end_sample_index: u64,
    pending_stop_sample_index: Option<u64>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum AsrRuntimeError {
    #[error("ASR configuration is invalid")]
    Config,
    #[error("ASR audio history is not contiguous")]
    AudioGap,
    #[error("ASR control seq discontinuity: expected {expected}, got {actual}")]
    ControlSequence { expected: u64, actual: u64 },
    #[error("ASR control identity does not match the input stream or active turn")]
    Identity,
    #[error("ASR start requires retained audio and no active turn")]
    Start,
    #[error("ASR stop is invalid for the active turn")]
    Stop,
    #[error("unsupported ASR control action {0}")]
    Action(String),
}

pub struct AsrRuntime {
    input_stream_id: String,
    history_frames: u64,
    history: VecDeque<BufferedAudio>,
    active: Option<ActiveTurn>,
    resynchronized_turn: Option<ActiveTurn>,
    expected_control_seq: u64,
}

impl AsrRuntime {
    pub fn new(input_stream_id: String, history_frames: u64) -> Result<Self, AsrRuntimeError> {
        if input_stream_id.is_empty() || history_frames == 0 {
            return Err(AsrRuntimeError::Config);
        }
        Ok(Self {
            input_stream_id,
            history_frames,
            history: VecDeque::new(),
            active: None,
            resynchronized_turn: None,
            expected_control_seq: 0,
        })
    }

    pub fn latest_sample_index(&self) -> Option<u64> {
        self.history.back().map(BufferedAudio::end_sample_index)
    }

    /// Flushes audio-clock-dependent state while preserving the independent
    /// ASR control sequence. Any in-flight GPU inference is explicitly
    /// cancelled before audio from the new clock epoch is accepted.
    pub fn resynchronize_audio(&mut self) -> Vec<InferenceCommand> {
        self.history.clear();
        self.resynchronized_turn = self.active.take();
        if self.resynchronized_turn.is_some() {
            vec![InferenceCommand::Cancel]
        } else {
            Vec::new()
        }
    }

    pub fn push_audio(
        &mut self,
        chunk: BufferedAudio,
    ) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        if chunk.samples.is_empty()
            || self
                .history
                .back()
                .is_some_and(|previous| chunk.sample_index != previous.end_sample_index())
        {
            return Err(AsrRuntimeError::AudioGap);
        }
        let chunk_end = chunk.end_sample_index();
        self.history.push_back(chunk);
        self.prune_history();
        let Some(active) = self.active.as_ref() else {
            return Ok(Vec::new());
        };
        let mut feed_end = chunk_end;
        if let Some(stop) = active.pending_stop_sample_index {
            feed_end = feed_end.min(stop);
        }
        let feed_start = active.fed_end_sample_index;
        let pending_stop = active.pending_stop_sample_index;
        let commands = self.feed_span(feed_start, feed_end)?;
        self.active.as_mut().unwrap().fed_end_sample_index = feed_end;
        if pending_stop.is_some_and(|stop| feed_end >= stop) {
            let stop = pending_stop.unwrap();
            let mut result = commands;
            result.push(self.finish_command(stop)?);
            self.active = None;
            return Ok(result);
        }
        Ok(commands)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn push_control(
        &mut self,
        action: &str,
        session_id: &str,
        user_turn_id: &str,
        stream_id: &str,
        seq: u64,
        start_sample_index: u64,
        stop_sample_index: u64,
    ) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        if seq != self.expected_control_seq {
            return Err(AsrRuntimeError::ControlSequence {
                expected: self.expected_control_seq,
                actual: seq,
            });
        }
        self.expected_control_seq += 1;
        if session_id.is_empty() || user_turn_id.is_empty() || stream_id != self.input_stream_id {
            return Err(AsrRuntimeError::Identity);
        }
        if self.active.is_none()
            && matches!(action, "stop" | "cancel")
            && self.resynchronized_turn.as_ref().is_some_and(|turn| {
                (
                    turn.session_id.as_str(),
                    turn.user_turn_id.as_str(),
                    turn.stream_id.as_str(),
                ) == (session_id, user_turn_id, stream_id)
            })
        {
            self.resynchronized_turn = None;
            return Ok(Vec::new());
        }
        match action {
            "start" => self.start(session_id, user_turn_id, stream_id, start_sample_index),
            "stop" => self.stop(session_id, user_turn_id, stream_id, stop_sample_index),
            "cancel" => self.cancel(session_id, user_turn_id, stream_id),
            other => Err(AsrRuntimeError::Action(other.to_owned())),
        }
    }

    fn start(
        &mut self,
        session_id: &str,
        user_turn_id: &str,
        stream_id: &str,
        start_sample_index: u64,
    ) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        if self.active.is_some() || self.history.is_empty() {
            return Err(AsrRuntimeError::Start);
        }
        let earliest = self.history.front().unwrap().sample_index;
        let latest = self.latest_sample_index().unwrap();
        if !(earliest..=latest).contains(&start_sample_index) {
            return Err(AsrRuntimeError::Start);
        }
        self.active = Some(ActiveTurn {
            session_id: session_id.to_owned(),
            user_turn_id: user_turn_id.to_owned(),
            stream_id: stream_id.to_owned(),
            start_sample_index,
            fed_end_sample_index: start_sample_index,
            pending_stop_sample_index: None,
        });
        let mut commands = vec![InferenceCommand::Start {
            session_id: session_id.to_owned(),
            user_turn_id: user_turn_id.to_owned(),
            stream_id: stream_id.to_owned(),
            start_sample_index,
        }];
        commands.extend(self.feed_span(start_sample_index, latest)?);
        self.active.as_mut().unwrap().fed_end_sample_index = latest;
        Ok(commands)
    }

    fn stop(
        &mut self,
        session_id: &str,
        user_turn_id: &str,
        stream_id: &str,
        stop_sample_index: u64,
    ) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        self.require_identity(session_id, user_turn_id, stream_id)?;
        let active = self.active.as_ref().unwrap();
        if stop_sample_index < active.start_sample_index {
            return Err(AsrRuntimeError::Stop);
        }
        let latest = self.latest_sample_index().ok_or(AsrRuntimeError::Stop)?;
        if stop_sample_index > latest {
            self.active.as_mut().unwrap().pending_stop_sample_index = Some(stop_sample_index);
            return Ok(Vec::new());
        }
        let command = self.finish_command(stop_sample_index)?;
        self.active = None;
        Ok(vec![command])
    }

    fn cancel(
        &mut self,
        session_id: &str,
        user_turn_id: &str,
        stream_id: &str,
    ) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        self.require_identity(session_id, user_turn_id, stream_id)?;
        self.active = None;
        Ok(vec![InferenceCommand::Cancel])
    }

    fn require_identity(
        &self,
        session_id: &str,
        user_turn_id: &str,
        stream_id: &str,
    ) -> Result<(), AsrRuntimeError> {
        let Some(active) = self.active.as_ref() else {
            return Err(AsrRuntimeError::Identity);
        };
        if (
            active.session_id.as_str(),
            active.user_turn_id.as_str(),
            active.stream_id.as_str(),
        ) != (session_id, user_turn_id, stream_id)
        {
            return Err(AsrRuntimeError::Identity);
        }
        Ok(())
    }

    fn finish_command(&self, stop_sample_index: u64) -> Result<InferenceCommand, AsrRuntimeError> {
        let active = self.active.as_ref().ok_or(AsrRuntimeError::Stop)?;
        if stop_sample_index > active.fed_end_sample_index {
            return Err(AsrRuntimeError::Stop);
        }
        Ok(InferenceCommand::Finish {
            end_sample_index: stop_sample_index,
            valid_samples: (stop_sample_index - active.start_sample_index) as usize,
        })
    }

    fn feed_span(&self, start: u64, end: u64) -> Result<Vec<InferenceCommand>, AsrRuntimeError> {
        if end <= start {
            return Ok(Vec::new());
        }
        let mut samples = Vec::with_capacity((end - start) as usize);
        let mut next = start;
        for chunk in &self.history {
            if chunk.end_sample_index() <= start {
                continue;
            }
            if chunk.sample_index >= end {
                break;
            }
            let slice_start = start.max(chunk.sample_index);
            let slice_end = end.min(chunk.end_sample_index());
            if slice_start != next {
                return Err(AsrRuntimeError::AudioGap);
            }
            samples.extend_from_slice(
                &chunk.samples[(slice_start - chunk.sample_index) as usize
                    ..(slice_end - chunk.sample_index) as usize],
            );
            next = slice_end;
        }
        if next != end {
            return Err(AsrRuntimeError::AudioGap);
        }
        Ok(vec![InferenceCommand::Audio(samples)])
    }

    fn prune_history(&mut self) {
        let Some(latest) = self.latest_sample_index() else {
            return;
        };
        let retain_from = latest.saturating_sub(self.history_frames);
        while self.history.len() > 1
            && self
                .history
                .front()
                .is_some_and(|chunk| chunk.end_sample_index() <= retain_from)
        {
            self.history.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{AsrRuntime, BufferedAudio, InferenceCommand};

    #[test]
    fn replays_exact_prebuffer_and_trims_final() {
        let mut runtime = AsrRuntime::new("mic".into(), 32_000).unwrap();
        runtime
            .push_audio(BufferedAudio {
                sample_index: 0,
                samples: (0..160).collect(),
            })
            .unwrap();
        let commands = runtime
            .push_control("start", "session", "turn", "mic", 0, 40, 0)
            .unwrap();
        assert!(matches!(commands[0], InferenceCommand::Start { .. }));
        assert!(matches!(&commands[1], InferenceCommand::Audio(samples) if samples.len() == 120));
        let commands = runtime
            .push_control("stop", "session", "turn", "mic", 1, 0, 100)
            .unwrap();
        assert_eq!(
            commands,
            [InferenceCommand::Finish {
                end_sample_index: 100,
                valid_samples: 60,
            }]
        );
    }

    #[test]
    fn waits_for_audio_when_stop_arrives_first() {
        let mut runtime = AsrRuntime::new("mic".into(), 32_000).unwrap();
        runtime
            .push_audio(BufferedAudio {
                sample_index: 0,
                samples: vec![0; 160],
            })
            .unwrap();
        runtime
            .push_control("start", "session", "turn", "mic", 0, 0, 0)
            .unwrap();
        assert!(
            runtime
                .push_control("stop", "session", "turn", "mic", 1, 0, 200)
                .unwrap()
                .is_empty()
        );
        let commands = runtime
            .push_audio(BufferedAudio {
                sample_index: 160,
                samples: vec![0; 160],
            })
            .unwrap();
        assert!(matches!(
            commands.last(),
            Some(InferenceCommand::Finish {
                end_sample_index: 200,
                ..
            })
        ));
    }

    #[test]
    fn audio_resync_cancels_inference_and_absorbs_the_old_stop() {
        let mut runtime = AsrRuntime::new("mic".into(), 32_000).unwrap();
        runtime
            .push_audio(BufferedAudio {
                sample_index: 0,
                samples: vec![0; 160],
            })
            .unwrap();
        runtime
            .push_control("start", "session", "turn", "mic", 0, 0, 0)
            .unwrap();
        assert_eq!(runtime.resynchronize_audio(), [InferenceCommand::Cancel]);
        runtime
            .push_audio(BufferedAudio {
                sample_index: 10_000,
                samples: vec![0; 160],
            })
            .unwrap();
        assert!(
            runtime
                .push_control("stop", "session", "turn", "mic", 1, 0, 10_000)
                .unwrap()
                .is_empty()
        );
    }
}
