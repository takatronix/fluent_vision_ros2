use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use fv_kws_ros2::manifest::RuntimeManifest;
use fv_kws_ros2::segmenter::{
    Activity, CandidateSegmenter, SegmentAction, SegmenterConfig, VAD_WINDOW_SAMPLES,
};
use fv_kws_ros2::spotter::{VoskSpotter, contains_wake_phrase};
use fv_kws_ros2::{
    ACTIVITY_SOURCE_ID, ACTIVITY_STREAM_ID, INPUT_SOURCE_ID, INPUT_STREAM_ID, KWS_SOURCE_ID,
    KWS_STREAM_ID, WAKE_KEYWORD,
};
use r2r::fv_speech_interfaces::msg::{AudioFrame, VoiceActivity, WakeWord};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::{Context, Node, QosProfile};

const DEFAULT_END_SILENCE_SAMPLES: usize = 3_072;
const DEFAULT_PRE_ROLL_SAMPLES: usize = 2_048;
const DEFAULT_MAX_CANDIDATE_SAMPLES: usize = 48_128;
const MAX_UNMATCHED_ACTIVITY: usize = 256;
// Silero loads independently and can begin publishing after microphone capture.
// Retain at most 65.536 seconds so the first absolute VoiceActivity sample index
// can align the streams. Exceeding this bound is a startup failure, not a reason
// to guess an alignment or advertise KWS readiness.
const MAX_PCM_BUFFER_SAMPLES: usize = 1_048_576;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::create()?;
    let mut node = Node::create(context, "fv_kws", "")?;
    let manifest_path = required_path_parameter(&node, "runtime_manifest")?;
    let manifest = RuntimeManifest::load_verified(&manifest_path)?;
    let mut spotter = VoskSpotter::load(&manifest)?;
    let mut segmenter = CandidateSegmenter::new(SegmenterConfig {
        end_silence_samples: sample_count_parameter(
            &node,
            "end_silence_samples",
            DEFAULT_END_SILENCE_SAMPLES,
        )?,
        pre_roll_samples: sample_count_parameter(
            &node,
            "pre_roll_samples",
            DEFAULT_PRE_ROLL_SAMPLES,
        )?,
        max_candidate_samples: sample_count_parameter(
            &node,
            "max_candidate_samples",
            DEFAULT_MAX_CANDIDATE_SAMPLES,
        )?,
    })?;

    let qos = speech_qos(256);
    let mut audio = node.subscribe::<AudioFrame>("/audio/mic/frame", qos.clone())?;
    let mut voice_activity =
        node.subscribe::<VoiceActivity>("/dialogue/vad/activity", qos.clone())?;
    let mut session_state =
        node.subscribe::<r2r::std_msgs::msg::String>("/aspa/dialogue/session_state", state_qos())?;
    let publisher = node.create_publisher::<WakeWord>("/dialogue/kws/wake_word", qos)?;
    let ready_publisher =
        node.create_publisher::<r2r::std_msgs::msg::Bool>("/dialogue/kws/ready", state_qos())?;
    ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;

    let mut dialogue_state = DialogueSessionState::Unknown;
    let mut audio_tracker = AudioTracker::default();
    let mut activity_tracker = ActivityTracker::default();
    let mut joiner = PcmActivityJoiner::default();
    let mut wake_sent = false;
    let mut ready_sent = false;
    let mut output_seq = 0_u64;
    eprintln!(
        "fv_kws model loaded: keyword={WAKE_KEYWORD}, engine=vosk, runtime={}; \
         waiting for dialogue session state",
        manifest_path.display()
    );

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        while let Some(Some(message)) = session_state.next().now_or_never() {
            let next = DialogueSessionState::parse(&message.data)?;
            if next != dialogue_state {
                spotter.reset();
                segmenter.reset();
                audio_tracker.reset();
                activity_tracker.reset();
                joiner.reset();
                wake_sent = false;
                if next == DialogueSessionState::Sleeping {
                    drain_pending(&mut audio);
                    drain_pending(&mut voice_activity);
                }
            }
            if dialogue_state == DialogueSessionState::Unknown {
                eprintln!(
                    "fv_kws session state received: dialogue_session_state={}",
                    next.as_str()
                );
            }
            dialogue_state = next;
        }

        while let Some(Some(message)) = voice_activity.next().now_or_never() {
            if dialogue_state != DialogueSessionState::Sleeping || wake_sent {
                continue;
            }
            if activity_tracker.validate(&message)? {
                eprintln!("fv_kws warning: VoiceActivity stream resynchronized");
                spotter.reset();
                segmenter.reset();
                audio_tracker.reset();
                joiner.reset();
                wake_sent = false;
                ready_sent = false;
                ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;
            }
            if message.final_ {
                ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;
                return Err("KWS VoiceActivity stream ended".into());
            }
            joiner.push_activity(message)?;
        }
        while let Some(Some(message)) = audio.next().now_or_never() {
            if dialogue_state != DialogueSessionState::Sleeping || wake_sent {
                continue;
            }
            if audio_tracker.validate(&message)? {
                eprintln!("fv_kws warning: microphone stream resynchronized");
                spotter.reset();
                segmenter.reset();
                activity_tracker.reset();
                joiner.reset();
                wake_sent = false;
                ready_sent = false;
                ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;
            }
            if message.final_ {
                ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;
                return Err("KWS microphone stream ended".into());
            }
            joiner.push_audio(message)?;
        }

        if dialogue_state != DialogueSessionState::Sleeping || wake_sent {
            joiner.reset();
            continue;
        }
        let ready_windows = joiner.take_ready()?;
        if !ready_windows.is_empty() && !ready_sent {
            ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: true })?;
            ready_sent = true;
            eprintln!("fv_kws ready: microphone and VoiceActivity are aligned");
        }
        for window in ready_windows {
            let activity = match window.decision.state.as_str() {
                "speech" => Activity::Speech,
                "silence" => Activity::Silence,
                value => return Err(format!("invalid KWS VAD state: {value:?}").into()),
            };
            match segmenter.push(activity, window.sample_index, window.samples)? {
                SegmentAction::Buffering => {}
                SegmentAction::Feed(samples) => spotter.accept(&samples)?,
                SegmentAction::Finalize {
                    detected_sample_index,
                } => {
                    let text = spotter.finalize_text();
                    spotter.reset();
                    if contains_wake_phrase(&text) {
                        publisher.publish(&WakeWord {
                            header: window.decision.header,
                            source_id: KWS_SOURCE_ID.to_owned(),
                            stream_id: KWS_STREAM_ID.to_owned(),
                            seq: output_seq,
                            keyword: WAKE_KEYWORD.to_owned(),
                            detected_sample_index,
                        })?;
                        output_seq = output_seq.checked_add(1).ok_or("KWS output seq overflow")?;
                        wake_sent = true;
                        joiner.reset();
                        eprintln!(
                            "wake word accepted: text={text:?}, sample_index={detected_sample_index}"
                        );
                        break;
                    }
                }
                SegmentAction::RejectOverlong => {
                    spotter.reset();
                    eprintln!("KWS candidate rejected: exceeded max_candidate_samples");
                }
            }
        }
    }
    ready_publisher.publish(&r2r::std_msgs::msg::Bool { data: false })?;
    Ok(())
}

fn drain_pending<T, S>(stream: &mut S)
where
    S: futures::Stream<Item = T> + Unpin,
{
    while stream.next().now_or_never().flatten().is_some() {}
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DialogueSessionState {
    Unknown,
    Sleeping,
    Awake,
}

impl DialogueSessionState {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "sleeping" => Ok(Self::Sleeping),
            "awake" => Ok(Self::Awake),
            _ => Err(format!("unsupported dialogue session state: {value:?}")),
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Sleeping => "sleeping",
            Self::Awake => "awake",
        }
    }
}

#[derive(Default)]
struct AudioTracker {
    previous_seq: Option<u64>,
    next_sample_index: Option<u64>,
}

impl AudioTracker {
    fn validate(&mut self, message: &AudioFrame) -> Result<bool, String> {
        if message.source_id != INPUT_SOURCE_ID || message.stream_id != INPUT_STREAM_ID {
            return Err("KWS microphone identity mismatch".to_owned());
        }
        if message.encoding != "PCM16LE"
            || message.sample_rate_hz != 16_000
            || message.channels != 1
            || message.bit_depth != 16
            || message.layout != "interleaved"
        {
            return Err("KWS input must be 16 kHz mono interleaved PCM16LE".to_owned());
        }
        let sequence_gap = self
            .previous_seq
            .is_some_and(|previous| message.seq != previous + 1);
        let sample_gap = self
            .next_sample_index
            .is_some_and(|expected| message.sample_index != expected);
        if message.final_ {
            if message.frame_count != 0 || !message.data.is_empty() {
                return Err("KWS microphone final marker must not contain audio".to_owned());
            }
        } else if message.frame_count == 0 || message.data.len() != message.frame_count as usize * 2
        {
            return Err("KWS microphone frame_count does not match PCM payload".to_owned());
        }
        self.previous_seq = Some(message.seq);
        self.next_sample_index = Some(
            message
                .sample_index
                .checked_add(message.frame_count.into())
                .ok_or("KWS microphone sample index overflow")?,
        );
        Ok(sequence_gap || sample_gap)
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

#[derive(Default)]
struct ActivityTracker {
    previous_seq: Option<u64>,
    next_sample_index: Option<u64>,
}

impl ActivityTracker {
    fn validate(&mut self, message: &VoiceActivity) -> Result<bool, String> {
        if message.source_id != ACTIVITY_SOURCE_ID || message.stream_id != ACTIVITY_STREAM_ID {
            return Err("KWS VoiceActivity identity mismatch".to_owned());
        }
        let sequence_gap = self
            .previous_seq
            .is_some_and(|previous| message.seq != previous + 1);
        let sample_gap = self
            .next_sample_index
            .is_some_and(|expected| message.sample_index != expected);
        if message.final_ {
            if message.frame_count != 0 {
                return Err("KWS VoiceActivity final marker must have zero frames".to_owned());
            }
        } else if message.frame_count as usize != VAD_WINDOW_SAMPLES {
            return Err(format!(
                "KWS VoiceActivity must describe exactly {VAD_WINDOW_SAMPLES} samples"
            ));
        }
        self.previous_seq = Some(message.seq);
        self.next_sample_index = Some(
            message
                .sample_index
                .checked_add(message.frame_count.into())
                .ok_or("KWS VoiceActivity sample index overflow")?,
        );
        Ok(sequence_gap || sample_gap)
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

struct JoinedWindow {
    sample_index: u64,
    samples: Vec<i16>,
    decision: VoiceActivity,
}

#[derive(Default)]
struct PcmActivityJoiner {
    pcm_start_sample_index: Option<u64>,
    pcm: VecDeque<i16>,
    activity: BTreeMap<u64, VoiceActivity>,
}

impl PcmActivityJoiner {
    fn push_audio(&mut self, message: AudioFrame) -> Result<(), String> {
        let samples = pcm16le_to_i16(&message.data)?;
        let expected_start = self
            .pcm_start_sample_index
            .map(|start| start + self.pcm.len() as u64);
        if let Some(expected) = expected_start {
            if message.sample_index != expected {
                return Err(format!(
                    "KWS PCM join discontinuity: expected {expected}, got {}",
                    message.sample_index
                ));
            }
        } else {
            self.pcm_start_sample_index = Some(message.sample_index);
        }
        self.pcm.extend(samples);
        if self.pcm.len() > MAX_PCM_BUFFER_SAMPLES {
            return Err("KWS PCM exceeded the bounded activity join buffer".to_owned());
        }
        Ok(())
    }

    fn push_activity(&mut self, message: VoiceActivity) -> Result<(), String> {
        let sample_index = message.sample_index;
        if self.activity.insert(sample_index, message).is_some() {
            return Err(format!(
                "duplicate KWS VoiceActivity sample index: {sample_index}"
            ));
        }
        if self.activity.len() > MAX_UNMATCHED_ACTIVITY {
            return Err("too many unmatched KWS VoiceActivity windows".to_owned());
        }
        Ok(())
    }

    fn take_ready(&mut self) -> Result<Vec<JoinedWindow>, String> {
        let mut ready = Vec::new();
        loop {
            let Some(activity_index) = self.activity.first_key_value().map(|(index, _)| *index)
            else {
                break;
            };
            let Some(pcm_start) = self.pcm_start_sample_index else {
                break;
            };
            let activity_end = activity_index
                .checked_add(VAD_WINDOW_SAMPLES as u64)
                .ok_or("KWS VoiceActivity sample index overflow")?;
            if activity_end <= pcm_start {
                self.activity.pop_first();
                continue;
            }
            if activity_index < pcm_start {
                self.activity.pop_first();
                continue;
            }
            if pcm_start < activity_index {
                let discard = usize::try_from(activity_index - pcm_start)
                    .map_err(|_| "KWS PCM alignment offset overflow")?;
                if discard > self.pcm.len() {
                    break;
                }
                self.pcm.drain(..discard);
                self.pcm_start_sample_index = Some(activity_index);
            }
            if self.pcm.len() < VAD_WINDOW_SAMPLES {
                break;
            }
            let samples = self.pcm.drain(..VAD_WINDOW_SAMPLES).collect();
            self.pcm_start_sample_index = Some(activity_end);
            let decision = self
                .activity
                .remove(&activity_index)
                .ok_or("KWS VoiceActivity join state corrupted")?;
            ready.push(JoinedWindow {
                sample_index: activity_index,
                samples,
                decision,
            });
        }
        Ok(ready)
    }

    fn reset(&mut self) {
        self.pcm_start_sample_index = None;
        self.pcm.clear();
        self.activity.clear();
    }
}

fn pcm16le_to_i16(payload: &[u8]) -> Result<Vec<i16>, String> {
    if !payload.len().is_multiple_of(2) {
        return Err("KWS PCM payload must contain complete i16 samples".to_owned());
    }
    Ok(payload
        .chunks_exact(2)
        .map(|bytes| i16::from_le_bytes([bytes[0], bytes[1]]))
        .collect())
}

fn speech_qos(depth: usize) -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    }
}

fn state_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    }
}

fn required_path_parameter(node: &Node, name: &str) -> Result<PathBuf, Box<dyn std::error::Error>> {
    let value = node.get_parameter::<String>(name)?;
    let path = PathBuf::from(value);
    if !path.is_absolute() {
        return Err(format!("ROS parameter {name} must be an absolute path").into());
    }
    Ok(path)
}

fn sample_count_parameter(node: &Node, name: &str, default: usize) -> Result<usize, String> {
    let value = node.get_parameter::<i64>(name).unwrap_or(default as i64);
    let value = usize::try_from(value).map_err(|_| format!("{name} must be positive"))?;
    if value == 0 || value % VAD_WINDOW_SAMPLES != 0 {
        return Err(format!(
            "{name} must be a positive multiple of {VAD_WINDOW_SAMPLES}"
        ));
    }
    Ok(value)
}
