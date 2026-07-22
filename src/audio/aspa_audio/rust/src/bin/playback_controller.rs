use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{FutureExt, Stream, StreamExt};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app::{AppSink, AppSrc};
use r2r::aspa_audio_interfaces::msg::{
    OutputDrained as OutputDrainedMessage, PlaybackControl as PlaybackControlMessage,
    PlaybackEvent as PlaybackEventMessage, PlaybackFrame,
};
use r2r::aspa_audio_interfaces::srv::FlushAudio;
use r2r::fv_speech_interfaces::msg::AudioFrame;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const MAX_ABORTED_SYSTEM_IDS: usize = 64;
const MAX_SAFE_PLAYOUT_GENERATION: u64 = (1_u64 << 53) - 1;
const STATUS_ACCEPTED: u8 = OutputDrainedMessage::ACCEPTED as u8;
const STATUS_DRAINED: u8 = OutputDrainedMessage::DRAINED as u8;
const STATUS_FLUSHED: u8 = OutputDrainedMessage::FLUSHED as u8;
const CONTROL_MESSAGES_PER_TICK: usize = 32;
const ACK_MESSAGES_PER_TICK: usize = 32;
const SYSTEM_MESSAGES_PER_TICK: usize = 16;
const CUE_MESSAGES_PER_TICK: usize = 8;
const AGENT_MESSAGES_PER_TICK: usize = 8;
const CONTROL_QOS_DEPTH: usize = 10;
const PLAYOUT_QOS_DEPTH: usize = 256;
const OUTPUT_PROGRESS_TIMEOUT: Duration = Duration::from_secs(5);
const OUTPUT_FLUSH_TIMEOUT: Duration = Duration::from_secs(5);
const PCM_CONVERSION_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Debug, Error)]
enum ControllerError {
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("playback output path failed: {0}")]
    Output(String),
    #[error("playback controller invariant failed: {0}")]
    Invariant(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Kind {
    Agent,
    System,
    Cue,
}

impl Kind {
    fn code(self) -> u8 {
        match self {
            Self::Agent => PlaybackFrame::AGENT as u8,
            Self::System => PlaybackFrame::SYSTEM as u8,
            Self::Cue => PlaybackFrame::CUE as u8,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EventName {
    AgentStarted,
    SystemStarted,
    CueStarted,
    AgentPaused,
    AgentResumed,
    AgentDiscarded,
    AgentCompleted,
    SystemCompleted,
    CueCompleted,
    SystemAborted,
}

impl EventName {
    fn code(self) -> u8 {
        match self {
            Self::AgentStarted => PlaybackEventMessage::AGENT_STARTED as u8,
            Self::SystemStarted => PlaybackEventMessage::SYSTEM_STARTED as u8,
            Self::CueStarted => PlaybackEventMessage::CUE_STARTED as u8,
            Self::AgentPaused => PlaybackEventMessage::AGENT_PAUSED as u8,
            Self::AgentResumed => PlaybackEventMessage::AGENT_RESUMED as u8,
            Self::AgentDiscarded => PlaybackEventMessage::AGENT_DISCARDED as u8,
            Self::AgentCompleted => PlaybackEventMessage::AGENT_COMPLETED as u8,
            Self::SystemCompleted => PlaybackEventMessage::SYSTEM_COMPLETED as u8,
            Self::CueCompleted => PlaybackEventMessage::CUE_COMPLETED as u8,
            Self::SystemAborted => PlaybackEventMessage::SYSTEM_ABORTED as u8,
        }
    }

    fn kind(self) -> Kind {
        match self {
            Self::AgentStarted
            | Self::AgentPaused
            | Self::AgentResumed
            | Self::AgentDiscarded
            | Self::AgentCompleted => Kind::Agent,
            Self::SystemStarted | Self::SystemCompleted | Self::SystemAborted => Kind::System,
            Self::CueStarted | Self::CueCompleted => Kind::Cue,
        }
    }

    fn invalidates(self) -> Option<Kind> {
        match self {
            Self::AgentPaused | Self::AgentDiscarded => Some(Kind::Agent),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PlaybackEvent {
    name: EventName,
    kind: Kind,
    utterance_id: String,
    played_frames: u64,
    total_frames: u64,
}

impl PlaybackEvent {
    fn fallback(name: EventName, utterance_id: impl Into<String>) -> Self {
        Self {
            name,
            kind: name.kind(),
            utterance_id: utterance_id.into(),
            played_frames: 0,
            total_frames: 0,
        }
    }

    fn to_message(&self, playout_generation: u64) -> PlaybackEventMessage {
        PlaybackEventMessage {
            event: self.name.code(),
            kind: self.kind.code(),
            utterance_id: self.utterance_id.clone(),
            played_frames: self.played_frames,
            total_frames: self.total_frames,
            playout_generation,
        }
    }
}

#[derive(Debug)]
struct Utterance {
    kind: Kind,
    utterance_id: String,
    pcm: Vec<u8>,
    sample_rate_hz: u32,
    channels: u32,
    bit_depth: u32,
    offset_bytes: usize,
    played_offset_frames: u64,
    started: bool,
    start_confirmed: bool,
}

impl Utterance {
    fn new(
        kind: Kind,
        utterance_id: String,
        pcm: Vec<u8>,
        sample_rate_hz: u32,
        channels: u32,
        bit_depth: u32,
    ) -> Result<Self, String> {
        if utterance_id.is_empty() {
            return Err("utterance_id must not be empty".to_owned());
        }
        if sample_rate_hz == 0 || channels == 0 || bit_depth != 16 {
            return Err("playback requires positive rate/channels and PCM16".to_owned());
        }
        if pcm.is_empty() {
            return Err("playback PCM must contain at least one audio frame".to_owned());
        }
        let bytes_per_frame = channels as usize * 2;
        if !pcm.len().is_multiple_of(bytes_per_frame) {
            return Err("PCM payload is not aligned to audio frames".to_owned());
        }
        Ok(Self {
            kind,
            utterance_id,
            pcm,
            sample_rate_hz,
            channels,
            bit_depth,
            offset_bytes: 0,
            played_offset_frames: 0,
            started: false,
            start_confirmed: false,
        })
    }

    fn bytes_per_frame(&self) -> usize {
        self.channels as usize * (self.bit_depth as usize / 8)
    }

    fn total_frames(&self) -> u64 {
        (self.pcm.len() / self.bytes_per_frame()) as u64
    }
}

type UtteranceRef = Rc<RefCell<Utterance>>;

#[derive(Clone, Debug, PartialEq, Eq)]
struct PlaybackChunk {
    utterance_id: String,
    kind: Kind,
    seq: u64,
    sample_index: u64,
    frame_count: u32,
    sample_rate_hz: u32,
    channels: u32,
    data: Vec<u8>,
    is_final: bool,
}

#[derive(Default)]
struct DeviceBufferTracker {
    kind: Option<Kind>,
    utterance_id: Option<String>,
}

impl DeviceBufferTracker {
    fn note_chunk(&mut self, kind: Kind, utterance_id: &str) {
        self.kind = Some(kind);
        self.utterance_id = Some(utterance_id.to_owned());
    }

    fn note_ack(&mut self, utterance_id: &str, is_final: bool, status: u8) {
        if self.utterance_id.as_deref() != Some(utterance_id) {
            return;
        }
        if is_final || status == STATUS_FLUSHED {
            self.kind = None;
            self.utterance_id = None;
        }
    }

    fn matches(&self, target_kind: Kind, target_utterance_id: Option<&str>) -> bool {
        self.kind == Some(target_kind)
            && target_utterance_id.is_none_or(|target| self.utterance_id.as_deref() == Some(target))
    }

    fn clear_matching(&mut self, target_kind: Kind, target_utterance_id: Option<&str>) {
        if self.matches(target_kind, target_utterance_id) {
            self.kind = None;
            self.utterance_id = None;
        }
    }
}

struct PlaybackMachine {
    chunk_frames: u64,
    agent_queue: VecDeque<UtteranceRef>,
    system_queue: VecDeque<UtteranceRef>,
    cue_queue: VecDeque<UtteranceRef>,
    active: Option<UtteranceRef>,
    paused_agent: Option<UtteranceRef>,
    user_hold: bool,
    system_holds: HashSet<String>,
    aborted_system_ids: HashSet<String>,
    aborted_system_order: VecDeque<String>,
    awaiting_drain: HashMap<String, UtteranceRef>,
}

impl PlaybackMachine {
    fn new(chunk_frames: u64) -> Result<Self, String> {
        if chunk_frames == 0 {
            return Err("chunk_frames must be positive".to_owned());
        }
        Ok(Self {
            chunk_frames,
            agent_queue: VecDeque::new(),
            system_queue: VecDeque::new(),
            cue_queue: VecDeque::new(),
            active: None,
            paused_agent: None,
            user_hold: false,
            system_holds: HashSet::new(),
            aborted_system_ids: HashSet::new(),
            aborted_system_order: VecDeque::new(),
            awaiting_drain: HashMap::new(),
        })
    }

    fn agent_pause_requested(&self) -> bool {
        self.user_hold || !self.system_holds.is_empty()
    }

    fn enqueue(&mut self, utterance: Utterance) -> Result<Vec<PlaybackEvent>, String> {
        let item = Rc::new(RefCell::new(utterance));
        let kind = item.borrow().kind;
        let utterance_id = item.borrow().utterance_id.clone();
        let mut events = Vec::new();
        match kind {
            Kind::System => {
                if self.aborted_system_ids.contains(&utterance_id) {
                    return Ok(vec![Self::event(EventName::SystemAborted, &item)]);
                }
                events.extend(self.reserve_system(&utterance_id)?);
                self.system_queue.push_back(item);
            }
            Kind::Cue => self.cue_queue.push_back(item),
            Kind::Agent => self.agent_queue.push_back(item),
        }
        Ok(events)
    }

    fn reserve_system(&mut self, utterance_id: &str) -> Result<Vec<PlaybackEvent>, String> {
        if utterance_id.is_empty() {
            return Err("system reservation requires utterance_id".to_owned());
        }
        if self.aborted_system_ids.contains(utterance_id)
            || self.system_holds.contains(utterance_id)
        {
            return Ok(Vec::new());
        }
        self.system_holds.insert(utterance_id.to_owned());
        Ok(self.pause_active_agent())
    }

    fn system_is_aborted(&self, utterance_id: &str) -> bool {
        self.aborted_system_ids.contains(utterance_id)
    }

    fn pause_agent(&mut self) -> Vec<PlaybackEvent> {
        self.user_hold = true;
        self.pause_active_agent()
    }

    fn resume_agent(&mut self) -> Vec<PlaybackEvent> {
        self.user_hold = false;
        if self.agent_pause_requested() {
            return Vec::new();
        }
        let Some(paused) = self.paused_agent.take() else {
            return Vec::new();
        };
        self.agent_queue.push_front(Rc::clone(&paused));
        vec![Self::event(EventName::AgentResumed, &paused)]
    }

    fn discard_agent(
        &mut self,
        release_hold: Kind,
        system_utterance_id: Option<&str>,
        minimum_agent_epoch: Option<u64>,
    ) -> Result<Vec<PlaybackEvent>, String> {
        if !matches!(release_hold, Kind::Agent | Kind::System) {
            return Err("release_hold must be user or system".to_owned());
        }
        if release_hold == Kind::System && system_utterance_id.is_none() {
            return Err("system hold release requires utterance_id".to_owned());
        }
        if release_hold == Kind::Agent && system_utterance_id.is_some() {
            return Err("user hold release must not specify utterance_id".to_owned());
        }

        let stale = |item: &UtteranceRef| -> Result<bool, String> {
            minimum_agent_epoch.map_or(Ok(true), |minimum| {
                Ok(agent_epoch(&item.borrow().utterance_id)? < minimum)
            })
        };
        let mut discarded = Vec::new();
        if self
            .active
            .as_ref()
            .is_some_and(|item| item.borrow().kind == Kind::Agent)
            && stale(self.active.as_ref().expect("checked above"))?
        {
            discarded.push(self.active.take().expect("checked above"));
        }
        if let Some(item) = self.paused_agent.as_ref() {
            if stale(item)? {
                discarded.push(self.paused_agent.take().expect("checked above"));
            }
        }
        let mut kept = VecDeque::new();
        while let Some(item) = self.agent_queue.pop_front() {
            if stale(&item)? {
                discarded.push(item);
            } else {
                kept.push_back(item);
            }
        }
        self.agent_queue = kept;
        let draining_ids = self.awaiting_drain.keys().cloned().collect::<Vec<_>>();
        for utterance_id in draining_ids {
            let item = self
                .awaiting_drain
                .get(&utterance_id)
                .expect("key snapshot");
            if item.borrow().kind == Kind::Agent && stale(item)? {
                discarded.push(
                    self.awaiting_drain
                        .remove(&utterance_id)
                        .expect("key snapshot"),
                );
            }
        }
        if release_hold == Kind::Agent {
            self.user_hold = false;
        } else if let Some(utterance_id) = system_utterance_id {
            self.system_holds.remove(utterance_id);
        }
        let mut events = discarded
            .iter()
            .map(|item| Self::event(EventName::AgentDiscarded, item))
            .collect::<Vec<_>>();
        if !self.agent_pause_requested() {
            if let Some(paused) = self.paused_agent.take() {
                self.agent_queue.push_front(Rc::clone(&paused));
                events.push(Self::event(EventName::AgentResumed, &paused));
            }
        }
        Ok(events)
    }

    fn abort_system(
        &mut self,
        utterance_id: &str,
        release_hold: bool,
    ) -> Result<Vec<PlaybackEvent>, String> {
        if utterance_id.is_empty() {
            return Err("system abort requires utterance_id".to_owned());
        }
        self.remember_aborted_system(utterance_id);
        let mut events = Vec::new();
        if self.active.as_ref().is_some_and(|item| {
            let item = item.borrow();
            item.kind == Kind::System && item.utterance_id == utterance_id
        }) {
            let active = self.active.take().expect("checked above");
            events.push(Self::event(EventName::SystemAborted, &active));
        }
        let mut queued = VecDeque::new();
        while let Some(item) = self.system_queue.pop_front() {
            if item.borrow().utterance_id == utterance_id {
                events.push(Self::event(EventName::SystemAborted, &item));
            } else {
                queued.push_back(item);
            }
        }
        self.system_queue = queued;
        if let Some(draining) = self.awaiting_drain.remove(utterance_id) {
            if draining.borrow().kind == Kind::System {
                events.push(Self::event(EventName::SystemAborted, &draining));
            } else {
                self.awaiting_drain
                    .insert(utterance_id.to_owned(), draining);
            }
        }
        if release_hold {
            self.system_holds.remove(utterance_id);
        }
        if !self.agent_pause_requested() {
            if let Some(paused) = self.paused_agent.take() {
                self.agent_queue.push_front(Rc::clone(&paused));
                events.push(Self::event(EventName::AgentResumed, &paused));
            }
        }
        Ok(events)
    }

    fn remember_aborted_system(&mut self, utterance_id: &str) {
        if self.aborted_system_ids.contains(utterance_id) {
            return;
        }
        if self.aborted_system_order.len() >= MAX_ABORTED_SYSTEM_IDS {
            if let Some(oldest) = self.aborted_system_order.pop_front() {
                self.aborted_system_ids.remove(&oldest);
            }
        }
        self.aborted_system_ids.insert(utterance_id.to_owned());
        self.aborted_system_order.push_back(utterance_id.to_owned());
    }

    fn next_chunk(&mut self) -> Result<Option<PlaybackChunk>, String> {
        if self.active.is_none() {
            self.active = self.select_next();
        }
        let Some(item_ref) = self.active.as_ref().cloned() else {
            return Ok(None);
        };
        let mut item = item_ref.borrow_mut();
        item.started = true;
        let bytes_per_frame = item.bytes_per_frame();
        let start = item.offset_bytes;
        let chunk_bytes = self.chunk_frames as usize * bytes_per_frame;
        let end = item.pcm.len().min(start.saturating_add(chunk_bytes));
        let data = item.pcm[start..end].to_vec();
        let sample_index = (item.offset_bytes / bytes_per_frame) as u64;
        item.offset_bytes = end;
        let is_final = end == item.pcm.len();
        let chunk = PlaybackChunk {
            utterance_id: item.utterance_id.clone(),
            kind: item.kind,
            seq: sample_index / self.chunk_frames,
            sample_index,
            frame_count: u32::try_from(data.len() / bytes_per_frame)
                .map_err(|_| "playback chunk exceeds uint32 frame count".to_owned())?,
            sample_rate_hz: item.sample_rate_hz,
            channels: item.channels,
            data,
            is_final,
        };
        let utterance_id = item.utterance_id.clone();
        drop(item);
        if is_final {
            self.awaiting_drain
                .insert(utterance_id, Rc::clone(&item_ref));
            self.active = None;
        }
        Ok(Some(chunk))
    }

    fn output_accepted(
        &mut self,
        utterance_id: &str,
        seq: u64,
    ) -> Result<Vec<PlaybackEvent>, String> {
        let item = self.find_item(utterance_id)?;
        if seq != 0 || item.borrow().start_confirmed {
            return Ok(Vec::new());
        }
        item.borrow_mut().start_confirmed = true;
        let name = started_event(item.borrow().kind);
        Ok(vec![Self::event(name, &item)])
    }

    fn output_drained(
        &mut self,
        utterance_id: &str,
        seq: u64,
        frame_count: u32,
        is_final: bool,
        status: u8,
    ) -> Result<Vec<PlaybackEvent>, String> {
        let item = self.find_item(utterance_id)?;
        let expected_seq = item.borrow().played_offset_frames / self.chunk_frames;
        if seq != expected_seq {
            return Err(format!(
                "non-contiguous output ack: {seq} != {expected_seq}"
            ));
        }
        {
            let mut value = item.borrow_mut();
            value.played_offset_frames = value
                .played_offset_frames
                .checked_add(u64::from(frame_count))
                .ok_or_else(|| "output ack frame count overflowed".to_owned())?;
            if value.played_offset_frames > value.total_frames() {
                return Err("output ack exceeds utterance length".to_owned());
            }
        }
        let queued_resume = self
            .paused_agent
            .as_ref()
            .is_some_and(|candidate| Rc::ptr_eq(candidate, &item))
            || (item.borrow().kind == Kind::Agent
                && self
                    .agent_queue
                    .iter()
                    .any(|candidate| Rc::ptr_eq(candidate, &item)));
        if queued_resume {
            let mut value = item.borrow_mut();
            value.offset_bytes = value.played_offset_frames as usize * value.bytes_per_frame();
        }
        let expected_final = item.borrow().played_offset_frames == item.borrow().total_frames();
        if status == STATUS_DRAINED && is_final != expected_final {
            return Err("output ack final flag does not match drained length".to_owned());
        }
        if status == STATUS_FLUSHED && (is_final || expected_final) {
            return Err("flushed output ack must describe a partial chunk".to_owned());
        }
        if status == STATUS_FLUSHED
            && self
                .paused_agent
                .as_ref()
                .is_some_and(|candidate| Rc::ptr_eq(candidate, &item))
        {
            return Ok(vec![Self::event(EventName::AgentPaused, &item)]);
        }
        if !is_final {
            return Ok(Vec::new());
        }
        self.awaiting_drain.remove(utterance_id);
        if self
            .paused_agent
            .as_ref()
            .is_some_and(|candidate| Rc::ptr_eq(candidate, &item))
        {
            self.paused_agent = None;
        }
        let name = completed_event(item.borrow().kind);
        Ok(vec![Self::event(name, &item)])
    }

    fn select_next(&mut self) -> Option<UtteranceRef> {
        self.system_queue
            .pop_front()
            .or_else(|| self.cue_queue.pop_front())
            .or_else(|| {
                (!self.agent_pause_requested())
                    .then(|| self.agent_queue.pop_front())
                    .flatten()
            })
    }

    fn pause_active_agent(&mut self) -> Vec<PlaybackEvent> {
        let mut paused = self
            .active
            .as_ref()
            .filter(|item| item.borrow().kind == Kind::Agent)
            .cloned();
        if paused.is_none() {
            let draining_id = self.awaiting_drain.iter().find_map(|(utterance_id, item)| {
                (item.borrow().kind == Kind::Agent).then(|| utterance_id.clone())
            });
            if let Some(utterance_id) = draining_id {
                paused = self.awaiting_drain.remove(&utterance_id);
            }
        }
        let Some(paused) = paused else {
            return Vec::new();
        };
        if self
            .active
            .as_ref()
            .is_some_and(|active| Rc::ptr_eq(active, &paused))
        {
            self.active = None;
        }
        {
            let mut item = paused.borrow_mut();
            item.offset_bytes = item.played_offset_frames as usize * item.bytes_per_frame();
        }
        self.paused_agent = Some(Rc::clone(&paused));
        vec![Self::event(EventName::AgentPaused, &paused)]
    }

    fn find_item(&self, utterance_id: &str) -> Result<UtteranceRef, String> {
        let direct = self
            .active
            .iter()
            .chain(self.paused_agent.iter())
            .chain(self.awaiting_drain.get(utterance_id));
        let queued = self
            .agent_queue
            .iter()
            .chain(self.system_queue.iter())
            .chain(self.cue_queue.iter());
        direct
            .chain(queued)
            .find(|item| item.borrow().utterance_id == utterance_id)
            .cloned()
            .ok_or_else(|| format!("output ack references unknown utterance: {utterance_id}"))
    }

    fn event(name: EventName, item: &UtteranceRef) -> PlaybackEvent {
        let item = item.borrow();
        PlaybackEvent {
            name,
            kind: item.kind,
            utterance_id: item.utterance_id.clone(),
            played_frames: item.played_offset_frames,
            total_frames: item.total_frames(),
        }
    }
}

fn started_event(kind: Kind) -> EventName {
    match kind {
        Kind::Agent => EventName::AgentStarted,
        Kind::System => EventName::SystemStarted,
        Kind::Cue => EventName::CueStarted,
    }
}

fn completed_event(kind: Kind) -> EventName {
    match kind {
        Kind::Agent => EventName::AgentCompleted,
        Kind::System => EventName::SystemCompleted,
        Kind::Cue => EventName::CueCompleted,
    }
}

struct PcmConverter {
    output_rate_hz: u32,
    output_channels: u32,
}

impl PcmConverter {
    fn new(output_rate_hz: u32, output_channels: u32) -> Result<Self, String> {
        if output_rate_hz == 0 || output_channels == 0 {
            return Err("output format must be positive".to_owned());
        }
        gst::init().map_err(|error| format!("cannot initialize GStreamer: {error}"))?;
        Ok(Self {
            output_rate_hz,
            output_channels,
        })
    }

    fn convert(
        &self,
        payload: &[u8],
        sample_rate_hz: u32,
        channels: u32,
        bit_depth: u32,
        encoding: &str,
    ) -> Result<Vec<u8>, String> {
        if encoding != "PCM16LE" || bit_depth != 16 {
            return Err("GStreamer boundary accepts PCM16LE only".to_owned());
        }
        if sample_rate_hz == 0 || channels == 0 {
            return Err("input format must be positive".to_owned());
        }
        let bytes_per_frame = channels as usize * 2;
        if !payload.len().is_multiple_of(bytes_per_frame) {
            return Err("input PCM is not frame aligned".to_owned());
        }
        if payload.is_empty() {
            return Ok(Vec::new());
        }
        let input_frames = payload.len() / bytes_per_frame;
        let expected_frames = round_ratio_ties_even(
            (input_frames as u128) * u128::from(self.output_rate_hz),
            u128::from(sample_rate_hz),
        )?;
        if expected_frames == 0 {
            return Err("PCM input is too short to produce one output frame".to_owned());
        }
        let expected_bytes = expected_frames
            .checked_mul(self.output_channels as usize)
            .and_then(|value| value.checked_mul(2))
            .ok_or_else(|| "converted PCM size overflowed".to_owned())?;

        let description = format!(
            "appsrc name=src format=time ! \
             audio/x-raw,format=S16LE,rate={sample_rate_hz},channels={channels},layout=interleaved ! \
             audioconvert ! audioresample ! \
             audio/x-raw,format=S16LE,rate={},channels={},layout=interleaved ! \
             appsink name=sink sync=false",
            self.output_rate_hz, self.output_channels
        );
        let pipeline = gst::parse::launch(&description)
            .map_err(|error| format!("cannot construct GStreamer PCM pipeline: {error}"))?
            .downcast::<gst::Pipeline>()
            .map_err(|_| "GStreamer PCM conversion did not create a pipeline".to_owned())?;
        let source = pipeline
            .by_name("src")
            .ok_or_else(|| "GStreamer PCM pipeline has no appsrc".to_owned())?
            .downcast::<AppSrc>()
            .map_err(|_| "GStreamer src element is not appsrc".to_owned())?;
        let sink = pipeline
            .by_name("sink")
            .ok_or_else(|| "GStreamer PCM pipeline has no appsink".to_owned())?
            .downcast::<AppSink>()
            .map_err(|_| "GStreamer sink element is not appsink".to_owned())?;

        let mut buffer = gst::Buffer::with_size(payload.len())
            .map_err(|error| format!("cannot allocate GStreamer input buffer: {error}"))?;
        {
            let buffer = buffer
                .get_mut()
                .ok_or_else(|| "GStreamer input buffer is unexpectedly shared".to_owned())?;
            buffer
                .copy_from_slice(0, payload)
                .map_err(|error| format!("cannot fill GStreamer input buffer: {error}"))?;
            buffer.set_pts(gst::ClockTime::ZERO);
            buffer.set_duration(gst::ClockTime::from_nseconds(
                (input_frames as u64).saturating_mul(1_000_000_000) / u64::from(sample_rate_hz),
            ));
        }

        pipeline
            .set_state(gst::State::Playing)
            .map_err(|error| format!("cannot start GStreamer PCM pipeline: {error}"))?;
        let result = (|| -> Result<Vec<u8>, String> {
            source
                .push_buffer(buffer)
                .map_err(|error| format!("GStreamer push-buffer failed: {error}"))?;
            source
                .end_of_stream()
                .map_err(|error| format!("GStreamer end-of-stream failed: {error}"))?;
            let bus = pipeline
                .bus()
                .ok_or_else(|| "GStreamer PCM pipeline has no bus".to_owned())?;
            let mut output = Vec::new();
            let deadline = Instant::now() + PCM_CONVERSION_TIMEOUT;
            loop {
                let now = Instant::now();
                if now >= deadline {
                    return Err(format!(
                        "GStreamer PCM conversion exceeded {} seconds",
                        PCM_CONVERSION_TIMEOUT.as_secs()
                    ));
                }
                let wait = deadline
                    .saturating_duration_since(now)
                    .min(Duration::from_secs(1));
                if let Some(sample) = sink.try_pull_sample(gst::ClockTime::from_nseconds(
                    u64::try_from(wait.as_nanos()).unwrap_or(u64::MAX),
                )) {
                    let output_buffer = sample
                        .buffer()
                        .ok_or_else(|| "GStreamer output sample has no buffer".to_owned())?;
                    let mapped = output_buffer
                        .map_readable()
                        .map_err(|error| format!("cannot map GStreamer output buffer: {error}"))?;
                    output.extend_from_slice(mapped.as_slice());
                    continue;
                }
                if sink.is_eos() {
                    break;
                }
                if let Some(message) = bus.pop_filtered(&[gst::MessageType::Error]) {
                    if let gst::MessageView::Error(error) = message.view() {
                        return Err(format!(
                            "GStreamer conversion failed: {}: {}",
                            error.error(),
                            error.debug().unwrap_or_default()
                        ));
                    }
                }
            }
            output.resize(expected_bytes, 0);
            output.truncate(expected_bytes);
            Ok(output)
        })();
        let null_result = pipeline.set_state(gst::State::Null);
        match (result, null_result) {
            (Ok(output), Ok(_)) => Ok(output),
            (Err(error), _) => Err(error),
            (Ok(_), Err(error)) => Err(format!("cannot stop GStreamer PCM pipeline: {error}")),
        }
    }
}

fn round_ratio_ties_even(numerator: u128, denominator: u128) -> Result<usize, String> {
    if denominator == 0 {
        return Err("cannot round a ratio with a zero denominator".to_owned());
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    let doubled_remainder = remainder
        .checked_mul(2)
        .ok_or_else(|| "PCM frame rounding overflowed".to_owned())?;
    let rounded = if doubled_remainder > denominator
        || (doubled_remainder == denominator && quotient % 2 == 1)
    {
        quotient
            .checked_add(1)
            .ok_or_else(|| "PCM frame rounding overflowed".to_owned())?
    } else {
        quotient
    };
    usize::try_from(rounded).map_err(|_| "converted PCM frame count exceeds usize".to_owned())
}

struct Assembly {
    sample_rate_hz: u32,
    channels: u32,
    bit_depth: u32,
    encoding: String,
    next_seq: u64,
    data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct OutputAck {
    utterance_id: String,
    seq: u64,
    frame_count: u32,
    is_final: bool,
    status: u8,
}

impl OutputAck {
    fn from_message(message: OutputDrainedMessage) -> Result<Self, String> {
        if message.utterance_id.is_empty() {
            return Err("output drained utterance_id must not be empty".to_owned());
        }
        if !matches!(
            message.status,
            STATUS_ACCEPTED | STATUS_DRAINED | STATUS_FLUSHED
        ) {
            return Err("unknown output drained status".to_owned());
        }
        if message.status == STATUS_FLUSHED && message.final_ {
            return Err("a flushed output chunk cannot be final".to_owned());
        }
        Ok(Self {
            utterance_id: message.utterance_id,
            seq: message.seq,
            frame_count: message.frame_count,
            is_final: message.final_,
            status: message.status,
        })
    }
}

#[derive(Clone)]
struct PendingOutput {
    expected: OutputAck,
    kind: Kind,
    accepted: bool,
    last_progress_at: Instant,
}

type OutputKey = (String, u64);

struct FlushState {
    target_kind: Kind,
    target_utterance_id: Option<String>,
    keys: HashSet<OutputKey>,
    partial_seen: bool,
    service_done: bool,
}

#[derive(Debug)]
struct FlushCommand {
    keys: Vec<OutputKey>,
}

enum Effect {
    Frame(PlaybackFrame),
    Event(PlaybackEventMessage),
    Flush(FlushCommand),
}

enum ControlAction {
    Pause,
    Resume,
    Discard {
        release_hold: Kind,
        minimum_agent_epoch: u64,
        utterance_id: Option<String>,
    },
    AbortSystem {
        utterance_id: String,
        release_hold: bool,
    },
}

impl ControlAction {
    fn from_message(message: PlaybackControlMessage) -> Result<Self, String> {
        match message.action {
            value if value == PlaybackControlMessage::PAUSE as u8 => {
                if message.release_hold != PlaybackControlMessage::NONE as u8
                    || message.minimum_agent_epoch != 0
                    || !message.utterance_id.is_empty()
                {
                    return Err("pause contains fields that must be empty".to_owned());
                }
                Ok(Self::Pause)
            }
            value if value == PlaybackControlMessage::RESUME as u8 => {
                if message.release_hold != PlaybackControlMessage::NONE as u8
                    || message.minimum_agent_epoch != 0
                    || !message.utterance_id.is_empty()
                {
                    return Err("resume contains fields that must be empty".to_owned());
                }
                Ok(Self::Resume)
            }
            value if value == PlaybackControlMessage::DISCARD as u8 => {
                let release_hold = match message.release_hold {
                    value if value == PlaybackControlMessage::USER as u8 => Kind::Agent,
                    value if value == PlaybackControlMessage::SYSTEM as u8 => Kind::System,
                    _ => return Err("discard requires release_hold=user or system".to_owned()),
                };
                if release_hold == Kind::System && message.utterance_id.is_empty() {
                    return Err("system discard requires utterance_id".to_owned());
                }
                if release_hold == Kind::Agent && !message.utterance_id.is_empty() {
                    return Err("user discard must not specify utterance_id".to_owned());
                }
                Ok(Self::Discard {
                    release_hold,
                    minimum_agent_epoch: message.minimum_agent_epoch,
                    utterance_id: (release_hold == Kind::System).then_some(message.utterance_id),
                })
            }
            value if value == PlaybackControlMessage::ABORT_SYSTEM as u8 => {
                if message.utterance_id.is_empty()
                    || message.minimum_agent_epoch != 0
                    || !matches!(
                        message.release_hold,
                        value if value == PlaybackControlMessage::NONE as u8
                            || value == PlaybackControlMessage::SYSTEM as u8
                    )
                {
                    return Err("system_abort fields are invalid".to_owned());
                }
                Ok(Self::AbortSystem {
                    utterance_id: message.utterance_id,
                    release_hold: message.release_hold == PlaybackControlMessage::SYSTEM as u8,
                })
            }
            _ => Err(format!(
                "playback action contains an unknown numeric enum: {}",
                message.action
            )),
        }
    }
}

struct PlaybackController {
    machine: PlaybackMachine,
    converter: PcmConverter,
    assemblies: HashMap<(Kind, String), Assembly>,
    minimum_agent_epoch: u64,
    output_inflight: Option<OutputAck>,
    output_inflight_kind: Option<Kind>,
    output_pending: HashMap<OutputKey, PendingOutput>,
    last_output_progress_at: Option<Instant>,
    suppressed_output_acks: HashSet<OutputKey>,
    device_buffer: DeviceBufferTracker,
    flush_request: Option<FlushState>,
    playout_generations: PlayoutGenerations,
}

#[derive(Clone, Copy, Debug)]
struct PlayoutGenerations {
    agent: u64,
    system: u64,
    cue: u64,
}

impl PlayoutGenerations {
    fn seeded(seed: u64) -> Self {
        Self {
            agent: seed,
            system: seed,
            cue: seed,
        }
    }

    fn get(self, kind: Kind) -> u64 {
        match kind {
            Kind::Agent => self.agent,
            Kind::System => self.system,
            Kind::Cue => self.cue,
        }
    }

    fn advance(&mut self, kind: Kind) -> Result<(), String> {
        let generation = match kind {
            Kind::Agent => &mut self.agent,
            Kind::System => &mut self.system,
            Kind::Cue => &mut self.cue,
        };
        *generation = advance_playout_generation(*generation)?;
        Ok(())
    }
}

impl PlaybackController {
    fn new(output_rate_hz: u32, output_channels: u32, chunk_ms: u32) -> Result<Self, String> {
        if chunk_ms == 0 || u64::from(output_rate_hz) * u64::from(chunk_ms) % 1000 != 0 {
            return Err("chunk_ms must produce an integral output frame count".to_owned());
        }
        let chunk_frames = u64::from(output_rate_hz) * u64::from(chunk_ms) / 1000;
        Ok(Self {
            machine: PlaybackMachine::new(chunk_frames)?,
            converter: PcmConverter::new(output_rate_hz, output_channels)?,
            assemblies: HashMap::new(),
            minimum_agent_epoch: 0,
            output_inflight: None,
            output_inflight_kind: None,
            output_pending: HashMap::new(),
            last_output_progress_at: None,
            suppressed_output_acks: HashSet::new(),
            device_buffer: DeviceBufferTracker::default(),
            flush_request: None,
            playout_generations: PlayoutGenerations::seeded(playout_generation_seed(
                SystemTime::now(),
            )?),
        })
    }

    fn on_frame(&mut self, kind: Kind, message: AudioFrame) -> Result<Vec<Effect>, String> {
        if kind == Kind::Agent && agent_epoch(&message.stream_id)? < self.minimum_agent_epoch {
            eprintln!(
                "dropping stale agent PCM {:?}; minimum floor is {}",
                message.stream_id, self.minimum_agent_epoch
            );
            return Ok(Vec::new());
        }
        if kind == Kind::System && self.machine.system_is_aborted(&message.stream_id) {
            self.assemblies.remove(&(kind, message.stream_id.clone()));
            eprintln!(
                "dropping PCM for aborted SYSTEM utterance {:?}",
                message.stream_id
            );
            return Ok(Vec::new());
        }
        let first_system_frame = kind == Kind::System
            && message.seq == 0
            && !self
                .assemblies
                .contains_key(&(kind, message.stream_id.clone()));
        let stream_id = message.stream_id.clone();
        let item = self.assemble(kind, message)?;
        let mut effects = Vec::new();
        if first_system_frame {
            let reserve_events = self.machine.reserve_system(&stream_id)?;
            if reserve_events
                .iter()
                .any(|event| event.name == EventName::AgentPaused)
            {
                if let Some(flush) = self.request_flush(true, Kind::Agent, None, None)? {
                    effects.push(Effect::Flush(flush));
                }
            }
            effects.extend(self.publish_events(reserve_events, None, None)?);
        }
        let Some(item) = item else {
            return Ok(effects);
        };
        let converted = self.converter.convert(
            &item.pcm,
            item.sample_rate_hz,
            item.channels,
            item.bit_depth,
            "PCM16LE",
        );
        let converted = match converted {
            Ok(converted) => converted,
            Err(error) if kind == Kind::System => {
                eprintln!("rejecting system audio conversion: {error}");
                let abort_events = self.machine.abort_system(&item.utterance_id, true)?;
                effects.extend(self.publish_events(
                    abort_events,
                    Some(PlaybackEvent::fallback(
                        EventName::SystemAborted,
                        &item.utterance_id,
                    )),
                    None,
                )?);
                return Ok(effects);
            }
            Err(error) => return Err(error),
        };
        let ready = Utterance::new(
            item.kind,
            item.utterance_id,
            converted,
            self.converter.output_rate_hz,
            self.converter.output_channels,
            16,
        )?;
        let events = self.machine.enqueue(ready)?;
        effects.extend(self.publish_events(events, None, None)?);
        Ok(effects)
    }

    fn reject_frame(&mut self, kind: Kind, utterance_id: &str) -> Result<Vec<Effect>, String> {
        if utterance_id.is_empty() {
            return Ok(Vec::new());
        }
        self.assemblies.remove(&(kind, utterance_id.to_owned()));
        if kind != Kind::System {
            return Ok(Vec::new());
        }
        let events = self.machine.abort_system(utterance_id, true)?;
        self.publish_events(
            events,
            Some(PlaybackEvent::fallback(
                EventName::SystemAborted,
                utterance_id,
            )),
            None,
        )
    }

    fn assemble(&mut self, kind: Kind, message: AudioFrame) -> Result<Option<Utterance>, String> {
        if message.encoding != "PCM16LE" || message.layout != "interleaved" {
            return Err("audio must be PCM16LE/interleaved".to_owned());
        }
        if message.stream_id.is_empty() {
            return Err("stream_id is the required utterance_id".to_owned());
        }
        if message.sample_rate_hz == 0 || message.channels == 0 || message.bit_depth != 16 {
            return Err("audio requires positive rate/channels and PCM16".to_owned());
        }
        if message.frame_count == 0 {
            return Err("audio frame_count must be positive".to_owned());
        }
        let expected_bytes = u64::from(message.frame_count)
            .checked_mul(u64::from(message.channels))
            .and_then(|value| value.checked_mul(2))
            .ok_or_else(|| "AudioFrame payload size overflowed".to_owned())?;
        let payload_bytes = u64::try_from(message.data.len())
            .map_err(|_| "AudioFrame payload length exceeds uint64".to_owned())?;
        if expected_bytes != payload_bytes {
            return Err("AudioFrame frame_count does not match payload".to_owned());
        }
        let key = (kind, message.stream_id.clone());
        if !self.assemblies.contains_key(&key) {
            if message.seq != 0 {
                return Err("utterance must begin at seq=0".to_owned());
            }
            self.assemblies.insert(
                key.clone(),
                Assembly {
                    sample_rate_hz: message.sample_rate_hz,
                    channels: message.channels,
                    bit_depth: message.bit_depth,
                    encoding: message.encoding.clone(),
                    next_seq: 0,
                    data: Vec::new(),
                },
            );
        }
        let assembly = self.assemblies.get_mut(&key).expect("inserted above");
        if message.seq != assembly.next_seq {
            return Err(format!(
                "non-contiguous audio seq: {} != {}",
                message.seq, assembly.next_seq
            ));
        }
        if (
            message.sample_rate_hz,
            message.channels,
            message.bit_depth,
            message.encoding.as_str(),
        ) != (
            assembly.sample_rate_hz,
            assembly.channels,
            assembly.bit_depth,
            assembly.encoding.as_str(),
        ) {
            return Err("audio format changed inside an utterance".to_owned());
        }
        assembly.data.extend_from_slice(&message.data);
        assembly.next_seq = assembly
            .next_seq
            .checked_add(1)
            .ok_or_else(|| "audio sequence overflowed".to_owned())?;
        if !message.final_ {
            return Ok(None);
        }
        let assembly = self.assemblies.remove(&key).expect("assembly exists");
        Utterance::new(
            kind,
            message.stream_id,
            assembly.data,
            assembly.sample_rate_hz,
            assembly.channels,
            assembly.bit_depth,
        )
        .map(Some)
    }

    fn on_control(&mut self, message: PlaybackControlMessage) -> Result<Vec<Effect>, String> {
        let control = ControlAction::from_message(message)?;
        let (events, fallback, invalidated_kind, flush) = match control {
            ControlAction::Pause => {
                let events = self.machine.pause_agent();
                let flush = self.request_flush(true, Kind::Agent, None, None)?;
                (
                    events,
                    Some(PlaybackEvent::fallback(EventName::AgentPaused, "")),
                    Some(Kind::Agent),
                    flush,
                )
            }
            ControlAction::Resume => (
                self.machine.resume_agent(),
                Some(PlaybackEvent::fallback(EventName::AgentResumed, "")),
                None,
                None,
            ),
            ControlAction::Discard {
                release_hold,
                minimum_agent_epoch,
                utterance_id,
            } => {
                self.minimum_agent_epoch = self.minimum_agent_epoch.max(minimum_agent_epoch);
                let mut stale_assemblies = Vec::new();
                for (kind, utterance_id) in self.assemblies.keys() {
                    if *kind == Kind::Agent && agent_epoch(utterance_id)? < self.minimum_agent_epoch
                    {
                        stale_assemblies.push((*kind, utterance_id.clone()));
                    }
                }
                for key in stale_assemblies {
                    self.assemblies.remove(&key);
                }
                let events = self.machine.discard_agent(
                    release_hold,
                    utterance_id.as_deref(),
                    Some(self.minimum_agent_epoch),
                )?;
                let flush =
                    self.request_flush(false, Kind::Agent, None, Some(self.minimum_agent_epoch))?;
                (
                    events,
                    Some(PlaybackEvent::fallback(EventName::AgentDiscarded, "")),
                    Some(Kind::Agent),
                    flush,
                )
            }
            ControlAction::AbortSystem {
                utterance_id,
                release_hold,
            } => {
                self.assemblies
                    .remove(&(Kind::System, utterance_id.clone()));
                let events = self.machine.abort_system(&utterance_id, release_hold)?;
                let flush = self.request_flush(false, Kind::System, Some(&utterance_id), None)?;
                (
                    events,
                    Some(PlaybackEvent::fallback(
                        EventName::SystemAborted,
                        utterance_id,
                    )),
                    None,
                    flush,
                )
            }
        };
        let mut effects = Vec::new();
        if let Some(flush) = flush {
            effects.push(Effect::Flush(flush));
        }
        effects.extend(self.publish_events(events, fallback, invalidated_kind)?);
        Ok(effects)
    }

    fn on_output_drained(&mut self, message: OutputDrainedMessage) -> Result<Vec<Effect>, String> {
        let ack = OutputAck::from_message(message)?;
        let key = (ack.utterance_id.clone(), ack.seq);
        let pending = self
            .output_pending
            .get_mut(&key)
            .ok_or_else(|| "output ack does not match a pending chunk".to_owned())?;
        let expected = pending.expected.clone();
        if ack.status == STATUS_ACCEPTED {
            if pending.accepted {
                return Err("duplicate output accepted ack".to_owned());
            }
            if ack.frame_count != expected.frame_count || ack.is_final != expected.is_final {
                return Err("accepted ack does not match the complete chunk".to_owned());
            }
            pending.accepted = true;
            let now = Instant::now();
            pending.last_progress_at = now;
            self.last_output_progress_at = Some(now);
            let events = if ack.seq == 0 && !self.suppressed_output_acks.contains(&key) {
                self.machine.output_accepted(&ack.utterance_id, ack.seq)?
            } else {
                Vec::new()
            };
            if !expected.is_final && self.inflight_key().as_ref() == Some(&key) {
                self.output_inflight = None;
                self.output_inflight_kind = None;
            }
            return self.publish_events(events, None, None);
        }
        let terminal_before_start = !pending.accepted
            && ack.status == STATUS_FLUSHED
            && ack.frame_count == 0
            && !ack.is_final;
        if !pending.accepted && !terminal_before_start {
            return Err("terminal output ack arrived before accepted ack".to_owned());
        }
        if ack.status == STATUS_DRAINED && ack != expected {
            return Err("drained ack does not match the complete pending chunk".to_owned());
        }
        if ack.status == STATUS_FLUSHED
            && !(ack.frame_count < expected.frame_count && !ack.is_final)
        {
            return Err("flushed ack does not describe a partial pending chunk".to_owned());
        }
        self.last_output_progress_at = Some(Instant::now());
        self.device_buffer
            .note_ack(&ack.utterance_id, ack.is_final, ack.status);
        self.output_pending.remove(&key);
        if self.inflight_key().as_ref() == Some(&key) {
            self.output_inflight = None;
            self.output_inflight_kind = None;
        }

        let belongs_to_flush = self
            .flush_request
            .as_ref()
            .is_some_and(|flush| flush.keys.contains(&key));
        let mut account_progress = !self.suppressed_output_acks.contains(&key);
        if belongs_to_flush
            && self
                .flush_request
                .as_ref()
                .is_some_and(|flush| flush.partial_seen)
        {
            account_progress = false;
        }
        let events = if account_progress {
            self.machine.output_drained(
                &ack.utterance_id,
                ack.seq,
                ack.frame_count,
                ack.is_final,
                ack.status,
            )?
        } else {
            Vec::new()
        };
        if belongs_to_flush {
            let flush = self.flush_request.as_mut().expect("checked above");
            if ack.status == STATUS_FLUSHED {
                flush.partial_seen = true;
            }
            flush.keys.remove(&key);
        }
        self.suppressed_output_acks.remove(&key);
        if self.output_pending.is_empty() {
            self.last_output_progress_at = None;
        }
        self.maybe_finish_flush();
        self.publish_events(events, None, None)
    }

    fn pump(&mut self) -> Result<Vec<Effect>, String> {
        if self.output_inflight.is_some() || self.flush_request.is_some() {
            return Ok(Vec::new());
        }
        let mut effects = Vec::new();
        if let Some(chunk) = self.machine.next_chunk()? {
            let expected = OutputAck {
                utterance_id: chunk.utterance_id.clone(),
                seq: chunk.seq,
                frame_count: chunk.frame_count,
                is_final: chunk.is_final,
                status: STATUS_DRAINED,
            };
            let key = (chunk.utterance_id.clone(), chunk.seq);
            if self.output_pending.contains_key(&key) {
                return Err(format!("duplicate pending output chunk: {key:?}"));
            }
            let now = Instant::now();
            self.output_pending.insert(
                key,
                PendingOutput {
                    expected: expected.clone(),
                    kind: chunk.kind,
                    accepted: false,
                    last_progress_at: now,
                },
            );
            if self.last_output_progress_at.is_none() {
                self.last_output_progress_at = Some(now);
            }
            self.output_inflight = Some(expected);
            self.output_inflight_kind = Some(chunk.kind);
            self.device_buffer
                .note_chunk(chunk.kind, &chunk.utterance_id);
            effects.push(Effect::Frame(PlaybackFrame {
                kind: chunk.kind.code(),
                utterance_id: chunk.utterance_id,
                seq: chunk.seq,
                sample_index: chunk.sample_index,
                frame_count: chunk.frame_count,
                sample_rate_hz: chunk.sample_rate_hz,
                channels: chunk.channels,
                pcm_s16le: chunk.data,
                final_: chunk.is_final,
                playout_generation: self.playout_generations.get(chunk.kind),
            }));
        }
        Ok(effects)
    }

    fn check_output_health(&self, now: Instant) -> Result<(), String> {
        for (key, pending) in &self.output_pending {
            if !pending.accepted
                && elapsed_exceeds(pending.last_progress_at, now, OUTPUT_PROGRESS_TIMEOUT)
            {
                return Err(format!(
                    "audio output acceptance timed out for chunk {key:?} after {} seconds",
                    OUTPUT_PROGRESS_TIMEOUT.as_secs()
                ));
            }
        }
        if !self.output_pending.is_empty()
            && self.last_output_progress_at.is_some_and(|last_progress| {
                elapsed_exceeds(last_progress, now, OUTPUT_PROGRESS_TIMEOUT)
            })
        {
            return Err(format!(
                "audio output acknowledgement/drain progress stopped for {} pending chunk(s) for more than {} seconds",
                self.output_pending.len(),
                OUTPUT_PROGRESS_TIMEOUT.as_secs()
            ));
        }
        Ok(())
    }

    fn request_flush(
        &mut self,
        account_ack: bool,
        target_kind: Kind,
        target_utterance_id: Option<&str>,
        minimum_agent_epoch: Option<u64>,
    ) -> Result<Option<FlushCommand>, String> {
        let is_target = |utterance_id: &str| -> Result<bool, String> {
            if target_utterance_id.is_some_and(|target| utterance_id != target) {
                return Ok(false);
            }
            if target_kind == Kind::Agent {
                if let Some(minimum) = minimum_agent_epoch {
                    return Ok(agent_epoch(utterance_id)? < minimum);
                }
            }
            Ok(true)
        };
        let mut matching_keys = HashSet::new();
        for (key, pending) in &self.output_pending {
            if pending.kind == target_kind && is_target(&pending.expected.utterance_id)? {
                matching_keys.insert(key.clone());
            }
        }
        let device_matches = if self.device_buffer.matches(target_kind, target_utterance_id) {
            match self.device_buffer.utterance_id.as_deref() {
                Some(utterance_id) => is_target(utterance_id)?,
                None => false,
            }
        } else {
            false
        };
        if matching_keys.is_empty() && !device_matches {
            return Ok(None);
        }
        if !account_ack {
            self.suppressed_output_acks
                .extend(matching_keys.iter().cloned());
        }
        if self.flush_request.is_some() {
            return Ok(None);
        }
        let mut ordered_keys = matching_keys.iter().cloned().collect::<Vec<_>>();
        ordered_keys.sort();
        self.flush_request = Some(FlushState {
            target_kind,
            target_utterance_id: target_utterance_id.map(str::to_owned),
            keys: matching_keys,
            partial_seen: false,
            service_done: false,
        });
        Ok(Some(FlushCommand { keys: ordered_keys }))
    }

    fn finish_flush_service(&mut self) {
        if let Some(request) = self.flush_request.as_mut() {
            self.device_buffer
                .clear_matching(request.target_kind, request.target_utterance_id.as_deref());
            request.service_done = true;
        }
        self.maybe_finish_flush();
    }

    fn maybe_finish_flush(&mut self) {
        if self
            .flush_request
            .as_ref()
            .is_some_and(|request| request.service_done && request.keys.is_empty())
        {
            self.flush_request = None;
        }
    }

    fn inflight_key(&self) -> Option<OutputKey> {
        self.output_inflight
            .as_ref()
            .map(|ack| (ack.utterance_id.clone(), ack.seq))
    }

    fn publish_events(
        &mut self,
        events: Vec<PlaybackEvent>,
        fallback_event: Option<PlaybackEvent>,
        force_invalidated_kind: Option<Kind>,
    ) -> Result<Vec<Effect>, String> {
        let mut invalidated = events
            .iter()
            .filter_map(|event| event.name.invalidates())
            .collect::<HashSet<_>>();
        if let Some(kind) = force_invalidated_kind {
            invalidated.insert(kind);
        }
        for kind in invalidated {
            self.playout_generations.advance(kind)?;
        }
        let fallback_needed = fallback_event
            .as_ref()
            .is_some_and(|fallback| !events.iter().any(|event| event.name == fallback.name));
        let mut effects = events
            .into_iter()
            .map(|event| Effect::Event(event.to_message(self.playout_generations.get(event.kind))))
            .collect::<Vec<_>>();
        if fallback_needed {
            let fallback = fallback_event.expect("checked above");
            effects.push(Effect::Event(
                fallback.to_message(self.playout_generations.get(fallback.kind)),
            ));
        }
        Ok(effects)
    }
}

fn process_audio_frame(
    controller: &mut PlaybackController,
    kind: Kind,
    message: AudioFrame,
) -> Result<Vec<Effect>, String> {
    let utterance_id = message.stream_id.clone();
    match controller.on_frame(kind, message) {
        Ok(effects) => Ok(effects),
        Err(error) => {
            eprintln!("rejecting {kind:?} audio frame: {error}");
            let effects =
                controller
                    .reject_frame(kind, &utterance_id)
                    .map_err(|cleanup_error| {
                        format!(
                    "cannot contain rejected {kind:?} utterance {utterance_id:?}: {cleanup_error}"
                )
                    })?;
            if kind == Kind::Agent {
                return Err(format!(
                    "trusted AGENT audio contract failed for {utterance_id:?}: {error}"
                ));
            }
            Ok(effects)
        }
    }
}

fn agent_epoch(utterance_id: &str) -> Result<u64, String> {
    let mut parts = utterance_id.splitn(3, '-');
    if parts.next() != Some("agent") {
        return Err("agent utterance_id must be agent-<floor_epoch>-<id>".to_owned());
    }
    let epoch = parts
        .next()
        .ok_or_else(|| "agent utterance_id must be agent-<floor_epoch>-<id>".to_owned())?;
    if parts.next().is_none() {
        return Err("agent utterance_id must be agent-<floor_epoch>-<id>".to_owned());
    }
    epoch
        .parse::<u64>()
        .map_err(|_| "agent utterance_id floor epoch is invalid".to_owned())
}

fn playout_generation_seed(now: SystemTime) -> Result<u64, String> {
    let milliseconds = now
        .duration_since(UNIX_EPOCH)
        .map_err(|_| "wall clock precedes UNIX epoch".to_owned())?
        .as_millis();
    let value =
        u64::try_from(milliseconds).map_err(|_| "playout generation exceeds uint64".to_owned())?;
    if value > MAX_SAFE_PLAYOUT_GENERATION {
        return Err("playout generation exceeds the JavaScript safe integer range".to_owned());
    }
    Ok(value)
}

fn advance_playout_generation(current: u64) -> Result<u64, String> {
    if current >= MAX_SAFE_PLAYOUT_GENERATION {
        return Err("playout generation exceeds the JavaScript safe integer range".to_owned());
    }
    Ok(current + 1)
}

fn elapsed_exceeds(started: Instant, now: Instant, timeout: Duration) -> bool {
    now.saturating_duration_since(started) > timeout
}

type FlushFuture = Pin<Box<dyn Future<Output = r2r::Result<FlushAudio::Response>>>>;
type AvailabilityFuture = Pin<Box<dyn Future<Output = r2r::Result<()>>>>;

fn take_ready_bounded<S>(stream: &mut S, limit: usize) -> Vec<S::Item>
where
    S: Stream + Unpin,
{
    let mut ready = Vec::with_capacity(limit);
    for _ in 0..limit {
        match stream.next().now_or_never() {
            Some(Some(item)) => ready.push(item),
            Some(None) | None => break,
        }
    }
    ready
}

fn reliable_volatile_qos(depth: usize) -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    }
}

fn main() -> Result<(), ControllerError> {
    let context = Context::create()?;
    let mut node = Node::create(context, "playback_controller", "")?;
    let output_rate = positive_u32_parameter(&node, "output_sample_rate_hz", 48_000)?;
    let output_channels = positive_u32_parameter(&node, "output_channels", 2)?;
    let chunk_ms = positive_u32_parameter(&node, "chunk_ms", 20)?;
    let mut controller = PlaybackController::new(output_rate, output_channels, chunk_ms)
        .map_err(ControllerError::Config)?;

    let control_qos = reliable_volatile_qos(CONTROL_QOS_DEPTH);
    let playout_qos = reliable_volatile_qos(PLAYOUT_QOS_DEPTH);
    let mut agent_frames =
        node.subscribe::<AudioFrame>("/audio/agent/frame", control_qos.clone())?;
    let mut system_frames =
        node.subscribe::<AudioFrame>("/audio/system/frame", control_qos.clone())?;
    let mut cue_frames = node.subscribe::<AudioFrame>("/audio/cue/frame", control_qos.clone())?;
    let mut controls =
        node.subscribe::<PlaybackControlMessage>("/audio/playback/control", control_qos.clone())?;
    let mut drained =
        node.subscribe::<OutputDrainedMessage>("/audio/output/drained", control_qos.clone())?;
    let output = node.create_publisher::<PlaybackFrame>("/audio/play", playout_qos)?;
    let events = node
        .create_publisher::<PlaybackEventMessage>("/audio/playback/event", control_qos.clone())?;
    let flush = node.create_client::<FlushAudio::Service>("/audio/output/flush", control_qos)?;
    let mut availability: Option<AvailabilityFuture> = Some(Box::pin(Node::is_available(&flush)?));
    let mut flush_available = false;
    let mut flush_future: Option<FlushFuture> = None;
    let mut flush_started: Option<Instant> = None;

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    eprintln!(
        "playback_controller Rust/GStreamer ready: output={}Hz channels={} chunk={}ms",
        output_rate, output_channels, chunk_ms
    );
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        let now = Instant::now();
        controller
            .check_output_health(now)
            .map_err(ControllerError::Output)?;
        if flush_started.is_some_and(|started| elapsed_exceeds(started, now, OUTPUT_FLUSH_TIMEOUT))
        {
            return Err(ControllerError::Output(format!(
                "/audio/output/flush timed out after {} seconds",
                OUTPUT_FLUSH_TIMEOUT.as_secs()
            )));
        }
        if !flush_available {
            if let Some(future) = availability.as_mut() {
                if let Some(result) = future.as_mut().now_or_never() {
                    result.map_err(|error| {
                        ControllerError::Output(format!(
                            "/audio/output/flush availability failed: {error}"
                        ))
                    })?;
                    flush_available = true;
                    availability = None;
                }
            }
        }
        if let Some(future) = flush_future.as_mut() {
            if let Some(result) = future.as_mut().now_or_never() {
                flush_future = None;
                flush_started = None;
                let response = result.map_err(|error| {
                    ControllerError::Output(format!("/audio/output/flush failed: {error}"))
                })?;
                if !response.success {
                    return Err(ControllerError::Output(format!(
                        "/audio/output/flush failed: {}",
                        response.message
                    )));
                }
                controller.finish_flush_service();
            }
        }

        let mut pending_effects = Vec::new();
        // Bound every source so a continuously-ready audio publisher cannot monopolize
        // the single-threaded ROS boundary. Controls run before SYSTEM PCM so an abort
        // already waiting in the same spin tombstones that PCM without pause/flush churn.
        for message in take_ready_bounded(&mut controls, CONTROL_MESSAGES_PER_TICK) {
            match controller.on_control(message) {
                Ok(effects) => pending_effects.extend(effects),
                Err(error) => eprintln!("rejecting playback control: {error}"),
            }
        }
        for message in take_ready_bounded(&mut drained, ACK_MESSAGES_PER_TICK) {
            pending_effects.extend(
                controller
                    .on_output_drained(message)
                    .map_err(ControllerError::Output)?,
            );
        }
        for message in take_ready_bounded(&mut system_frames, SYSTEM_MESSAGES_PER_TICK) {
            pending_effects.extend(
                process_audio_frame(&mut controller, Kind::System, message)
                    .map_err(ControllerError::Invariant)?,
            );
        }
        for message in take_ready_bounded(&mut cue_frames, CUE_MESSAGES_PER_TICK) {
            pending_effects.extend(
                process_audio_frame(&mut controller, Kind::Cue, message)
                    .map_err(ControllerError::Invariant)?,
            );
        }
        for message in take_ready_bounded(&mut agent_frames, AGENT_MESSAGES_PER_TICK) {
            pending_effects.extend(
                process_audio_frame(&mut controller, Kind::Agent, message)
                    .map_err(ControllerError::Output)?,
            );
        }
        pending_effects.extend(controller.pump().map_err(ControllerError::Invariant)?);

        for effect in pending_effects {
            match effect {
                Effect::Frame(message) => output.publish(&message)?,
                Effect::Event(message) => events.publish(&message)?,
                Effect::Flush(command) => {
                    if !flush_available {
                        return Err(ControllerError::Output(
                            "/audio/output/flush is unavailable".to_owned(),
                        ));
                    }
                    if flush_future.is_some() {
                        return Err(ControllerError::Invariant(
                            "multiple output flush requests are active".to_owned(),
                        ));
                    }
                    let request = FlushAudio::Request {
                        utterance_ids: command
                            .keys
                            .iter()
                            .map(|(utterance_id, _)| utterance_id.clone())
                            .collect(),
                        seqs: command.keys.iter().map(|(_, seq)| *seq).collect(),
                    };
                    let future = flush.request(&request).map_err(|error| {
                        ControllerError::Output(format!(
                            "/audio/output/flush request failed: {error}"
                        ))
                    })?;
                    flush_future = Some(Box::pin(future));
                    flush_started = Some(Instant::now());
                }
            }
        }
    }
    Ok(())
}

fn positive_u32_parameter(node: &Node, name: &str, default: i64) -> Result<u32, ControllerError> {
    let value = node.get_parameter::<i64>(name).unwrap_or(default);
    u32::try_from(value)
        .ok()
        .filter(|value| *value > 0)
        .ok_or_else(|| ControllerError::Config(format!("{name} must be a positive uint32")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn utterance(kind: Kind, name: &str, frames: usize) -> Utterance {
        Utterance::new(kind, name.to_owned(), vec![0; frames * 2], 16_000, 1, 16).unwrap()
    }

    #[test]
    fn barge_in_resumes_at_the_drained_offset() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 8))
            .unwrap();
        assert_eq!(machine.next_chunk().unwrap().unwrap().sample_index, 0);
        assert_eq!(
            machine.output_accepted("agent-0-a", 0).unwrap()[0].name,
            EventName::AgentStarted
        );
        machine
            .output_drained("agent-0-a", 0, 2, false, STATUS_DRAINED)
            .unwrap();
        assert_eq!(machine.pause_agent()[0].played_frames, 2);
        assert!(machine.next_chunk().unwrap().is_none());
        machine.resume_agent();
        assert_eq!(machine.next_chunk().unwrap().unwrap().sample_index, 2);
    }

    #[test]
    fn floor_discard_preserves_new_agent_audio() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-old", 8))
            .unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-1-new", 8))
            .unwrap();
        machine.next_chunk().unwrap();
        let events = machine.discard_agent(Kind::Agent, None, Some(1)).unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].utterance_id, "agent-0-old");
        assert_eq!(
            machine.next_chunk().unwrap().unwrap().utterance_id,
            "agent-1-new"
        );
    }

    #[test]
    fn system_reservation_preempts_and_abort_resumes_agent() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 8))
            .unwrap();
        machine.next_chunk().unwrap();
        let events = machine.reserve_system("system-a").unwrap();
        assert_eq!(events[0].name, EventName::AgentPaused);
        assert!(machine.reserve_system("system-a").unwrap().is_empty());
        assert!(machine.next_chunk().unwrap().is_none());
        let events = machine.abort_system("system-a", true).unwrap();
        assert_eq!(events[0].name, EventName::AgentResumed);
        assert_eq!(machine.next_chunk().unwrap().unwrap().kind, Kind::Agent);
    }

    #[test]
    fn system_completion_keeps_agent_held_until_semantic_result() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 8))
            .unwrap();
        machine
            .enqueue(utterance(Kind::System, "system-a", 2))
            .unwrap();
        let system = machine.next_chunk().unwrap().unwrap();
        assert_eq!(system.kind, Kind::System);
        machine
            .output_drained("system-a", 0, 2, true, STATUS_DRAINED)
            .unwrap();
        assert!(machine.next_chunk().unwrap().is_none());
        machine.abort_system("system-a", true).unwrap();
        assert_eq!(machine.next_chunk().unwrap().unwrap().kind, Kind::Agent);
    }

    #[test]
    fn overlapping_system_holds_are_independent() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 8))
            .unwrap();
        machine.reserve_system("system-a").unwrap();
        machine.reserve_system("system-b").unwrap();
        machine.abort_system("system-a", true).unwrap();
        assert!(machine.agent_pause_requested());
        assert!(machine.next_chunk().unwrap().is_none());
        machine.abort_system("system-b", true).unwrap();
        assert!(!machine.agent_pause_requested());
        assert_eq!(machine.next_chunk().unwrap().unwrap().kind, Kind::Agent);
    }

    #[test]
    fn system_abort_tombstone_rejects_late_pcm() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine.abort_system("system-a", true).unwrap();
        let events = machine
            .enqueue(utterance(Kind::System, "system-a", 2))
            .unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].name, EventName::SystemAborted);
        assert!(machine.next_chunk().unwrap().is_none());
    }

    #[test]
    fn flushed_ack_advances_paused_resume_position() {
        let mut machine = PlaybackMachine::new(4).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 12))
            .unwrap();
        machine.next_chunk().unwrap();
        machine.pause_agent();
        let events = machine
            .output_drained("agent-0-a", 0, 2, false, STATUS_FLUSHED)
            .unwrap();
        assert_eq!(events[0].name, EventName::AgentPaused);
        machine.resume_agent();
        assert_eq!(machine.next_chunk().unwrap().unwrap().sample_index, 2);
    }

    #[test]
    fn final_ack_during_pause_completes_without_resume() {
        let mut machine = PlaybackMachine::new(4).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 4))
            .unwrap();
        machine.next_chunk().unwrap();
        machine.pause_agent();
        let events = machine
            .output_drained("agent-0-a", 0, 4, true, STATUS_DRAINED)
            .unwrap();
        assert_eq!(events[0].name, EventName::AgentCompleted);
        assert!(machine.resume_agent().is_empty());
    }

    #[test]
    fn device_buffer_only_matches_its_kind_and_target() {
        let mut tracker = DeviceBufferTracker::default();
        tracker.note_chunk(Kind::System, "system-a");
        assert!(!tracker.matches(Kind::Agent, None));
        assert!(tracker.matches(Kind::System, Some("system-a")));
        assert!(!tracker.matches(Kind::System, Some("system-b")));
        tracker.note_ack("system-a", false, STATUS_FLUSHED);
        assert!(!tracker.matches(Kind::System, None));
    }

    #[test]
    fn playback_generation_is_js_safe_and_monotonic() {
        let first =
            playout_generation_seed(UNIX_EPOCH + Duration::from_nanos(1_750_000_000_123_456_789))
                .unwrap();
        let restarted =
            playout_generation_seed(UNIX_EPOCH + Duration::from_nanos(1_750_000_001_123_456_789))
                .unwrap();
        assert_eq!(first, 1_750_000_000_123);
        assert!(restarted > first);
        assert_eq!(
            advance_playout_generation(restarted).unwrap(),
            restarted + 1
        );
        assert!(advance_playout_generation(MAX_SAFE_PLAYOUT_GENERATION).is_err());
    }

    #[test]
    fn invalidation_generations_are_kind_scoped() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller.playout_generations = PlayoutGenerations::seeded(0);

        let aborted = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::ABORT_SYSTEM as u8,
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: "system-queued".to_owned(),
            })
            .unwrap();
        assert!(aborted.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::SYSTEM_ABORTED as u8
                    && message.playout_generation == 0
        )));
        assert_eq!(controller.playout_generations.get(Kind::System), 0);

        let paused = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::PAUSE as u8,
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: String::new(),
            })
            .unwrap();
        assert!(paused.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_PAUSED as u8
                    && message.playout_generation == 1
        )));
        assert_eq!(controller.playout_generations.get(Kind::Agent), 1);
        assert_eq!(controller.playout_generations.get(Kind::System), 0);
        assert_eq!(controller.playout_generations.get(Kind::Cue), 0);
    }

    #[test]
    fn control_contract_rejects_unknown_and_cross_action_fields() {
        let mut message = PlaybackControlMessage {
            action: 0,
            release_hold: 0,
            minimum_agent_epoch: 0,
            utterance_id: String::new(),
        };
        assert!(ControlAction::from_message(message.clone()).is_err());
        message.action = PlaybackControlMessage::PAUSE as u8;
        message.utterance_id = "not-allowed".to_owned();
        assert!(ControlAction::from_message(message).is_err());
    }

    #[test]
    fn pcm_converter_resamples_and_maps_channels() {
        let converter = PcmConverter::new(48_000, 2).unwrap();
        let input = vec![0_u8; 160 * 2];
        let output = converter.convert(&input, 16_000, 1, 16, "PCM16LE").unwrap();
        assert_eq!(output.len(), 480 * 2 * 2);
    }

    fn input_frame(stream_id: &str, seq: u64, frame_count: u32, final_: bool) -> AudioFrame {
        AudioFrame {
            header: r2r::std_msgs::msg::Header {
                stamp: r2r::builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                frame_id: String::new(),
            },
            source_id: "test".to_owned(),
            stream_id: stream_id.to_owned(),
            seq,
            sample_index: seq * u64::from(frame_count),
            capture_time_ns: 0,
            frame_count,
            encoding: "PCM16LE".to_owned(),
            sample_rate_hz: 16_000,
            channels: 1,
            bit_depth: 16,
            layout: "interleaved".to_owned(),
            data: vec![0; frame_count as usize * 2],
            final_,
        }
    }

    fn output_ack(
        utterance_id: &str,
        seq: u64,
        frame_count: u32,
        is_final: bool,
        status: u8,
    ) -> OutputDrainedMessage {
        OutputDrainedMessage {
            utterance_id: utterance_id.to_owned(),
            seq,
            frame_count,
            final_: is_final,
            status,
        }
    }

    fn control(action: u8, release_hold: u8, utterance_id: &str) -> PlaybackControlMessage {
        PlaybackControlMessage {
            action,
            release_hold,
            minimum_agent_epoch: 0,
            utterance_id: utterance_id.to_owned(),
        }
    }

    fn frame_effect(effects: Vec<Effect>) -> PlaybackFrame {
        effects
            .into_iter()
            .find_map(|effect| match effect {
                Effect::Frame(frame) => Some(frame),
                Effect::Event(_) | Effect::Flush(_) => None,
            })
            .expect("a playback frame effect is required")
    }

    #[test]
    fn malformed_input_is_contained_and_a_stream_id_can_restart_at_seq_zero() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        assert!(process_audio_frame(
            &mut controller,
            Kind::Agent,
            input_frame("agent-0-bad", 0, 2, false),
        )
        .unwrap()
        .is_empty());
        assert!(controller
            .assemblies
            .contains_key(&(Kind::Agent, "agent-0-bad".to_owned())));

        let mut invalid = input_frame("agent-0-bad", 1, 1, true);
        invalid.bit_depth = 8;
        assert!(process_audio_frame(&mut controller, Kind::Agent, invalid).is_err());
        assert!(!controller
            .assemblies
            .contains_key(&(Kind::Agent, "agent-0-bad".to_owned())));
        assert!(controller.pump().unwrap().is_empty());

        let restarted = controller
            .assemble(Kind::Agent, input_frame("agent-0-bad", 0, 16, true))
            .unwrap()
            .expect("the restarted utterance must assemble");
        assert_eq!(restarted.total_frames(), 16);
    }

    #[test]
    fn zero_frame_input_never_reaches_the_output_boundary() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        let effects = process_audio_frame(
            &mut controller,
            Kind::Cue,
            input_frame("cue-empty", 0, 0, true),
        );
        assert!(effects.unwrap().is_empty());
        assert!(controller.pump().unwrap().is_empty());
        assert!(
            Utterance::new(Kind::Cue, "cue-empty".to_owned(), Vec::new(), 16_000, 1, 16).is_err()
        );
    }

    #[test]
    fn malformed_system_pcm_aborts_and_releases_its_hold() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        process_audio_frame(
            &mut controller,
            Kind::System,
            input_frame("system-bad", 0, 2, false),
        )
        .unwrap();
        assert!(controller.machine.agent_pause_requested());

        let mut invalid = input_frame("system-bad", 1, 2, true);
        invalid.sample_rate_hz = 0;
        let effects = process_audio_frame(&mut controller, Kind::System, invalid).unwrap();

        assert!(!controller.machine.agent_pause_requested());
        assert!(controller.machine.system_is_aborted("system-bad"));
        assert!(!controller
            .assemblies
            .contains_key(&(Kind::System, "system-bad".to_owned())));
        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::SYSTEM_ABORTED as u8
        )));
    }

    #[test]
    fn a_ready_abort_tombstones_system_pcm_before_it_can_reserve_playback() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .on_control(control(
                PlaybackControlMessage::ABORT_SYSTEM as u8,
                PlaybackControlMessage::SYSTEM as u8,
                "system-late",
            ))
            .unwrap();

        let effects = process_audio_frame(
            &mut controller,
            Kind::System,
            input_frame("system-late", 0, 16, true),
        )
        .unwrap();

        assert!(effects.is_empty());
        assert!(!controller.machine.agent_pause_requested());
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn ready_stream_drain_is_bounded_per_tick() {
        let mut stream = futures::stream::iter(0..10);
        assert_eq!(take_ready_bounded(&mut stream, 3), vec![0, 1, 2]);
        assert_eq!(take_ready_bounded(&mut stream, 3), vec![3, 4, 5]);
        assert_eq!(take_ready_bounded(&mut stream, 10), vec![6, 7, 8, 9]);
    }

    #[test]
    fn playout_qos_retains_the_old_browser_backlog_depth() {
        let qos = reliable_volatile_qos(PLAYOUT_QOS_DEPTH);
        assert_eq!(qos.history, HistoryPolicy::KeepLast);
        assert_eq!(qos.depth, 256);
        assert_eq!(qos.reliability, ReliabilityPolicy::Reliable);
        assert_eq!(qos.durability, DurabilityPolicy::Volatile);
    }

    #[test]
    fn missing_output_ack_becomes_a_fatal_health_error() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-timeout", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());
        let started = controller
            .output_pending
            .values()
            .next()
            .unwrap()
            .last_progress_at;

        let error = controller
            .check_output_health(started + OUTPUT_PROGRESS_TIMEOUT + Duration::from_millis(1))
            .unwrap_err();
        assert!(error.contains("acceptance timed out"));
        assert!(error.contains("agent-0-timeout"));
    }

    #[test]
    fn accepted_chunks_use_global_drain_progress_not_their_individual_age() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-long", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());
        let started = controller.last_output_progress_at.unwrap();
        let pending = controller.output_pending.values_mut().next().unwrap();
        pending.accepted = true;
        pending.last_progress_at = started;
        controller.last_output_progress_at = Some(started + Duration::from_secs(9));

        controller
            .check_output_health(started + Duration::from_secs(10))
            .unwrap();
    }

    #[test]
    fn stalled_global_drain_progress_is_fatal() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-stalled", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());
        let started = controller.last_output_progress_at.unwrap();
        controller
            .output_pending
            .values_mut()
            .next()
            .unwrap()
            .accepted = true;

        let error = controller
            .check_output_health(started + OUTPUT_PROGRESS_TIMEOUT + Duration::from_millis(1))
            .unwrap_err();
        assert!(error.contains("acknowledgement/drain progress stopped"));
        assert!(error.contains("1 pending chunk"));
    }

    #[test]
    fn timeout_boundary_is_not_declared_early() {
        let started = Instant::now();
        assert!(!elapsed_exceeds(
            started,
            started + OUTPUT_FLUSH_TIMEOUT,
            OUTPUT_FLUSH_TIMEOUT
        ));
        assert!(elapsed_exceeds(
            started,
            started + OUTPUT_FLUSH_TIMEOUT + Duration::from_nanos(1),
            OUTPUT_FLUSH_TIMEOUT
        ));
    }

    #[test]
    fn resample_frame_rounding_matches_python_ties_to_even() {
        assert_eq!(round_ratio_ties_even(1, 2).unwrap(), 0);
        assert_eq!(round_ratio_ties_even(3, 2).unwrap(), 2);
        assert_eq!(round_ratio_ties_even(5, 2).unwrap(), 2);
        assert_eq!(round_ratio_ties_even(7, 2).unwrap(), 4);
        assert_eq!(round_ratio_ties_even(3 * 48_000, 32_000).unwrap(), 4);
    }

    #[test]
    fn flush_accounts_only_the_first_played_prefix_across_resident_chunks() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-flush", 32))
            .unwrap();

        let first = frame_effect(controller.pump().unwrap());
        assert_eq!((first.seq, first.frame_count, first.final_), (0, 16, false));
        controller
            .on_output_drained(output_ack("agent-0-flush", 0, 16, false, STATUS_ACCEPTED))
            .unwrap();
        let second = frame_effect(controller.pump().unwrap());
        assert_eq!(
            (second.seq, second.frame_count, second.final_),
            (1, 16, true)
        );

        let pause = controller
            .on_control(control(
                PlaybackControlMessage::PAUSE as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();
        assert!(pause.into_iter().any(|effect| matches!(
            effect,
            Effect::Flush(command) if command.keys.len() == 2
        )));
        controller.finish_flush_service();
        controller
            .on_output_drained(output_ack("agent-0-flush", 0, 6, false, STATUS_FLUSHED))
            .unwrap();
        controller
            .on_output_drained(output_ack("agent-0-flush", 1, 0, false, STATUS_FLUSHED))
            .unwrap();
        assert!(controller.flush_request.is_none());
        assert!(controller.output_pending.is_empty());

        controller
            .on_control(control(
                PlaybackControlMessage::RESUME as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();
        assert_eq!(frame_effect(controller.pump().unwrap()).sample_index, 6);
    }

    #[test]
    fn abort_before_accept_suppresses_late_flush_accounting_and_releases_hold() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::System, "system-abort", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());

        let effects = controller
            .on_control(control(
                PlaybackControlMessage::ABORT_SYSTEM as u8,
                PlaybackControlMessage::SYSTEM as u8,
                "system-abort",
            ))
            .unwrap();
        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Flush(command) if command.keys == vec![("system-abort".to_owned(), 0)]
        )));
        assert!(!controller.machine.agent_pause_requested());
        controller.finish_flush_service();
        assert!(controller
            .on_output_drained(output_ack("system-abort", 0, 0, false, STATUS_FLUSHED,))
            .unwrap()
            .is_empty());
        assert!(controller.flush_request.is_none());
        assert!(controller.output_pending.is_empty());
        assert!(controller.suppressed_output_acks.is_empty());
    }

    #[test]
    fn targeted_system_abort_preserves_other_system_and_its_hold() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-old", 8))
            .unwrap();
        machine.next_chunk().unwrap();
        machine
            .enqueue(utterance(Kind::System, "system-1", 2))
            .unwrap();
        machine
            .enqueue(utterance(Kind::System, "system-2", 2))
            .unwrap();

        let first = machine.next_chunk().unwrap().unwrap();
        assert_eq!(first.utterance_id, "system-1");
        let aborted = machine.abort_system("system-2", true).unwrap();
        assert_eq!(aborted.len(), 1);
        assert_eq!(aborted[0].utterance_id, "system-2");
        assert!(machine.agent_pause_requested());
        machine
            .output_drained(
                &first.utterance_id,
                first.seq,
                first.frame_count,
                first.is_final,
                STATUS_DRAINED,
            )
            .unwrap();
        assert!(machine.next_chunk().unwrap().is_none());
        assert_eq!(
            machine.abort_system("system-1", true).unwrap()[0].name,
            EventName::AgentResumed
        );
    }
}
