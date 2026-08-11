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
use r2r::fv_audio_interfaces::msg::{
    OutputDrained as OutputDrainedMessage, PlaybackControl as PlaybackControlMessage,
    PlaybackEvent as PlaybackEventMessage, PlaybackFrame,
    SynthesizedSpeechChunk as SynthesizedSpeechChunkMessage,
    TtsAlignmentEvent as TtsAlignmentEventMessage, TtsSourceProgress as TtsSourceProgressMessage,
    TtsTimingEvent,
};
use r2r::fv_audio_interfaces::srv::FlushAudio;
use r2r::fv_speech_interfaces::msg::AudioFrame;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const MAX_ABORTED_SYSTEM_IDS: usize = 64;
// Both synthesized-speech subscriptions use KEEP_LAST(10), so a terminal ID
// cannot legitimately reappear after 1024 newer utterances. Keep a much wider
// reorder window while bounding the lifetime memory of the always-on node.
const MAX_TERMINAL_SPEECH_IDS: usize = 1024;
const MAGPIE_CODEC_FRAME_SAMPLES: u64 = 1024;
const MAGPIE_INITIAL_CODEC_FRAMES: u64 = 4;
const MAGPIE_STEADY_CODEC_FRAMES: u64 = 8;
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
// Agent and SYSTEM are MagpieTTS-RT streams. Their first converted playout
// chunk is published immediately and pacing begins from that boundary; waiting
// for or bursting additional chunks would move the streaming contract away
// from the model's first available PCM. CUE is fully assembled before enqueue
// and retains its existing three-chunk device prefill.
const TTS_INITIAL_PLAYOUT_CHUNKS: usize = 1;
const CUE_INITIAL_PLAYOUT_CHUNKS: usize = 3;
const OUTPUT_PROGRESS_TIMEOUT: Duration = Duration::from_secs(5);
const OUTPUT_FLUSH_TIMEOUT: Duration = Duration::from_secs(5);
const PCM_CONVERSION_TIMEOUT: Duration = Duration::from_secs(10);
// fv_tts fails a request after 30 seconds without native progress and allows
// two more seconds for its cancellation terminal. If that process disappears,
// no ABORT can arrive. This wider receiver deadline turns the otherwise
// unbounded partial stream into an explicit targeted abort without racing the
// producer's own terminal path.
const SPEECH_INPUT_PROGRESS_TIMEOUT: Duration = Duration::from_secs(35);
const MAX_BUFFERED_PLAYBACK_SECONDS: u64 = 120;
const MAX_BUFFERED_UTTERANCES: usize = 16;
const MAX_CUE_ASSEMBLIES: usize = 16;
const MAX_CUE_ASSEMBLY_BYTES: usize = 8 * 1024 * 1024;
// A PAUSE transaction normally defers at most one RESUME or DISCARD. Keep a
// bounded FIFO so malformed publishers cannot grow process-lifetime state.
const MAX_DEFERRED_PAUSE_CONTROLS: usize = 8;
const RESAMPLER_FLUSH_INPUT_FRAMES: usize = 64;

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
    AgentPauseCommitted,
    AgentResumed,
    AgentDiscarded,
    AgentAborted,
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
            Self::AgentPauseCommitted => PlaybackEventMessage::AGENT_PAUSE_COMMITTED as u8,
            Self::AgentResumed => PlaybackEventMessage::AGENT_RESUMED as u8,
            Self::AgentDiscarded => PlaybackEventMessage::AGENT_DISCARDED as u8,
            Self::AgentAborted => PlaybackEventMessage::AGENT_ABORTED as u8,
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
            | Self::AgentPauseCommitted
            | Self::AgentResumed
            | Self::AgentDiscarded
            | Self::AgentAborted
            | Self::AgentCompleted => Kind::Agent,
            Self::SystemStarted | Self::SystemCompleted | Self::SystemAborted => Kind::System,
            Self::CueStarted | Self::CueCompleted => Kind::Cue,
        }
    }

    fn invalidates(self) -> Option<Kind> {
        match self {
            Self::AgentPaused | Self::AgentDiscarded | Self::AgentAborted => Some(Kind::Agent),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PlaybackEvent {
    name: EventName,
    kind: Kind,
    pause_id: String,
    request_id: String,
    generation_id: String,
    utterance_id: String,
    played_frames: u64,
    total_frames_valid: bool,
    total_frames: u64,
    source_position_valid: bool,
    played_source_characters: u64,
    total_source_characters: u64,
}

impl PlaybackEvent {
    fn terminal_from_identity(
        name: EventName,
        request_id: String,
        generation_id: String,
        utterance_id: String,
    ) -> Self {
        Self {
            name,
            kind: name.kind(),
            pause_id: String::new(),
            request_id,
            generation_id,
            utterance_id,
            played_frames: 0,
            total_frames_valid: false,
            total_frames: 0,
            source_position_valid: false,
            played_source_characters: 0,
            total_source_characters: 0,
        }
    }

    fn to_message(&self, playout_generation: u64) -> PlaybackEventMessage {
        PlaybackEventMessage {
            event: self.name.code(),
            kind: self.kind.code(),
            pause_id: self.pause_id.clone(),
            request_id: self.request_id.clone(),
            generation_id: self.generation_id.clone(),
            utterance_id: self.utterance_id.clone(),
            played_frames: self.played_frames,
            total_frames_valid: self.total_frames_valid,
            total_frames: self.total_frames,
            source_position_valid: self.source_position_valid,
            played_source_characters: self.played_source_characters,
            total_source_characters: self.total_source_characters,
            playout_generation,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct SourceAlignment {
    end_frame: u64,
    source_end: u64,
}

#[derive(Clone, Copy, Debug)]
struct PcmFormat {
    sample_rate_hz: u32,
    channels: u32,
    bit_depth: u32,
}

impl PcmFormat {
    fn pcm16(sample_rate_hz: u32, channels: u32) -> Self {
        Self {
            sample_rate_hz,
            channels,
            bit_depth: 16,
        }
    }
}

#[derive(Debug)]
struct PlaybackIdentity {
    kind: Kind,
    request_id: String,
    generation_id: String,
    utterance_id: String,
}

#[derive(Debug)]
struct Utterance {
    kind: Kind,
    request_id: String,
    generation_id: String,
    utterance_id: String,
    pcm: Vec<u8>,
    sample_rate_hz: u32,
    channels: u32,
    bit_depth: u32,
    alignments: Vec<SourceAlignment>,
    total_source_characters: u64,
    conversion_complete: bool,
    offset_bytes: usize,
    played_offset_frames: u64,
    started: bool,
    start_confirmed: bool,
    first_playback_timing_emitted: bool,
}

impl Utterance {
    fn new(
        identity: PlaybackIdentity,
        pcm: Vec<u8>,
        format: PcmFormat,
        alignments: Vec<SourceAlignment>,
        total_source_characters: u64,
        conversion_complete: bool,
    ) -> Result<Self, String> {
        if identity.utterance_id.is_empty() {
            return Err("utterance_id must not be empty".to_owned());
        }
        if identity.kind == Kind::Cue {
            if !identity.request_id.is_empty() || !identity.generation_id.is_empty() {
                return Err("cue playback cannot carry TTS identity".to_owned());
            }
        } else {
            validate_uuid(&identity.request_id, "request_id")?;
            validate_uuid(&identity.generation_id, "generation_id")?;
        }
        if format.sample_rate_hz == 0 || format.channels == 0 || format.bit_depth != 16 {
            return Err("playback requires positive rate/channels and PCM16".to_owned());
        }
        let bytes_per_frame = format.channels as usize * 2;
        if !pcm.len().is_multiple_of(bytes_per_frame) {
            return Err("PCM payload is not aligned to audio frames".to_owned());
        }
        let total_frames = (pcm.len() / bytes_per_frame) as u64;
        if identity.kind == Kind::Cue {
            if !alignments.is_empty() || total_source_characters != 0 {
                return Err("cue playback must not contain speech alignment".to_owned());
            }
        } else {
            validate_source_alignments(
                &alignments,
                total_source_characters,
                total_frames,
                conversion_complete,
            )?;
        }
        if conversion_complete && pcm.is_empty() {
            return Err("completed playback PCM must contain at least one audio frame".to_owned());
        }
        Ok(Self {
            kind: identity.kind,
            request_id: identity.request_id,
            generation_id: identity.generation_id,
            utterance_id: identity.utterance_id,
            pcm,
            sample_rate_hz: format.sample_rate_hz,
            channels: format.channels,
            bit_depth: format.bit_depth,
            alignments,
            total_source_characters,
            conversion_complete,
            offset_bytes: 0,
            played_offset_frames: 0,
            started: false,
            start_confirmed: false,
            first_playback_timing_emitted: false,
        })
    }

    fn bytes_per_frame(&self) -> usize {
        self.channels as usize * (self.bit_depth as usize / 8)
    }

    fn total_frames(&self) -> u64 {
        (self.pcm.len() / self.bytes_per_frame()) as u64
    }

    fn append_pcm(&mut self, pcm: &[u8]) -> Result<(), String> {
        if self.conversion_complete {
            return Err("cannot append PCM after conversion completed".to_owned());
        }
        if !pcm.len().is_multiple_of(self.bytes_per_frame()) {
            return Err("converted PCM is not aligned to output frames".to_owned());
        }
        self.pcm.extend_from_slice(pcm);
        Ok(())
    }

    fn append_alignments(&mut self, alignments: Vec<SourceAlignment>) -> Result<(), String> {
        if self.conversion_complete {
            return Err("cannot append alignment after conversion completed".to_owned());
        }
        self.alignments.extend(alignments);
        validate_source_alignments(
            &self.alignments,
            self.total_source_characters,
            self.total_frames(),
            false,
        )
    }

    fn complete_conversion(&mut self) -> Result<(), String> {
        if self.conversion_complete {
            return Err("speech conversion completed more than once".to_owned());
        }
        self.conversion_complete = true;
        if self.pcm.is_empty() {
            return Err("completed playback PCM must contain at least one audio frame".to_owned());
        }
        validate_source_alignments(
            &self.alignments,
            self.total_source_characters,
            self.total_frames(),
            true,
        )
    }

    fn source_position(&self) -> (bool, u64, u64) {
        if self.kind != Kind::Agent {
            return (false, 0, 0);
        }
        let played = self
            .alignments
            .iter()
            .take_while(|mark| mark.end_frame <= self.played_offset_frames)
            .last()
            .map_or(0, |mark| mark.source_end);
        (true, played, self.total_source_characters)
    }
}

fn validate_source_alignments(
    alignments: &[SourceAlignment],
    total_source_characters: u64,
    total_frames: u64,
    conversion_complete: bool,
) -> Result<(), String> {
    if total_source_characters == 0 {
        return Err("speech playback requires non-empty source text".to_owned());
    }
    let mut previous_source_end = 0;
    let mut previous_end_frame = 0;
    for alignment in alignments {
        if alignment.source_end < previous_source_end
            || alignment.source_end > total_source_characters
            || alignment.end_frame < previous_end_frame
            || (conversion_complete && alignment.end_frame > total_frames)
        {
            return Err("speech source alignment is malformed".to_owned());
        }
        previous_source_end = alignment.source_end;
        previous_end_frame = alignment.end_frame;
    }
    if conversion_complete
        && alignments
            .last()
            .is_none_or(|alignment| alignment.source_end != total_source_characters)
    {
        return Err("completed speech alignment does not reach the source text end".to_owned());
    }
    Ok(())
}

type UtteranceRef = Rc<RefCell<Utterance>>;

#[derive(Clone, Debug, PartialEq, Eq)]
struct PlaybackChunk {
    request_id: String,
    generation_id: String,
    utterance_id: String,
    kind: Kind,
    seq: u64,
    sample_index: u64,
    frame_count: u32,
    sample_rate_hz: u32,
    channels: u32,
    data: Vec<u8>,
    is_final: bool,
    emit_first_playback_timing: bool,
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

    fn unique_items(&self) -> Vec<UtteranceRef> {
        let mut seen = HashSet::new();
        let mut items = Vec::new();
        for item in self
            .active
            .iter()
            .chain(self.paused_agent.iter())
            .chain(self.agent_queue.iter())
            .chain(self.system_queue.iter())
            .chain(self.cue_queue.iter())
            .chain(self.awaiting_drain.values())
        {
            if seen.insert(Rc::as_ptr(item)) {
                items.push(Rc::clone(item));
            }
        }
        items
    }

    fn retained_pcm_bytes(&self) -> Result<usize, String> {
        self.unique_items()
            .into_iter()
            .try_fold(0_usize, |total, item| {
                total
                    .checked_add(item.borrow().pcm.len())
                    .ok_or_else(|| "retained playback PCM byte count overflowed".to_owned())
            })
    }

    fn utterance_count(&self) -> usize {
        self.unique_items().len()
    }

    fn enqueue(&mut self, utterance: Utterance) -> Result<Vec<PlaybackEvent>, String> {
        self.enqueue_with_ref(utterance).map(|(_, events)| events)
    }

    fn enqueue_with_ref(
        &mut self,
        utterance: Utterance,
    ) -> Result<(UtteranceRef, Vec<PlaybackEvent>), String> {
        let item = Rc::new(RefCell::new(utterance));
        let kind = item.borrow().kind;
        let utterance_id = item.borrow().utterance_id.clone();
        let mut events = Vec::new();
        match kind {
            Kind::System => {
                if self.aborted_system_ids.contains(&utterance_id) {
                    return Ok((
                        Rc::clone(&item),
                        vec![Self::event(EventName::SystemAborted, &item)],
                    ));
                }
                events.extend(self.reserve_system(&utterance_id)?);
                self.system_queue.push_back(Rc::clone(&item));
            }
            Kind::Cue => self.cue_queue.push_back(Rc::clone(&item)),
            Kind::Agent => self.agent_queue.push_back(Rc::clone(&item)),
        }
        Ok((item, events))
    }

    fn abort_speech(
        &mut self,
        kind: Kind,
        utterance_id: &str,
    ) -> Result<Vec<PlaybackEvent>, String> {
        if !matches!(kind, Kind::Agent | Kind::System) || utterance_id.is_empty() {
            return Err("speech abort requires agent/system kind and utterance_id".to_owned());
        }
        if kind == Kind::System {
            return self.abort_system(utterance_id, true);
        }

        let mut removed = Vec::new();
        if self.active.as_ref().is_some_and(|item| {
            let item = item.borrow();
            item.kind == kind && item.utterance_id == utterance_id
        }) {
            removed.push(self.active.take().expect("checked above"));
        }
        if self.paused_agent.as_ref().is_some_and(|item| {
            let item = item.borrow();
            item.utterance_id == utterance_id
        }) {
            removed.push(self.paused_agent.take().expect("checked above"));
        }
        let mut queued = VecDeque::new();
        while let Some(item) = self.agent_queue.pop_front() {
            if item.borrow().utterance_id == utterance_id {
                removed.push(item);
            } else {
                queued.push_back(item);
            }
        }
        self.agent_queue = queued;
        if let Some(item) = self.awaiting_drain.remove(utterance_id) {
            if item.borrow().kind == kind {
                removed.push(item);
            } else {
                self.awaiting_drain.insert(utterance_id.to_owned(), item);
            }
        }
        if removed.len() > 1 {
            return Err("utterance exists in multiple playback states".to_owned());
        }
        Ok(removed
            .iter()
            .map(|item| Self::event(EventName::AgentAborted, item))
            .collect())
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

    fn pause_committed(&self, pause_id: String) -> PlaybackEvent {
        if let Some(paused) = self.paused_agent.as_ref() {
            let mut event = Self::event(EventName::AgentPauseCommitted, paused);
            event.pause_id = pause_id;
            return event;
        }
        PlaybackEvent {
            name: EventName::AgentPauseCommitted,
            kind: Kind::Agent,
            pause_id,
            request_id: String::new(),
            generation_id: String::new(),
            utterance_id: String::new(),
            played_frames: 0,
            total_frames_valid: false,
            total_frames: 0,
            source_position_valid: false,
            played_source_characters: 0,
            total_source_characters: 0,
        }
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
        self.select_active_if_needed();
        let Some(item_ref) = self.active.as_ref().cloned() else {
            return Ok(None);
        };
        let mut item = item_ref.borrow_mut();
        let bytes_per_frame = item.bytes_per_frame();
        let start = item.offset_bytes;
        let chunk_bytes = self.chunk_frames as usize * bytes_per_frame;
        let available_bytes = item.pcm.len().saturating_sub(start);
        // Keep one not-yet-final output chunk buffered until COMPLETE. Otherwise a
        // streaming utterance ending exactly on a 20 ms boundary would have no
        // PlaybackFrame on which to carry the physical final flag.
        if available_bytes <= chunk_bytes && !item.conversion_complete {
            return Ok(None);
        }
        if available_bytes == 0 {
            if item.conversion_complete {
                return Err("completed utterance has no unpublished final PCM".to_owned());
            }
            return Ok(None);
        }
        item.started = true;
        let end = item.pcm.len().min(start.saturating_add(chunk_bytes));
        let data = item.pcm[start..end].to_vec();
        let sample_index = (item.offset_bytes / bytes_per_frame) as u64;
        let emit_first_playback_timing =
            item.kind != Kind::Cue && !item.first_playback_timing_emitted;
        if emit_first_playback_timing && sample_index != 0 {
            return Err("first TTS playback timing does not start at sample zero".to_owned());
        }
        if emit_first_playback_timing {
            item.first_playback_timing_emitted = true;
        }
        item.offset_bytes = end;
        let is_final = item.conversion_complete && end == item.pcm.len();
        let chunk = PlaybackChunk {
            request_id: item.request_id.clone(),
            generation_id: item.generation_id.clone(),
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
            emit_first_playback_timing,
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

    fn next_kind(&mut self) -> Option<Kind> {
        self.select_active_if_needed();
        self.active.as_ref().map(|item| item.borrow().kind)
    }

    fn select_active_if_needed(&mut self) {
        if self.active.is_none() {
            self.active = self.select_next();
        }
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
        // A partial flush updates the retained resume offset, but must not emit
        // a second logical pause event. The one exact physical prefix is
        // published only by AGENT_PAUSE_COMMITTED after both the keyed acks and
        // the device flush service complete.
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
        let (source_position_valid, played_source_characters, total_source_characters) =
            item.source_position();
        PlaybackEvent {
            name,
            kind: item.kind,
            pause_id: String::new(),
            request_id: item.request_id.clone(),
            generation_id: item.generation_id.clone(),
            utterance_id: item.utterance_id.clone(),
            played_frames: item.played_offset_frames,
            total_frames_valid: item.conversion_complete,
            total_frames: if item.conversion_complete {
                item.total_frames()
            } else {
                0
            },
            source_position_valid,
            played_source_characters,
            total_source_characters,
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

    fn start_speech(
        &self,
        input_rate_hz: u32,
        input_channels: u32,
    ) -> Result<StreamingPcmConverter, String> {
        StreamingPcmConverter::new(
            input_rate_hz,
            input_channels,
            self.output_rate_hz,
            self.output_channels,
        )
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
        let maximum_output_frames = round_ratio_ties_even(
            (input_frames
                .checked_add(RESAMPLER_FLUSH_INPUT_FRAMES)
                .ok_or_else(|| "cue resampler flush frame count overflowed".to_owned())?)
                as u128
                * u128::from(self.output_rate_hz),
            u128::from(sample_rate_hz),
        )?;
        let maximum_output_bytes = maximum_output_frames
            .checked_mul(self.output_channels as usize)
            .and_then(|value| value.checked_mul(2))
            .ok_or_else(|| "cue resampler maximum output size overflowed".to_owned())?;

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
            // audioresample retains a short interpolation edge. Flush that
            // edge by sending an explicit zero boundary through GStreamer,
            // then retain exactly the original source duration. A short or
            // unexpectedly long result is still rejected below.
            let padding_bytes = RESAMPLER_FLUSH_INPUT_FRAMES
                .checked_mul(bytes_per_frame)
                .ok_or_else(|| "cue resampler flush byte count overflowed".to_owned())?;
            let mut padding = gst::Buffer::with_size(padding_bytes)
                .map_err(|error| format!("cannot allocate GStreamer flush buffer: {error}"))?;
            {
                let padding = padding
                    .get_mut()
                    .ok_or_else(|| "GStreamer flush buffer is unexpectedly shared".to_owned())?;
                let mut mapped = padding
                    .map_writable()
                    .map_err(|error| format!("cannot map GStreamer flush buffer: {error}"))?;
                mapped.as_mut_slice().fill(0);
                drop(mapped);
                padding.set_pts(gst::ClockTime::from_nseconds(frames_to_nanos(
                    u64::try_from(input_frames)
                        .map_err(|_| "cue input frame count exceeds uint64".to_owned())?,
                    sample_rate_hz,
                )?));
                padding.set_duration(gst::ClockTime::from_nseconds(frames_to_nanos(
                    u64::try_from(RESAMPLER_FLUSH_INPUT_FRAMES)
                        .map_err(|_| "cue flush frame count exceeds uint64".to_owned())?,
                    sample_rate_hz,
                )?));
            }
            source
                .push_buffer(padding)
                .map_err(|error| format!("GStreamer flush push-buffer failed: {error}"))?;
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
            if output.len() < expected_bytes || output.len() > maximum_output_bytes {
                return Err(format!(
                    "GStreamer cue conversion produced {} bytes; expected range is {expected_bytes}..={maximum_output_bytes}",
                    output.len(),
                ));
            }
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

struct StreamingPcmConverter {
    pipeline: gst::Pipeline,
    source: AppSrc,
    sink: AppSink,
    input_rate_hz: u32,
    input_channels: u32,
    output_rate_hz: u32,
    output_channels: u32,
    next_input_frame: u64,
    output_frames_emitted: u64,
    stopped: bool,
}

impl StreamingPcmConverter {
    fn new(
        input_rate_hz: u32,
        input_channels: u32,
        output_rate_hz: u32,
        output_channels: u32,
    ) -> Result<Self, String> {
        if input_rate_hz == 0 || input_channels == 0 || output_rate_hz == 0 || output_channels == 0
        {
            return Err("streaming PCM formats must be positive".to_owned());
        }
        let description = format!(
            "appsrc name=src format=time ! \
             audio/x-raw,format=F32LE,rate={input_rate_hz},channels={input_channels},layout=interleaved ! \
             audioconvert ! audioresample ! \
             audio/x-raw,format=S16LE,rate={output_rate_hz},channels={output_channels},layout=interleaved ! \
             appsink name=sink sync=false"
        );
        let pipeline = gst::parse::launch(&description)
            .map_err(|error| format!("cannot construct streaming GStreamer PCM pipeline: {error}"))?
            .downcast::<gst::Pipeline>()
            .map_err(|_| "streaming GStreamer conversion did not create a pipeline".to_owned())?;
        let source = pipeline
            .by_name("src")
            .ok_or_else(|| "streaming GStreamer PCM pipeline has no appsrc".to_owned())?
            .downcast::<AppSrc>()
            .map_err(|_| "streaming GStreamer src element is not appsrc".to_owned())?;
        let sink = pipeline
            .by_name("sink")
            .ok_or_else(|| "streaming GStreamer PCM pipeline has no appsink".to_owned())?
            .downcast::<AppSink>()
            .map_err(|_| "streaming GStreamer sink element is not appsink".to_owned())?;
        pipeline
            .set_state(gst::State::Playing)
            .map_err(|error| format!("cannot start streaming GStreamer PCM pipeline: {error}"))?;
        Ok(Self {
            pipeline,
            source,
            sink,
            input_rate_hz,
            input_channels,
            output_rate_hz,
            output_channels,
            next_input_frame: 0,
            output_frames_emitted: 0,
            stopped: false,
        })
    }

    fn push(&mut self, first_input_frame: u64, pcm: &[f32]) -> Result<Vec<u8>, String> {
        if self.stopped {
            return Err("cannot push PCM after streaming conversion stopped".to_owned());
        }
        if first_input_frame != self.next_input_frame {
            return Err(format!(
                "streaming converter input is not contiguous: {first_input_frame} != {}",
                self.next_input_frame
            ));
        }
        if pcm.is_empty() || !pcm.len().is_multiple_of(self.input_channels as usize) {
            return Err("streaming float32 PCM is empty or not frame aligned".to_owned());
        }
        let frame_count = u64::try_from(pcm.len() / self.input_channels as usize)
            .map_err(|_| "streaming PCM frame count exceeds uint64".to_owned())?;
        let payload_len = pcm
            .len()
            .checked_mul(std::mem::size_of::<f32>())
            .ok_or_else(|| "streaming PCM byte length overflowed".to_owned())?;
        let pts_ns = frames_to_nanos(first_input_frame, self.input_rate_hz)?;
        let duration_ns = frames_to_nanos(frame_count, self.input_rate_hz)?;
        let mut buffer = gst::Buffer::with_size(payload_len)
            .map_err(|error| format!("cannot allocate streaming GStreamer buffer: {error}"))?;
        {
            let buffer = buffer
                .get_mut()
                .ok_or_else(|| "streaming GStreamer input buffer is shared".to_owned())?;
            let mut mapped = buffer
                .map_writable()
                .map_err(|error| format!("cannot map streaming GStreamer input: {error}"))?;
            for (destination, sample) in mapped
                .as_mut_slice()
                .chunks_exact_mut(std::mem::size_of::<f32>())
                .zip(pcm)
            {
                destination.copy_from_slice(&sample.to_le_bytes());
            }
            drop(mapped);
            buffer.set_pts(gst::ClockTime::from_nseconds(pts_ns));
            buffer.set_duration(gst::ClockTime::from_nseconds(duration_ns));
            buffer.set_offset(first_input_frame);
            buffer.set_offset_end(
                first_input_frame
                    .checked_add(frame_count)
                    .ok_or_else(|| "streaming input buffer offset overflowed".to_owned())?,
            );
        }
        self.source
            .push_buffer(buffer)
            .map_err(|error| format!("streaming GStreamer push-buffer failed: {error}"))?;
        self.next_input_frame = self
            .next_input_frame
            .checked_add(frame_count)
            .ok_or_else(|| "streaming input frame index overflowed".to_owned())?;
        let output = self.drain_ready(Duration::from_millis(10))?;
        self.note_output(&output)?;
        let maximum_output_frames = ceil_ratio_u64(
            u128::from(self.next_input_frame) * u128::from(self.output_rate_hz),
            u128::from(self.input_rate_hz),
        )?;
        if self.output_frames_emitted > maximum_output_frames {
            return Err("GStreamer emitted audio beyond the available source duration".to_owned());
        }
        Ok(output)
    }

    fn finish(mut self) -> Result<Vec<u8>, String> {
        let source_input_frames = self.next_input_frame;
        let target_output_frames = ceil_ratio_u64(
            u128::from(source_input_frames) * u128::from(self.output_rate_hz),
            u128::from(self.input_rate_hz),
        )?;
        if self.output_frames_emitted > target_output_frames {
            return Err("GStreamer emitted audio beyond the source duration".to_owned());
        }
        let needed_frames = target_output_frames - self.output_frames_emitted;
        // audioresample retains a short interpolation edge at the end of a stream.
        // Feed an explicit zero boundary through the same resampler, then expose
        // exactly the ceil-scaled source duration. This is part of the conversion
        // contract, not a substitute for missing upstream PCM.
        let padding_frames = 64_usize;
        let padding_samples = padding_frames
            .checked_mul(self.input_channels as usize)
            .ok_or_else(|| "streaming resampler padding size overflowed".to_owned())?;
        let padding = vec![0.0_f32; padding_samples];
        let mut output = self.push(source_input_frames, &padding)?;
        self.source
            .end_of_stream()
            .map_err(|error| format!("streaming GStreamer end-of-stream failed: {error}"))?;
        let result = self.drain_to_eos().and_then(|tail| {
            self.note_output(&tail)?;
            output.extend(tail);
            let needed_bytes = usize::try_from(needed_frames)
                .ok()
                .and_then(|frames| frames.checked_mul(self.output_channels as usize))
                .and_then(|samples| samples.checked_mul(2))
                .ok_or_else(|| "streaming output duration exceeds usize".to_owned())?;
            if output.len() < needed_bytes {
                return Err("GStreamer did not produce the complete source duration".to_owned());
            }
            output.truncate(needed_bytes);
            Ok(output)
        });
        let stop_result = self.stop();
        match (result, stop_result) {
            (Ok(output), Ok(())) => Ok(output),
            (Err(error), _) => Err(error),
            (Ok(_), Err(error)) => Err(error),
        }
    }

    fn abort(mut self) -> Result<(), String> {
        self.stop()
    }

    fn drain_ready(&self, first_wait: Duration) -> Result<Vec<u8>, String> {
        let mut output = Vec::new();
        let first_wait_ns = u64::try_from(first_wait.as_nanos())
            .map_err(|_| "GStreamer wait duration exceeds uint64 nanoseconds".to_owned())?;
        if let Some(sample) = self
            .sink
            .try_pull_sample(gst::ClockTime::from_nseconds(first_wait_ns))
        {
            append_gstreamer_sample(&sample, self.output_channels, &mut output)?;
        }
        while let Some(sample) = self.sink.try_pull_sample(gst::ClockTime::ZERO) {
            append_gstreamer_sample(&sample, self.output_channels, &mut output)?;
        }
        self.check_bus_error()?;
        if self.sink.is_eos() {
            return Err("streaming GStreamer reached EOS before COMPLETE".to_owned());
        }
        Ok(output)
    }

    fn drain_to_eos(&self) -> Result<Vec<u8>, String> {
        let mut output = Vec::new();
        let deadline = Instant::now() + PCM_CONVERSION_TIMEOUT;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Err(format!(
                    "streaming GStreamer EOS drain exceeded {} seconds",
                    PCM_CONVERSION_TIMEOUT.as_secs()
                ));
            }
            let wait = deadline
                .saturating_duration_since(now)
                .min(Duration::from_secs(1));
            let wait_ns = u64::try_from(wait.as_nanos())
                .map_err(|_| "GStreamer wait duration exceeds uint64 nanoseconds".to_owned())?;
            if let Some(sample) = self
                .sink
                .try_pull_sample(gst::ClockTime::from_nseconds(wait_ns))
            {
                append_gstreamer_sample(&sample, self.output_channels, &mut output)?;
                continue;
            }
            self.check_bus_error()?;
            if self.sink.is_eos() {
                break;
            }
        }
        Ok(output)
    }

    fn check_bus_error(&self) -> Result<(), String> {
        let bus = self
            .pipeline
            .bus()
            .ok_or_else(|| "streaming GStreamer PCM pipeline has no bus".to_owned())?;
        if let Some(message) = bus.pop_filtered(&[gst::MessageType::Error]) {
            if let gst::MessageView::Error(error) = message.view() {
                return Err(format!(
                    "streaming GStreamer conversion failed: {}: {}",
                    error.error(),
                    error.debug().unwrap_or_default()
                ));
            }
        }
        Ok(())
    }

    fn note_output(&mut self, output: &[u8]) -> Result<(), String> {
        let bytes_per_frame = self.output_channels as usize * 2;
        if !output.len().is_multiple_of(bytes_per_frame) {
            return Err("streaming GStreamer output is not frame aligned".to_owned());
        }
        self.output_frames_emitted = self
            .output_frames_emitted
            .checked_add(
                u64::try_from(output.len() / bytes_per_frame)
                    .map_err(|_| "streaming output frame count exceeds uint64".to_owned())?,
            )
            .ok_or_else(|| "streaming output frame count overflowed".to_owned())?;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), String> {
        if self.stopped {
            return Ok(());
        }
        self.pipeline
            .set_state(gst::State::Null)
            .map_err(|error| format!("cannot stop streaming GStreamer PCM pipeline: {error}"))?;
        self.stopped = true;
        Ok(())
    }
}

impl Drop for StreamingPcmConverter {
    fn drop(&mut self) {
        if !self.stopped {
            let _ = self.pipeline.set_state(gst::State::Null);
            self.stopped = true;
        }
    }
}

fn append_gstreamer_sample(
    sample: &gst::Sample,
    output_channels: u32,
    output: &mut Vec<u8>,
) -> Result<(), String> {
    let buffer = sample
        .buffer()
        .ok_or_else(|| "streaming GStreamer output sample has no buffer".to_owned())?;
    let mapped = buffer
        .map_readable()
        .map_err(|error| format!("cannot map streaming GStreamer output: {error}"))?;
    let bytes_per_frame = output_channels as usize * 2;
    if !mapped.size().is_multiple_of(bytes_per_frame) {
        return Err("streaming GStreamer output is not frame aligned".to_owned());
    }
    output.extend_from_slice(mapped.as_slice());
    Ok(())
}

fn frames_to_nanos(frames: u64, sample_rate_hz: u32) -> Result<u64, String> {
    if sample_rate_hz == 0 {
        return Err("cannot timestamp audio with a zero sample rate".to_owned());
    }
    u64::try_from(
        u128::from(frames)
            .checked_mul(1_000_000_000)
            .ok_or_else(|| "audio timestamp multiplication overflowed".to_owned())?
            / u128::from(sample_rate_hz),
    )
    .map_err(|_| "audio timestamp exceeds uint64 nanoseconds".to_owned())
}

fn ceil_ratio_u64(numerator: u128, denominator: u128) -> Result<u64, String> {
    if denominator == 0 {
        return Err("cannot ceil a ratio with a zero denominator".to_owned());
    }
    let adjusted = numerator
        .checked_add(denominator - 1)
        .ok_or_else(|| "alignment scaling overflowed".to_owned())?;
    u64::try_from(adjusted / denominator)
        .map_err(|_| "scaled alignment boundary exceeds uint64".to_owned())
}

struct SpeechStream {
    kind: Kind,
    sample_rate_hz: u32,
    channels: u32,
    next_sequence: u64,
    next_sample_index: u64,
    total_prepared_tokens: u64,
    committed_text_tokens: u64,
    last_alignment_sample: u64,
    last_alignment_tokens: u64,
    final_audio_seen: bool,
    source_progress: Vec<TtsSourceProgressMessage>,
    last_progress_at: Instant,
    item: UtteranceRef,
    converter: StreamingPcmConverter,
}

impl SpeechStream {
    fn validate_identity(&self, message: &SynthesizedSpeechChunkMessage) -> Result<(), String> {
        let item = self.item.borrow();
        if message.request_id != item.request_id || message.generation_id != item.generation_id {
            return Err("TTS request/generation identity changed inside an utterance".to_owned());
        }
        Ok(())
    }

    fn validate_audio(
        &self,
        message: &SynthesizedSpeechChunkMessage,
        output_rate_hz: u32,
    ) -> Result<(u64, Vec<SourceAlignment>), String> {
        self.validate_identity(message)?;
        if message.sequence != self.next_sequence {
            return Err(format!(
                "non-contiguous synthesized speech sequence: {} != {}",
                message.sequence, self.next_sequence
            ));
        }
        if message.first_sample_index != self.next_sample_index {
            return Err(format!(
                "non-contiguous synthesized speech sample index: {} != {}",
                message.first_sample_index, self.next_sample_index
            ));
        }
        if message.sample_rate_hz != self.sample_rate_hz || message.channels != self.channels {
            return Err("synthesized speech format changed inside an utterance".to_owned());
        }
        if !message.source_text.is_empty() || !message.source_progress.is_empty() {
            return Err(
                "source text/progress must appear only on synthesized speech sequence zero"
                    .to_owned(),
            );
        }
        if !message.pcm_f32.is_empty() {
            validate_float_pcm(&message.pcm_f32, self.channels)?;
        }
        let frame_count = u64::try_from(message.pcm_f32.len() / self.channels as usize)
            .map_err(|_| "synthesized speech frame count exceeds uint64".to_owned())?;
        if self.final_audio_seen {
            return Err("synthesized speech audio arrived after its final chunk".to_owned());
        }
        validate_magpie_audio_shape(message.sequence, frame_count, message.final_chunk)?;
        let next_sample_index = self
            .next_sample_index
            .checked_add(frame_count)
            .ok_or_else(|| "synthesized speech sample index overflowed".to_owned())?;
        let alignments = validate_alignment_events(
            &message.alignment_events,
            message.committed_text_tokens,
            AlignmentValidation {
                previous_committed_text_tokens: self.committed_text_tokens,
                previous_sample_index: self.last_alignment_sample,
                previous_alignment_tokens: self.last_alignment_tokens,
                total_prepared_tokens: self.total_prepared_tokens,
                payload_first_sample: self.next_sample_index,
                available_input_samples: next_sample_index,
                source_progress: &self.source_progress,
                input_rate_hz: self.sample_rate_hz,
                output_rate_hz,
            },
        )?;
        Ok((frame_count, alignments))
    }

    fn validate_terminal(&self, message: &SynthesizedSpeechChunkMessage) -> Result<(), String> {
        self.validate_identity(message)?;
        if message.sequence != self.next_sequence
            || message.first_sample_index != self.next_sample_index
        {
            return Err("speech terminal does not carry the next sequence/sample index".to_owned());
        }
        if message.sample_rate_hz != self.sample_rate_hz || message.channels != self.channels {
            return Err("speech terminal format does not match its audio stream".to_owned());
        }
        validate_empty_terminal_payload(message)?;
        Ok(())
    }

    fn accept_audio(
        &mut self,
        message: &SynthesizedSpeechChunkMessage,
        frame_count: u64,
        output: &[u8],
        alignments: Vec<SourceAlignment>,
    ) -> Result<(), String> {
        if !output.is_empty() {
            self.item.borrow_mut().append_pcm(output)?;
        }
        if !alignments.is_empty() {
            self.item.borrow_mut().append_alignments(alignments)?;
        }
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| "synthesized speech sequence overflowed".to_owned())?;
        self.next_sample_index = self
            .next_sample_index
            .checked_add(frame_count)
            .ok_or_else(|| "synthesized speech sample index overflowed".to_owned())?;
        self.committed_text_tokens = message.committed_text_tokens;
        self.final_audio_seen = message.final_chunk;
        if let Some(last) = message.alignment_events.last() {
            self.last_alignment_sample = last.sample_index;
            self.last_alignment_tokens = last.committed_text_tokens;
        }
        self.last_progress_at = Instant::now();
        Ok(())
    }
}

fn validate_source_progress(
    source_text: &str,
    progress: &[TtsSourceProgressMessage],
) -> Result<(u64, u64), String> {
    if source_text.is_empty() {
        return Err("synthesized speech source_text must not be empty".to_owned());
    }
    if progress.is_empty() {
        return Err("synthesized speech source_progress must not be empty".to_owned());
    }
    let total_characters = u64::try_from(source_text.chars().count())
        .map_err(|_| "source character count exceeds uint64".to_owned())?;
    let total_utf8 = u64::try_from(source_text.len())
        .map_err(|_| "source UTF-8 length exceeds uint64".to_owned())?;
    let mut previous_char_end = 0;
    let mut previous_utf8_end = 0;
    for (index, item) in progress.iter().enumerate() {
        let expected_tokens =
            u64::try_from(index).map_err(|_| "source progress index exceeds uint64".to_owned())?;
        let utf8_end = usize::try_from(item.source_utf8_end)
            .map_err(|_| "source UTF-8 offset exceeds usize".to_owned())?;
        if item.committed_text_tokens != expected_tokens
            || item.source_char_end < previous_char_end
            || item.source_char_end > total_characters
            || item.source_utf8_end < previous_utf8_end
            || item.source_utf8_end > total_utf8
            || !source_text.is_char_boundary(utf8_end)
            || source_text[..utf8_end].chars().count() as u64 != item.source_char_end
        {
            return Err("synthesized speech source_progress is malformed".to_owned());
        }
        previous_char_end = item.source_char_end;
        previous_utf8_end = item.source_utf8_end;
    }
    let last = progress.last().expect("non-empty progress checked above");
    if progress[0].source_char_end != 0
        || progress[0].source_utf8_end != 0
        || last.source_char_end != total_characters
        || last.source_utf8_end != total_utf8
    {
        return Err("source_progress does not span the exact source text".to_owned());
    }
    Ok((total_characters, last.committed_text_tokens))
}

fn validate_float_pcm(pcm: &[f32], channels: u32) -> Result<(), String> {
    if channels == 0 || pcm.is_empty() || !pcm.len().is_multiple_of(channels as usize) {
        return Err("synthesized float32 PCM is empty or not frame aligned".to_owned());
    }
    if pcm.iter().any(|sample| !sample.is_finite()) {
        return Err("synthesized float32 PCM contains a non-finite sample".to_owned());
    }
    Ok(())
}

fn validate_magpie_audio_shape(
    sequence: u64,
    frame_count: u64,
    final_chunk: bool,
) -> Result<(), String> {
    if !frame_count.is_multiple_of(MAGPIE_CODEC_FRAME_SAMPLES) {
        return Err("MagpieTTS-RT PCM is not aligned to a 1024-sample codec frame".to_owned());
    }
    let codec_frames = frame_count / MAGPIE_CODEC_FRAME_SAMPLES;
    if sequence == 0 {
        if codec_frames != MAGPIE_INITIAL_CODEC_FRAMES || final_chunk {
            return Err(
                "first MagpieTTS-RT chunk must contain exactly four non-final codec frames"
                    .to_owned(),
            );
        }
    } else if codec_frames == 0 && final_chunk {
        // A zero-frame FINAL is a control marker. It closes the stream
        // without writing PCM or advancing the sample index.
    } else if codec_frames == 0 || codec_frames > MAGPIE_STEADY_CODEC_FRAMES {
        return Err(
            "later MagpieTTS-RT chunks must contain one through eight codec frames".to_owned(),
        );
    } else if !final_chunk && codec_frames != MAGPIE_STEADY_CODEC_FRAMES {
        return Err(
            "non-final MagpieTTS-RT chunks after the first must contain exactly eight codec frames"
                .to_owned(),
        );
    }
    Ok(())
}

struct AlignmentValidation<'a> {
    previous_committed_text_tokens: u64,
    previous_sample_index: u64,
    previous_alignment_tokens: u64,
    total_prepared_tokens: u64,
    payload_first_sample: u64,
    available_input_samples: u64,
    source_progress: &'a [TtsSourceProgressMessage],
    input_rate_hz: u32,
    output_rate_hz: u32,
}

fn validate_alignment_events(
    events: &[TtsAlignmentEventMessage],
    committed_text_tokens: u64,
    validation: AlignmentValidation<'_>,
) -> Result<Vec<SourceAlignment>, String> {
    if committed_text_tokens < validation.previous_committed_text_tokens
        || committed_text_tokens > validation.total_prepared_tokens
    {
        return Err("committed_text_tokens is not monotonic or exceeds the source map".to_owned());
    }
    let payload_sample_count = validation
        .available_input_samples
        .checked_sub(validation.payload_first_sample)
        .ok_or_else(|| "synthesized speech sample range is reversed".to_owned())?;
    let maximum_events = payload_sample_count.div_ceil(2 * MAGPIE_CODEC_FRAME_SAMPLES);
    if u64::try_from(events.len()).map_err(|_| "alignment event count exceeds uint64".to_owned())?
        > maximum_events
    {
        return Err("synthesized speech has more than one alignment per decoder step".to_owned());
    }
    let mut sample_cursor = validation.previous_sample_index;
    let mut token_cursor = validation.previous_alignment_tokens;
    let mut converted = Vec::with_capacity(events.len());
    for event in events {
        if event.sample_index <= sample_cursor
            || event.sample_index <= validation.payload_first_sample
            || event.sample_index > validation.available_input_samples
            || !event
                .sample_index
                .is_multiple_of(MAGPIE_CODEC_FRAME_SAMPLES)
            || event.committed_text_tokens <= token_cursor
            || event.committed_text_tokens > committed_text_tokens
        {
            return Err("synthesized speech alignment event is malformed".to_owned());
        }
        let progress_index = usize::try_from(event.committed_text_tokens)
            .map_err(|_| "alignment token position exceeds usize".to_owned())?;
        let progress = validation
            .source_progress
            .get(progress_index)
            .ok_or_else(|| "alignment token position is absent from source_progress".to_owned())?;
        converted.push(SourceAlignment {
            end_frame: ceil_ratio_u64(
                u128::from(event.sample_index) * u128::from(validation.output_rate_hz),
                u128::from(validation.input_rate_hz),
            )?,
            source_end: progress.source_char_end,
        });
        sample_cursor = event.sample_index;
        token_cursor = event.committed_text_tokens;
    }
    if committed_text_tokens > validation.previous_committed_text_tokens
        && events
            .last()
            .is_none_or(|event| event.committed_text_tokens != committed_text_tokens)
    {
        return Err("advanced committed_text_tokens lacks a matching alignment event".to_owned());
    }
    Ok(converted)
}

fn validate_empty_terminal_payload(message: &SynthesizedSpeechChunkMessage) -> Result<(), String> {
    if !message.pcm_f32.is_empty()
        || message.final_chunk
        || message.committed_text_tokens != 0
        || !message.alignment_events.is_empty()
        || !message.source_text.is_empty()
        || !message.source_progress.is_empty()
    {
        return Err("speech COMPLETE/ABORT must not carry audio or alignment data".to_owned());
    }
    Ok(())
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
    pause_id: Option<String>,
}

#[derive(Debug)]
struct FlushCommand {
    keys: Vec<OutputKey>,
}

enum Effect {
    Frame(PlaybackFrame, bool),
    Event(PlaybackEventMessage),
    Flush(FlushCommand),
    Control(PlaybackControlMessage),
}

enum ControlAction {
    Pause {
        pause_id: String,
    },
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
    AbortAgent {
        utterance_id: String,
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
                validate_uuid(&message.pause_id, "pause_id")?;
                Ok(Self::Pause {
                    pause_id: message.pause_id,
                })
            }
            value if value == PlaybackControlMessage::RESUME as u8 => {
                if message.release_hold != PlaybackControlMessage::NONE as u8
                    || message.minimum_agent_epoch != 0
                    || !message.utterance_id.is_empty()
                    || !message.pause_id.is_empty()
                {
                    return Err("resume contains fields that must be empty".to_owned());
                }
                Ok(Self::Resume)
            }
            value if value == PlaybackControlMessage::DISCARD as u8 => {
                if !message.pause_id.is_empty() {
                    return Err("discard must not contain pause_id".to_owned());
                }
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
                    || !message.pause_id.is_empty()
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
            value if value == PlaybackControlMessage::ABORT_AGENT as u8 => {
                if message.utterance_id.is_empty()
                    || message.minimum_agent_epoch != 0
                    || message.release_hold != PlaybackControlMessage::NONE as u8
                    || !message.pause_id.is_empty()
                    || agent_epoch(&message.utterance_id).is_err()
                {
                    return Err("agent_abort fields are invalid".to_owned());
                }
                Ok(Self::AbortAgent {
                    utterance_id: message.utterance_id,
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
    maximum_buffered_playback_bytes: usize,
    speech_streams: HashMap<String, SpeechStream>,
    terminal_speech_ids: HashSet<String>,
    terminal_speech_order: VecDeque<String>,
    assemblies: HashMap<(Kind, String), Assembly>,
    minimum_agent_epoch: u64,
    output_pending: HashMap<OutputKey, PendingOutput>,
    last_output_progress_at: Option<Instant>,
    suppressed_output_acks: HashSet<OutputKey>,
    device_buffer: DeviceBufferTracker,
    flush_request: Option<FlushState>,
    deferred_pause_controls: VecDeque<ControlAction>,
    playout_generations: PlayoutGenerations,
    chunk_period: Duration,
    next_send_deadline: Option<Instant>,
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
        let maximum_buffered_playback_bytes = usize::try_from(
            u64::from(output_rate_hz)
                .checked_mul(u64::from(output_channels))
                .and_then(|value| value.checked_mul(2))
                .and_then(|value| value.checked_mul(MAX_BUFFERED_PLAYBACK_SECONDS))
                .ok_or_else(|| "maximum buffered playback byte count overflowed".to_owned())?,
        )
        .map_err(|_| "maximum buffered playback byte count exceeds usize".to_owned())?;
        Ok(Self {
            machine: PlaybackMachine::new(chunk_frames)?,
            converter: PcmConverter::new(output_rate_hz, output_channels)?,
            maximum_buffered_playback_bytes,
            speech_streams: HashMap::new(),
            terminal_speech_ids: HashSet::new(),
            terminal_speech_order: VecDeque::new(),
            assemblies: HashMap::new(),
            minimum_agent_epoch: 0,
            output_pending: HashMap::new(),
            last_output_progress_at: None,
            suppressed_output_acks: HashSet::new(),
            device_buffer: DeviceBufferTracker::default(),
            flush_request: None,
            deferred_pause_controls: VecDeque::new(),
            playout_generations: PlayoutGenerations::seeded(playout_generation_seed(
                SystemTime::now(),
            )?),
            chunk_period: Duration::from_millis(u64::from(chunk_ms)),
            next_send_deadline: None,
        })
    }

    fn ensure_playback_capacity(
        &self,
        additional_bytes: usize,
        adds_utterance: bool,
    ) -> Result<(), String> {
        if adds_utterance && self.machine.utterance_count() >= MAX_BUFFERED_UTTERANCES {
            return Err(format!(
                "playback backlog reached its {MAX_BUFFERED_UTTERANCES}-utterance limit"
            ));
        }
        let retained = self.machine.retained_pcm_bytes()?;
        let next = retained
            .checked_add(additional_bytes)
            .ok_or_else(|| "playback backlog byte count overflowed".to_owned())?;
        if next > self.maximum_buffered_playback_bytes {
            return Err(format!(
                "playback backlog would exceed its {}-byte limit",
                self.maximum_buffered_playback_bytes
            ));
        }
        Ok(())
    }

    fn cue_assembly_bytes(&self) -> Result<usize, String> {
        self.assemblies
            .values()
            .try_fold(0_usize, |total, assembly| {
                total
                    .checked_add(assembly.data.len())
                    .ok_or_else(|| "cue assembly byte count overflowed".to_owned())
            })
    }

    fn remember_terminal_speech(&mut self, utterance_id: String) {
        if self.terminal_speech_ids.contains(&utterance_id) {
            return;
        }
        if self.terminal_speech_order.len() >= MAX_TERMINAL_SPEECH_IDS {
            if let Some(oldest) = self.terminal_speech_order.pop_front() {
                self.terminal_speech_ids.remove(&oldest);
            }
        }
        self.terminal_speech_ids.insert(utterance_id.clone());
        self.terminal_speech_order.push_back(utterance_id);
    }

    fn on_speech_chunk(
        &mut self,
        kind: Kind,
        message: SynthesizedSpeechChunkMessage,
    ) -> Result<Vec<Effect>, String> {
        if kind == Kind::Cue {
            return Err("SynthesizedSpeechChunk cannot carry cue audio".to_owned());
        }
        let expected_kind = match kind {
            Kind::Agent => SynthesizedSpeechChunkMessage::AGENT as u8,
            Kind::System => SynthesizedSpeechChunkMessage::SYSTEM as u8,
            Kind::Cue => unreachable!("checked above"),
        };
        if message.kind != expected_kind {
            return Err("SynthesizedSpeechChunk kind does not match its topic".to_owned());
        }
        if message.utterance_id.is_empty() {
            return Err("SynthesizedSpeechChunk utterance_id must not be empty".to_owned());
        }
        validate_uuid(&message.request_id, "request_id")?;
        validate_uuid(&message.generation_id, "generation_id")?;
        if kind == Kind::Agent && agent_epoch(&message.utterance_id)? < self.minimum_agent_epoch {
            eprintln!(
                "dropping stale agent TTS stream {:?}; minimum floor is {}",
                message.utterance_id, self.minimum_agent_epoch
            );
            return Ok(Vec::new());
        }
        if kind == Kind::System && self.machine.system_is_aborted(&message.utterance_id) {
            eprintln!(
                "dropping PCM for aborted SYSTEM utterance {:?}",
                message.utterance_id
            );
            return Ok(Vec::new());
        }
        if self.terminal_speech_ids.contains(&message.utterance_id) {
            return Err("synthesized speech arrived after its terminal event".to_owned());
        }

        match message.event {
            value if value == SynthesizedSpeechChunkMessage::AUDIO as u8 => {
                self.accept_speech_audio(kind, message)
            }
            value if value == SynthesizedSpeechChunkMessage::COMPLETE as u8 => {
                self.complete_speech(kind, message)
            }
            value if value == SynthesizedSpeechChunkMessage::ABORT as u8 => {
                self.abort_speech_message(kind, message)
            }
            _ => Err("SynthesizedSpeechChunk event enum is invalid".to_owned()),
        }
    }

    fn accept_speech_audio(
        &mut self,
        kind: Kind,
        message: SynthesizedSpeechChunkMessage,
    ) -> Result<Vec<Effect>, String> {
        if let Some(stream) = self.speech_streams.get(&message.utterance_id) {
            if stream.kind != kind {
                return Err("utterance_id changed speech kind across topics".to_owned());
            }
            let (frame_count, alignments) =
                stream.validate_audio(&message, self.converter.output_rate_hz)?;
            if frame_count == 0 {
                self.speech_streams
                    .get_mut(&message.utterance_id)
                    .expect("stream existence was checked above")
                    .accept_audio(&message, 0, &[], alignments)?;
                return Ok(Vec::new());
            }
            let output = {
                let stream = self
                    .speech_streams
                    .get_mut(&message.utterance_id)
                    .expect("stream existence was checked above");
                stream
                    .converter
                    .push(message.first_sample_index, &message.pcm_f32)?
            };
            self.ensure_playback_capacity(output.len(), false)?;
            self.speech_streams
                .get_mut(&message.utterance_id)
                .expect("stream existence was checked above")
                .accept_audio(&message, frame_count, &output, alignments)?;
            return Ok(Vec::new());
        }

        if message.sequence != 0 || message.first_sample_index != 0 {
            return Err("synthesized speech must begin at sequence/sample zero".to_owned());
        }
        if message.sample_rate_hz != 22_050 || message.channels != 1 {
            return Err("MagpieTTS-RT speech must be 22050 Hz mono float32".to_owned());
        }
        validate_float_pcm(&message.pcm_f32, message.channels)?;
        let (total_source_characters, total_prepared_tokens) =
            validate_source_progress(&message.source_text, &message.source_progress)?;
        let frame_count = u64::try_from(message.pcm_f32.len())
            .map_err(|_| "synthesized speech frame count exceeds uint64".to_owned())?;
        validate_magpie_audio_shape(message.sequence, frame_count, message.final_chunk)?;
        let alignments = validate_alignment_events(
            &message.alignment_events,
            message.committed_text_tokens,
            AlignmentValidation {
                previous_committed_text_tokens: 0,
                previous_sample_index: 0,
                previous_alignment_tokens: 0,
                total_prepared_tokens,
                payload_first_sample: 0,
                available_input_samples: frame_count,
                source_progress: &message.source_progress,
                input_rate_hz: message.sample_rate_hz,
                output_rate_hz: self.converter.output_rate_hz,
            },
        )?;

        let converter = self
            .converter
            .start_speech(message.sample_rate_hz, message.channels)?;
        self.ensure_playback_capacity(0, true)?;
        let utterance = Utterance::new(
            PlaybackIdentity {
                kind,
                request_id: message.request_id.clone(),
                generation_id: message.generation_id.clone(),
                utterance_id: message.utterance_id.clone(),
            },
            Vec::new(),
            PcmFormat::pcm16(
                self.converter.output_rate_hz,
                self.converter.output_channels,
            ),
            Vec::new(),
            total_source_characters,
            false,
        )?;
        let (item, events) = self.machine.enqueue_with_ref(utterance)?;
        let mut effects = Vec::new();
        if events
            .iter()
            .any(|event| event.name == EventName::AgentPaused)
        {
            if let Some(flush) = self.request_flush(true, Kind::Agent, None, None, None)? {
                effects.push(Effect::Flush(flush));
            }
        }
        effects.extend(self.publish_events(events, None, None)?);

        let stream = SpeechStream {
            kind,
            sample_rate_hz: message.sample_rate_hz,
            channels: message.channels,
            next_sequence: 0,
            next_sample_index: 0,
            total_prepared_tokens,
            committed_text_tokens: 0,
            last_alignment_sample: 0,
            last_alignment_tokens: 0,
            final_audio_seen: false,
            source_progress: message.source_progress.clone(),
            last_progress_at: Instant::now(),
            item,
            converter,
        };
        self.speech_streams
            .insert(message.utterance_id.clone(), stream);
        let output = {
            let stream = self
                .speech_streams
                .get_mut(&message.utterance_id)
                .expect("stream was inserted above");
            stream
                .converter
                .push(message.first_sample_index, &message.pcm_f32)?
        };
        self.ensure_playback_capacity(output.len(), false)?;
        self.speech_streams
            .get_mut(&message.utterance_id)
            .expect("stream was inserted above")
            .accept_audio(&message, frame_count, &output, alignments)?;
        Ok(effects)
    }

    fn complete_speech(
        &mut self,
        kind: Kind,
        message: SynthesizedSpeechChunkMessage,
    ) -> Result<Vec<Effect>, String> {
        let stream = self
            .speech_streams
            .get(&message.utterance_id)
            .ok_or_else(|| "speech COMPLETE arrived before any AUDIO".to_owned())?;
        if stream.kind != kind {
            return Err("utterance_id changed speech kind across topics".to_owned());
        }
        stream.validate_terminal(&message)?;
        if stream.committed_text_tokens != stream.total_prepared_tokens
            || stream.last_alignment_tokens != stream.total_prepared_tokens
        {
            return Err("speech COMPLETE arrived before all source tokens were aligned".to_owned());
        }
        if !stream.final_audio_seen {
            return Err("speech COMPLETE arrived before the final audio chunk".to_owned());
        }
        if stream.next_sequence < 2 {
            return Err("speech COMPLETE arrived before a post-initial codec chunk".to_owned());
        }
        let stream = self
            .speech_streams
            .remove(&message.utterance_id)
            .expect("stream existence was checked above");
        let tail = stream.converter.finish()?;
        self.ensure_playback_capacity(tail.len(), false)?;
        {
            let mut item = stream.item.borrow_mut();
            item.append_pcm(&tail)?;
            item.complete_conversion()?;
        }
        self.remember_terminal_speech(message.utterance_id);
        Ok(Vec::new())
    }

    fn abort_speech_message(
        &mut self,
        kind: Kind,
        message: SynthesizedSpeechChunkMessage,
    ) -> Result<Vec<Effect>, String> {
        if let Some(stream) = self.speech_streams.get(&message.utterance_id) {
            if stream.kind != kind {
                return Err("utterance_id changed speech kind across topics".to_owned());
            }
            stream.validate_terminal(&message)?;
            let stream = self
                .speech_streams
                .remove(&message.utterance_id)
                .expect("stream existence was checked above");
            stream.converter.abort()?;
        } else {
            if message.sequence != 0 || message.first_sample_index != 0 {
                return Err("speech ABORT without AUDIO must carry sequence/sample zero".to_owned());
            }
            if message.sample_rate_hz != 22_050 || message.channels != 1 {
                return Err("speech ABORT format must be 22050 Hz mono".to_owned());
            }
            validate_empty_terminal_payload(&message)?;
        }
        self.remember_terminal_speech(message.utterance_id.clone());
        let events = self.machine.abort_speech(kind, &message.utterance_id)?;
        let mut effects = Vec::new();
        if let Some(flush) =
            self.request_flush(false, kind, Some(&message.utterance_id), None, None)?
        {
            effects.push(Effect::Flush(flush));
        }
        let event_name = match kind {
            Kind::Agent => EventName::AgentAborted,
            Kind::System => EventName::SystemAborted,
            Kind::Cue => unreachable!("speech chunk cannot be cue"),
        };
        effects.extend(self.publish_events(
            events,
            Some(PlaybackEvent::terminal_from_identity(
                event_name,
                message.request_id,
                message.generation_id,
                message.utterance_id,
            )),
            None,
        )?);
        Ok(effects)
    }

    fn on_cue_frame(&mut self, message: AudioFrame) -> Result<Vec<Effect>, String> {
        let item = self.assemble_cue(message)?;
        let Some(item) = item else {
            return Ok(Vec::new());
        };
        let converted = self.converter.convert(
            &item.pcm,
            item.sample_rate_hz,
            item.channels,
            item.bit_depth,
            "PCM16LE",
        )?;
        self.ensure_playback_capacity(converted.len(), true)?;
        let ready = Utterance::new(
            PlaybackIdentity {
                kind: Kind::Cue,
                request_id: String::new(),
                generation_id: String::new(),
                utterance_id: item.utterance_id,
            },
            converted,
            PcmFormat::pcm16(
                self.converter.output_rate_hz,
                self.converter.output_channels,
            ),
            Vec::new(),
            0,
            true,
        )?;
        let events = self.machine.enqueue(ready)?;
        self.publish_events(events, None, None)
    }

    fn reject_speech(
        &mut self,
        received_kind: Kind,
        received_request_id: &str,
        received_generation_id: &str,
        utterance_id: &str,
    ) -> Result<Vec<Effect>, String> {
        if received_kind == Kind::Cue {
            return Err("cue cannot be rejected as speech".to_owned());
        }
        if utterance_id.is_empty() {
            return Err("rejected speech has no utterance_id".to_owned());
        }
        if self.terminal_speech_ids.contains(utterance_id) {
            return Ok(Vec::new());
        }

        // Once sequence zero has been accepted, the retained stream identity is
        // authoritative. A malformed later message may drift to the other topic
        // or carry different UUIDs. Cleaning up by those untrusted fields would
        // abort the wrong playback kind and leave the real partial utterance
        // queued forever. Before sequence zero there is no retained identity, so
        // the exact validated topic/message identity remains the only target.
        let (kind, request_id, generation_id) =
            if let Some(stream) = self.speech_streams.get(utterance_id) {
                let item = stream.item.borrow();
                (
                    stream.kind,
                    item.request_id.clone(),
                    item.generation_id.clone(),
                )
            } else {
                validate_uuid(received_request_id, "request_id")?;
                validate_uuid(received_generation_id, "generation_id")?;
                (
                    received_kind,
                    received_request_id.to_owned(),
                    received_generation_id.to_owned(),
                )
            };
        if kind == Kind::Agent {
            agent_epoch(utterance_id)?;
        }
        if let Some(stream) = self.speech_streams.remove(utterance_id) {
            stream.converter.abort()?;
        }
        self.remember_terminal_speech(utterance_id.to_owned());
        let events = self.machine.abort_speech(kind, utterance_id)?;
        let mut effects = Vec::new();
        if let Some(flush) = self.request_flush(false, kind, Some(utterance_id), None, None)? {
            effects.push(Effect::Flush(flush));
        }
        let event_name = match kind {
            Kind::Agent => EventName::AgentAborted,
            Kind::System => EventName::SystemAborted,
            Kind::Cue => unreachable!("cue cannot be rejected as speech"),
        };
        effects.extend(self.publish_events(
            events,
            Some(PlaybackEvent::terminal_from_identity(
                event_name,
                request_id,
                generation_id,
                utterance_id.to_owned(),
            )),
            None,
        )?);
        let cancel = match kind {
            Kind::Agent => PlaybackControlMessage {
                action: PlaybackControlMessage::ABORT_AGENT as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: utterance_id.to_owned(),
            },
            Kind::System => PlaybackControlMessage {
                action: PlaybackControlMessage::ABORT_SYSTEM as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: utterance_id.to_owned(),
            },
            Kind::Cue => unreachable!("cue cannot be rejected as speech"),
        };
        effects.push(Effect::Control(cancel));
        Ok(effects)
    }

    fn expire_stalled_speech(&mut self, now: Instant) -> Result<Vec<Effect>, String> {
        let expired = self
            .speech_streams
            .iter()
            .filter(|(_, stream)| {
                elapsed_exceeds(stream.last_progress_at, now, SPEECH_INPUT_PROGRESS_TIMEOUT)
            })
            .map(|(utterance_id, stream)| {
                let item = stream.item.borrow();
                (
                    stream.kind,
                    item.request_id.clone(),
                    item.generation_id.clone(),
                    utterance_id.clone(),
                )
            })
            .collect::<Vec<_>>();
        let mut effects = Vec::new();
        for (kind, request_id, generation_id, utterance_id) in expired {
            eprintln!(
                "rejecting stalled {kind:?} synthesized speech stream {utterance_id:?} after {:.3} seconds without input progress",
                SPEECH_INPUT_PROGRESS_TIMEOUT.as_secs_f64()
            );
            effects.extend(self.reject_speech(kind, &request_id, &generation_id, &utterance_id)?);
        }
        Ok(effects)
    }

    fn reject_cue_frame(&mut self, utterance_id: &str) {
        if !utterance_id.is_empty() {
            self.assemblies
                .remove(&(Kind::Cue, utterance_id.to_owned()));
        }
    }

    fn assemble_cue(&mut self, message: AudioFrame) -> Result<Option<Utterance>, String> {
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
        let key = (Kind::Cue, message.stream_id.clone());
        if !self.assemblies.contains_key(&key) {
            if message.seq != 0 {
                return Err("utterance must begin at seq=0".to_owned());
            }
            if self.assemblies.len() >= MAX_CUE_ASSEMBLIES {
                return Err(format!(
                    "cue assembly backlog reached its {MAX_CUE_ASSEMBLIES}-stream limit"
                ));
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
        let assembly = self.assemblies.get(&key).expect("inserted above");
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
        let next_assembly_bytes = self
            .cue_assembly_bytes()?
            .checked_add(message.data.len())
            .ok_or_else(|| "cue assembly byte count overflowed".to_owned())?;
        if next_assembly_bytes > MAX_CUE_ASSEMBLY_BYTES {
            return Err(format!(
                "cue assembly backlog would exceed its {MAX_CUE_ASSEMBLY_BYTES}-byte limit"
            ));
        }
        let assembly = self.assemblies.get_mut(&key).expect("inserted above");
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
            PlaybackIdentity {
                kind: Kind::Cue,
                request_id: String::new(),
                generation_id: String::new(),
                utterance_id: message.stream_id,
            },
            assembly.data,
            PcmFormat {
                sample_rate_hz: assembly.sample_rate_hz,
                channels: assembly.channels,
                bit_depth: assembly.bit_depth,
            },
            Vec::new(),
            0,
            true,
        )
        .map(Some)
    }

    fn on_control(&mut self, message: PlaybackControlMessage) -> Result<Vec<Effect>, String> {
        let control = ControlAction::from_message(message)?;
        if self
            .flush_request
            .as_ref()
            .is_some_and(|flush| flush.pause_id.is_some())
        {
            match &control {
                ControlAction::Pause { .. } => {
                    return Err(
                        "a second PAUSE arrived before the pending PAUSE committed".to_owned()
                    );
                }
                ControlAction::Resume
                | ControlAction::Discard { .. }
                | ControlAction::AbortAgent { .. }
                | ControlAction::AbortSystem { .. } => {
                    if self.deferred_pause_controls.len() >= MAX_DEFERRED_PAUSE_CONTROLS {
                        return Err(format!(
                            "pending PAUSE deferred-control FIFO reached its \
                             {MAX_DEFERRED_PAUSE_CONTROLS}-item limit"
                        ));
                    }
                    self.deferred_pause_controls.push_back(control);
                    return Ok(Vec::new());
                }
            }
        }
        self.apply_control(control)
    }

    fn apply_control(&mut self, control: ControlAction) -> Result<Vec<Effect>, String> {
        let (events, required_terminal_event, invalidated_kind, flush) = match control {
            ControlAction::Pause { pause_id } => {
                if self.flush_request.is_some() {
                    return Err(
                        "PAUSE cannot start while another physical flush is active".to_owned()
                    );
                }
                let mut events = self.machine.pause_agent();
                for event in &mut events {
                    if event.name == EventName::AgentPaused {
                        event.pause_id = pause_id.clone();
                    }
                }
                let flush =
                    self.request_flush(true, Kind::Agent, None, None, Some(pause_id.clone()))?;
                if flush.is_none() {
                    events.push(self.machine.pause_committed(pause_id));
                }
                (events, None, Some(Kind::Agent), flush)
            }
            ControlAction::Resume => (self.machine.resume_agent(), None, None, None),
            ControlAction::Discard {
                release_hold,
                minimum_agent_epoch,
                utterance_id,
            } => {
                self.minimum_agent_epoch = self.minimum_agent_epoch.max(minimum_agent_epoch);
                let mut stale_assemblies = Vec::new();
                for (utterance_id, stream) in &self.speech_streams {
                    if stream.kind == Kind::Agent
                        && agent_epoch(utterance_id)? < self.minimum_agent_epoch
                    {
                        stale_assemblies.push(utterance_id.clone());
                    }
                }
                for utterance_id in stale_assemblies {
                    if let Some(stream) = self.speech_streams.remove(&utterance_id) {
                        stream.converter.abort()?;
                    }
                    self.remember_terminal_speech(utterance_id);
                }
                let events = self.machine.discard_agent(
                    release_hold,
                    utterance_id.as_deref(),
                    Some(self.minimum_agent_epoch),
                )?;
                let flush = self.request_flush(
                    false,
                    Kind::Agent,
                    None,
                    Some(self.minimum_agent_epoch),
                    None,
                )?;
                (events, None, Some(Kind::Agent), flush)
            }
            ControlAction::AbortSystem {
                utterance_id,
                release_hold,
            } => {
                if self.terminal_speech_ids.contains(&utterance_id) {
                    return Ok(Vec::new());
                }
                if let Some(stream) = self.speech_streams.remove(&utterance_id) {
                    if stream.kind != Kind::System {
                        return Err(
                            "system abort references an agent synthesized speech stream".to_owned()
                        );
                    }
                    stream.converter.abort()?;
                }
                self.remember_terminal_speech(utterance_id.clone());
                let events = self.machine.abort_system(&utterance_id, release_hold)?;
                let flush =
                    self.request_flush(false, Kind::System, Some(&utterance_id), None, None)?;
                (events, None, None, flush)
            }
            ControlAction::AbortAgent { utterance_id } => {
                if self.terminal_speech_ids.contains(&utterance_id) {
                    return Ok(Vec::new());
                }
                if let Some(stream) = self.speech_streams.remove(&utterance_id) {
                    if stream.kind != Kind::Agent {
                        return Err(
                            "agent abort references a system synthesized speech stream".to_owned()
                        );
                    }
                    stream.converter.abort()?;
                }
                self.remember_terminal_speech(utterance_id.clone());
                let events = self.machine.abort_speech(Kind::Agent, &utterance_id)?;
                let flush =
                    self.request_flush(false, Kind::Agent, Some(&utterance_id), None, None)?;
                (events, None, None, flush)
            }
        };
        let mut effects = Vec::new();
        if let Some(flush) = flush {
            effects.push(Effect::Flush(flush));
        }
        effects.extend(self.publish_events(events, required_terminal_event, invalidated_kind)?);
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
        if ack.status == STATUS_FLUSHED && (ack.frame_count >= expected.frame_count || ack.is_final)
        {
            return Err("flushed ack does not describe a partial pending chunk".to_owned());
        }
        self.last_output_progress_at = Some(Instant::now());
        self.device_buffer
            .note_ack(&ack.utterance_id, ack.is_final, ack.status);
        self.output_pending.remove(&key);

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
        let mut effects = self.publish_events(events, None, None)?;
        effects.extend(self.maybe_finish_flush()?);
        Ok(effects)
    }

    fn pump(&mut self) -> Result<Vec<Effect>, String> {
        self.pump_at(Instant::now())
    }

    fn pump_at(&mut self, now: Instant) -> Result<Vec<Effect>, String> {
        if self.flush_request.is_some() {
            return Ok(Vec::new());
        }
        let initial_prefill = self.next_send_deadline.is_none();
        let initial_kind = if initial_prefill {
            self.machine.next_kind()
        } else {
            None
        };
        let maximum_send_limit = match self.next_send_deadline {
            None => initial_kind
                .map(initial_playout_chunks)
                .unwrap_or(TTS_INITIAL_PLAYOUT_CHUNKS),
            Some(deadline) if now >= deadline => TTS_INITIAL_PLAYOUT_CHUNKS,
            Some(_) => return Ok(Vec::new()),
        };
        let mut effects = Vec::new();
        for _ in 0..maximum_send_limit {
            if let Some(expected_kind) = initial_kind {
                if self
                    .machine
                    .next_kind()
                    .is_some_and(|kind| kind != expected_kind)
                {
                    break;
                }
            }
            let Some(chunk) = self.machine.next_chunk()? else {
                self.next_send_deadline = None;
                break;
            };
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
            self.output_pending.insert(
                key,
                PendingOutput {
                    expected,
                    kind: chunk.kind,
                    accepted: false,
                    last_progress_at: now,
                },
            );
            if self.last_output_progress_at.is_none() {
                self.last_output_progress_at = Some(now);
            }
            self.device_buffer
                .note_chunk(chunk.kind, &chunk.utterance_id);
            effects.push(Effect::Frame(
                PlaybackFrame {
                    kind: chunk.kind.code(),
                    request_id: chunk.request_id,
                    generation_id: chunk.generation_id,
                    utterance_id: chunk.utterance_id,
                    seq: chunk.seq,
                    sample_index: chunk.sample_index,
                    frame_count: chunk.frame_count,
                    sample_rate_hz: chunk.sample_rate_hz,
                    channels: chunk.channels,
                    pcm_s16le: chunk.data,
                    final_: chunk.is_final,
                    playout_generation: self.playout_generations.get(chunk.kind),
                },
                chunk.emit_first_playback_timing,
            ));
        }
        if !effects.is_empty() {
            if initial_prefill {
                let initial_send_limit = initial_kind
                    .map(initial_playout_chunks)
                    .ok_or_else(|| "initial playout kind was not recorded".to_owned())?;
                if effects.len() == initial_send_limit {
                    self.next_send_deadline = Some(
                        now.checked_add(self.chunk_period)
                            .ok_or_else(|| "playback pacing deadline overflowed".to_owned())?,
                    );
                }
            } else {
                let deadline = self
                    .next_send_deadline
                    .expect("a periodic send has a deadline");
                self.next_send_deadline =
                    Some(next_periodic_deadline(deadline, now, self.chunk_period)?);
            }
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
        pause_id: Option<String>,
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
        // A flush invalidates any queued timing credit. The next playable audio starts
        // with a fresh prefill instead of inheriting a stale pre-flush deadline.
        self.next_send_deadline = None;
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
            pause_id,
        });
        Ok(Some(FlushCommand { keys: ordered_keys }))
    }

    fn finish_flush_service(&mut self) -> Result<Vec<Effect>, String> {
        if let Some(request) = self.flush_request.as_mut() {
            self.device_buffer
                .clear_matching(request.target_kind, request.target_utterance_id.as_deref());
            request.service_done = true;
        }
        self.maybe_finish_flush()
    }

    fn maybe_finish_flush(&mut self) -> Result<Vec<Effect>, String> {
        if !self
            .flush_request
            .as_ref()
            .is_some_and(|request| request.service_done && request.keys.is_empty())
        {
            return Ok(Vec::new());
        }
        let completed = self.flush_request.take().expect("completion was checked");
        let mut effects = Vec::new();
        if let Some(pause_id) = completed.pause_id {
            let committed = self.machine.pause_committed(pause_id);
            effects.extend(self.publish_events(vec![committed], None, None)?);
        }
        while self.flush_request.is_none() {
            let Some(control) = self.deferred_pause_controls.pop_front() else {
                break;
            };
            effects.extend(self.apply_control(control)?);
        }
        Ok(effects)
    }

    fn publish_events(
        &mut self,
        events: Vec<PlaybackEvent>,
        required_terminal_event: Option<PlaybackEvent>,
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
        let required_terminal_missing = required_terminal_event
            .as_ref()
            .is_some_and(|required| !events.iter().any(|event| event.name == required.name));
        let mut effects = events
            .into_iter()
            .map(|event| Effect::Event(event.to_message(self.playout_generations.get(event.kind))))
            .collect::<Vec<_>>();
        if required_terminal_missing {
            let required = required_terminal_event.expect("checked above");
            effects.push(Effect::Event(
                required.to_message(self.playout_generations.get(required.kind)),
            ));
        }
        Ok(effects)
    }
}

fn process_speech_chunk(
    controller: &mut PlaybackController,
    kind: Kind,
    message: SynthesizedSpeechChunkMessage,
) -> Result<Vec<Effect>, String> {
    if kind == Kind::Cue {
        return Err("SynthesizedSpeechChunk cannot carry cue audio".to_owned());
    }
    let request_id = message.request_id.clone();
    let generation_id = message.generation_id.clone();
    let utterance_id = message.utterance_id.clone();
    if utterance_id.is_empty() {
        return Err("SynthesizedSpeechChunk utterance_id must not be empty".to_owned());
    }
    let retained_identity = controller.speech_streams.contains_key(&utterance_id);
    if !retained_identity {
        validate_uuid(&request_id, "request_id")?;
        validate_uuid(&generation_id, "generation_id")?;
    }
    let expected_kind = match kind {
        Kind::Agent => SynthesizedSpeechChunkMessage::AGENT as u8,
        Kind::System => SynthesizedSpeechChunkMessage::SYSTEM as u8,
        Kind::Cue => unreachable!("checked above"),
    };
    if message.kind != expected_kind && !retained_identity {
        return Err(
            "SynthesizedSpeechChunk kind does not match its topic and has no retained stream identity"
                .to_owned(),
        );
    }
    if kind == Kind::Agent && !retained_identity {
        agent_epoch(&utterance_id)?;
    }
    match controller.on_speech_chunk(kind, message) {
        Ok(effects) => Ok(effects),
        Err(error) => {
            eprintln!("rejecting {kind:?} synthesized speech stream: {error}");
            let effects = controller
                .reject_speech(kind, &request_id, &generation_id, &utterance_id)
                .map_err(|cleanup_error| {
                    format!(
                    "cannot contain rejected {kind:?} utterance {utterance_id:?}: {cleanup_error}"
                )
                })?;
            Ok(effects)
        }
    }
}

fn process_cue_frame(
    controller: &mut PlaybackController,
    message: AudioFrame,
) -> Result<Vec<Effect>, String> {
    let utterance_id = message.stream_id.clone();
    match controller.on_cue_frame(message) {
        Ok(effects) => Ok(effects),
        Err(error) => {
            eprintln!("rejecting CUE audio frame: {error}");
            controller.reject_cue_frame(&utterance_id);
            Ok(Vec::new())
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

fn validate_uuid(value: &str, field: &str) -> Result<(), String> {
    let bytes = value.as_bytes();
    if bytes.len() != 36
        || !matches!(bytes.get(8), Some(b'-'))
        || !matches!(bytes.get(13), Some(b'-'))
        || !matches!(bytes.get(18), Some(b'-'))
        || !matches!(bytes.get(23), Some(b'-'))
        || bytes.iter().enumerate().any(|(index, byte)| {
            !matches!(index, 8 | 13 | 18 | 23) && !matches!(byte, b'0'..=b'9' | b'a'..=b'f')
        })
    {
        return Err(format!("{field} must be a canonical lowercase UUID"));
    }
    Ok(())
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

fn monotonic_time_ns() -> Result<u64, String> {
    let mut timestamp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: timestamp points to initialized writable storage and
    // CLOCK_MONOTONIC is process-independent on one Linux host.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut timestamp) } != 0 {
        return Err(format!(
            "CLOCK_MONOTONIC failed: {}",
            std::io::Error::last_os_error()
        ));
    }
    if timestamp.tv_sec < 0 || !(0..1_000_000_000).contains(&timestamp.tv_nsec) {
        return Err("CLOCK_MONOTONIC returned an invalid timespec".to_owned());
    }
    let seconds = u64::try_from(timestamp.tv_sec)
        .map_err(|_| "CLOCK_MONOTONIC seconds exceed uint64".to_owned())?;
    let nanoseconds = u64::try_from(timestamp.tv_nsec)
        .map_err(|_| "CLOCK_MONOTONIC nanoseconds exceed uint64".to_owned())?;
    seconds
        .checked_mul(1_000_000_000)
        .and_then(|value| value.checked_add(nanoseconds))
        .ok_or_else(|| "CLOCK_MONOTONIC nanoseconds overflow uint64".to_owned())
}

fn next_periodic_deadline(
    previous_deadline: Instant,
    now: Instant,
    period: Duration,
) -> Result<Instant, String> {
    if period.is_zero() {
        return Err("playback pacing period must be positive".to_owned());
    }
    let elapsed_periods =
        now.saturating_duration_since(previous_deadline).as_nanos() / period.as_nanos();
    let periods_to_advance = elapsed_periods
        .checked_add(1)
        .ok_or_else(|| "playback pacing period count overflowed".to_owned())?;
    let periods_to_advance = u32::try_from(periods_to_advance)
        .map_err(|_| "playback pacing delay exceeds the supported range".to_owned())?;
    let advance = period
        .checked_mul(periods_to_advance)
        .ok_or_else(|| "playback pacing duration overflowed".to_owned())?;
    previous_deadline
        .checked_add(advance)
        .ok_or_else(|| "playback pacing deadline overflowed".to_owned())
}

fn initial_playout_chunks(kind: Kind) -> usize {
    match kind {
        Kind::Agent | Kind::System => TTS_INITIAL_PLAYOUT_CHUNKS,
        Kind::Cue => CUE_INITIAL_PLAYOUT_CHUNKS,
    }
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
        node.subscribe::<SynthesizedSpeechChunkMessage>("/audio/agent/frame", control_qos.clone())?;
    let mut system_frames = node
        .subscribe::<SynthesizedSpeechChunkMessage>("/audio/system/frame", control_qos.clone())?;
    let mut cue_frames = node.subscribe::<AudioFrame>("/audio/cue/frame", control_qos.clone())?;
    let mut controls =
        node.subscribe::<PlaybackControlMessage>("/audio/playback/control", control_qos.clone())?;
    let mut drained =
        node.subscribe::<OutputDrainedMessage>("/audio/output/drained", control_qos.clone())?;
    let output = node.create_publisher::<PlaybackFrame>("/audio/play", playout_qos.clone())?;
    let events = node
        .create_publisher::<PlaybackEventMessage>("/audio/playback/event", control_qos.clone())?;
    let timing_events =
        node.create_publisher::<TtsTimingEvent>("/aspa/tts/timing", playout_qos.clone())?;
    let control_feedback = node.create_publisher::<PlaybackControlMessage>(
        "/audio/playback/control",
        control_qos.clone(),
    )?;
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
        let mut pending_effects = Vec::new();
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
                pending_effects.extend(
                    controller
                        .finish_flush_service()
                        .map_err(ControllerError::Invariant)?,
                );
            }
        }

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
                process_speech_chunk(&mut controller, Kind::System, message)
                    .map_err(ControllerError::Invariant)?,
            );
        }
        for message in take_ready_bounded(&mut cue_frames, CUE_MESSAGES_PER_TICK) {
            pending_effects.extend(
                process_cue_frame(&mut controller, message).map_err(ControllerError::Invariant)?,
            );
        }
        for message in take_ready_bounded(&mut agent_frames, AGENT_MESSAGES_PER_TICK) {
            pending_effects.extend(
                process_speech_chunk(&mut controller, Kind::Agent, message)
                    .map_err(ControllerError::Output)?,
            );
        }
        pending_effects.extend(
            controller
                .expire_stalled_speech(Instant::now())
                .map_err(ControllerError::Invariant)?,
        );
        pending_effects.extend(controller.pump().map_err(ControllerError::Invariant)?);

        for effect in pending_effects {
            match effect {
                Effect::Frame(message, publish_first_tts_frame) => {
                    // Capture before publishing the frame. audio_output can consume
                    // it on another process before Publisher::publish returns.
                    let first_tts_frame_time_ns = if publish_first_tts_frame {
                        Some(monotonic_time_ns().map_err(ControllerError::Invariant)?)
                    } else {
                        None
                    };
                    output.publish(&message)?;
                    if let Some(first_tts_frame_time_ns) = first_tts_frame_time_ns {
                        timing_events.publish(&TtsTimingEvent {
                            version: TtsTimingEvent::TTS_TIMING_SCHEMA_VERSION as u32,
                            stage: TtsTimingEvent::FIRST_PLAYBACK_FRAME_PUBLISHED as u8,
                            kind: message.kind,
                            request_id: message.request_id.clone(),
                            generation_id: message.generation_id.clone(),
                            utterance_id: message.utterance_id.clone(),
                            monotonic_time_ns: first_tts_frame_time_ns,
                            underrun_frames: 0,
                        })?;
                    }
                }
                Effect::Event(message) => events.publish(&message)?,
                Effect::Control(message) => control_feedback.publish(&message)?,
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

    fn source_alignments(frames: usize) -> Vec<SourceAlignment> {
        vec![SourceAlignment {
            source_end: 1,
            end_frame: frames as u64,
        }]
    }

    fn utterance(kind: Kind, name: &str, frames: usize) -> Utterance {
        let alignments = if kind == Kind::Cue {
            Vec::new()
        } else {
            source_alignments(frames)
        };
        let (request_id, generation_id) = if kind == Kind::Cue {
            (String::new(), String::new())
        } else {
            (
                "00000000-0000-4000-8000-000000000001".to_owned(),
                "00000000-0000-4000-8000-000000000002".to_owned(),
            )
        };
        Utterance::new(
            PlaybackIdentity {
                kind,
                request_id,
                generation_id,
                utterance_id: name.to_owned(),
            },
            vec![0; frames * 2],
            PcmFormat::pcm16(16_000, 1),
            alignments,
            u64::from(kind != Kind::Cue),
            true,
        )
        .unwrap()
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
    fn playback_backlog_accounting_counts_each_shared_utterance_once() {
        let mut machine = PlaybackMachine::new(2).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-a", 8))
            .unwrap();
        machine
            .enqueue(utterance(Kind::System, "system-a", 4))
            .unwrap();
        assert_eq!(machine.utterance_count(), 2);
        assert_eq!(machine.retained_pcm_bytes().unwrap(), 24);

        let chunk = machine.next_chunk().unwrap().unwrap();
        assert_eq!(chunk.utterance_id, "system-a");
        assert_eq!(machine.utterance_count(), 2);
        assert_eq!(machine.retained_pcm_bytes().unwrap(), 24);
    }

    #[test]
    fn playback_backlog_limits_fail_closed() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller.maximum_buffered_playback_bytes = 3;
        controller
            .machine
            .enqueue(utterance(Kind::Cue, "cue-a", 2))
            .unwrap();
        assert!(controller.ensure_playback_capacity(0, false).is_err());

        controller.maximum_buffered_playback_bytes = usize::MAX;
        for index in 1..MAX_BUFFERED_UTTERANCES {
            controller
                .machine
                .enqueue(utterance(Kind::Cue, &format!("cue-{index}"), 1))
                .unwrap();
        }
        assert_eq!(
            controller.machine.utterance_count(),
            MAX_BUFFERED_UTTERANCES
        );
        assert!(controller.ensure_playback_capacity(0, true).is_err());
    }

    #[test]
    fn agent_event_reports_only_fully_played_source_spans() {
        let alignments = vec![
            SourceAlignment {
                source_end: 2,
                end_frame: 4,
            },
            SourceAlignment {
                source_end: 3,
                end_frame: 7,
            },
        ];
        let item = Rc::new(RefCell::new(
            Utterance::new(
                PlaybackIdentity {
                    kind: Kind::Agent,
                    request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
                    generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
                    utterance_id: "agent-0-aligned".to_owned(),
                },
                vec![0; 8 * 2],
                PcmFormat::pcm16(16_000, 1),
                alignments,
                3,
                true,
            )
            .unwrap(),
        ));
        item.borrow_mut().played_offset_frames = 3;
        let first = PlaybackMachine::event(EventName::AgentPaused, &item);
        assert_eq!(first.played_source_characters, 0);
        assert_eq!(first.total_source_characters, 3);
        item.borrow_mut().played_offset_frames = 4;
        let second = PlaybackMachine::event(EventName::AgentPaused, &item);
        assert_eq!(second.played_source_characters, 2);
        assert!(second.source_position_valid);
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
        assert!(events.is_empty());
        machine.resume_agent();
        assert_eq!(machine.next_chunk().unwrap().unwrap().sample_index, 2);
    }

    #[test]
    fn replayed_sequence_zero_does_not_repeat_first_playback_timing() {
        let mut machine = PlaybackMachine::new(4).unwrap();
        machine
            .enqueue(utterance(Kind::Agent, "agent-0-timing", 12))
            .unwrap();
        let first = machine.next_chunk().unwrap().unwrap();
        assert_eq!(first.seq, 0);
        assert!(first.emit_first_playback_timing);

        machine.pause_agent();
        machine
            .output_drained("agent-0-timing", 0, 0, false, STATUS_FLUSHED)
            .unwrap();
        machine.resume_agent();
        let replay = machine.next_chunk().unwrap().unwrap();
        assert_eq!(replay.seq, 0);
        assert_eq!(replay.sample_index, 0);
        assert!(!replay.emit_first_playback_timing);
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
    fn empty_control_transitions_advance_only_their_scoped_generation_without_fake_events() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller.playout_generations = PlayoutGenerations::seeded(0);

        let aborted = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::ABORT_SYSTEM as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: "system-queued".to_owned(),
            })
            .unwrap();
        assert!(aborted.is_empty());
        assert_eq!(controller.playout_generations.get(Kind::System), 0);

        let paused = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::PAUSE as u8,
                pause_id: "00000000-0000-4000-8000-000000000003".to_owned(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: String::new(),
            })
            .unwrap();
        let paused_events = paused
            .into_iter()
            .filter_map(|effect| match effect {
                Effect::Event(event) => Some(event),
                Effect::Frame(_, _) | Effect::Flush(_) | Effect::Control(_) => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(paused_events.len(), 1);
        assert_eq!(
            paused_events[0].event,
            PlaybackEventMessage::AGENT_PAUSE_COMMITTED as u8
        );
        assert_eq!(
            paused_events[0].pause_id,
            "00000000-0000-4000-8000-000000000003"
        );
        assert!(paused_events[0].request_id.is_empty());
        assert!(paused_events[0].generation_id.is_empty());
        assert!(paused_events[0].utterance_id.is_empty());
        assert_eq!(paused_events[0].played_frames, 0);
        assert!(!paused_events[0].total_frames_valid);
        assert!(!paused_events[0].source_position_valid);
        assert_eq!(controller.playout_generations.get(Kind::Agent), 1);
        assert_eq!(controller.playout_generations.get(Kind::System), 0);
        assert_eq!(controller.playout_generations.get(Kind::Cue), 0);
    }

    #[test]
    fn control_contract_rejects_unknown_and_cross_action_fields() {
        let mut message = PlaybackControlMessage {
            action: 0,
            pause_id: String::new(),
            release_hold: 0,
            minimum_agent_epoch: 0,
            utterance_id: String::new(),
        };
        assert!(ControlAction::from_message(message.clone()).is_err());
        message.action = PlaybackControlMessage::PAUSE as u8;
        message.utterance_id = "not-allowed".to_owned();
        assert!(ControlAction::from_message(message).is_err());

        let mut invalid_pause_id = control(
            PlaybackControlMessage::PAUSE as u8,
            PlaybackControlMessage::NONE as u8,
            "",
        );
        invalid_pause_id.pause_id = "00000000-0000-4000-8000-00000000000A".to_owned();
        assert!(ControlAction::from_message(invalid_pause_id).is_err());

        let mut resume_with_pause_id = control(
            PlaybackControlMessage::RESUME as u8,
            PlaybackControlMessage::NONE as u8,
            "",
        );
        resume_with_pause_id.pause_id = "00000000-0000-4000-8000-000000000003".to_owned();
        assert!(ControlAction::from_message(resume_with_pause_id).is_err());
    }

    #[test]
    fn protocol_abort_feedback_is_idempotent_when_playback_receives_its_own_control() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller.remember_terminal_speech("agent-4-rejected".to_owned());

        let effects = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::ABORT_AGENT as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: "agent-4-rejected".to_owned(),
            })
            .expect("looped-back targeted abort");

        assert!(effects.is_empty());
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

    fn input_speech(
        kind: Kind,
        utterance_id: &str,
        frame_count: usize,
    ) -> SynthesizedSpeechChunkMessage {
        let kind_code = match kind {
            Kind::Agent => SynthesizedSpeechChunkMessage::AGENT as u8,
            Kind::System => SynthesizedSpeechChunkMessage::SYSTEM as u8,
            Kind::Cue => panic!("cue uses AudioFrame"),
        };
        SynthesizedSpeechChunkMessage {
            kind: kind_code,
            event: SynthesizedSpeechChunkMessage::AUDIO as u8,
            request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
            generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
            utterance_id: utterance_id.to_owned(),
            sequence: 0,
            first_sample_index: 0,
            sample_rate_hz: 22_050,
            channels: 1,
            pcm_f32: vec![0.0; frame_count],
            final_chunk: false,
            committed_text_tokens: 1,
            alignment_events: vec![TtsAlignmentEventMessage {
                sample_index: frame_count as u64,
                committed_text_tokens: 1,
            }],
            source_text: "あ".to_owned(),
            source_progress: vec![
                TtsSourceProgressMessage {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                TtsSourceProgressMessage {
                    committed_text_tokens: 1,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
            ],
        }
    }

    fn terminal_speech(
        kind: Kind,
        utterance_id: &str,
        event: u8,
        sequence: u64,
        sample_index: u64,
    ) -> SynthesizedSpeechChunkMessage {
        let kind_code = match kind {
            Kind::Agent => SynthesizedSpeechChunkMessage::AGENT as u8,
            Kind::System => SynthesizedSpeechChunkMessage::SYSTEM as u8,
            Kind::Cue => panic!("cue uses AudioFrame"),
        };
        SynthesizedSpeechChunkMessage {
            kind: kind_code,
            event,
            request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
            generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
            utterance_id: utterance_id.to_owned(),
            sequence,
            first_sample_index: sample_index,
            sample_rate_hz: 22_050,
            channels: 1,
            pcm_f32: Vec::new(),
            final_chunk: false,
            committed_text_tokens: 0,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        }
    }

    fn first_two_token_speech(kind: Kind, utterance_id: &str) -> SynthesizedSpeechChunkMessage {
        let mut message = input_speech(kind, utterance_id, 4_096);
        message.source_text = "あい".to_owned();
        message.source_progress.push(TtsSourceProgressMessage {
            committed_text_tokens: 2,
            source_char_end: 2,
            source_utf8_end: 6,
        });
        message
    }

    fn second_two_token_speech(kind: Kind, utterance_id: &str) -> SynthesizedSpeechChunkMessage {
        let mut message = input_speech(kind, utterance_id, 1_024);
        message.sequence = 1;
        message.first_sample_index = 4_096;
        message.final_chunk = true;
        message.committed_text_tokens = 2;
        message.alignment_events = vec![TtsAlignmentEventMessage {
            sample_index: 5_120,
            committed_text_tokens: 2,
        }];
        message.source_text.clear();
        message.source_progress.clear();
        message
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
            pause_id: if action == PlaybackControlMessage::PAUSE as u8 {
                "00000000-0000-4000-8000-000000000003".to_owned()
            } else {
                String::new()
            },
            release_hold,
            minimum_agent_epoch: 0,
            utterance_id: utterance_id.to_owned(),
        }
    }

    fn frame_effect(effects: Vec<Effect>) -> PlaybackFrame {
        frame_effects(effects)
            .into_iter()
            .next()
            .expect("a playback frame effect is required")
    }

    fn frame_effects(effects: Vec<Effect>) -> Vec<PlaybackFrame> {
        effects
            .into_iter()
            .filter_map(|effect| match effect {
                Effect::Frame(frame, _) => Some(frame),
                Effect::Event(_) | Effect::Flush(_) | Effect::Control(_) => None,
            })
            .collect()
    }

    fn event_effects(effects: Vec<Effect>) -> Vec<PlaybackEventMessage> {
        effects
            .into_iter()
            .filter_map(|effect| match effect {
                Effect::Event(event) => Some(event),
                Effect::Frame(_, _) | Effect::Flush(_) | Effect::Control(_) => None,
            })
            .collect()
    }

    #[test]
    fn malformed_cue_input_is_contained_and_a_stream_id_can_restart_at_seq_zero() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        assert!(
            process_cue_frame(&mut controller, input_frame("cue-bad", 0, 2, false))
                .unwrap()
                .is_empty()
        );
        assert!(controller
            .assemblies
            .contains_key(&(Kind::Cue, "cue-bad".to_owned())));

        let mut invalid = input_frame("cue-bad", 1, 1, true);
        invalid.bit_depth = 8;
        assert!(process_cue_frame(&mut controller, invalid)
            .unwrap()
            .is_empty());
        assert!(!controller
            .assemblies
            .contains_key(&(Kind::Cue, "cue-bad".to_owned())));
        assert!(controller.pump().unwrap().is_empty());

        let restarted = controller
            .assemble_cue(input_frame("cue-bad", 0, 16, true))
            .unwrap()
            .expect("the restarted utterance must assemble");
        assert_eq!(restarted.total_frames(), 16);
    }

    #[test]
    fn cue_assembly_memory_limit_rejects_and_releases_the_stream() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        let maximum_frames =
            u32::try_from(MAX_CUE_ASSEMBLY_BYTES / 2).expect("test limit fits uint32");
        assert!(process_cue_frame(
            &mut controller,
            input_frame("cue-bounded", 0, maximum_frames, false),
        )
        .unwrap()
        .is_empty());
        assert_eq!(
            controller.cue_assembly_bytes().unwrap(),
            MAX_CUE_ASSEMBLY_BYTES
        );

        assert!(
            process_cue_frame(&mut controller, input_frame("cue-bounded", 1, 1, false),)
                .unwrap()
                .is_empty()
        );
        assert!(!controller
            .assemblies
            .contains_key(&(Kind::Cue, "cue-bounded".to_owned())));
        assert_eq!(controller.cue_assembly_bytes().unwrap(), 0);
    }

    #[test]
    fn zero_frame_input_never_reaches_the_output_boundary() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        let effects = process_cue_frame(&mut controller, input_frame("cue-empty", 0, 0, true));
        assert!(effects.unwrap().is_empty());
        assert!(controller.pump().unwrap().is_empty());
        assert!(Utterance::new(
            PlaybackIdentity {
                kind: Kind::Cue,
                request_id: String::new(),
                generation_id: String::new(),
                utterance_id: "cue-empty".to_owned(),
            },
            Vec::new(),
            PcmFormat::pcm16(16_000, 1),
            Vec::new(),
            0,
            true,
        )
        .is_err());
    }

    #[test]
    fn malformed_system_speech_is_tombstoned_without_leaking_a_hold() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        let mut invalid = input_speech(Kind::System, "system-bad", 2);
        invalid.sample_rate_hz = 16_000;
        let effects = process_speech_chunk(&mut controller, Kind::System, invalid).unwrap();

        assert!(!controller.machine.agent_pause_requested());
        assert!(controller.machine.system_is_aborted("system-bad"));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::SYSTEM_ABORTED as u8
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_SYSTEM as u8
                    && message.release_hold == PlaybackControlMessage::NONE as u8
                    && message.minimum_agent_epoch == 0
                    && message.utterance_id == "system-bad"
        )));
    }

    #[test]
    fn malformed_tts_identity_is_a_fatal_contract_error_before_cleanup() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        let mut invalid = input_speech(Kind::System, "system-bad-identity", 4 * 1_024);
        invalid.request_id.clear();

        assert!(process_speech_chunk(&mut controller, Kind::System, invalid).is_err());
        assert!(!controller.machine.agent_pause_requested());
        assert!(!controller
            .terminal_speech_ids
            .contains("system-bad-identity"));
    }

    #[test]
    fn cross_topic_drift_aborts_the_retained_agent_identity() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let utterance_id = "agent-0-cross-topic";
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, utterance_id),
        )
        .unwrap();

        let mut drifted = second_two_token_speech(Kind::System, utterance_id);
        drifted.request_id = "00000000-0000-4000-8000-000000000011".to_owned();
        drifted.generation_id = "00000000-0000-4000-8000-000000000012".to_owned();
        let effects = process_speech_chunk(&mut controller, Kind::System, drifted).unwrap();

        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert_eq!(controller.machine.utterance_count(), 0);
        assert!(controller.terminal_speech_ids.contains(utterance_id));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.kind == PlaybackEventMessage::AGENT as u8
                    && message.request_id == "00000000-0000-4000-8000-000000000001"
                    && message.generation_id == "00000000-0000-4000-8000-000000000002"
                    && message.utterance_id == utterance_id
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
                    && message.utterance_id == utterance_id
        )));
        assert!(!effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_SYSTEM as u8
        )));
    }

    #[test]
    fn cross_topic_drift_aborts_the_retained_system_identity() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let utterance_id = "system-cross-topic";
        process_speech_chunk(
            &mut controller,
            Kind::System,
            first_two_token_speech(Kind::System, utterance_id),
        )
        .unwrap();

        let mut drifted = second_two_token_speech(Kind::Agent, utterance_id);
        drifted.request_id = "00000000-0000-4000-8000-000000000011".to_owned();
        drifted.generation_id = "00000000-0000-4000-8000-000000000012".to_owned();
        let effects = process_speech_chunk(&mut controller, Kind::Agent, drifted).unwrap();

        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert_eq!(controller.machine.utterance_count(), 0);
        assert!(controller.terminal_speech_ids.contains(utterance_id));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::SYSTEM_ABORTED as u8
                    && message.kind == PlaybackEventMessage::SYSTEM as u8
                    && message.request_id == "00000000-0000-4000-8000-000000000001"
                    && message.generation_id == "00000000-0000-4000-8000-000000000002"
                    && message.utterance_id == utterance_id
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_SYSTEM as u8
                    && message.utterance_id == utterance_id
        )));
        assert!(!effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
        )));
    }

    #[test]
    fn invalid_uuid_and_cross_topic_drift_aborts_the_retained_agent_identity() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let utterance_id = "agent-0-invalid-cross-topic";
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, utterance_id),
        )
        .unwrap();

        let mut drifted = second_two_token_speech(Kind::System, utterance_id);
        drifted.request_id = "not-a-request-uuid".to_owned();
        drifted.generation_id = "not-a-generation-uuid".to_owned();
        let effects = process_speech_chunk(&mut controller, Kind::System, drifted).unwrap();

        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert_eq!(controller.machine.utterance_count(), 0);
        assert!(controller.terminal_speech_ids.contains(utterance_id));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.kind == PlaybackEventMessage::AGENT as u8
                    && message.request_id == "00000000-0000-4000-8000-000000000001"
                    && message.generation_id == "00000000-0000-4000-8000-000000000002"
                    && message.utterance_id == utterance_id
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
                    && message.utterance_id == utterance_id
        )));
        assert!(!effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_SYSTEM as u8
        )));
    }

    #[test]
    fn initial_topic_kind_mismatch_fails_without_creating_a_tombstone() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let utterance_id = "system-no-retained-identity";
        let mismatch = first_two_token_speech(Kind::Agent, utterance_id);

        assert!(process_speech_chunk(&mut controller, Kind::System, mismatch).is_err());
        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert_eq!(controller.machine.utterance_count(), 0);
        assert!(!controller.terminal_speech_ids.contains(utterance_id));
    }

    #[test]
    fn stalled_stream_timeout_aborts_only_its_retained_identity() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let expired_id = "agent-0-stalled";
        let surviving_id = "system-current";
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, expired_id),
        )
        .unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::System,
            first_two_token_speech(Kind::System, surviving_id),
        )
        .unwrap();
        let now = Instant::now();
        controller
            .speech_streams
            .get_mut(expired_id)
            .expect("expired stream")
            .last_progress_at = now - SPEECH_INPUT_PROGRESS_TIMEOUT - Duration::from_millis(1);
        controller
            .speech_streams
            .get_mut(surviving_id)
            .expect("surviving stream")
            .last_progress_at = now;

        let effects = controller.expire_stalled_speech(now).unwrap();

        assert!(!controller.speech_streams.contains_key(expired_id));
        assert!(controller.speech_streams.contains_key(surviving_id));
        assert!(controller.terminal_speech_ids.contains(expired_id));
        assert!(!controller.terminal_speech_ids.contains(surviving_id));
        assert_eq!(controller.machine.utterance_count(), 1);
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == expired_id
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
                    && message.utterance_id == expired_id
        )));
        assert!(!effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_SYSTEM as u8
        )));
    }

    #[test]
    fn multi_chunk_speech_is_playable_before_complete_and_finalizes_once() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-stream"),
        )
        .unwrap();
        let started = Instant::now();
        let initial_output = frame_effects(controller.pump_at(started).unwrap());
        assert_eq!(
            initial_output
                .iter()
                .map(|frame| frame.seq)
                .collect::<Vec<_>>(),
            vec![0]
        );
        assert_eq!(
            (
                initial_output[0].sample_index,
                initial_output[0].frame_count
            ),
            (0, 960)
        );
        assert!(initial_output.iter().all(|frame| !frame.final_));

        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            second_two_token_speech(Kind::Agent, "agent-0-stream"),
        )
        .unwrap();
        let before_complete = {
            let stream = controller
                .speech_streams
                .get("agent-0-stream")
                .expect("stream stays open until COMPLETE");
            PlaybackMachine::event(EventName::AgentPaused, &stream.item)
        };
        assert!(!before_complete.total_frames_valid);
        assert_eq!(before_complete.total_frames, 0);

        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                "agent-0-stream",
                SynthesizedSpeechChunkMessage::COMPLETE as u8,
                2,
                5_120,
            ),
        )
        .unwrap();
        let mut final_output = None;
        for _ in 0..64 {
            let deadline = controller
                .next_send_deadline
                .expect("an unfinished playback must retain its next deadline");
            for output in frame_effects(controller.pump_at(deadline).unwrap()) {
                if output.final_ {
                    final_output = Some(output);
                    break;
                }
            }
            if final_output.is_some() {
                break;
            }
        }
        let final_output = final_output.expect("completed stream must publish one final frame");
        assert!(final_output.seq > 0);
        assert!(final_output.sample_index >= 960);
        assert!(final_output.final_);
        assert!(final_output.frame_count > 0);
        assert!(!controller.speech_streams.contains_key("agent-0-stream"));
        assert!(controller.terminal_speech_ids.contains("agent-0-stream"));
    }

    #[test]
    fn independently_segmented_synthesis_is_one_contiguous_logical_playback_stream() {
        let utterance_id = "agent-4-segmented";
        let kind = SynthesizedSpeechChunkMessage::AGENT as u8;
        let source_progress = [
            (0, 0, 0),
            (1, 1, 3),
            (2, 2, 6),
            (3, 3, 9),
            (4, 3, 9),
            (5, 4, 12),
            (6, 4, 12),
        ]
        .into_iter()
        .map(
            |(committed_text_tokens, source_char_end, source_utf8_end)| TtsSourceProgressMessage {
                committed_text_tokens,
                source_char_end,
                source_utf8_end,
            },
        )
        .collect::<Vec<_>>();
        let logical_audio = [
            (0, 0, 4, false, 2, 4_096),
            (1, 4_096, 8, false, 5, 12_288),
            (2, 12_288, 3, true, 6, 15_360),
        ];
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();

        for (sequence, first_sample_index, codec_frames, final_chunk, tokens, alignment_sample) in
            logical_audio
        {
            let effects = process_speech_chunk(
                &mut controller,
                Kind::Agent,
                SynthesizedSpeechChunkMessage {
                    kind,
                    event: SynthesizedSpeechChunkMessage::AUDIO as u8,
                    request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
                    generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
                    utterance_id: utterance_id.to_owned(),
                    sequence,
                    first_sample_index,
                    sample_rate_hz: 22_050,
                    channels: 1,
                    pcm_f32: vec![0.0; codec_frames * MAGPIE_CODEC_FRAME_SAMPLES as usize],
                    final_chunk,
                    committed_text_tokens: tokens,
                    alignment_events: vec![TtsAlignmentEventMessage {
                        sample_index: alignment_sample,
                        committed_text_tokens: tokens,
                    }],
                    source_text: if sequence == 0 {
                        "一。二。".to_owned()
                    } else {
                        String::new()
                    },
                    source_progress: if sequence == 0 {
                        source_progress.clone()
                    } else {
                        Vec::new()
                    },
                },
            )
            .expect("logical audio chunk");
            assert!(effects
                .iter()
                .all(|effect| !matches!(effect, Effect::Control(_))));
        }

        let stream = controller
            .speech_streams
            .get(utterance_id)
            .expect("stream remains open until logical COMPLETE");
        assert_eq!(stream.next_sequence, 3);
        assert_eq!(stream.next_sample_index, 15_360);
        assert_eq!(stream.committed_text_tokens, 6);
        assert!(stream.final_audio_seen);

        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                utterance_id,
                SynthesizedSpeechChunkMessage::COMPLETE as u8,
                3,
                15_360,
            ),
        )
        .expect("one logical completion");
        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert!(controller.terminal_speech_ids.contains(utterance_id));
    }

    #[test]
    fn speech_sequence_gap_aborts_only_that_utterance() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-gap"),
        )
        .unwrap();
        let mut gap = second_two_token_speech(Kind::Agent, "agent-0-gap");
        gap.sequence = 2;
        let effects = process_speech_chunk(&mut controller, Kind::Agent, gap).unwrap();

        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == "agent-0-gap"
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
                    && message.release_hold == PlaybackControlMessage::NONE as u8
                    && message.minimum_agent_epoch == 0
                    && message.utterance_id == "agent-0-gap"
        )));
        assert!(!controller.speech_streams.contains_key("agent-0-gap"));
        assert!(controller.terminal_speech_ids.contains("agent-0-gap"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn invalid_initial_magpie_chunk_is_rejected_fail_closed() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        let effects = process_speech_chunk(
            &mut controller,
            Kind::Agent,
            input_speech(Kind::Agent, "agent-0-invalid-initial", 4_095),
        )
        .unwrap();

        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == "agent-0-invalid-initial"
        )));
        assert!(!controller
            .speech_streams
            .contains_key("agent-0-invalid-initial"));
        assert!(controller
            .terminal_speech_ids
            .contains("agent-0-invalid-initial"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn audio_after_an_explicit_final_magpie_chunk_is_rejected_fail_closed() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-after-tail"),
        )
        .unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            second_two_token_speech(Kind::Agent, "agent-0-after-tail"),
        )
        .unwrap();

        let mut after_tail = second_two_token_speech(Kind::Agent, "agent-0-after-tail");
        after_tail.sequence = 2;
        after_tail.first_sample_index = 5_120;
        after_tail.alignment_events.clear();
        let effects = process_speech_chunk(&mut controller, Kind::Agent, after_tail).unwrap();

        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == "agent-0-after-tail"
        )));
        assert!(!controller.speech_streams.contains_key("agent-0-after-tail"));
        assert!(controller
            .terminal_speech_ids
            .contains("agent-0-after-tail"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn short_non_final_magpie_chunk_is_rejected_fail_closed() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-short-non-final"),
        )
        .unwrap();
        let mut invalid = second_two_token_speech(Kind::Agent, "agent-0-short-non-final");
        invalid.final_chunk = false;
        let effects = process_speech_chunk(&mut controller, Kind::Agent, invalid).unwrap();

        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == "agent-0-short-non-final"
        )));
        assert!(!controller
            .speech_streams
            .contains_key("agent-0-short-non-final"));
        assert!(controller
            .terminal_speech_ids
            .contains("agent-0-short-non-final"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn explicit_eight_frame_final_tail_is_not_treated_as_steady_audio() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-eight-final"),
        )
        .unwrap();
        let mut final_eight = second_two_token_speech(Kind::Agent, "agent-0-eight-final");
        final_eight.pcm_f32 = vec![0.0; 8 * MAGPIE_CODEC_FRAME_SAMPLES as usize];
        final_eight.alignment_events = vec![TtsAlignmentEventMessage {
            sample_index: 12_288,
            committed_text_tokens: 2,
        }];
        process_speech_chunk(&mut controller, Kind::Agent, final_eight).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                "agent-0-eight-final",
                SynthesizedSpeechChunkMessage::COMPLETE as u8,
                2,
                12_288,
            ),
        )
        .unwrap();

        assert!(!controller
            .speech_streams
            .contains_key("agent-0-eight-final"));
        assert!(controller
            .terminal_speech_ids
            .contains("agent-0-eight-final"));
    }

    #[test]
    fn zero_frame_final_marker_closes_without_writing_pcm_or_advancing_samples() {
        let utterance_id = "agent-0-zero-final";
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            input_speech(Kind::Agent, utterance_id, 4_096),
        )
        .unwrap();
        let pcm_bytes_before_marker = controller
            .speech_streams
            .get(utterance_id)
            .expect("open stream")
            .item
            .borrow()
            .pcm
            .len();
        let mut marker = terminal_speech(
            Kind::Agent,
            utterance_id,
            SynthesizedSpeechChunkMessage::AUDIO as u8,
            1,
            4_096,
        );
        marker.final_chunk = true;
        marker.committed_text_tokens = 1;
        process_speech_chunk(&mut controller, Kind::Agent, marker).unwrap();

        let stream = controller
            .speech_streams
            .get(utterance_id)
            .expect("stream remains open until COMPLETE");
        assert_eq!(stream.next_sequence, 2);
        assert_eq!(stream.next_sample_index, 4_096);
        assert_eq!(stream.item.borrow().pcm.len(), pcm_bytes_before_marker);
        assert!(stream.final_audio_seen);

        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                utterance_id,
                SynthesizedSpeechChunkMessage::COMPLETE as u8,
                2,
                4_096,
            ),
        )
        .unwrap();
        assert!(!controller.speech_streams.contains_key(utterance_id));
        assert!(controller.terminal_speech_ids.contains(utterance_id));
    }

    #[test]
    fn zero_frame_non_final_audio_is_rejected_fail_closed() {
        let utterance_id = "agent-0-zero-non-final";
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            input_speech(Kind::Agent, utterance_id, 4_096),
        )
        .unwrap();
        let mut marker = terminal_speech(
            Kind::Agent,
            utterance_id,
            SynthesizedSpeechChunkMessage::AUDIO as u8,
            1,
            4_096,
        );
        marker.committed_text_tokens = 1;
        let effects = process_speech_chunk(&mut controller, Kind::Agent, marker).unwrap();
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Control(message)
                if message.action == PlaybackControlMessage::ABORT_AGENT as u8
                    && message.utterance_id == utterance_id
        )));
        assert!(!controller.speech_streams.contains_key(utterance_id));
    }

    #[test]
    fn complete_before_a_post_initial_chunk_is_rejected_fail_closed() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            input_speech(Kind::Agent, "agent-0-no-tail", 4_096),
        )
        .unwrap();
        let effects = process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                "agent-0-no-tail",
                SynthesizedSpeechChunkMessage::COMPLETE as u8,
                1,
                4_096,
            ),
        )
        .unwrap();

        assert!(effects.into_iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && message.utterance_id == "agent-0-no-tail"
        )));
        assert!(!controller.speech_streams.contains_key("agent-0-no-tail"));
        assert!(controller.terminal_speech_ids.contains("agent-0-no-tail"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn source_progress_uses_ceil_scaled_boundaries_and_physical_drain() {
        assert_eq!(
            ceil_ratio_u64(u128::from(1_u64) * 48_000, 22_050).unwrap(),
            3
        );
        let progress = vec![
            TtsSourceProgressMessage {
                committed_text_tokens: 0,
                source_char_end: 0,
                source_utf8_end: 0,
            },
            TtsSourceProgressMessage {
                committed_text_tokens: 1,
                source_char_end: 1,
                source_utf8_end: 3,
            },
        ];
        assert_eq!(validate_source_progress("あ", &progress).unwrap(), (1, 1));
        let mut malformed = progress.clone();
        malformed[1].source_utf8_end = 1;
        assert!(validate_source_progress("あ", &malformed).is_err());

        let item = Rc::new(RefCell::new(
            Utterance::new(
                PlaybackIdentity {
                    kind: Kind::Agent,
                    request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
                    generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
                    utterance_id: "agent-0-progress".to_owned(),
                },
                vec![0; 6],
                PcmFormat::pcm16(48_000, 1),
                vec![SourceAlignment {
                    end_frame: 3,
                    source_end: 1,
                }],
                1,
                true,
            )
            .unwrap(),
        ));
        item.borrow_mut().played_offset_frames = 2;
        assert_eq!(
            PlaybackMachine::event(EventName::AgentPaused, &item).played_source_characters,
            0
        );
        item.borrow_mut().played_offset_frames = 3;
        let event = PlaybackMachine::event(EventName::AgentPaused, &item);
        assert_eq!(event.played_source_characters, 1);
        assert!(event.total_frames_valid);
    }

    #[test]
    fn speech_abort_flushes_published_partial_audio() {
        let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            first_two_token_speech(Kind::Agent, "agent-0-abort"),
        )
        .unwrap();
        process_speech_chunk(
            &mut controller,
            Kind::Agent,
            second_two_token_speech(Kind::Agent, "agent-0-abort"),
        )
        .unwrap();
        let published = frame_effects(controller.pump().unwrap());
        assert_eq!(published.len(), TTS_INITIAL_PLAYOUT_CHUNKS);
        assert!(published.iter().all(|frame| !frame.final_));
        let published_keys = published
            .iter()
            .map(|frame| (frame.utterance_id.clone(), frame.seq))
            .collect::<Vec<_>>();

        let effects = process_speech_chunk(
            &mut controller,
            Kind::Agent,
            terminal_speech(
                Kind::Agent,
                "agent-0-abort",
                SynthesizedSpeechChunkMessage::ABORT as u8,
                2,
                5_120,
            ),
        )
        .unwrap();
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Flush(command)
                if command.keys == published_keys
        )));
        assert!(effects.iter().any(|effect| matches!(
            effect,
            Effect::Event(message)
                if message.event == PlaybackEventMessage::AGENT_ABORTED as u8
                    && !message.total_frames_valid
        )));
        assert!(!controller.speech_streams.contains_key("agent-0-abort"));
        assert!(controller.pump().unwrap().is_empty());
    }

    #[test]
    fn streaming_converter_preserves_state_across_input_chunks() {
        let factory = PcmConverter::new(48_000, 1).unwrap();
        let pcm = (0..882)
            .map(|index| ((index as f32) * 0.013).sin() * 0.25)
            .collect::<Vec<_>>();

        let mut split = factory.start_speech(22_050, 1).unwrap();
        let mut split_output = split.push(0, &pcm[..441]).unwrap();
        split_output.extend(split.push(441, &pcm[441..]).unwrap());
        split_output.extend(split.finish().unwrap());

        let mut whole = factory.start_speech(22_050, 1).unwrap();
        let mut whole_output = whole.push(0, &pcm).unwrap();
        whole_output.extend(whole.finish().unwrap());

        assert_eq!(split_output, whole_output);
        assert_eq!(split_output.len(), 1_920 * 2);
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

        let effects = process_speech_chunk(
            &mut controller,
            Kind::System,
            input_speech(Kind::System, "system-late", 16),
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
    fn magpie_tts_starts_with_exactly_its_first_converted_playout_chunk() {
        for (kind, utterance_id) in [
            (Kind::Agent, "agent-0-immediate"),
            (Kind::System, "system-immediate"),
        ] {
            let mut controller = PlaybackController::new(48_000, 1, 20).unwrap();
            process_speech_chunk(
                &mut controller,
                kind,
                first_two_token_speech(kind, utterance_id),
            )
            .unwrap();
            let started = Instant::now();

            let first = controller.pump_at(started).unwrap();
            assert_eq!(first.len(), TTS_INITIAL_PLAYOUT_CHUNKS);
            match &first[0] {
                Effect::Frame(frame, emit_first_tts_timing) => {
                    assert_eq!(frame.kind, kind.code());
                    assert_eq!(frame.utterance_id, utterance_id);
                    assert_eq!(frame.seq, 0);
                    assert_eq!(frame.sample_index, 0);
                    assert!(*emit_first_tts_timing);
                }
                _ => panic!("first Magpie playout effect must be one frame"),
            }
            assert_eq!(controller.output_pending.len(), 1);
            assert!(controller
                .pump_at(started + Duration::from_millis(19))
                .unwrap()
                .is_empty());

            let second = frame_effect(
                controller
                    .pump_at(started + Duration::from_millis(20))
                    .unwrap(),
            );
            assert_eq!(second.seq, 1);
            assert_eq!(controller.output_pending.len(), 2);
        }
    }

    #[test]
    fn fully_assembled_cue_retains_its_three_chunk_prefill() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Cue, "cue-prefill", 80))
            .unwrap();

        let prefill = frame_effects(controller.pump_at(Instant::now()).unwrap());
        assert_eq!(
            prefill.iter().map(|frame| frame.seq).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );
        assert_eq!(prefill.len(), CUE_INITIAL_PLAYOUT_CHUNKS);
    }

    #[test]
    fn late_pacer_tick_preserves_the_deadline_grid_without_bursting() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-late", 80))
            .unwrap();
        let started = Instant::now();
        assert_eq!(
            frame_effects(controller.pump_at(started).unwrap()).len(),
            TTS_INITIAL_PLAYOUT_CHUNKS
        );

        let late = started + Duration::from_millis(4);
        assert_eq!(frame_effect(controller.pump_at(late).unwrap()).seq, 1);
        assert!(controller
            .pump_at(late + Duration::from_micros(999))
            .unwrap()
            .is_empty());
        assert_eq!(
            frame_effect(
                controller
                    .pump_at(started + Duration::from_millis(5))
                    .unwrap()
            )
            .seq,
            2
        );
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

        let mut frames = frame_effects(controller.pump().unwrap());
        let second_deadline = controller
            .next_send_deadline
            .expect("the second Agent frame must be paced");
        frames.extend(frame_effects(controller.pump_at(second_deadline).unwrap()));
        assert_eq!(frames.len(), 2);
        let first = &frames[0];
        assert_eq!((first.seq, first.frame_count, first.final_), (0, 16, false));
        let second = &frames[1];
        assert_eq!(
            (second.seq, second.frame_count, second.final_),
            (1, 16, true)
        );
        controller
            .on_output_drained(output_ack("agent-0-flush", 0, 16, false, STATUS_ACCEPTED))
            .unwrap();

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
        assert!(controller.finish_flush_service().unwrap().is_empty());
        assert!(controller
            .on_output_drained(output_ack("agent-0-flush", 0, 6, false, STATUS_FLUSHED))
            .unwrap()
            .is_empty());
        let committed = controller
            .on_output_drained(output_ack("agent-0-flush", 1, 0, false, STATUS_FLUSHED))
            .unwrap();
        let committed_events = committed
            .into_iter()
            .filter_map(|effect| match effect {
                Effect::Event(event) => Some(event),
                Effect::Frame(_, _) | Effect::Flush(_) | Effect::Control(_) => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(committed_events.len(), 1);
        assert_eq!(
            committed_events[0].event,
            PlaybackEventMessage::AGENT_PAUSE_COMMITTED as u8
        );
        assert_eq!(
            committed_events[0].pause_id,
            "00000000-0000-4000-8000-000000000003"
        );
        assert_eq!(committed_events[0].played_frames, 6);
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
    fn pause_commit_waits_for_flush_service_after_all_output_acknowledgements() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-ack-first", 16))
            .unwrap();
        let frame = frame_effect(controller.pump().unwrap());
        controller
            .on_output_drained(output_ack(
                &frame.utterance_id,
                frame.seq,
                frame.frame_count,
                frame.final_,
                STATUS_ACCEPTED,
            ))
            .unwrap();

        let pause = controller
            .on_control(control(
                PlaybackControlMessage::PAUSE as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();
        let paused = event_effects(pause);
        assert_eq!(paused.len(), 1);
        assert_eq!(paused[0].event, PlaybackEventMessage::AGENT_PAUSED as u8);
        assert_eq!(paused[0].pause_id, "00000000-0000-4000-8000-000000000003");

        let before_service = controller
            .on_output_drained(output_ack(
                &frame.utterance_id,
                frame.seq,
                5,
                false,
                STATUS_FLUSHED,
            ))
            .unwrap();
        assert!(event_effects(before_service).is_empty());
        assert!(controller.flush_request.is_some());

        let committed = event_effects(controller.finish_flush_service().unwrap());
        assert_eq!(committed.len(), 1);
        assert_eq!(
            committed[0].event,
            PlaybackEventMessage::AGENT_PAUSE_COMMITTED as u8
        );
        assert_eq!(committed[0].pause_id, paused[0].pause_id);
        assert_eq!(committed[0].played_frames, 5);
        assert!(controller.flush_request.is_none());
    }

    #[test]
    fn discard_racing_pause_runs_after_the_exact_pause_commit() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-old", 16))
            .unwrap();
        let frame = frame_effect(controller.pump().unwrap());
        controller
            .on_output_drained(output_ack(
                &frame.utterance_id,
                frame.seq,
                frame.frame_count,
                frame.final_,
                STATUS_ACCEPTED,
            ))
            .unwrap();
        controller
            .on_control(control(
                PlaybackControlMessage::PAUSE as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();

        let deferred = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::DISCARD as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::USER as u8,
                minimum_agent_epoch: 1,
                utterance_id: String::new(),
            })
            .unwrap();
        assert!(deferred.is_empty());
        assert_eq!(controller.deferred_pause_controls.len(), 1);
        assert!(controller.finish_flush_service().unwrap().is_empty());

        let events = event_effects(
            controller
                .on_output_drained(output_ack(
                    &frame.utterance_id,
                    frame.seq,
                    4,
                    false,
                    STATUS_FLUSHED,
                ))
                .unwrap(),
        );
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0].event,
            PlaybackEventMessage::AGENT_PAUSE_COMMITTED as u8
        );
        assert_eq!(events[0].played_frames, 4);
        assert_eq!(events[1].event, PlaybackEventMessage::AGENT_DISCARDED as u8);
        assert!(events[1].pause_id.is_empty());
        assert_eq!(controller.minimum_agent_epoch, 1);
        assert!(controller.deferred_pause_controls.is_empty());
    }

    #[test]
    fn pending_pause_rejects_a_second_pause_and_bounds_deferred_controls() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-pending", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());
        controller
            .on_control(control(
                PlaybackControlMessage::PAUSE as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();

        let second_pause = controller
            .on_control(PlaybackControlMessage {
                action: PlaybackControlMessage::PAUSE as u8,
                pause_id: "00000000-0000-4000-8000-000000000004".to_owned(),
                release_hold: PlaybackControlMessage::NONE as u8,
                minimum_agent_epoch: 0,
                utterance_id: String::new(),
            })
            .err()
            .expect("a second pause must be rejected");
        assert!(second_pause.contains("second PAUSE"));

        for _ in 0..MAX_DEFERRED_PAUSE_CONTROLS {
            assert!(controller
                .on_control(control(
                    PlaybackControlMessage::RESUME as u8,
                    PlaybackControlMessage::NONE as u8,
                    "",
                ))
                .unwrap()
                .is_empty());
        }
        let overflow = controller
            .on_control(control(
                PlaybackControlMessage::RESUME as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .err()
            .expect("the bounded deferred-control FIFO must reject overflow");
        assert!(overflow.contains("deferred-control FIFO"));
        assert_eq!(
            controller.deferred_pause_controls.len(),
            MAX_DEFERRED_PAUSE_CONTROLS
        );
    }

    #[test]
    fn every_non_pause_control_enters_the_pending_pause_fifo_in_arrival_order() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        controller
            .machine
            .enqueue(utterance(Kind::Agent, "agent-0-pending", 16))
            .unwrap();
        frame_effect(controller.pump().unwrap());
        controller
            .on_control(control(
                PlaybackControlMessage::PAUSE as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ))
            .unwrap();

        let controls = [
            control(
                PlaybackControlMessage::RESUME as u8,
                PlaybackControlMessage::NONE as u8,
                "",
            ),
            PlaybackControlMessage {
                action: PlaybackControlMessage::DISCARD as u8,
                pause_id: String::new(),
                release_hold: PlaybackControlMessage::USER as u8,
                minimum_agent_epoch: 1,
                utterance_id: String::new(),
            },
            control(
                PlaybackControlMessage::ABORT_AGENT as u8,
                PlaybackControlMessage::NONE as u8,
                "agent-0-pending",
            ),
            control(
                PlaybackControlMessage::ABORT_SYSTEM as u8,
                PlaybackControlMessage::NONE as u8,
                "system-deferred",
            ),
        ];
        for control in controls {
            assert!(controller.on_control(control).unwrap().is_empty());
        }

        assert!(matches!(
            controller.deferred_pause_controls.front(),
            Some(ControlAction::Resume)
        ));
        assert!(matches!(
            controller.deferred_pause_controls.get(1),
            Some(ControlAction::Discard { .. })
        ));
        assert!(matches!(
            controller.deferred_pause_controls.get(2),
            Some(ControlAction::AbortAgent { .. })
        ));
        assert!(matches!(
            controller.deferred_pause_controls.get(3),
            Some(ControlAction::AbortSystem { .. })
        ));
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
        assert!(controller.finish_flush_service().unwrap().is_empty());
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

    #[test]
    fn terminal_speech_tombstones_are_bounded_for_long_running_dialogue() {
        let mut controller = PlaybackController::new(16_000, 1, 1).unwrap();
        for index in 0..=MAX_TERMINAL_SPEECH_IDS {
            controller.remember_terminal_speech(format!("agent-0-{index}"));
        }

        assert_eq!(
            controller.terminal_speech_ids.len(),
            MAX_TERMINAL_SPEECH_IDS
        );
        assert_eq!(
            controller.terminal_speech_order.len(),
            MAX_TERMINAL_SPEECH_IDS
        );
        assert!(!controller.terminal_speech_ids.contains("agent-0-0"));
        assert!(controller
            .terminal_speech_ids
            .contains(&format!("agent-0-{MAX_TERMINAL_SPEECH_IDS}")));
    }
}
