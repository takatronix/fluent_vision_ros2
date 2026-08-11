use std::collections::{HashSet, VecDeque};
use std::io::{self, Write};

use serde::Serialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

const AGENT_ID_PREFIX: &str = "agent-";
const SEED_DOMAIN: &[u8] = b"fv_tts/magpie-seed/v1\0";
const MAX_SYSTEM_ABORT_TOMBSTONES: usize = 64;
// DDS subscriptions use bounded KEEP_LAST histories. Retain a much wider
// terminal-ID reorder window without turning a unique request ID into a
// process-lifetime memory leak. Producers still own the globally-unique ID
// contract; this receiver detects duplicates while they are live or recent.
const MAX_RECENT_UTTERANCE_IDS: usize = 1024;
pub const MAX_UTTERANCE_ID_BYTES: usize = 256;
pub const MAX_FRONTEND_REQUEST_BYTES: usize = 1024 * 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpeechKind {
    Agent,
    System,
}

impl SpeechKind {
    pub const fn ros_value(self) -> u8 {
        match self {
            Self::Agent => 1,
            Self::System => 2,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Agent => "agent",
            Self::System => "system",
        }
    }

    pub fn from_ros(value: u8) -> Result<Self, ContractError> {
        match value {
            1 => Ok(Self::Agent),
            2 => Ok(Self::System),
            _ => Err(ContractError::InvalidSpeechKind),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SayRequest {
    pub kind: SpeechKind,
    pub utterance_id: String,
    pub text: String,
}

impl SayRequest {
    pub fn from_fields(
        kind: u8,
        utterance_id: String,
        text: String,
    ) -> Result<Self, ContractError> {
        let kind = SpeechKind::from_ros(kind)?;
        let trimmed_utterance_id = trim_python_whitespace(&utterance_id);
        if trimmed_utterance_id.is_empty() {
            return Err(ContractError::EmptySayField("utterance_id"));
        }
        if trimmed_utterance_id != utterance_id {
            return Err(ContractError::SurroundingUtteranceIdWhitespace);
        }
        if trim_python_whitespace(&text).is_empty() {
            return Err(ContractError::EmptySayField("text"));
        }
        if utterance_id.as_bytes().contains(&0) {
            return Err(ContractError::NulUtteranceId);
        }
        if utterance_id.len() > MAX_UTTERANCE_ID_BYTES {
            return Err(ContractError::UtteranceIdTooLong);
        }
        validate_frontend_request_transport(&utterance_id, &text)?;
        if kind == SpeechKind::Agent {
            let _ = agent_floor_epoch(&utterance_id)?;
        }
        Ok(Self {
            kind,
            utterance_id,
            text,
        })
    }
}

pub fn validate_uuid(value: &str, field: &'static str) -> Result<(), ContractError> {
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
        return Err(ContractError::InvalidUuid(field));
    }
    Ok(())
}

#[derive(Serialize)]
struct FrontendRequestTransport<'a> {
    schema_version: u32,
    request_id: &'a str,
    text: &'a str,
}

pub fn validate_frontend_request_transport(
    request_id: &str,
    text: &str,
) -> Result<(), ContractError> {
    if request_id.is_empty() {
        return Err(ContractError::EmptySayField("utterance_id"));
    }
    if request_id.as_bytes().contains(&0) {
        return Err(ContractError::NulUtteranceId);
    }
    if request_id.len() > MAX_UTTERANCE_ID_BYTES {
        return Err(ContractError::UtteranceIdTooLong);
    }
    let mut counter = BoundedByteCounter::new(MAX_FRONTEND_REQUEST_BYTES);
    let serialize_result = serde_json::to_writer(
        &mut counter,
        &FrontendRequestTransport {
            schema_version: 1,
            request_id,
            text,
        },
    );
    if counter.exceeded {
        return Err(ContractError::FrontendRequestTooLarge);
    }
    serialize_result.map_err(ContractError::SerializeFrontendRequest)?;
    counter
        .write_all(b"\n")
        .map_err(|_| ContractError::FrontendRequestTooLarge)
}

struct BoundedByteCounter {
    length: usize,
    maximum_bytes: usize,
    exceeded: bool,
}

impl BoundedByteCounter {
    const fn new(maximum_bytes: usize) -> Self {
        Self {
            length: 0,
            maximum_bytes,
            exceeded: false,
        }
    }
}

impl Write for BoundedByteCounter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let Some(next_length) = self.length.checked_add(bytes.len()) else {
            self.exceeded = true;
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "frontend request byte length overflowed",
            ));
        };
        if next_length > self.maximum_bytes {
            self.exceeded = true;
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "frontend request exceeds its bounded JSONL transport",
            ));
        }
        self.length = next_length;
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResultStatus {
    Completed,
    Failed,
    Cancelled,
}

impl ResultStatus {
    pub const fn ros_value(self) -> u8 {
        match self {
            Self::Completed => 1,
            Self::Failed => 2,
            Self::Cancelled => 3,
        }
    }
}

pub fn deterministic_seed(
    manifest_sha256: &[u8; 32],
    utterance_id: &str,
    source_text: &str,
) -> u32 {
    let mut digest = Sha256::new();
    digest.update(SEED_DOMAIN);
    digest.update(manifest_sha256);
    append_length_framed(&mut digest, utterance_id.as_bytes());
    append_length_framed(&mut digest, source_text.as_bytes());
    let bytes: [u8; 32] = digest.finalize().into();
    u32::from_le_bytes(bytes[..4].try_into().expect("four-byte digest prefix"))
}

fn append_length_framed(digest: &mut Sha256, value: &[u8]) {
    let length = u64::try_from(value.len()).expect("Rust slice length fits u64");
    digest.update(length.to_le_bytes());
    digest.update(value);
}

pub fn decode_sha256(value: &str, field: &'static str) -> Result<[u8; 32], ContractError> {
    let bytes = value.as_bytes();
    if bytes.len() != 64 {
        return Err(ContractError::InvalidSha256(field));
    }
    let mut decoded = [0_u8; 32];
    for (index, output) in decoded.iter_mut().enumerate() {
        let offset = index * 2;
        let high = hex_nibble(bytes[offset]).ok_or(ContractError::InvalidSha256(field))?;
        let low = hex_nibble(bytes[offset + 1]).ok_or(ContractError::InvalidSha256(field))?;
        *output = high << 4 | low;
    }
    if decoded.iter().all(|byte| *byte == 0) {
        return Err(ContractError::InvalidSha256(field));
    }
    Ok(decoded)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

pub fn encode_sha256(value: &[u8; 32]) -> String {
    let mut encoded = String::with_capacity(64);
    for byte in value {
        use std::fmt::Write;
        write!(&mut encoded, "{byte:02x}").expect("writing to String cannot fail");
    }
    encoded
}

pub fn agent_floor_epoch(utterance_id: &str) -> Result<u64, ContractError> {
    let suffix = utterance_id
        .strip_prefix(AGENT_ID_PREFIX)
        .ok_or(ContractError::InvalidAgentUtteranceId)?;
    let (epoch, identifier) = suffix
        .split_once('-')
        .ok_or(ContractError::InvalidAgentUtteranceId)?;
    if epoch.is_empty() || identifier.is_empty() || !epoch.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Err(ContractError::InvalidAgentUtteranceId);
    }
    epoch
        .parse()
        .map_err(|_| ContractError::InvalidAgentUtteranceId)
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceProgress {
    pub committed_text_tokens: u64,
    pub source_char_end: u64,
    pub source_utf8_end: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedUtterance {
    pub source_text: String,
    pub token_ids: Vec<i64>,
    pub progress: Vec<SourceProgress>,
}

impl PreparedUtterance {
    pub fn validate(
        source_text: String,
        token_ids: Vec<i64>,
        progress: Vec<SourceProgress>,
        tokenizer_vocabulary_size: u32,
        eos_token_id: u32,
        maximum_text_tokens: u32,
    ) -> Result<Self, ContractError> {
        if source_text.is_empty() {
            return Err(ContractError::InvalidFrontendData(
                "source_text must not be empty",
            ));
        }
        if token_ids.is_empty() || token_ids.len() > maximum_text_tokens as usize {
            return Err(ContractError::InvalidFrontendData(
                "prepared token count is outside the authenticated model limit",
            ));
        }
        let final_index = token_ids.len() - 1;
        if token_ids[final_index] != i64::from(eos_token_id) {
            return Err(ContractError::InvalidFrontendData(
                "prepared token sequence must end with the authenticated EOS",
            ));
        }
        if token_ids[..final_index]
            .iter()
            .any(|token| *token < 0 || *token >= i64::from(tokenizer_vocabulary_size))
        {
            return Err(ContractError::InvalidFrontendData(
                "non-final prepared token ID is outside normal tokenizer rows",
            ));
        }
        if progress.len() != token_ids.len() + 1 {
            return Err(ContractError::InvalidFrontendData(
                "source progress must contain one entry for every end-exclusive token position",
            ));
        }
        let total_chars = source_text.chars().count() as u64;
        let total_utf8 = source_text.len() as u64;
        let mut previous_chars = 0_u64;
        let mut previous_utf8 = 0_u64;
        for (index, entry) in progress.iter().enumerate() {
            let expected_tokens = index as u64;
            let utf8_end = usize::try_from(entry.source_utf8_end).map_err(|_| {
                ContractError::InvalidFrontendData("source UTF-8 offset exceeds usize")
            })?;
            if entry.committed_text_tokens != expected_tokens
                || entry.source_char_end < previous_chars
                || entry.source_char_end > total_chars
                || entry.source_utf8_end < previous_utf8
                || entry.source_utf8_end > total_utf8
                || !source_text.is_char_boundary(utf8_end)
                || source_text[..utf8_end].chars().count() as u64 != entry.source_char_end
            {
                return Err(ContractError::InvalidFrontendData(
                    "source progress is not an exact monotonic source-text map",
                ));
            }
            previous_chars = entry.source_char_end;
            previous_utf8 = entry.source_utf8_end;
        }
        let first = progress.first().expect("non-empty progress checked above");
        let last = progress.last().expect("non-empty progress checked above");
        if first.source_char_end != 0
            || first.source_utf8_end != 0
            || last.source_char_end != total_chars
            || last.source_utf8_end != total_utf8
        {
            return Err(ContractError::InvalidFrontendData(
                "source progress does not span the exact source text",
            ));
        }
        Ok(Self {
            source_text,
            token_ids,
            progress,
        })
    }

    pub fn total_prepared_tokens(&self) -> u64 {
        self.token_ids.len() as u64
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedSegment {
    pub segment_index: u64,
    pub source_char_start: u64,
    pub source_char_end: u64,
    pub source_utf8_start: u64,
    pub source_utf8_end: u64,
    pub prepared: PreparedUtterance,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedRequest {
    pub source_text: String,
    pub segments: Vec<PreparedSegment>,
    pub progress: Vec<SourceProgress>,
    pub total_prepared_tokens: u64,
}

impl PreparedRequest {
    pub fn from_segments(
        source_text: String,
        segments: Vec<PreparedSegment>,
    ) -> Result<Self, ContractError> {
        if source_text.is_empty() || segments.is_empty() {
            return Err(ContractError::InvalidFrontendData(
                "prepared segments require non-empty source text and at least one segment",
            ));
        }
        let total_chars = source_text.chars().count() as u64;
        let total_utf8 = source_text.len() as u64;
        let mut expected_char_start = 0_u64;
        let mut expected_utf8_start = 0_u64;
        let mut total_prepared_tokens = 0_u64;
        let mut progress = vec![SourceProgress {
            committed_text_tokens: 0,
            source_char_end: 0,
            source_utf8_end: 0,
        }];
        for (expected_index, segment) in segments.iter().enumerate() {
            let expected_index = u64::try_from(expected_index).map_err(|_| {
                ContractError::InvalidFrontendData("prepared segment index exceeds uint64")
            })?;
            if segment.segment_index != expected_index
                || segment.source_char_start != expected_char_start
                || segment.source_utf8_start != expected_utf8_start
                || segment.source_char_end <= segment.source_char_start
                || segment.source_char_end > total_chars
                || segment.source_utf8_end <= segment.source_utf8_start
                || segment.source_utf8_end > total_utf8
            {
                return Err(ContractError::InvalidFrontendData(
                    "prepared segments are not exact contiguous source intervals",
                ));
            }
            let byte_start = usize::try_from(segment.source_utf8_start).map_err(|_| {
                ContractError::InvalidFrontendData("segment UTF-8 start exceeds usize")
            })?;
            let byte_end = usize::try_from(segment.source_utf8_end).map_err(|_| {
                ContractError::InvalidFrontendData("segment UTF-8 end exceeds usize")
            })?;
            if !source_text.is_char_boundary(byte_start)
                || !source_text.is_char_boundary(byte_end)
                || source_text[..byte_start].chars().count() as u64 != segment.source_char_start
                || source_text[..byte_end].chars().count() as u64 != segment.source_char_end
                || source_text[byte_start..byte_end] != segment.prepared.source_text
            {
                return Err(ContractError::InvalidFrontendData(
                    "prepared segment source offsets do not select its exact source text",
                ));
            }
            for local in segment.prepared.progress.iter().skip(1) {
                progress.push(SourceProgress {
                    committed_text_tokens: total_prepared_tokens
                        .checked_add(local.committed_text_tokens)
                        .ok_or(ContractError::InvalidFrontendData(
                            "combined prepared token count overflowed",
                        ))?,
                    source_char_end: segment
                        .source_char_start
                        .checked_add(local.source_char_end)
                        .ok_or(ContractError::InvalidFrontendData(
                            "combined source character offset overflowed",
                        ))?,
                    source_utf8_end: segment
                        .source_utf8_start
                        .checked_add(local.source_utf8_end)
                        .ok_or(ContractError::InvalidFrontendData(
                            "combined source UTF-8 offset overflowed",
                        ))?,
                });
            }
            total_prepared_tokens = total_prepared_tokens
                .checked_add(segment.prepared.total_prepared_tokens())
                .ok_or(ContractError::InvalidFrontendData(
                    "combined prepared token count overflowed",
                ))?;
            expected_char_start = segment.source_char_end;
            expected_utf8_start = segment.source_utf8_end;
        }
        if expected_char_start != total_chars
            || expected_utf8_start != total_utf8
            || progress.last().is_none_or(|last| {
                last.committed_text_tokens != total_prepared_tokens
                    || last.source_char_end != total_chars
                    || last.source_utf8_end != total_utf8
            })
        {
            return Err(ContractError::InvalidFrontendData(
                "prepared segments do not span the exact logical source",
            ));
        }
        Ok(Self {
            source_text,
            segments,
            progress,
            total_prepared_tokens,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PlaybackCommand {
    Ignore,
    AdvanceAgentFloor(u64),
    AbortSystem(String),
    AbortAgent(String),
    SetPlaybackAvailable { kind: SpeechKind, available: bool },
}

pub fn parse_playback_control(
    action: u8,
    release_hold: u8,
    minimum_agent_epoch: u64,
    utterance_id: &str,
) -> Result<PlaybackCommand, ContractError> {
    const PAUSE: u8 = 1;
    const RESUME: u8 = 2;
    const DISCARD: u8 = 3;
    const ABORT_SYSTEM: u8 = 4;
    const ABORT_AGENT: u8 = 5;
    const NONE: u8 = 0;
    const USER: u8 = 1;
    const SYSTEM: u8 = 2;

    match action {
        PAUSE | RESUME => {
            if release_hold != NONE || minimum_agent_epoch != 0 || !utterance_id.is_empty() {
                return Err(ContractError::InvalidPlaybackControl);
            }
            Ok(PlaybackCommand::Ignore)
        }
        DISCARD => {
            if release_hold != USER && release_hold != SYSTEM {
                return Err(ContractError::InvalidPlaybackControl);
            }
            if (release_hold == USER && !utterance_id.is_empty())
                || (release_hold == SYSTEM && utterance_id.is_empty())
            {
                return Err(ContractError::InvalidPlaybackControl);
            }
            Ok(PlaybackCommand::AdvanceAgentFloor(minimum_agent_epoch))
        }
        ABORT_SYSTEM => {
            if utterance_id.is_empty()
                || minimum_agent_epoch != 0
                || (release_hold != NONE && release_hold != SYSTEM)
            {
                return Err(ContractError::InvalidPlaybackControl);
            }
            Ok(PlaybackCommand::AbortSystem(utterance_id.to_owned()))
        }
        ABORT_AGENT => {
            if utterance_id.is_empty()
                || minimum_agent_epoch != 0
                || release_hold != NONE
                || agent_floor_epoch(utterance_id).is_err()
            {
                return Err(ContractError::InvalidPlaybackControl);
            }
            Ok(PlaybackCommand::AbortAgent(utterance_id.to_owned()))
        }
        _ => Err(ContractError::InvalidPlaybackControl),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SubmitDisposition {
    Queued,
    Cancelled,
    Stale,
}

#[derive(Debug, Default)]
pub struct RequestQueue {
    agent: VecDeque<SayRequest>,
    system: VecDeque<SayRequest>,
    minimum_agent_epoch: u64,
    cancelled_system: HashSet<String>,
    cancelled_system_order: VecDeque<String>,
    in_flight_utterance_ids: HashSet<String>,
    recent_utterance_ids: HashSet<String>,
    recent_utterance_order: VecDeque<String>,
}

impl RequestQueue {
    pub fn submit(&mut self, request: SayRequest) -> Result<SubmitDisposition, ContractError> {
        let agent_epoch = match request.kind {
            SpeechKind::Agent => Some(agent_floor_epoch(&request.utterance_id)?),
            SpeechKind::System => None,
        };
        if self.in_flight_utterance_ids.contains(&request.utterance_id)
            || self.recent_utterance_ids.contains(&request.utterance_id)
        {
            return Err(ContractError::ReusedUtteranceId);
        }
        self.in_flight_utterance_ids
            .insert(request.utterance_id.clone());
        match request.kind {
            SpeechKind::Agent
                if agent_epoch.expect("agent kind was validated") < self.minimum_agent_epoch =>
            {
                self.finish_request(&request.utterance_id)?;
                Ok(SubmitDisposition::Stale)
            }
            SpeechKind::Agent => {
                self.agent.push_back(request);
                Ok(SubmitDisposition::Queued)
            }
            SpeechKind::System if self.cancelled_system.remove(&request.utterance_id) => {
                self.cancelled_system_order
                    .retain(|identifier| identifier != &request.utterance_id);
                self.finish_request(&request.utterance_id)?;
                Ok(SubmitDisposition::Cancelled)
            }
            SpeechKind::System => {
                self.system.push_back(request);
                Ok(SubmitDisposition::Queued)
            }
        }
    }

    pub fn take_next(&mut self) -> Option<SayRequest> {
        self.system.pop_front().or_else(|| self.agent.pop_front())
    }

    pub fn advance_agent_floor(&mut self, floor: u64) -> Vec<SayRequest> {
        self.minimum_agent_epoch = self.minimum_agent_epoch.max(floor);
        let mut cancelled = Vec::new();
        self.agent.retain(|request| {
            let keep = agent_floor_epoch(&request.utterance_id).expect("validated agent request")
                >= self.minimum_agent_epoch;
            if !keep {
                cancelled.push(request.clone());
            }
            keep
        });
        for request in &cancelled {
            self.finish_request(&request.utterance_id)
                .expect("queued request must be in flight");
        }
        cancelled
    }

    pub fn agent_is_stale(&self, request: &SayRequest) -> bool {
        request.kind == SpeechKind::Agent
            && agent_floor_epoch(&request.utterance_id).expect("validated agent request")
                < self.minimum_agent_epoch
    }

    pub fn abort_system(&mut self, utterance_id: &str, active_match: bool) -> Vec<SayRequest> {
        let mut cancelled = Vec::new();
        self.system.retain(|request| {
            let keep = request.utterance_id != utterance_id;
            if !keep {
                cancelled.push(request.clone());
            }
            keep
        });
        for request in &cancelled {
            self.finish_request(&request.utterance_id)
                .expect("queued request must be in flight");
        }
        if !active_match && cancelled.is_empty() {
            self.remember_system_abort(utterance_id);
        }
        cancelled
    }

    pub fn abort_agent(&mut self, utterance_id: &str) -> Vec<SayRequest> {
        let mut cancelled = Vec::new();
        self.agent.retain(|request| {
            let keep = request.utterance_id != utterance_id;
            if !keep {
                cancelled.push(request.clone());
            }
            keep
        });
        for request in &cancelled {
            self.finish_request(&request.utterance_id)
                .expect("queued request must be in flight");
        }
        cancelled
    }

    pub fn finish_request(&mut self, utterance_id: &str) -> Result<(), ContractError> {
        if !self.in_flight_utterance_ids.remove(utterance_id) {
            return Err(ContractError::UnknownInFlightUtteranceId);
        }
        if self.cancelled_system.remove(utterance_id) {
            self.cancelled_system_order
                .retain(|identifier| identifier != utterance_id);
        }
        self.remember_recent_utterance(utterance_id);
        Ok(())
    }

    pub fn pending_len(&self) -> usize {
        self.agent.len() + self.system.len()
    }

    pub fn cancel_kind(&mut self, kind: SpeechKind) -> Vec<SayRequest> {
        let mut cancelled = Vec::new();
        let queue = match kind {
            SpeechKind::Agent => &mut self.agent,
            SpeechKind::System => &mut self.system,
        };
        queue.drain(..).for_each(|request| cancelled.push(request));
        for request in &cancelled {
            self.finish_request(&request.utterance_id)
                .expect("queued request must be in flight");
        }
        cancelled
    }

    #[cfg(test)]
    fn tracked_utterance_id_count(&self) -> usize {
        self.in_flight_utterance_ids.len() + self.recent_utterance_ids.len()
    }

    fn remember_system_abort(&mut self, utterance_id: &str) {
        if self.cancelled_system.contains(utterance_id) {
            return;
        }
        if self.cancelled_system_order.len() >= MAX_SYSTEM_ABORT_TOMBSTONES {
            let expired = self
                .cancelled_system_order
                .pop_front()
                .expect("non-empty tombstone order");
            self.cancelled_system.remove(&expired);
        }
        self.cancelled_system.insert(utterance_id.to_owned());
        self.cancelled_system_order
            .push_back(utterance_id.to_owned());
    }

    fn remember_recent_utterance(&mut self, utterance_id: &str) {
        if self.recent_utterance_ids.contains(utterance_id) {
            return;
        }
        if self.recent_utterance_order.len() >= MAX_RECENT_UTTERANCE_IDS {
            let expired = self
                .recent_utterance_order
                .pop_front()
                .expect("non-empty recent utterance order");
            self.recent_utterance_ids.remove(&expired);
        }
        self.recent_utterance_ids.insert(utterance_id.to_owned());
        self.recent_utterance_order
            .push_back(utterance_id.to_owned());
    }
}

fn trim_python_whitespace(value: &str) -> String {
    value
        .trim_matches(|character: char| {
            character.is_whitespace() || matches!(character, '\u{001c}'..='\u{001f}' | '\u{0085}')
        })
        .to_owned()
}

#[derive(Debug, Error)]
pub enum ContractError {
    #[error("TTS kind must be AGENT or SYSTEM")]
    InvalidSpeechKind,
    #[error("TTS {0} must be a canonical lowercase UUID")]
    InvalidUuid(&'static str),
    #[error("TTS {0} must not be empty")]
    EmptySayField(&'static str),
    #[error("TTS utterance_id must not have leading or trailing whitespace")]
    SurroundingUtteranceIdWhitespace,
    #[error("TTS utterance_id must not contain NUL")]
    NulUtteranceId,
    #[error("TTS utterance_id must not exceed 256 UTF-8 bytes")]
    UtteranceIdTooLong,
    #[error("Japanese frontend request must not exceed one MiB including its LF terminator")]
    FrontendRequestTooLarge,
    #[error("cannot validate Japanese frontend request transport: {0}")]
    SerializeFrontendRequest(serde_json::Error),
    #[error("agent utterance_id must be agent-<floor_epoch>-<id>")]
    InvalidAgentUtteranceId,
    #[error("{0} must be a nonzero lowercase or uppercase SHA-256 hex digest")]
    InvalidSha256(&'static str),
    #[error("frontend returned invalid data: {0}")]
    InvalidFrontendData(&'static str),
    #[error("playback control fields violate the typed contract")]
    InvalidPlaybackControl,
    #[error("utterance_id is already in flight or still inside the bounded reorder window")]
    ReusedUtteranceId,
    #[error("scheduler tried to finish an utterance_id that is not in flight")]
    UnknownInFlightUtteranceId,
}

#[cfg(test)]
mod tests {
    use super::{
        PreparedRequest, PreparedSegment, PreparedUtterance, RequestQueue, SayRequest,
        SourceProgress, SpeechKind, SubmitDisposition, agent_floor_epoch, decode_sha256,
        deterministic_seed, parse_playback_control, validate_uuid,
    };

    fn request(kind: SpeechKind, identifier: &str) -> SayRequest {
        SayRequest {
            kind,
            utterance_id: identifier.to_owned(),
            text: "テスト。".to_owned(),
        }
    }

    #[test]
    fn say_request_is_strict_and_preserves_exact_source_text() {
        let parsed =
            SayRequest::from_fields(1, "agent-12-x".to_owned(), "　こんにちは。　".to_owned())
                .expect("valid typed request");
        assert_eq!(parsed.kind, SpeechKind::Agent);
        assert_eq!(parsed.utterance_id, "agent-12-x");
        assert_eq!(parsed.text, "　こんにちは。　");
        assert!(SayRequest::from_fields(3, "agent-12-x".to_owned(), "x".to_owned()).is_err());
        assert!(SayRequest::from_fields(1, " agent-12-x ".to_owned(), "x".to_owned(),).is_err());
        let oversized_id = format!("agent-12-{}", "あ".repeat(83));
        assert!(SayRequest::from_fields(1, oversized_id, "x".to_owned()).is_err());
        assert!(SayRequest::from_fields(1, "x".to_owned(), "x".to_owned()).is_err());
    }

    #[test]
    fn uuid_and_agent_epoch_are_exact() {
        assert_eq!(agent_floor_epoch("agent-42-a").expect("epoch"), 42);
        assert!(agent_floor_epoch("agent--1-a").is_err());
        validate_uuid("01234567-89ab-cdef-8123-456789abcdef", "generation_id")
            .expect("canonical UUID");
        assert!(validate_uuid("01234567-89AB-CDEF-8123-456789ABCDEF", "generation_id",).is_err());
    }

    #[test]
    fn deterministic_seed_is_stable_and_length_framed() {
        let manifest = decode_sha256(
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            "manifest",
        )
        .expect("digest");
        assert_eq!(
            deterministic_seed(&manifest, "agent-1-x", "こんにちは。"),
            deterministic_seed(&manifest, "agent-1-x", "こんにちは。")
        );
        assert_ne!(
            deterministic_seed(&manifest, "agent-1-x", "こんにちは。"),
            deterministic_seed(&manifest, "agent-1", "xこんにちは。")
        );
    }

    #[test]
    fn prepared_progress_must_cover_every_token_position_and_utf8_boundary() {
        let prepared = PreparedUtterance::validate(
            "あ。".to_owned(),
            vec![1, 3_358],
            vec![
                SourceProgress {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                SourceProgress {
                    committed_text_tokens: 1,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
                SourceProgress {
                    committed_text_tokens: 2,
                    source_char_end: 2,
                    source_utf8_end: 6,
                },
            ],
            3_357,
            3_358,
            8,
        )
        .expect("prepared");
        assert_eq!(prepared.total_prepared_tokens(), 2);
        assert!(
            PreparedUtterance::validate(
                "あ。".to_owned(),
                vec![3_358],
                vec![
                    SourceProgress {
                        committed_text_tokens: 0,
                        source_char_end: 0,
                        source_utf8_end: 0,
                    },
                    SourceProgress {
                        committed_text_tokens: 1,
                        source_char_end: 1,
                        source_utf8_end: 1,
                    },
                ],
                3_357,
                3_358,
                8,
            )
            .is_err()
        );
    }

    #[test]
    fn prepared_tokens_require_one_final_eos_and_only_normal_rows_before_it() {
        fn progress(token_count: u64) -> Vec<SourceProgress> {
            (0..=token_count)
                .map(|committed_text_tokens| SourceProgress {
                    committed_text_tokens,
                    source_char_end: if committed_text_tokens == 0 { 0 } else { 1 },
                    source_utf8_end: if committed_text_tokens == 0 { 0 } else { 3 },
                })
                .collect()
        }

        PreparedUtterance::validate(
            "あ".to_owned(),
            vec![1, 3_358],
            progress(2),
            3_357,
            3_358,
            8,
        )
        .expect("normal tokenizer row followed by exact EOS");

        for (case, rejected) in [
            ("missing final EOS", vec![1, 2]),
            ("interior BOS", vec![3_357, 3_358]),
            ("interior EOS", vec![3_358, 1, 3_358]),
            (
                "unauthenticated embedding-or-higher row",
                vec![3_359, 3_358],
            ),
        ] {
            let token_count = rejected.len() as u64;
            assert!(
                PreparedUtterance::validate(
                    "あ".to_owned(),
                    rejected,
                    progress(token_count),
                    3_357,
                    3_358,
                    8,
                )
                .is_err(),
                "{case} must fail closed"
            );
        }
    }

    #[test]
    fn prepared_segments_become_one_exact_global_source_and_token_map() {
        let first = PreparedUtterance::validate(
            "一。".to_owned(),
            vec![11, 3_358],
            vec![
                SourceProgress {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                SourceProgress {
                    committed_text_tokens: 1,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
                SourceProgress {
                    committed_text_tokens: 2,
                    source_char_end: 2,
                    source_utf8_end: 6,
                },
            ],
            3_357,
            3_358,
            8,
        )
        .expect("first segment");
        let second = PreparedUtterance::validate(
            "二。".to_owned(),
            vec![21, 22, 3_358],
            vec![
                SourceProgress {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                SourceProgress {
                    committed_text_tokens: 1,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
                SourceProgress {
                    committed_text_tokens: 2,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
                SourceProgress {
                    committed_text_tokens: 3,
                    source_char_end: 2,
                    source_utf8_end: 6,
                },
            ],
            3_357,
            3_358,
            8,
        )
        .expect("second segment");
        let prepared = PreparedRequest::from_segments(
            "一。二。".to_owned(),
            vec![
                PreparedSegment {
                    segment_index: 0,
                    source_char_start: 0,
                    source_char_end: 2,
                    source_utf8_start: 0,
                    source_utf8_end: 6,
                    prepared: first,
                },
                PreparedSegment {
                    segment_index: 1,
                    source_char_start: 2,
                    source_char_end: 4,
                    source_utf8_start: 6,
                    source_utf8_end: 12,
                    prepared: second,
                },
            ],
        )
        .expect("combined request");

        assert_eq!(prepared.total_prepared_tokens, 5);
        assert_eq!(
            prepared.progress,
            vec![
                SourceProgress {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                SourceProgress {
                    committed_text_tokens: 1,
                    source_char_end: 1,
                    source_utf8_end: 3,
                },
                SourceProgress {
                    committed_text_tokens: 2,
                    source_char_end: 2,
                    source_utf8_end: 6,
                },
                SourceProgress {
                    committed_text_tokens: 3,
                    source_char_end: 3,
                    source_utf8_end: 9,
                },
                SourceProgress {
                    committed_text_tokens: 4,
                    source_char_end: 3,
                    source_utf8_end: 9,
                },
                SourceProgress {
                    committed_text_tokens: 5,
                    source_char_end: 4,
                    source_utf8_end: 12,
                },
            ]
        );
    }

    #[test]
    fn prepared_segments_reject_a_gap_in_the_logical_source() {
        let prepared = PreparedUtterance::validate(
            "二。".to_owned(),
            vec![3_358],
            vec![
                SourceProgress {
                    committed_text_tokens: 0,
                    source_char_end: 0,
                    source_utf8_end: 0,
                },
                SourceProgress {
                    committed_text_tokens: 1,
                    source_char_end: 2,
                    source_utf8_end: 6,
                },
            ],
            3_357,
            3_358,
            8,
        )
        .expect("local segment");
        let result = PreparedRequest::from_segments(
            "一。二。".to_owned(),
            vec![PreparedSegment {
                segment_index: 0,
                source_char_start: 2,
                source_char_end: 4,
                source_utf8_start: 6,
                source_utf8_end: 12,
                prepared,
            }],
        );
        assert!(result.is_err());
    }

    #[test]
    fn system_overtakes_agent_and_floor_cancels_only_stale_agents() {
        let mut queue = RequestQueue::default();
        assert_eq!(
            queue
                .submit(request(SpeechKind::Agent, "agent-1-a"))
                .expect("submit"),
            SubmitDisposition::Queued
        );
        assert_eq!(
            queue
                .submit(request(SpeechKind::System, "system-ready"))
                .expect("submit"),
            SubmitDisposition::Queued
        );
        assert_eq!(
            queue.take_next().expect("next").utterance_id,
            "system-ready"
        );
        let cancelled = queue.advance_agent_floor(2);
        assert_eq!(cancelled[0].utterance_id, "agent-1-a");
        assert_eq!(
            queue
                .submit(request(SpeechKind::Agent, "agent-1-b"))
                .expect("submit"),
            SubmitDisposition::Stale
        );
        assert_eq!(
            queue
                .submit(request(SpeechKind::Agent, "agent-2-c"))
                .expect("submit"),
            SubmitDisposition::Queued
        );
    }

    #[test]
    fn system_abort_tombstone_cancels_a_late_request_once() {
        let mut queue = RequestQueue::default();
        assert!(queue.abort_system("system-late", false).is_empty());
        assert_eq!(
            queue
                .submit(request(SpeechKind::System, "system-late"))
                .expect("submit"),
            SubmitDisposition::Cancelled
        );
        assert!(
            queue
                .submit(request(SpeechKind::System, "system-late"))
                .is_err()
        );
    }

    #[test]
    fn cancel_kind_removes_only_that_playback_stream() {
        let mut queue = RequestQueue::default();
        queue
            .submit(request(SpeechKind::Agent, "agent-1-a"))
            .expect("agent");
        queue
            .submit(request(SpeechKind::System, "system-a"))
            .expect("system");

        let cancelled = queue.cancel_kind(SpeechKind::Agent);
        assert_eq!(cancelled.len(), 1);
        assert_eq!(cancelled[0].utterance_id, "agent-1-a");
        assert_eq!(
            queue.take_next().expect("remaining system").utterance_id,
            "system-a"
        );
    }

    #[test]
    fn playback_control_validation_does_not_guess_missing_fields() {
        assert_eq!(
            parse_playback_control(3, 1, 9, "").expect("discard"),
            super::PlaybackCommand::AdvanceAgentFloor(9)
        );
        assert!(parse_playback_control(3, 0, 9, "").is_err());
        assert!(parse_playback_control(4, 0, 0, "").is_err());
        assert_eq!(
            parse_playback_control(5, 0, 0, "agent-9-rejected").expect("agent abort"),
            super::PlaybackCommand::AbortAgent("agent-9-rejected".to_owned())
        );
        assert!(parse_playback_control(5, 1, 0, "agent-9-rejected").is_err());
        assert!(parse_playback_control(5, 0, 0, "not-an-agent-id").is_err());
    }

    #[test]
    fn targeted_agent_abort_removes_only_the_named_request() {
        let mut queue = RequestQueue::default();
        queue
            .submit(request(SpeechKind::Agent, "agent-9-rejected"))
            .expect("rejected agent");
        queue
            .submit(request(SpeechKind::Agent, "agent-9-survivor"))
            .expect("surviving agent");

        let cancelled = queue.abort_agent("agent-9-rejected");
        assert_eq!(cancelled.len(), 1);
        assert_eq!(cancelled[0].utterance_id, "agent-9-rejected");
        assert_eq!(
            queue.take_next().expect("remaining agent").utterance_id,
            "agent-9-survivor"
        );
    }

    #[test]
    fn completed_id_tracking_is_bounded_for_an_always_on_node() {
        let mut queue = RequestQueue::default();
        for index in 0..(super::MAX_RECENT_UTTERANCE_IDS + 64) {
            let identifier = format!("system-{index}");
            queue
                .submit(request(SpeechKind::System, &identifier))
                .expect("unique request");
            let active = queue.take_next().expect("queued request");
            queue
                .finish_request(&active.utterance_id)
                .expect("finish live request");
            assert!(
                queue.tracked_utterance_id_count() <= super::MAX_RECENT_UTTERANCE_IDS,
                "terminal IDs must not grow for the process lifetime"
            );
        }
    }
}
