use std::fs;
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use magpie_tts_rt::ModelInfo;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::contracts::{
    ContractError, MAX_FRONTEND_REQUEST_BYTES, PreparedRequest, PreparedSegment, PreparedUtterance,
    SourceProgress, validate_frontend_request_transport,
};

const FRONTEND_SCHEMA_VERSION: u32 = 1;
const MAX_RESPONSE_BYTES: usize = 4 * 1024 * 1024;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrontendConfig {
    pub python_executable: PathBuf,
    pub server_script: PathBuf,
    pub oracle_lock: PathBuf,
    pub frontend_contract: PathBuf,
    pub response_timeout: Duration,
}

impl FrontendConfig {
    pub fn validate(&self) -> Result<(), FrontendError> {
        for (label, path, allow_symlink) in [
            ("frontend_python", self.python_executable.as_path(), true),
            ("frontend_server", self.server_script.as_path(), false),
            ("frontend_lock", self.oracle_lock.as_path(), false),
            ("frontend_contract", self.frontend_contract.as_path(), false),
        ] {
            require_regular_absolute_file(label, path, allow_symlink)?;
        }
        if self.response_timeout.is_zero() {
            return Err(FrontendError::InvalidConfiguration(
                "frontend response timeout must be positive".to_owned(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrontendIdentity {
    pub tokenizer_identity_sha256: [u8; 32],
    pub tokenizer_vocabulary_size: u32,
    pub text_embedding_rows: u32,
    pub bos_token_id: u32,
    pub eos_token_id: u32,
    pub japanese_global_pad_token_id: u32,
}

impl FrontendIdentity {
    pub const fn from_model_info(model: &ModelInfo) -> Self {
        Self {
            tokenizer_identity_sha256: model.tokenizer_identity_sha256,
            tokenizer_vocabulary_size: model.tokenizer_vocabulary_size,
            text_embedding_rows: model.text_embedding_rows,
            bos_token_id: model.bos_token_id,
            eos_token_id: model.eos_token_id,
            japanese_global_pad_token_id: model.japanese_global_pad_token_id,
        }
    }
}

#[derive(Debug, Serialize)]
struct FrontendRequest<'a> {
    schema_version: u32,
    request_id: &'a str,
    text: &'a str,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, tag = "type")]
enum FrontendResponse {
    #[serde(rename = "ready")]
    Ready {
        schema_version: u32,
        tokenizer_identity_sha256: String,
        tokenizer_vocabulary_size: u32,
        text_embedding_rows: u32,
        bos_token_id: u32,
        eos_token_id: u32,
        japanese_global_pad_token_id: u32,
    },
    #[serde(rename = "prepared_segments")]
    PreparedSegments {
        schema_version: u32,
        request_id: String,
        tokenizer_identity_sha256: String,
        source_text: String,
        segmentation_mode: String,
        segments: Vec<FrontendSegment>,
    },
    #[serde(rename = "error")]
    Error {
        schema_version: u32,
        request_id: String,
        error_code: String,
        message: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontendSegment {
    segment_index: u64,
    source_char_start: u64,
    source_char_end: u64,
    source_utf8_start: u64,
    source_utf8_end: u64,
    normalized_text: String,
    global_token_ids: Vec<i64>,
    progress: Vec<FrontendProgress>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct FrontendProgress {
    committed_text_tokens: u64,
    source_char_end: u64,
    source_utf8_end: u64,
}

impl From<FrontendProgress> for SourceProgress {
    fn from(value: FrontendProgress) -> Self {
        Self {
            committed_text_tokens: value.committed_text_tokens,
            source_char_end: value.source_char_end,
            source_utf8_end: value.source_utf8_end,
        }
    }
}

enum ReaderEvent {
    Line(String),
    Failed(String),
    Eof,
}

pub struct FrontendClient {
    identity: FrontendIdentity,
    input: BufWriter<ChildStdin>,
    responses: Receiver<ReaderEvent>,
    child: Child,
    reader: Option<JoinHandle<()>>,
    reader_overflowed: Arc<AtomicBool>,
    response_timeout: Duration,
}

impl FrontendClient {
    pub fn spawn(
        config: FrontendConfig,
        expected_model: &ModelInfo,
    ) -> Result<Self, FrontendError> {
        config.validate()?;
        let mut child = frontend_command(&config)
            .spawn()
            .map_err(FrontendError::Spawn)?;
        let input = child
            .stdin
            .take()
            .ok_or(FrontendError::MissingPipe("stdin"))?;
        let output = child
            .stdout
            .take()
            .ok_or(FrontendError::MissingPipe("stdout"))?;
        let (response_sender, responses) = mpsc::sync_channel(1);
        let reader_overflowed = Arc::new(AtomicBool::new(false));
        let thread_overflowed = Arc::clone(&reader_overflowed);
        let reader = thread::Builder::new()
            .name("magpie-japanese-frontend-stdout".to_owned())
            .spawn(move || {
                read_responses(BufReader::new(output), response_sender, thread_overflowed)
            })
            .map_err(FrontendError::ReaderSpawn)?;

        let mut client = Self {
            identity: FrontendIdentity {
                tokenizer_identity_sha256: [0; 32],
                tokenizer_vocabulary_size: 0,
                text_embedding_rows: 0,
                bos_token_id: 0,
                eos_token_id: 0,
                japanese_global_pad_token_id: 0,
            },
            input: BufWriter::new(input),
            responses,
            child,
            reader: Some(reader),
            reader_overflowed,
            response_timeout: config.response_timeout,
        };
        client.identity = validate_ready_response(client.receive()?, expected_model)?;
        Ok(client)
    }

    pub fn identity(&self) -> &FrontendIdentity {
        &self.identity
    }

    pub fn prepare(
        &mut self,
        request_id: &str,
        text: &str,
        maximum_text_tokens: u32,
    ) -> Result<PreparedRequest, FrontendError> {
        let encoded = encode_request(request_id, text)?;
        self.input
            .write_all(&encoded)
            .and_then(|()| self.input.flush())
            .map_err(FrontendError::Write)?;

        validate_prepared_response(
            self.receive()?,
            request_id,
            text,
            &self.identity,
            maximum_text_tokens,
        )
    }

    fn receive(&mut self) -> Result<FrontendResponse, FrontendError> {
        if self.reader_overflowed.load(Ordering::SeqCst) {
            return Err(FrontendError::Protocol(
                "frontend emitted more than one response for an outstanding request".to_owned(),
            ));
        }
        let event = match self.responses.recv_timeout(self.response_timeout) {
            Ok(event) => event,
            Err(RecvTimeoutError::Timeout) => return Err(FrontendError::Timeout),
            Err(RecvTimeoutError::Disconnected) => {
                return Err(FrontendError::Protocol(
                    "frontend stdout reader exited".to_owned(),
                ));
            }
        };
        if self.reader_overflowed.load(Ordering::SeqCst) {
            return Err(FrontendError::Protocol(
                "frontend emitted more than one response for an outstanding request".to_owned(),
            ));
        }
        match event {
            ReaderEvent::Line(line) => {
                serde_json::from_str(&line).map_err(FrontendError::InvalidResponse)
            }
            ReaderEvent::Failed(message) => Err(FrontendError::Protocol(message)),
            ReaderEvent::Eof => {
                let status = self.child.try_wait().map_err(FrontendError::Wait)?;
                Err(FrontendError::Protocol(format!(
                    "frontend process closed stdout with status {status:?}"
                )))
            }
        }
    }
}

fn validate_ready_response(
    response: FrontendResponse,
    expected_model: &ModelInfo,
) -> Result<FrontendIdentity, FrontendError> {
    let FrontendResponse::Ready {
        schema_version,
        tokenizer_identity_sha256,
        tokenizer_vocabulary_size,
        text_embedding_rows,
        bos_token_id,
        eos_token_id,
        japanese_global_pad_token_id,
    } = response
    else {
        return Err(FrontendError::Protocol(
            "first frontend response must be ready".to_owned(),
        ));
    };
    require_schema(schema_version)?;
    let actual = FrontendIdentity {
        tokenizer_identity_sha256: crate::contracts::decode_sha256(
            &tokenizer_identity_sha256,
            "frontend tokenizer identity",
        )
        .map_err(|error| FrontendError::Protocol(error.to_string()))?,
        tokenizer_vocabulary_size,
        text_embedding_rows,
        bos_token_id,
        eos_token_id,
        japanese_global_pad_token_id,
    };
    require_ready_identity(&actual, &FrontendIdentity::from_model_info(expected_model))?;
    Ok(actual)
}

fn require_ready_identity(
    actual: &FrontendIdentity,
    expected: &FrontendIdentity,
) -> Result<(), FrontendError> {
    if actual.tokenizer_identity_sha256 != expected.tokenizer_identity_sha256 {
        return Err(FrontendError::IdentityMismatch);
    }
    for (field, actual_value, expected_value) in [
        (
            "tokenizer_vocabulary_size",
            actual.tokenizer_vocabulary_size,
            expected.tokenizer_vocabulary_size,
        ),
        (
            "text_embedding_rows",
            actual.text_embedding_rows,
            expected.text_embedding_rows,
        ),
        ("bos_token_id", actual.bos_token_id, expected.bos_token_id),
        ("eos_token_id", actual.eos_token_id, expected.eos_token_id),
        (
            "japanese_global_pad_token_id",
            actual.japanese_global_pad_token_id,
            expected.japanese_global_pad_token_id,
        ),
    ] {
        if actual_value != expected_value {
            return Err(FrontendError::ReadyFieldMismatch {
                field,
                expected: expected_value,
                actual: actual_value,
            });
        }
    }
    Ok(())
}

fn validate_prepared_response(
    response: FrontendResponse,
    request_id: &str,
    text: &str,
    identity: &FrontendIdentity,
    maximum_text_tokens: u32,
) -> Result<PreparedRequest, FrontendError> {
    match response {
        FrontendResponse::PreparedSegments {
            schema_version,
            request_id: response_id,
            tokenizer_identity_sha256,
            source_text,
            segmentation_mode,
            segments,
        } => {
            require_schema(schema_version)?;
            if response_id != request_id {
                return Err(FrontendError::Protocol(
                    "prepared response request_id does not match the outstanding request"
                        .to_owned(),
                ));
            }
            let response_identity = crate::contracts::decode_sha256(
                &tokenizer_identity_sha256,
                "prepared tokenizer identity",
            )
            .map_err(|error| FrontendError::Protocol(error.to_string()))?;
            if response_identity != identity.tokenizer_identity_sha256 {
                return Err(FrontendError::IdentityMismatch);
            }
            if source_text != text {
                return Err(FrontendError::Protocol(
                    "frontend changed the exact source_text".to_owned(),
                ));
            }
            if segmentation_mode != "independent_sentence_segments_v1" {
                return Err(FrontendError::Protocol(
                    "frontend returned an unsupported segmentation mode".to_owned(),
                ));
            }
            let mut prepared_segments = Vec::with_capacity(segments.len());
            for segment in segments {
                let byte_start = usize::try_from(segment.source_utf8_start).map_err(|_| {
                    FrontendError::Protocol("segment source_utf8_start exceeds usize".to_owned())
                })?;
                let byte_end = usize::try_from(segment.source_utf8_end).map_err(|_| {
                    FrontendError::Protocol("segment source_utf8_end exceeds usize".to_owned())
                })?;
                if byte_start >= byte_end
                    || byte_end > source_text.len()
                    || !source_text.is_char_boundary(byte_start)
                    || !source_text.is_char_boundary(byte_end)
                    || segment.normalized_text.is_empty()
                {
                    return Err(FrontendError::Protocol(
                        "frontend returned an invalid segment source interval".to_owned(),
                    ));
                }
                let prepared = PreparedUtterance::validate(
                    source_text[byte_start..byte_end].to_owned(),
                    segment.global_token_ids,
                    segment.progress.into_iter().map(Into::into).collect(),
                    identity.tokenizer_vocabulary_size,
                    identity.eos_token_id,
                    maximum_text_tokens,
                )
                .map_err(|error| FrontendError::Protocol(error.to_string()))?;
                prepared_segments.push(PreparedSegment {
                    segment_index: segment.segment_index,
                    source_char_start: segment.source_char_start,
                    source_char_end: segment.source_char_end,
                    source_utf8_start: segment.source_utf8_start,
                    source_utf8_end: segment.source_utf8_end,
                    prepared,
                });
            }
            PreparedRequest::from_segments(source_text, prepared_segments)
                .map_err(|error| FrontendError::Protocol(error.to_string()))
        }
        FrontendResponse::Error {
            schema_version,
            request_id: response_id,
            error_code,
            message,
        } => {
            require_schema(schema_version)?;
            if response_id != request_id {
                return Err(FrontendError::Protocol(
                    "error response request_id does not match the outstanding request".to_owned(),
                ));
            }
            if error_code.is_empty() || message.is_empty() {
                return Err(FrontendError::Protocol(
                    "frontend error response requires error_code and message".to_owned(),
                ));
            }
            Err(FrontendError::Rejected {
                error_code,
                message,
            })
        }
        FrontendResponse::Ready { .. } => Err(FrontendError::Protocol(
            "frontend emitted a second ready response".to_owned(),
        )),
    }
}

fn frontend_command(config: &FrontendConfig) -> Command {
    let mut command = Command::new(&config.python_executable);
    command
        .arg("-I")
        .arg("-B")
        .arg(&config.server_script)
        .arg("--lock")
        .arg(&config.oracle_lock)
        .arg("--frontend-contract")
        .arg(&config.frontend_contract)
        .env("PYTHONNOUSERSITE", "1")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit());
    command
}

fn encode_request(request_id: &str, text: &str) -> Result<Vec<u8>, FrontendError> {
    validate_frontend_request_transport(request_id, text).map_err(|error| match error {
        ContractError::FrontendRequestTooLarge => FrontendError::RequestTooLarge,
        ContractError::EmptySayField(_)
        | ContractError::NulUtteranceId
        | ContractError::UtteranceIdTooLong => FrontendError::InvalidRequestId,
        ContractError::SerializeFrontendRequest(error) => FrontendError::SerializeRequest(error),
        other => FrontendError::InvalidTransport(other.to_string()),
    })?;
    let mut buffer = BoundedBuffer::new(MAX_FRONTEND_REQUEST_BYTES);
    let serialize_result = serde_json::to_writer(
        &mut buffer,
        &FrontendRequest {
            schema_version: FRONTEND_SCHEMA_VERSION,
            request_id,
            text,
        },
    );
    if buffer.exceeded {
        return Err(FrontendError::RequestTooLarge);
    }
    serialize_result.map_err(FrontendError::SerializeRequest)?;
    buffer
        .write_all(b"\n")
        .map_err(|_| FrontendError::RequestTooLarge)?;
    Ok(buffer.bytes)
}

struct BoundedBuffer {
    bytes: Vec<u8>,
    maximum_bytes: usize,
    exceeded: bool,
}

impl BoundedBuffer {
    fn new(maximum_bytes: usize) -> Self {
        Self {
            bytes: Vec::new(),
            maximum_bytes,
            exceeded: false,
        }
    }
}

impl Write for BoundedBuffer {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let remaining = self.maximum_bytes.saturating_sub(self.bytes.len());
        if bytes.len() > remaining {
            self.exceeded = true;
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "frontend request exceeds its bounded JSONL transport",
            ));
        }
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for FrontendClient {
    fn drop(&mut self) {
        drop(self.input.flush());
        match self.child.try_wait() {
            Ok(Some(_)) => {}
            Ok(None) | Err(_) => {
                let _ = self.child.kill();
            }
        }
        let _ = self.child.wait();
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

fn read_responses<R: Read>(
    mut reader: BufReader<R>,
    sender: mpsc::SyncSender<ReaderEvent>,
    overflowed: Arc<AtomicBool>,
) {
    loop {
        let mut bytes = Vec::new();
        let result = {
            let mut limited = reader.by_ref().take((MAX_RESPONSE_BYTES + 1) as u64);
            limited.read_until(b'\n', &mut bytes)
        };
        match result {
            Ok(0) => {
                send_reader_event(&sender, &overflowed, ReaderEvent::Eof);
                return;
            }
            Ok(_) if bytes.len() > MAX_RESPONSE_BYTES || bytes.last() != Some(&b'\n') => {
                send_reader_event(
                    &sender,
                    &overflowed,
                    ReaderEvent::Failed(
                        "frontend response exceeds the bounded JSONL transport".to_owned(),
                    ),
                );
                return;
            }
            Ok(_) => {
                bytes.pop();
                if bytes.last() == Some(&b'\r') {
                    bytes.pop();
                }
                let line = match String::from_utf8(bytes) {
                    Ok(line) => line,
                    Err(_) => {
                        send_reader_event(
                            &sender,
                            &overflowed,
                            ReaderEvent::Failed("frontend response is not UTF-8".to_owned()),
                        );
                        return;
                    }
                };
                if !send_reader_event(&sender, &overflowed, ReaderEvent::Line(line)) {
                    return;
                }
            }
            Err(error) => {
                send_reader_event(
                    &sender,
                    &overflowed,
                    ReaderEvent::Failed(format!("cannot read frontend stdout: {error}")),
                );
                return;
            }
        }
    }
}

fn send_reader_event(
    sender: &mpsc::SyncSender<ReaderEvent>,
    overflowed: &AtomicBool,
    event: ReaderEvent,
) -> bool {
    match sender.try_send(event) {
        Ok(()) => true,
        Err(mpsc::TrySendError::Full(_)) => {
            overflowed.store(true, Ordering::SeqCst);
            false
        }
        Err(mpsc::TrySendError::Disconnected(_)) => false,
    }
}

fn require_schema(schema_version: u32) -> Result<(), FrontendError> {
    if schema_version != FRONTEND_SCHEMA_VERSION {
        return Err(FrontendError::Protocol(
            "frontend schema_version must be 1".to_owned(),
        ));
    }
    Ok(())
}

fn require_regular_absolute_file(
    label: &str,
    path: &Path,
    allow_symlink: bool,
) -> Result<(), FrontendError> {
    if !path.is_absolute() {
        return Err(FrontendError::InvalidConfiguration(format!(
            "{label} must be an absolute path"
        )));
    }
    let metadata = fs::symlink_metadata(path).map_err(|error| {
        FrontendError::InvalidConfiguration(format!(
            "cannot inspect {label} {}: {error}",
            path.display()
        ))
    })?;
    if metadata.file_type().is_symlink() && allow_symlink {
        let target = fs::metadata(path).map_err(|error| {
            FrontendError::InvalidConfiguration(format!(
                "cannot inspect {label} target {}: {error}",
                path.display()
            ))
        })?;
        if target.is_file() {
            return Ok(());
        }
    }
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(FrontendError::InvalidConfiguration(format!(
            "{label} must be a regular non-symlink file: {}",
            path.display()
        )));
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum FrontendError {
    #[error("invalid Japanese frontend configuration: {0}")]
    InvalidConfiguration(String),
    #[error("cannot spawn Japanese frontend: {0}")]
    Spawn(std::io::Error),
    #[error("Japanese frontend did not expose its {0} pipe")]
    MissingPipe(&'static str),
    #[error("cannot start Japanese frontend stdout reader: {0}")]
    ReaderSpawn(std::io::Error),
    #[error("cannot serialize Japanese frontend request: {0}")]
    SerializeRequest(serde_json::Error),
    #[error("Japanese frontend request_id violates the 256-byte transport contract")]
    InvalidRequestId,
    #[error("Japanese frontend request exceeds the one-MiB JSONL transport contract")]
    RequestTooLarge,
    #[error("Japanese frontend request violates the shared transport contract: {0}")]
    InvalidTransport(String),
    #[error("cannot write Japanese frontend request: {0}")]
    Write(std::io::Error),
    #[error("Japanese frontend response timed out")]
    Timeout,
    #[error("Japanese frontend emitted invalid JSON: {0}")]
    InvalidResponse(serde_json::Error),
    #[error("Japanese frontend protocol failure: {0}")]
    Protocol(String),
    #[error("Japanese frontend tokenizer identity does not match the authenticated Magpie bundle")]
    IdentityMismatch,
    #[error(
        "Japanese frontend ready field {field} does not match the authenticated Magpie bundle: expected {expected}, got {actual}"
    )]
    ReadyFieldMismatch {
        field: &'static str,
        expected: u32,
        actual: u32,
    },
    #[error("Japanese frontend rejected the text ({error_code}): {message}")]
    Rejected { error_code: String, message: String },
    #[error("cannot inspect Japanese frontend process: {0}")]
    Wait(std::io::Error),
}

impl FrontendError {
    pub const fn is_process_fatal(&self) -> bool {
        !matches!(
            self,
            Self::Rejected { .. }
                | Self::InvalidRequestId
                | Self::RequestTooLarge
                | Self::InvalidTransport(_)
        )
    }
}

#[cfg(test)]
mod tests {
    use std::io::{BufReader, Cursor};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, mpsc};

    use magpie_tts_rt::ModelInfo;

    use super::{
        FrontendConfig, FrontendError, FrontendIdentity, FrontendResponse, ReaderEvent,
        encode_request, frontend_command, read_responses, require_ready_identity,
        validate_prepared_response, validate_ready_response,
    };
    use crate::contracts::MAX_FRONTEND_REQUEST_BYTES;

    fn model_info() -> ModelInfo {
        ModelInfo {
            tokenizer_vocabulary_size: 3_357,
            text_embedding_rows: 3_359,
            bos_token_id: 3_357,
            eos_token_id: 3_358,
            japanese_global_pad_token_id: 1_015,
            maximum_text_tokens: 512,
            maximum_audio_frames: 1_024,
            sample_rate_hz: 22_050,
            channels: 1,
            codec_frame_samples: 1_024,
            initial_frames: 4,
            steady_frames: 8,
            tail_min_frames: 1,
            tail_max_frames: 8,
            tokenizer_identity_sha256: [0xaa; 32],
        }
    }

    fn frontend_identity() -> FrontendIdentity {
        FrontendIdentity::from_model_info(&model_info())
    }

    #[test]
    fn response_schema_rejects_unknown_fields() {
        let response = r#"{"schema_version":1,"type":"ready","tokenizer_identity_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","tokenizer_vocabulary_size":3357,"text_embedding_rows":3359,"bos_token_id":3357,"eos_token_id":3358,"japanese_global_pad_token_id":1015,"extra":true}"#;
        assert!(serde_json::from_str::<FrontendResponse>(response).is_err());
    }

    #[test]
    fn response_schema_rejects_the_removed_vocabulary_size_field() {
        let response = r#"{"schema_version":1,"type":"ready","tokenizer_identity_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","vocabulary_size":3357,"text_embedding_rows":3359,"bos_token_id":3357,"eos_token_id":3358,"japanese_global_pad_token_id":1015}"#;
        assert!(serde_json::from_str::<FrontendResponse>(response).is_err());
    }

    #[test]
    fn ready_requires_every_authenticated_model_info_field() {
        let response = serde_json::from_str::<FrontendResponse>(
            r#"{
                "schema_version": 1,
                "type": "ready",
                "tokenizer_identity_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                "tokenizer_vocabulary_size": 3357,
                "text_embedding_rows": 3359,
                "bos_token_id": 3357,
                "eos_token_id": 3358,
                "japanese_global_pad_token_id": 1015
            }"#,
        )
        .expect("strict ready response");
        assert_eq!(
            validate_ready_response(response, &model_info()).expect("matching ready"),
            frontend_identity()
        );

        let expected = frontend_identity();
        let mut wrong_identity = expected.clone();
        wrong_identity.tokenizer_identity_sha256 = [0xbb; 32];
        assert!(matches!(
            require_ready_identity(&wrong_identity, &expected),
            Err(FrontendError::IdentityMismatch)
        ));

        let mut wrong_vocabulary = expected.clone();
        wrong_vocabulary.tokenizer_vocabulary_size += 1;
        let mut wrong_embedding_rows = expected.clone();
        wrong_embedding_rows.text_embedding_rows += 1;
        let mut wrong_bos = expected.clone();
        wrong_bos.bos_token_id += 1;
        let mut wrong_eos = expected.clone();
        wrong_eos.eos_token_id -= 1;
        let mut wrong_pad = expected.clone();
        wrong_pad.japanese_global_pad_token_id += 1;
        for candidate in [
            wrong_vocabulary,
            wrong_embedding_rows,
            wrong_bos,
            wrong_eos,
            wrong_pad,
        ] {
            assert!(matches!(
                require_ready_identity(&candidate, &expected),
                Err(FrontendError::ReadyFieldMismatch { .. })
            ));
        }
    }

    #[test]
    fn prepared_segments_json_becomes_one_validated_logical_request() {
        let response = serde_json::from_str::<FrontendResponse>(
            r#"{
                "schema_version": 1,
                "type": "prepared_segments",
                "request_id": "agent-1-long",
                "tokenizer_identity_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                "source_text": "一。二。",
                "segmentation_mode": "independent_sentence_segments_v1",
                "segments": [
                    {
                        "segment_index": 0,
                        "source_char_start": 0,
                        "source_char_end": 2,
                        "source_utf8_start": 0,
                        "source_utf8_end": 6,
                        "normalized_text": "一。",
                        "global_token_ids": [1, 3358],
                        "progress": [
                            {"committed_text_tokens": 0, "source_char_end": 0, "source_utf8_end": 0},
                            {"committed_text_tokens": 1, "source_char_end": 1, "source_utf8_end": 3},
                            {"committed_text_tokens": 2, "source_char_end": 2, "source_utf8_end": 6}
                        ]
                    },
                    {
                        "segment_index": 1,
                        "source_char_start": 2,
                        "source_char_end": 4,
                        "source_utf8_start": 6,
                        "source_utf8_end": 12,
                        "normalized_text": "二。",
                        "global_token_ids": [3, 3358],
                        "progress": [
                            {"committed_text_tokens": 0, "source_char_end": 0, "source_utf8_end": 0},
                            {"committed_text_tokens": 1, "source_char_end": 1, "source_utf8_end": 3},
                            {"committed_text_tokens": 2, "source_char_end": 2, "source_utf8_end": 6}
                        ]
                    }
                ]
            }"#,
        )
        .expect("strict prepared_segments JSON");
        let prepared = validate_prepared_response(
            response,
            "agent-1-long",
            "一。二。",
            &frontend_identity(),
            8,
        )
        .expect("validated logical request");

        assert_eq!(prepared.segments.len(), 2);
        assert_eq!(prepared.total_prepared_tokens, 4);
        assert_eq!(prepared.progress.last().expect("final").source_char_end, 4);
        assert_eq!(prepared.progress.last().expect("final").source_utf8_end, 12);
    }

    #[test]
    fn prepared_segments_reject_an_unrecognized_segmentation_mode() {
        let response = serde_json::from_str::<FrontendResponse>(
            r#"{
                "schema_version": 1,
                "type": "prepared_segments",
                "request_id": "agent-1-long",
                "tokenizer_identity_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                "source_text": "一。",
                "segmentation_mode": "stateful_history_v1",
                "segments": []
            }"#,
        )
        .expect("well-formed response");
        assert!(matches!(
            validate_prepared_response(response, "agent-1-long", "一。", &frontend_identity(), 8,),
            Err(FrontendError::Protocol(_))
        ));
    }

    #[test]
    fn stdout_reader_requires_utf8_lf_bounded_jsonl() {
        let (sender, receiver) = mpsc::sync_channel(1);
        let overflowed = Arc::new(AtomicBool::new(false));
        read_responses(
            BufReader::new(Cursor::new(b"{\"type\":\"ready\"}\n")),
            sender,
            Arc::clone(&overflowed),
        );
        match receiver.recv().expect("line") {
            ReaderEvent::Line(line) => assert_eq!(line, "{\"type\":\"ready\"}"),
            _ => panic!("expected line"),
        }

        let (sender, receiver) = mpsc::sync_channel(1);
        let overflowed = Arc::new(AtomicBool::new(false));
        read_responses(
            BufReader::new(Cursor::new(b"unterminated")),
            sender,
            overflowed,
        );
        assert!(matches!(
            receiver.recv().expect("failure"),
            ReaderEvent::Failed(_)
        ));
    }

    #[test]
    fn stdout_reader_fails_closed_when_the_bounded_response_slot_overflows() {
        let (sender, receiver) = mpsc::sync_channel(1);
        let overflowed = Arc::new(AtomicBool::new(false));
        read_responses(
            BufReader::new(Cursor::new(b"first\nsecond\n")),
            sender,
            Arc::clone(&overflowed),
        );
        assert!(matches!(
            receiver.recv().expect("first line"),
            ReaderEvent::Line(line) if line == "first"
        ));
        assert!(overflowed.load(Ordering::SeqCst));
    }

    #[test]
    fn request_encoder_matches_the_frontend_transport_bounds() {
        let encoded = encode_request("agent-1-a", "　こんにちは。　").expect("request");
        assert!(encoded.ends_with(b"\n"));
        assert!(encoded.len() <= MAX_FRONTEND_REQUEST_BYTES);

        assert!(matches!(
            encode_request(&"x".repeat(257), "test"),
            Err(FrontendError::InvalidRequestId)
        ));
        assert!(matches!(
            encode_request("agent-1-a", &"x".repeat(MAX_FRONTEND_REQUEST_BYTES)),
            Err(FrontendError::RequestTooLarge)
        ));
    }

    #[test]
    fn frontend_python_is_isolated_from_user_site_and_bytecode_writes() {
        let config = FrontendConfig {
            python_executable: "/opt/frontend/bin/python".into(),
            server_script: "/opt/frontend/server.py".into(),
            oracle_lock: "/opt/frontend/oracle-lock.json".into(),
            frontend_contract: "/opt/frontend/frontend-contract.json".into(),
            response_timeout: std::time::Duration::from_secs(1),
        };
        let command = frontend_command(&config);
        let arguments: Vec<_> = command
            .get_args()
            .map(|argument| argument.to_string_lossy().into_owned())
            .collect();
        assert_eq!(arguments[0..2], ["-I", "-B"]);
        assert_eq!(
            command
                .get_envs()
                .find(|(name, _)| *name == "PYTHONNOUSERSITE")
                .and_then(|(_, value)| value)
                .expect("PYTHONNOUSERSITE"),
            "1"
        );
    }
}
