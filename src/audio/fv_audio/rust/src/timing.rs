use std::collections::{HashMap, VecDeque};
use std::fs::{self, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub const TIMING_SCHEMA_VERSION: u32 = 2;
pub const TIMING_RECEIPT_SCHEMA_VERSION: u32 = 1;
pub const REQUEST_ACCEPTED: u8 = 1;
pub const FRONTEND_COMPLETED: u8 = 2;
pub const NATIVE_REQUEST_STARTED: u8 = 3;
pub const FIRST_NATIVE_AUDIO: u8 = 4;
pub const FIRST_ROS_AUDIO_PUBLISHED: u8 = 5;
pub const FIRST_PLAYBACK_FRAME_PUBLISHED: u8 = 6;
pub const PHYSICAL_PLAYBACK_STARTED: u8 = 7;
pub const PHYSICAL_PLAYBACK_ENDED: u8 = 8;
pub const PLAYBACK_UNDERRUN: u8 = 9;
const REQUIRED_STAGE_COUNT: usize = 8;
const AGENT: u8 = 1;
const SYSTEM: u8 = 2;
const MAX_UTTERANCE_ID_BYTES: usize = 256;
const MAX_RECEIPT_LIMIT: usize = 4_096;
const MAX_RECEIPT_FILE_BYTES: u64 = 16 * 1024 * 1024;

#[derive(Clone, Debug, Eq, Hash, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingIdentity {
    pub kind: u8,
    pub request_id: String,
    pub generation_id: String,
    pub utterance_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingEvent {
    pub version: u32,
    pub stage: u8,
    pub identity: TimingIdentity,
    pub monotonic_time_ns: u64,
    pub underrun_frames: u64,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingReceipt {
    pub version: u32,
    pub kind: u8,
    pub request_id: String,
    pub generation_id: String,
    pub utterance_id: String,
    pub request_accepted_ns: u64,
    pub frontend_completed_ns: u64,
    pub native_request_started_ns: u64,
    pub first_native_audio_ns: u64,
    pub first_ros_audio_published_ns: u64,
    pub first_playback_frame_published_ns: u64,
    pub physical_playback_started_ns: u64,
    pub physical_playback_ended_ns: u64,
    pub total_underrun_frames: u64,
}

impl TimingReceipt {
    pub fn validate(&self) -> Result<(), String> {
        if self.version != TIMING_RECEIPT_SCHEMA_VERSION {
            return Err("TTS timing receipt schema version is unsupported".to_owned());
        }
        validate_identity(&TimingIdentity {
            kind: self.kind,
            request_id: self.request_id.clone(),
            generation_id: self.generation_id.clone(),
            utterance_id: self.utterance_id.clone(),
        })?;
        let stages = self.stage_times();
        if stages.contains(&0) {
            return Err("TTS timing receipt contains a zero timestamp".to_owned());
        }
        if stages.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err("TTS timing receipt timestamps are not strictly increasing".to_owned());
        }
        Ok(())
    }

    pub fn streaming_playback_accepted(&self) -> bool {
        self.total_underrun_frames == 0
    }

    fn stage_times(&self) -> [u64; REQUIRED_STAGE_COUNT] {
        [
            self.request_accepted_ns,
            self.frontend_completed_ns,
            self.native_request_started_ns,
            self.first_native_audio_ns,
            self.first_ros_audio_published_ns,
            self.first_playback_frame_published_ns,
            self.physical_playback_started_ns,
            self.physical_playback_ended_ns,
        ]
    }
}

#[derive(Clone, Debug)]
struct TimingChain {
    identity: TimingIdentity,
    stages: [Option<u64>; REQUIRED_STAGE_COUNT],
    first_observed_time_ns: u64,
    last_underrun_time_ns: Option<u64>,
    total_underrun_frames: u64,
}

impl TimingChain {
    fn new(event: &TimingEvent) -> Self {
        Self {
            identity: event.identity.clone(),
            stages: [None; REQUIRED_STAGE_COUNT],
            first_observed_time_ns: event.monotonic_time_ns,
            last_underrun_time_ns: None,
            total_underrun_frames: 0,
        }
    }

    fn record(&mut self, event: &TimingEvent) -> Result<Option<TimingReceipt>, String> {
        if event.identity != self.identity {
            return Err(format!(
                "TTS timing identity drifted for request {}",
                self.identity.request_id
            ));
        }
        if event.stage == PLAYBACK_UNDERRUN {
            return self.record_underrun(event).map(|()| None);
        }
        let stage_index = usize::from(event.stage)
            .checked_sub(1)
            .filter(|index| *index < REQUIRED_STAGE_COUNT)
            .ok_or_else(|| "TTS timing stage is invalid".to_owned())?;
        if event.underrun_frames != 0 {
            return Err("required TTS timing stage carries underrun frames".to_owned());
        }
        if self.stages[stage_index].is_some() {
            return Err(format!(
                "TTS timing stage {} is duplicated for request {}",
                event.stage, self.identity.request_id
            ));
        }
        for earlier in self.stages[..stage_index].iter().flatten() {
            if *earlier >= event.monotonic_time_ns {
                return Err(format!(
                    "TTS timing stage {} moved backwards for request {}",
                    event.stage, self.identity.request_id
                ));
            }
        }
        for later in self.stages[stage_index + 1..].iter().flatten() {
            if event.monotonic_time_ns >= *later {
                return Err(format!(
                    "TTS timing stage {} reverses an observed later stage for request {}",
                    event.stage, self.identity.request_id
                ));
            }
        }
        if event.stage == PHYSICAL_PLAYBACK_STARTED
            && self
                .last_underrun_time_ns
                .is_some_and(|timestamp| timestamp <= event.monotonic_time_ns)
        {
            return Err("TTS underrun does not follow physical playback start".to_owned());
        }
        if event.stage == PHYSICAL_PLAYBACK_ENDED
            && self
                .last_underrun_time_ns
                .is_some_and(|timestamp| timestamp >= event.monotonic_time_ns)
        {
            return Err("TTS underrun does not precede physical playback end".to_owned());
        }
        self.stages[stage_index] = Some(event.monotonic_time_ns);
        if self.stages.iter().any(Option::is_none) {
            return Ok(None);
        }
        self.build_receipt().map(Some)
    }

    fn record_underrun(&mut self, event: &TimingEvent) -> Result<(), String> {
        if event.underrun_frames == 0 {
            return Err("TTS underrun event must report positive frames".to_owned());
        }
        if self.stages[REQUIRED_STAGE_COUNT - 1].is_some() {
            return Err("TTS underrun arrived after physical playback end".to_owned());
        }
        if let Some(start) = self.stages[usize::from(PHYSICAL_PLAYBACK_STARTED) - 1] {
            if event.monotonic_time_ns <= start {
                return Err("TTS underrun does not follow physical playback start".to_owned());
            }
        }
        if self
            .last_underrun_time_ns
            .is_some_and(|previous| event.monotonic_time_ns <= previous)
        {
            return Err("TTS underrun timestamps did not increase".to_owned());
        }
        self.total_underrun_frames = self
            .total_underrun_frames
            .checked_add(event.underrun_frames)
            .ok_or_else(|| "TTS underrun frame total overflowed".to_owned())?;
        self.last_underrun_time_ns = Some(event.monotonic_time_ns);
        Ok(())
    }

    fn build_receipt(&self) -> Result<TimingReceipt, String> {
        let stage = |index: usize| {
            self.stages[index].ok_or_else(|| {
                format!(
                    "TTS timing chain ended with missing stage {} for request {}",
                    index + 1,
                    self.identity.request_id
                )
            })
        };
        let receipt = TimingReceipt {
            version: TIMING_RECEIPT_SCHEMA_VERSION,
            kind: self.identity.kind,
            request_id: self.identity.request_id.clone(),
            generation_id: self.identity.generation_id.clone(),
            utterance_id: self.identity.utterance_id.clone(),
            request_accepted_ns: stage(0)?,
            frontend_completed_ns: stage(1)?,
            native_request_started_ns: stage(2)?,
            first_native_audio_ns: stage(3)?,
            first_ros_audio_published_ns: stage(4)?,
            first_playback_frame_published_ns: stage(5)?,
            physical_playback_started_ns: stage(6)?,
            physical_playback_ended_ns: stage(7)?,
            total_underrun_frames: self.total_underrun_frames,
        };
        receipt.validate()?;
        Ok(receipt)
    }
}

pub struct TimingCollector {
    active_limit: usize,
    completed_limit: usize,
    timeout_ns: u64,
    active: HashMap<String, TimingChain>,
    completed: VecDeque<TimingIdentity>,
}

impl TimingCollector {
    pub fn new(
        active_limit: usize,
        completed_limit: usize,
        timeout_ns: u64,
        existing: impl IntoIterator<Item = TimingReceipt>,
    ) -> Result<Self, String> {
        if active_limit == 0 || completed_limit == 0 || timeout_ns == 0 {
            return Err("TTS timing collector limits must be positive".to_owned());
        }
        let mut collector = Self {
            active_limit,
            completed_limit,
            timeout_ns,
            active: HashMap::new(),
            completed: VecDeque::new(),
        };
        for receipt in existing {
            receipt.validate()?;
            collector.remember_completed(TimingIdentity {
                kind: receipt.kind,
                request_id: receipt.request_id,
                generation_id: receipt.generation_id,
                utterance_id: receipt.utterance_id,
            })?;
        }
        Ok(collector)
    }

    pub fn record(&mut self, event: TimingEvent) -> Result<Option<TimingReceipt>, String> {
        validate_event(&event)?;
        self.reject_identity_drift(&event.identity)?;
        if let Some(chain) = self.active.get_mut(&event.identity.request_id) {
            let receipt = chain.record(&event)?;
            if let Some(receipt) = receipt {
                self.active.remove(&event.identity.request_id);
                self.remember_completed(event.identity)?;
                return Ok(Some(receipt));
            }
            return Ok(None);
        }
        if self.active.len() >= self.active_limit {
            return Err("TTS timing active-chain bound exceeded".to_owned());
        }
        let request_id = event.identity.request_id.clone();
        self.active
            .insert(request_id.clone(), TimingChain::new(&event));
        let receipt = self
            .active
            .get_mut(&request_id)
            .ok_or_else(|| "TTS timing chain insertion failed".to_owned())?
            .record(&event)?;
        if let Some(receipt) = receipt {
            self.active.remove(&request_id);
            self.remember_completed(event.identity)?;
            return Ok(Some(receipt));
        }
        Ok(None)
    }

    pub fn check_timeouts(&self, now_ns: u64) -> Result<(), String> {
        if now_ns == 0 {
            return Err("TTS timing collector clock returned zero".to_owned());
        }
        for chain in self.active.values() {
            let elapsed = now_ns
                .checked_sub(chain.first_observed_time_ns)
                .ok_or_else(|| {
                    format!(
                        "TTS timing collector clock predates request {}",
                        chain.identity.request_id
                    )
                })?;
            if elapsed > self.timeout_ns {
                return Err(format!(
                    "TTS timing chain timed out for request {}",
                    chain.identity.request_id
                ));
            }
        }
        Ok(())
    }

    fn reject_identity_drift(&self, identity: &TimingIdentity) -> Result<(), String> {
        for known in self.active.values().map(|chain| &chain.identity) {
            if known.request_id == identity.request_id && known != identity {
                return Err(format!(
                    "TTS timing request identity drifted for {}",
                    identity.request_id
                ));
            }
            if known.utterance_id == identity.utterance_id && known != identity {
                return Err(format!(
                    "TTS timing utterance identity drifted for {}",
                    identity.utterance_id
                ));
            }
        }
        for known in &self.completed {
            if known.request_id == identity.request_id && known != identity {
                return Err(format!(
                    "TTS timing request identity drifted for {}",
                    identity.request_id
                ));
            }
            if known.utterance_id == identity.utterance_id && known != identity {
                return Err(format!(
                    "TTS timing utterance identity drifted for {}",
                    identity.utterance_id
                ));
            }
            if known == identity {
                return Err(format!(
                    "completed TTS timing identity was replayed for request {}",
                    identity.request_id
                ));
            }
        }
        Ok(())
    }

    fn remember_completed(&mut self, identity: TimingIdentity) -> Result<(), String> {
        if self.completed.iter().any(|known| {
            known.request_id == identity.request_id || known.utterance_id == identity.utterance_id
        }) {
            return Err("TTS timing receipt store contains duplicate identity".to_owned());
        }
        self.completed.push_back(identity);
        while self.completed.len() > self.completed_limit {
            self.completed.pop_front();
        }
        Ok(())
    }
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ReceiptFile {
    schema_version: u32,
    max_receipts: usize,
    receipts: Vec<TimingReceipt>,
}

pub struct TimingReceiptStore {
    path: PathBuf,
    max_receipts: usize,
    receipts: VecDeque<TimingReceipt>,
}

impl TimingReceiptStore {
    pub fn open(path: PathBuf, max_receipts: usize) -> Result<Self, String> {
        if !(1..=MAX_RECEIPT_LIMIT).contains(&max_receipts) {
            return Err(format!(
                "TTS timing receipt bound must be between 1 and {MAX_RECEIPT_LIMIT}"
            ));
        }
        validate_receipt_path(&path)?;
        let receipts = if let Some(bytes) = read_existing_receipt_file(&path)? {
            let file: ReceiptFile = serde_json::from_slice(&bytes).map_err(|error| {
                format!(
                    "failed to parse TTS timing receipt file {}: {error}",
                    path.display()
                )
            })?;
            if file.schema_version != TIMING_RECEIPT_SCHEMA_VERSION {
                return Err("TTS timing receipt file schema is unsupported".to_owned());
            }
            if file.max_receipts != max_receipts {
                return Err(format!(
                    "TTS timing receipt bound changed from {} to {max_receipts}",
                    file.max_receipts
                ));
            }
            if file.receipts.len() > max_receipts {
                return Err("TTS timing receipt file exceeds its declared bound".to_owned());
            }
            for receipt in &file.receipts {
                receipt.validate()?;
            }
            VecDeque::from(file.receipts)
        } else {
            VecDeque::new()
        };
        Ok(Self {
            path,
            max_receipts,
            receipts,
        })
    }

    pub fn existing(&self) -> impl Iterator<Item = TimingReceipt> + '_ {
        self.receipts.iter().cloned()
    }

    pub fn append(&mut self, receipt: TimingReceipt) -> Result<(), String> {
        receipt.validate()?;
        if self.receipts.iter().any(|known| {
            known.request_id == receipt.request_id || known.utterance_id == receipt.utterance_id
        }) {
            return Err("TTS timing receipt identity is duplicated".to_owned());
        }
        self.receipts.push_back(receipt);
        while self.receipts.len() > self.max_receipts {
            self.receipts.pop_front();
        }
        self.persist()
    }

    fn persist(&self) -> Result<(), String> {
        let parent = self
            .path
            .parent()
            .filter(|path| !path.as_os_str().is_empty())
            .ok_or_else(|| "TTS timing receipt path has no parent directory".to_owned())?;
        validate_receipt_parent(parent)?;
        let payload = ReceiptFile {
            schema_version: TIMING_RECEIPT_SCHEMA_VERSION,
            max_receipts: self.max_receipts,
            receipts: self.receipts.iter().cloned().collect(),
        };
        let bytes = serde_json::to_vec_pretty(&payload)
            .map_err(|error| format!("failed to serialize TTS timing receipt file: {error}"))?;
        let temporary = temporary_path(&self.path);
        let mut file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .mode(0o600)
            .open(&temporary)
            .map_err(|error| {
                format!(
                    "failed to create TTS timing receipt temporary file {}: {error}",
                    temporary.display()
                )
            })?;
        validate_receipt_file_metadata(&file.metadata().map_err(|error| {
            format!(
                "failed to inspect TTS timing receipt temporary file {}: {error}",
                temporary.display()
            )
        })?)?;
        file.write_all(&bytes).map_err(|error| {
            format!(
                "failed to write TTS timing receipt temporary file {}: {error}",
                temporary.display()
            )
        })?;
        file.write_all(b"\n").map_err(|error| {
            format!(
                "failed to terminate TTS timing receipt temporary file {}: {error}",
                temporary.display()
            )
        })?;
        file.sync_all().map_err(|error| {
            format!(
                "failed to sync TTS timing receipt temporary file {}: {error}",
                temporary.display()
            )
        })?;
        drop(file);
        validate_receipt_parent(parent)?;
        match fs::symlink_metadata(&self.path) {
            Ok(metadata) => validate_receipt_file_metadata(&metadata)?,
            Err(error) if error.kind() == ErrorKind::NotFound => {}
            Err(error) => {
                return Err(format!(
                    "failed to inspect TTS timing receipt target {}: {error}",
                    self.path.display()
                ));
            }
        }
        fs::rename(&temporary, &self.path).map_err(|error| {
            format!(
                "failed to atomically replace TTS timing receipt file {}: {error}",
                self.path.display()
            )
        })?;
        validate_receipt_file_metadata(&fs::symlink_metadata(&self.path).map_err(|error| {
            format!(
                "failed to inspect replaced TTS timing receipt file {}: {error}",
                self.path.display()
            )
        })?)?;
        let directory = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(parent)
            .map_err(|error| {
                format!(
                    "failed to open TTS timing receipt directory {}: {error}",
                    parent.display()
                )
            })?;
        directory.sync_all().map_err(|error| {
            format!(
                "failed to sync TTS timing receipt directory {}: {error}",
                parent.display()
            )
        })
    }
}

fn validate_receipt_path(path: &Path) -> Result<(), String> {
    if !path.is_absolute() {
        return Err("TTS timing receipt path must be absolute".to_owned());
    }
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .ok_or_else(|| "TTS timing receipt path has no parent directory".to_owned())?;
    if path.file_name().is_none() {
        return Err("TTS timing receipt path has no file name".to_owned());
    }
    validate_receipt_parent(parent)
}

fn validate_receipt_parent(parent: &Path) -> Result<(), String> {
    let canonical = fs::canonicalize(parent).map_err(|error| {
        format!(
            "failed to resolve TTS timing receipt directory {}: {error}",
            parent.display()
        )
    })?;
    if canonical != parent {
        return Err(format!(
            "TTS timing receipt directory must be canonical and contain no symlink: {}",
            parent.display()
        ));
    }
    let metadata = fs::symlink_metadata(parent).map_err(|error| {
        format!(
            "failed to inspect TTS timing receipt directory {}: {error}",
            parent.display()
        )
    })?;
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err("TTS timing receipt parent is not a real directory".to_owned());
    }
    if metadata.uid() != current_effective_uid() {
        return Err("TTS timing receipt directory is owned by another user".to_owned());
    }
    if metadata.mode() & 0o022 != 0 {
        return Err("TTS timing receipt directory is writable by group or other users".to_owned());
    }
    Ok(())
}

fn read_existing_receipt_file(path: &Path) -> Result<Option<Vec<u8>>, String> {
    let file = match OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)
    {
        Ok(file) => file,
        Err(error) if error.kind() == ErrorKind::NotFound => return Ok(None),
        Err(error) => {
            return Err(format!(
                "failed to open TTS timing receipt file {}: {error}",
                path.display()
            ));
        }
    };
    let metadata = file.metadata().map_err(|error| {
        format!(
            "failed to inspect TTS timing receipt file {}: {error}",
            path.display()
        )
    })?;
    validate_receipt_file_metadata(&metadata)?;
    if metadata.len() > MAX_RECEIPT_FILE_BYTES {
        return Err(format!(
            "TTS timing receipt file exceeds {MAX_RECEIPT_FILE_BYTES} bytes"
        ));
    }
    let mut bytes = Vec::new();
    file.take(MAX_RECEIPT_FILE_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|error| {
            format!(
                "failed to read TTS timing receipt file {}: {error}",
                path.display()
            )
        })?;
    if u64::try_from(bytes.len())
        .map_err(|_| "TTS timing receipt file length exceeds uint64".to_owned())?
        > MAX_RECEIPT_FILE_BYTES
    {
        return Err(format!(
            "TTS timing receipt file exceeds {MAX_RECEIPT_FILE_BYTES} bytes"
        ));
    }
    Ok(Some(bytes))
}

fn validate_receipt_file_metadata(metadata: &fs::Metadata) -> Result<(), String> {
    if !metadata.file_type().is_file() || metadata.file_type().is_symlink() {
        return Err("TTS timing receipt is not a regular file".to_owned());
    }
    if metadata.uid() != current_effective_uid() {
        return Err("TTS timing receipt is owned by another user".to_owned());
    }
    if metadata.nlink() != 1 {
        return Err("TTS timing receipt must have exactly one hard link".to_owned());
    }
    if metadata.mode() & 0o777 != 0o600 {
        return Err("TTS timing receipt mode must be 0600".to_owned());
    }
    Ok(())
}

fn current_effective_uid() -> u32 {
    // SAFETY: geteuid has no arguments, dereferences no pointers, and has no
    // failure sentinel. It returns the effective owner used for filesystem
    // permission checks in this process.
    unsafe { libc::geteuid() }
}

fn temporary_path(path: &Path) -> PathBuf {
    let name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("tts-timing-receipts.json");
    path.with_file_name(format!(".{name}.{}.tmp", std::process::id()))
}

fn validate_event(event: &TimingEvent) -> Result<(), String> {
    if event.version != TIMING_SCHEMA_VERSION {
        return Err("TTS timing event schema version is unsupported".to_owned());
    }
    validate_identity(&event.identity)?;
    if event.monotonic_time_ns == 0 {
        return Err("TTS timing event timestamp is zero".to_owned());
    }
    if !(REQUEST_ACCEPTED..=PLAYBACK_UNDERRUN).contains(&event.stage) {
        return Err("TTS timing event stage is invalid".to_owned());
    }
    Ok(())
}

fn validate_identity(identity: &TimingIdentity) -> Result<(), String> {
    if !matches!(identity.kind, AGENT | SYSTEM) {
        return Err("TTS timing identity kind must be AGENT or SYSTEM".to_owned());
    }
    if !canonical_uuid(&identity.request_id) || !canonical_uuid(&identity.generation_id) {
        return Err("TTS timing identity contains a non-canonical UUID".to_owned());
    }
    if identity.utterance_id.trim().is_empty() {
        return Err("TTS timing utterance_id must not be empty".to_owned());
    }
    if identity.utterance_id.len() > MAX_UTTERANCE_ID_BYTES {
        return Err(format!(
            "TTS timing utterance_id exceeds {MAX_UTTERANCE_ID_BYTES} UTF-8 bytes"
        ));
    }
    Ok(())
}

fn canonical_uuid(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() == 36
        && matches!(bytes.get(8), Some(b'-'))
        && matches!(bytes.get(13), Some(b'-'))
        && matches!(bytes.get(18), Some(b'-'))
        && matches!(bytes.get(23), Some(b'-'))
        && bytes.iter().enumerate().all(|(index, byte)| {
            matches!(index, 8 | 13 | 18 | 23) || matches!(byte, b'0'..=b'9' | b'a'..=b'f')
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    const REQUEST: &str = "00000000-0000-4000-8000-000000000001";
    const GENERATION: &str = "00000000-0000-4000-8000-000000000002";

    fn event(stage: u8, timestamp: u64) -> TimingEvent {
        TimingEvent {
            version: TIMING_SCHEMA_VERSION,
            stage,
            identity: TimingIdentity {
                kind: AGENT,
                request_id: REQUEST.to_owned(),
                generation_id: GENERATION.to_owned(),
                utterance_id: "agent-timing-test".to_owned(),
            },
            monotonic_time_ns: timestamp,
            underrun_frames: 0,
        }
    }

    fn complete(collector: &mut TimingCollector) -> TimingReceipt {
        let mut receipt = None;
        for stage in REQUEST_ACCEPTED..=PHYSICAL_PLAYBACK_ENDED {
            receipt = collector
                .record(event(stage, u64::from(stage) * 10))
                .unwrap();
        }
        receipt.expect("physical end must complete the receipt")
    }

    #[test]
    fn completes_only_the_exact_strictly_ordered_chain() {
        let mut collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        let receipt = complete(&mut collector);
        assert_eq!(receipt.request_id, REQUEST);
        assert_eq!(receipt.physical_playback_ended_ns, 80);
        assert_eq!(receipt.total_underrun_frames, 0);
        assert!(receipt.streaming_playback_accepted());
    }

    #[test]
    fn rejects_unbounded_identity_and_receipt_configuration() {
        let mut collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        let mut oversized = event(REQUEST_ACCEPTED, 10);
        oversized.identity.utterance_id = "a".repeat(MAX_UTTERANCE_ID_BYTES + 1);
        assert!(collector
            .record(oversized)
            .unwrap_err()
            .contains("exceeds 256 UTF-8 bytes"));
        let invalid_bound_error = match TimingReceiptStore::open(
            PathBuf::from("/not-used-for-an-invalid-bound/receipts.json"),
            MAX_RECEIPT_LIMIT + 1,
        ) {
            Ok(_) => panic!("an unbounded receipt configuration was accepted"),
            Err(error) => error,
        };
        assert!(invalid_bound_error.contains("between 1 and 4096"));
    }

    #[test]
    fn retains_a_missing_chain_until_timeout_and_rejects_duplicate_and_reversed_stages() {
        let mut missing = TimingCollector::new(4, 8, 1_000, []).unwrap();
        missing.record(event(REQUEST_ACCEPTED, 10)).unwrap();
        assert_eq!(
            missing.record(event(FRONTEND_COMPLETED, 1_000)).unwrap(),
            None
        );
        assert!(missing
            .check_timeouts(1_011)
            .unwrap_err()
            .contains("timed out"));

        let mut duplicate = TimingCollector::new(4, 8, 1_000, []).unwrap();
        duplicate.record(event(REQUEST_ACCEPTED, 10)).unwrap();
        assert!(duplicate
            .record(event(REQUEST_ACCEPTED, 11))
            .unwrap_err()
            .contains("duplicated"));

        let mut reversed = TimingCollector::new(4, 8, 1_000, []).unwrap();
        reversed.record(event(REQUEST_ACCEPTED, 20)).unwrap();
        assert!(reversed
            .record(event(FRONTEND_COMPLETED, 10))
            .unwrap_err()
            .contains("backwards"));
    }

    #[test]
    fn completes_after_legal_cross_writer_reordering() {
        let mut collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        let arrival_order = [
            PHYSICAL_PLAYBACK_ENDED,
            NATIVE_REQUEST_STARTED,
            FRONTEND_COMPLETED,
            FIRST_NATIVE_AUDIO,
            FIRST_PLAYBACK_FRAME_PUBLISHED,
            FIRST_ROS_AUDIO_PUBLISHED,
            PHYSICAL_PLAYBACK_STARTED,
            REQUEST_ACCEPTED,
        ];
        let mut receipt = None;
        for stage in arrival_order {
            receipt = collector
                .record(event(stage, u64::from(stage) * 10))
                .unwrap();
        }
        assert_eq!(
            receipt
                .expect("the last missing stage completes the reordered chain")
                .physical_playback_ended_ns,
            80
        );
    }

    #[test]
    fn aggregates_repeatable_underruns_inside_the_physical_window() {
        let mut collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        for stage in REQUEST_ACCEPTED..=PHYSICAL_PLAYBACK_STARTED {
            collector
                .record(event(stage, u64::from(stage) * 10))
                .unwrap();
        }
        let mut first = event(PLAYBACK_UNDERRUN, 72);
        first.underrun_frames = 24;
        collector.record(first).unwrap();
        let mut second = event(PLAYBACK_UNDERRUN, 75);
        second.underrun_frames = 12;
        collector.record(second).unwrap();
        let receipt = collector
            .record(event(PHYSICAL_PLAYBACK_ENDED, 80))
            .unwrap()
            .expect("end completes receipt");
        assert_eq!(receipt.total_underrun_frames, 36);
        assert!(!receipt.streaming_playback_accepted());
    }

    #[test]
    fn retains_only_the_configured_number_of_receipts_on_disk() {
        use std::os::unix::fs::DirBuilderExt;

        let directory = std::env::temp_dir().join(format!(
            "fv-audio-timing-{}-{}",
            std::process::id(),
            REQUEST
        ));
        let _ = fs::remove_dir_all(&directory);
        let mut builder = fs::DirBuilder::new();
        builder.mode(0o700).create(&directory).unwrap();
        let path = directory.join("receipts.json");
        let _ = fs::remove_file(&path);
        let _ = fs::remove_file(temporary_path(&path));
        let mut store = TimingReceiptStore::open(path.clone(), 1).unwrap();
        let mut first_collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        store.append(complete(&mut first_collector)).unwrap();
        let mut second_collector = TimingCollector::new(4, 8, 1_000, []).unwrap();
        for stage in REQUEST_ACCEPTED..=PHYSICAL_PLAYBACK_ENDED {
            let mut value = event(stage, 100 + u64::from(stage) * 10);
            value.identity.request_id = "00000000-0000-4000-8000-000000000003".to_owned();
            value.identity.utterance_id = "agent-timing-test-2".to_owned();
            if let Some(receipt) = second_collector.record(value).unwrap() {
                store.append(receipt).unwrap();
            }
        }
        let reopened = TimingReceiptStore::open(path.clone(), 1).unwrap();
        let saved = reopened.existing().collect::<Vec<_>>();
        assert_eq!(saved.len(), 1);
        assert_eq!(saved[0].request_id, "00000000-0000-4000-8000-000000000003");
        fs::remove_file(path).unwrap();
        fs::remove_dir(directory).unwrap();
    }
}
