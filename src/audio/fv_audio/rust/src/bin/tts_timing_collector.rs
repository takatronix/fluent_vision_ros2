use std::env;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use fv_audio_ros2::timing::{
    TimingCollector, TimingEvent, TimingIdentity, TimingReceipt, TimingReceiptStore,
};
use r2r::fv_audio_interfaces::msg::{TtsTimingEvent, TtsTimingReceipt};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const TIMING_QOS_DEPTH: usize = 256;
const RECEIPT_QOS_DEPTH: usize = 64;
const TIMING_MESSAGES_PER_TICK: usize = 64;
const ACTIVE_CHAIN_LIMIT: usize = 64;
const DEFAULT_RECEIPT_LIMIT: usize = 256;
const DEFAULT_CHAIN_TIMEOUT_SECONDS: u64 = 180;

#[derive(Debug, Error)]
enum CollectorError {
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("TTS timing contract failed: {0}")]
    Contract(String),
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

fn main() -> Result<(), CollectorError> {
    let receipt_limit =
        env_positive_usize("FV_AUDIO_TTS_TIMING_RECEIPT_LIMIT", DEFAULT_RECEIPT_LIMIT)?;
    let timeout_seconds = env_positive_u64(
        "FV_AUDIO_TTS_TIMING_TIMEOUT_SECONDS",
        DEFAULT_CHAIN_TIMEOUT_SECONDS,
    )?;
    let receipt_path = env::var("FV_AUDIO_TTS_TIMING_RECEIPT_PATH")
        .map(PathBuf::from)
        .map_err(|_| {
            CollectorError::Config("FV_AUDIO_TTS_TIMING_RECEIPT_PATH is required".to_owned())
        })?;
    let timeout_ns = timeout_seconds
        .checked_mul(1_000_000_000)
        .ok_or_else(|| CollectorError::Config("TTS timing timeout overflowed".to_owned()))?;
    let mut store =
        TimingReceiptStore::open(receipt_path, receipt_limit).map_err(CollectorError::Contract)?;
    let mut collector = TimingCollector::new(
        ACTIVE_CHAIN_LIMIT,
        receipt_limit,
        timeout_ns,
        store.existing(),
    )
    .map_err(CollectorError::Contract)?;

    let context = Context::create()?;
    let mut node = Node::create(context, "tts_timing_collector", "")?;
    let event_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: TIMING_QOS_DEPTH,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    };
    let mut events = node.subscribe::<TtsTimingEvent>("/aspa/tts/timing", event_qos)?;
    let receipt_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: RECEIPT_QOS_DEPTH,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    };
    let receipts =
        node.create_publisher::<TtsTimingReceipt>("/aspa/tts/timing/receipt", receipt_qos)?;
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        for _ in 0..TIMING_MESSAGES_PER_TICK {
            let message = match events.next().now_or_never() {
                None => break,
                Some(Some(message)) => message,
                Some(None) => {
                    return Err(CollectorError::Contract(
                        "TTS timing DDS stream ended".to_owned(),
                    ));
                }
            };
            let event = TimingEvent {
                version: message.version,
                stage: message.stage,
                identity: TimingIdentity {
                    kind: message.kind,
                    request_id: message.request_id,
                    generation_id: message.generation_id,
                    utterance_id: message.utterance_id,
                },
                monotonic_time_ns: message.monotonic_time_ns,
                underrun_frames: message.underrun_frames,
            };
            if let Some(receipt) = collector.record(event).map_err(CollectorError::Contract)? {
                store
                    .append(receipt.clone())
                    .map_err(CollectorError::Contract)?;
                if !receipt.streaming_playback_accepted() {
                    eprintln!(
                        "TTS streaming playback acceptance failed for {}/{}/{}: {} underrun frame(s)",
                        receipt.request_id,
                        receipt.generation_id,
                        receipt.utterance_id,
                        receipt.total_underrun_frames
                    );
                }
                receipts.publish(&ros_receipt(receipt))?;
            }
        }
        collector
            .check_timeouts(monotonic_time_ns()?)
            .map_err(CollectorError::Contract)?;
    }
    Ok(())
}

fn ros_receipt(receipt: TimingReceipt) -> TtsTimingReceipt {
    TtsTimingReceipt {
        version: TtsTimingReceipt::TTS_TIMING_RECEIPT_SCHEMA_VERSION as u32,
        kind: receipt.kind,
        request_id: receipt.request_id,
        generation_id: receipt.generation_id,
        utterance_id: receipt.utterance_id,
        request_accepted_ns: receipt.request_accepted_ns,
        frontend_completed_ns: receipt.frontend_completed_ns,
        native_request_started_ns: receipt.native_request_started_ns,
        first_native_audio_ns: receipt.first_native_audio_ns,
        first_ros_audio_published_ns: receipt.first_ros_audio_published_ns,
        first_playback_frame_published_ns: receipt.first_playback_frame_published_ns,
        physical_playback_started_ns: receipt.physical_playback_started_ns,
        physical_playback_ended_ns: receipt.physical_playback_ended_ns,
        total_underrun_frames: receipt.total_underrun_frames,
    }
}

fn monotonic_time_ns() -> Result<u64, CollectorError> {
    let mut timestamp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: timestamp points to initialized writable storage and
    // CLOCK_MONOTONIC is a process-independent host clock.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut timestamp) } != 0 {
        return Err(CollectorError::Contract(format!(
            "CLOCK_MONOTONIC failed: {}",
            std::io::Error::last_os_error()
        )));
    }
    if timestamp.tv_sec < 0 || !(0..1_000_000_000).contains(&timestamp.tv_nsec) {
        return Err(CollectorError::Contract(
            "CLOCK_MONOTONIC returned an invalid timespec".to_owned(),
        ));
    }
    let seconds = u64::try_from(timestamp.tv_sec)
        .map_err(|_| CollectorError::Contract("monotonic seconds exceed uint64".to_owned()))?;
    let nanoseconds = u64::try_from(timestamp.tv_nsec)
        .map_err(|_| CollectorError::Contract("monotonic nanoseconds exceed uint64".to_owned()))?;
    seconds
        .checked_mul(1_000_000_000)
        .and_then(|value| value.checked_add(nanoseconds))
        .ok_or_else(|| CollectorError::Contract("monotonic timestamp overflowed".to_owned()))
}

fn env_positive_usize(name: &str, default: usize) -> Result<usize, CollectorError> {
    let value = env::var(name).map_or(Ok(default), |raw| {
        raw.parse()
            .map_err(|_| CollectorError::Config(format!("{name} must be an integer")))
    })?;
    if value == 0 {
        return Err(CollectorError::Config(format!("{name} must be positive")));
    }
    Ok(value)
}

fn env_positive_u64(name: &str, default: u64) -> Result<u64, CollectorError> {
    let value = env::var(name).map_or(Ok(default), |raw| {
        raw.parse()
            .map_err(|_| CollectorError::Config(format!("{name} must be an integer")))
    })?;
    if value == 0 {
        return Err(CollectorError::Config(format!("{name} must be positive")));
    }
    Ok(value)
}
