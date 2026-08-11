use std::collections::VecDeque;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use libloading::Library;
use magpie_tts_rt::{
    GetApiFn, InferenceWorker, ModelInfo, OwnedAudioChunk, RuntimeConfig, SynthesisEvent,
    SynthesisStream, WorkerConfig, WorkerError,
};
use thiserror::Error;

use crate::contracts::{
    PlaybackCommand, PreparedRequest, PreparedSegment, RequestQueue, ResultStatus, SayRequest,
    SourceProgress, SpeechKind, SubmitDisposition, deterministic_seed,
};
use crate::frontend::{FrontendClient, FrontendConfig, FrontendIdentity};

const SAY_CHANNEL_CAPACITY: usize = 32;
const CONTROL_CHANNEL_CAPACITY: usize = 32;
const MAX_PENDING_REQUESTS: usize = 32;
const SCHEDULER_POLL_INTERVAL: Duration = Duration::from_millis(2);
const CODEC_FRAME_SAMPLES: usize = 1_024;
const INITIAL_CODEC_FRAMES: usize = 4;
const STEADY_CODEC_FRAMES: usize = 8;

#[derive(Clone, Debug)]
pub struct SchedulerConfig {
    pub native_library: PathBuf,
    pub bundle_path: PathBuf,
    pub expected_manifest_sha256: [u8; 32],
    pub cuda_device_index: i32,
    pub frontend: FrontendConfig,
    pub request_progress_timeout: Duration,
    pub cancellation_terminal_timeout: Duration,
    pub startup_timeout: Duration,
    pub scheduler_watchdog: Duration,
}

impl SchedulerConfig {
    fn validate(&self) -> Result<(), SchedulerError> {
        require_absolute_regular_file("native_library", &self.native_library)?;
        if !self.bundle_path.is_absolute() {
            return Err(SchedulerError::Configuration(
                "bundle_path must be absolute".to_owned(),
            ));
        }
        let metadata = fs::symlink_metadata(&self.bundle_path).map_err(|error| {
            SchedulerError::Configuration(format!(
                "cannot inspect bundle_path {}: {error}",
                self.bundle_path.display()
            ))
        })?;
        if metadata.file_type().is_symlink() || !metadata.is_dir() {
            return Err(SchedulerError::Configuration(format!(
                "bundle_path must be a non-symlink directory: {}",
                self.bundle_path.display()
            )));
        }
        if self.expected_manifest_sha256.iter().all(|byte| *byte == 0) {
            return Err(SchedulerError::Configuration(
                "expected_manifest_sha256 must not be zero".to_owned(),
            ));
        }
        if self.cuda_device_index < 0 {
            return Err(SchedulerError::Configuration(
                "cuda_device_index must be nonnegative".to_owned(),
            ));
        }
        if self.request_progress_timeout.is_zero()
            || self.cancellation_terminal_timeout.is_zero()
            || self.startup_timeout.is_zero()
            || self.scheduler_watchdog.is_zero()
        {
            return Err(SchedulerError::Configuration(
                "scheduler timeouts must be positive".to_owned(),
            ));
        }
        idle_heartbeat_interval(self.scheduler_watchdog)?;
        self.frontend
            .validate()
            .map_err(|error| SchedulerError::Configuration(error.to_string()))
    }
}

fn idle_heartbeat_interval(scheduler_watchdog: Duration) -> Result<Duration, SchedulerError> {
    let interval = (scheduler_watchdog / 4).min(Duration::from_secs(1));
    if interval.is_zero() {
        return Err(SchedulerError::Configuration(
            "scheduler_watchdog is too small to derive a positive idle heartbeat interval"
                .to_owned(),
        ));
    }
    Ok(interval)
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReadyInfo {
    pub model: ModelInfo,
    pub expected_manifest_sha256: [u8; 32],
    pub cuda_device_index: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignmentPoint {
    pub sample_index: u64,
    pub committed_text_tokens: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpeechEvent {
    Audio,
    Complete,
    Abort,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpeechOutput {
    pub kind: SpeechKind,
    pub event: SpeechEvent,
    pub utterance_id: String,
    pub sequence: u64,
    pub first_sample_index: u64,
    pub sample_rate_hz: u32,
    pub channels: u32,
    pub pcm_f32: Vec<f32>,
    pub final_chunk: bool,
    pub committed_text_tokens: u64,
    pub alignment_events: Vec<AlignmentPoint>,
    pub source_text: String,
    pub source_progress: Vec<SourceProgress>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResultOutput {
    pub kind: SpeechKind,
    pub utterance_id: String,
    pub status: ResultStatus,
    pub error: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimingStage {
    FrontendCompleted,
    NativeRequestStarted,
    FirstNativeAudio,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingOutput {
    pub kind: SpeechKind,
    pub utterance_id: String,
    pub stage: TimingStage,
    pub monotonic_time_ns: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SchedulerOutput {
    Audio(SpeechOutput),
    Timing(TimingOutput),
    Terminal {
        speech: SpeechOutput,
        result: ResultOutput,
    },
    Fatal(String),
}

pub struct SchedulerShutdown {
    pub outputs: Vec<SchedulerOutput>,
    pub result: Result<(), SchedulerError>,
}

pub struct SchedulerHandle {
    say: SyncSender<SayRequest>,
    control: SyncSender<PlaybackCommand>,
    wake: SyncSender<()>,
    output: Receiver<SchedulerOutput>,
    shutdown: Arc<AtomicBool>,
    heartbeat: Arc<Mutex<Instant>>,
    thread: Option<JoinHandle<Result<(), SchedulerError>>>,
}

impl SchedulerHandle {
    pub fn spawn(config: SchedulerConfig) -> Result<(Self, ReadyInfo), SchedulerError> {
        config.validate()?;
        let startup_timeout = config.startup_timeout;
        let (say_sender, say_receiver) = mpsc::sync_channel(SAY_CHANNEL_CAPACITY);
        let (control_sender, control_receiver) = mpsc::sync_channel(CONTROL_CHANNEL_CAPACITY);
        // Keep the payload channels separate so control traffic cannot be
        // starved by a full speech queue. This one-slot channel only wakes the
        // idle scheduler after either payload channel receives work.
        let (wake_sender, wake_receiver) = mpsc::sync_channel(1);
        let (output_sender, output_receiver) = mpsc::sync_channel(1);
        let (ready_sender, ready_receiver) = mpsc::sync_channel(1);
        let shutdown = Arc::new(AtomicBool::new(false));
        let thread_shutdown = Arc::clone(&shutdown);
        let heartbeat = Arc::new(Mutex::new(Instant::now()));
        let thread_heartbeat = Arc::clone(&heartbeat);
        let scheduler_thread = thread::Builder::new()
            .name("fv-tts-magpie-scheduler".to_owned())
            .spawn(move || {
                let result = run_scheduler(
                    config,
                    say_receiver,
                    control_receiver,
                    wake_receiver,
                    output_sender,
                    ready_sender.clone(),
                    thread_shutdown,
                    thread_heartbeat,
                );
                if let Err(error) = &result {
                    let _ = ready_sender.try_send(Err(error.to_string()));
                }
                result
            })
            .map_err(SchedulerError::ThreadSpawn)?;

        match ready_receiver.recv_timeout(startup_timeout) {
            Ok(Ok(ready)) => Ok((
                Self {
                    say: say_sender,
                    control: control_sender,
                    wake: wake_sender,
                    output: output_receiver,
                    shutdown,
                    heartbeat,
                    thread: Some(scheduler_thread),
                },
                ready,
            )),
            Ok(Err(message)) => {
                let _ = scheduler_thread.join();
                Err(SchedulerError::Startup(message))
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                shutdown.store(true, Ordering::SeqCst);
                // Session creation can be blocked in a native CUDA call. Do not
                // pretend the detached initializer was cleaned up; the node
                // caller must terminate this process so the OS releases it.
                Err(SchedulerError::StartupTimeout)
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let _ = scheduler_thread.join();
                Err(SchedulerError::Startup(
                    "scheduler exited without a readiness result".to_owned(),
                ))
            }
        }
    }

    pub fn try_submit(&self, request: SayRequest) -> Result<(), SchedulerSendError> {
        self.say.try_send(request).map_err(|error| match error {
            TrySendError::Full(_) => SchedulerSendError::SayQueueFull,
            TrySendError::Disconnected(_) => SchedulerSendError::Disconnected,
        })?;
        notify_scheduler(&self.wake)?;
        Ok(())
    }

    pub fn try_control(&self, control: PlaybackCommand) -> Result<(), SchedulerSendError> {
        self.control
            .try_send(control)
            .map_err(|error| match error {
                TrySendError::Full(_) => SchedulerSendError::ControlQueueFull,
                TrySendError::Disconnected(_) => SchedulerSendError::Disconnected,
            })?;
        notify_scheduler(&self.wake)?;
        Ok(())
    }

    pub fn try_output(&self) -> Result<Option<SchedulerOutput>, SchedulerError> {
        match self.output.try_recv() {
            Ok(output) => Ok(Some(output)),
            Err(TryRecvError::Empty) => Ok(None),
            Err(TryRecvError::Disconnected) => Err(SchedulerError::OutputDisconnected),
        }
    }

    pub fn heartbeat_age(&self) -> Result<Duration, SchedulerError> {
        let heartbeat = self
            .heartbeat
            .lock()
            .map_err(|_| SchedulerError::HeartbeatPoisoned)?;
        Ok(heartbeat.elapsed())
    }

    pub fn shutdown(mut self, timeout: Duration) -> SchedulerShutdown {
        let mut outputs = Vec::new();
        if timeout.is_zero() {
            return SchedulerShutdown {
                outputs,
                result: Err(SchedulerError::Configuration(
                    "shutdown timeout must be positive".to_owned(),
                )),
            };
        }
        self.shutdown.store(true, Ordering::SeqCst);
        let _ = notify_scheduler(&self.wake);
        let deadline = Instant::now() + timeout;
        loop {
            drain_available_outputs(&self.output, &mut outputs);
            if self
                .thread
                .as_ref()
                .is_none_or(std::thread::JoinHandle::is_finished)
            {
                let result = self.join();
                drain_available_outputs(&self.output, &mut outputs);
                return SchedulerShutdown { outputs, result };
            }
            if Instant::now() >= deadline {
                // Dropping a JoinHandle detaches it. The caller returns from
                // main immediately after this error, so process teardown is
                // the only truthful cleanup for a blocked native call.
                self.thread.take();
                drain_available_outputs(&self.output, &mut outputs);
                return SchedulerShutdown {
                    outputs,
                    result: Err(SchedulerError::ShutdownTimeout),
                };
            }
            thread::sleep(SCHEDULER_POLL_INTERVAL);
        }
    }

    fn join(&mut self) -> Result<(), SchedulerError> {
        let Some(thread) = self.thread.take() else {
            return Ok(());
        };
        thread.join().map_err(|_| SchedulerError::ThreadPanicked)?
    }
}

fn drain_available_outputs(
    receiver: &Receiver<SchedulerOutput>,
    outputs: &mut Vec<SchedulerOutput>,
) {
    while let Ok(output) = receiver.try_recv() {
        outputs.push(output);
    }
}

struct ActiveSynthesis {
    request: SayRequest,
    prepared: PreparedRequest,
    segment_index: usize,
    segment_sample_base: u64,
    segment_token_base: u64,
    stream: SynthesisStream,
    segment_progress: StreamProgress,
    logical_progress: LogicalProgress,
    cancellation: Option<Cancellation>,
    cancellation_requested_at: Option<Instant>,
    last_progress: Instant,
}

#[derive(Debug, Default)]
struct LogicalProgress {
    next_sequence: u64,
    next_sample_index: u64,
    committed_text_tokens: u64,
    pending_first_sample_index: u64,
    pending_samples: Vec<f32>,
    pending_alignments: VecDeque<AlignmentPoint>,
    first_audio_published: bool,
}

#[derive(Debug, Default)]
struct StreamProgress {
    native_next_sequence: u64,
    native_next_sample_index: u64,
    published_next_sequence: u64,
    published_next_sample_index: u64,
    committed_text_tokens: u64,
    final_chunk_seen: bool,
    audio_seen: bool,
}

#[derive(Clone, Debug)]
enum Cancellation {
    Cancelled,
    Failed(String),
    Shutdown,
}

struct SchedulerRuntime {
    worker: InferenceWorker,
    frontend: FrontendClient,
    queue: RequestQueue,
    active: Option<ActiveSynthesis>,
    pending_output: VecDeque<SchedulerOutput>,
    manifest_sha256: [u8; 32],
    progress_timeout: Duration,
    cancellation_terminal_timeout: Duration,
    agent_playback_available: bool,
    system_playback_available: bool,
}

#[allow(clippy::too_many_arguments)]
fn run_scheduler(
    config: SchedulerConfig,
    say_receiver: Receiver<SayRequest>,
    control_receiver: Receiver<PlaybackCommand>,
    wake_receiver: Receiver<()>,
    output_sender: SyncSender<SchedulerOutput>,
    ready_sender: SyncSender<Result<ReadyInfo, String>>,
    shutdown: Arc<AtomicBool>,
    heartbeat: Arc<Mutex<Instant>>,
) -> Result<(), SchedulerError> {
    let idle_heartbeat_interval = idle_heartbeat_interval(config.scheduler_watchdog)?;
    // SAFETY: the path is explicit and validated as a regular file. The
    // Library value remains alive until after the inference worker is shut
    // down and joined.
    let native_library =
        unsafe { Library::new(&config.native_library) }.map_err(SchedulerError::LoadLibrary)?;
    // SAFETY: ABI v1 exposes exactly this sole symbol and InferenceWorker
    // negotiates its table before using any function pointer.
    let get_api = unsafe {
        *native_library
            .get::<GetApiFn>(b"mtt_get_api\0")
            .map_err(SchedulerError::LoadSymbol)?
    };
    let runtime_config =
        RuntimeConfig::new(config.cuda_device_index).map_err(SchedulerError::MagpieRuntime)?;
    let worker_config = WorkerConfig::new(
        runtime_config,
        config
            .bundle_path
            .to_str()
            .ok_or_else(|| {
                SchedulerError::Configuration("bundle_path must be valid UTF-8".to_owned())
            })?
            .to_owned(),
        config.expected_manifest_sha256,
    )
    .map_err(SchedulerError::Worker)?;
    // SAFETY: native_library remains live through worker.shutdown below.
    let worker = unsafe { InferenceWorker::spawn_with_get_api(get_api, worker_config) }
        .map_err(SchedulerError::Worker)?;
    let model_info = worker.model_info().clone();
    let frontend = match FrontendClient::spawn(config.frontend, &model_info) {
        Ok(frontend) => frontend,
        Err(error) => {
            worker.shutdown().map_err(SchedulerError::Worker)?;
            return Err(SchedulerError::Frontend(error.to_string()));
        }
    };
    if frontend.identity() != &FrontendIdentity::from_model_info(&model_info) {
        worker.shutdown().map_err(SchedulerError::Worker)?;
        return Err(SchedulerError::Frontend(
            "frontend identity changed after readiness".to_owned(),
        ));
    }
    ready_sender
        .send(Ok(ReadyInfo {
            model: model_info,
            expected_manifest_sha256: config.expected_manifest_sha256,
            cuda_device_index: config.cuda_device_index,
        }))
        .map_err(|_| SchedulerError::ReadyDisconnected)?;

    let mut runtime = SchedulerRuntime {
        worker,
        frontend,
        queue: RequestQueue::default(),
        active: None,
        pending_output: VecDeque::new(),
        manifest_sha256: config.expected_manifest_sha256,
        progress_timeout: config.request_progress_timeout,
        cancellation_terminal_timeout: config.cancellation_terminal_timeout,
        agent_playback_available: false,
        system_playback_available: false,
    };
    let loop_result = scheduler_loop(
        &mut runtime,
        &say_receiver,
        &control_receiver,
        &wake_receiver,
        &output_sender,
        &shutdown,
        &heartbeat,
        idle_heartbeat_interval,
    );
    let shutdown_result = runtime.worker.shutdown().map_err(SchedulerError::Worker);
    drop(native_library);
    loop_result?;
    shutdown_result
}

fn scheduler_loop(
    runtime: &mut SchedulerRuntime,
    say_receiver: &Receiver<SayRequest>,
    control_receiver: &Receiver<PlaybackCommand>,
    wake_receiver: &Receiver<()>,
    output_sender: &SyncSender<SchedulerOutput>,
    shutdown: &AtomicBool,
    heartbeat: &Mutex<Instant>,
    idle_heartbeat_interval: Duration,
) -> Result<(), SchedulerError> {
    loop {
        update_heartbeat(heartbeat)?;
        flush_one_output(&mut runtime.pending_output, output_sender)?;
        drain_controls(runtime, control_receiver)?;
        drain_say_requests(runtime, say_receiver)?;

        if shutdown.load(Ordering::SeqCst) {
            for kind in [SpeechKind::System, SpeechKind::Agent] {
                for request in runtime.queue.cancel_kind(kind) {
                    runtime.pending_output.push_back(cancelled_terminal(
                        &request,
                        runtime.worker.model_info().sample_rate_hz,
                    ));
                }
            }
            if let Some(active) = runtime.active.as_mut() {
                request_cancellation(active, Cancellation::Shutdown)?;
            } else if runtime.pending_output.is_empty() && runtime.queue.pending_len() == 0 {
                return Ok(());
            }
        }

        if runtime.pending_output.is_empty() {
            if runtime.active.is_none() {
                start_next(runtime)?;
            }
            if runtime.active.as_ref().is_some_and(|active| {
                cancellation_deadline_expired(
                    active.cancellation_requested_at,
                    runtime.cancellation_terminal_timeout,
                )
            }) {
                let active = runtime.active.take().expect("active checked");
                runtime.queue.finish_request(&active.request.utterance_id)?;
                runtime.pending_output.push_back(failed_terminal(
                    &active.request,
                    active.logical_progress.next_sequence,
                    active.logical_progress.next_sample_index,
                    active.worker_sample_rate(),
                    format!(
                        "MagpieTTS-RT did not reach a terminal event within {:.3} seconds after cancellation",
                        runtime.cancellation_terminal_timeout.as_secs_f64()
                    ),
                ));
                runtime.pending_output.push_back(SchedulerOutput::Fatal(
                    "MagpieTTS-RT cancellation terminal deadline expired".to_owned(),
                ));
                continue;
            }
            if let Some(active) = runtime.active.as_mut() {
                if active.last_progress.elapsed() >= runtime.progress_timeout
                    && active.cancellation.is_none()
                {
                    request_cancellation(
                        active,
                        Cancellation::Failed(format!(
                            "MagpieTTS-RT request made no progress for {:.3} seconds",
                            runtime.progress_timeout.as_secs_f64()
                        )),
                    )?;
                }
                match active.stream.recv_timeout(SCHEDULER_POLL_INTERVAL) {
                    Ok(Some(event)) => {
                        active.last_progress = Instant::now();
                        let runtime_failure = match &event {
                            SynthesisEvent::RuntimeError(error) => Some(error.clone()),
                            _ => None,
                        };
                        match process_synthesis_event(
                            active,
                            event,
                            &runtime.worker,
                            &runtime.manifest_sha256,
                        ) {
                            Ok(outputs) => {
                                let terminal = outputs.iter().any(|output| {
                                    matches!(output, SchedulerOutput::Terminal { .. })
                                });
                                runtime.pending_output.extend(outputs);
                                if let Some(error) = runtime_failure {
                                    runtime.pending_output.push_back(SchedulerOutput::Fatal(
                                        format!("MagpieTTS-RT runtime contract failed: {error}"),
                                    ));
                                }
                                if terminal {
                                    runtime.queue.finish_request(&active.request.utterance_id)?;
                                    runtime.active = None;
                                }
                            }
                            Err(error) => {
                                let request = active.request.clone();
                                runtime.pending_output.extend(streaming_protocol_failure(
                                    &request,
                                    active.logical_progress.next_sequence,
                                    active.logical_progress.next_sample_index,
                                    active.worker_sample_rate(),
                                    &error,
                                ));
                                runtime.queue.finish_request(&active.request.utterance_id)?;
                                runtime.active = None;
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(error) => {
                        let request = active.request.clone();
                        let terminal = failed_terminal(
                            &request,
                            active.logical_progress.next_sequence,
                            active.logical_progress.next_sample_index,
                            active.worker_sample_rate(),
                            format!("MagpieTTS-RT event stream failed: {error}"),
                        );
                        runtime.pending_output.push_back(terminal);
                        runtime
                            .pending_output
                            .push_back(SchedulerOutput::Fatal(format!(
                                "MagpieTTS-RT worker event channel failed: {error}"
                            )));
                        runtime.queue.finish_request(&active.request.utterance_id)?;
                        runtime.active = None;
                    }
                }
            }
        } else {
            thread::sleep(SCHEDULER_POLL_INTERVAL);
        }

        if runtime.active.is_none()
            && runtime.queue.pending_len() == 0
            && runtime.pending_output.is_empty()
            && !shutdown.load(Ordering::SeqCst)
        {
            wait_for_scheduler_work(wake_receiver, idle_heartbeat_interval)?;
        }
    }
}

fn notify_scheduler(wake: &SyncSender<()>) -> Result<(), SchedulerSendError> {
    // A queued wake represents every payload already waiting in the bounded
    // say/control channels, so coalescing does not drop semantic work.
    match wake.try_send(()) {
        Ok(()) | Err(TrySendError::Full(())) => Ok(()),
        Err(TrySendError::Disconnected(())) => Err(SchedulerSendError::Disconnected),
    }
}

fn wait_for_scheduler_work(
    wake: &Receiver<()>,
    idle_heartbeat_interval: Duration,
) -> Result<(), SchedulerError> {
    match wake.recv_timeout(idle_heartbeat_interval) {
        Ok(()) | Err(RecvTimeoutError::Timeout) => Ok(()),
        Err(RecvTimeoutError::Disconnected) => Err(SchedulerError::WakeDisconnected),
    }
}

fn streaming_protocol_failure(
    request: &SayRequest,
    next_sequence: u64,
    next_sample_index: u64,
    sample_rate_hz: u32,
    error: &SchedulerError,
) -> [SchedulerOutput; 2] {
    [
        failed_terminal(
            request,
            next_sequence,
            next_sample_index,
            sample_rate_hz,
            error.to_string(),
        ),
        SchedulerOutput::Fatal(format!("MagpieTTS-RT streaming contract failed: {error}")),
    ]
}

fn start_next(runtime: &mut SchedulerRuntime) -> Result<(), SchedulerError> {
    let Some(request) = runtime.queue.take_next() else {
        return Ok(());
    };
    let prepared = match runtime.frontend.prepare(
        &request.utterance_id,
        &request.text,
        runtime.worker.model_info().maximum_text_tokens,
    ) {
        Ok(prepared) => prepared,
        Err(error) => {
            runtime.pending_output.push_back(failed_terminal(
                &request,
                0,
                0,
                runtime.worker.model_info().sample_rate_hz,
                error.to_string(),
            ));
            if error.is_process_fatal() {
                runtime
                    .pending_output
                    .push_back(SchedulerOutput::Fatal(format!(
                        "persistent Japanese frontend failed: {error}"
                    )));
            }
            runtime.queue.finish_request(&request.utterance_id)?;
            return Ok(());
        }
    };
    runtime
        .pending_output
        .push_back(SchedulerOutput::Timing(TimingOutput {
            kind: request.kind,
            utterance_id: request.utterance_id.clone(),
            stage: TimingStage::FrontendCompleted,
            monotonic_time_ns: monotonic_time_ns()?,
        }));
    let first_segment = prepared
        .segments
        .first()
        .expect("frontend validation requires at least one segment");
    let native_request_started_ns = monotonic_time_ns()?;
    let stream = match synthesize_segment(
        &runtime.worker,
        &runtime.manifest_sha256,
        &request,
        first_segment,
    ) {
        Ok(stream) => stream,
        Err(error) => {
            runtime.pending_output.push_back(failed_terminal(
                &request,
                0,
                0,
                runtime.worker.model_info().sample_rate_hz,
                format!("MagpieTTS-RT could not start synthesis: {error}"),
            ));
            runtime
                .pending_output
                .push_back(SchedulerOutput::Fatal(format!(
                    "MagpieTTS-RT request start failed: {error}"
                )));
            runtime.queue.finish_request(&request.utterance_id)?;
            return Ok(());
        }
    };
    runtime
        .pending_output
        .push_back(SchedulerOutput::Timing(TimingOutput {
            kind: request.kind,
            utterance_id: request.utterance_id.clone(),
            stage: TimingStage::NativeRequestStarted,
            monotonic_time_ns: native_request_started_ns,
        }));
    runtime.active = Some(ActiveSynthesis {
        request,
        prepared,
        segment_index: 0,
        segment_sample_base: 0,
        segment_token_base: 0,
        stream,
        segment_progress: StreamProgress::default(),
        logical_progress: LogicalProgress::default(),
        cancellation: None,
        cancellation_requested_at: None,
        last_progress: Instant::now(),
    });
    Ok(())
}

fn process_synthesis_event(
    active: &mut ActiveSynthesis,
    event: SynthesisEvent,
    worker: &InferenceWorker,
    manifest_sha256: &[u8; 32],
) -> Result<Vec<SchedulerOutput>, SchedulerError> {
    match event {
        SynthesisEvent::Audio(chunk) => {
            if active.cancellation.is_some() {
                validate_suppressed_chunk(active, &chunk)?;
                return Ok(Vec::new());
            }
            let first_native_audio =
                active.segment_index == 0 && !active.segment_progress.audio_seen;
            let mut outputs = validate_audio_chunk(active, chunk)?;
            if first_native_audio {
                outputs.insert(
                    0,
                    SchedulerOutput::Timing(TimingOutput {
                        kind: active.request.kind,
                        utterance_id: active.request.utterance_id.clone(),
                        stage: TimingStage::FirstNativeAudio,
                        monotonic_time_ns: monotonic_time_ns()?,
                    }),
                );
            }
            Ok(outputs)
        }
        SynthesisEvent::Completed => {
            if let Some(cancellation) = active.cancellation.clone() {
                return Ok(vec![terminal_for_cancellation(active, cancellation)]);
            }
            let current_segment = active.current_segment();
            if !active.segment_progress.audio_seen
                || !active.segment_progress.final_chunk_seen
                || active.segment_progress.committed_text_tokens
                    != current_segment.prepared.total_prepared_tokens()
            {
                return Err(SchedulerError::Protocol(
                    "MagpieTTS-RT segment completed without a final fully-aligned audio chunk"
                        .to_owned(),
                ));
            }
            if should_advance_segment(
                active.cancellation.as_ref(),
                active.segment_index,
                active.prepared.segments.len(),
            ) {
                advance_segment(active, worker, manifest_sha256)?;
                return Ok(Vec::new());
            }
            let mut outputs = flush_logical_final(active)?;
            outputs.push(completed_terminal(active));
            Ok(outputs)
        }
        SynthesisEvent::Cancelled => {
            let cancellation = active.cancellation.clone().ok_or_else(|| {
                SchedulerError::Protocol(
                    "MagpieTTS-RT cancelled a request without a cancellation command".to_owned(),
                )
            })?;
            Ok(vec![terminal_for_cancellation(active, cancellation)])
        }
        SynthesisEvent::Failed(error) => Ok(vec![failed_terminal(
            &active.request,
            active.logical_progress.next_sequence,
            active.logical_progress.next_sample_index,
            active.worker_sample_rate(),
            format!("MagpieTTS-RT synthesis failed: {error}"),
        )]),
        SynthesisEvent::RuntimeError(error) => {
            let terminal = failed_terminal(
                &active.request,
                active.logical_progress.next_sequence,
                active.logical_progress.next_sample_index,
                active.worker_sample_rate(),
                format!("MagpieTTS-RT runtime contract failed: {error}"),
            );
            Ok(vec![terminal])
        }
    }
}

fn should_advance_segment(
    cancellation: Option<&Cancellation>,
    segment_index: usize,
    segment_count: usize,
) -> bool {
    cancellation.is_none() && segment_index + 1 < segment_count
}

fn validate_audio_chunk(
    active: &mut ActiveSynthesis,
    chunk: OwnedAudioChunk,
) -> Result<Vec<SchedulerOutput>, SchedulerError> {
    let segment_total_tokens = active.current_segment().prepared.total_prepared_tokens();
    let accepted = active.segment_progress.accept(
        &chunk,
        segment_total_tokens,
        active.worker_sample_rate(),
    )?;
    active.logical_progress.accept_audio(
        active.request.kind,
        &active.request.utterance_id,
        &active.prepared.source_text,
        &active.prepared.progress,
        active.segment_sample_base,
        active.segment_token_base,
        chunk.samples,
        accepted,
        active.worker_sample_rate(),
    )
}

fn validate_suppressed_chunk(
    active: &mut ActiveSynthesis,
    chunk: &OwnedAudioChunk,
) -> Result<(), SchedulerError> {
    active
        .segment_progress
        .suppress(chunk, active.worker_sample_rate())
}

fn synthesize_segment(
    worker: &InferenceWorker,
    manifest_sha256: &[u8; 32],
    request: &SayRequest,
    segment: &PreparedSegment,
) -> Result<SynthesisStream, WorkerError> {
    let seed_identifier = format!(
        "{}/independent-segment/{}",
        request.utterance_id, segment.segment_index
    );
    let seed = u64::from(deterministic_seed(
        manifest_sha256,
        &seed_identifier,
        &segment.prepared.source_text,
    ));
    worker.synthesize(segment.prepared.token_ids.clone(), seed)
}

fn advance_segment(
    active: &mut ActiveSynthesis,
    worker: &InferenceWorker,
    manifest_sha256: &[u8; 32],
) -> Result<(), SchedulerError> {
    let completed_tokens = active.current_segment().prepared.total_prepared_tokens();
    active.segment_token_base = active
        .segment_token_base
        .checked_add(completed_tokens)
        .ok_or_else(|| SchedulerError::Protocol("logical token offset overflowed".to_owned()))?;
    active.segment_sample_base = active
        .segment_sample_base
        .checked_add(active.segment_progress.native_next_sample_index)
        .ok_or_else(|| SchedulerError::Protocol("logical sample offset overflowed".to_owned()))?;
    active.segment_index += 1;
    let stream = synthesize_segment(
        worker,
        manifest_sha256,
        &active.request,
        active.current_segment(),
    )
    .map_err(SchedulerError::Worker)?;
    active.stream = stream;
    active.segment_progress = StreamProgress::default();
    active.last_progress = Instant::now();
    Ok(())
}

fn flush_logical_final(
    active: &mut ActiveSynthesis,
) -> Result<Vec<SchedulerOutput>, SchedulerError> {
    let pending_codec_frames = active.logical_progress.pending_samples.len() / CODEC_FRAME_SAMPLES;
    if !active
        .logical_progress
        .pending_samples
        .len()
        .is_multiple_of(CODEC_FRAME_SAMPLES)
        || pending_codec_frames > STEADY_CODEC_FRAMES
    {
        return Err(SchedulerError::Protocol(
            "logical segmented stream does not end with a zero-through-eight-frame FINAL"
                .to_owned(),
        ));
    }
    let projected_final_tokens = active
        .logical_progress
        .pending_alignments
        .back()
        .map_or(active.logical_progress.committed_text_tokens, |event| {
            event.committed_text_tokens
        });
    if projected_final_tokens != active.prepared.total_prepared_tokens {
        return Err(SchedulerError::Protocol(
            "logical segmented stream FINAL does not align the complete source".to_owned(),
        ));
    }
    let output = if active.logical_progress.pending_samples.is_empty() {
        active.logical_progress.emit_final_marker(
            active.request.kind,
            &active.request.utterance_id,
            active.worker_sample_rate(),
        )?
    } else {
        active.logical_progress.emit_chunk(
            active.request.kind,
            &active.request.utterance_id,
            &active.prepared.source_text,
            &active.prepared.progress,
            active.logical_progress.pending_samples.len(),
            true,
            active.worker_sample_rate(),
        )?
    };
    if active.logical_progress.committed_text_tokens != active.prepared.total_prepared_tokens {
        return Err(SchedulerError::Protocol(
            "logical segmented stream final tail did not align the complete source".to_owned(),
        ));
    }
    Ok(vec![SchedulerOutput::Audio(output)])
}

struct AcceptedChunk {
    first: bool,
    alignment_events: Vec<AlignmentPoint>,
}

impl StreamProgress {
    fn validate_common(
        &self,
        chunk: &OwnedAudioChunk,
        sample_rate_hz: u32,
    ) -> Result<(), SchedulerError> {
        if chunk.sequence != self.native_next_sequence
            || chunk.first_sample_index != self.native_next_sample_index
            || chunk.sample_rate_hz != sample_rate_hz
            || chunk.first != (self.native_next_sequence == 0)
            || self.final_chunk_seen
            || chunk.samples.iter().any(|sample| !sample.is_finite())
        {
            return Err(SchedulerError::Protocol(
                "native audio chunk violates sequence, format, flags, or PCM constraints"
                    .to_owned(),
            ));
        }
        if chunk.samples.is_empty()
            && (!chunk.final_chunk
                || chunk.first
                || chunk
                    .committed_text_tokens
                    .is_some_and(|tokens| tokens != self.committed_text_tokens)
                || !chunk.alignment_events.is_empty())
        {
            return Err(SchedulerError::Protocol(
                "zero-frame native audio is only a progress-preserving non-first FINAL marker"
                    .to_owned(),
            ));
        }
        Ok(())
    }

    fn accept(
        &mut self,
        chunk: &OwnedAudioChunk,
        total_prepared_tokens: u64,
        sample_rate_hz: u32,
    ) -> Result<AcceptedChunk, SchedulerError> {
        self.validate_common(chunk, sample_rate_hz)?;
        let chunk_sample_count = u64::try_from(chunk.samples.len())
            .map_err(|_| SchedulerError::Protocol("PCM length exceeds uint64".to_owned()))?;
        let next_sample_index = self
            .native_next_sample_index
            .checked_add(chunk_sample_count)
            .ok_or_else(|| SchedulerError::Protocol("PCM sample index overflowed".to_owned()))?;
        // A native lease without ALIGNMENT_VALID reports no lease-local token
        // value. The ROS stream carries cumulative progress, so retain the
        // last authenticated native position instead of resetting it to zero.
        let committed = chunk
            .committed_text_tokens
            .unwrap_or(self.committed_text_tokens);
        if committed < self.committed_text_tokens || committed > total_prepared_tokens {
            return Err(SchedulerError::Protocol(
                "native committed_text_tokens is not monotonic or exceeds the frontend map"
                    .to_owned(),
            ));
        }
        let mut previous_sample = self.native_next_sample_index;
        let mut previous_tokens = self.committed_text_tokens;
        let mut alignment_events = Vec::with_capacity(chunk.alignment_events.len());
        for event in &chunk.alignment_events {
            if event.sample_index < previous_sample
                || event.sample_index > next_sample_index
                || event.committed_text_tokens <= previous_tokens
                || event.committed_text_tokens > committed
            {
                return Err(SchedulerError::Protocol(
                    "native alignment event is outside its ordered audio/token range".to_owned(),
                ));
            }
            previous_sample = event.sample_index;
            previous_tokens = event.committed_text_tokens;
            alignment_events.push(AlignmentPoint {
                sample_index: event.sample_index,
                committed_text_tokens: event.committed_text_tokens,
            });
        }
        if committed > self.committed_text_tokens
            && alignment_events
                .last()
                .is_none_or(|event| event.committed_text_tokens != committed)
        {
            return Err(SchedulerError::Protocol(
                "native committed token advance lacks a matching alignment event".to_owned(),
            ));
        }

        let accepted = AcceptedChunk {
            first: self.native_next_sequence == 0,
            alignment_events,
        };
        self.advance_native(chunk_sample_count, chunk.final_chunk)?;
        self.published_next_sequence = self.native_next_sequence;
        self.published_next_sample_index = self.native_next_sample_index;
        self.committed_text_tokens = committed;
        self.audio_seen = true;
        Ok(accepted)
    }

    fn suppress(
        &mut self,
        chunk: &OwnedAudioChunk,
        sample_rate_hz: u32,
    ) -> Result<(), SchedulerError> {
        self.validate_common(chunk, sample_rate_hz)?;
        let sample_count = u64::try_from(chunk.samples.len())
            .map_err(|_| SchedulerError::Protocol("PCM length exceeds uint64".to_owned()))?;
        self.advance_native(sample_count, chunk.final_chunk)
    }

    fn advance_native(
        &mut self,
        sample_count: u64,
        final_chunk: bool,
    ) -> Result<(), SchedulerError> {
        self.native_next_sequence = self
            .native_next_sequence
            .checked_add(1)
            .ok_or_else(|| SchedulerError::Protocol("audio sequence overflowed".to_owned()))?;
        self.native_next_sample_index = self
            .native_next_sample_index
            .checked_add(sample_count)
            .ok_or_else(|| SchedulerError::Protocol("PCM sample index overflowed".to_owned()))?;
        self.final_chunk_seen = final_chunk;
        Ok(())
    }
}

fn completed_terminal(active: &ActiveSynthesis) -> SchedulerOutput {
    SchedulerOutput::Terminal {
        speech: empty_terminal(active, SpeechEvent::Complete),
        result: ResultOutput {
            kind: active.request.kind,
            utterance_id: active.request.utterance_id.clone(),
            status: ResultStatus::Completed,
            error: None,
        },
    }
}

fn terminal_for_cancellation(
    active: &ActiveSynthesis,
    cancellation: Cancellation,
) -> SchedulerOutput {
    let (status, error) = match cancellation {
        Cancellation::Cancelled | Cancellation::Shutdown => (ResultStatus::Cancelled, None),
        Cancellation::Failed(message) => (ResultStatus::Failed, Some(message)),
    };
    SchedulerOutput::Terminal {
        speech: empty_terminal(active, SpeechEvent::Abort),
        result: ResultOutput {
            kind: active.request.kind,
            utterance_id: active.request.utterance_id.clone(),
            status,
            error,
        },
    }
}

fn empty_terminal(active: &ActiveSynthesis, event: SpeechEvent) -> SpeechOutput {
    SpeechOutput {
        kind: active.request.kind,
        event,
        utterance_id: active.request.utterance_id.clone(),
        sequence: active.logical_progress.next_sequence,
        first_sample_index: active.logical_progress.next_sample_index,
        sample_rate_hz: active.worker_sample_rate(),
        channels: 1,
        pcm_f32: Vec::new(),
        final_chunk: false,
        committed_text_tokens: 0,
        alignment_events: Vec::new(),
        source_text: String::new(),
        source_progress: Vec::new(),
    }
}

fn failed_terminal(
    request: &SayRequest,
    sequence: u64,
    first_sample_index: u64,
    sample_rate_hz: u32,
    error: String,
) -> SchedulerOutput {
    SchedulerOutput::Terminal {
        speech: SpeechOutput {
            kind: request.kind,
            event: SpeechEvent::Abort,
            utterance_id: request.utterance_id.clone(),
            sequence,
            first_sample_index,
            sample_rate_hz,
            channels: 1,
            pcm_f32: Vec::new(),
            final_chunk: false,
            committed_text_tokens: 0,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        },
        result: ResultOutput {
            kind: request.kind,
            utterance_id: request.utterance_id.clone(),
            status: ResultStatus::Failed,
            error: Some(error),
        },
    }
}

fn drain_controls(
    runtime: &mut SchedulerRuntime,
    receiver: &Receiver<PlaybackCommand>,
) -> Result<(), SchedulerError> {
    loop {
        match receiver.try_recv() {
            Ok(PlaybackCommand::Ignore) => {}
            Ok(PlaybackCommand::AdvanceAgentFloor(floor)) => {
                let cancelled = runtime.queue.advance_agent_floor(floor);
                for request in cancelled {
                    runtime.pending_output.push_back(cancelled_terminal(
                        &request,
                        runtime.worker.model_info().sample_rate_hz,
                    ));
                }
                if runtime
                    .active
                    .as_ref()
                    .is_some_and(|active| runtime.queue.agent_is_stale(&active.request))
                {
                    request_cancellation(
                        runtime.active.as_mut().expect("active checked"),
                        Cancellation::Cancelled,
                    )?;
                }
            }
            Ok(PlaybackCommand::AbortSystem(utterance_id)) => {
                let active_match = runtime.active.as_ref().is_some_and(|active| {
                    active.request.kind == SpeechKind::System
                        && active.request.utterance_id == utterance_id
                });
                let cancelled = runtime.queue.abort_system(&utterance_id, active_match);
                for request in cancelled {
                    runtime.pending_output.push_back(cancelled_terminal(
                        &request,
                        runtime.worker.model_info().sample_rate_hz,
                    ));
                }
                if active_match {
                    request_cancellation(
                        runtime.active.as_mut().expect("active checked"),
                        Cancellation::Cancelled,
                    )?;
                }
            }
            Ok(PlaybackCommand::AbortAgent(utterance_id)) => {
                let active_match = runtime.active.as_ref().is_some_and(|active| {
                    active.request.kind == SpeechKind::Agent
                        && active.request.utterance_id == utterance_id
                });
                let cancelled = runtime.queue.abort_agent(&utterance_id);
                for request in cancelled {
                    runtime.pending_output.push_back(cancelled_terminal(
                        &request,
                        runtime.worker.model_info().sample_rate_hz,
                    ));
                }
                if active_match {
                    request_cancellation(
                        runtime.active.as_mut().expect("active checked"),
                        Cancellation::Cancelled,
                    )?;
                }
            }
            Ok(PlaybackCommand::SetPlaybackAvailable { kind, available }) => {
                runtime.set_playback_available(kind, available);
                if !available {
                    let cancelled = runtime.queue.cancel_kind(kind);
                    for request in cancelled {
                        runtime.pending_output.push_back(cancelled_terminal(
                            &request,
                            runtime.worker.model_info().sample_rate_hz,
                        ));
                    }
                    if runtime
                        .active
                        .as_ref()
                        .is_some_and(|active| active.request.kind == kind)
                    {
                        request_cancellation(
                            runtime.active.as_mut().expect("active checked"),
                            Cancellation::Cancelled,
                        )?;
                    }
                }
            }
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => {
                return Err(SchedulerError::ControlDisconnected);
            }
        }
    }
}

fn drain_say_requests(
    runtime: &mut SchedulerRuntime,
    receiver: &Receiver<SayRequest>,
) -> Result<(), SchedulerError> {
    while runtime
        .queue
        .pending_len()
        .saturating_add(runtime.pending_output.len())
        < MAX_PENDING_REQUESTS
    {
        let request = match receiver.try_recv() {
            Ok(request) => request,
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => return Err(SchedulerError::SayDisconnected),
        };
        match runtime.queue.submit(request.clone())? {
            SubmitDisposition::Queued if runtime.playback_available(request.kind) => {}
            SubmitDisposition::Queued => {
                for unavailable in runtime.queue.cancel_kind(request.kind) {
                    runtime.pending_output.push_back(cancelled_terminal(
                        &unavailable,
                        runtime.worker.model_info().sample_rate_hz,
                    ));
                }
            }
            SubmitDisposition::Cancelled | SubmitDisposition::Stale => {
                runtime.pending_output.push_back(cancelled_terminal(
                    &request,
                    runtime.worker.model_info().sample_rate_hz,
                ));
            }
        }
    }
    Ok(())
}

fn cancelled_terminal(request: &SayRequest, sample_rate_hz: u32) -> SchedulerOutput {
    SchedulerOutput::Terminal {
        speech: SpeechOutput {
            kind: request.kind,
            event: SpeechEvent::Abort,
            utterance_id: request.utterance_id.clone(),
            sequence: 0,
            first_sample_index: 0,
            sample_rate_hz,
            channels: 1,
            pcm_f32: Vec::new(),
            final_chunk: false,
            committed_text_tokens: 0,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        },
        result: ResultOutput {
            kind: request.kind,
            utterance_id: request.utterance_id.clone(),
            status: ResultStatus::Cancelled,
            error: None,
        },
    }
}

fn request_cancellation(
    active: &mut ActiveSynthesis,
    reason: Cancellation,
) -> Result<(), SchedulerError> {
    if active.cancellation.is_some() {
        return Ok(());
    }
    active.stream.cancel().map_err(SchedulerError::Worker)?;
    active.cancellation = Some(reason);
    active.cancellation_requested_at = Some(Instant::now());
    active.last_progress = Instant::now();
    Ok(())
}

fn cancellation_deadline_expired(
    cancellation_requested_at: Option<Instant>,
    timeout: Duration,
) -> bool {
    cancellation_requested_at.is_some_and(|started| started.elapsed() >= timeout)
}

impl SchedulerRuntime {
    fn playback_available(&self, kind: SpeechKind) -> bool {
        match kind {
            SpeechKind::Agent => self.agent_playback_available,
            SpeechKind::System => self.system_playback_available,
        }
    }

    fn set_playback_available(&mut self, kind: SpeechKind, available: bool) {
        match kind {
            SpeechKind::Agent => self.agent_playback_available = available,
            SpeechKind::System => self.system_playback_available = available,
        }
    }
}

fn flush_one_output(
    pending: &mut VecDeque<SchedulerOutput>,
    sender: &SyncSender<SchedulerOutput>,
) -> Result<(), SchedulerError> {
    let Some(output) = pending.pop_front() else {
        return Ok(());
    };
    match sender.try_send(output) {
        Ok(()) => Ok(()),
        Err(TrySendError::Full(output)) => {
            pending.push_front(output);
            Ok(())
        }
        Err(TrySendError::Disconnected(_)) => Err(SchedulerError::OutputDisconnected),
    }
}

fn update_heartbeat(heartbeat: &Mutex<Instant>) -> Result<(), SchedulerError> {
    *heartbeat
        .lock()
        .map_err(|_| SchedulerError::HeartbeatPoisoned)? = Instant::now();
    Ok(())
}

fn monotonic_time_ns() -> Result<u64, SchedulerError> {
    let mut timestamp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: timestamp points to initialized writable storage and
    // CLOCK_MONOTONIC is shared by every process on this Linux host.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut timestamp) } != 0 {
        return Err(SchedulerError::Clock(std::io::Error::last_os_error()));
    }
    if timestamp.tv_sec < 0 || !(0..1_000_000_000).contains(&timestamp.tv_nsec) {
        return Err(SchedulerError::Protocol(
            "CLOCK_MONOTONIC returned an invalid timespec".to_owned(),
        ));
    }
    let seconds = u64::try_from(timestamp.tv_sec)
        .map_err(|_| SchedulerError::Protocol("monotonic seconds exceed uint64".to_owned()))?;
    let nanoseconds = u64::try_from(timestamp.tv_nsec)
        .map_err(|_| SchedulerError::Protocol("monotonic nanoseconds exceed uint64".to_owned()))?;
    seconds
        .checked_mul(1_000_000_000)
        .and_then(|value| value.checked_add(nanoseconds))
        .ok_or_else(|| SchedulerError::Protocol("monotonic nanoseconds overflow uint64".to_owned()))
}

fn require_absolute_regular_file(label: &str, path: &Path) -> Result<(), SchedulerError> {
    if !path.is_absolute() {
        return Err(SchedulerError::Configuration(format!(
            "{label} must be absolute"
        )));
    }
    let metadata = fs::symlink_metadata(path).map_err(|error| {
        SchedulerError::Configuration(format!(
            "cannot inspect {label} {}: {error}",
            path.display()
        ))
    })?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(SchedulerError::Configuration(format!(
            "{label} must be a regular non-symlink file: {}",
            path.display()
        )));
    }
    Ok(())
}

impl ActiveSynthesis {
    fn current_segment(&self) -> &PreparedSegment {
        &self.prepared.segments[self.segment_index]
    }

    fn worker_sample_rate(&self) -> u32 {
        // ABI v1 fixes this value and ModelInfo is validated by the safe
        // wrapper before a request can be created.
        22_050
    }
}

impl LogicalProgress {
    #[allow(clippy::too_many_arguments)]
    fn accept_audio(
        &mut self,
        kind: SpeechKind,
        utterance_id: &str,
        source_text: &str,
        source_progress: &[SourceProgress],
        segment_sample_base: u64,
        segment_token_base: u64,
        samples: Vec<f32>,
        accepted: AcceptedChunk,
        sample_rate_hz: u32,
    ) -> Result<Vec<SchedulerOutput>, SchedulerError> {
        for event in accepted.alignment_events {
            self.pending_alignments.push_back(AlignmentPoint {
                sample_index: segment_sample_base
                    .checked_add(event.sample_index)
                    .ok_or_else(|| {
                        SchedulerError::Protocol(
                            "logical alignment sample offset overflowed".to_owned(),
                        )
                    })?,
                committed_text_tokens: segment_token_base
                    .checked_add(event.committed_text_tokens)
                    .ok_or_else(|| {
                        SchedulerError::Protocol(
                            "logical alignment token offset overflowed".to_owned(),
                        )
                    })?,
            });
        }
        if !self.first_audio_published {
            if !accepted.first || samples.len() != INITIAL_CODEC_FRAMES * CODEC_FRAME_SAMPLES {
                return Err(SchedulerError::Protocol(
                    "logical stream must start with exactly four codec frames".to_owned(),
                ));
            }
            self.pending_samples = samples;
            self.pending_first_sample_index = 0;
            let output = self.emit_chunk(
                kind,
                utterance_id,
                source_text,
                source_progress,
                INITIAL_CODEC_FRAMES * CODEC_FRAME_SAMPLES,
                false,
                sample_rate_hz,
            )?;
            self.first_audio_published = true;
            return Ok(vec![SchedulerOutput::Audio(output)]);
        }

        self.pending_samples.extend(samples);
        let mut outputs = Vec::new();
        let steady_samples = STEADY_CODEC_FRAMES * CODEC_FRAME_SAMPLES;
        // A complete steady batch is playback-ready on arrival. Holding an
        // exact eight-frame batch until a future native event adds one full
        // codec interval of avoidable latency. Only an incomplete residual
        // remains buffered for the logical FINAL.
        while self.pending_samples.len() >= steady_samples {
            outputs.push(SchedulerOutput::Audio(self.emit_chunk(
                kind,
                utterance_id,
                source_text,
                source_progress,
                steady_samples,
                false,
                sample_rate_hz,
            )?));
        }
        Ok(outputs)
    }

    #[allow(clippy::too_many_arguments)]
    fn emit_chunk(
        &mut self,
        kind: SpeechKind,
        utterance_id: &str,
        source_text: &str,
        source_progress: &[SourceProgress],
        sample_count: usize,
        final_chunk: bool,
        sample_rate_hz: u32,
    ) -> Result<SpeechOutput, SchedulerError> {
        if sample_count == 0 || sample_count > self.pending_samples.len() {
            return Err(SchedulerError::Protocol(
                "logical rechunker requested an invalid PCM range".to_owned(),
            ));
        }
        let first_sample_index = self.pending_first_sample_index;
        if first_sample_index != self.next_sample_index {
            return Err(SchedulerError::Protocol(
                "logical rechunker sample position is not contiguous".to_owned(),
            ));
        }
        let sample_count_u64 = u64::try_from(sample_count).map_err(|_| {
            SchedulerError::Protocol("logical PCM length exceeds uint64".to_owned())
        })?;
        let sample_end = first_sample_index
            .checked_add(sample_count_u64)
            .ok_or_else(|| {
                SchedulerError::Protocol("logical sample index overflowed".to_owned())
            })?;
        let mut alignment_events = Vec::new();
        while self
            .pending_alignments
            .front()
            .is_some_and(|event| event.sample_index <= sample_end)
        {
            let event = self
                .pending_alignments
                .pop_front()
                .expect("front was checked");
            if event.sample_index <= first_sample_index
                || event.committed_text_tokens <= self.committed_text_tokens
            {
                return Err(SchedulerError::Protocol(
                    "logical alignment is not strictly ordered inside the emitted PCM".to_owned(),
                ));
            }
            self.committed_text_tokens = event.committed_text_tokens;
            alignment_events.push(event);
        }
        let pcm_f32 = self.pending_samples.drain(..sample_count).collect();
        let first = self.next_sequence == 0;
        let output = SpeechOutput {
            kind,
            event: SpeechEvent::Audio,
            utterance_id: utterance_id.to_owned(),
            sequence: self.next_sequence,
            first_sample_index,
            sample_rate_hz,
            channels: 1,
            pcm_f32,
            final_chunk,
            committed_text_tokens: self.committed_text_tokens,
            alignment_events,
            source_text: if first {
                source_text.to_owned()
            } else {
                String::new()
            },
            source_progress: if first {
                source_progress.to_vec()
            } else {
                Vec::new()
            },
        };
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| SchedulerError::Protocol("logical sequence overflowed".to_owned()))?;
        self.next_sample_index = sample_end;
        self.pending_first_sample_index = sample_end;
        Ok(output)
    }

    fn emit_final_marker(
        &mut self,
        kind: SpeechKind,
        utterance_id: &str,
        sample_rate_hz: u32,
    ) -> Result<SpeechOutput, SchedulerError> {
        if !self.first_audio_published
            || !self.pending_samples.is_empty()
            || !self.pending_alignments.is_empty()
            || self.next_sequence == 0
            || self.pending_first_sample_index != self.next_sample_index
        {
            return Err(SchedulerError::Protocol(
                "logical zero-frame FINAL marker requires a drained non-empty stream".to_owned(),
            ));
        }
        let output = SpeechOutput {
            kind,
            event: SpeechEvent::Audio,
            utterance_id: utterance_id.to_owned(),
            sequence: self.next_sequence,
            first_sample_index: self.next_sample_index,
            sample_rate_hz,
            channels: 1,
            pcm_f32: Vec::new(),
            final_chunk: true,
            committed_text_tokens: self.committed_text_tokens,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        };
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| SchedulerError::Protocol("logical sequence overflowed".to_owned()))?;
        Ok(output)
    }
}

impl From<crate::contracts::ContractError> for SchedulerError {
    fn from(error: crate::contracts::ContractError) -> Self {
        Self::Protocol(error.to_string())
    }
}

#[derive(Debug, Error)]
pub enum SchedulerSendError {
    #[error("TTS request queue is full")]
    SayQueueFull,
    #[error("TTS playback-control queue is full")]
    ControlQueueFull,
    #[error("TTS scheduler is disconnected")]
    Disconnected,
}

#[derive(Debug, Error)]
pub enum SchedulerError {
    #[error("invalid scheduler configuration: {0}")]
    Configuration(String),
    #[error("cannot start scheduler thread: {0}")]
    ThreadSpawn(std::io::Error),
    #[error("cannot read CLOCK_MONOTONIC: {0}")]
    Clock(std::io::Error),
    #[error("TTS scheduler startup failed: {0}")]
    Startup(String),
    #[error("TTS scheduler startup timed out")]
    StartupTimeout,
    #[error("cannot load MagpieTTS-RT library: {0}")]
    LoadLibrary(libloading::Error),
    #[error("cannot resolve MagpieTTS-RT C ABI: {0}")]
    LoadSymbol(libloading::Error),
    #[error("MagpieTTS-RT runtime configuration failed: {0}")]
    MagpieRuntime(magpie_tts_rt::Error),
    #[error("MagpieTTS-RT worker failed: {0}")]
    Worker(WorkerError),
    #[error("Japanese frontend failed: {0}")]
    Frontend(String),
    #[error("scheduler readiness receiver disconnected")]
    ReadyDisconnected,
    #[error("scheduler output channel disconnected")]
    OutputDisconnected,
    #[error("scheduler wake channel disconnected")]
    WakeDisconnected,
    #[error("scheduler say channel disconnected")]
    SayDisconnected,
    #[error("scheduler control channel disconnected")]
    ControlDisconnected,
    #[error("scheduler heartbeat lock was poisoned")]
    HeartbeatPoisoned,
    #[error("scheduler thread panicked")]
    ThreadPanicked,
    #[error("scheduler shutdown timed out")]
    ShutdownTimeout,
    #[error("streaming synthesis protocol failed: {0}")]
    Protocol(String),
}

#[cfg(test)]
mod tests {
    use magpie_tts_rt::{AlignmentEvent, OwnedAudioChunk};
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    use super::{
        AcceptedChunk, AlignmentPoint, Cancellation, LogicalProgress, ResultOutput,
        SchedulerOutput, SpeechEvent, SpeechOutput, StreamProgress, cancellation_deadline_expired,
        idle_heartbeat_interval, notify_scheduler, should_advance_segment,
        streaming_protocol_failure, wait_for_scheduler_work,
    };
    use crate::contracts::{ResultStatus, SayRequest, SourceProgress, SpeechKind};

    #[test]
    fn idle_heartbeat_is_derived_below_the_watchdog() {
        assert_eq!(
            idle_heartbeat_interval(Duration::from_secs(5)).expect("default watchdog"),
            Duration::from_secs(1)
        );
        assert_eq!(
            idle_heartbeat_interval(Duration::from_millis(400)).expect("short watchdog"),
            Duration::from_millis(100)
        );
        assert!(idle_heartbeat_interval(Duration::from_nanos(3)).is_err());
    }

    #[test]
    fn idle_wake_notifications_coalesce_without_losing_the_next_wakeup() {
        let (sender, receiver) = mpsc::sync_channel(1);
        notify_scheduler(&sender).expect("first wake");
        notify_scheduler(&sender).expect("coalesced wake");
        wait_for_scheduler_work(&receiver, Duration::from_secs(1)).expect("coalesced wake");

        notify_scheduler(&sender).expect("subsequent notification");
        wait_for_scheduler_work(&receiver, Duration::from_secs(1)).expect("subsequent wake");
    }

    #[test]
    fn disconnected_idle_wake_is_reported_to_the_sender() {
        let (sender, receiver) = mpsc::sync_channel(1);
        drop(receiver);
        assert!(notify_scheduler(&sender).is_err());
    }

    #[test]
    fn terminal_output_keeps_abort_and_result_in_one_ordered_event() {
        let output = SchedulerOutput::Terminal {
            speech: SpeechOutput {
                kind: SpeechKind::Agent,
                event: SpeechEvent::Abort,
                utterance_id: "agent-7-x".to_owned(),
                sequence: 2,
                first_sample_index: 4096,
                sample_rate_hz: 22_050,
                channels: 1,
                pcm_f32: Vec::new(),
                final_chunk: false,
                committed_text_tokens: 0,
                alignment_events: Vec::<AlignmentPoint>::new(),
                source_text: String::new(),
                source_progress: Vec::new(),
            },
            result: ResultOutput {
                kind: SpeechKind::Agent,
                utterance_id: "agent-7-x".to_owned(),
                status: ResultStatus::Cancelled,
                error: None,
            },
        };
        assert!(matches!(
            output,
            SchedulerOutput::Terminal {
                speech: SpeechOutput {
                    event: SpeechEvent::Abort,
                    sequence: 2,
                    ..
                },
                result: ResultOutput {
                    status: ResultStatus::Cancelled,
                    ..
                }
            }
        ));
    }

    #[test]
    fn suppressed_native_chunks_do_not_advance_the_ros_terminal_position() {
        let mut progress = StreamProgress::default();
        let first = chunk(0, 0, true, false, Some(2), &[(4, 2)], 4);
        progress.accept(&first, 3, 22_050).expect("first chunk");
        assert_eq!(progress.published_next_sequence, 1);
        assert_eq!(progress.published_next_sample_index, 4);

        let suppressed = chunk(1, 4, false, true, Some(3), &[(7, 3)], 3);
        progress
            .suppress(&suppressed, 22_050)
            .expect("suppressed chunk");
        assert_eq!(progress.native_next_sequence, 2);
        assert_eq!(progress.native_next_sample_index, 7);
        assert_eq!(progress.published_next_sequence, 1);
        assert_eq!(progress.published_next_sample_index, 4);
    }

    #[test]
    fn a_chunk_without_new_alignment_retains_exact_cumulative_progress() {
        let mut progress = StreamProgress::default();
        progress
            .accept(&chunk(0, 0, true, false, Some(2), &[(4, 2)], 4), 3, 22_050)
            .expect("first chunk");
        let accepted = progress
            .accept(&chunk(1, 4, false, false, None, &[], 4), 3, 22_050)
            .expect("unaligned chunk");
        assert_eq!(progress.committed_text_tokens, 2);
        assert!(accepted.alignment_events.is_empty());
    }

    #[test]
    fn native_eight_frame_final_is_explicit_and_closes_the_audio_sequence() {
        let mut progress = StreamProgress::default();
        progress
            .accept(
                &chunk(0, 0, true, false, Some(2), &[(4_096, 2)], 4_096),
                3,
                22_050,
            )
            .expect("initial four-frame chunk");
        let final_chunk = chunk(1, 4_096, false, true, Some(3), &[(12_288, 3)], 8 * 1_024);
        progress
            .accept(&final_chunk, 3, 22_050)
            .expect("explicit final eight-frame chunk");
        assert!(final_chunk.final_chunk);
        assert!(progress.final_chunk_seen);
        assert_eq!(progress.published_next_sequence, 2);
        assert_eq!(progress.published_next_sample_index, 12_288);
        assert!(
            progress
                .accept(
                    &chunk(2, 12_288, false, false, None, &[], 8 * 1_024),
                    3,
                    22_050,
                )
                .is_err()
        );
    }

    #[test]
    fn native_zero_frame_final_marker_advances_only_sequence_and_final_state() {
        let mut progress = StreamProgress::default();
        progress
            .accept(
                &chunk(0, 0, true, false, Some(3), &[(4_096, 3)], 4_096),
                3,
                22_050,
            )
            .expect("initial audio");
        let marker = chunk(1, 4_096, false, true, Some(3), &[], 0);
        let accepted = progress
            .accept(&marker, 3, 22_050)
            .expect("zero-frame final marker");

        assert!(accepted.alignment_events.is_empty());
        assert_eq!(progress.native_next_sequence, 2);
        assert_eq!(progress.native_next_sample_index, 4_096);
        assert_eq!(progress.committed_text_tokens, 3);
        assert!(progress.final_chunk_seen);

        let mut invalid_progress = StreamProgress::default();
        invalid_progress
            .accept(
                &chunk(0, 0, true, false, Some(3), &[(4_096, 3)], 4_096),
                3,
                22_050,
            )
            .expect("initial audio");
        let invalid = chunk(1, 4_096, false, false, Some(3), &[], 0);
        assert!(invalid_progress.accept(&invalid, 3, 22_050).is_err());
    }

    #[test]
    fn native_sequence_gap_is_rejected_without_mutating_progress() {
        let mut progress = StreamProgress::default();
        assert!(
            progress
                .accept(&chunk(1, 0, false, false, None, &[], 4), 3, 22_050)
                .is_err()
        );
        assert_eq!(progress.native_next_sequence, 0);
        assert_eq!(progress.published_next_sequence, 0);
    }

    #[test]
    fn cancellation_has_a_terminal_deadline_independent_of_audio_progress() {
        let started = Instant::now() - Duration::from_millis(20);
        assert!(cancellation_deadline_expired(
            Some(started),
            Duration::from_millis(10)
        ));
        assert!(!cancellation_deadline_expired(
            None,
            Duration::from_millis(10)
        ));
    }

    #[test]
    fn complete_steady_batch_is_emitted_on_its_arrival_call() {
        let mut logical = LogicalProgress::default();
        let initial = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-stream-timing",
                "テスト",
                &[],
                0,
                0,
                vec![0.0; 4 * 1_024],
                AcceptedChunk {
                    first: true,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("initial four frames");
        assert_eq!(initial.len(), 1);

        let steady = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-stream-timing",
                "テスト",
                &[],
                0,
                0,
                vec![0.0; 8 * 1_024],
                AcceptedChunk {
                    first: false,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("complete steady eight frames");
        let SchedulerOutput::Audio(steady) = &steady[0] else {
            panic!("steady arrival must emit audio");
        };
        assert_eq!(steady.sequence, 1);
        assert_eq!(steady.first_sample_index, 4 * 1_024);
        assert_eq!(steady.pcm_f32.len(), 8 * 1_024);
        assert!(!steady.final_chunk);
        assert!(logical.pending_samples.is_empty());

        let residual = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-stream-timing",
                "テスト",
                &[],
                0,
                0,
                vec![0.0; 1_024],
                AcceptedChunk {
                    first: false,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("one-frame residual");
        assert!(residual.is_empty());
        assert_eq!(logical.pending_samples.len(), 1_024);
    }

    #[test]
    fn two_independent_segments_form_one_exact_logical_ros_stream() {
        let source_text = "一。二。";
        let source_progress = (0_u64..=6)
            .map(|position| SourceProgress {
                committed_text_tokens: position,
                source_char_end: position.min(4),
                source_utf8_end: position.min(4) * 3,
            })
            .collect::<Vec<_>>();
        let mut logical = LogicalProgress::default();
        let first = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                0,
                0,
                vec![0.0; 4 * 1_024],
                AcceptedChunk {
                    first: true,
                    alignment_events: vec![AlignmentPoint {
                        sample_index: 4_096,
                        committed_text_tokens: 2,
                    }],
                },
                22_050,
            )
            .expect("first segment initial chunk");
        let intermediate_final = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                0,
                0,
                vec![0.0; 3 * 1_024],
                AcceptedChunk {
                    first: false,
                    alignment_events: vec![AlignmentPoint {
                        sample_index: 7_168,
                        committed_text_tokens: 3,
                    }],
                },
                22_050,
            )
            .expect("intermediate native PCM tail");
        assert!(intermediate_final.is_empty());
        let intermediate_final_marker = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                0,
                0,
                Vec::new(),
                AcceptedChunk {
                    first: false,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("intermediate native zero-frame FINAL");
        assert!(intermediate_final_marker.is_empty());

        let second_initial = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                7_168,
                3,
                vec![0.0; 4 * 1_024],
                AcceptedChunk {
                    first: true,
                    alignment_events: vec![AlignmentPoint {
                        sample_index: 4_096,
                        committed_text_tokens: 2,
                    }],
                },
                22_050,
            )
            .expect("second segment initial chunk");
        assert!(second_initial.is_empty());
        let steady = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                7_168,
                3,
                vec![0.0; 4 * 1_024],
                AcceptedChunk {
                    first: false,
                    alignment_events: vec![AlignmentPoint {
                        sample_index: 8_192,
                        committed_text_tokens: 3,
                    }],
                },
                22_050,
            )
            .expect("second segment native PCM tail");
        let last_final_marker = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                7_168,
                3,
                Vec::new(),
                AcceptedChunk {
                    first: false,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("last native zero-frame FINAL");
        assert!(last_final_marker.is_empty());
        let final_output = logical
            .emit_chunk(
                SpeechKind::Agent,
                "agent-4-long",
                source_text,
                &source_progress,
                logical.pending_samples.len(),
                true,
                22_050,
            )
            .expect("logical final tail");

        let mut audio = first
            .into_iter()
            .chain(steady)
            .map(|output| match output {
                SchedulerOutput::Audio(speech) => speech,
                _ => panic!("expected audio"),
            })
            .collect::<Vec<_>>();
        audio.push(final_output);
        assert_eq!(
            audio.iter().map(|item| item.sequence).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );
        assert_eq!(
            audio
                .iter()
                .map(|item| item.first_sample_index)
                .collect::<Vec<_>>(),
            vec![0, 4_096, 12_288]
        );
        assert_eq!(
            audio
                .iter()
                .map(|item| item.pcm_f32.len())
                .collect::<Vec<_>>(),
            vec![4 * 1_024, 8 * 1_024, 3 * 1_024]
        );
        assert_eq!(
            audio
                .iter()
                .map(|item| item.final_chunk)
                .collect::<Vec<_>>(),
            vec![false, false, true]
        );
        assert_eq!(audio[0].source_text, source_text);
        assert_eq!(audio[0].source_progress, source_progress);
        assert!(audio[1].source_text.is_empty());
        assert_eq!(audio[2].committed_text_tokens, 6);
        assert_eq!(audio[2].alignment_events[0].sample_index, 15_360);
    }

    #[test]
    fn logical_stream_uses_a_zero_frame_final_when_no_pcm_tail_is_pending() {
        let source_progress = vec![
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
        ];
        let mut logical = LogicalProgress::default();
        let first = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-short",
                "あ",
                &source_progress,
                0,
                0,
                vec![0.0; 4 * 1_024],
                AcceptedChunk {
                    first: true,
                    alignment_events: vec![AlignmentPoint {
                        sample_index: 4_096,
                        committed_text_tokens: 1,
                    }],
                },
                22_050,
            )
            .expect("initial audio");
        let native_marker = logical
            .accept_audio(
                SpeechKind::Agent,
                "agent-4-short",
                "あ",
                &source_progress,
                0,
                0,
                Vec::new(),
                AcceptedChunk {
                    first: false,
                    alignment_events: Vec::new(),
                },
                22_050,
            )
            .expect("native marker");
        assert!(native_marker.is_empty());
        let marker = logical
            .emit_final_marker(SpeechKind::Agent, "agent-4-short", 22_050)
            .expect("logical marker");

        assert_eq!(first.len(), 1);
        assert_eq!(marker.sequence, 1);
        assert_eq!(marker.first_sample_index, 4_096);
        assert!(marker.pcm_f32.is_empty());
        assert!(marker.final_chunk);
        assert_eq!(marker.committed_text_tokens, 1);
        assert_eq!(logical.next_sample_index, 4_096);
    }

    #[test]
    fn cancellation_at_a_segment_boundary_never_starts_the_next_segment() {
        assert!(!should_advance_segment(
            Some(&Cancellation::Cancelled),
            0,
            2
        ));
        assert!(should_advance_segment(None, 0, 2));
    }

    #[test]
    fn later_segment_start_failure_has_one_logical_abort_and_result() {
        let request = SayRequest {
            kind: SpeechKind::System,
            utterance_id: "system-long".to_owned(),
            text: "一。二。".to_owned(),
        };
        let outputs = streaming_protocol_failure(
            &request,
            2,
            12_288,
            22_050,
            &super::SchedulerError::Protocol(
                "second independent segment could not start".to_owned(),
            ),
        );
        assert_eq!(
            outputs
                .iter()
                .filter(|output| matches!(output, SchedulerOutput::Terminal { .. }))
                .count(),
            1
        );
        assert!(matches!(
            &outputs[0],
            SchedulerOutput::Terminal {
                speech: SpeechOutput {
                    event: SpeechEvent::Abort,
                    sequence: 2,
                    first_sample_index: 12_288,
                    ..
                },
                result: ResultOutput {
                    status: ResultStatus::Failed,
                    ..
                }
            }
        ));
        assert!(matches!(&outputs[1], SchedulerOutput::Fatal(_)));
    }

    fn chunk(
        sequence: u64,
        first_sample_index: u64,
        first: bool,
        final_chunk: bool,
        committed_text_tokens: Option<u64>,
        alignments: &[(u64, u64)],
        sample_count: usize,
    ) -> OwnedAudioChunk {
        OwnedAudioChunk {
            sequence,
            first_sample_index,
            sample_rate_hz: 22_050,
            first,
            final_chunk,
            committed_text_tokens,
            alignment_events: alignments
                .iter()
                .map(|(sample_index, committed_text_tokens)| AlignmentEvent {
                    sample_index: *sample_index,
                    committed_text_tokens: *committed_text_tokens,
                })
                .collect(),
            samples: vec![0.0; sample_count],
        }
    }
}
