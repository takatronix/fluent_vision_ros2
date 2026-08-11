use std::collections::{HashMap, HashSet, VecDeque};
use std::error::Error;
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use fv_tts_ros2::contracts::{
    PlaybackCommand, ResultStatus, SayRequest, SpeechKind, decode_sha256, encode_sha256,
    parse_playback_control, validate_uuid,
};
use fv_tts_ros2::frontend::FrontendConfig;
use fv_tts_ros2::scheduler::{
    ReadyInfo, ResultOutput, SchedulerConfig, SchedulerHandle, SchedulerOutput, SchedulerSendError,
    SpeechEvent, SpeechOutput, TimingOutput, TimingStage,
};
use r2r::fv_audio_interfaces::msg::{
    PlaybackControl, SynthesizedSpeechChunk, TtsAlignmentEvent, TtsResult, TtsRuntimeState, TtsSay,
    TtsSourceProgress, TtsTimingEvent,
};
use r2r::fv_audio_interfaces::srv::QuiesceTts;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::std_srvs::srv::Trigger;
use r2r::{Context, Node, QosProfile};

const SAY_TOPIC: &str = "/aspa/tts/say";
const RESULT_TOPIC: &str = "/aspa/tts/result";
const STATUS_TOPIC: &str = "/aspa/tts/status";
const TIMING_TOPIC: &str = "/aspa/tts/timing";
const READY_SERVICE: &str = "/aspa/tts/ready";
const QUIESCE_SERVICE: &str = "/aspa/tts/quiesce";
const CONTROL_TOPIC: &str = "/audio/playback/control";
const AGENT_TOPIC: &str = "/audio/agent/frame";
const SYSTEM_TOPIC: &str = "/audio/system/frame";
const VOICE_NAME: &str = "Sofia";
const IDLE_ROS_WAIT_INTERVAL: Duration = Duration::from_millis(20);
const ACTIVE_ROS_WAIT_INTERVAL: Duration = Duration::from_millis(2);
const PLAYBACK_CONNECTIVITY_POLL_INTERVAL: Duration = Duration::from_millis(20);
const TIMING_COLLECTOR_DISCOVERY_POLL_INTERVAL: Duration = Duration::from_millis(2);
const TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL: Duration = Duration::from_millis(20);
const STATUS_HEARTBEAT_INTERVAL: Duration = Duration::from_millis(500);
const MAX_RECENT_REQUEST_IDS: usize = 1024;

#[derive(Clone, Debug)]
struct RequestIdentity {
    kind: SpeechKind,
    request_id: String,
    generation_id: String,
    utterance_id: String,
}

#[derive(Default)]
struct RequestLedger {
    in_flight: HashMap<String, RequestIdentity>,
    live_request_ids: HashSet<String>,
    recent_request_ids: HashSet<String>,
    recent_request_order: VecDeque<String>,
}

impl RequestLedger {
    fn register(
        &mut self,
        message: TtsSay,
        runtime_generation_id: &str,
    ) -> Result<(SayRequest, RequestIdentity), String> {
        validate_uuid(&message.request_id, "request_id").map_err(|error| error.to_string())?;
        validate_uuid(&message.generation_id, "generation_id")
            .map_err(|error| error.to_string())?;
        if message.generation_id != runtime_generation_id {
            return Err(format!(
                "SAY generation {} does not match runtime generation {runtime_generation_id}",
                message.generation_id
            ));
        }
        if self.live_request_ids.contains(&message.request_id)
            || self.recent_request_ids.contains(&message.request_id)
        {
            return Err(
                "request_id is already live or inside the terminal reorder window".to_owned(),
            );
        }
        if self.in_flight.contains_key(&message.utterance_id) {
            return Err("utterance_id is already in flight".to_owned());
        }
        let request =
            SayRequest::from_fields(message.kind, message.utterance_id.clone(), message.text)
                .map_err(|error| error.to_string())?;
        let identity = RequestIdentity {
            kind: request.kind,
            request_id: message.request_id,
            generation_id: message.generation_id,
            utterance_id: request.utterance_id.clone(),
        };
        self.live_request_ids.insert(identity.request_id.clone());
        self.in_flight
            .insert(identity.utterance_id.clone(), identity.clone());
        Ok((request, identity))
    }

    fn identity(&self, utterance_id: &str) -> Result<&RequestIdentity, String> {
        self.in_flight.get(utterance_id).ok_or_else(|| {
            format!("scheduler output has no request identity for utterance {utterance_id}")
        })
    }

    fn finish(&mut self, utterance_id: &str) -> Result<RequestIdentity, String> {
        let identity = self.in_flight.remove(utterance_id).ok_or_else(|| {
            format!("terminal scheduler output has no request identity for {utterance_id}")
        })?;
        if !self.live_request_ids.remove(&identity.request_id) {
            return Err(format!(
                "terminal request_id {} was not live",
                identity.request_id
            ));
        }
        if self.recent_request_order.len() >= MAX_RECENT_REQUEST_IDS {
            let expired = self
                .recent_request_order
                .pop_front()
                .expect("non-empty recent request order");
            self.recent_request_ids.remove(&expired);
        }
        self.recent_request_ids.insert(identity.request_id.clone());
        self.recent_request_order
            .push_back(identity.request_id.clone());
        Ok(identity)
    }

    fn len(&self) -> usize {
        self.in_flight.len()
    }

    fn is_empty(&self) -> bool {
        self.in_flight.is_empty()
    }
}

struct RuntimeStatus {
    message: TtsRuntimeState,
}

impl RuntimeStatus {
    fn starting(generation_id: String, config: &SchedulerConfig) -> Self {
        Self {
            message: TtsRuntimeState {
                version: 1,
                generation_id,
                heartbeat_sequence: 0,
                state: TtsRuntimeState::STARTING as u8,
                ready: false,
                process_id: std::process::id(),
                voice: VOICE_NAME.to_owned(),
                engine: "MagpieTTS-RT".to_owned(),
                bundle_manifest_sha256: encode_sha256(&config.expected_manifest_sha256),
                tokenizer_identity_sha256: String::new(),
                cuda_device_index: config.cuda_device_index,
                sample_rate_hz: 0,
                channels: 0,
                codec_frame_samples: 0,
                initial_frames: 0,
                steady_frames: 0,
                tail_min_frames: 0,
                tail_max_frames: 0,
                last_result_valid: false,
                last_result: empty_result(),
                error: String::new(),
            },
        }
    }

    fn set_ready(&mut self, info: &ReadyInfo) {
        self.message.state = TtsRuntimeState::READY as u8;
        self.message.ready = true;
        self.message.tokenizer_identity_sha256 =
            encode_sha256(&info.model.tokenizer_identity_sha256);
        self.message.sample_rate_hz = info.model.sample_rate_hz;
        self.message.channels = info.model.channels;
        self.message.codec_frame_samples = info.model.codec_frame_samples;
        self.message.initial_frames = info.model.initial_frames;
        self.message.steady_frames = info.model.steady_frames;
        self.message.tail_min_frames = info.model.tail_min_frames;
        self.message.tail_max_frames = info.model.tail_max_frames;
        self.message.error.clear();
    }

    fn set_terminal_state(&mut self, state: u8, error: Option<String>) {
        self.message.state = state;
        self.message.ready = false;
        self.message.error = error.unwrap_or_default();
    }

    fn record_result(&mut self, result: TtsResult) {
        self.message.last_result_valid = true;
        self.message.last_result = result;
    }

    fn heartbeat(&mut self) -> Result<TtsRuntimeState, Box<dyn Error>> {
        self.message.heartbeat_sequence = self
            .message
            .heartbeat_sequence
            .checked_add(1)
            .ok_or("TTS status heartbeat sequence overflowed")?;
        Ok(self.message.clone())
    }
}

fn empty_result() -> TtsResult {
    TtsResult {
        kind: 0,
        status: 0,
        request_id: String::new(),
        generation_valid: false,
        generation_id: String::new(),
        utterance_id: String::new(),
        error: String::new(),
    }
}

fn process_generation_id() -> Result<String, Box<dyn Error>> {
    let generation_id = fs::read_to_string("/proc/sys/kernel/random/uuid")?;
    let generation_id = generation_id.trim().to_owned();
    validate_uuid(&generation_id, "generation_id")?;
    Ok(generation_id)
}

fn main() -> Result<(), Box<dyn Error>> {
    let context = Context::create()?;
    let mut node = Node::create(context, "fv_tts", "")?;
    let config = scheduler_config(&node)?;
    let scheduler_watchdog = config.scheduler_watchdog;
    let timing_collector_discovery_timeout =
        positive_duration_parameter(&node, "timing_collector_discovery_timeout_seconds")?;
    let quiesce_timeout = config.cancellation_terminal_timeout;
    let generation_id = process_generation_id()?;
    let qos = reliable_volatile_qos(32);
    let mut say_messages = node.subscribe::<TtsSay>(SAY_TOPIC, qos.clone())?;
    let mut controls = node.subscribe::<PlaybackControl>(CONTROL_TOPIC, qos.clone())?;
    let agent_publisher =
        node.create_publisher::<SynthesizedSpeechChunk>(AGENT_TOPIC, qos.clone())?;
    let system_publisher =
        node.create_publisher::<SynthesizedSpeechChunk>(SYSTEM_TOPIC, qos.clone())?;
    let result_publisher = node.create_publisher::<TtsResult>(RESULT_TOPIC, qos.clone())?;
    let timing_publisher = node.create_publisher::<TtsTimingEvent>(TIMING_TOPIC, qos)?;
    let status_publisher = node.create_publisher::<TtsRuntimeState>(STATUS_TOPIC, state_qos())?;

    let publishers = Publishers {
        agent: &agent_publisher,
        system: &system_publisher,
        result: &result_publisher,
        status: &status_publisher,
        timing: &timing_publisher,
    };
    let mut status = RuntimeStatus::starting(generation_id.clone(), &config);
    publish_status(&status_publisher, &mut status)?;
    let mut timing_collector = TimingCollectorConnectivity::new();
    if let Err(error) = wait_for_timing_collector(
        &mut node,
        &timing_publisher,
        &mut timing_collector,
        timing_collector_discovery_timeout,
    ) {
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
        publish_status(&status_publisher, &mut status)?;
        return Err(error);
    }
    let (scheduler, ready_info) = match SchedulerHandle::spawn(config) {
        Ok(ready) => ready,
        Err(error) => {
            status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
            publish_status(&status_publisher, &mut status)?;
            return Err(error.into());
        }
    };
    let mut playback_connectivity = PlaybackConnectivity::new();
    playback_connectivity.refresh(&publishers, &scheduler, true)?;
    if let Err(error) = timing_collector.require_available(&timing_publisher) {
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
        publish_status(&status_publisher, &mut status)?;
        let shutdown = scheduler.shutdown(scheduler_watchdog);
        if !shutdown.outputs.is_empty() {
            return Err(
                "TTS scheduler produced output before the runtime admitted any request".into(),
            );
        }
        shutdown.result?;
        return Err(error);
    }
    status.set_ready(&ready_info);
    publish_status(&status_publisher, &mut status)?;
    // Readiness and quiesce services are intentionally absent while native
    // session creation, the golden generation, and frontend identity checks
    // are still in progress. ROS graph presence must never be mistaken for a
    // successfully admitted MagpieTTS-RT generation.
    let mut ready_requests =
        node.create_service::<Trigger::Service>(READY_SERVICE, QosProfile::default())?;
    let mut quiesce_requests =
        node.create_service::<QuiesceTts::Service>(QUIESCE_SERVICE, QosProfile::default())?;
    eprintln!(
        "fv_tts ready: generation={} engine=MagpieTTS-RT voice={VOICE_NAME} cuda_device={} bundle_manifest_sha256={} tokenizer_identity_sha256={}",
        generation_id,
        ready_info.cuda_device_index,
        encode_sha256(&ready_info.expected_manifest_sha256),
        encode_sha256(&ready_info.model.tokenizer_identity_sha256),
    );

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    let mut ledger = RequestLedger::default();

    let run_result = run_ros_loop(
        &mut node,
        &scheduler,
        &mut say_messages,
        &mut controls,
        &mut ready_requests,
        &mut quiesce_requests,
        &publishers,
        &mut playback_connectivity,
        &mut timing_collector,
        &mut status,
        &mut ledger,
        &running,
        scheduler_watchdog,
        quiesce_timeout,
    );
    let shutdown = scheduler.shutdown(scheduler_watchdog);
    let shutdown_output_result =
        publish_shutdown_outputs(shutdown.outputs, &publishers, &mut status, &mut ledger);
    let shutdown_result = shutdown.result;
    if let Err(error) = &run_result {
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
    } else if let Err(error) = &shutdown_output_result {
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
    } else if let Err(error) = &shutdown_result {
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(error.to_string()));
    } else {
        status.set_terminal_state(TtsRuntimeState::STOPPED as u8, None);
    }
    let final_status_result = publish_status(&status_publisher, &mut status);
    run_result?;
    shutdown_output_result?;
    shutdown_result?;
    final_status_result?;
    Ok(())
}

fn publish_shutdown_outputs(
    outputs: Vec<SchedulerOutput>,
    publishers: &Publishers<'_>,
    status: &mut RuntimeStatus,
    ledger: &mut RequestLedger,
) -> Result<(), Box<dyn Error>> {
    for output in outputs {
        match output {
            SchedulerOutput::Audio(speech) => {
                let identity = ledger.identity(&speech.utterance_id)?;
                publish_speech(publishers, identity, speech)?;
            }
            SchedulerOutput::Timing(timing) => {
                publish_scheduler_timing(publishers, ledger, timing)?;
            }
            SchedulerOutput::Terminal { speech, result } => (|| {
                let identity = ledger.identity(&speech.utterance_id)?.clone();
                publish_speech(publishers, &identity, speech)?;
                let result_message = result_message(&identity, &result)?;
                publishers.result.publish(&result_message)?;
                status.record_result(result_message);
                ledger.finish(&result.utterance_id)?;
                publish_status(publishers.status, status)
            })()?,
            SchedulerOutput::Fatal(message) => {
                return Err(format!("fv_tts scheduler failed during shutdown: {message}").into());
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn run_ros_loop(
    node: &mut Node,
    scheduler: &SchedulerHandle,
    say_messages: &mut (impl futures::Stream<Item = TtsSay> + Unpin),
    controls: &mut (impl futures::Stream<Item = PlaybackControl> + Unpin),
    ready_requests: &mut (impl futures::Stream<Item = r2r::ServiceRequest<Trigger::Service>> + Unpin),
    quiesce_requests: &mut (
             impl futures::Stream<Item = r2r::ServiceRequest<QuiesceTts::Service>> + Unpin
         ),
    publishers: &Publishers<'_>,
    playback_connectivity: &mut PlaybackConnectivity,
    timing_collector: &mut TimingCollectorConnectivity,
    status: &mut RuntimeStatus,
    ledger: &mut RequestLedger,
    running: &AtomicBool,
    scheduler_watchdog: Duration,
    quiesce_timeout: Duration,
) -> Result<(), Box<dyn Error>> {
    let mut next_status_heartbeat = Instant::now() + STATUS_HEARTBEAT_INTERVAL;
    let mut pending_quiesce = None;
    while running.load(Ordering::SeqCst) {
        // ROS input wakes rcl_wait immediately. Only the no-request case uses
        // the longer timeout; once a request is accepted, the short interval
        // bounds delivery latency for scheduler output, which is not part of
        // the ROS wait set.
        node.spin_once(runtime_spin_interval(ledger.is_empty()));
        require_runtime_timing_collector(
            timing_collector,
            publishers,
            playback_connectivity,
            scheduler,
            status,
        )?;
        if status.message.ready {
            playback_connectivity.refresh(publishers, scheduler, false)?;
        }

        while let Some(Some(request)) = ready_requests.next().now_or_never() {
            request.respond(Trigger::Response {
                success: status.message.ready,
                message: format!(
                    "generation={} state={} bundle_manifest_sha256={}",
                    status.message.generation_id,
                    status.message.state,
                    status.message.bundle_manifest_sha256
                ),
            })?;
        }

        while let Some(Some(request)) = quiesce_requests.next().now_or_never() {
            if pending_quiesce.is_some() {
                request.respond(QuiesceTts::Response {
                    success: false,
                    generation_id: status.message.generation_id.clone(),
                    cancelled_requests: 0,
                    error: "another quiesce request is already pending".to_owned(),
                })?;
                continue;
            }
            let requested_generation = request.message.generation_id.clone();
            let invalid = validate_uuid(&requested_generation, "generation_id")
                .map_err(|error| error.to_string())
                .and_then(|_| {
                    (requested_generation == status.message.generation_id)
                        .then_some(())
                        .ok_or_else(|| {
                            format!(
                                "requested generation {requested_generation} is not runtime generation {}",
                                status.message.generation_id
                            )
                        })
                });
            if let Err(error) = invalid {
                request.respond(QuiesceTts::Response {
                    success: false,
                    generation_id: status.message.generation_id.clone(),
                    cancelled_requests: 0,
                    error,
                })?;
                continue;
            }
            status.set_terminal_state(TtsRuntimeState::QUIESCING as u8, None);
            publish_status(publishers.status, status)?;
            playback_connectivity.force_unavailable(scheduler)?;
            let cancelled_requests = u32::try_from(ledger.len())
                .map_err(|_| "in-flight TTS request count exceeds uint32")?;
            pending_quiesce = Some((
                request,
                Instant::now() + quiesce_timeout,
                cancelled_requests,
            ));
        }

        while let Some(Some(message)) = controls.next().now_or_never() {
            let control = parse_playback_control(
                message.action,
                message.release_hold,
                message.minimum_agent_epoch,
                &message.utterance_id,
            )?;
            if control != PlaybackCommand::Ignore
                && should_forward_playback_control(status.message.ready, &control)
            {
                scheduler.try_control(control)?;
            }
        }

        while let Some(Some(message)) = say_messages.next().now_or_never() {
            require_runtime_timing_collector(
                timing_collector,
                publishers,
                playback_connectivity,
                scheduler,
                status,
            )?;
            if !status.message.ready {
                publish_rejected(
                    publishers.result,
                    &message,
                    "TTS runtime is not admitting requests",
                )?;
                continue;
            }
            let (request, identity) =
                match ledger.register(message.clone(), &status.message.generation_id) {
                    Ok(request) => request,
                    Err(error) => {
                        publish_rejected(publishers.result, &message, &error)?;
                        continue;
                    }
                };
            // Sample immediately before the bounded scheduler handoff. The
            // event is published only after that handoff succeeds, while this
            // pre-send timestamp remains causally earlier than frontend work
            // that may begin on the scheduler thread as soon as send returns.
            let accepted_time_ns = monotonic_time_ns()?;
            if let Err(error) = scheduler.try_submit(request.clone()) {
                publish_overload_terminal(
                    &request,
                    &identity,
                    error,
                    status.message.sample_rate_hz,
                    publishers,
                    status,
                    ledger,
                )?;
                return Err("fv_tts bounded request queue overflowed".into());
            }
            publish_timing(
                publishers.timing,
                &identity,
                TtsTimingEvent::REQUEST_ACCEPTED as u8,
                accepted_time_ns,
            )?;
        }

        while let Some(output) = scheduler.try_output()? {
            publish_scheduler_output(output, publishers, status, ledger)?;
        }

        if pending_quiesce.is_some() && ledger.is_empty() {
            let (request, _, cancelled_requests) = pending_quiesce.take().expect("quiesce checked");
            request.respond(QuiesceTts::Response {
                success: true,
                generation_id: status.message.generation_id.clone(),
                cancelled_requests,
                error: String::new(),
            })?;
        } else if pending_quiesce
            .as_ref()
            .is_some_and(|(_, deadline, _)| Instant::now() >= *deadline)
        {
            let (request, _, cancelled_requests) = pending_quiesce.take().expect("quiesce checked");
            let error = format!(
                "TTS quiesce timed out with {} request(s) still in flight",
                ledger.len()
            );
            request.respond(QuiesceTts::Response {
                success: false,
                generation_id: status.message.generation_id.clone(),
                cancelled_requests,
                error: error.clone(),
            })?;
            return Err(error.into());
        }

        if Instant::now() >= next_status_heartbeat {
            publish_status(publishers.status, status)?;
            next_status_heartbeat = Instant::now() + STATUS_HEARTBEAT_INTERVAL;
        }

        if scheduler.heartbeat_age()? > scheduler_watchdog {
            eprintln!(
                "fv_tts fatal: scheduler heartbeat exceeded {:.3} seconds; terminating without waiting for a blocked native thread",
                scheduler_watchdog.as_secs_f64()
            );
            std::process::exit(1);
        }
    }
    Ok(())
}

fn runtime_spin_interval(request_ledger_empty: bool) -> Duration {
    if request_ledger_empty {
        IDLE_ROS_WAIT_INTERVAL
    } else {
        ACTIVE_ROS_WAIT_INTERVAL
    }
}

fn should_forward_playback_control(runtime_ready: bool, control: &PlaybackCommand) -> bool {
    runtime_ready
        || !matches!(
            control,
            PlaybackCommand::SetPlaybackAvailable {
                kind: _,
                available: _,
            }
        )
}

fn publish_overload_terminal(
    request: &SayRequest,
    identity: &RequestIdentity,
    error: SchedulerSendError,
    sample_rate_hz: u32,
    publishers: &Publishers<'_>,
    status: &mut RuntimeStatus,
    ledger: &mut RequestLedger,
) -> Result<(), Box<dyn Error>> {
    let error_message = error.to_string();
    publish_speech(
        publishers,
        identity,
        SpeechOutput {
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
    )?;
    let output = ResultOutput {
        kind: request.kind,
        utterance_id: request.utterance_id.clone(),
        status: ResultStatus::Failed,
        error: Some(error_message),
    };
    let result = result_message(identity, &output)?;
    publishers.result.publish(&result)?;
    status.record_result(result);
    ledger.finish(&request.utterance_id)?;
    publish_status(publishers.status, status)
}

fn publish_scheduler_output(
    output: SchedulerOutput,
    publishers: &Publishers<'_>,
    status: &mut RuntimeStatus,
    ledger: &mut RequestLedger,
) -> Result<(), Box<dyn Error>> {
    match output {
        SchedulerOutput::Audio(speech) => {
            let identity = ledger.identity(&speech.utterance_id)?;
            publish_speech(publishers, identity, speech)
        }
        SchedulerOutput::Timing(timing) => publish_scheduler_timing(publishers, ledger, timing),
        SchedulerOutput::Terminal { speech, result } => {
            let identity = ledger.identity(&speech.utterance_id)?.clone();
            if result.utterance_id != speech.utterance_id {
                return Err(format!(
                    "terminal speech/result utterance mismatch: {} != {}",
                    speech.utterance_id, result.utterance_id
                )
                .into());
            }
            publish_speech(publishers, &identity, speech)?;
            let message = result_message(&identity, &result)?;
            publishers.result.publish(&message)?;
            status.record_result(message);
            ledger.finish(&result.utterance_id)?;
            publish_status(publishers.status, status)
        }
        SchedulerOutput::Fatal(message) => {
            Err(format!("fv_tts scheduler failed: {message}").into())
        }
    }
}

fn publish_rejected(
    publisher: &r2r::Publisher<TtsResult>,
    message: &TtsSay,
    error: &str,
) -> Result<(), Box<dyn Error>> {
    validate_uuid(&message.request_id, "request_id")?;
    validate_uuid(&message.generation_id, "generation_id")?;
    let kind = SpeechKind::from_ros(message.kind)?;
    if message.utterance_id.is_empty() {
        return Err("cannot link a rejection to an empty utterance_id".into());
    }
    if error.is_empty() {
        return Err("TTS rejection must contain a reason".into());
    }
    publisher.publish(&TtsResult {
        kind: kind.ros_value(),
        status: TtsResult::REJECTED as u8,
        request_id: message.request_id.clone(),
        generation_valid: true,
        generation_id: message.generation_id.clone(),
        utterance_id: message.utterance_id.clone(),
        error: error.to_owned(),
    })?;
    Ok(())
}

fn result_message(
    identity: &RequestIdentity,
    output: &ResultOutput,
) -> Result<TtsResult, Box<dyn Error>> {
    if output.kind != identity.kind || output.utterance_id != identity.utterance_id {
        return Err(format!(
            "scheduler result identity mismatch for request {}",
            identity.request_id
        )
        .into());
    }
    let error = match output.status {
        ResultStatus::Failed => output
            .error
            .clone()
            .filter(|message| !message.is_empty())
            .ok_or("failed TTS terminal is missing its error")?,
        ResultStatus::Completed | ResultStatus::Cancelled => {
            if output.error.is_some() {
                return Err(
                    "successful/cancelled TTS terminal unexpectedly contains an error".into(),
                );
            }
            String::new()
        }
    };
    Ok(TtsResult {
        kind: identity.kind.ros_value(),
        status: output.status.ros_value(),
        request_id: identity.request_id.clone(),
        generation_valid: true,
        generation_id: identity.generation_id.clone(),
        utterance_id: identity.utterance_id.clone(),
        error,
    })
}

fn publish_speech(
    publishers: &Publishers<'_>,
    identity: &RequestIdentity,
    speech: SpeechOutput,
) -> Result<(), Box<dyn Error>> {
    let (kind, message) = into_ros_speech_message(identity, speech)?;
    let first_audio = message.event == SynthesizedSpeechChunk::AUDIO as u8 && message.sequence == 0;
    // Capture the causal boundary before the data publish. A downstream process
    // can receive the PCM before this process returns from publish(), so taking
    // the timestamp afterwards can invert adjacent pipeline stages.
    let first_audio_time_ns = if first_audio {
        Some(monotonic_time_ns()?)
    } else {
        None
    };
    match kind {
        SpeechKind::Agent => publishers.agent.publish(&message)?,
        SpeechKind::System => publishers.system.publish(&message)?,
    }
    if let Some(first_audio_time_ns) = first_audio_time_ns {
        publish_timing(
            publishers.timing,
            identity,
            TtsTimingEvent::FIRST_ROS_AUDIO_PUBLISHED as u8,
            first_audio_time_ns,
        )?;
    }
    Ok(())
}

fn publish_scheduler_timing(
    publishers: &Publishers<'_>,
    ledger: &RequestLedger,
    timing: TimingOutput,
) -> Result<(), Box<dyn Error>> {
    let identity = ledger.identity(&timing.utterance_id)?;
    if identity.kind != timing.kind {
        return Err(format!(
            "scheduler timing kind differs from request {}",
            identity.request_id
        )
        .into());
    }
    let stage = match timing.stage {
        TimingStage::FrontendCompleted => TtsTimingEvent::FRONTEND_COMPLETED as u8,
        TimingStage::NativeRequestStarted => TtsTimingEvent::NATIVE_REQUEST_STARTED as u8,
        TimingStage::FirstNativeAudio => TtsTimingEvent::FIRST_NATIVE_AUDIO as u8,
    };
    publish_timing(publishers.timing, identity, stage, timing.monotonic_time_ns)
}

fn publish_timing(
    publisher: &r2r::Publisher<TtsTimingEvent>,
    identity: &RequestIdentity,
    stage: u8,
    monotonic_time_ns: u64,
) -> Result<(), Box<dyn Error>> {
    if monotonic_time_ns == 0 {
        return Err("TTS timing event has a zero monotonic timestamp".into());
    }
    publisher.publish(&TtsTimingEvent {
        version: TtsTimingEvent::TTS_TIMING_SCHEMA_VERSION as u32,
        stage,
        kind: identity.kind.ros_value(),
        request_id: identity.request_id.clone(),
        generation_id: identity.generation_id.clone(),
        utterance_id: identity.utterance_id.clone(),
        monotonic_time_ns,
        underrun_frames: 0,
    })?;
    Ok(())
}

fn monotonic_time_ns() -> Result<u64, Box<dyn Error>> {
    let mut timestamp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: timestamp points to initialized writable storage and
    // CLOCK_MONOTONIC is shared by every process on this Linux host.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut timestamp) } != 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    if timestamp.tv_sec < 0 || !(0..1_000_000_000).contains(&timestamp.tv_nsec) {
        return Err("CLOCK_MONOTONIC returned an invalid timespec".into());
    }
    let seconds = u64::try_from(timestamp.tv_sec)?;
    let nanoseconds = u64::try_from(timestamp.tv_nsec)?;
    seconds
        .checked_mul(1_000_000_000)
        .and_then(|value| value.checked_add(nanoseconds))
        .ok_or_else(|| "CLOCK_MONOTONIC nanoseconds overflow uint64".into())
}

fn into_ros_speech_message(
    identity: &RequestIdentity,
    speech: SpeechOutput,
) -> Result<(SpeechKind, SynthesizedSpeechChunk), Box<dyn Error>> {
    if speech.kind != identity.kind || speech.utterance_id != identity.utterance_id {
        return Err(format!(
            "scheduler speech identity mismatch for request {}",
            identity.request_id
        )
        .into());
    }
    let kind = speech.kind;
    let event = match speech.event {
        SpeechEvent::Audio => SynthesizedSpeechChunk::AUDIO as u8,
        SpeechEvent::Complete => SynthesizedSpeechChunk::COMPLETE as u8,
        SpeechEvent::Abort => SynthesizedSpeechChunk::ABORT as u8,
    };
    let message = SynthesizedSpeechChunk {
        kind: speech.kind.ros_value(),
        event,
        request_id: identity.request_id.clone(),
        generation_id: identity.generation_id.clone(),
        utterance_id: speech.utterance_id,
        sequence: speech.sequence,
        first_sample_index: speech.first_sample_index,
        sample_rate_hz: speech.sample_rate_hz,
        channels: speech.channels,
        pcm_f32: speech.pcm_f32,
        final_chunk: speech.final_chunk,
        committed_text_tokens: speech.committed_text_tokens,
        alignment_events: speech
            .alignment_events
            .into_iter()
            .map(|event| TtsAlignmentEvent {
                sample_index: event.sample_index,
                committed_text_tokens: event.committed_text_tokens,
            })
            .collect(),
        source_text: speech.source_text,
        source_progress: speech
            .source_progress
            .into_iter()
            .map(|progress| TtsSourceProgress {
                committed_text_tokens: progress.committed_text_tokens,
                source_char_end: progress.source_char_end,
                source_utf8_end: progress.source_utf8_end,
            })
            .collect(),
    };
    Ok((kind, message))
}

struct Publishers<'a> {
    agent: &'a r2r::Publisher<SynthesizedSpeechChunk>,
    system: &'a r2r::Publisher<SynthesizedSpeechChunk>,
    result: &'a r2r::Publisher<TtsResult>,
    status: &'a r2r::Publisher<TtsRuntimeState>,
    timing: &'a r2r::Publisher<TtsTimingEvent>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TimingCollectorState {
    AwaitingDiscovery,
    Available,
}

struct TimingCollectorConnectivity {
    discovered: bool,
    next_runtime_check: Instant,
}

impl TimingCollectorConnectivity {
    fn new() -> Self {
        Self::new_at(Instant::now())
    }

    fn new_at(now: Instant) -> Self {
        Self {
            discovered: false,
            next_runtime_check: now,
        }
    }

    fn observe_subscription_count(
        &mut self,
        inter_process_subscription_count: usize,
    ) -> Result<TimingCollectorState, String> {
        if inter_process_subscription_count > 0 {
            self.discovered = true;
            return Ok(TimingCollectorState::Available);
        }
        if self.discovered {
            return Err(
                "required /aspa/tts/timing inter-process collector subscriber disappeared"
                    .to_owned(),
            );
        }
        Ok(TimingCollectorState::AwaitingDiscovery)
    }

    fn require_available(
        &mut self,
        publisher: &r2r::Publisher<TtsTimingEvent>,
    ) -> Result<(), Box<dyn Error>> {
        let count = publisher.get_inter_process_subscription_count()?;
        match self.observe_subscription_count(count)? {
            TimingCollectorState::Available => Ok(()),
            TimingCollectorState::AwaitingDiscovery => {
                Err("required /aspa/tts/timing inter-process collector subscriber is absent".into())
            }
        }
    }

    fn require_runtime_available(
        &mut self,
        publisher: &r2r::Publisher<TtsTimingEvent>,
    ) -> Result<(), Box<dyn Error>> {
        let now = Instant::now();
        if !self.runtime_check_due(now) {
            return Ok(());
        }
        self.require_available(publisher)
    }

    fn runtime_check_due(&mut self, now: Instant) -> bool {
        if now < self.next_runtime_check {
            return false;
        }
        self.next_runtime_check = now + TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL;
        true
    }
}

fn wait_for_timing_collector(
    node: &mut Node,
    publisher: &r2r::Publisher<TtsTimingEvent>,
    connectivity: &mut TimingCollectorConnectivity,
    timeout: Duration,
) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + timeout;
    loop {
        node.spin_once(TIMING_COLLECTOR_DISCOVERY_POLL_INTERVAL);
        let count = publisher.get_inter_process_subscription_count()?;
        match connectivity.observe_subscription_count(count)? {
            TimingCollectorState::Available => return Ok(()),
            TimingCollectorState::AwaitingDiscovery if Instant::now() < deadline => {}
            TimingCollectorState::AwaitingDiscovery => {
                return Err(format!(
                    "required /aspa/tts/timing inter-process collector subscriber was not discovered within {:.3} seconds",
                    timeout.as_secs_f64()
                )
                .into());
            }
        }
    }
}

fn require_runtime_timing_collector(
    connectivity: &mut TimingCollectorConnectivity,
    publishers: &Publishers<'_>,
    playback_connectivity: &mut PlaybackConnectivity,
    scheduler: &SchedulerHandle,
    status: &mut RuntimeStatus,
) -> Result<(), Box<dyn Error>> {
    if let Err(error) = connectivity.require_runtime_available(publishers.timing) {
        let message = error.to_string();
        status.set_terminal_state(TtsRuntimeState::FAILED as u8, Some(message.clone()));
        publish_status(publishers.status, status)?;
        playback_connectivity.force_unavailable(scheduler)?;
        return Err(message.into());
    }
    Ok(())
}

struct PlaybackConnectivity {
    agent_available: Option<bool>,
    system_available: Option<bool>,
    next_check: Instant,
}

impl PlaybackConnectivity {
    fn new() -> Self {
        Self {
            agent_available: None,
            system_available: None,
            next_check: Instant::now(),
        }
    }

    fn refresh(
        &mut self,
        publishers: &Publishers<'_>,
        scheduler: &SchedulerHandle,
        force: bool,
    ) -> Result<(), Box<dyn Error>> {
        let now = Instant::now();
        if !force && now < self.next_check {
            return Ok(());
        }
        self.next_check = now + PLAYBACK_CONNECTIVITY_POLL_INTERVAL;
        let agent_available = publishers.agent.get_inter_process_subscription_count()? > 0;
        let system_available = publishers.system.get_inter_process_subscription_count()? > 0;
        Self::send_change(
            &mut self.agent_available,
            SpeechKind::Agent,
            agent_available,
            scheduler,
        )?;
        Self::send_change(
            &mut self.system_available,
            SpeechKind::System,
            system_available,
            scheduler,
        )?;
        Ok(())
    }

    fn force_unavailable(&mut self, scheduler: &SchedulerHandle) -> Result<(), SchedulerSendError> {
        scheduler.try_control(PlaybackCommand::SetPlaybackAvailable {
            kind: SpeechKind::Agent,
            available: false,
        })?;
        scheduler.try_control(PlaybackCommand::SetPlaybackAvailable {
            kind: SpeechKind::System,
            available: false,
        })?;
        self.agent_available = Some(false);
        self.system_available = Some(false);
        Ok(())
    }

    fn send_change(
        previous: &mut Option<bool>,
        kind: SpeechKind,
        available: bool,
        scheduler: &SchedulerHandle,
    ) -> Result<(), SchedulerSendError> {
        if *previous == Some(available) {
            return Ok(());
        }
        scheduler.try_control(PlaybackCommand::SetPlaybackAvailable { kind, available })?;
        *previous = Some(available);
        Ok(())
    }
}

fn publish_status(
    publisher: &r2r::Publisher<TtsRuntimeState>,
    status: &mut RuntimeStatus,
) -> Result<(), Box<dyn Error>> {
    publisher.publish(&status.heartbeat()?)?;
    Ok(())
}

fn scheduler_config(node: &Node) -> Result<SchedulerConfig, Box<dyn Error>> {
    let expected_manifest_sha256 = decode_sha256(
        &required_string_parameter(node, "manifest_sha256")?,
        "manifest_sha256",
    )?;
    let cuda_device = node.get_parameter::<i64>("cuda_device_index")?;
    let cuda_device_index =
        i32::try_from(cuda_device).map_err(|_| "cuda_device_index must fit int32")?;
    Ok(SchedulerConfig {
        native_library: required_absolute_path_parameter(node, "native_library")?,
        bundle_path: required_absolute_path_parameter(node, "bundle_path")?,
        expected_manifest_sha256,
        cuda_device_index,
        frontend: FrontendConfig {
            python_executable: required_absolute_path_parameter(node, "frontend_python")?,
            server_script: required_absolute_path_parameter(node, "frontend_server")?,
            oracle_lock: required_absolute_path_parameter(node, "frontend_lock")?,
            frontend_contract: required_absolute_path_parameter(node, "frontend_contract")?,
            response_timeout: positive_duration_parameter(node, "frontend_timeout_seconds")?,
        },
        request_progress_timeout: positive_duration_parameter(
            node,
            "request_progress_timeout_seconds",
        )?,
        cancellation_terminal_timeout: positive_duration_parameter(
            node,
            "cancellation_timeout_seconds",
        )?,
        startup_timeout: positive_duration_parameter(node, "startup_timeout_seconds")?,
        scheduler_watchdog: positive_duration_parameter(node, "scheduler_watchdog_seconds")?,
    })
}

fn required_string_parameter(node: &Node, name: &str) -> Result<String, Box<dyn Error>> {
    let value = node.get_parameter::<String>(name)?;
    if value.is_empty() {
        return Err(format!("ROS parameter {name} must not be empty").into());
    }
    Ok(value)
}

fn required_absolute_path_parameter(node: &Node, name: &str) -> Result<PathBuf, Box<dyn Error>> {
    let path = PathBuf::from(required_string_parameter(node, name)?);
    if !path.is_absolute() {
        return Err(format!("ROS parameter {name} must be an absolute path").into());
    }
    Ok(path)
}

fn positive_duration_parameter(node: &Node, name: &str) -> Result<Duration, Box<dyn Error>> {
    let seconds = node.get_parameter::<f64>(name)?;
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err(format!("{name} must be finite and positive").into());
    }
    Ok(Duration::from_secs_f64(seconds))
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

fn state_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ACTIVE_ROS_WAIT_INTERVAL, IDLE_ROS_WAIT_INTERVAL, PlaybackCommand, RequestIdentity,
        ResultOutput, ResultStatus, SpeechEvent, SpeechKind, SpeechOutput, SynthesizedSpeechChunk,
        TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL, TimingCollectorConnectivity, TimingCollectorState,
        TtsResult, into_ros_speech_message, result_message, runtime_spin_interval,
        should_forward_playback_control,
    };
    use std::time::{Duration, Instant};

    #[test]
    fn idle_ros_wait_blocks_until_input_without_delaying_active_pcm_delivery() {
        assert_eq!(runtime_spin_interval(true), IDLE_ROS_WAIT_INTERVAL);
        assert_eq!(runtime_spin_interval(false), ACTIVE_ROS_WAIT_INTERVAL);
        assert_eq!(
            IDLE_ROS_WAIT_INTERVAL,
            TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL
        );
        assert!(ACTIVE_ROS_WAIT_INTERVAL < IDLE_ROS_WAIT_INTERVAL);
    }

    #[test]
    fn timing_collector_is_required_before_readiness_and_loss_is_fatal() {
        let mut connectivity = TimingCollectorConnectivity::new();
        assert_eq!(
            connectivity
                .observe_subscription_count(0)
                .expect("initial absence remains an explicit startup wait"),
            TimingCollectorState::AwaitingDiscovery
        );
        assert_eq!(
            connectivity
                .observe_subscription_count(1)
                .expect("collector discovery opens the timing dependency"),
            TimingCollectorState::Available
        );
        assert_eq!(
            connectivity
                .observe_subscription_count(2)
                .expect("additional observability subscribers do not close the dependency"),
            TimingCollectorState::Available
        );
        assert!(
            connectivity
                .observe_subscription_count(0)
                .expect_err("losing every inter-process timing subscriber must fail closed")
                .contains("collector subscriber disappeared")
        );
    }

    #[test]
    fn runtime_timing_discovery_poll_has_a_fixed_twenty_millisecond_bound() {
        assert_eq!(
            TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL,
            Duration::from_millis(20)
        );
        assert!(TIMING_COLLECTOR_RUNTIME_POLL_INTERVAL <= Duration::from_millis(100));

        let start = Instant::now();
        let mut connectivity = TimingCollectorConnectivity::new_at(start);
        assert!(connectivity.runtime_check_due(start));
        assert!(!connectivity.runtime_check_due(start + Duration::from_millis(19)));
        assert!(connectivity.runtime_check_due(start + Duration::from_millis(20)));
    }

    #[test]
    fn quiescing_runtime_never_accepts_connectivity_reenable() {
        let reenable = PlaybackCommand::SetPlaybackAvailable {
            kind: SpeechKind::Agent,
            available: true,
        };
        let disable = PlaybackCommand::SetPlaybackAvailable {
            kind: SpeechKind::System,
            available: false,
        };
        let abort = PlaybackCommand::AbortAgent("agent-3-a".to_owned());

        assert!(should_forward_playback_control(true, &reenable));
        assert!(!should_forward_playback_control(false, &reenable));
        assert!(!should_forward_playback_control(false, &disable));
        assert!(should_forward_playback_control(false, &abort));
    }

    #[test]
    fn typed_result_preserves_request_and_generation_identity() {
        let identity = request_identity("agent-3-a");
        let result = result_message(
            &identity,
            &ResultOutput {
                kind: SpeechKind::Agent,
                utterance_id: "agent-3-a".to_owned(),
                status: ResultStatus::Completed,
                error: None,
            },
        )
        .expect("valid terminal");
        assert_eq!(result.status, TtsResult::COMPLETED as u8);
        assert_eq!(result.request_id, identity.request_id);
        assert_eq!(result.generation_id, identity.generation_id);
        assert!(result.generation_valid);
    }

    #[test]
    fn explicit_eight_frame_final_is_preserved_before_complete_on_ros() {
        let final_audio = SpeechOutput {
            kind: SpeechKind::Agent,
            event: SpeechEvent::Audio,
            utterance_id: "agent-4-final-eight".to_owned(),
            sequence: 1,
            first_sample_index: 4_096,
            sample_rate_hz: 22_050,
            channels: 1,
            pcm_f32: vec![0.0; 8 * 1_024],
            final_chunk: true,
            committed_text_tokens: 2,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        };
        let complete = SpeechOutput {
            kind: SpeechKind::Agent,
            event: SpeechEvent::Complete,
            utterance_id: "agent-4-final-eight".to_owned(),
            sequence: 2,
            first_sample_index: 12_288,
            sample_rate_hz: 22_050,
            channels: 1,
            pcm_f32: Vec::new(),
            final_chunk: false,
            committed_text_tokens: 0,
            alignment_events: Vec::new(),
            source_text: String::new(),
            source_progress: Vec::new(),
        };

        let identity = request_identity("agent-4-final-eight");
        let messages = [final_audio, complete]
            .into_iter()
            .map(|speech| into_ros_speech_message(&identity, speech))
            .collect::<Result<Vec<_>, _>>()
            .expect("valid speech identity")
            .into_iter()
            .map(|(_, message)| message)
            .collect::<Vec<_>>();
        assert_eq!(messages[0].event, SynthesizedSpeechChunk::AUDIO as u8);
        assert_eq!(messages[0].pcm_f32.len(), 8 * 1_024);
        assert!(messages[0].final_chunk);
        assert_eq!(messages[1].event, SynthesizedSpeechChunk::COMPLETE as u8);
        assert!(messages[1].pcm_f32.is_empty());
        assert!(!messages[1].final_chunk);
        assert_eq!(messages[1].sequence, messages[0].sequence + 1);
        assert_eq!(
            messages[1].first_sample_index,
            messages[0].first_sample_index + messages[0].pcm_f32.len() as u64
        );
    }

    #[test]
    fn zero_frame_final_marker_preserves_sample_and_token_progress_on_ros() {
        let marker = into_ros_speech_message(
            &request_identity("agent-4-zero-final"),
            SpeechOutput {
                kind: SpeechKind::Agent,
                event: SpeechEvent::Audio,
                utterance_id: "agent-4-zero-final".to_owned(),
                sequence: 2,
                first_sample_index: 12_288,
                sample_rate_hz: 22_050,
                channels: 1,
                pcm_f32: Vec::new(),
                final_chunk: true,
                committed_text_tokens: 7,
                alignment_events: Vec::new(),
                source_text: String::new(),
                source_progress: Vec::new(),
            },
        )
        .expect("valid speech identity")
        .1;

        assert_eq!(marker.event, SynthesizedSpeechChunk::AUDIO as u8);
        assert_eq!(marker.sequence, 2);
        assert_eq!(marker.first_sample_index, 12_288);
        assert!(marker.pcm_f32.is_empty());
        assert!(marker.final_chunk);
        assert_eq!(marker.committed_text_tokens, 7);
        assert!(marker.alignment_events.is_empty());
    }

    fn request_identity(utterance_id: &str) -> RequestIdentity {
        RequestIdentity {
            kind: SpeechKind::Agent,
            request_id: "00000000-0000-4000-8000-000000000001".to_owned(),
            generation_id: "00000000-0000-4000-8000-000000000002".to_owned(),
            utterance_id: utterance_id.to_owned(),
        }
    }
}
