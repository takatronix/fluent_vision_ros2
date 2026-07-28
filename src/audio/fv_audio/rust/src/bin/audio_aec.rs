use std::collections::VecDeque;
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, TrySendError, sync_channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::{FutureExt, Stream, StreamExt};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app::{AppSink, AppSinkCallbacks, AppSrc};
use r2r::fv_audio_interfaces::msg::PlaybackEvent;
use r2r::fv_speech_interfaces::msg::AudioFrame;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::std_msgs::msg::String as StringMessage;
use r2r::{Context, Node, QosProfile};
use thiserror::Error;
use webrtc_audio_processing::Processor;
use webrtc_audio_processing_config::{Config as WebRtcConfig, EchoCanceller};

const RAW_SOURCE_ID: &str = "fv_audio_capture";
const RAW_STREAM_ID: &str = "audio/mic/raw";
const RENDER_SOURCE_ID: &str = "fv_audio_output";
const RENDER_STREAM_ID: &str = "audio/render_reference/main";
const OUTPUT_SOURCE_ID: &str = "fv_audio_aec";
const OUTPUT_STREAM_ID: &str = "audio/mic/main";
const RAW_TOPIC: &str = "/audio/mic/raw";
const RENDER_TOPIC: &str = "/audio/output/render_reference";
const OUTPUT_TOPIC: &str = "/audio/mic/frame";
const PLAYBACK_EVENT_TOPIC: &str = "/audio/playback/event";
const STATUS_TOPIC: &str = "/audio/aec/status";
const AEC_SAMPLE_RATE_HZ: u32 = 16_000;
const AEC_CHANNELS: u32 = 1;
const FRAME_SAMPLES: usize = AEC_SAMPLE_RATE_HZ as usize / 100;
const AUDIO_QOS_DEPTH: usize = 256;
const FAR_MESSAGES_PER_TICK: usize = 16;
const NEAR_MESSAGES_PER_TICK: usize = 32;
const CONTROL_QOS_DEPTH: usize = 10;
const AEC_TAIL_MS: usize = 500;
const FAR_WAIT_MS: usize = 80;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AecMode {
    Bypass,
    Active,
    Tail,
    Degraded,
}

impl AecMode {
    fn as_str(self) -> &'static str {
        match self {
            Self::Bypass => "bypass",
            Self::Active => "active",
            Self::Tail => "tail",
            Self::Degraded => "degraded",
        }
    }
}

#[derive(Default)]
struct PlaybackActivity {
    agent: bool,
    system: bool,
    cue: bool,
}

impl PlaybackActivity {
    fn update(&mut self, event: &PlaybackEvent) {
        match event.event {
            value
                if value == PlaybackEvent::AGENT_STARTED as u8
                    || value == PlaybackEvent::AGENT_RESUMED as u8 =>
            {
                self.agent = true
            }
            value
                if value == PlaybackEvent::AGENT_PAUSED as u8
                    || value == PlaybackEvent::AGENT_DISCARDED as u8
                    || value == PlaybackEvent::AGENT_COMPLETED as u8 =>
            {
                self.agent = false
            }
            value if value == PlaybackEvent::SYSTEM_STARTED as u8 => self.system = true,
            value
                if value == PlaybackEvent::SYSTEM_COMPLETED as u8
                    || value == PlaybackEvent::SYSTEM_ABORTED as u8 =>
            {
                self.system = false
            }
            value if value == PlaybackEvent::CUE_STARTED as u8 => self.cue = true,
            value if value == PlaybackEvent::CUE_COMPLETED as u8 => self.cue = false,
            _ => {}
        }
    }

    fn active(&self) -> bool {
        self.agent || self.system || self.cue
    }
}

#[derive(Debug, Error)]
enum AecError {
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("invalid microphone stream: {0}")]
    Near(String),
    #[error("invalid render-reference stream: {0}")]
    Far(String),
    #[error("WebRTC AEC3 failed: {0}")]
    Processing(String),
    #[error("GStreamer render-reference conversion failed: {0}")]
    GStreamer(String),
    #[error("AEC input buffering failed: {0}")]
    Buffer(String),
}

struct RenderConverter {
    pipeline: gst::Pipeline,
    source: AppSrc,
    receiver: Receiver<Vec<u8>>,
    failed: Arc<AtomicBool>,
    error: Arc<Mutex<Option<String>>>,
    input_rate_hz: u32,
    input_channels: u32,
}

impl RenderConverter {
    fn new(input_rate_hz: u32, input_channels: u32) -> Result<Self, AecError> {
        gst::init().map_err(|error| AecError::GStreamer(error.to_string()))?;
        let description = format!(
            "appsrc name=src format=time is-live=true block=false ! \
             audio/x-raw,format=S16LE,rate={input_rate_hz},channels={input_channels},layout=interleaved ! \
             audioconvert ! audioresample ! \
             audio/x-raw,format=S16LE,rate={AEC_SAMPLE_RATE_HZ},channels=1,layout=interleaved ! \
             appsink name=sink sync=false"
        );
        let pipeline = gst::parse::launch(&description)
            .map_err(|error| {
                AecError::GStreamer(format!(
                    "cannot construct render-reference conversion pipeline: {error}"
                ))
            })?
            .downcast::<gst::Pipeline>()
            .map_err(|_| {
                AecError::GStreamer(
                    "render-reference conversion did not create a pipeline".to_owned(),
                )
            })?;
        let source = pipeline
            .by_name("src")
            .ok_or_else(|| {
                AecError::GStreamer("render-reference conversion pipeline has no appsrc".to_owned())
            })?
            .downcast::<AppSrc>()
            .map_err(|_| {
                AecError::GStreamer("render-reference conversion source is not appsrc".to_owned())
            })?;
        let sink = pipeline
            .by_name("sink")
            .ok_or_else(|| {
                AecError::GStreamer(
                    "render-reference conversion pipeline has no appsink".to_owned(),
                )
            })?
            .downcast::<AppSink>()
            .map_err(|_| {
                AecError::GStreamer("render-reference conversion sink is not appsink".to_owned())
            })?;
        let (sender, receiver) = sync_channel::<Vec<u8>>(64);
        let failed = Arc::new(AtomicBool::new(false));
        let callback_failed = Arc::clone(&failed);
        let error = Arc::new(Mutex::new(None::<String>));
        let callback_error = Arc::clone(&error);
        sink.set_wait_on_eos(false);
        sink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample(move |sink| {
                    let result = pull_render_sample(sink, &sender);
                    if let Err(message) = result {
                        if !callback_failed.swap(true, Ordering::SeqCst) {
                            *callback_error.lock().expect("AEC error mutex poisoned") =
                                Some(message);
                        }
                        return Err(gst::FlowError::Error);
                    }
                    Ok(gst::FlowSuccess::Ok)
                })
                .build(),
        );
        pipeline.set_state(gst::State::Playing).map_err(|error| {
            AecError::GStreamer(format!(
                "cannot start render-reference conversion pipeline: {error}"
            ))
        })?;
        Ok(Self {
            pipeline,
            source,
            receiver,
            failed,
            error,
            input_rate_hz,
            input_channels,
        })
    }

    fn push(&self, data: &[u8]) -> Result<(), AecError> {
        self.check_error()?;
        self.source
            .push_buffer(gst::Buffer::from_mut_slice(data.to_vec()))
            .map_err(|error| {
                AecError::GStreamer(format!(
                    "cannot push render-reference PCM into GStreamer: {error}"
                ))
            })?;
        Ok(())
    }

    fn drain(&self, runtime: &mut AecRuntime) -> Result<(), AecError> {
        loop {
            match self.receiver.try_recv() {
                Ok(data) => runtime.push_far_converted(s16le_bytes_to_i16(&data))?,
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(AecError::GStreamer(
                        "render-reference converter channel disconnected".to_owned(),
                    ));
                }
            }
        }
        self.check_error()
    }

    fn reset(&mut self) -> Result<(), AecError> {
        self.pipeline.set_state(gst::State::Null).map_err(|error| {
            AecError::GStreamer(format!(
                "cannot stop the old render-reference conversion pipeline: {error}"
            ))
        })?;
        *self = Self::new(self.input_rate_hz, self.input_channels)?;
        Ok(())
    }

    fn check_error(&self) -> Result<(), AecError> {
        if self.failed.load(Ordering::SeqCst) {
            return Err(AecError::GStreamer(
                self.error
                    .lock()
                    .expect("AEC error mutex poisoned")
                    .clone()
                    .unwrap_or_else(|| {
                        "unknown render-reference conversion callback failure".to_owned()
                    }),
            ));
        }
        let bus = self.pipeline.bus().ok_or_else(|| {
            AecError::GStreamer("render-reference pipeline has no message bus".to_owned())
        })?;
        if let Some(message) = bus.pop_filtered(&[gst::MessageType::Error]) {
            if let gst::MessageView::Error(error) = message.view() {
                return Err(AecError::GStreamer(format!(
                    "{}: {}",
                    error.error(),
                    error.debug().unwrap_or_default()
                )));
            }
        }
        Ok(())
    }
}

impl Drop for RenderConverter {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

fn pull_render_sample(sink: &AppSink, sender: &SyncSender<Vec<u8>>) -> Result<(), String> {
    let sample = sink
        .pull_sample()
        .map_err(|error| format!("cannot pull converted render reference: {error}"))?;
    validate_render_caps(&sample)?;
    let buffer = sample
        .buffer()
        .ok_or_else(|| "converted render reference has no buffer".to_owned())?;
    let mapped = buffer
        .map_readable()
        .map_err(|error| format!("cannot map converted render reference: {error}"))?;
    sender
        .try_send(mapped.as_slice().to_vec())
        .map_err(|error| match error {
            TrySendError::Full(_) => {
                "converted render-reference callback queue overflowed".to_owned()
            }
            TrySendError::Disconnected(_) => {
                "converted render-reference receiver disconnected".to_owned()
            }
        })
}

fn validate_render_caps(sample: &gst::Sample) -> Result<(), String> {
    let caps = sample
        .caps()
        .ok_or_else(|| "converted render reference has no caps".to_owned())?;
    let structure = caps
        .structure(0)
        .ok_or_else(|| "converted render-reference caps are empty".to_owned())?;
    let format = structure
        .get::<String>("format")
        .map_err(|error| format!("render-reference caps have no format: {error}"))?;
    let rate = structure
        .get::<i32>("rate")
        .map_err(|error| format!("render-reference caps have no rate: {error}"))?;
    let channels = structure
        .get::<i32>("channels")
        .map_err(|error| format!("render-reference caps have no channels: {error}"))?;
    if format != "S16LE" || rate != AEC_SAMPLE_RATE_HZ as i32 || channels != 1 {
        return Err(format!(
            "converted render-reference caps violate the AEC contract: format={format}, rate={rate}, channels={channels}"
        ));
    }
    Ok(())
}

struct NearFrame {
    samples: Vec<i16>,
    sample_index: u64,
    capture_time_ns: u64,
}

struct StreamValidator {
    source_id: &'static str,
    stream_id: &'static str,
    sample_rate_hz: u32,
    channels: u32,
    expected_seq: Option<u64>,
    expected_sample_index: Option<u64>,
}

impl StreamValidator {
    fn new(
        source_id: &'static str,
        stream_id: &'static str,
        sample_rate_hz: u32,
        channels: u32,
    ) -> Self {
        Self {
            source_id,
            stream_id,
            sample_rate_hz,
            channels,
            expected_seq: None,
            expected_sample_index: None,
        }
    }

    /// Validates the immutable stream contract and advances continuity state.
    /// A sequence/sample gap is recoverable and returned to the caller so it
    /// can flush DSP state without killing the long-running audio graph.
    fn validate(&mut self, message: &AudioFrame) -> Result<Option<String>, String> {
        if message.source_id != self.source_id
            || message.stream_id != self.stream_id
            || message.encoding != "PCM16LE"
            || message.sample_rate_hz != self.sample_rate_hz
            || message.channels != self.channels
            || message.bit_depth != 16
            || message.layout != "interleaved"
        {
            return Err(format!(
                "identity/format mismatch: source={:?}, stream={:?}, encoding={:?}, rate={}, channels={}, bit_depth={}, layout={:?}",
                message.source_id,
                message.stream_id,
                message.encoding,
                message.sample_rate_hz,
                message.channels,
                message.bit_depth,
                message.layout
            ));
        }
        let expected_bytes = message.frame_count as usize * self.channels as usize * 2;
        if message.final_ {
            if message.frame_count != 0 || !message.data.is_empty() {
                return Err("final marker contains PCM data".to_owned());
            }
        } else if message.frame_count == 0 || message.data.len() != expected_bytes {
            return Err(format!(
                "payload length mismatch: frame_count={}, expected_bytes={expected_bytes}, actual_bytes={}",
                message.frame_count,
                message.data.len()
            ));
        }
        let mut discontinuities = Vec::new();
        if let Some(expected) = self.expected_seq {
            if message.seq != expected {
                discontinuities.push(format!(
                    "sequence discontinuity: expected {expected}, got {}",
                    message.seq
                ));
            }
        }
        if let Some(expected) = self.expected_sample_index {
            if message.sample_index != expected {
                discontinuities.push(format!(
                    "sample-index discontinuity: expected {expected}, got {}",
                    message.sample_index
                ));
            }
        }
        self.expected_seq = Some(
            message
                .seq
                .checked_add(1)
                .ok_or_else(|| "sequence overflow".to_owned())?,
        );
        self.expected_sample_index = Some(
            message
                .sample_index
                .checked_add(u64::from(message.frame_count))
                .ok_or_else(|| "sample index overflow".to_owned())?,
        );
        Ok((!discontinuities.is_empty()).then(|| discontinuities.join("; ")))
    }

    fn reset(&mut self) {
        self.expected_seq = None;
        self.expected_sample_index = None;
    }
}

struct WebRtcAec {
    processor: Processor,
    stream_delay_ms: u16,
}

impl WebRtcAec {
    fn new(stream_delay_ms: u16) -> Result<Self, AecError> {
        Ok(Self {
            processor: create_processor(stream_delay_ms)?,
            stream_delay_ms,
        })
    }

    fn reset(&mut self) -> Result<(), AecError> {
        self.processor = create_processor(self.stream_delay_ms)?;
        Ok(())
    }

    fn process(&mut self, far: &[i16], near: &[i16]) -> Result<Vec<i16>, AecError> {
        let mut render = vec![i16_frame_to_f32(far)];
        self.processor
            .process_render_frame(&mut render)
            .map_err(|error| AecError::Processing(format!("render frame: {error:?}")))?;
        let mut capture = vec![i16_frame_to_f32(near)];
        self.processor
            .process_capture_frame(&mut capture)
            .map_err(|error| AecError::Processing(format!("capture frame: {error:?}")))?;
        Ok(f32_frame_to_i16(&capture[0]))
    }
}

fn create_processor(stream_delay_ms: u16) -> Result<Processor, AecError> {
    let processor = Processor::new(AEC_SAMPLE_RATE_HZ)
        .map_err(|error| AecError::Processing(format!("initialization: {error:?}")))?;
    processor.set_config(WebRtcConfig {
        echo_canceller: Some(EchoCanceller::Full {
            stream_delay_ms: Some(stream_delay_ms),
        }),
        ..Default::default()
    });
    Ok(processor)
}

struct FarDelayLine {
    delayed: VecDeque<i16>,
}

impl FarDelayLine {
    fn new(delay_samples: usize) -> Self {
        Self {
            delayed: VecDeque::from(vec![0; delay_samples]),
        }
    }

    fn push(&mut self, samples: impl IntoIterator<Item = i16>, output: &mut VecDeque<i16>) {
        for sample in samples {
            self.delayed.push_back(sample);
            if let Some(delayed) = self.delayed.pop_front() {
                output.push_back(delayed);
            }
        }
    }
}

struct AecRuntime {
    aec: WebRtcAec,
    near: VecDeque<NearFrame>,
    far: VecDeque<i16>,
    far_delay: FarDelayLine,
    delay_samples: usize,
    max_near_frames: usize,
    max_far_samples: usize,
    output_seq: u64,
    output_sample_index: Option<u64>,
    playback_active: bool,
    tail_frames_remaining: usize,
    far_wait_frames: usize,
    mode: AecMode,
}

impl AecRuntime {
    fn new(
        stream_delay_ms: u16,
        max_near_frames: usize,
        max_far_samples: usize,
    ) -> Result<Self, AecError> {
        let delay_samples = usize::from(stream_delay_ms) * AEC_SAMPLE_RATE_HZ as usize / 1_000;
        Ok(Self {
            aec: WebRtcAec::new(stream_delay_ms)?,
            near: VecDeque::new(),
            far: VecDeque::new(),
            far_delay: FarDelayLine::new(delay_samples),
            delay_samples,
            max_near_frames,
            max_far_samples,
            output_seq: 0,
            output_sample_index: None,
            playback_active: false,
            tail_frames_remaining: 0,
            far_wait_frames: FAR_WAIT_MS / 10,
            mode: AecMode::Bypass,
        })
    }

    fn set_playback_active(&mut self, active: bool) {
        if active {
            self.playback_active = true;
            self.tail_frames_remaining = 0;
        } else if self.playback_active {
            self.playback_active = false;
            self.tail_frames_remaining = AEC_TAIL_MS / 10;
        }
    }

    fn finish_input(&mut self) {
        self.playback_active = false;
        self.tail_frames_remaining = 0;
        self.far_wait_frames = 0;
    }

    fn push_near(&mut self, message: &AudioFrame) -> Result<(), AecError> {
        if message.frame_count as usize % FRAME_SAMPLES != 0 {
            return Err(AecError::Near(format!(
                "frame_count {} is not a whole number of WebRTC 10 ms frames",
                message.frame_count
            )));
        }
        let samples = s16le_bytes_to_i16(&message.data);
        for (index, frame) in samples.chunks_exact(FRAME_SAMPLES).enumerate() {
            let frame_offset = index * FRAME_SAMPLES;
            let capture_time_ns = message
                .capture_time_ns
                .checked_add(frames_to_ns(frame_offset as u64, AEC_SAMPLE_RATE_HZ)?)
                .ok_or_else(|| AecError::Near("capture timestamp overflow".to_owned()))?;
            self.near.push_back(NearFrame {
                samples: frame.to_vec(),
                sample_index: message.sample_index + frame_offset as u64,
                capture_time_ns,
            });
        }
        Ok(())
    }

    fn push_far_converted(&mut self, samples: Vec<i16>) -> Result<(), AecError> {
        self.far_delay.push(samples, &mut self.far);
        if self.far.len() > self.max_far_samples {
            let excess = self.far.len() - self.max_far_samples;
            self.far.drain(..excess);
            eprintln!(
                "audio_aec warning: discarded {excess} stale render-reference samples to keep the queue bounded"
            );
        }
        Ok(())
    }

    fn reset_far(&mut self) -> Result<(), AecError> {
        self.far.clear();
        self.far_delay = FarDelayLine::new(self.delay_samples);
        self.aec.reset()
    }

    fn resynchronize_near(&mut self) -> Result<(), AecError> {
        self.near.clear();
        self.reset_far()
    }

    fn flush_near_raw(
        &mut self,
        publisher: &r2r::Publisher<AudioFrame>,
    ) -> Result<(), AecError> {
        while let Some(near) = self.near.pop_front() {
            if self.far.len() >= FRAME_SAMPLES {
                self.far.drain(..FRAME_SAMPLES);
            }
            self.publish_frame(publisher, near.samples.clone(), &near)?;
        }
        self.mode = AecMode::Degraded;
        Ok(())
    }

    fn publish_frame(
        &mut self,
        publisher: &r2r::Publisher<AudioFrame>,
        samples: Vec<i16>,
        near: &NearFrame,
    ) -> Result<(), AecError> {
        let sample_index = *self.output_sample_index.get_or_insert(near.sample_index);
        publisher.publish(&audio_message(
            samples,
            self.output_seq,
            sample_index,
            near.capture_time_ns,
            false,
        ))?;
        self.output_seq = self
            .output_seq
            .checked_add(1)
            .ok_or_else(|| AecError::Buffer("output sequence overflow".to_owned()))?;
        self.output_sample_index = Some(
            sample_index
                .checked_add(FRAME_SAMPLES as u64)
                .ok_or_else(|| AecError::Buffer("output sample index overflow".to_owned()))?,
        );
        Ok(())
    }

    fn process_ready(
        &mut self,
        publisher: &r2r::Publisher<AudioFrame>,
    ) -> Result<AecMode, AecError> {
        while !self.near.is_empty() {
            let wants_aec = self.playback_active || self.tail_frames_remaining > 0;
            if wants_aec
                && self.far.len() < FRAME_SAMPLES
                && self.near.len() <= self.far_wait_frames.min(self.max_near_frames)
            {
                break;
            }
            let near = self.near.pop_front().expect("near queue became empty");
            if wants_aec && self.far.len() >= FRAME_SAMPLES {
                let far = self.far.drain(..FRAME_SAMPLES).collect::<Vec<_>>();
                let cleaned = self.aec.process(&far, &near.samples)?;
                self.publish_frame(publisher, cleaned, &near)?;
                self.mode = if self.playback_active {
                    AecMode::Active
                } else {
                    AecMode::Tail
                };
            } else {
                if self.far.len() >= FRAME_SAMPLES {
                    self.far.drain(..FRAME_SAMPLES);
                }
                self.publish_frame(publisher, near.samples.clone(), &near)?;
                self.mode = if wants_aec {
                    AecMode::Degraded
                } else {
                    AecMode::Bypass
                };
            }
            if !self.playback_active && self.tail_frames_remaining > 0 {
                self.tail_frames_remaining -= 1;
            }
        }
        Ok(self.mode)
    }
}

fn main() -> Result<(), AecError> {
    let stream_delay_ms = env_u16("FV_AUDIO_AEC_STREAM_DELAY_MS", 40)?;
    let far_rate = env_u32("FV_AUDIO_AEC_FAR_SAMPLE_RATE", 48_000)?;
    let far_channels = env_u32("FV_AUDIO_AEC_FAR_CHANNELS", 2)?;
    let max_near_frames = env_usize("FV_AUDIO_AEC_MAX_NEAR_FRAMES", 100)?;
    let max_far_samples = env_usize("FV_AUDIO_AEC_MAX_FAR_SAMPLES", 64_000)?;
    if far_rate == 0 || far_channels == 0 || max_near_frames == 0 || max_far_samples < FRAME_SAMPLES
    {
        return Err(AecError::Config(
            "far rate, channels, and buffer limits must be positive".to_owned(),
        ));
    }
    let mut runtime = AecRuntime::new(stream_delay_ms, max_near_frames, max_far_samples)?;
    let mut render_converter = RenderConverter::new(far_rate, far_channels)?;
    let mut near_validator = StreamValidator::new(
        RAW_SOURCE_ID,
        RAW_STREAM_ID,
        AEC_SAMPLE_RATE_HZ,
        AEC_CHANNELS,
    );
    let mut far_validator =
        StreamValidator::new(RENDER_SOURCE_ID, RENDER_STREAM_ID, far_rate, far_channels);

    let context = Context::create()?;
    let mut node = Node::create(context, "audio_aec", "")?;
    let qos = audio_qos();
    let mut near_input = node.subscribe::<AudioFrame>(RAW_TOPIC, qos.clone())?;
    let mut far_input = node.subscribe::<AudioFrame>(RENDER_TOPIC, qos.clone())?;
    let output = node.create_publisher::<AudioFrame>(OUTPUT_TOPIC, qos)?;
    let mut playback_events =
        node.subscribe::<PlaybackEvent>(PLAYBACK_EVENT_TOPIC, control_qos())?;
    let status = node.create_publisher::<StringMessage>(STATUS_TOPIC, status_qos())?;

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    publish_status(&status, AecMode::Bypass)?;
    eprintln!(
        "audio_aec ready: raw bypass while idle, WebRTC AEC3 while playing + {AEC_TAIL_MS} ms tail, far={far_rate} Hz/{far_channels} ch, stream_delay_ms={stream_delay_ms}"
    );

    let mut near_final = false;
    let mut playback = PlaybackActivity::default();
    let mut published_mode = AecMode::Bypass;
    let mut last_status_publish = Instant::now();
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        for event in take_ready_bounded(&mut playback_events, CONTROL_QOS_DEPTH) {
            playback.update(&event);
            runtime.set_playback_active(playback.active());
        }
        for message in take_ready_bounded(&mut far_input, FAR_MESSAGES_PER_TICK) {
            if let Some(reason) = far_validator.validate(&message).map_err(AecError::Far)? {
                eprintln!("audio_aec warning: render reference resynchronized: {reason}");
                runtime.reset_far()?;
                render_converter.reset()?;
            }
            if message.final_ {
                runtime.reset_far()?;
                render_converter.reset()?;
                far_validator.reset();
                continue;
            }
            render_converter.push(&message.data)?;
            render_converter.drain(&mut runtime)?;
        }
        render_converter.drain(&mut runtime)?;
        for message in take_ready_bounded(&mut near_input, NEAR_MESSAGES_PER_TICK) {
            if let Some(reason) = near_validator.validate(&message).map_err(AecError::Near)? {
                eprintln!("audio_aec warning: microphone stream resynchronized: {reason}");
                runtime.flush_near_raw(&output)?;
                runtime.resynchronize_near()?;
                render_converter.reset()?;
                near_final = false;
            }
            if message.final_ {
                near_final = true;
                continue;
            }
            if near_final {
                eprintln!(
                    "audio_aec warning: microphone resumed after a final marker; resynchronizing"
                );
                runtime.resynchronize_near()?;
                render_converter.reset()?;
                near_final = false;
            }
            runtime.push_near(&message)?;
        }
        render_converter.drain(&mut runtime)?;
        if near_final {
            runtime.finish_input();
        }
        let mode = runtime.process_ready(&output)?;
        if mode != published_mode || last_status_publish.elapsed() >= Duration::from_secs(1) {
            publish_status(&status, mode)?;
            if mode != published_mode {
                eprintln!("audio_aec mode: {}", mode.as_str());
            }
            published_mode = mode;
            last_status_publish = Instant::now();
        }
        if near_final {
            if !runtime.near.is_empty() {
                continue;
            }
            let final_sample_index = runtime
                .output_sample_index
                .or(near_validator.expected_sample_index)
                .ok_or_else(|| {
                    AecError::Near("final marker arrived before microphone PCM".to_owned())
                })?;
            output.publish(&audio_message(
                Vec::new(),
                runtime.output_seq,
                final_sample_index,
                unix_time_ns()?,
                true,
            ))?;
            return Ok(());
        }
    }
    Ok(())
}

fn take_ready_bounded<T>(stream: &mut (impl Stream<Item = T> + Unpin), limit: usize) -> Vec<T> {
    let mut ready = Vec::with_capacity(limit);
    for _ in 0..limit {
        match stream.next().now_or_never() {
            Some(Some(item)) => ready.push(item),
            Some(None) | None => break,
        }
    }
    ready
}

fn audio_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: AUDIO_QOS_DEPTH,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    }
}

fn control_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: CONTROL_QOS_DEPTH,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    }
}

fn status_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    }
}

fn publish_status(
    publisher: &r2r::Publisher<StringMessage>,
    mode: AecMode,
) -> Result<(), AecError> {
    publisher.publish(&StringMessage {
        data: mode.as_str().to_owned(),
    })?;
    Ok(())
}

fn audio_message(
    samples: Vec<i16>,
    seq: u64,
    sample_index: u64,
    capture_time_ns: u64,
    final_marker: bool,
) -> AudioFrame {
    let data = i16_to_s16le(samples);
    AudioFrame {
        header: r2r::std_msgs::msg::Header {
            stamp: r2r::builtin_interfaces::msg::Time {
                sec: (capture_time_ns / 1_000_000_000) as i32,
                nanosec: (capture_time_ns % 1_000_000_000) as u32,
            },
            frame_id: String::new(),
        },
        source_id: OUTPUT_SOURCE_ID.to_owned(),
        stream_id: OUTPUT_STREAM_ID.to_owned(),
        seq,
        sample_index,
        capture_time_ns,
        frame_count: if final_marker {
            0
        } else {
            (data.len() / 2) as u32
        },
        encoding: "PCM16LE".to_owned(),
        sample_rate_hz: AEC_SAMPLE_RATE_HZ,
        channels: AEC_CHANNELS,
        bit_depth: 16,
        layout: "interleaved".to_owned(),
        data,
        final_: final_marker,
    }
}

fn s16le_bytes_to_i16(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|bytes| i16::from_le_bytes([bytes[0], bytes[1]]))
        .collect()
}

fn i16_to_s16le(samples: Vec<i16>) -> Vec<u8> {
    samples
        .into_iter()
        .flat_map(i16::to_le_bytes)
        .collect::<Vec<_>>()
}

fn i16_frame_to_f32(frame: &[i16]) -> Vec<f32> {
    frame
        .iter()
        .map(|sample| f32::from(*sample) / 32_768.0)
        .collect()
}

fn f32_frame_to_i16(frame: &[f32]) -> Vec<i16> {
    frame
        .iter()
        .map(|sample| (sample.clamp(-1.0, 1.0) * 32_767.0).round() as i16)
        .collect()
}

fn frames_to_ns(frames: u64, sample_rate_hz: u32) -> Result<u64, AecError> {
    frames
        .checked_mul(1_000_000_000)
        .map(|nanoseconds| nanoseconds / u64::from(sample_rate_hz))
        .ok_or_else(|| AecError::Near("frame timestamp conversion overflow".to_owned()))
}

fn unix_time_ns() -> Result<u64, AecError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| AecError::Near(format!("system clock predates UNIX epoch: {error}")))?;
    u64::try_from(duration.as_nanos())
        .map_err(|_| AecError::Near("system clock exceeds ROS nanosecond range".to_owned()))
}

fn env_u16(name: &str, default: u16) -> Result<u16, AecError> {
    env::var(name).map_or(Ok(default), |value| {
        value.parse().map_err(|_| AecError::Config(name.to_owned()))
    })
}

fn env_u32(name: &str, default: u32) -> Result<u32, AecError> {
    env::var(name).map_or(Ok(default), |value| {
        value.parse().map_err(|_| AecError::Config(name.to_owned()))
    })
}

fn env_usize(name: &str, default: usize) -> Result<usize, AecError> {
    env::var(name).map_or(Ok(default), |value| {
        value.parse().map_err(|_| AecError::Config(name.to_owned()))
    })
}

#[cfg(test)]
mod tests {
    use super::{
        AEC_SAMPLE_RATE_HZ, AEC_TAIL_MS, AUDIO_QOS_DEPTH, AecRuntime, FRAME_SAMPLES, FarDelayLine,
        RenderConverter, WebRtcAec, audio_qos, take_ready_bounded,
    };
    use futures::stream;
    use std::collections::VecDeque;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn ros_messages_are_consumed_in_bounded_ordered_batches() {
        let mut values = stream::iter(0..10);
        assert_eq!(take_ready_bounded(&mut values, 3), vec![0, 1, 2]);
        assert_eq!(take_ready_bounded(&mut values, 3), vec![3, 4, 5]);
        assert_eq!(take_ready_bounded(&mut values, 10), vec![6, 7, 8, 9]);
    }

    #[test]
    fn audio_transport_uses_the_shared_reliable_history_depth() {
        assert_eq!(audio_qos().depth, AUDIO_QOS_DEPTH);
        assert_eq!(AUDIO_QOS_DEPTH, 256);
    }

    #[test]
    fn playback_completion_keeps_the_configured_aec_tail() {
        let mut runtime = AecRuntime::new(40, 100, 64_000).expect("AEC runtime failed");
        runtime.set_playback_active(true);
        assert!(runtime.playback_active);
        runtime.set_playback_active(false);
        assert!(!runtime.playback_active);
        assert_eq!(runtime.tail_frames_remaining, AEC_TAIL_MS / 10);
    }

    #[test]
    fn gstreamer_converts_48k_stereo_render_reference_to_16k_mono() {
        let converter =
            RenderConverter::new(48_000, 2).expect("render converter initialization failed");
        let input = (0..4_800)
            .flat_map(|frame| {
                let sample = ((frame as f32 / 20.0).sin() * 8_000.0) as i16;
                [sample, sample]
                    .into_iter()
                    .flat_map(i16::to_le_bytes)
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        converter
            .push(&input)
            .expect("render converter rejected valid PCM");
        let mut runtime = AecRuntime::new(40, 100, 64_000).expect("AEC runtime failed");
        for _ in 0..100 {
            converter
                .drain(&mut runtime)
                .expect("render converter drain failed");
            if runtime.far.len() >= FRAME_SAMPLES {
                break;
            }
            thread::sleep(Duration::from_millis(1));
        }
        assert!(runtime.far.len() >= FRAME_SAMPLES);
    }

    #[test]
    fn render_delay_keeps_sample_order() {
        let mut delay = FarDelayLine::new(3);
        let mut output = VecDeque::new();
        delay.push([10, 20, 30, 40], &mut output);
        assert_eq!(output.into_iter().collect::<Vec<_>>(), vec![0, 0, 0, 10]);
    }

    #[test]
    fn webrtc_aec3_passes_silence_as_silence() {
        let mut aec = WebRtcAec::new(40).expect("WebRTC AEC3 initialization failed");
        let silence = vec![0_i16; FRAME_SAMPLES];
        let cleaned = aec
            .process(&silence, &silence)
            .expect("WebRTC AEC3 silence processing failed");
        assert_eq!(cleaned.len(), FRAME_SAMPLES);
        assert!(cleaned.iter().all(|sample| *sample == 0));
    }

    #[test]
    fn webrtc_aec3_attenuates_a_converged_identity_echo() {
        let mut aec = WebRtcAec::new(40).expect("WebRTC AEC3 initialization failed");
        let mut phase = 0.0_f64;
        let mut make_frame = || {
            (0..FRAME_SAMPLES)
                .map(|_| {
                    phase += 2.0 * std::f64::consts::PI * 440.0 / f64::from(AEC_SAMPLE_RATE_HZ);
                    (phase.sin() * 8_000.0) as i16
                })
                .collect::<Vec<_>>()
        };
        let input = make_frame();
        let input_rms = rms(&input);
        let mut cleaned_rms = input_rms;
        for _ in 0..200 {
            let echo = make_frame();
            let cleaned = aec
                .process(&echo, &echo)
                .expect("WebRTC AEC3 echo processing failed");
            cleaned_rms = rms(&cleaned);
        }
        assert!(
            cleaned_rms < input_rms * 0.5,
            "echo not attenuated: input_rms={input_rms:.1}, cleaned_rms={cleaned_rms:.1}"
        );
    }

    fn rms(frame: &[i16]) -> f64 {
        let sum = frame
            .iter()
            .map(|sample| f64::from(*sample).powi(2))
            .sum::<f64>();
        (sum / frame.len() as f64).sqrt()
    }
}
