use std::collections::{HashSet, VecDeque};
use std::env;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{
    channel, sync_channel, Receiver, Sender, SyncSender, TryRecvError, TrySendError,
};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{
    BufferSize, Device, OutputCallbackInfo, SampleFormat, StreamConfig, SupportedStreamConfig,
};
use futures::{FutureExt, StreamExt};
use r2r::aspa_audio_interfaces::msg::{OutputDrained, PlaybackFrame};
use r2r::aspa_audio_interfaces::srv::FlushAudio;
use r2r::fv_speech_interfaces::msg::AudioFrame;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::std_msgs::msg::Float32;
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const KIND_AGENT: u8 = PlaybackFrame::AGENT as u8;
const KIND_SYSTEM: u8 = PlaybackFrame::SYSTEM as u8;
const KIND_CUE: u8 = PlaybackFrame::CUE as u8;
const STATUS_ACCEPTED: u8 = OutputDrained::ACCEPTED as u8;
const STATUS_DRAINED: u8 = OutputDrained::DRAINED as u8;
const STATUS_FLUSHED: u8 = OutputDrained::FLUSHED as u8;
const PLAYOUT_QOS_DEPTH: usize = 256;
const RENDER_SOURCE_ID: &str = "aspa_audio_output";
const RENDER_STREAM_ID: &str = "audio/render_reference/main";
const RENDER_TOPIC: &str = "/audio/output/render_reference";

#[derive(Debug, Error)]
enum OutputError {
    #[error("no CPAL output device is available")]
    NoDevice,
    #[error("CPAL output device {0:?} was not found")]
    DeviceNotFound(String),
    #[error("failed to enumerate CPAL devices: {0}")]
    Enumerate(#[source] cpal::Error),
    #[error("failed to query CPAL output formats: {0}")]
    NativeConfig(#[source] cpal::Error),
    #[error("failed to build CPAL output stream: {0}")]
    Build(#[source] cpal::Error),
    #[error("failed to start CPAL output stream: {0}")]
    Start(#[source] cpal::Error),
    #[error("CPAL output stream failed: {0}")]
    Stream(String),
    #[error("short device buffer overflowed; playback_controller is not pacing output")]
    Overflow,
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

#[derive(Clone, Debug)]
struct OutputAck {
    utterance_id: String,
    seq: u64,
    frame_count: u32,
    is_final: bool,
    status: u8,
}

struct OutputPacket {
    samples: Vec<i16>,
    cursor: usize,
    channels: usize,
    playback_start: Option<Instant>,
    playback_end: Option<Instant>,
    ack: OutputAck,
}

struct RenderReferencePacket {
    samples: Vec<i16>,
    playback_time_ns: u64,
}

#[derive(Clone, Copy)]
struct ClockAnchor {
    instant: Instant,
    unix_time_ns: u64,
}

impl OutputPacket {
    fn acks_at_flush(&self, now: Instant) -> Vec<OutputAck> {
        let copied_frames = (self.cursor / self.channels) as u32;
        let played_frames =
            estimate_played_frames(self.playback_start, self.playback_end, copied_frames, now);
        let mut acks = Vec::with_capacity(2);
        if played_frames > 0 {
            acks.push(OutputAck {
                status: STATUS_ACCEPTED,
                ..self.ack.clone()
            });
        }
        if played_frames >= self.ack.frame_count {
            acks.push(self.ack.clone());
            return acks;
        }
        acks.push(OutputAck {
            utterance_id: self.ack.utterance_id.clone(),
            seq: self.ack.seq,
            frame_count: played_frames,
            is_final: false,
            status: STATUS_FLUSHED,
        });
        acks
    }
}

#[derive(Clone)]
struct ScheduledAck {
    ack: OutputAck,
    playback_start: Instant,
    playback_end: Instant,
    publish_after: Instant,
}

impl ScheduledAck {
    fn ack_at_flush(&self, now: Instant) -> Option<OutputAck> {
        if self.ack.status == STATUS_ACCEPTED {
            return (now >= self.playback_start).then(|| self.ack.clone());
        }
        let played_frames = estimate_played_frames(
            Some(self.playback_start),
            Some(self.playback_end),
            self.ack.frame_count,
            now,
        );
        if played_frames >= self.ack.frame_count {
            return Some(self.ack.clone());
        }
        Some(OutputAck {
            utterance_id: self.ack.utterance_id.clone(),
            seq: self.ack.seq,
            frame_count: played_frames,
            is_final: false,
            status: STATUS_FLUSHED,
        })
    }
}

fn main() -> Result<(), OutputError> {
    let ros_rate = env_u32("ASPA_AUDIO_OUTPUT_RATE", 48_000)?;
    let ros_channels = env_u16("ASPA_AUDIO_OUTPUT_CHANNELS", 2)?;
    let max_buffer_ms = env_usize("ASPA_AUDIO_OUTPUT_BUFFER_MS", 80)?;
    let buffer_frames = env_optional_u32("ASPA_AUDIO_OUTPUT_BUFFER_FRAMES")?;
    let render_queue_chunks = env_usize("ASPA_AUDIO_RENDER_REFERENCE_QUEUE_CHUNKS", 64)?;
    if ros_rate == 0 || ros_channels == 0 || max_buffer_ms == 0 || render_queue_chunks == 0 {
        return Err(OutputError::Config(
            "numeric values must be positive".to_owned(),
        ));
    }
    let packets = Arc::new(Mutex::new(VecDeque::<OutputPacket>::new()));
    let stream_failed = Arc::new(AtomicBool::new(false));
    let stream_error = Arc::new(Mutex::new(None::<String>));
    let output_gain = Arc::new(AtomicU32::new(1.0_f32.to_bits()));
    let (ack_sender, ack_receiver) = channel::<ScheduledAck>();
    let (render_sender, render_receiver) =
        sync_channel::<RenderReferencePacket>(render_queue_chunks);
    let mut scheduled_acks = VecDeque::<ScheduledAck>::new();
    let mut rejected_keys = HashSet::<(String, u64)>::new();
    let clock_anchor = ClockAnchor {
        instant: Instant::now(),
        unix_time_ns: unix_time_ns()?,
    };

    let host = cpal::default_host();
    let device = select_device(&host)?;
    let device_config = select_output_config(&device, ros_rate, ros_channels)?;
    let max_samples = ros_rate as usize * ros_channels as usize * max_buffer_ms / 1000;
    eprintln!(
        "audio_output exact device format: PCM16, {} Hz, {} channels",
        ros_rate, ros_channels
    );
    let mut stream = build_native_stream(
        &device,
        device_config,
        buffer_frames,
        Arc::clone(&packets),
        ack_sender.clone(),
        render_sender.clone(),
        clock_anchor,
        Arc::clone(&output_gain),
        Arc::clone(&stream_failed),
        Arc::clone(&stream_error),
    )?;

    let context = Context::create()?;
    let mut node = Node::create(context, "audio_output", "")?;
    let playout_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: PLAYOUT_QOS_DEPTH,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    };
    let mut frames = node.subscribe::<PlaybackFrame>("/audio/play", playout_qos.clone())?;
    let drained =
        node.create_publisher::<OutputDrained>("/audio/output/drained", QosProfile::default())?;
    let render_reference =
        node.create_publisher::<AudioFrame>(RENDER_TOPIC, playout_qos.clone())?;
    let mut flush =
        node.create_service::<FlushAudio::Service>("/audio/output/flush", QosProfile::default())?;
    let volume_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    };
    let mut volumes = node.subscribe::<Float32>("/audio/output/volume", volume_qos)?;
    stream.play().map_err(OutputError::Start)?;
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    let mut render_seq = 0_u64;
    let mut render_sample_index = 0_u64;

    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        publish_render_reference(
            &render_receiver,
            &render_reference,
            ros_rate,
            ros_channels,
            &mut render_seq,
            &mut render_sample_index,
        )?;
        while let Some(Some(volume)) = volumes.next().now_or_never() {
            if volume.data.is_finite() {
                output_gain.store(volume.data.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
            } else {
                eprintln!("ignoring non-finite /audio/output/volume value");
            }
        }
        while let Some(Some(frame)) = frames.next().now_or_never() {
            let key = (frame.utterance_id.clone(), frame.seq);
            if rejected_keys.remove(&key) {
                publish_ack(
                    &drained,
                    OutputAck {
                        utterance_id: frame.utterance_id,
                        seq: frame.seq,
                        frame_count: 0,
                        is_final: false,
                        status: STATUS_FLUSHED,
                    },
                )?;
                continue;
            }
            if let Err(error) = validate_frame(&frame, ros_rate, ros_channels) {
                eprintln!("rejecting invalid /audio/play frame {key:?}: {error}");
                // /audio/play is an internal, trusted controller contract.  If
                // this node silently drops a frame, playback_controller keeps
                // that key in flight forever while both processes look alive.
                // Exit instead; the launch contract shuts down the audio stack.
                return Err(error);
            }
            enqueue_frame(&packets, frame, max_samples)?;
        }
        collect_scheduled_acks(&ack_receiver, &mut scheduled_acks)?;
        publish_due_acks(&mut scheduled_acks, &drained, Instant::now())?;
        while let Some(Some(request)) = flush.next().now_or_never() {
            if request.message.utterance_ids.len() != request.message.seqs.len() {
                request.respond(FlushAudio::Response {
                    success: false,
                    message: "utterance_ids and seqs lengths differ".to_owned(),
                })?;
                continue;
            }
            let requested_keys = request
                .message
                .utterance_ids
                .iter()
                .cloned()
                .zip(request.message.seqs.iter().copied())
                .collect::<HashSet<_>>();
            collect_scheduled_acks(&ack_receiver, &mut scheduled_acks)?;
            publish_due_acks(&mut scheduled_acks, &drained, Instant::now())?;
            // Closing the CPAL stream is the only portable way to discard samples
            // already handed to the host buffer instead of merely pausing them.
            let now = Instant::now();
            drop(stream);
            discard_render_reference(&render_receiver)?;
            render_reference.publish(&render_reference_message(
                Vec::new(),
                render_seq,
                render_sample_index,
                unix_time_ns()?,
                ros_rate,
                ros_channels,
                true,
            ))?;
            render_seq = 0;
            render_sample_index = 0;
            let queued = {
                let mut queue = packets.lock().expect("audio queue poisoned");
                queue
                    .drain(..)
                    .flat_map(|packet| packet.acks_at_flush(now))
                    .collect::<Vec<_>>()
            };
            // A callback can complete immediately before the queue lock above.
            collect_scheduled_acks(&ack_receiver, &mut scheduled_acks)?;
            let mut flushed = scheduled_acks
                .drain(..)
                .filter_map(|scheduled| scheduled.ack_at_flush(now))
                .collect::<Vec<_>>();
            // Completed packets precede packets that remained in the software queue.
            flushed.extend(queued);
            let observed_keys = flushed
                .iter()
                .map(|ack| (ack.utterance_id.clone(), ack.seq))
                .collect::<HashSet<_>>();
            rejected_keys.extend(requested_keys.difference(&observed_keys).cloned());
            for ack in flushed {
                publish_ack(&drained, ack)?;
            }
            stream_failed.store(false, Ordering::SeqCst);
            *stream_error.lock().expect("error mutex poisoned") = None;
            stream = build_native_stream(
                &device,
                device_config,
                buffer_frames,
                Arc::clone(&packets),
                ack_sender.clone(),
                render_sender.clone(),
                clock_anchor,
                Arc::clone(&output_gain),
                Arc::clone(&stream_failed),
                Arc::clone(&stream_error),
            )?;
            stream.play().map_err(OutputError::Start)?;
            request.respond(FlushAudio::Response {
                success: true,
                message: "device buffer flushed".to_owned(),
            })?;
        }
        collect_scheduled_acks(&ack_receiver, &mut scheduled_acks)?;
        publish_due_acks(&mut scheduled_acks, &drained, Instant::now())?;
        if stream_failed.load(Ordering::SeqCst) {
            let error = stream_error
                .lock()
                .expect("error mutex poisoned")
                .clone()
                .unwrap_or_else(|| "unknown CPAL output error".to_owned());
            return Err(OutputError::Stream(error));
        }
    }
    discard_render_reference(&render_receiver)?;
    render_reference.publish(&render_reference_message(
        Vec::new(),
        render_seq,
        render_sample_index,
        unix_time_ns()?,
        ros_rate,
        ros_channels,
        true,
    ))?;
    Ok(())
}

fn build_native_stream(
    device: &Device,
    native: SupportedStreamConfig,
    buffer_frames: Option<u32>,
    packets: Arc<Mutex<VecDeque<OutputPacket>>>,
    ack_sender: Sender<ScheduledAck>,
    render_sender: SyncSender<RenderReferencePacket>,
    clock_anchor: ClockAnchor,
    output_gain: Arc<AtomicU32>,
    stream_failed: Arc<AtomicBool>,
    stream_error: Arc<Mutex<Option<String>>>,
) -> Result<cpal::Stream, OutputError> {
    let mut config: StreamConfig = native.into();
    if let Some(buffer_frames) = buffer_frames {
        config.buffer_size = BufferSize::Fixed(buffer_frames);
        eprintln!(
            "audio_output requested callback period: {} frames ({:.1} ms)",
            buffer_frames,
            buffer_frames as f64 * 1_000.0 / native.sample_rate() as f64
        );
    }
    let callback_stream_failed = Arc::clone(&stream_failed);
    let callback_stream_error = Arc::clone(&stream_error);
    device
        .build_output_stream(
            config,
            move |output: &mut [i16], info| {
                fill_device_buffer(
                    output,
                    info,
                    native.sample_rate(),
                    native.channels(),
                    &packets,
                    &ack_sender,
                    &render_sender,
                    clock_anchor,
                    &output_gain,
                    &callback_stream_failed,
                    &callback_stream_error,
                );
            },
            move |error| {
                *stream_error.lock().expect("error mutex poisoned") = Some(error.to_string());
                stream_failed.store(true, Ordering::SeqCst);
            },
            None,
        )
        .map_err(OutputError::Build)
}

fn fill_device_buffer(
    output: &mut [i16],
    info: &OutputCallbackInfo,
    sample_rate: u32,
    channels: u16,
    packets: &Arc<Mutex<VecDeque<OutputPacket>>>,
    ack_sender: &Sender<ScheduledAck>,
    render_sender: &SyncSender<RenderReferencePacket>,
    clock_anchor: ClockAnchor,
    output_gain: &Arc<AtomicU32>,
    stream_failed: &Arc<AtomicBool>,
    stream_error: &Arc<Mutex<Option<String>>>,
) {
    let callback_now = Instant::now();
    let timestamp = info.timestamp();
    let device_latency = timestamp.playback.duration_since(timestamp.callback);
    let buffer_playback_start = callback_now + device_latency;
    let gain = f32::from_bits(output_gain.load(Ordering::Relaxed));
    let mut queue = packets.lock().expect("audio queue poisoned");
    let mut output_offset = 0;
    while output_offset < output.len() {
        let Some(packet) = queue.front_mut() else {
            for sample in &mut output[output_offset..] {
                *sample = 0;
            }
            break;
        };
        let available = packet.samples.len() - packet.cursor;
        let count = available.min(output.len() - output_offset);
        for (target, source) in output[output_offset..output_offset + count]
            .iter_mut()
            .zip(&packet.samples[packet.cursor..packet.cursor + count])
        {
            *target = scale_sample(*source, gain);
        }
        let segment_start =
            buffer_playback_start + frames_duration(output_offset / channels as usize, sample_rate);
        output_offset += count;
        packet.cursor += count;
        let segment_end =
            buffer_playback_start + frames_duration(output_offset / channels as usize, sample_rate);
        packet.playback_start.get_or_insert(segment_start);
        packet.playback_end = Some(segment_end);
        if packet.cursor == packet.samples.len() {
            let completed = queue.pop_front().expect("front packet disappeared");
            let playback_start = completed
                .playback_start
                .expect("completed packet has no playback start");
            let playback_end = completed
                .playback_end
                .expect("completed packet has no playback deadline");
            for scheduled in
                schedule_completed_packet(completed.ack, playback_start, playback_end, callback_now)
            {
                let _ = ack_sender.send(scheduled);
            }
        }
    }
    if output.len() % channels as usize != 0 {
        mark_stream_failed(
            "CPAL output callback returned a partial interleaved frame".to_owned(),
            stream_failed,
            stream_error,
        );
        return;
    }
    let playback_time_ns = match instant_to_unix_ns(clock_anchor, buffer_playback_start) {
        Ok(value) => value,
        Err(error) => {
            mark_stream_failed(error, stream_failed, stream_error);
            return;
        }
    };
    let packet = RenderReferencePacket {
        samples: output.to_vec(),
        playback_time_ns,
    };
    if let Err(error) = render_sender.try_send(packet) {
        let message = match error {
            TrySendError::Full(_) => "render-reference callback queue overflowed",
            TrySendError::Disconnected(_) => "render-reference receiver disconnected",
        };
        mark_stream_failed(message.to_owned(), stream_failed, stream_error);
    }
}

fn publish_render_reference(
    receiver: &Receiver<RenderReferencePacket>,
    publisher: &r2r::Publisher<AudioFrame>,
    sample_rate_hz: u32,
    channels: u16,
    seq: &mut u64,
    sample_index: &mut u64,
) -> Result<(), OutputError> {
    loop {
        match receiver.try_recv() {
            Ok(packet) => {
                if packet.samples.is_empty() || packet.samples.len() % channels as usize != 0 {
                    return Err(OutputError::Stream(
                        "render-reference packet violates the interleaved PCM contract".to_owned(),
                    ));
                }
                let frame_count = packet.samples.len() / channels as usize;
                publisher.publish(&render_reference_message(
                    packet.samples,
                    *seq,
                    *sample_index,
                    packet.playback_time_ns,
                    sample_rate_hz,
                    channels,
                    false,
                ))?;
                *seq = seq
                    .checked_add(1)
                    .ok_or_else(|| OutputError::Stream("render sequence overflow".to_owned()))?;
                *sample_index = sample_index
                    .checked_add(frame_count as u64)
                    .ok_or_else(|| {
                        OutputError::Stream("render sample index overflow".to_owned())
                    })?;
            }
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => {
                return Err(OutputError::Stream(
                    "render-reference callback channel disconnected".to_owned(),
                ))
            }
        }
    }
}

fn discard_render_reference(receiver: &Receiver<RenderReferencePacket>) -> Result<(), OutputError> {
    loop {
        match receiver.try_recv() {
            Ok(_) => {}
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => {
                return Err(OutputError::Stream(
                    "render-reference callback channel disconnected".to_owned(),
                ))
            }
        }
    }
}

fn render_reference_message(
    samples: Vec<i16>,
    seq: u64,
    sample_index: u64,
    playback_time_ns: u64,
    sample_rate_hz: u32,
    channels: u16,
    final_marker: bool,
) -> AudioFrame {
    let data = samples
        .into_iter()
        .flat_map(i16::to_le_bytes)
        .collect::<Vec<_>>();
    AudioFrame {
        header: r2r::std_msgs::msg::Header {
            stamp: r2r::builtin_interfaces::msg::Time {
                sec: (playback_time_ns / 1_000_000_000) as i32,
                nanosec: (playback_time_ns % 1_000_000_000) as u32,
            },
            frame_id: String::new(),
        },
        source_id: RENDER_SOURCE_ID.to_owned(),
        stream_id: RENDER_STREAM_ID.to_owned(),
        seq,
        sample_index,
        capture_time_ns: playback_time_ns,
        frame_count: if final_marker {
            0
        } else {
            (data.len() / (channels as usize * 2)) as u32
        },
        encoding: "PCM16LE".to_owned(),
        sample_rate_hz,
        channels: channels as u32,
        bit_depth: 16,
        layout: "interleaved".to_owned(),
        data,
        final_: final_marker,
    }
}

fn mark_stream_failed(
    error: String,
    stream_failed: &Arc<AtomicBool>,
    stream_error: &Arc<Mutex<Option<String>>>,
) {
    if !stream_failed.swap(true, Ordering::SeqCst) {
        *stream_error.lock().expect("error mutex poisoned") = Some(error);
    }
}

fn instant_to_unix_ns(anchor: ClockAnchor, instant: Instant) -> Result<u64, String> {
    let elapsed = instant
        .checked_duration_since(anchor.instant)
        .ok_or_else(|| "CPAL playback timestamp predates the output clock anchor".to_owned())?;
    let elapsed_ns = u64::try_from(elapsed.as_nanos())
        .map_err(|_| "CPAL playback timestamp exceeds the ROS nanosecond range".to_owned())?;
    anchor
        .unix_time_ns
        .checked_add(elapsed_ns)
        .ok_or_else(|| "CPAL playback timestamp overflowed".to_owned())
}

fn unix_time_ns() -> Result<u64, OutputError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| {
            OutputError::Stream(format!("system clock predates UNIX epoch: {error}"))
        })?;
    u64::try_from(duration.as_nanos())
        .map_err(|_| OutputError::Stream("system clock exceeds ROS nanosecond range".to_owned()))
}

fn schedule_completed_packet(
    ack: OutputAck,
    playback_start: Instant,
    playback_end: Instant,
    _callback_now: Instant,
) -> [ScheduledAck; 2] {
    let accepted = OutputAck {
        status: STATUS_ACCEPTED,
        ..ack.clone()
    };
    [
        ScheduledAck {
            ack: accepted,
            playback_start,
            playback_end,
            // CPAL accepting a buffer is earlier than the first sample reaching
            // the device.  The semantic `*_started` event must not lead the
            // physical playback boundary used for interruption context.
            publish_after: playback_start,
        },
        ScheduledAck {
            ack,
            playback_start,
            playback_end,
            publish_after: playback_end,
        },
    ]
}

fn enqueue_frame(
    packets: &Arc<Mutex<VecDeque<OutputPacket>>>,
    frame: PlaybackFrame,
    max_samples: usize,
) -> Result<(), OutputError> {
    let samples = frame
        .pcm_s16le
        .chunks_exact(2)
        .map(|pair| i16::from_le_bytes([pair[0], pair[1]]))
        .collect::<Vec<_>>();
    let mut queue = packets.lock().expect("audio queue poisoned");
    let buffered = queue
        .iter()
        .map(|packet| packet.samples.len() - packet.cursor)
        .sum::<usize>();
    if buffered + samples.len() > max_samples {
        return Err(OutputError::Overflow);
    }
    queue.push_back(OutputPacket {
        samples,
        cursor: 0,
        channels: frame.channels as usize,
        playback_start: None,
        playback_end: None,
        ack: OutputAck {
            utterance_id: frame.utterance_id,
            seq: frame.seq,
            frame_count: frame.frame_count,
            is_final: frame.final_,
            status: STATUS_DRAINED,
        },
    });
    Ok(())
}

fn collect_scheduled_acks(
    receiver: &Receiver<ScheduledAck>,
    scheduled: &mut VecDeque<ScheduledAck>,
) -> Result<(), OutputError> {
    loop {
        match receiver.try_recv() {
            Ok(ack) => scheduled.push_back(ack),
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => {
                return Err(OutputError::Stream(
                    "output acknowledgement channel disconnected".to_owned(),
                ))
            }
        }
    }
}

fn publish_due_acks(
    scheduled: &mut VecDeque<ScheduledAck>,
    publisher: &r2r::Publisher<OutputDrained>,
    now: Instant,
) -> Result<(), OutputError> {
    for ack in take_due_acks(scheduled, now) {
        publish_ack(publisher, ack)?;
    }
    Ok(())
}

fn take_due_acks(scheduled: &mut VecDeque<ScheduledAck>, now: Instant) -> Vec<OutputAck> {
    let mut waiting = VecDeque::new();
    let mut due = Vec::new();
    while let Some(scheduled_ack) = scheduled.pop_front() {
        if scheduled_ack.publish_after <= now {
            due.push(scheduled_ack.ack);
        } else {
            waiting.push_back(scheduled_ack);
        }
    }
    *scheduled = waiting;
    due
}

fn frames_duration(frames: usize, sample_rate: u32) -> Duration {
    Duration::from_secs_f64(frames as f64 / sample_rate as f64)
}

fn estimate_played_frames(
    start: Option<Instant>,
    end: Option<Instant>,
    copied_frames: u32,
    now: Instant,
) -> u32 {
    let (Some(start), Some(end)) = (start, end) else {
        return 0;
    };
    if now <= start || copied_frames == 0 {
        return 0;
    }
    if now >= end || end <= start {
        return copied_frames;
    }
    let elapsed = now.duration_since(start).as_nanos();
    let total = end.duration_since(start).as_nanos();
    ((elapsed * copied_frames as u128) / total) as u32
}

fn publish_ack(
    publisher: &r2r::Publisher<OutputDrained>,
    ack: OutputAck,
) -> Result<(), OutputError> {
    publisher.publish(&OutputDrained {
        utterance_id: ack.utterance_id,
        seq: ack.seq,
        frame_count: ack.frame_count,
        final_: ack.is_final,
        status: ack.status,
    })?;
    Ok(())
}

fn validate_frame(frame: &PlaybackFrame, rate: u32, channels: u16) -> Result<(), OutputError> {
    if !matches!(frame.kind, KIND_AGENT | KIND_SYSTEM | KIND_CUE)
        || frame.utterance_id.is_empty()
        || frame.frame_count == 0
        || frame.sample_rate_hz != rate
        || frame.channels != channels as u32
        || frame.pcm_s16le.len() != frame.frame_count as usize * channels as usize * 2
    {
        return Err(OutputError::Config(
            "/audio/play format mismatch".to_owned(),
        ));
    }
    Ok(())
}

fn scale_sample(sample: i16, gain: f32) -> i16 {
    (sample as f32 * gain).round() as i16
}

fn select_output_config(
    device: &Device,
    rate: u32,
    channels: u16,
) -> Result<SupportedStreamConfig, OutputError> {
    device
        .supported_output_configs()
        .map_err(OutputError::NativeConfig)?
        .find(|candidate| {
            candidate.channels() == channels
                && candidate.sample_format() == SampleFormat::I16
                && candidate.min_sample_rate() <= rate
                && rate <= candidate.max_sample_rate()
        })
        .map(|candidate| candidate.with_sample_rate(rate))
        .ok_or_else(|| {
            OutputError::Config(format!(
                "selected device has no PCM16 output config for {rate} Hz / {channels} channels"
            ))
        })
}

fn select_device(host: &cpal::Host) -> Result<Device, OutputError> {
    let requested = env::var("ASPA_AUDIO_OUTPUT_DEVICE")
        .ok()
        .filter(|name| !name.is_empty() && name != "default");
    if let Some(name) = requested {
        for device in host.output_devices().map_err(OutputError::Enumerate)? {
            let id = device.id().ok().map(|value| value.to_string());
            if requested_device_matches(&name, id.as_deref(), &device.to_string()) {
                return Ok(device);
            }
        }
        return Err(OutputError::DeviceNotFound(name));
    }
    host.default_output_device().ok_or(OutputError::NoDevice)
}

fn requested_device_matches(requested: &str, id: Option<&str>, display_name: &str) -> bool {
    id == Some(requested) || display_name == requested
}

fn env_u32(name: &str, default: u32) -> Result<u32, OutputError> {
    env::var(name).map_or(Ok(default), |value| {
        value
            .parse()
            .map_err(|_| OutputError::Config(name.to_owned()))
    })
}
fn env_u16(name: &str, default: u16) -> Result<u16, OutputError> {
    env::var(name).map_or(Ok(default), |value| {
        value
            .parse()
            .map_err(|_| OutputError::Config(name.to_owned()))
    })
}
fn env_usize(name: &str, default: usize) -> Result<usize, OutputError> {
    env::var(name).map_or(Ok(default), |value| {
        value
            .parse()
            .map_err(|_| OutputError::Config(name.to_owned()))
    })
}

fn env_optional_u32(name: &str) -> Result<Option<u32>, OutputError> {
    env::var(name).map_or(Ok(None), |value| {
        value
            .parse()
            .ok()
            .filter(|parsed| *parsed > 0)
            .map(Some)
            .ok_or_else(|| OutputError::Config(name.to_owned()))
    })
}

#[cfg(test)]
mod tests {
    use super::{
        estimate_played_frames, render_reference_message, requested_device_matches, scale_sample,
        schedule_completed_packet, take_due_acks, validate_frame, OutputAck, OutputPacket,
        KIND_AGENT, RENDER_SOURCE_ID, RENDER_STREAM_ID, STATUS_ACCEPTED, STATUS_DRAINED,
        STATUS_FLUSHED,
    };
    use r2r::aspa_audio_interfaces::msg::PlaybackFrame;
    use std::collections::VecDeque;
    use std::time::{Duration, Instant};

    #[test]
    fn selects_cpal_device_by_stable_id_or_display_name() {
        assert!(requested_device_matches(
            "alsa:aspa_usb_speaker",
            Some("alsa:aspa_usb_speaker"),
            "ASPA USB speaker (mono duplicated to FL/FR)",
        ));
        assert!(requested_device_matches(
            "ASPA USB speaker (mono duplicated to FL/FR)",
            Some("alsa:aspa_usb_speaker"),
            "ASPA USB speaker (mono duplicated to FL/FR)",
        ));
    }

    #[test]
    fn invalid_playout_frame_is_rejected_without_entering_the_device_queue() {
        let mut frame = PlaybackFrame {
            kind: KIND_AGENT,
            utterance_id: "agent-0-invalid".to_owned(),
            seq: 0,
            sample_index: 0,
            frame_count: 1,
            sample_rate_hz: 48_000,
            channels: 2,
            pcm_s16le: vec![0; 4],
            final_: true,
            playout_generation: 0,
        };
        assert!(validate_frame(&frame, 48_000, 2).is_ok());
        frame.frame_count = 0;
        frame.pcm_s16le.clear();
        assert!(validate_frame(&frame, 48_000, 2).is_err());
    }

    #[test]
    fn estimates_only_frames_that_reached_the_device() {
        let start = Instant::now();
        let end = start + Duration::from_millis(20);
        assert_eq!(
            estimate_played_frames(Some(start), Some(end), 960, start),
            0
        );
        assert_eq!(
            estimate_played_frames(
                Some(start),
                Some(end),
                960,
                start + Duration::from_millis(10),
            ),
            480
        );
        assert_eq!(
            estimate_played_frames(Some(start), Some(end), 960, end),
            960
        );
    }

    #[test]
    fn separates_device_acceptance_from_actual_playback_progress() {
        let now = Instant::now();
        let playback_start = now + Duration::from_millis(10);
        let playback_end = playback_start + Duration::from_millis(20);
        let scheduled = schedule_completed_packet(
            OutputAck {
                utterance_id: "agent-0-test".to_owned(),
                seq: 0,
                frame_count: 960,
                is_final: false,
                status: STATUS_DRAINED,
            },
            playback_start,
            playback_end,
            now,
        );

        assert_eq!(scheduled[0].ack.status, STATUS_ACCEPTED);
        assert_eq!(scheduled[0].publish_after, playback_start);
        assert_eq!(scheduled[1].ack.status, STATUS_DRAINED);
        assert_eq!(scheduled[1].publish_after, playback_end);
        assert!(scheduled[0].ack_at_flush(now).is_none());
        let before_start = scheduled[1]
            .ack_at_flush(now)
            .expect("terminal flush acknowledgement is required");
        assert_eq!(before_start.status, STATUS_FLUSHED);
        assert_eq!(before_start.frame_count, 0);
        let corrected = scheduled[1]
            .ack_at_flush(playback_start + Duration::from_millis(5))
            .expect("terminal flush acknowledgement is required");
        assert_eq!(corrected.status, STATUS_FLUSHED);
        assert_eq!(corrected.frame_count, 240);
    }

    #[test]
    fn partial_packet_flush_reports_start_only_after_audio_became_audible() {
        let playback_start = Instant::now() + Duration::from_millis(10);
        let playback_end = playback_start + Duration::from_millis(4);
        let packet = OutputPacket {
            samples: vec![0; 8],
            cursor: 4,
            channels: 1,
            playback_start: Some(playback_start),
            playback_end: Some(playback_end),
            ack: OutputAck {
                utterance_id: "agent-0-partial".to_owned(),
                seq: 0,
                frame_count: 8,
                is_final: false,
                status: STATUS_DRAINED,
            },
        };

        let before = packet.acks_at_flush(playback_start - Duration::from_millis(1));
        assert_eq!(before.len(), 1);
        assert_eq!(before[0].status, STATUS_FLUSHED);
        assert_eq!(before[0].frame_count, 0);

        let after = packet.acks_at_flush(playback_start + Duration::from_millis(2));
        assert_eq!(after.len(), 2);
        assert_eq!(after[0].status, STATUS_ACCEPTED);
        assert_eq!(after[1].status, STATUS_FLUSHED);
        assert_eq!(after[1].frame_count, 2);
    }

    #[test]
    fn due_acceptance_is_not_blocked_by_an_earlier_future_drain() {
        let now = Instant::now();
        let playback_start = now + Duration::from_millis(5);
        let playback_end = now + Duration::from_millis(25);
        let first = schedule_completed_packet(
            OutputAck {
                utterance_id: "agent-0-test".to_owned(),
                seq: 0,
                frame_count: 960,
                is_final: false,
                status: STATUS_DRAINED,
            },
            playback_start,
            playback_end,
            now,
        );
        let mut scheduled = VecDeque::from([first[1].clone(), first[0].clone()]);

        let due = take_due_acks(&mut scheduled, playback_start);

        assert_eq!(due.len(), 1);
        assert_eq!(due[0].status, STATUS_ACCEPTED);
        assert_eq!(scheduled.len(), 1);
        assert_eq!(scheduled[0].ack.status, STATUS_DRAINED);
    }

    #[test]
    fn applies_output_gain_at_the_device_sample_boundary() {
        assert_eq!(scale_sample(20_000, 1.0), 20_000);
        assert_eq!(scale_sample(20_000, 0.5), 10_000);
        assert_eq!(scale_sample(-20_000, 0.5), -10_000);
        assert_eq!(scale_sample(i16::MAX, 0.0), 0);
    }

    #[test]
    fn render_reference_uses_the_callback_pcm_contract() {
        let message = render_reference_message(vec![10, 20, 30, 40], 7, 960, 123, 48_000, 2, false);
        assert_eq!(message.source_id, RENDER_SOURCE_ID);
        assert_eq!(message.stream_id, RENDER_STREAM_ID);
        assert_eq!(message.seq, 7);
        assert_eq!(message.sample_index, 960);
        assert_eq!(message.capture_time_ns, 123);
        assert_eq!(message.frame_count, 2);
        assert_eq!(message.sample_rate_hz, 48_000);
        assert_eq!(message.channels, 2);
        assert_eq!(message.data.len(), 8);
        assert!(!message.final_);
    }
}
