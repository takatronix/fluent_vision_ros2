use std::collections::{HashSet, VecDeque};
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, OutputCallbackInfo, SampleFormat, StreamConfig, SupportedStreamConfig};
use futures::{FutureExt, StreamExt};
use r2r::fluent_dialogue_dora_interfaces::msg::AudioFrame;
use r2r::fluent_dialogue_dora_interfaces::srv::FlushAudio;
use r2r::std_msgs::msg::String as StringMessage;
use r2r::{Context, Node, QosProfile};
use serde::Serialize;
use thiserror::Error;

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
    #[error("failed to serialize output acknowledgement: {0}")]
    Serialize(#[from] serde_json::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

#[derive(Clone, Debug, Serialize)]
struct OutputAck {
    utterance_id: String,
    seq: u64,
    frame_count: u32,
    #[serde(rename = "final")]
    is_final: bool,
    status: String,
}

struct OutputPacket {
    samples: Vec<i16>,
    cursor: usize,
    channels: usize,
    playback_start: Option<Instant>,
    playback_end: Option<Instant>,
    ack: OutputAck,
}

impl OutputPacket {
    fn acks_at_flush(&self, now: Instant) -> Vec<OutputAck> {
        let copied_frames = (self.cursor / self.channels) as u32;
        let played_frames =
            estimate_played_frames(self.playback_start, self.playback_end, copied_frames, now);
        let mut acks = Vec::with_capacity(2);
        if played_frames > 0 {
            acks.push(OutputAck {
                status: "accepted".to_owned(),
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
            status: "flushed".to_owned(),
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
        if self.ack.status == "accepted" {
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
            status: "flushed".to_owned(),
        })
    }
}

fn main() -> Result<(), OutputError> {
    let ros_rate = env_u32("ASPA_AUDIO_OUTPUT_RATE", 48_000)?;
    let ros_channels = env_u16("ASPA_AUDIO_OUTPUT_CHANNELS", 2)?;
    let max_buffer_ms = env_usize("ASPA_AUDIO_OUTPUT_BUFFER_MS", 80)?;
    if ros_rate == 0 || ros_channels == 0 || max_buffer_ms == 0 {
        return Err(OutputError::Config(
            "numeric values must be positive".to_owned(),
        ));
    }
    let packets = Arc::new(Mutex::new(VecDeque::<OutputPacket>::new()));
    let stream_failed = Arc::new(AtomicBool::new(false));
    let stream_error = Arc::new(Mutex::new(None::<String>));
    let (ack_sender, ack_receiver) = channel::<ScheduledAck>();
    let mut scheduled_acks = VecDeque::<ScheduledAck>::new();
    let mut rejected_keys = HashSet::<(String, u64)>::new();

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
        Arc::clone(&packets),
        ack_sender.clone(),
        Arc::clone(&stream_failed),
        Arc::clone(&stream_error),
    )?;
    stream.play().map_err(OutputError::Start)?;

    let context = Context::create()?;
    let mut node = Node::create(context, "audio_output", "")?;
    let mut frames = node.subscribe::<AudioFrame>("/audio/output/frame", QosProfile::default())?;
    let drained =
        node.create_publisher::<StringMessage>("/audio/output/drained", QosProfile::default())?;
    let mut flush =
        node.create_service::<FlushAudio::Service>("/audio/output/flush", QosProfile::default())?;
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        while let Some(Some(frame)) = frames.next().now_or_never() {
            validate_frame(&frame, ros_rate, ros_channels)?;
            let key = (frame.stream_id.clone(), frame.seq);
            if rejected_keys.remove(&key) {
                publish_ack(
                    &drained,
                    OutputAck {
                        utterance_id: frame.stream_id,
                        seq: frame.seq,
                        frame_count: 0,
                        is_final: false,
                        status: "flushed".to_owned(),
                    },
                )?;
                continue;
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
                Arc::clone(&packets),
                ack_sender.clone(),
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
    Ok(())
}

fn build_native_stream(
    device: &Device,
    native: SupportedStreamConfig,
    packets: Arc<Mutex<VecDeque<OutputPacket>>>,
    ack_sender: Sender<ScheduledAck>,
    stream_failed: Arc<AtomicBool>,
    stream_error: Arc<Mutex<Option<String>>>,
) -> Result<cpal::Stream, OutputError> {
    let config: StreamConfig = native.into();
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
                );
            },
            move |error| {
                stream_failed.store(true, Ordering::SeqCst);
                *stream_error.lock().expect("error mutex poisoned") = Some(error.to_string());
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
) {
    let callback_now = Instant::now();
    let timestamp = info.timestamp();
    let device_latency = timestamp.playback.duration_since(timestamp.callback);
    let buffer_playback_start = callback_now + device_latency;
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
        output[output_offset..output_offset + count]
            .copy_from_slice(&packet.samples[packet.cursor..packet.cursor + count]);
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
}

fn schedule_completed_packet(
    ack: OutputAck,
    playback_start: Instant,
    playback_end: Instant,
    _callback_now: Instant,
) -> [ScheduledAck; 2] {
    let accepted = OutputAck {
        status: "accepted".to_owned(),
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
    frame: AudioFrame,
    max_samples: usize,
) -> Result<(), OutputError> {
    let samples = frame
        .data
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
            utterance_id: frame.stream_id,
            seq: frame.seq,
            frame_count: frame.frame_count,
            is_final: frame.final_,
            status: "drained".to_owned(),
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
    publisher: &r2r::Publisher<StringMessage>,
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
    publisher: &r2r::Publisher<StringMessage>,
    ack: OutputAck,
) -> Result<(), OutputError> {
    publisher.publish(&StringMessage {
        data: serde_json::to_string(&ack)?,
    })?;
    Ok(())
}

fn validate_frame(frame: &AudioFrame, rate: u32, channels: u16) -> Result<(), OutputError> {
    if frame.stream_id.is_empty()
        || frame.frame_count == 0
        || frame.encoding != "PCM16LE"
        || frame.layout != "interleaved"
        || frame.bit_depth != 16
        || frame.sample_rate_hz != rate
        || frame.channels != channels as u32
        || frame.data.len() != frame.frame_count as usize * channels as usize * 2
    {
        return Err(OutputError::Config(
            "/audio/output/frame format mismatch".to_owned(),
        ));
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::{
        estimate_played_frames, requested_device_matches, schedule_completed_packet, take_due_acks,
        OutputAck, OutputPacket,
    };
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
                status: "drained".to_owned(),
            },
            playback_start,
            playback_end,
            now,
        );

        assert_eq!(scheduled[0].ack.status, "accepted");
        assert_eq!(scheduled[0].publish_after, playback_start);
        assert_eq!(scheduled[1].ack.status, "drained");
        assert_eq!(scheduled[1].publish_after, playback_end);
        assert!(scheduled[0].ack_at_flush(now).is_none());
        let before_start = scheduled[1]
            .ack_at_flush(now)
            .expect("terminal flush acknowledgement is required");
        assert_eq!(before_start.status, "flushed");
        assert_eq!(before_start.frame_count, 0);
        let corrected = scheduled[1]
            .ack_at_flush(playback_start + Duration::from_millis(5))
            .expect("terminal flush acknowledgement is required");
        assert_eq!(corrected.status, "flushed");
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
                status: "drained".to_owned(),
            },
        };

        let before = packet.acks_at_flush(playback_start - Duration::from_millis(1));
        assert_eq!(before.len(), 1);
        assert_eq!(before[0].status, "flushed");
        assert_eq!(before[0].frame_count, 0);

        let after = packet.acks_at_flush(playback_start + Duration::from_millis(2));
        assert_eq!(after.len(), 2);
        assert_eq!(after[0].status, "accepted");
        assert_eq!(after[1].status, "flushed");
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
                status: "drained".to_owned(),
            },
            playback_start,
            playback_end,
            now,
        );
        let mut scheduled = VecDeque::from([first[1].clone(), first[0].clone()]);

        let due = take_due_acks(&mut scheduled, playback_start);

        assert_eq!(due.len(), 1);
        assert_eq!(due[0].status, "accepted");
        assert_eq!(scheduled.len(), 1);
        assert_eq!(scheduled[0].ack.status, "drained");
    }
}
