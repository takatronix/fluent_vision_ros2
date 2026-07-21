use std::collections::VecDeque;
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{
    Device, FromSample, Sample, SampleFormat, SizedSample, StreamConfig, SupportedStreamConfig,
    I24, U24,
};
use r2r::fluent_dialogue_dora_interfaces::msg::AudioFrame;
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const SOURCE_ID: &str = "aspa_audio_capture";
const STREAM_ID: &str = "audio/mic/main";
const ROS_SAMPLE_RATE: u32 = 16_000;
const ROS_CHANNELS: u16 = 1;

#[derive(Debug, Error)]
enum CaptureError {
    #[error("no CPAL input device is available")]
    NoDevice,
    #[error("CPAL input device {0:?} was not found")]
    DeviceNotFound(String),
    #[error("failed to enumerate CPAL devices: {0}")]
    Enumerate(#[source] cpal::Error),
    #[error("failed to query the CPAL native input format: {0}")]
    NativeConfig(#[source] cpal::Error),
    #[error("failed to build CPAL input stream: {0}")]
    Build(#[source] cpal::Error),
    #[error("failed to start CPAL input stream: {0}")]
    Start(#[source] cpal::Error),
    #[error("capture callback queue overflowed")]
    Overflow,
    #[error("CPAL input stream failed: {0}")]
    Stream(String),
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

enum CaptureMessage {
    Data(Vec<i16>),
    Error(String),
}

struct LinearMonoResampler {
    step: f64,
    next_output_position: f64,
    input_index: u64,
    previous: f32,
    has_previous: bool,
}

impl LinearMonoResampler {
    fn new(input_rate: u32, output_rate: u32) -> Self {
        Self {
            step: input_rate as f64 / output_rate as f64,
            next_output_position: 0.0,
            input_index: 0,
            previous: 0.0,
            has_previous: false,
        }
    }

    fn push(&mut self, sample: f32, output: &mut Vec<i16>) {
        let index = self.input_index as f64;
        if !self.has_previous {
            self.previous = sample;
            self.has_previous = true;
        }
        while self.next_output_position <= index {
            let value = if self.input_index == 0 {
                sample
            } else {
                let fraction = (self.next_output_position - (index - 1.0)) as f32;
                self.previous + (sample - self.previous) * fraction.clamp(0.0, 1.0)
            };
            output.push(float_to_i16(value));
            self.next_output_position += self.step;
        }
        self.previous = sample;
        self.input_index += 1;
    }
}

fn main() -> Result<(), CaptureError> {
    let chunk_frames = env_usize("ASPA_AUDIO_CAPTURE_CHUNK_FRAMES", 160)?;
    let queue_capacity = env_usize("ASPA_AUDIO_CAPTURE_QUEUE_CHUNKS", 8)?;
    if chunk_frames == 0 || queue_capacity == 0 {
        return Err(CaptureError::Config(
            "numeric values must be positive".to_owned(),
        ));
    }

    let context = Context::create()?;
    let mut node = Node::create(context, "audio_capture", "")?;
    let publisher =
        node.create_publisher::<AudioFrame>("/audio/mic/frame", QosProfile::default())?;

    let host = cpal::default_host();
    let device = select_device(&host)?;
    let native = device
        .default_input_config()
        .map_err(CaptureError::NativeConfig)?;
    let native_format = native.sample_format();
    let native_rate = native.sample_rate();
    let native_channels = native.channels();
    eprintln!(
        "audio_capture native format: {:?}, {} Hz, {} channels; ROS contract: {} Hz mono PCM16LE",
        native_format, native_rate, native_channels, ROS_SAMPLE_RATE
    );

    let (sender, receiver) = sync_channel(queue_capacity);
    let overflowed = Arc::new(AtomicBool::new(false));
    let stream_failed = Arc::new(AtomicBool::new(false));
    let stream = build_native_stream(
        &device,
        native,
        chunk_frames,
        sender,
        Arc::clone(&overflowed),
        Arc::clone(&stream_failed),
    )?;
    stream.play().map_err(CaptureError::Start)?;

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    let mut seq = 0_u64;
    let mut sample_index = 0_u64;
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        if overflowed.load(Ordering::SeqCst) {
            return Err(CaptureError::Overflow);
        }
        drain_capture(
            &receiver,
            &publisher,
            chunk_frames,
            &mut seq,
            &mut sample_index,
        )?;
        if stream_failed.load(Ordering::SeqCst) {
            match receiver.try_recv() {
                Ok(CaptureMessage::Error(error)) => return Err(CaptureError::Stream(error)),
                _ => {
                    return Err(CaptureError::Stream(
                        "unknown CPAL callback failure".to_owned(),
                    ))
                }
            }
        }
    }
    publisher.publish(&audio_message(Vec::new(), seq, sample_index, true))?;
    Ok(())
}

fn build_native_stream(
    device: &Device,
    native: SupportedStreamConfig,
    chunk_frames: usize,
    sender: SyncSender<CaptureMessage>,
    overflowed: Arc<AtomicBool>,
    stream_failed: Arc<AtomicBool>,
) -> Result<cpal::Stream, CaptureError> {
    let sample_format = native.sample_format();
    let native_rate = native.sample_rate();
    let native_channels = native.channels();
    let config: StreamConfig = native.into();
    macro_rules! build {
        ($sample:ty) => {
            build_typed_stream::<$sample>(
                device,
                config,
                native_rate,
                native_channels,
                chunk_frames,
                sender,
                overflowed,
                stream_failed,
            )
        };
    }
    match sample_format {
        SampleFormat::I8 => build!(i8),
        SampleFormat::I16 => build!(i16),
        SampleFormat::I24 => build!(I24),
        SampleFormat::I32 => build!(i32),
        SampleFormat::I64 => build!(i64),
        SampleFormat::U8 => build!(u8),
        SampleFormat::U16 => build!(u16),
        SampleFormat::U24 => build!(U24),
        SampleFormat::U32 => build!(u32),
        SampleFormat::U64 => build!(u64),
        SampleFormat::F32 => build!(f32),
        SampleFormat::F64 => build!(f64),
        unsupported => Err(CaptureError::Config(format!(
            "unsupported native input sample format: {unsupported:?}"
        ))),
    }
}

#[allow(clippy::too_many_arguments)]
fn build_typed_stream<T>(
    device: &Device,
    config: StreamConfig,
    native_rate: u32,
    native_channels: u16,
    chunk_frames: usize,
    sender: SyncSender<CaptureMessage>,
    overflowed: Arc<AtomicBool>,
    stream_failed: Arc<AtomicBool>,
) -> Result<cpal::Stream, CaptureError>
where
    T: SizedSample,
    f32: FromSample<T>,
{
    let error_sender = sender.clone();
    let mut pending = VecDeque::<i16>::with_capacity(chunk_frames * 2);
    let mut resampler = LinearMonoResampler::new(native_rate, ROS_SAMPLE_RATE);
    device
        .build_input_stream(
            config,
            move |data: &[T], _| {
                let mut converted = Vec::new();
                for frame in data.chunks_exact(native_channels as usize) {
                    let mono = frame
                        .iter()
                        .map(|sample| f32::from_sample(*sample))
                        .sum::<f32>()
                        / native_channels as f32;
                    resampler.push(mono, &mut converted);
                }
                pending.extend(converted);
                while pending.len() >= chunk_frames {
                    let chunk = pending.drain(..chunk_frames).collect::<Vec<_>>();
                    if let Err(error) = sender.try_send(CaptureMessage::Data(chunk)) {
                        if matches!(error, TrySendError::Full(_)) {
                            overflowed.store(true, Ordering::SeqCst);
                        }
                        return;
                    }
                }
            },
            move |error| {
                stream_failed.store(true, Ordering::SeqCst);
                let _ = error_sender.try_send(CaptureMessage::Error(error.to_string()));
            },
            None,
        )
        .map_err(CaptureError::Build)
}

fn drain_capture(
    receiver: &Receiver<CaptureMessage>,
    publisher: &r2r::Publisher<AudioFrame>,
    chunk_frames: usize,
    seq: &mut u64,
    sample_index: &mut u64,
) -> Result<(), CaptureError> {
    loop {
        match receiver.try_recv() {
            Ok(CaptureMessage::Data(samples)) => {
                let mut data = Vec::with_capacity(samples.len() * 2);
                for sample in samples {
                    data.extend_from_slice(&sample.to_le_bytes());
                }
                publisher.publish(&audio_message(data, *seq, *sample_index, false))?;
                *seq += 1;
                *sample_index += chunk_frames as u64;
            }
            Ok(CaptureMessage::Error(error)) => return Err(CaptureError::Stream(error)),
            Err(TryRecvError::Empty) => return Ok(()),
            Err(TryRecvError::Disconnected) => {
                return Err(CaptureError::Stream(
                    "capture channel disconnected".to_owned(),
                ))
            }
        }
    }
}

fn audio_message(data: Vec<u8>, seq: u64, sample_index: u64, final_marker: bool) -> AudioFrame {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    AudioFrame {
        header: r2r::std_msgs::msg::Header {
            stamp: r2r::builtin_interfaces::msg::Time {
                sec: now.as_secs() as i32,
                nanosec: now.subsec_nanos(),
            },
            frame_id: String::new(),
        },
        source_id: SOURCE_ID.to_owned(),
        stream_id: STREAM_ID.to_owned(),
        seq,
        sample_index,
        capture_time_ns: now.as_nanos() as u64,
        frame_count: if final_marker {
            0
        } else {
            (data.len() / (ROS_CHANNELS as usize * 2)) as u32
        },
        encoding: "PCM16LE".to_owned(),
        sample_rate_hz: ROS_SAMPLE_RATE,
        channels: ROS_CHANNELS as u32,
        bit_depth: 16,
        layout: "interleaved".to_owned(),
        data,
        final_: final_marker,
    }
}

fn float_to_i16(value: f32) -> i16 {
    (value.clamp(-1.0, 1.0) * i16::MAX as f32).round() as i16
}

fn select_device(host: &cpal::Host) -> Result<Device, CaptureError> {
    let requested = env::var("ASPA_AUDIO_CAPTURE_DEVICE")
        .ok()
        .filter(|name| !name.is_empty() && name != "default");
    if let Some(name) = requested {
        for device in host.input_devices().map_err(CaptureError::Enumerate)? {
            let id = device.id().ok().map(|value| value.to_string());
            if requested_device_matches(&name, id.as_deref(), &device.to_string()) {
                return Ok(device);
            }
        }
        return Err(CaptureError::DeviceNotFound(name));
    }
    host.default_input_device().ok_or(CaptureError::NoDevice)
}

fn requested_device_matches(requested: &str, id: Option<&str>, display_name: &str) -> bool {
    id == Some(requested) || display_name == requested
}

fn env_usize(name: &str, default: usize) -> Result<usize, CaptureError> {
    env::var(name).map_or(Ok(default), |value| {
        value
            .parse()
            .map_err(|_| CaptureError::Config(name.to_owned()))
    })
}

#[cfg(test)]
mod tests {
    use super::{requested_device_matches, LinearMonoResampler};

    #[test]
    fn selects_cpal_device_by_stable_id_or_display_name() {
        assert!(requested_device_matches(
            "alsa:hw:CARD=C920,DEV=0",
            Some("alsa:hw:CARD=C920,DEV=0"),
            "HD Pro Webcam C920, USB Audio",
        ));
        assert!(requested_device_matches(
            "HD Pro Webcam C920, USB Audio",
            Some("alsa:hw:CARD=C920,DEV=0"),
            "HD Pro Webcam C920, USB Audio",
        ));
    }

    #[test]
    fn resamples_48k_to_16k_without_resetting_between_callbacks() {
        let mut resampler = LinearMonoResampler::new(48_000, 16_000);
        let mut output = Vec::new();
        for sample in [0.0, 0.1, 0.2, 0.3] {
            resampler.push(sample, &mut output);
        }
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], 0);
        assert!(output[1] > 9_000);
    }
}
