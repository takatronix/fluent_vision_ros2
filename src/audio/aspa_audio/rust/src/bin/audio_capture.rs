use std::collections::VecDeque;
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{
    BufferSize, Device, SampleFormat, SizedSample, StreamConfig, SupportedStreamConfig, I24, U24,
};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app::{AppSink, AppSinkCallbacks, AppSrc};
use r2r::fv_speech_interfaces::msg::AudioFrame;
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const SOURCE_ID: &str = "aspa_audio_capture";
const STREAM_ID: &str = "audio/mic/raw";
const OUTPUT_TOPIC: &str = "/audio/mic/raw";
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
    #[error("GStreamer input conversion failed: {0}")]
    GStreamer(String),
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

enum CaptureMessage {
    Data(Vec<u8>),
    CpalError(String),
    GStreamerError(String),
}

trait GStreamerPcmSample: SizedSample {
    const FORMAT: &'static str;

    fn append_bytes(&self, output: &mut Vec<u8>);
}

macro_rules! impl_gstreamer_pcm_sample {
    ($sample:ty, $format:literal) => {
        impl GStreamerPcmSample for $sample {
            const FORMAT: &'static str = $format;

            fn append_bytes(&self, output: &mut Vec<u8>) {
                output.extend_from_slice(&self.to_le_bytes());
            }
        }
    };
}

impl_gstreamer_pcm_sample!(i16, "S16LE");
impl_gstreamer_pcm_sample!(u16, "U16LE");
impl_gstreamer_pcm_sample!(i32, "S32LE");
impl_gstreamer_pcm_sample!(u32, "U32LE");
impl_gstreamer_pcm_sample!(f32, "F32LE");
impl_gstreamer_pcm_sample!(f64, "F64LE");

impl GStreamerPcmSample for i8 {
    const FORMAT: &'static str = "S8";

    fn append_bytes(&self, output: &mut Vec<u8>) {
        output.push(*self as u8);
    }
}

impl GStreamerPcmSample for u8 {
    const FORMAT: &'static str = "U8";

    fn append_bytes(&self, output: &mut Vec<u8>) {
        output.push(*self);
    }
}

impl GStreamerPcmSample for I24 {
    const FORMAT: &'static str = "S24LE";

    fn append_bytes(&self, output: &mut Vec<u8>) {
        output.extend_from_slice(&self.inner().to_le_bytes()[..3]);
    }
}

impl GStreamerPcmSample for U24 {
    const FORMAT: &'static str = "U24LE";

    fn append_bytes(&self, output: &mut Vec<u8>) {
        output.extend_from_slice(&self.inner().to_le_bytes()[..3]);
    }
}

struct CapturePipeline {
    pipeline: gst::Pipeline,
    source: AppSrc,
}

impl CapturePipeline {
    fn new<T: GStreamerPcmSample>(
        input_rate: u32,
        input_channels: u16,
        chunk_bytes: usize,
        sender: SyncSender<CaptureMessage>,
        overflowed: Arc<AtomicBool>,
        failed: Arc<AtomicBool>,
    ) -> Result<Self, CaptureError> {
        gst::init().map_err(|error| CaptureError::GStreamer(error.to_string()))?;
        let description = format!(
            "appsrc name=src format=time is-live=true block=false ! \
             audio/x-raw,format={},rate={input_rate},channels={input_channels},layout=interleaved ! \
             audioconvert ! audioresample ! \
             audio/x-raw,format=S16LE,rate={ROS_SAMPLE_RATE},channels={ROS_CHANNELS},layout=interleaved ! \
             appsink name=sink sync=false",
            <T as GStreamerPcmSample>::FORMAT
        );
        let pipeline = gst::parse::launch(&description)
            .map_err(|error| {
                CaptureError::GStreamer(format!(
                    "cannot construct required input conversion pipeline: {error}"
                ))
            })?
            .downcast::<gst::Pipeline>()
            .map_err(|_| {
                CaptureError::GStreamer(
                    "required input conversion did not create a pipeline".to_owned(),
                )
            })?;
        let source = pipeline
            .by_name("src")
            .ok_or_else(|| {
                CaptureError::GStreamer(
                    "required input conversion pipeline has no appsrc".to_owned(),
                )
            })?
            .downcast::<AppSrc>()
            .map_err(|_| {
                CaptureError::GStreamer("required input conversion source is not appsrc".to_owned())
            })?;
        let sink = pipeline
            .by_name("sink")
            .ok_or_else(|| {
                CaptureError::GStreamer(
                    "required input conversion pipeline has no appsink".to_owned(),
                )
            })?
            .downcast::<AppSink>()
            .map_err(|_| {
                CaptureError::GStreamer("required input conversion sink is not appsink".to_owned())
            })?;

        sink.set_wait_on_eos(false);
        let pending_capacity = chunk_bytes
            .checked_mul(2)
            .ok_or_else(|| CaptureError::Config("capture pending size overflowed".to_owned()))?;
        let mut pending = VecDeque::<u8>::with_capacity(pending_capacity);
        sink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample(move |sink| {
                    let result = (|| -> Result<(), String> {
                        let sample = sink
                            .pull_sample()
                            .map_err(|error| format!("cannot pull converted input: {error}"))?;
                        validate_converted_caps(&sample)?;
                        let buffer = sample
                            .buffer()
                            .ok_or_else(|| "converted input sample has no buffer".to_owned())?;
                        let mapped = buffer.map_readable().map_err(|error| {
                            format!("cannot map converted input buffer: {error}")
                        })?;
                        pending.extend(mapped.as_slice());
                        while pending.len() >= chunk_bytes {
                            let chunk = pending.drain(..chunk_bytes).collect::<Vec<_>>();
                            sender
                                .try_send(CaptureMessage::Data(chunk))
                                .map_err(|error| match error {
                                    TrySendError::Full(_) => {
                                        overflowed.store(true, Ordering::SeqCst);
                                        "converted input queue overflowed".to_owned()
                                    }
                                    TrySendError::Disconnected(_) => {
                                        "converted input receiver disconnected".to_owned()
                                    }
                                })?;
                        }
                        Ok(())
                    })();
                    match result {
                        Ok(()) => Ok(gst::FlowSuccess::Ok),
                        Err(error) => {
                            eprintln!("audio_capture GStreamer callback failed: {error}");
                            let _ = sender.try_send(CaptureMessage::GStreamerError(error));
                            failed.store(true, Ordering::SeqCst);
                            Err(gst::FlowError::Error)
                        }
                    }
                })
                .build(),
        );
        pipeline.set_state(gst::State::Playing).map_err(|error| {
            CaptureError::GStreamer(format!(
                "cannot start required input conversion pipeline: {error}"
            ))
        })?;
        eprintln!(
            "audio_capture required GStreamer conversion: {} {} Hz {} channels -> S16LE {} Hz {} channel",
            <T as GStreamerPcmSample>::FORMAT,
            input_rate,
            input_channels,
            ROS_SAMPLE_RATE,
            ROS_CHANNELS
        );
        Ok(Self { pipeline, source })
    }

    fn check_error(&self) -> Result<(), CaptureError> {
        let bus = self.pipeline.bus().ok_or_else(|| {
            CaptureError::GStreamer(
                "required input conversion pipeline has no message bus".to_owned(),
            )
        })?;
        if let Some(message) = bus.pop_filtered(&[gst::MessageType::Error]) {
            if let gst::MessageView::Error(error) = message.view() {
                return Err(CaptureError::GStreamer(format!(
                    "{}: {}",
                    error.error(),
                    error.debug().unwrap_or_default()
                )));
            }
        }
        Ok(())
    }

    fn stop(&self) -> Result<(), CaptureError> {
        self.pipeline.set_state(gst::State::Null).map_err(|error| {
            CaptureError::GStreamer(format!(
                "cannot stop required input conversion pipeline: {error}"
            ))
        })?;
        Ok(())
    }
}

impl Drop for CapturePipeline {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

fn validate_converted_caps(sample: &gst::Sample) -> Result<(), String> {
    let caps = sample
        .caps()
        .ok_or_else(|| "converted input sample has no caps".to_owned())?;
    let structure = caps
        .structure(0)
        .ok_or_else(|| "converted input caps have no structure".to_owned())?;
    let format = structure
        .get::<String>("format")
        .map_err(|error| format!("converted input caps have no format: {error}"))?;
    let rate = structure
        .get::<i32>("rate")
        .map_err(|error| format!("converted input caps have no rate: {error}"))?;
    let channels = structure
        .get::<i32>("channels")
        .map_err(|error| format!("converted input caps have no channels: {error}"))?;
    if format != "S16LE" || rate != ROS_SAMPLE_RATE as i32 || channels != ROS_CHANNELS as i32 {
        return Err(format!(
            "converted input caps violate ROS contract: format={format}, rate={rate}, channels={channels}"
        ));
    }
    Ok(())
}

fn main() -> Result<(), CaptureError> {
    let chunk_frames = env_usize("ASPA_AUDIO_CAPTURE_CHUNK_FRAMES", 160)?;
    let queue_capacity = env_usize("ASPA_AUDIO_CAPTURE_QUEUE_CHUNKS", 8)?;
    let buffer_frames = env_optional_u32("ASPA_AUDIO_CAPTURE_BUFFER_FRAMES")?;
    if chunk_frames == 0 || queue_capacity == 0 {
        return Err(CaptureError::Config(
            "numeric values must be positive".to_owned(),
        ));
    }
    let chunk_bytes = chunk_frames
        .checked_mul(ROS_CHANNELS as usize)
        .and_then(|frames| frames.checked_mul(std::mem::size_of::<i16>()))
        .ok_or_else(|| CaptureError::Config("capture chunk size overflowed".to_owned()))?;
    let context = Context::create()?;
    let mut node = Node::create(context, "audio_capture", "")?;
    let publisher = node.create_publisher::<AudioFrame>(OUTPUT_TOPIC, QosProfile::default())?;

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
    let cpal_failed = Arc::new(AtomicBool::new(false));
    let gstreamer_failed = Arc::new(AtomicBool::new(false));
    let (stream, converter) = build_native_stream(
        &device,
        native,
        buffer_frames,
        chunk_bytes,
        sender,
        Arc::clone(&overflowed),
        Arc::clone(&cpal_failed),
        Arc::clone(&gstreamer_failed),
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
            chunk_bytes,
            &mut seq,
            &mut sample_index,
        )?;
        converter.check_error()?;
        if gstreamer_failed.load(Ordering::SeqCst) {
            return Err(CaptureError::GStreamer(
                "unknown input conversion callback failure".to_owned(),
            ));
        }
        if cpal_failed.load(Ordering::SeqCst) {
            return Err(CaptureError::Stream(
                "unknown CPAL callback failure".to_owned(),
            ));
        }
    }
    converter.stop()?;
    publisher.publish(&audio_message(Vec::new(), seq, sample_index, true))?;
    Ok(())
}

fn build_native_stream(
    device: &Device,
    native: SupportedStreamConfig,
    buffer_frames: Option<u32>,
    chunk_bytes: usize,
    sender: SyncSender<CaptureMessage>,
    overflowed: Arc<AtomicBool>,
    cpal_failed: Arc<AtomicBool>,
    gstreamer_failed: Arc<AtomicBool>,
) -> Result<(cpal::Stream, CapturePipeline), CaptureError> {
    let sample_format = native.sample_format();
    let native_rate = native.sample_rate();
    let native_channels = native.channels();
    let mut config: StreamConfig = native.into();
    if let Some(buffer_frames) = buffer_frames {
        config.buffer_size = BufferSize::Fixed(buffer_frames);
        eprintln!(
            "audio_capture requested callback period: {} frames ({:.1} ms)",
            buffer_frames,
            buffer_frames as f64 * 1_000.0 / native_rate as f64
        );
    }
    macro_rules! build {
        ($sample:ty) => {
            build_typed_stream::<$sample>(
                device,
                config,
                native_rate,
                native_channels,
                chunk_bytes,
                sender,
                overflowed,
                cpal_failed,
                gstreamer_failed,
            )
        };
    }
    match sample_format {
        SampleFormat::I8 => build!(i8),
        SampleFormat::I16 => build!(i16),
        SampleFormat::I24 => build!(I24),
        SampleFormat::I32 => build!(i32),
        SampleFormat::U8 => build!(u8),
        SampleFormat::U16 => build!(u16),
        SampleFormat::U24 => build!(U24),
        SampleFormat::U32 => build!(u32),
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
    chunk_bytes: usize,
    sender: SyncSender<CaptureMessage>,
    overflowed: Arc<AtomicBool>,
    cpal_failed: Arc<AtomicBool>,
    gstreamer_failed: Arc<AtomicBool>,
) -> Result<(cpal::Stream, CapturePipeline), CaptureError>
where
    T: GStreamerPcmSample,
{
    let converter = CapturePipeline::new::<T>(
        native_rate,
        native_channels,
        chunk_bytes,
        sender.clone(),
        Arc::clone(&overflowed),
        Arc::clone(&gstreamer_failed),
    )?;
    let source = converter.source.clone();
    let error_sender = sender.clone();
    let conversion_failed = Arc::clone(&gstreamer_failed);
    let mut input_frame_index = 0_u64;
    let stream = device
        .build_input_stream(
            config,
            move |data: &[T], _| {
                if let Err(error) = push_native_input(
                    &source,
                    data,
                    native_rate,
                    native_channels,
                    &mut input_frame_index,
                ) {
                    eprintln!("audio_capture GStreamer appsrc failed: {error}");
                    let _ = sender.try_send(CaptureMessage::GStreamerError(error));
                    conversion_failed.store(true, Ordering::SeqCst);
                }
            },
            move |error| {
                let _ = error_sender.try_send(CaptureMessage::CpalError(error.to_string()));
                cpal_failed.store(true, Ordering::SeqCst);
            },
            None,
        )
        .map_err(CaptureError::Build)?;
    Ok((stream, converter))
}

fn push_native_input<T: GStreamerPcmSample>(
    source: &AppSrc,
    data: &[T],
    native_rate: u32,
    native_channels: u16,
    input_frame_index: &mut u64,
) -> Result<(), String> {
    if native_channels == 0 || data.len() % native_channels as usize != 0 {
        return Err("CPAL input buffer is not aligned to native channels".to_owned());
    }
    let frame_count = data.len() / native_channels as usize;
    let mut bytes = Vec::with_capacity(std::mem::size_of_val(data));
    for sample in data {
        sample.append_bytes(&mut bytes);
    }
    let mut buffer = gst::Buffer::from_mut_slice(bytes);
    let buffer_ref = buffer
        .get_mut()
        .ok_or_else(|| "native input buffer is unexpectedly shared".to_owned())?;
    buffer_ref.set_pts(gst::ClockTime::from_nseconds(
        input_frame_index.saturating_mul(1_000_000_000) / u64::from(native_rate),
    ));
    buffer_ref.set_duration(gst::ClockTime::from_nseconds(
        (frame_count as u64).saturating_mul(1_000_000_000) / u64::from(native_rate),
    ));
    source
        .push_buffer(buffer)
        .map_err(|error| format!("GStreamer appsrc rejected native input: {error}"))?;
    *input_frame_index = input_frame_index.saturating_add(frame_count as u64);
    Ok(())
}

fn drain_capture(
    receiver: &Receiver<CaptureMessage>,
    publisher: &r2r::Publisher<AudioFrame>,
    chunk_frames: usize,
    chunk_bytes: usize,
    seq: &mut u64,
    sample_index: &mut u64,
) -> Result<(), CaptureError> {
    loop {
        match receiver.try_recv() {
            Ok(CaptureMessage::Data(data)) => {
                if data.len() != chunk_bytes {
                    return Err(CaptureError::GStreamer(format!(
                        "converted input chunk violates ROS contract: expected {chunk_bytes} bytes, got {}",
                        data.len()
                    )));
                }
                publisher.publish(&audio_message(data, *seq, *sample_index, false))?;
                *seq += 1;
                *sample_index += chunk_frames as u64;
            }
            Ok(CaptureMessage::CpalError(error)) => return Err(CaptureError::Stream(error)),
            Ok(CaptureMessage::GStreamerError(error)) => {
                return Err(CaptureError::GStreamer(error))
            }
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

fn env_optional_u32(name: &str) -> Result<Option<u32>, CaptureError> {
    env::var(name).map_or(Ok(None), |value| {
        value
            .parse()
            .ok()
            .filter(|parsed| *parsed > 0)
            .map(Some)
            .ok_or_else(|| CaptureError::Config(name.to_owned()))
    })
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;
    use std::sync::mpsc::sync_channel;
    use std::sync::Arc;
    use std::time::Duration;

    use super::{
        push_native_input, requested_device_matches, CaptureMessage, CapturePipeline,
        ROS_SAMPLE_RATE,
    };

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
    fn gstreamer_converts_48k_stereo_f32_to_16k_mono_pcm16le() {
        let (sender, receiver) = sync_channel(64);
        let pipeline = CapturePipeline::new::<f32>(
            48_000,
            2,
            ROS_SAMPLE_RATE as usize / 100 * 2,
            sender,
            Arc::new(AtomicBool::new(false)),
            Arc::new(AtomicBool::new(false)),
        )
        .expect("GStreamer input conversion pipeline must start");
        let mut input_frame_index = 0;
        let input = vec![0.25_f32; 48_000 / 50 * 2];
        for _ in 0..5 {
            push_native_input(&pipeline.source, &input, 48_000, 2, &mut input_frame_index)
                .expect("GStreamer must accept consecutive native DJI-format input");
        }

        let mut converted = Vec::new();
        while converted.len() < ROS_SAMPLE_RATE as usize / 50 * 2 {
            let message = receiver
                .recv_timeout(Duration::from_secs(2))
                .expect("GStreamer must emit converted input");
            match message {
                CaptureMessage::Data(data) => converted.extend(data),
                CaptureMessage::CpalError(error) => {
                    panic!("CPAL unexpectedly failed: {error}")
                }
                CaptureMessage::GStreamerError(error) => {
                    panic!("GStreamer conversion unexpectedly failed: {error}")
                }
            }
        }
        assert!(converted.len() >= ROS_SAMPLE_RATE as usize / 50 * 2);
        assert!(converted
            .chunks_exact(2)
            .any(|sample| i16::from_le_bytes([sample[0], sample[1]]) != 0));
        pipeline.stop().expect("GStreamer pipeline must stop");
    }
}
