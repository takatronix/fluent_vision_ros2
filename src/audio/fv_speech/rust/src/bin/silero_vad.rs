use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use fv_speech_ros2::audio::{AudioPacket, AudioValidator, pcm16le_to_i16};
use fv_speech_ros2::manifest::{RuntimeManifest, default_runtime_manifest_path};
use fv_speech_ros2::ros::{header, speech_qos};
use fv_speech_ros2::vad::{SileroVad, VadResult};
use fv_speech_ros2::{INPUT_SOURCE_ID, INPUT_STREAM_ID, VAD_SOURCE_ID, VAD_STREAM_ID};
use r2r::fv_speech_interfaces::msg::{AudioFrame, VoiceActivity};
use r2r::{Context, Node};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_path = std::env::var_os("ASPA_PARAKEET_RUNTIME_MANIFEST")
        .map(PathBuf::from)
        .unwrap_or_else(default_runtime_manifest_path);
    let manifest = RuntimeManifest::load_verified(manifest_path)?;
    manifest.initialize_ort()?;

    let context = Context::create()?;
    let mut node = Node::create(context, "silero_vad", "")?;
    let model_path = required_string(&node, "model_path")?;
    let threshold = node.get_parameter::<f64>("threshold").unwrap_or(0.5) as f32;
    let mut vad = SileroVad::load(&PathBuf::from(&model_path), threshold)?;
    let mut validator = AudioValidator::default();
    let qos = speech_qos(100);
    let mut audio = node.subscribe::<AudioFrame>("/audio/mic/frame", qos.clone())?;
    let publisher = node.create_publisher::<VoiceActivity>("/dialogue/vad/activity", qos)?;
    let mut base_sample_index = None;
    let mut output_seq = 0_u64;
    eprintln!(
        "silero_vad ready: model={model_path}, threshold={}",
        vad.threshold()
    );

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        while let Some(Some(message)) = audio.next().now_or_never() {
            if message.source_id != INPUT_SOURCE_ID || message.stream_id != INPUT_STREAM_ID {
                return Err("Silero input identity mismatch".into());
            }
            validator.validate(&AudioPacket {
                seq: message.seq,
                sample_index: message.sample_index,
                frame_count: message.frame_count,
                data: &message.data,
                final_marker: message.final_,
                encoding: &message.encoding,
                sample_rate_hz: message.sample_rate_hz,
                channels: message.channels,
                bit_depth: message.bit_depth,
                layout: &message.layout,
            })?;
            if message.final_ {
                let base = base_sample_index.ok_or("Silero final marker arrived before audio")?;
                for result in vad.flush()? {
                    publish_result(&publisher, &mut output_seq, base, result)?;
                }
                publisher.publish(&VoiceActivity {
                    header: header(VAD_STREAM_ID),
                    source_id: VAD_SOURCE_ID.to_owned(),
                    stream_id: VAD_STREAM_ID.to_owned(),
                    seq: output_seq,
                    sample_index: message.sample_index,
                    frame_count: 0,
                    state: String::new(),
                    speech_probability: 0.0,
                    final_: true,
                })?;
                output_seq += 1;
                continue;
            }
            let base = *base_sample_index.get_or_insert(message.sample_index);
            for result in vad.push_i16(&pcm16le_to_i16(&message.data)?)? {
                publish_result(&publisher, &mut output_seq, base, result)?;
            }
        }
    }
    Ok(())
}

fn publish_result(
    publisher: &r2r::Publisher<VoiceActivity>,
    output_seq: &mut u64,
    base_sample_index: u64,
    result: VadResult,
) -> Result<(), r2r::Error> {
    publisher.publish(&VoiceActivity {
        header: header(VAD_STREAM_ID),
        source_id: VAD_SOURCE_ID.to_owned(),
        stream_id: VAD_STREAM_ID.to_owned(),
        seq: *output_seq,
        sample_index: base_sample_index + result.window_start_frame,
        frame_count: result.window_frames,
        state: if result.is_speech {
            "speech"
        } else {
            "silence"
        }
        .to_owned(),
        speech_probability: result.probability,
        final_: false,
    })?;
    *output_seq += 1;
    Ok(())
}

fn required_string(node: &Node, name: &str) -> Result<String, Box<dyn std::error::Error>> {
    let value = node.get_parameter::<String>(name)?;
    if value.is_empty() {
        return Err(format!("ROS parameter {name} must be non-empty").into());
    }
    Ok(value)
}
