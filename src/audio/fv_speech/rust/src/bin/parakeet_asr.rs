use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Sender, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use fv_speech_ros2::asr::{AsrRuntime, BufferedAudio, InferenceCommand};
use fv_speech_ros2::audio::{AudioPacket, AudioValidation, AudioValidator, pcm16le_to_i16};
use fv_speech_ros2::inference::{InferenceEvent, run_inference_thread};
use fv_speech_ros2::manifest::{RuntimeManifest, default_runtime_manifest_path};
use fv_speech_ros2::ros::{header, ready_qos, speech_qos};
use fv_speech_ros2::{INPUT_SOURCE_ID, INPUT_STREAM_ID, TRANSCRIPT_STREAM_ID};
use r2r::fv_speech_interfaces::msg::{AsrControl, AudioFrame, Transcript};
use r2r::std_msgs::msg::Bool;
use r2r::{Context, Node};

const READY_HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_path = std::env::var_os("ASPA_PARAKEET_RUNTIME_MANIFEST")
        .map(PathBuf::from)
        .unwrap_or_else(default_runtime_manifest_path);
    let manifest = RuntimeManifest::load_verified(&manifest_path)?;
    eprintln!("parakeet_asr startup: CUDA runtime manifest validated");
    manifest.initialize_ort()?;
    eprintln!("parakeet_asr startup: pinned ONNX Runtime loaded");

    eprintln!("parakeet_asr startup: creating ROS 2 node");
    let context = Context::create()?;
    let mut node = Node::create(context, "parakeet_asr", "")?;
    eprintln!("parakeet_asr startup: ROS 2 node created");
    let history_frames = node
        .get_parameter::<i64>("history_frames")
        .unwrap_or(32_000);
    if history_frames <= 0 {
        return Err("history_frames must be positive".into());
    }
    eprintln!("parakeet_asr startup: parameters loaded");

    let qos = speech_qos(256);
    let mut audio = node.subscribe::<AudioFrame>("/audio/mic/frame", qos.clone())?;
    let mut controls = node.subscribe::<AsrControl>("/dialogue/asr/control", qos.clone())?;
    let transcripts = node.create_publisher::<Transcript>("/dialogue/asr/transcript", qos)?;
    let ready = node.create_publisher::<Bool>("/dialogue/asr/ready", ready_qos())?;
    ready.publish(&Bool { data: false })?;

    let (command_sender, command_receiver) = mpsc::channel();
    let (event_sender, event_receiver) = mpsc::channel();
    let inference = thread::Builder::new()
        .name("parakeet-cuda-inference".to_owned())
        .spawn(move || run_inference_thread(manifest, command_receiver, event_sender))?;
    let mut runtime = AsrRuntime::new(INPUT_STREAM_ID.to_owned(), history_frames as u64)?;
    let mut validator = AudioValidator::default();
    let mut inference_ready = false;
    let mut next_ready_heartbeat = Instant::now();
    let mut transcript_seq = 0_u64;
    eprintln!(
        "parakeet_asr loading pinned CUDA model from {}",
        manifest_path.display()
    );

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    let run_result = (|| -> Result<(), Box<dyn std::error::Error>> {
        while running.load(Ordering::SeqCst) {
            node.spin_once(Duration::from_millis(5));
            while let Some(Some(message)) = audio.next().now_or_never() {
                handle_audio(&mut validator, &mut runtime, &command_sender, message)?;
            }
            while let Some(Some(message)) = controls.next().now_or_never() {
                let commands = runtime.push_control(
                    &message.action,
                    &message.session_id,
                    &message.user_turn_id,
                    &message.stream_id,
                    message.seq,
                    message.start_sample_index,
                    message.stop_sample_index,
                )?;
                send_commands(&command_sender, commands)?;
            }
            loop {
                match event_receiver.try_recv() {
                    Ok(InferenceEvent::Ready {
                        provider,
                        chunk_samples,
                    }) => {
                        if inference_ready || provider != "cuda" || chunk_samples != 5120 {
                            return Err("invalid parakeet CUDA readiness event".into());
                        }
                        inference_ready = true;
                        ready.publish(&Bool { data: true })?;
                        next_ready_heartbeat = Instant::now() + READY_HEARTBEAT_INTERVAL;
                        eprintln!("parakeet_asr CUDA ready: chunk_samples={chunk_samples}");
                    }
                    Ok(event @ (InferenceEvent::Partial { .. } | InferenceEvent::Final { .. })) => {
                        if !inference_ready {
                            return Err(
                                "parakeet emitted a transcript before CUDA readiness".into()
                            );
                        }
                        publish_transcript(&transcripts, &mut transcript_seq, event)?;
                    }
                    Ok(InferenceEvent::Fatal(error)) => {
                        return Err(format!("parakeet CUDA inference failed: {error}").into());
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        return Err("parakeet CUDA inference thread exited unexpectedly".into());
                    }
                }
            }
            let now = Instant::now();
            if ready_heartbeat_due(inference_ready, now, next_ready_heartbeat) {
                ready.publish(&Bool { data: true })?;
                next_ready_heartbeat = now + READY_HEARTBEAT_INTERVAL;
            }
        }
        Ok(())
    })();
    let _ = command_sender.send(InferenceCommand::Shutdown);
    let join_result = inference
        .join()
        .map_err(|_| "parakeet CUDA inference thread panicked");
    run_result?;
    join_result?;
    Ok(())
}

fn ready_heartbeat_due(inference_ready: bool, now: Instant, next: Instant) -> bool {
    inference_ready && now >= next
}

fn handle_audio(
    validator: &mut AudioValidator,
    runtime: &mut AsrRuntime,
    sender: &Sender<InferenceCommand>,
    message: AudioFrame,
) -> Result<(), Box<dyn std::error::Error>> {
    if message.source_id != INPUT_SOURCE_ID || message.stream_id != INPUT_STREAM_ID {
        return Err("parakeet input identity mismatch".into());
    }
    let validation = validator.validate_resyncing(&AudioPacket {
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
    if let AudioValidation::Resynchronized(reason) = validation {
        eprintln!("parakeet_asr warning: audio stream resynchronized: {reason}");
        send_commands(sender, runtime.resynchronize_audio())?;
    }
    if message.final_ {
        return Ok(());
    }
    let commands = runtime.push_audio(BufferedAudio {
        sample_index: message.sample_index,
        samples: pcm16le_to_i16(&message.data)?,
    })?;
    send_commands(sender, commands)?;
    Ok(())
}

fn send_commands(
    sender: &Sender<InferenceCommand>,
    commands: Vec<InferenceCommand>,
) -> Result<(), Box<dyn std::error::Error>> {
    for command in commands {
        sender
            .send(command)
            .map_err(|_| "parakeet CUDA inference command channel closed")?;
    }
    Ok(())
}

fn publish_transcript(
    publisher: &r2r::Publisher<Transcript>,
    transcript_seq: &mut u64,
    event: InferenceEvent,
) -> Result<(), Box<dyn std::error::Error>> {
    let (kind, session_id, user_turn_id, stream_id, start, end, text) = match event {
        InferenceEvent::Partial {
            session_id,
            user_turn_id,
            stream_id,
            start_sample_index,
            end_sample_index,
            text,
        } => (
            "partial",
            session_id,
            user_turn_id,
            stream_id,
            start_sample_index,
            end_sample_index,
            text,
        ),
        InferenceEvent::Final {
            session_id,
            user_turn_id,
            stream_id,
            start_sample_index,
            end_sample_index,
            text,
        } => (
            "final",
            session_id,
            user_turn_id,
            stream_id,
            start_sample_index,
            end_sample_index,
            text,
        ),
        _ => return Err("non-transcript event passed to publisher".into()),
    };
    if stream_id != INPUT_STREAM_ID || start > end {
        return Err("parakeet transcript identity or sample span is invalid".into());
    }
    publisher.publish(&Transcript {
        header: header(TRANSCRIPT_STREAM_ID),
        kind: kind.to_owned(),
        session_id,
        user_turn_id,
        stream_id: TRANSCRIPT_STREAM_ID.to_owned(),
        seq: *transcript_seq,
        text,
        start_sample_index: start,
        end_sample_index: end,
    })?;
    *transcript_seq += 1;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::ready_heartbeat_due;

    #[test]
    fn readiness_heartbeat_requires_a_ready_runtime_and_elapsed_deadline() {
        let now = Instant::now();
        assert!(!ready_heartbeat_due(false, now, now));
        assert!(!ready_heartbeat_due(
            true,
            now,
            now + Duration::from_millis(1)
        ));
        assert!(ready_heartbeat_due(true, now, now));
    }
}
