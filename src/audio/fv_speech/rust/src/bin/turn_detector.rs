use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use futures::{FutureExt, StreamExt};
use fv_speech_ros2::ros::{header, speech_qos};
use fv_speech_ros2::turn::{Activity, Turn, TurnDetector, TurnError};
use fv_speech_ros2::{SESSION_ID, TURN_STREAM_ID, VAD_SOURCE_ID, VAD_STREAM_ID};
use r2r::fv_speech_interfaces::msg::{TurnEvent, VoiceActivity};
use r2r::{Context, Node};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::create()?;
    let mut node = Node::create(context, "turn_detector", "")?;
    let session_id = string_parameter(&node, "session_id", SESSION_ID);
    let output_stream = string_parameter(&node, "output_stream_id", TURN_STREAM_ID);
    let end_silence_frames = node
        .get_parameter::<i64>("end_silence_frames")
        .unwrap_or(12_000);
    if end_silence_frames <= 0 {
        return Err("end_silence_frames must be positive".into());
    }
    let turn_id_prefix = string_parameter(&node, "turn_id_prefix", "user-turn");
    let mut detector = TurnDetector::new(
        session_id.clone(),
        output_stream.clone(),
        end_silence_frames as u64,
        turn_id_prefix,
    )?;
    let qos = speech_qos(256);
    let mut activity = node.subscribe::<VoiceActivity>("/dialogue/vad/activity", qos.clone())?;
    let publisher = node.create_publisher::<TurnEvent>("/dialogue/turn/event", qos)?;
    let mut finished = false;
    eprintln!("turn_detector ready: end_silence_frames={end_silence_frames}");

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(5));
        while let Some(Some(message)) = activity.next().now_or_never() {
            if finished {
                return Err("turn detector received activity after final marker".into());
            }
            if message.source_id != VAD_SOURCE_ID || message.stream_id != VAD_STREAM_ID {
                return Err("turn detector input identity mismatch".into());
            }
            if message.final_ {
                if let Some(event) = detector.finish(message.sample_index, message.seq)? {
                    publish_turn(&publisher, event)?;
                }
                publisher.publish(&TurnEvent {
                    header: header(&output_stream),
                    session_id: session_id.clone(),
                    user_turn_id: String::new(),
                    stream_id: output_stream.clone(),
                    seq: detector.next_output_seq(),
                    sample_index: message.sample_index,
                    state: String::new(),
                    confidence_present: false,
                    confidence: 0.0,
                    final_: true,
                })?;
                finished = true;
                continue;
            }
            let next = Activity {
                seq: message.seq,
                sample_index: message.sample_index,
                frame_count: message.frame_count,
                state: message.state,
            };
            let event = match detector.push(next.clone()) {
                Ok(event) => event,
                Err(TurnError::Sequence { .. } | TurnError::Sample { .. }) => {
                    eprintln!("turn_detector warning: VoiceActivity stream resynchronized");
                    if let Some(ended) = detector.resynchronize_input() {
                        publish_turn(&publisher, ended)?;
                    }
                    detector.push(next)?
                }
                Err(error) => return Err(error.into()),
            };
            if let Some(event) = event {
                publish_turn(&publisher, event)?;
            }
        }
    }
    Ok(())
}

fn publish_turn(publisher: &r2r::Publisher<TurnEvent>, event: Turn) -> Result<(), r2r::Error> {
    publisher.publish(&TurnEvent {
        header: header(&event.stream_id),
        session_id: event.session_id,
        user_turn_id: event.user_turn_id,
        stream_id: event.stream_id,
        seq: event.seq,
        sample_index: event.sample_index,
        state: event.state.to_owned(),
        confidence_present: false,
        confidence: 0.0,
        final_: false,
    })
}

fn string_parameter(node: &Node, name: &str, default: &str) -> String {
    node.get_parameter::<String>(name)
        .ok()
        .filter(|value| !value.is_empty())
        .unwrap_or_else(|| default.to_owned())
}
