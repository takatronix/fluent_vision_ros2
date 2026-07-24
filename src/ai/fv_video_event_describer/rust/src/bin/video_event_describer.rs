use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use futures::{FutureExt, StreamExt};
use fv_video_event_describer::{
    EventDetector, Frame, FrameRing, RecorderEvent, Transition, begin_recorder_event,
    changed_ratio, describe, end_recorder_event, patch_recorder_event, prepare_frame,
};
use r2r::fv_episode_msgs::msg::{EnvironmentChange, EnvironmentEvent};
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::sensor_msgs::msg::CompressedImage;
use r2r::{Context, Node, QosProfile};
use uuid::Uuid;

struct Job {
    episode_id: String,
    frames: Vec<Vec<u8>>,
}

struct ResultMessage {
    episode_id: String,
    result: Result<String, String>,
}

enum RecorderCommand {
    Started { episode_id: String },
    Ended { episode_id: String },
    Annotation { episode_id: String, text: String },
    Forget { episode_id: String },
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::create()?;
    let mut node = Node::create(context, "video_event_describer", "")?;
    let image_topic = string_parameter(&node, "image_topic", "/aspa/restamped/color_compressed");
    let change_topic = string_parameter(&node, "change_topic", "/environment/change");
    let annotation_topic = string_parameter(&node, "annotation_topic", "/environment/annotation");
    let sample_fps = positive_f64_parameter(&node, "sample_fps", 1.0)?;
    let window_seconds = positive_i64_parameter(&node, "window_seconds", 32)? as usize;
    let window_frames = (window_seconds as f64 * sample_fps).ceil() as usize;
    let start_ratio = positive_f64_parameter(&node, "start_changed_ratio", 0.03)?;
    let stop_ratio = nonnegative_f64_parameter(&node, "stop_changed_ratio", 0.01)?;
    let stable_frames = positive_i64_parameter(&node, "stable_frames", 2)? as u32;
    let vlm_url = string_parameter(
        &node,
        "vlm_url",
        "http://127.0.0.1:18082/v1/chat/completions",
    );
    let vlm_model = string_parameter(&node, "vlm_model", "qwen3-vl-4b");
    let vlm_timeout =
        Duration::from_secs_f64(positive_f64_parameter(&node, "vlm_timeout_seconds", 120.0)?);
    let recorder_api_url =
        string_parameter(&node, "recorder_api_url", "http://127.0.0.1:7798/api/v1");
    let recorder_profile = string_parameter(&node, "recorder_profile", "aspa_dev");
    let recorder_timeout = Duration::from_secs_f64(positive_f64_parameter(
        &node,
        "recorder_timeout_seconds",
        2.0,
    )?);

    let sensor_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 2,
        reliability: ReliabilityPolicy::BestEffort,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    };
    let event_qos = QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 10,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::Volatile,
        ..QosProfile::default()
    };
    let mut images = node.subscribe::<CompressedImage>(&image_topic, sensor_qos)?;
    let change_publisher =
        node.create_publisher::<EnvironmentChange>(&change_topic, event_qos.clone())?;
    let annotation_publisher =
        node.create_publisher::<EnvironmentEvent>(&annotation_topic, event_qos)?;

    let (job_tx, job_rx) = mpsc::channel::<Job>();
    let (result_tx, result_rx) = mpsc::channel::<ResultMessage>();
    thread::Builder::new()
        .name("qwen3-vl".to_owned())
        .spawn(move || {
            while let Ok(job) = job_rx.recv() {
                let result = describe(&vlm_url, &vlm_model, vlm_timeout, &job.frames);
                if result_tx
                    .send(ResultMessage {
                        episode_id: job.episode_id,
                        result,
                    })
                    .is_err()
                {
                    break;
                }
            }
        })?;

    let (recorder_tx, recorder_rx) = mpsc::channel::<RecorderCommand>();
    thread::Builder::new()
        .name("episode-recorder".to_owned())
        .spawn(move || {
            let mut recorder_events = HashMap::<String, RecorderEvent>::new();
            while let Ok(command) = recorder_rx.recv() {
                match command {
                    RecorderCommand::Started { episode_id } => {
                        match begin_recorder_event(
                            &recorder_api_url,
                            recorder_timeout,
                            &recorder_profile,
                        ) {
                            Ok(recorder_event) => {
                                recorder_events.insert(episode_id, recorder_event);
                            }
                            Err(error) => eprintln!("[video_event_describer] {error}"),
                        }
                    }
                    RecorderCommand::Ended { episode_id } => {
                        let Some(recorder_event) = recorder_events.get(&episode_id) else {
                            continue;
                        };
                        if let Err(error) =
                            end_recorder_event(&recorder_api_url, recorder_timeout, recorder_event)
                        {
                            eprintln!("[video_event_describer] {error}");
                        }
                    }
                    RecorderCommand::Annotation { episode_id, text } => {
                        let Some(recorder_event) = recorder_events.remove(&episode_id) else {
                            continue;
                        };
                        if let Err(error) = patch_recorder_event(
                            &recorder_api_url,
                            recorder_timeout,
                            &recorder_event,
                            &text,
                        ) {
                            eprintln!("[video_event_describer] {error}");
                        }
                    }
                    RecorderCommand::Forget { episode_id } => {
                        recorder_events.remove(&episode_id);
                    }
                }
            }
        })?;

    let mut ring = FrameRing::new(window_frames)?;
    let mut detector =
        EventDetector::new(start_ratio, stop_ratio, stable_frames, window_frames as u64)?;
    let mut sequence = 0_u64;
    let mut active_episode = None::<String>;
    let sample_interval = Duration::from_secs_f64(1.0 / sample_fps);
    let mut last_sample = Instant::now()
        .checked_sub(sample_interval)
        .unwrap_or_else(Instant::now);
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    eprintln!(
        "video_event_describer ready: image={image_topic} fps={sample_fps} window={window_seconds}s"
    );
    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(10));
        while let Some(Some(message)) = images.next().now_or_never() {
            if last_sample.elapsed() < sample_interval {
                continue;
            }
            last_sample = Instant::now();
            let (jpeg, motion_pixels) = match prepare_frame(&message.data) {
                Ok(frame) => frame,
                Err(error) => {
                    eprintln!("[video_event_describer] image ignored: {error}");
                    continue;
                }
            };
            let ratio = ring
                .last()
                .map(|previous| changed_ratio(&previous.motion_pixels, &motion_pixels))
                .transpose()?;
            ring.push(Frame {
                sequence,
                jpeg,
                motion_pixels,
            });
            let Some(ratio) = ratio else {
                sequence += 1;
                continue;
            };
            match detector.update(sequence, ratio) {
                Some(Transition::Started { .. }) => {
                    let episode_id = Uuid::new_v4().to_string();
                    change_publisher.publish(&EnvironmentChange {
                        episode_id: episode_id.clone(),
                        state: "started".to_owned(),
                    })?;
                    recorder_tx.send(RecorderCommand::Started {
                        episode_id: episode_id.clone(),
                    })?;
                    active_episode = Some(episode_id);
                }
                Some(Transition::Ended {
                    start_sequence,
                    end_sequence,
                }) => {
                    if let Some(episode_id) = active_episode.take() {
                        change_publisher.publish(&EnvironmentChange {
                            episode_id: episode_id.clone(),
                            state: "ended".to_owned(),
                        })?;
                        recorder_tx.send(RecorderCommand::Ended {
                            episode_id: episode_id.clone(),
                        })?;
                        job_tx.send(Job {
                            episode_id,
                            frames: ring.interval(start_sequence, end_sequence),
                        })?;
                    }
                }
                None => {}
            }
            sequence += 1;
        }
        while let Ok(message) = result_rx.try_recv() {
            match message.result {
                Ok(text) => {
                    recorder_tx.send(RecorderCommand::Annotation {
                        episode_id: message.episode_id.clone(),
                        text: text.clone(),
                    })?;
                    annotation_publisher.publish(&EnvironmentEvent {
                        episode_id: message.episode_id,
                        annotation_id: Uuid::new_v4().to_string(),
                        text,
                    })?;
                }
                Err(error) => {
                    recorder_tx.send(RecorderCommand::Forget {
                        episode_id: message.episode_id,
                    })?;
                    eprintln!("[video_event_describer] {error}");
                }
            }
        }
    }
    Ok(())
}

fn string_parameter(node: &Node, name: &str, default: &str) -> String {
    node.get_parameter::<String>(name)
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| default.to_owned())
}

fn positive_f64_parameter(node: &Node, name: &str, default: f64) -> Result<f64, String> {
    let value = node.get_parameter::<f64>(name).unwrap_or(default);
    if value <= 0.0 {
        return Err(format!("{name} must be positive"));
    }
    Ok(value)
}

fn nonnegative_f64_parameter(node: &Node, name: &str, default: f64) -> Result<f64, String> {
    let value = node.get_parameter::<f64>(name).unwrap_or(default);
    if value < 0.0 {
        return Err(format!("{name} must be nonnegative"));
    }
    Ok(value)
}

fn positive_i64_parameter(node: &Node, name: &str, default: i64) -> Result<i64, String> {
    let value = node.get_parameter::<i64>(name).unwrap_or(default);
    if value <= 0 {
        return Err(format!("{name} must be positive"));
    }
    Ok(value)
}
