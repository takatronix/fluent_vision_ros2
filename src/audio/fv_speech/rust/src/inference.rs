use std::sync::mpsc::{Receiver, Sender};

use ort::ep::ExecutionProvider as _;
use parakeet_rs::{ExecutionConfig, Nemotron, NemotronHandle, NemotronMode};

use crate::asr::InferenceCommand;
use crate::manifest::RuntimeManifest;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InferenceEvent {
    Ready {
        provider: &'static str,
        chunk_samples: usize,
    },
    Partial {
        session_id: String,
        user_turn_id: String,
        stream_id: String,
        start_sample_index: u64,
        end_sample_index: u64,
        text: String,
    },
    Final {
        session_id: String,
        user_turn_id: String,
        stream_id: String,
        start_sample_index: u64,
        end_sample_index: u64,
        text: String,
    },
    Fatal(String),
}

struct ActiveTurn {
    session_id: String,
    user_turn_id: String,
    stream_id: String,
    start_sample_index: u64,
    samples: Vec<f32>,
    streaming_pending: Vec<f32>,
    streaming_chunk_samples: usize,
    streaming: Nemotron,
    last_partial: String,
}

pub fn run_inference_thread(
    manifest: RuntimeManifest,
    commands: Receiver<InferenceCommand>,
    events: Sender<InferenceEvent>,
) {
    if let Err(error) = run(manifest, commands, &events) {
        let _ = events.send(InferenceEvent::Fatal(error.to_string()));
    }
}

fn run(
    manifest: RuntimeManifest,
    commands: Receiver<InferenceCommand>,
    events: &Sender<InferenceEvent>,
) -> Result<(), Box<dyn std::error::Error>> {
    if !ort::ep::CUDA::default().is_available()? {
        return Err("CUDAExecutionProvider is not compiled into ONNX Runtime".into());
    }
    // Registration failure is fatal. ORT may still place shape/helper nodes on
    // its built-in CPU provider, but model computation is requested from CUDA
    // only; there is no TensorRT registration or engine-cache fallback path.
    let execution = ExecutionConfig::new().with_custom_configure(|builder| {
        let cuda = ort::ep::CUDA::default()
            .with_device_id(0)
            .with_tf32(false)
            .build()
            .error_on_failure();
        Ok(builder.with_execution_providers([cuda])?)
    });
    let handle = NemotronHandle::from_pretrained(&manifest.model_dir, Some(execution))?;
    if handle.mode() != NemotronMode::Multilingual || handle.chunk_samples() != 5120 {
        return Err("expected multilingual 320 ms Nemotron model".into());
    }
    let mut warmup = new_model(&handle, &manifest.language)?;
    let _ = warmup.transcribe_audio(&vec![0.0_f32; handle.chunk_samples()])?;
    eprintln!("parakeet CUDA warmup complete: provider=cuda,cpu-helper,tf32=false");
    events.send(InferenceEvent::Ready {
        provider: "cuda",
        chunk_samples: handle.chunk_samples(),
    })?;

    let mut active: Option<ActiveTurn> = None;
    while let Ok(command) = commands.recv() {
        match command {
            InferenceCommand::Start {
                session_id,
                user_turn_id,
                stream_id,
                start_sample_index,
            } => {
                if active.is_some() {
                    return Err("ASR start received while a turn is active".into());
                }
                active = Some(ActiveTurn {
                    session_id,
                    user_turn_id,
                    stream_id,
                    start_sample_index,
                    samples: Vec::new(),
                    streaming_pending: Vec::new(),
                    streaming_chunk_samples: handle.chunk_samples(),
                    streaming: new_model(&handle, &manifest.language)?,
                    last_partial: String::new(),
                });
            }
            InferenceCommand::Audio(samples) => {
                let turn = active
                    .as_mut()
                    .ok_or("ASR audio received without active turn")?;
                let waveform = samples
                    .into_iter()
                    .map(|sample| f32::from(sample) / 32768.0)
                    .collect::<Vec<_>>();
                turn.samples.extend_from_slice(&waveform);
                turn.streaming_pending.extend_from_slice(&waveform);
                while turn.streaming_pending.len() >= turn.streaming_chunk_samples {
                    let remainder = turn
                        .streaming_pending
                        .split_off(turn.streaming_chunk_samples);
                    let chunk = std::mem::replace(&mut turn.streaming_pending, remainder);
                    let _ = turn.streaming.transcribe_chunk(&chunk)?;
                }
                let text = turn.streaming.get_transcript();
                if !text.is_empty() && text != turn.last_partial {
                    turn.last_partial.clone_from(&text);
                    let processed = turn.samples.len() - turn.streaming_pending.len();
                    events.send(InferenceEvent::Partial {
                        session_id: turn.session_id.clone(),
                        user_turn_id: turn.user_turn_id.clone(),
                        stream_id: turn.stream_id.clone(),
                        start_sample_index: turn.start_sample_index,
                        end_sample_index: turn.start_sample_index + processed as u64,
                        text,
                    })?;
                }
            }
            InferenceCommand::Finish {
                end_sample_index,
                valid_samples,
            } => {
                let mut turn = active
                    .take()
                    .ok_or("ASR finish received without active turn")?;
                let bounded = end_sample_index
                    .checked_sub(turn.start_sample_index)
                    .ok_or("ASR finish precedes turn start")?
                    as usize;
                if bounded != valid_samples || valid_samples > turn.samples.len() {
                    return Err("ASR finish sample span is invalid".into());
                }
                turn.samples.truncate(valid_samples);
                let mut final_model = new_model(&handle, &manifest.language)?;
                let text = final_model.transcribe_audio(&turn.samples)?;
                events.send(InferenceEvent::Final {
                    session_id: turn.session_id,
                    user_turn_id: turn.user_turn_id,
                    stream_id: turn.stream_id,
                    start_sample_index: turn.start_sample_index,
                    end_sample_index,
                    text,
                })?;
            }
            InferenceCommand::Cancel => active = None,
            InferenceCommand::Shutdown => return Ok(()),
        }
    }
    Err("ROS 2 ASR command channel disconnected without shutdown".into())
}

fn new_model(
    handle: &NemotronHandle,
    language: &str,
) -> Result<Nemotron, Box<dyn std::error::Error>> {
    let mut model = Nemotron::from_shared(handle);
    if model.mode() != NemotronMode::Multilingual {
        return Err("ASR requires the multilingual Nemotron model".into());
    }
    model.set_target_lang(language)?;
    Ok(model)
}
