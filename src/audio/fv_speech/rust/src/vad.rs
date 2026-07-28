use std::collections::VecDeque;
use std::fs::File;
use std::io::Read;
use std::path::Path;

use ort::session::Session;
use ort::value::Tensor;
use sha2::{Digest, Sha256};
use thiserror::Error;

pub const EXPECTED_MODEL_SHA256: &str =
    "7ed98ddbad84ccac4cd0aeb3099049280713df825c610a8ed34543318f1b2c49";
pub const WINDOW_FRAMES: usize = 512;
pub const CONTEXT_FRAMES: usize = 64;

#[derive(Clone, Debug, PartialEq)]
pub struct VadResult {
    pub probability: f32,
    pub is_speech: bool,
    pub window_start_frame: u64,
    pub window_frames: u32,
    pub padded_frames: u32,
}

#[derive(Debug, Error)]
pub enum VadError {
    #[error("Silero threshold must be in [0, 1]")]
    Threshold,
    #[error("cannot read Silero model: {0}")]
    ModelIo(#[from] std::io::Error),
    #[error("Silero model sha256 mismatch: {0}")]
    ModelHash(String),
    #[error("Silero ONNX Runtime error: {0}")]
    Ort(#[from] ort::Error),
    #[error("Silero ONNX model contract mismatch: {0}")]
    Contract(String),
}

pub struct SileroVad {
    threshold: f32,
    session: Session,
    state: Vec<f32>,
    context: Vec<f32>,
    buffer: VecDeque<f32>,
    next_window_start: u64,
}

impl SileroVad {
    pub fn load(path: &Path, threshold: f32) -> Result<Self, VadError> {
        if !(0.0..=1.0).contains(&threshold) {
            return Err(VadError::Threshold);
        }
        let actual = sha256_file(path)?;
        if actual != EXPECTED_MODEL_SHA256 {
            return Err(VadError::ModelHash(actual));
        }
        // Silero runs one tiny 32 ms recurrent window at a time. ORT's
        // automatic intra-op pool fans each inference out across the host
        // cores, which costs more scheduling/coordination CPU than the model
        // work itself on Thor. Keep this session single-threaded; Parakeet's
        // separate CUDA session is unaffected.
        let session = Session::builder()?
            .with_intra_threads(1)
            .map_err(|error| VadError::Ort(error.into()))?
            .commit_from_file(path)?;
        let inputs = session
            .inputs()
            .iter()
            .map(|input| input.name())
            .collect::<Vec<_>>();
        let outputs = session
            .outputs()
            .iter()
            .map(|output| output.name())
            .collect::<Vec<_>>();
        if inputs != ["input", "state", "sr"] || outputs != ["output", "stateN"] {
            return Err(VadError::Contract(format!(
                "inputs={inputs:?}, outputs={outputs:?}"
            )));
        }
        Ok(Self {
            threshold,
            session,
            state: vec![0.0; 2 * 128],
            context: vec![0.0; CONTEXT_FRAMES],
            buffer: VecDeque::new(),
            next_window_start: 0,
        })
    }

    pub fn threshold(&self) -> f32 {
        self.threshold
    }

    /// Drops recurrent and buffered state after an upstream audio clock jump.
    /// The loaded session is retained, so resynchronization is inexpensive.
    pub fn reset(&mut self) {
        self.state.fill(0.0);
        self.context.fill(0.0);
        self.buffer.clear();
        self.next_window_start = 0;
    }

    pub fn push_i16(&mut self, samples: &[i16]) -> Result<Vec<VadResult>, VadError> {
        self.buffer
            .extend(samples.iter().map(|sample| f32::from(*sample) / 32768.0));
        let mut results = Vec::new();
        while self.buffer.len() >= WINDOW_FRAMES {
            let window = self.buffer.drain(..WINDOW_FRAMES).collect::<Vec<_>>();
            results.push(self.run_window(window, 0)?);
        }
        Ok(results)
    }

    pub fn flush(&mut self) -> Result<Vec<VadResult>, VadError> {
        if self.buffer.is_empty() {
            return Ok(Vec::new());
        }
        let actual = self.buffer.len();
        let mut window = self.buffer.drain(..).collect::<Vec<_>>();
        window.resize(WINDOW_FRAMES, 0.0);
        Ok(vec![self.run_window(window, WINDOW_FRAMES - actual)?])
    }

    fn run_window(
        &mut self,
        window: Vec<f32>,
        padded_frames: usize,
    ) -> Result<VadResult, VadError> {
        let mut model_input = Vec::with_capacity(CONTEXT_FRAMES + WINDOW_FRAMES);
        model_input.extend_from_slice(&self.context);
        model_input.extend_from_slice(&window);
        let input = Tensor::from_array(([1_usize, CONTEXT_FRAMES + WINDOW_FRAMES], model_input))?;
        let state = Tensor::from_array(([2_usize, 1, 128], self.state.clone()))?;
        let sample_rate = Tensor::from_array(((), vec![16_000_i64]))?;
        let outputs = self.session.run(ort::inputs! {
            "input" => input,
            "state" => state,
            "sr" => sample_rate,
        })?;
        let (output_shape, output) = outputs["output"].try_extract_tensor::<f32>()?;
        let (state_shape, next_state) = outputs["stateN"].try_extract_tensor::<f32>()?;
        if output_shape.as_ref() != [1, 1]
            || output.len() != 1
            || state_shape.as_ref() != [2, 1, 128]
            || next_state.len() != 256
        {
            return Err(VadError::Contract(format!(
                "output={output_shape:?}, stateN={state_shape:?}"
            )));
        }
        let probability = output[0];
        if !(0.0..=1.0).contains(&probability) {
            return Err(VadError::Contract(format!(
                "invalid probability {probability}"
            )));
        }
        self.state.copy_from_slice(next_state);
        self.context
            .copy_from_slice(&window[WINDOW_FRAMES - CONTEXT_FRAMES..]);
        let result = VadResult {
            probability,
            is_speech: probability >= self.threshold,
            window_start_frame: self.next_window_start,
            window_frames: WINDOW_FRAMES as u32,
            padded_frames: padded_frames as u32,
        };
        self.next_window_start += WINDOW_FRAMES as u64;
        Ok(result)
    }
}

fn sha256_file(path: &Path) -> Result<String, std::io::Error> {
    let mut source = File::open(path)?;
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 1024 * 1024];
    loop {
        let read = source.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        digest.update(&buffer[..read]);
    }
    Ok(format!("{:x}", digest.finalize()))
}
