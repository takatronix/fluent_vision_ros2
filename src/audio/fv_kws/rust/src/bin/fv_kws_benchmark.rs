use std::env;
use std::path::{Path, PathBuf};
use std::time::Instant;

use fv_kws_ros2::manifest::RuntimeManifest;
use fv_kws_ros2::spotter::{VoskSpotter, contains_wake_phrase};
use serde::Serialize;

const SAMPLE_RATE_HZ: u32 = 16_000;
const FRAME_SAMPLES: usize = 160;

struct BenchmarkOptions {
    manifest_path: PathBuf,
    wav_paths: Vec<PathBuf>,
    normalize_rms_dbfs: Option<f32>,
}

#[derive(Serialize)]
struct BenchmarkResult {
    path: PathBuf,
    duration_samples: usize,
    input_rms_dbfs: Option<f32>,
    applied_gain_db: f32,
    detected: bool,
    detected_sample_index: Option<usize>,
    keyword: Option<String>,
    transcript: String,
    inference_elapsed_us: u128,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let options = parse_options()?;
    let manifest = RuntimeManifest::load_verified(options.manifest_path)?;
    let mut spotter = VoskSpotter::load(&manifest)?;
    let results = options
        .wav_paths
        .iter()
        .map(|path| benchmark_file(&mut spotter, path, options.normalize_rms_dbfs))
        .collect::<Result<Vec<_>, _>>()?;
    println!("{}", serde_json::to_string_pretty(&results)?);
    Ok(())
}

fn parse_options() -> Result<BenchmarkOptions, Box<dyn std::error::Error>> {
    let mut arguments = env::args().skip(1);
    let mut normalize_rms_dbfs = None;
    let mut paths = Vec::new();
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--normalize-rms-dbfs" => {
                normalize_rms_dbfs = Some(
                    arguments
                        .next()
                        .ok_or("--normalize-rms-dbfs requires a value")?
                        .parse()?,
                );
            }
            _ if argument.starts_with("--") => {
                return Err(format!("unknown option: {argument}").into());
            }
            _ => paths.push(PathBuf::from(argument)),
        }
    }
    if paths.len() < 2 {
        return Err(
            "usage: fv_kws_benchmark [--normalize-rms-dbfs VALUE] RUNTIME_JSON WAV [WAV ...]"
                .into(),
        );
    }
    let manifest_path = paths.remove(0);
    Ok(BenchmarkOptions {
        manifest_path,
        wav_paths: paths,
        normalize_rms_dbfs,
    })
}

fn benchmark_file(
    spotter: &mut VoskSpotter,
    path: &Path,
    normalize_rms_dbfs: Option<f32>,
) -> Result<BenchmarkResult, Box<dyn std::error::Error>> {
    spotter.reset();
    let samples = read_wav(path)?;
    let input_rms_dbfs = rms_dbfs(&samples);
    let applied_gain_db = normalize_rms_dbfs
        .zip(input_rms_dbfs)
        .map_or(0.0, |(target, input)| target - input);
    let gain = 10.0_f32.powf(applied_gain_db / 20.0);
    let normalized_samples = samples
        .iter()
        .map(|sample| (f32::from(*sample) * gain).clamp(-32_768.0, 32_767.0) as i16)
        .collect::<Vec<_>>();
    let started_at = Instant::now();
    for frame in normalized_samples.chunks(FRAME_SAMPLES) {
        spotter.accept(frame)?;
    }
    let transcript = spotter.finalize_text();
    let detected = contains_wake_phrase(&transcript);
    let elapsed = started_at.elapsed().as_micros();
    spotter.reset();
    Ok(BenchmarkResult {
        path: path.to_path_buf(),
        duration_samples: samples.len(),
        input_rms_dbfs,
        applied_gain_db,
        detected,
        detected_sample_index: detected.then_some(samples.len()),
        keyword: detected.then(|| fv_kws_ros2::WAKE_KEYWORD.to_owned()),
        transcript,
        inference_elapsed_us: elapsed,
    })
}

fn read_wav(path: &Path) -> Result<Vec<i16>, Box<dyn std::error::Error>> {
    let mut reader = hound::WavReader::open(path)?;
    let specification = reader.spec();
    if specification.channels != 1
        || specification.sample_rate != SAMPLE_RATE_HZ
        || specification.bits_per_sample != 16
        || specification.sample_format != hound::SampleFormat::Int
    {
        return Err(format!(
            "{} must be mono 16-bit PCM at {SAMPLE_RATE_HZ} Hz",
            path.display()
        )
        .into());
    }
    reader
        .samples::<i16>()
        .collect::<Result<Vec<_>, _>>()
        .map_err(Into::into)
}

fn rms_dbfs(samples: &[i16]) -> Option<f32> {
    if samples.is_empty() {
        return None;
    }
    let mean_square = samples
        .iter()
        .map(|sample| {
            let normalized = f64::from(*sample) / 32_768.0;
            normalized * normalized
        })
        .sum::<f64>()
        / samples.len() as f64;
    if mean_square == 0.0 {
        return None;
    }
    Some((20.0 * mean_square.sqrt().log10()) as f32)
}
