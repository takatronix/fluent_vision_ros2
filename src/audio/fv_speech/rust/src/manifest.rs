use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

pub const PARAKEET_REVISION: &str = "d317e75cc3adfc15cdeafe9d702a28b05d1006e4";
pub const ONNXRUNTIME_REVISION: &str = "8c546c37b43caaca1fa25db430dab94b901cf277";
pub const EXPECTED_GOLDEN_TEXT_SHA256: &str =
    "4fcaced359f13a516ac029c0d3f36e0629bc1354a71715eb71e8126ffb6126f4";
pub const EXPECTED_GOLDEN_TEXT: &str =
    "うちの中学は弁当性で持っていけない場合は五十円の学校販売のパンを買う";
pub const EXPECTED_GOLDEN_WAV_SHA256: &str =
    "460bd8dccb0d2a5f4e29c628f837be4082d13defc64c3fc21dd1b6bb0e119095";
pub const EXPECTED_CUDA_TOOLKIT_VERSION: &str = "13.2";
const EXPECTED_RUNTIME_DEPENDENCIES: [&str; 14] = [
    "libcudnn.so.9",
    "libcudnn_adv.so.9",
    "libcudnn_cnn.so.9",
    "libcudnn_engines_precompiled.so.9",
    "libcudnn_engines_runtime_compiled.so.9",
    "libcudnn_engines_tensor_ir.so.9",
    "libcudnn_ext.so.9",
    "libcudnn_graph.so.9",
    "libcudnn_heuristic.so.9",
    "libcudnn_ops.so.9",
    "libcublasLt.so.13",
    "libcublas.so.13",
    "libcufft.so.12",
    "libcudart.so.13",
];

pub fn default_runtime_manifest_path() -> PathBuf {
    if let Some(data_home) = std::env::var_os("XDG_DATA_HOME") {
        let data_home = PathBuf::from(data_home);
        if data_home.is_absolute() {
            return data_home.join("aspa-navigation/parakeet-cuda/runtime.json");
        }
    }
    if let Some(home) = std::env::var_os("HOME") {
        return PathBuf::from(home).join(".local/share/aspa-navigation/parakeet-cuda/runtime.json");
    }
    PathBuf::from("/var/lib/aspa-navigation/parakeet-cuda/runtime.json")
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerificationReceipt {
    pub method: String,
    pub cuda_kernel_rows: u64,
    pub probe_binary: PathBuf,
    pub golden_wav: PathBuf,
    pub nsys_report: PathBuf,
    pub probe_log: PathBuf,
    pub nvidia_driver_version: String,
    pub cuda_toolkit_version: String,
    pub cuda_compiler_version: String,
    pub runtime_dependencies: BTreeMap<String, PathBuf>,
    pub expected_text_sha256: String,
    pub artifacts_sha256: BTreeMap<String, String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeManifest {
    pub schema_version: u32,
    pub provider: String,
    pub precision: String,
    pub tf32: bool,
    pub ort_library: PathBuf,
    pub runtime_library_dirs: Vec<PathBuf>,
    pub model_dir: PathBuf,
    pub language: String,
    pub sample_rate_hz: u32,
    pub right_context_frames: u32,
    pub chunk_ms: u32,
    pub parakeet_revision: String,
    pub onnxruntime_revision: String,
    pub verification: VerificationReceipt,
}

#[derive(Debug, Error)]
pub enum ManifestError {
    #[error("cannot read CUDA runtime manifest {path}: {source}")]
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid CUDA runtime manifest: {0}")]
    Json(#[from] serde_json::Error),
    #[error("CUDA runtime manifest is not the pinned ROS 2 profile: {0}")]
    Contract(String),
    #[error("failed to initialize pinned ONNX Runtime: {0}")]
    Ort(#[from] ort::Error),
}

impl RuntimeManifest {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ManifestError> {
        let path = path.as_ref();
        let source = fs::read_to_string(path).map_err(|source| ManifestError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        let manifest: Self = serde_json::from_str(&source)?;
        manifest.validate()?;
        manifest.validate_generation_ownership(path)?;
        Ok(manifest)
    }

    pub fn load_verified(path: impl AsRef<Path>) -> Result<Self, ManifestError> {
        let manifest = Self::load(path)?;
        manifest.validate_verification()?;
        Ok(manifest)
    }

    pub fn validate(&self) -> Result<(), ManifestError> {
        if self.schema_version != 7
            || self.provider != "cuda"
            || self.precision != "fp32"
            || self.tf32
            || self.language != "ja-JP"
            || self.sample_rate_hz != 16_000
            || self.right_context_frames != 3
            || self.chunk_ms != 320
            || self.parakeet_revision != PARAKEET_REVISION
            || self.onnxruntime_revision != ONNXRUNTIME_REVISION
        {
            return Err(ManifestError::Contract("profile values differ".to_owned()));
        }
        if self.verification.method != "nsys_cuda_kernels_and_exact_golden"
            || self.verification.cuda_kernel_rows == 0
            || self.verification.expected_text_sha256 != EXPECTED_GOLDEN_TEXT_SHA256
            || self.verification.nvidia_driver_version.is_empty()
            || self.verification.cuda_toolkit_version != EXPECTED_CUDA_TOOLKIT_VERSION
            || !is_pinned_cuda_compiler(&self.verification.cuda_compiler_version)
            || self.verification.runtime_dependencies.len() != EXPECTED_RUNTIME_DEPENDENCIES.len()
            || self
                .verification
                .runtime_dependencies
                .keys()
                .any(|name| !EXPECTED_RUNTIME_DEPENDENCIES.contains(&name.as_str()))
        {
            return Err(ManifestError::Contract(
                "CUDA golden verification receipt differs".to_owned(),
            ));
        }
        if !self.ort_library.is_file() {
            return Err(ManifestError::Contract(format!(
                "ONNX Runtime library is missing: {}",
                self.ort_library.display()
            )));
        }
        if self.runtime_library_dirs.iter().any(|path| !path.is_dir()) {
            return Err(ManifestError::Contract(
                "CUDA/cuDNN runtime library directory is missing".to_owned(),
            ));
        }
        if self.runtime_library_dirs.len() != 2 {
            return Err(ManifestError::Contract(
                "CUDA runtime receipt must select exactly cuDNN and CUDA library directories"
                    .to_owned(),
            ));
        }
        let cudnn_dir = self
            .verification
            .runtime_dependencies
            .get("libcudnn.so.9")
            .and_then(|path| path.parent())
            .ok_or_else(|| {
                ManifestError::Contract("cuDNN runtime dependency has no parent".to_owned())
            })?;
        let cuda_dir = self
            .verification
            .runtime_dependencies
            .get("libcudart.so.13")
            .and_then(|path| path.parent())
            .ok_or_else(|| {
                ManifestError::Contract("CUDA runtime dependency has no parent".to_owned())
            })?;
        if self.runtime_library_dirs[0] != cudnn_dir || self.runtime_library_dirs[1] != cuda_dir {
            return Err(ManifestError::Contract(
                "CUDA runtime library directories do not exactly bind the dependency receipt"
                    .to_owned(),
            ));
        }
        if self
            .verification
            .runtime_dependencies
            .iter()
            .any(|(name, path)| {
                let expected_dir = if name.starts_with("libcudnn") {
                    cudnn_dir
                } else {
                    cuda_dir
                };
                !path.is_file() || path.parent() != Some(expected_dir)
            })
        {
            return Err(ManifestError::Contract(
                "verified CUDA/cuDNN dependency is missing or in the wrong exact runtime path"
                    .to_owned(),
            ));
        }
        let runtime_dir = self.ort_library.parent().ok_or_else(|| {
            ManifestError::Contract("ONNX Runtime library has no parent directory".to_owned())
        })?;
        if runtime_dir
            .join("libonnxruntime_providers_tensorrt.so")
            .exists()
        {
            return Err(ManifestError::Contract(
                "TensorRT provider must not be present in the CUDA-only runtime".to_owned(),
            ));
        }
        for file in [
            "config.json",
            "encoder.onnx",
            "encoder.onnx.data",
            "decoder_joint.onnx",
            "tokenizer.model",
        ] {
            if !self.model_dir.join(file).is_file() {
                return Err(ManifestError::Contract(format!(
                    "Nemotron model file is missing: {file}"
                )));
            }
        }
        let config: serde_json::Value = serde_json::from_str(
            &fs::read_to_string(self.model_dir.join("config.json")).map_err(|source| {
                ManifestError::Read {
                    path: self.model_dir.join("config.json"),
                    source,
                }
            })?,
        )?;
        if config["att_context_size"] != serde_json::json!([56, 3])
            || config["chunk_size_output_frames"] != 4
            || config["blank_id"] != 13_087
            || config["prompt_dictionary"]["ja-JP"] != 10
        {
            return Err(ManifestError::Contract(
                "Nemotron export must be right-context=3, 320 ms, ja-JP prompt=10".to_owned(),
            ));
        }
        Ok(())
    }

    fn validate_generation_ownership(&self, manifest_path: &Path) -> Result<(), ManifestError> {
        let root = manifest_path
            .parent()
            .ok_or_else(|| ManifestError::Contract("runtime manifest has no parent".to_owned()))?
            .canonicalize()
            .map_err(|source| ManifestError::Read {
                path: manifest_path.to_path_buf(),
                source,
            })?;
        let mut owned = vec![
            &self.ort_library,
            &self.model_dir,
            &self.verification.probe_binary,
            &self.verification.golden_wav,
            &self.verification.nsys_report,
            &self.verification.probe_log,
        ];
        owned.extend(self.runtime_library_dirs.iter());
        owned.extend(self.verification.runtime_dependencies.values());
        for path in owned {
            let canonical = path.canonicalize().map_err(|source| ManifestError::Read {
                path: path.clone(),
                source,
            })?;
            if canonical != *path {
                return Err(ManifestError::Contract(format!(
                    "schema 7 artifact path is not canonical: {}",
                    path.display()
                )));
            }
            if !canonical.starts_with(&root) {
                return Err(ManifestError::Contract(format!(
                    "schema 7 artifact is outside the persistent runtime generation: {}",
                    path.display()
                )));
            }
        }
        Ok(())
    }

    pub fn validate_verification(&self) -> Result<(), ManifestError> {
        let driver_version = current_nvidia_driver_version()?;
        self.validate_verification_for_driver(&driver_version, EXPECTED_GOLDEN_WAV_SHA256)
    }

    fn validate_verification_for_driver(
        &self,
        driver_version: &str,
        expected_golden_wav_sha256: &str,
    ) -> Result<(), ManifestError> {
        if driver_version != self.verification.nvidia_driver_version {
            return Err(ManifestError::Contract(format!(
                "NVIDIA driver differs from verification receipt: {driver_version}"
            )));
        }
        let runtime_dir = self.ort_library.parent().ok_or_else(|| {
            ManifestError::Contract("ONNX Runtime library has no parent directory".to_owned())
        })?;
        let mut expected_paths = BTreeMap::from([
            ("onnxruntime".to_owned(), self.ort_library.clone()),
            (
                "cuda_provider".to_owned(),
                runtime_dir.join("libonnxruntime_providers_cuda.so"),
            ),
            (
                "probe_binary".to_owned(),
                self.verification.probe_binary.clone(),
            ),
            ("config.json".to_owned(), self.model_dir.join("config.json")),
            (
                "encoder.onnx".to_owned(),
                self.model_dir.join("encoder.onnx"),
            ),
            (
                "encoder.onnx.data".to_owned(),
                self.model_dir.join("encoder.onnx.data"),
            ),
            (
                "decoder_joint.onnx".to_owned(),
                self.model_dir.join("decoder_joint.onnx"),
            ),
            (
                "tokenizer.model".to_owned(),
                self.model_dir.join("tokenizer.model"),
            ),
            (
                "golden.wav".to_owned(),
                self.verification.golden_wav.clone(),
            ),
            (
                "nsys_report".to_owned(),
                self.verification.nsys_report.clone(),
            ),
        ]);
        expected_paths.extend(
            self.verification
                .runtime_dependencies
                .iter()
                .map(|(name, path)| (format!("runtime:{name}"), path.clone())),
        );
        expected_paths.insert(
            "provider_shared".to_owned(),
            runtime_dir.join("libonnxruntime_providers_shared.so"),
        );
        expected_paths.insert("probe.log".to_owned(), self.verification.probe_log.clone());
        if self.verification.artifacts_sha256.len() != expected_paths.len() {
            return Err(ManifestError::Contract(
                "CUDA verification receipt artifact set differs".to_owned(),
            ));
        }
        if self
            .verification
            .artifacts_sha256
            .get("golden.wav")
            .map(String::as_str)
            != Some(expected_golden_wav_sha256)
        {
            return Err(ManifestError::Contract(
                "CUDA verification receipt selects a different golden WAV".to_owned(),
            ));
        }
        for (name, path) in expected_paths {
            let actual = sha256_file(&path)?;
            if self.verification.artifacts_sha256.get(&name) != Some(&actual) {
                return Err(ManifestError::Contract(format!(
                    "CUDA verification receipt digest mismatch: {name}"
                )));
            }
        }
        let probe_log = &self.verification.probe_log;
        let contents = fs::read_to_string(probe_log).map_err(|source| ManifestError::Read {
            path: probe_log.clone(),
            source,
        })?;
        validate_probe_log(&contents)?;
        Ok(())
    }

    pub fn initialize_ort(&self) -> Result<(), ManifestError> {
        let pinned = self
            .ort_library
            .canonicalize()
            .map_err(|source| ManifestError::Read {
                path: self.ort_library.clone(),
                source,
            })?;
        if let Some(configured) = std::env::var_os("ORT_DYLIB_PATH") {
            let configured_path = PathBuf::from(configured);
            let configured =
                configured_path
                    .canonicalize()
                    .map_err(|source| ManifestError::Read {
                        path: configured_path,
                        source,
                    })?;
            if configured != pinned {
                return Err(ManifestError::Contract(format!(
                    "ORT_DYLIB_PATH does not select the pinned runtime: {}",
                    configured.display()
                )));
            }
        } else {
            // This runs before ROS or inference threads are created. `ort` rc12's
            // regular dynamic loader reads this variable on its first API call.
            unsafe { std::env::set_var("ORT_DYLIB_PATH", &pinned) };
        }
        // Do not commit an `EnvironmentBuilder` before the first dynamic API
        // lookup. In ort rc12 that re-enters its logger while the API OnceLock
        // is held on aarch64. The default environment is created when the
        // first session is built, after this pinned-library check succeeds.
        if !ort::info().contains("8c546c3") {
            return Err(ManifestError::Contract(format!(
                "loaded ONNX Runtime build is not pinned: {}",
                ort::info()
            )));
        }
        Ok(())
    }
}

fn validate_probe_log(contents: &str) -> Result<(), ManifestError> {
    let transcript = format!("PARAKEET_RS_TEXT={EXPECTED_GOLDEN_TEXT}");
    let required = [
        "CUDA_EP_COMPILED=true",
        "EXECUTION_PROVIDER=cuda",
        "TF32_ENABLED=false",
        transcript.as_str(),
        "PARITY_OK=true",
        "WARM_PARITY_OK=true",
    ];
    for expected in required {
        if contents.lines().filter(|line| *line == expected).count() != 1 {
            return Err(ManifestError::Contract(format!(
                "CUDA probe receipt must contain exactly one {expected:?} line"
            )));
        }
    }
    Ok(())
}

fn current_nvidia_driver_version() -> Result<String, ManifestError> {
    let output = Command::new("nvidia-smi")
        .args(["--query-gpu=driver_version", "--format=csv,noheader"])
        .output()
        .map_err(|error| ManifestError::Contract(format!("cannot execute nvidia-smi: {error}")))?;
    if !output.status.success() {
        return Err(ManifestError::Contract(format!(
            "nvidia-smi driver query failed with {}",
            output.status
        )));
    }
    let version = String::from_utf8_lossy(&output.stdout)
        .lines()
        .next()
        .unwrap_or_default()
        .trim()
        .to_owned();
    if version.is_empty() {
        return Err(ManifestError::Contract(
            "nvidia-smi returned no driver version".to_owned(),
        ));
    }
    Ok(version)
}

fn is_pinned_cuda_compiler(version: &str) -> bool {
    let mut components = version.split('.');
    matches!(
        (
            components.next(),
            components.next(),
            components.next(),
            components.next(),
        ),
        (Some("13"), Some("2"), Some(patch), None)
            if !patch.is_empty() && patch.bytes().all(|value| value.is_ascii_digit())
    )
}

fn sha256_file(path: &Path) -> Result<String, ManifestError> {
    let mut source = File::open(path).map_err(|source| ManifestError::Read {
        path: path.to_path_buf(),
        source,
    })?;
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 1024 * 1024];
    loop {
        let read = source
            .read(&mut buffer)
            .map_err(|source| ManifestError::Read {
                path: path.to_path_buf(),
                source,
            })?;
        if read == 0 {
            break;
        }
        digest.update(&buffer[..read]);
    }
    Ok(format!("{:x}", digest.finalize()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    struct ManifestFixture {
        root: PathBuf,
        manifest_path: PathBuf,
        manifest: RuntimeManifest,
        golden_sha256: String,
    }

    impl ManifestFixture {
        fn new() -> Self {
            let nonce = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("system clock")
                .as_nanos();
            let root = std::env::temp_dir().join(format!(
                "fv-speech-manifest-fixture-{}-{nonce}",
                std::process::id()
            ));
            let generation = root.join("generations/test");
            let runtime = generation.join("lib");
            let cudnn = generation.join("deps/cudnn");
            let cuda = generation.join("deps/cuda");
            let model = generation.join("model");
            let golden_dir = generation.join("golden");
            let verification_dir = generation.join("verification");
            for directory in [
                &runtime,
                &cudnn,
                &cuda,
                &model,
                &golden_dir,
                &verification_dir,
            ] {
                fs::create_dir_all(directory).expect("create manifest fixture directory");
            }
            let root = root.canonicalize().expect("canonical fixture root");
            let generation = root.join("generations/test");
            let runtime = generation.join("lib");
            let cudnn = generation.join("deps/cudnn");
            let cuda = generation.join("deps/cuda");
            let model = generation.join("model");
            let golden_wav = generation.join("golden/ja.wav");
            let verification_dir = generation.join("verification");

            let ort_library = runtime.join("libonnxruntime.so.1.26.0");
            fs::write(&ort_library, b"ort").expect("write ORT fixture");
            fs::write(runtime.join("libonnxruntime_providers_cuda.so"), b"cuda")
                .expect("write CUDA provider fixture");
            fs::write(
                runtime.join("libonnxruntime_providers_shared.so"),
                b"shared",
            )
            .expect("write shared provider fixture");
            fs::write(
                model.join("config.json"),
                br#"{"att_context_size":[56,3],"chunk_size_output_frames":4,"blank_id":13087,"prompt_dictionary":{"ja-JP":10}}"#,
            )
            .expect("write config fixture");
            for name in [
                "encoder.onnx",
                "encoder.onnx.data",
                "decoder_joint.onnx",
                "tokenizer.model",
            ] {
                fs::write(model.join(name), name.as_bytes()).expect("write model fixture");
            }
            fs::write(&golden_wav, b"golden").expect("write golden fixture");
            let probe_binary = verification_dir.join("aspa_cuda_probe");
            let nsys_report = verification_dir.join("parakeet-cuda.nsys-rep");
            let probe_log = verification_dir.join("probe.log");
            fs::write(&probe_binary, b"probe").expect("write probe fixture");
            fs::write(&nsys_report, b"nsys").expect("write nsys fixture");
            fs::write(
                &probe_log,
                format!(
                    "CUDA_EP_COMPILED=true\nEXECUTION_PROVIDER=cuda\nTF32_ENABLED=false\nPARAKEET_RS_TEXT={EXPECTED_GOLDEN_TEXT}\nPARITY_OK=true\nWARM_PARITY_OK=true\n"
                ),
            )
            .expect("write probe log fixture");

            let mut runtime_dependencies = BTreeMap::new();
            for name in EXPECTED_RUNTIME_DEPENDENCIES {
                let directory = if name.starts_with("libcudnn") {
                    &cudnn
                } else {
                    &cuda
                };
                let path = directory.join(name);
                fs::write(&path, name.as_bytes()).expect("write runtime dependency fixture");
                runtime_dependencies.insert(name.to_owned(), path);
            }
            let runtime_library_dirs = vec![cudnn, cuda];
            let runtime_dir = ort_library.parent().expect("ORT parent");
            let mut artifact_paths = BTreeMap::from([
                ("onnxruntime".to_owned(), ort_library.clone()),
                (
                    "cuda_provider".to_owned(),
                    runtime_dir.join("libonnxruntime_providers_cuda.so"),
                ),
                (
                    "provider_shared".to_owned(),
                    runtime_dir.join("libonnxruntime_providers_shared.so"),
                ),
                ("probe_binary".to_owned(), probe_binary.clone()),
                ("config.json".to_owned(), model.join("config.json")),
                ("encoder.onnx".to_owned(), model.join("encoder.onnx")),
                (
                    "encoder.onnx.data".to_owned(),
                    model.join("encoder.onnx.data"),
                ),
                (
                    "decoder_joint.onnx".to_owned(),
                    model.join("decoder_joint.onnx"),
                ),
                ("tokenizer.model".to_owned(), model.join("tokenizer.model")),
                ("golden.wav".to_owned(), golden_wav.clone()),
                ("nsys_report".to_owned(), nsys_report.clone()),
                ("probe.log".to_owned(), probe_log.clone()),
            ]);
            artifact_paths.extend(
                runtime_dependencies
                    .iter()
                    .map(|(name, path)| (format!("runtime:{name}"), path.clone())),
            );
            let artifacts_sha256 = artifact_paths
                .iter()
                .map(|(name, path)| {
                    (
                        name.clone(),
                        sha256_file(path).expect("hash manifest fixture artifact"),
                    )
                })
                .collect();
            let golden_sha256 = sha256_file(&golden_wav).expect("hash golden fixture");
            let manifest = RuntimeManifest {
                schema_version: 7,
                provider: "cuda".to_owned(),
                precision: "fp32".to_owned(),
                tf32: false,
                ort_library,
                runtime_library_dirs,
                model_dir: model,
                language: "ja-JP".to_owned(),
                sample_rate_hz: 16_000,
                right_context_frames: 3,
                chunk_ms: 320,
                parakeet_revision: PARAKEET_REVISION.to_owned(),
                onnxruntime_revision: ONNXRUNTIME_REVISION.to_owned(),
                verification: VerificationReceipt {
                    method: "nsys_cuda_kernels_and_exact_golden".to_owned(),
                    cuda_kernel_rows: 1,
                    probe_binary,
                    golden_wav,
                    nsys_report,
                    probe_log,
                    nvidia_driver_version: "test-driver".to_owned(),
                    cuda_toolkit_version: EXPECTED_CUDA_TOOLKIT_VERSION.to_owned(),
                    cuda_compiler_version: "13.2.78".to_owned(),
                    runtime_dependencies,
                    expected_text_sha256: EXPECTED_GOLDEN_TEXT_SHA256.to_owned(),
                    artifacts_sha256,
                },
            };
            let manifest_path = root.join("runtime.json");
            fs::write(&manifest_path, b"{}").expect("write manifest path fixture");
            Self {
                root,
                manifest_path,
                manifest,
                golden_sha256,
            }
        }

        fn validate_all(&self) -> Result<(), ManifestError> {
            self.manifest.validate()?;
            self.manifest
                .validate_generation_ownership(&self.manifest_path)?;
            self.manifest
                .validate_verification_for_driver("test-driver", &self.golden_sha256)
        }
    }

    impl Drop for ManifestFixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.root);
        }
    }

    #[test]
    fn sha256_file_is_exact() {
        let path = std::env::temp_dir().join(format!(
            "fv-speech-manifest-sha-{}-{}",
            std::process::id(),
            std::thread::current().name().unwrap_or("test")
        ));
        fs::write(&path, b"abc").expect("write digest fixture");
        assert_eq!(
            sha256_file(&path).expect("hash fixture"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        fs::remove_file(path).expect("remove digest fixture");
    }

    #[test]
    fn schema_7_probe_log_proves_cuda_tf32_off_and_exact_parity() {
        let log = format!(
            "CUDA_EP_COMPILED=true\nEXECUTION_PROVIDER=cuda\nTF32_ENABLED=false\nPARAKEET_RS_TEXT={EXPECTED_GOLDEN_TEXT}\nPARITY_OK=true\nWARM_PARITY_OK=true\n"
        );
        validate_probe_log(&log).expect("accept exact verification receipt");
        assert!(
            validate_probe_log(&log.replace("TF32_ENABLED=false", "TF32_ENABLED=true")).is_err()
        );
    }

    #[test]
    fn schema_7_fixture_accepts_only_the_pinned_profile() {
        let fixture = ManifestFixture::new();
        fixture.validate_all().expect("accept pinned fixture");

        let mut invalid = vec![];
        let mut schema = fixture.manifest.clone();
        schema.schema_version = 5;
        invalid.push(schema);
        let mut provider = fixture.manifest.clone();
        provider.provider = "tensorrt".to_owned();
        invalid.push(provider);
        let mut precision = fixture.manifest.clone();
        precision.precision = "fp16".to_owned();
        invalid.push(precision);
        let mut tf32 = fixture.manifest.clone();
        tf32.tf32 = true;
        invalid.push(tf32);
        let mut cuda_toolkit = fixture.manifest.clone();
        cuda_toolkit.verification.cuda_toolkit_version = "13.1".to_owned();
        invalid.push(cuda_toolkit);
        let mut cuda_compiler = fixture.manifest.clone();
        cuda_compiler.verification.cuda_compiler_version = "13.2.invalid".to_owned();
        invalid.push(cuda_compiler);
        for manifest in invalid {
            assert!(manifest.validate().is_err());
        }
    }

    #[test]
    fn schema_7_fixture_rejects_runtime_substitution_and_tensorrt() {
        let fixture = ManifestFixture::new();
        let mut substituted = fixture.manifest.clone();
        substituted.runtime_library_dirs.swap(0, 1);
        assert!(substituted.validate().is_err());

        let tensorrt = fixture
            .manifest
            .ort_library
            .parent()
            .expect("ORT parent")
            .join("libonnxruntime_providers_tensorrt.so");
        fs::write(&tensorrt, b"trt").expect("write forbidden TensorRT fixture");
        assert!(fixture.manifest.validate().is_err());
    }

    #[test]
    fn schema_7_fixture_rejects_driver_artifact_probe_and_ownership_tampering() {
        let fixture = ManifestFixture::new();
        assert!(
            fixture
                .manifest
                .validate_verification_for_driver("other-driver", &fixture.golden_sha256)
                .is_err()
        );

        fs::write(fixture.manifest.model_dir.join("encoder.onnx"), b"tampered")
            .expect("tamper model fixture");
        assert!(
            fixture
                .manifest
                .validate_verification_for_driver("test-driver", &fixture.golden_sha256)
                .is_err()
        );

        let probe_fixture = ManifestFixture::new();
        fs::write(&probe_fixture.manifest.verification.probe_log, b"not CUDA")
            .expect("tamper probe fixture");
        let mut probe_manifest = probe_fixture.manifest.clone();
        probe_manifest.verification.artifacts_sha256.insert(
            "probe.log".to_owned(),
            sha256_file(&probe_manifest.verification.probe_log).expect("hash tampered probe"),
        );
        assert!(
            probe_manifest
                .validate_verification_for_driver("test-driver", &probe_fixture.golden_sha256,)
                .is_err()
        );

        let ownership_fixture = ManifestFixture::new();
        let outside = ownership_fixture.root.with_extension("outside-probe");
        fs::write(&outside, b"outside").expect("write outside fixture");
        let mut outside_manifest = ownership_fixture.manifest.clone();
        outside_manifest.verification.probe_binary = outside.clone();
        assert!(
            outside_manifest
                .validate_generation_ownership(&ownership_fixture.manifest_path)
                .is_err()
        );
        fs::remove_file(outside).expect("remove outside fixture");
    }
}
