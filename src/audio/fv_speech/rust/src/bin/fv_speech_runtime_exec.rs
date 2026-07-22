use std::os::unix::process::CommandExt;
use std::path::PathBuf;
use std::process::Command;

use fv_speech_ros2::manifest::{RuntimeManifest, default_runtime_manifest_path};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args_os();
    let _program = arguments.next();
    let target = arguments
        .next()
        .ok_or("usage: fv_speech_runtime_exec <parakeet_asr|silero_vad> [ROS arguments]")?;
    let target = target
        .to_str()
        .ok_or("fv_speech runtime target is not valid UTF-8")?;
    if !matches!(target, "parakeet_asr" | "silero_vad") {
        return Err(format!("unsupported fv_speech runtime target: {target}").into());
    }

    let manifest_path = std::env::var_os("ASPA_PARAKEET_RUNTIME_MANIFEST")
        .map(PathBuf::from)
        .unwrap_or_else(default_runtime_manifest_path);
    let manifest = RuntimeManifest::load_verified(&manifest_path)?;

    let current_exe = std::env::current_exe()?;
    let target_exe = current_exe
        .parent()
        .ok_or("fv_speech runtime wrapper has no executable directory")?
        .join(target);
    if !target_exe.is_file() {
        return Err(format!("fv_speech target is missing: {}", target_exe.display()).into());
    }

    let mut library_paths = manifest.runtime_library_dirs.clone();
    if let Some(inherited) = std::env::var_os("LD_LIBRARY_PATH") {
        library_paths.extend(std::env::split_paths(&inherited));
    }
    let library_path = std::env::join_paths(library_paths)?;

    let error = Command::new(&target_exe)
        .args(arguments)
        .env("ASPA_PARAKEET_RUNTIME_MANIFEST", &manifest_path)
        .env("ORT_DYLIB_PATH", &manifest.ort_library)
        .env("LD_LIBRARY_PATH", library_path)
        .exec();
    Err(error.into())
}
