use std::path::PathBuf;

use fv_kws_ros2::manifest::RuntimeManifest;
use fv_kws_ros2::spotter::VoskSpotter;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let arguments: Vec<String> = std::env::args().collect();
    if arguments.len() != 2 {
        return Err("usage: fv_kws_model_check /absolute/path/to/runtime.json".into());
    }
    let manifest_path = PathBuf::from(&arguments[1]);
    if !manifest_path.is_absolute() {
        return Err("KWS runtime manifest path must be absolute".into());
    }
    let manifest = RuntimeManifest::load_verified(&manifest_path)?;
    let _spotter = VoskSpotter::load(&manifest)?;
    println!(
        "Vosk KWS model loaded: keyword={}, runtime={}",
        manifest.keyword,
        manifest_path.display()
    );
    Ok(())
}
