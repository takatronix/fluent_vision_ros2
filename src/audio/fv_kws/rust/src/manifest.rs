use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{VOSK_GRAMMAR, WAKE_KEYWORD};

const SCHEMA: &str = "aspa.vosk-kws-runtime.v1";
const VOSK_VERSION: &str = "0.3.45";

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerifiedFile {
    pub path: PathBuf,
    pub sha256: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeManifest {
    pub schema: String,
    pub vosk_version: String,
    pub keyword: String,
    pub grammar: [String; 2],
    pub library: VerifiedFile,
    pub model_path: PathBuf,
    pub model_files: Vec<VerifiedFile>,
}

#[derive(Debug, Error)]
pub enum ManifestError {
    #[error("failed to read KWS runtime manifest {path}: {source}")]
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid KWS runtime manifest JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("KWS runtime manifest schema must be {SCHEMA}")]
    Schema,
    #[error("KWS runtime requires Vosk {VOSK_VERSION}")]
    Version,
    #[error("KWS runtime keyword or grammar does not match the compiled contract")]
    KeywordContract,
    #[error("KWS runtime path must be absolute: {0}")]
    RelativePath(PathBuf),
    #[error("KWS runtime library must be named libvosk.so")]
    LibraryName,
    #[error("KWS model receipt must list at least one file")]
    EmptyModel,
    #[error("KWS model receipt contains a duplicate path: {0}")]
    DuplicateModelFile(PathBuf),
    #[error("KWS model artifact is outside model_path: {0}")]
    ModelFileOutsideRoot(PathBuf),
    #[error("failed to read KWS artifact {path}: {source}")]
    ArtifactRead {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("KWS artifact SHA-256 mismatch: {0}")]
    Digest(PathBuf),
}

impl RuntimeManifest {
    pub fn load_verified(path: impl AsRef<Path>) -> Result<Self, ManifestError> {
        let path = path.as_ref();
        let data = fs::read(path).map_err(|source| ManifestError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        let manifest: Self = serde_json::from_slice(&data)?;
        if manifest.schema != SCHEMA {
            return Err(ManifestError::Schema);
        }
        if manifest.vosk_version != VOSK_VERSION {
            return Err(ManifestError::Version);
        }
        if manifest.keyword != WAKE_KEYWORD
            || manifest.grammar != VOSK_GRAMMAR.map(ToOwned::to_owned)
        {
            return Err(ManifestError::KeywordContract);
        }
        require_absolute(&manifest.library.path)?;
        require_absolute(&manifest.model_path)?;
        if manifest
            .library
            .path
            .file_name()
            .and_then(|name| name.to_str())
            != Some("libvosk.so")
        {
            return Err(ManifestError::LibraryName);
        }
        verify(&manifest.library)?;
        if manifest.model_files.is_empty() {
            return Err(ManifestError::EmptyModel);
        }
        let mut paths = HashSet::new();
        for file in &manifest.model_files {
            require_absolute(&file.path)?;
            if !file.path.starts_with(&manifest.model_path) {
                return Err(ManifestError::ModelFileOutsideRoot(file.path.clone()));
            }
            if !paths.insert(file.path.clone()) {
                return Err(ManifestError::DuplicateModelFile(file.path.clone()));
            }
            verify(file)?;
        }
        Ok(manifest)
    }
}

fn require_absolute(path: &Path) -> Result<(), ManifestError> {
    if !path.is_absolute() {
        return Err(ManifestError::RelativePath(path.to_path_buf()));
    }
    Ok(())
}

fn verify(file: &VerifiedFile) -> Result<(), ManifestError> {
    let data = fs::read(&file.path).map_err(|source| ManifestError::ArtifactRead {
        path: file.path.clone(),
        source,
    })?;
    let digest = format!("{:x}", Sha256::digest(data));
    if digest != file.sha256 {
        return Err(ManifestError::Digest(file.path.clone()));
    }
    Ok(())
}
