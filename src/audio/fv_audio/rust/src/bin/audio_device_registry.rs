use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cpal::traits::{DeviceTrait, HostTrait};
use cpal::{Device, Host};
use futures::{FutureExt, StreamExt};
use r2r::fv_audio_interfaces::msg::AudioDevice;
use r2r::fv_audio_interfaces::srv::ListAudioDevices;
use r2r::{Context, Node, QosProfile};
use thiserror::Error;

const INPUT: u8 = AudioDevice::INPUT as u8;
const OUTPUT: u8 = AudioDevice::OUTPUT as u8;
const SERVICE_NAME: &str = "/audio/devices/list";

#[derive(Debug, Error)]
enum RegistryError {
    #[error("failed to enumerate CPAL devices: {0}")]
    Enumerate(#[source] cpal::Error),
    #[error("CPAL device metadata is unavailable: {0}")]
    Metadata(#[source] cpal::Error),
    #[error("direction must be ListAudioDevices.INPUT or ListAudioDevices.OUTPUT")]
    InvalidDirection,
    #[error("ROS 2 error: {0}")]
    Ros(#[from] r2r::Error),
    #[error("signal handler error: {0}")]
    Signal(#[from] ctrlc::Error),
}

fn main() -> Result<(), RegistryError> {
    let host = cpal::default_host();
    let context = Context::create()?;
    let mut node = Node::create(context, "audio_device_registry", "")?;
    let mut requests =
        node.create_service::<ListAudioDevices::Service>(SERVICE_NAME, QosProfile::default())?;

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    while running.load(Ordering::SeqCst) {
        node.spin_once(Duration::from_millis(20));
        while let Some(Some(request)) = requests.next().now_or_never() {
            let response = match enumerate_devices(&host, request.message.direction) {
                Ok(devices) => ListAudioDevices::Response {
                    success: true,
                    message: format!(
                        "found {} {} device(s) on {}",
                        devices.len(),
                        direction_name(request.message.direction),
                        host.id()
                    ),
                    devices,
                },
                Err(error) => ListAudioDevices::Response {
                    success: false,
                    message: error.to_string(),
                    devices: Vec::new(),
                },
            };
            request.respond(response)?;
        }
    }
    Ok(())
}

fn enumerate_devices(host: &Host, direction: u8) -> Result<Vec<AudioDevice>, RegistryError> {
    let (devices, default_device) = match direction {
        INPUT => (
            host.input_devices()
                .map_err(RegistryError::Enumerate)?
                .collect::<Vec<_>>(),
            host.default_input_device(),
        ),
        OUTPUT => (
            host.output_devices()
                .map_err(RegistryError::Enumerate)?
                .collect::<Vec<_>>(),
            host.default_output_device(),
        ),
        _ => {
            return Err(RegistryError::InvalidDirection);
        }
    };

    let default_id = default_device
        .map(|device| device.id().map(|id| id.to_string()))
        .transpose()
        .map_err(RegistryError::Metadata)?;

    devices
        .into_iter()
        .map(|device| audio_device(host, device, direction, default_id.as_deref()))
        .collect()
}

fn audio_device(
    host: &Host,
    device: Device,
    direction: u8,
    default_id: Option<&str>,
) -> Result<AudioDevice, RegistryError> {
    let id = device.id().map_err(RegistryError::Metadata)?.to_string();
    let description = device.description().map_err(RegistryError::Metadata)?;
    Ok(AudioDevice {
        direction,
        id: id.clone(),
        name: description.name().to_owned(),
        host: host.id().to_string(),
        is_default: default_id == Some(id.as_str()),
        manufacturer: description.manufacturer().unwrap_or_default().to_owned(),
        driver: description.driver().unwrap_or_default().to_owned(),
        device_type: description.device_type().to_string(),
        interface_type: description.interface_type().to_string(),
        address: description.address().unwrap_or_default().to_owned(),
        details: description.extended().map(str::to_owned).collect(),
    })
}

fn direction_name(direction: u8) -> &'static str {
    match direction {
        INPUT => "input",
        OUTPUT => "output",
        _ => "invalid",
    }
}

#[cfg(test)]
mod tests {
    use super::{direction_name, INPUT, OUTPUT};

    #[test]
    fn direction_names_are_exact() {
        assert_eq!(direction_name(INPUT), "input");
        assert_eq!(direction_name(OUTPUT), "output");
        assert_eq!(direction_name(0), "invalid");
    }
}
