use std::time::{SystemTime, UNIX_EPOCH};

use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::{QosProfile, builtin_interfaces, std_msgs};

pub fn speech_qos(depth: usize) -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth,
        reliability: ReliabilityPolicy::Reliable,
        ..QosProfile::default()
    }
}

pub fn ready_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    }
}

pub fn header(frame_id: &str) -> std_msgs::msg::Header {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: now.as_secs() as i32,
            nanosec: now.subsec_nanos(),
        },
        frame_id: frame_id.to_owned(),
    }
}
