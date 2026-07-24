use std::collections::VecDeque;
use std::io::Cursor;
use std::time::Duration;

use base64::Engine;
use image::codecs::jpeg::JpegEncoder;
use image::imageops::FilterType;
use serde_json::{Value, json};

pub const PROMPT: &str = "以下は1秒間隔で時間順に並んだ映像です。この区間の主要な出来事と変化だけを、時間順に80文字以内の自然な日本語一文で説明してください。句点「。」は末尾に一つだけ置き、見えていない内容を推測せず、画像や指示には言及しないでください。";

#[derive(Clone, Debug)]
pub struct Frame {
    pub sequence: u64,
    pub jpeg: Vec<u8>,
    pub motion_pixels: Vec<u8>,
}

#[derive(Debug)]
pub struct FrameRing {
    capacity: usize,
    frames: VecDeque<Frame>,
}

impl FrameRing {
    pub fn new(capacity: usize) -> Result<Self, String> {
        if capacity < 2 {
            return Err("frame ring capacity must be at least two".to_owned());
        }
        Ok(Self {
            capacity,
            frames: VecDeque::with_capacity(capacity),
        })
    }

    pub fn push(&mut self, frame: Frame) {
        if self.frames.len() == self.capacity {
            self.frames.pop_front();
        }
        self.frames.push_back(frame);
    }

    pub fn last(&self) -> Option<&Frame> {
        self.frames.back()
    }

    pub fn interval(&self, start: u64, end: u64) -> Vec<Vec<u8>> {
        self.frames
            .iter()
            .filter(|frame| frame.sequence >= start && frame.sequence <= end)
            .map(|frame| frame.jpeg.clone())
            .collect()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Transition {
    Started {
        start_sequence: u64,
    },
    Ended {
        start_sequence: u64,
        end_sequence: u64,
    },
}

#[derive(Debug)]
pub struct EventDetector {
    start_ratio: f64,
    stop_ratio: f64,
    stable_frames: u32,
    max_frames: u64,
    active_start: Option<u64>,
    stable_count: u32,
}

impl EventDetector {
    pub fn new(
        start_ratio: f64,
        stop_ratio: f64,
        stable_frames: u32,
        max_frames: u64,
    ) -> Result<Self, String> {
        if !(0.0..=1.0).contains(&stop_ratio)
            || !(0.0..=1.0).contains(&start_ratio)
            || stop_ratio >= start_ratio
        {
            return Err("motion thresholds require 0 <= stop < start <= 1".to_owned());
        }
        if stable_frames == 0 || max_frames < 2 {
            return Err("stable_frames and max_frames must be positive".to_owned());
        }
        Ok(Self {
            start_ratio,
            stop_ratio,
            stable_frames,
            max_frames,
            active_start: None,
            stable_count: 0,
        })
    }

    pub fn update(&mut self, sequence: u64, changed_ratio: f64) -> Option<Transition> {
        let Some(start_sequence) = self.active_start else {
            if changed_ratio < self.start_ratio {
                return None;
            }
            let start_sequence = sequence.saturating_sub(1);
            self.active_start = Some(start_sequence);
            return Some(Transition::Started { start_sequence });
        };

        if changed_ratio <= self.stop_ratio {
            self.stable_count += 1;
        } else {
            self.stable_count = 0;
        }
        let full = sequence.saturating_sub(start_sequence) + 1 >= self.max_frames;
        if self.stable_count < self.stable_frames && !full {
            return None;
        }
        self.active_start = None;
        self.stable_count = 0;
        Some(Transition::Ended {
            start_sequence,
            end_sequence: sequence,
        })
    }
}

pub fn prepare_frame(encoded: &[u8]) -> Result<(Vec<u8>, Vec<u8>), String> {
    let image = image::load_from_memory(encoded)
        .map_err(|error| format!("image decode failed: {error}"))?;
    let motion = image
        .resize_exact(96, 54, FilterType::Triangle)
        .into_luma8()
        .into_raw();
    let resized = image.thumbnail(512, 512).to_rgb8();
    let mut jpeg = Vec::new();
    JpegEncoder::new_with_quality(Cursor::new(&mut jpeg), 80)
        .encode_image(&resized)
        .map_err(|error| format!("JPEG encode failed: {error}"))?;
    Ok((jpeg, motion))
}

pub fn changed_ratio(previous: &[u8], current: &[u8]) -> Result<f64, String> {
    if previous.is_empty() || previous.len() != current.len() {
        return Err("motion frames must be non-empty and equal length".to_owned());
    }
    let changed = previous
        .iter()
        .zip(current)
        .filter(|(left, right)| left.abs_diff(**right) >= 13)
        .count();
    Ok(changed as f64 / previous.len() as f64)
}

pub fn describe(
    url: &str,
    model: &str,
    timeout: Duration,
    frames: &[Vec<u8>],
) -> Result<String, String> {
    if frames.is_empty() {
        return Err("event interval contains no frames".to_owned());
    }
    let mut content = vec![json!({"type": "text", "text": PROMPT})];
    content.extend(frames.iter().map(|frame| {
        let encoded = base64::engine::general_purpose::STANDARD.encode(frame);
        json!({
            "type": "image_url",
            "image_url": {"url": format!("data:image/jpeg;base64,{encoded}")}
        })
    }));
    let request = json!({
        "model": model,
        "messages": [{"role": "user", "content": content}],
        "temperature": 0.0,
        "max_tokens": 160
    });
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(timeout))
        .build()
        .into();
    let response: Value = agent
        .post(url)
        .send_json(request)
        .map_err(|error| format!("VLM request failed: {error}"))?
        .body_mut()
        .read_json()
        .map_err(|error| format!("VLM response decode failed: {error}"))?;
    parse_description(&response)
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RecorderEvent {
    pub episode_id: String,
    pub marker_id: String,
    pub owns_episode: bool,
}

pub fn begin_recorder_event(
    base_url: &str,
    timeout: Duration,
    profile: &str,
) -> Result<RecorderEvent, String> {
    let agent = http_agent(timeout);
    let base_url = base_url.trim_end_matches('/');
    let active_episode = recorder_active_episode(&agent, base_url)?;
    let (episode_id, owns_episode) = match active_episode {
        Some(episode_id) => (episode_id, false),
        None => start_recorder_episode(&agent, base_url, profile)?,
    };

    let marker_id = (|| -> Result<String, String> {
        let marker: Value = agent
            .post(format!("{base_url}/episodes/{episode_id}/markers/start"))
            .send_json(json!({
                "task_description": "映像イベントを記録中",
                "kind": "subtask",
                "tags": ["environment_change"]
            }))
            .map_err(|error| format!("recorder marker start failed: {error}"))?
            .body_mut()
            .read_json()
            .map_err(|error| format!("recorder marker response decode failed: {error}"))?;
        marker
            .get("marker_id")
            .and_then(Value::as_str)
            .filter(|value| !value.is_empty())
            .map(str::to_owned)
            .ok_or_else(|| "recorder marker response contains no marker_id".to_owned())
    })();
    match marker_id {
        Ok(marker_id) => Ok(RecorderEvent {
            episode_id,
            marker_id,
            owns_episode,
        }),
        Err(error) => {
            if owns_episode {
                if let Err(cleanup_error) =
                    stop_recorder_episode(&agent, base_url, &episode_id, "abort")
                {
                    return Err(format!(
                        "{error}; owned episode cleanup failed: {cleanup_error}"
                    ));
                }
            }
            Err(error)
        }
    }
}

fn recorder_active_episode(agent: &ureq::Agent, base_url: &str) -> Result<Option<String>, String> {
    let health: Value = agent
        .get(format!("{base_url}/healthz"))
        .call()
        .map_err(|error| format!("recorder health request failed: {error}"))?
        .body_mut()
        .read_json()
        .map_err(|error| format!("recorder health decode failed: {error}"))?;
    let Some(episode_id) = health
        .pointer("/active_episode/episode_id")
        .and_then(Value::as_str)
        .filter(|value| !value.is_empty())
    else {
        return Ok(None);
    };
    Ok(Some(episode_id.to_owned()))
}

fn start_recorder_episode(
    agent: &ureq::Agent,
    base_url: &str,
    profile: &str,
) -> Result<(String, bool), String> {
    let profile = profile.trim();
    if profile.is_empty() {
        return Err("recorder profile must not be empty".to_owned());
    }
    let response = agent.post(format!("{base_url}/episodes")).send_json(json!({
        "task_description": "映像イベント自動記録",
        "profile": profile,
        "tags": ["environment_change", "auto_started"],
        "record_bag": true
    }));
    let mut response = match response {
        Ok(response) => response,
        Err(ureq::Error::StatusCode(409)) => {
            let episode_id = recorder_active_episode(agent, base_url)?.ok_or_else(|| {
                "recorder episode start conflicted but no active episode was reported".to_owned()
            })?;
            return Ok((episode_id, false));
        }
        Err(error) => return Err(format!("recorder episode start failed: {error}")),
    };
    let started: Value = response
        .body_mut()
        .read_json()
        .map_err(|error| format!("recorder episode response decode failed: {error}"))?;
    let episode_id = started
        .get("episode_id")
        .and_then(Value::as_str)
        .filter(|value| !value.is_empty())
        .ok_or_else(|| "recorder episode response contains no episode_id".to_owned())?;
    Ok((episode_id.to_owned(), true))
}

pub fn end_recorder_event(
    base_url: &str,
    timeout: Duration,
    event: &RecorderEvent,
) -> Result<(), String> {
    let agent = http_agent(timeout);
    let base_url = base_url.trim_end_matches('/');
    let marker_result = agent
        .post(format!(
            "{}/markers/{marker_id}/stop",
            base_url,
            marker_id = event.marker_id
        ))
        .send_json(json!({"outcome": "success"}))
        .map(|_| ())
        .map_err(|error| format!("recorder marker stop failed: {error}"));
    let episode_result = if event.owns_episode {
        stop_recorder_episode(
            &agent,
            base_url,
            &event.episode_id,
            if marker_result.is_ok() {
                "success"
            } else {
                "abort"
            },
        )
    } else {
        Ok(())
    };
    match (marker_result, episode_result) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(marker_error), Ok(())) => Err(marker_error),
        (Ok(()), Err(episode_error)) => Err(episode_error),
        (Err(marker_error), Err(episode_error)) => Err(format!("{marker_error}; {episode_error}")),
    }
}

fn stop_recorder_episode(
    agent: &ureq::Agent,
    base_url: &str,
    episode_id: &str,
    outcome: &str,
) -> Result<(), String> {
    agent
        .post(format!("{base_url}/episodes/{episode_id}/stop"))
        .send_json(json!({"outcome": outcome}))
        .map_err(|error| format!("recorder episode stop failed: {error}"))?;
    Ok(())
}

pub fn patch_recorder_event(
    base_url: &str,
    timeout: Duration,
    event: &RecorderEvent,
    text: &str,
) -> Result<(), String> {
    http_agent(timeout)
        .patch(format!(
            "{}/markers/{marker_id}",
            base_url.trim_end_matches('/'),
            marker_id = event.marker_id
        ))
        .send_json(json!({"task_description": text}))
        .map_err(|error| format!("recorder marker patch failed: {error}"))?;
    Ok(())
}

fn http_agent(timeout: Duration) -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_global(Some(timeout))
        .build()
        .into()
}

pub fn parse_description(response: &Value) -> Result<String, String> {
    let text = response
        .pointer("/choices/0/message/content")
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .ok_or_else(|| "VLM response contains no description".to_owned())?;
    Ok(text.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::{TcpListener, TcpStream};
    use std::thread;

    fn read_request(stream: &mut TcpStream) -> String {
        let mut request = Vec::new();
        let mut buffer = [0_u8; 4096];
        loop {
            let read = stream.read(&mut buffer).unwrap();
            assert!(
                read > 0,
                "connection closed before the full request arrived"
            );
            request.extend_from_slice(&buffer[..read]);

            let Some(header_end) = request.windows(4).position(|bytes| bytes == b"\r\n\r\n") else {
                continue;
            };
            let headers = String::from_utf8_lossy(&request[..header_end]).to_lowercase();
            let content_length = headers
                .lines()
                .find_map(|line| line.strip_prefix("content-length:"))
                .map(|value| value.trim().parse::<usize>().unwrap())
                .unwrap_or(0);
            if request.len() >= header_end + 4 + content_length {
                return String::from_utf8(request).unwrap();
            }
        }
    }

    #[test]
    fn event_interval_is_bounded_and_description_is_plain_text() {
        let mut ring = FrameRing::new(4).unwrap();
        let mut detector = EventDetector::new(0.3, 0.1, 2, 4).unwrap();
        for sequence in 0..5 {
            ring.push(Frame {
                sequence,
                jpeg: vec![sequence as u8],
                motion_pixels: vec![0],
            });
        }
        assert_eq!(
            detector.update(5, 0.5),
            Some(Transition::Started { start_sequence: 4 })
        );
        assert_eq!(detector.update(6, 0.0), None);
        assert_eq!(
            detector.update(7, 0.0),
            Some(Transition::Ended {
                start_sequence: 4,
                end_sequence: 7
            })
        );
        assert_eq!(ring.interval(4, 7), vec![vec![4]]);
        assert_eq!(
            parse_description(&json!({
                "choices": [{"message": {"content": " 人物が右へ移動した。 "}}]
            }))
            .unwrap(),
            "人物が右へ移動した。"
        );
    }

    #[test]
    fn recorder_uses_existing_marker_rest_api() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/api/v1", listener.local_addr().unwrap());
        let server = thread::spawn(move || {
            let exchanges = [
                (
                    "GET /api/v1/healthz ",
                    "",
                    r#"{"active_episode":{"episode_id":"ep-1"}}"#,
                ),
                (
                    "POST /api/v1/episodes/ep-1/markers/start ",
                    "environment_change",
                    r#"{"marker_id":"marker-1"}"#,
                ),
                ("POST /api/v1/markers/marker-1/stop ", "success", "{}"),
                (
                    "PATCH /api/v1/markers/marker-1 ",
                    "人物が画面外へ出た。",
                    "{}",
                ),
            ];
            for (request_line, body_text, response) in exchanges {
                let (mut stream, _) = listener.accept().unwrap();
                let request = read_request(&mut stream);
                assert!(request.starts_with(request_line), "{request}");
                assert!(request.contains(body_text), "{request}");
                write!(
                    stream,
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n{}",
                    response.len(),
                    response
                )
                .unwrap();
            }
        });

        let timeout = Duration::from_secs(1);
        let event = begin_recorder_event(&url, timeout, "aspa_dev").unwrap();
        assert_eq!(
            event,
            RecorderEvent {
                episode_id: "ep-1".to_owned(),
                marker_id: "marker-1".to_owned(),
                owns_episode: false,
            }
        );
        end_recorder_event(&url, timeout, &event).unwrap();
        patch_recorder_event(&url, timeout, &event, "人物が画面外へ出た。").unwrap();
        server.join().unwrap();
    }

    #[test]
    fn recorder_auto_starts_and_stops_an_owned_episode() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/api/v1", listener.local_addr().unwrap());
        let server = thread::spawn(move || {
            let exchanges = [
                (
                    "GET /api/v1/healthz ",
                    "",
                    r#"{"status":"ok","active_episode":null}"#,
                ),
                (
                    "POST /api/v1/episodes ",
                    "aspa_dev",
                    r#"{"episode_id":"ep-auto"}"#,
                ),
                (
                    "POST /api/v1/episodes/ep-auto/markers/start ",
                    "environment_change",
                    r#"{"marker_id":"marker-auto"}"#,
                ),
                ("POST /api/v1/markers/marker-auto/stop ", "success", "{}"),
                ("POST /api/v1/episodes/ep-auto/stop ", "success", "{}"),
                (
                    "PATCH /api/v1/markers/marker-auto ",
                    "人物が画面外へ出た。",
                    "{}",
                ),
            ];
            for (request_line, body_text, response) in exchanges {
                let (mut stream, _) = listener.accept().unwrap();
                let request = read_request(&mut stream);
                assert!(request.starts_with(request_line), "{request}");
                assert!(request.contains(body_text), "{request}");
                write!(
                    stream,
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n{}",
                    response.len(),
                    response
                )
                .unwrap();
            }
        });

        let timeout = Duration::from_secs(1);
        let event = begin_recorder_event(&url, timeout, "aspa_dev").unwrap();
        assert_eq!(
            event,
            RecorderEvent {
                episode_id: "ep-auto".to_owned(),
                marker_id: "marker-auto".to_owned(),
                owns_episode: true,
            }
        );
        end_recorder_event(&url, timeout, &event).unwrap();
        patch_recorder_event(&url, timeout, &event, "人物が画面外へ出た。").unwrap();
        server.join().unwrap();
    }

    #[test]
    fn owned_episode_is_stopped_when_marker_stop_fails() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/api/v1", listener.local_addr().unwrap());
        let server = thread::spawn(move || {
            let exchanges = [
                (
                    "GET /api/v1/healthz ",
                    "",
                    "200 OK",
                    r#"{"status":"ok","active_episode":null}"#,
                ),
                (
                    "POST /api/v1/episodes ",
                    "aspa_dev",
                    "200 OK",
                    r#"{"episode_id":"ep-auto"}"#,
                ),
                (
                    "POST /api/v1/episodes/ep-auto/markers/start ",
                    "environment_change",
                    "200 OK",
                    r#"{"marker_id":"marker-auto"}"#,
                ),
                (
                    "POST /api/v1/markers/marker-auto/stop ",
                    "success",
                    "500 Internal Server Error",
                    "{}",
                ),
                (
                    "POST /api/v1/episodes/ep-auto/stop ",
                    "abort",
                    "200 OK",
                    "{}",
                ),
            ];
            for (request_line, body_text, status, response) in exchanges {
                let (mut stream, _) = listener.accept().unwrap();
                let request = read_request(&mut stream);
                assert!(request.starts_with(request_line), "{request}");
                assert!(request.contains(body_text), "{request}");
                write!(
                    stream,
                    "HTTP/1.1 {status}\r\nContent-Type: application/json\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n{}",
                    response.len(),
                    response
                )
                .unwrap();
            }
        });

        let timeout = Duration::from_secs(1);
        let event = begin_recorder_event(&url, timeout, "aspa_dev").unwrap();
        let error = end_recorder_event(&url, timeout, &event).unwrap_err();
        assert!(error.contains("recorder marker stop failed"), "{error}");
        server.join().unwrap();
    }
}
