use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum AudioContractError {
    #[error("audio must be 16 kHz mono interleaved PCM16LE")]
    Format,
    #[error("audio sequence values must be non-negative and payload must match frame_count")]
    Payload,
    #[error("audio seq discontinuity: expected {expected}, got {actual}")]
    Sequence { expected: u64, actual: u64 },
    #[error("audio sample discontinuity: expected {expected}, got {actual}")]
    Sample { expected: u64, actual: u64 },
    #[error("audio final marker must have no frames or payload")]
    Final,
    #[error("received audio after the final marker")]
    AfterFinal,
}

#[derive(Debug)]
pub struct AudioPacket<'a> {
    pub seq: u64,
    pub sample_index: u64,
    pub frame_count: u32,
    pub data: &'a [u8],
    pub final_marker: bool,
    pub encoding: &'a str,
    pub sample_rate_hz: u32,
    pub channels: u32,
    pub bit_depth: u32,
    pub layout: &'a str,
}

#[derive(Default)]
pub struct AudioValidator {
    previous_seq: Option<u64>,
    previous_end: Option<u64>,
    finished: bool,
}

impl AudioValidator {
    pub fn validate(&mut self, packet: &AudioPacket<'_>) -> Result<(), AudioContractError> {
        if self.finished {
            return Err(AudioContractError::AfterFinal);
        }
        if packet.encoding != "PCM16LE"
            || packet.sample_rate_hz != 16_000
            || packet.channels != 1
            || packet.bit_depth != 16
            || packet.layout != "interleaved"
        {
            return Err(AudioContractError::Format);
        }
        if let Some(previous) = self.previous_seq
            && packet.seq != previous + 1
        {
            return Err(AudioContractError::Sequence {
                expected: previous + 1,
                actual: packet.seq,
            });
        }
        if let Some(previous) = self.previous_end
            && packet.sample_index != previous
        {
            return Err(AudioContractError::Sample {
                expected: previous,
                actual: packet.sample_index,
            });
        }
        if packet.final_marker {
            if packet.frame_count != 0 || !packet.data.is_empty() {
                return Err(AudioContractError::Final);
            }
            self.finished = true;
        } else if packet.frame_count == 0 || packet.data.len() != packet.frame_count as usize * 2 {
            return Err(AudioContractError::Payload);
        }
        self.previous_seq = Some(packet.seq);
        self.previous_end = Some(packet.sample_index + u64::from(packet.frame_count));
        Ok(())
    }
}

pub fn pcm16le_to_i16(payload: &[u8]) -> Result<Vec<i16>, AudioContractError> {
    if !payload.len().is_multiple_of(2) {
        return Err(AudioContractError::Payload);
    }
    Ok(payload
        .chunks_exact(2)
        .map(|bytes| i16::from_le_bytes([bytes[0], bytes[1]]))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::{AudioContractError, AudioPacket, AudioValidator, pcm16le_to_i16};

    fn packet(
        seq: u64,
        sample_index: u64,
        frames: u32,
        final_marker: bool,
    ) -> AudioPacket<'static> {
        let data = if final_marker {
            &[][..]
        } else {
            Box::leak(vec![0_u8; frames as usize * 2].into_boxed_slice())
        };
        AudioPacket {
            seq,
            sample_index,
            frame_count: frames,
            data,
            final_marker,
            encoding: "PCM16LE",
            sample_rate_hz: 16_000,
            channels: 1,
            bit_depth: 16,
            layout: "interleaved",
        }
    }

    #[test]
    fn accepts_contiguous_audio_and_one_final_marker() {
        let mut validator = AudioValidator::default();
        validator.validate(&packet(0, 0, 160, false)).unwrap();
        validator.validate(&packet(1, 160, 160, false)).unwrap();
        validator.validate(&packet(2, 320, 0, true)).unwrap();
        assert_eq!(
            validator.validate(&packet(3, 320, 160, false)),
            Err(AudioContractError::AfterFinal)
        );
    }

    #[test]
    fn rejects_sample_gaps() {
        let mut validator = AudioValidator::default();
        validator.validate(&packet(0, 0, 160, false)).unwrap();
        assert_eq!(
            validator.validate(&packet(1, 161, 160, false)),
            Err(AudioContractError::Sample {
                expected: 160,
                actual: 161,
            })
        );
    }

    #[test]
    fn decodes_little_endian_pcm() {
        assert_eq!(
            pcm16le_to_i16(&[0x34, 0x12, 0x00, 0x80]).unwrap(),
            [0x1234, -32768]
        );
    }
}
