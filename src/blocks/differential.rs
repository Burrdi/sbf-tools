//! Differential correction blocks (DiffCorrIn)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::SbfBlockParse;

// ============================================================================
// DiffCorrIn Block
// ============================================================================

/// DiffCorrIn block (Block ID 5919)
///
/// Incoming RTCM or CMR differential correction message.
#[derive(Debug, Clone)]
pub struct DiffCorrInBlock {
    tow_ms: u32,
    wnc: u16,
    /// Differential mode
    pub mode: u8,
    /// Correction source
    pub source: u8,
    /// Raw message content (RTCM or CMR bytes)
    pub message_content: Vec<u8>,
}

impl DiffCorrInBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
}

impl SbfBlockParse for DiffCorrInBlock {
    const BLOCK_ID: u16 = block_ids::DIFF_CORR_IN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 14;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("DiffCorrIn too short".into()));
        }

        let mode = data[12];
        let source = data[13];
        let message_content = data[14..].to_vec();

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            source,
            message_content,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_diff_corr_in_parse() {
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::DIFF_CORR_IN,
            block_rev: 0,
            length: 20,
            tow_ms: 5000,
            wnc: 2100,
        };
        let mut data = vec![0u8; 20];
        data[0..2].copy_from_slice(&0u16.to_le_bytes());
        data[2..4].copy_from_slice(&block_ids::DIFF_CORR_IN.to_le_bytes());
        data[4..6].copy_from_slice(&20u16.to_le_bytes());
        data[6..10].copy_from_slice(&5000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2100u16.to_le_bytes());
        data[12] = 1; // mode
        data[13] = 2; // source
        data[14..18].copy_from_slice(&[0xD3, 0x00, 0x13, 0x40]); // sample RTCM prefix

        let block = DiffCorrInBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_ms(), 5000);
        assert_eq!(block.wnc(), 2100);
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.mode, 1);
        assert_eq!(block.source, 2);
        assert_eq!(block.message_content.len(), 6);
        assert_eq!(block.message_content[0], 0xD3);
    }

    #[test]
    fn test_diff_corr_in_dnu_empty_message() {
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::DIFF_CORR_IN,
            block_rev: 0,
            length: 14,
            tow_ms: 0,
            wnc: 0,
        };
        let mut data = vec![0u8; 14];
        data[12] = 0;
        data[13] = 0;

        let block = DiffCorrInBlock::parse(&header, &data).unwrap();
        assert!(block.message_content.is_empty());
    }
}
