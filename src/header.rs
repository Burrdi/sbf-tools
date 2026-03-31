//! SBF block header parsing
//!
//! Every SBF block starts with an 8-byte header:
//! ```text
//! [Sync: 2] [CRC: 2] [ID: 2] [Length: 2]
//! ```
//! Followed by optional TOW and WNc fields (6 bytes) in most blocks.

use crate::crc::crc16_ccitt;
use crate::error::{SbfError, SbfResult};

/// SBF sync bytes
pub const SBF_SYNC: [u8; 2] = [0x24, 0x40]; // "$@"

/// Minimum block length (header only)
pub const MIN_BLOCK_LENGTH: u16 = 8;

/// SBF block header
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SbfHeader {
    /// CRC-16 checksum
    pub crc: u16,
    /// Block ID (13 bits)
    pub block_id: u16,
    /// Block revision (3 bits)
    pub block_rev: u8,
    /// Total block length (including header)
    pub length: u16,
    /// Time of week in milliseconds (0xFFFFFFFF if not available)
    pub tow_ms: u32,
    /// GPS week number (0xFFFF if not available)
    pub wnc: u16,
}

impl SbfHeader {
    /// Parse header from bytes (starting after sync bytes)
    ///
    /// # Arguments
    /// * `data` - Data starting from CRC field (after 0x24 0x40 sync)
    ///
    /// # Returns
    /// Parsed header or error
    pub fn parse(data: &[u8]) -> SbfResult<Self> {
        if data.len() < 6 {
            return Err(SbfError::IncompleteBlock {
                needed: 6,
                have: data.len(),
            });
        }

        let crc = u16::from_le_bytes([data[0], data[1]]);
        let id_rev = u16::from_le_bytes([data[2], data[3]]);
        let length = u16::from_le_bytes([data[4], data[5]]);

        let block_id = id_rev & 0x1FFF;
        let block_rev = ((id_rev >> 13) & 0x07) as u8;

        // Validate length
        if length < MIN_BLOCK_LENGTH || (length & 0x03) != 0 {
            return Err(SbfError::InvalidLength(length));
        }

        // Parse TOW and WNc if available (most blocks have these at offset 6-11)
        let (tow_ms, wnc) = if data.len() >= 12 {
            (
                u32::from_le_bytes([data[6], data[7], data[8], data[9]]),
                u16::from_le_bytes([data[10], data[11]]),
            )
        } else {
            (0xFFFFFFFF, 0xFFFF)
        };

        Ok(Self {
            crc,
            block_id,
            block_rev,
            length,
            tow_ms,
            wnc,
        })
    }

    /// Parse header from complete block data (including sync bytes)
    ///
    /// # Arguments
    /// * `block_data` - Complete block data starting with sync bytes
    ///
    /// # Returns
    /// Parsed header or error
    pub fn parse_from_block(block_data: &[u8]) -> SbfResult<Self> {
        if block_data.len() < 2 {
            return Err(SbfError::IncompleteBlock {
                needed: 2,
                have: block_data.len(),
            });
        }

        // Verify sync bytes
        if block_data[0] != SBF_SYNC[0] || block_data[1] != SBF_SYNC[1] {
            return Err(SbfError::InvalidSync);
        }

        Self::parse(&block_data[2..])
    }

    /// Validate CRC against block data
    ///
    /// # Arguments
    /// * `block_data` - Complete block data starting with sync bytes
    ///
    /// # Returns
    /// `Ok(())` if CRC is valid, `Err(SbfError::CrcMismatch)` otherwise
    pub fn validate_crc(&self, block_data: &[u8]) -> SbfResult<()> {
        let length = self.length as usize;
        if block_data.len() < length {
            return Err(SbfError::IncompleteBlock {
                needed: length,
                have: block_data.len(),
            });
        }

        // CRC is calculated over ID + Length + Body (offset 4 to length)
        let calculated_crc = crc16_ccitt(&block_data[4..length]);

        if calculated_crc != self.crc {
            return Err(SbfError::CrcMismatch {
                expected: self.crc,
                actual: calculated_crc,
            });
        }

        Ok(())
    }

    /// Get TOW in seconds (scaled from milliseconds)
    ///
    /// Returns `None` if TOW is not available (0xFFFFFFFF)
    pub fn tow_seconds(&self) -> Option<f64> {
        if self.tow_ms == 0xFFFFFFFF {
            None
        } else {
            Some(self.tow_ms as f64 * 0.001)
        }
    }

    /// Get raw TOW in milliseconds
    ///
    /// Returns `None` if TOW is not available (0xFFFFFFFF)
    pub fn tow_ms_raw(&self) -> Option<u32> {
        if self.tow_ms == 0xFFFFFFFF {
            None
        } else {
            Some(self.tow_ms)
        }
    }

    /// Get week number
    ///
    /// Returns `None` if WNc is not available (0xFFFF)
    pub fn week_number(&self) -> Option<u16> {
        if self.wnc == 0xFFFF {
            None
        } else {
            Some(self.wnc)
        }
    }

    /// Check if time fields are valid
    pub fn has_valid_time(&self) -> bool {
        self.tow_ms != 0xFFFFFFFF && self.wnc != 0xFFFF
    }

    /// Get body offset (where block-specific data starts)
    ///
    /// Most blocks have TOW (4 bytes) + WNc (2 bytes) after the 8-byte header,
    /// so body starts at offset 14 from block start, or offset 12 from CRC.
    pub const fn body_offset() -> usize {
        12 // CRC(2) + ID(2) + Length(2) + TOW(4) + WNc(2) = 12 bytes from start of CRC
    }
}

impl std::fmt::Display for SbfHeader {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SbfHeader {{ id: {}, rev: {}, len: {}, tow: {}ms, wnc: {} }}",
            self.block_id, self.block_rev, self.length, self.tow_ms, self.wnc
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_parse() {
        // Minimal header data: CRC(2) + ID/Rev(2) + Length(2) + TOW(4) + WNc(2)
        let data = [
            0x00, 0x00, // CRC (placeholder)
            0xAB, 0x0F, // ID=0x0FAB (4011), Rev=0
            0x10, 0x00, // Length=16
            0xE8, 0x03, 0x00, 0x00, // TOW=1000ms
            0x64, 0x00, // WNc=100
        ];

        let header = SbfHeader::parse(&data).unwrap();
        assert_eq!(header.block_id, 4011);
        assert_eq!(header.block_rev, 0);
        assert_eq!(header.length, 16);
        assert_eq!(header.tow_ms, 1000);
        assert_eq!(header.wnc, 100);
    }

    #[test]
    fn test_header_id_rev_extraction() {
        // ID=4007 (PVTGeodetic), Rev=2
        // id_rev = (2 << 13) | 4007 = 0x4FA7
        let data = [
            0x00, 0x00, // CRC
            0xA7, 0x4F, // ID/Rev: 0x4FA7 -> ID=4007, Rev=2
            0x10, 0x00, // Length=16
            0x00, 0x00, 0x00, 0x00, // TOW
            0x00, 0x00, // WNc
        ];

        let header = SbfHeader::parse(&data).unwrap();
        assert_eq!(header.block_id, 4007);
        assert_eq!(header.block_rev, 2);
    }

    #[test]
    fn test_header_invalid_length() {
        // Length not divisible by 4
        let data = [
            0x00, 0x00, // CRC
            0xAB, 0x0F, // ID/Rev
            0x09, 0x00, // Length=9 (invalid)
            0x00, 0x00, 0x00, 0x00, // TOW
            0x00, 0x00, // WNc
        ];

        let result = SbfHeader::parse(&data);
        assert!(matches!(result, Err(SbfError::InvalidLength(9))));
    }

    #[test]
    fn test_header_too_short() {
        let data = [0x00, 0x00, 0x00];

        let result = SbfHeader::parse(&data);
        assert!(matches!(result, Err(SbfError::IncompleteBlock { .. })));
    }

    #[test]
    fn test_tow_seconds() {
        let header = SbfHeader {
            crc: 0,
            block_id: 4007,
            block_rev: 0,
            length: 16,
            tow_ms: 1500,
            wnc: 100,
        };

        assert_eq!(header.tow_seconds(), Some(1.5));

        let header_no_tow = SbfHeader {
            tow_ms: 0xFFFFFFFF,
            ..header
        };
        assert_eq!(header_no_tow.tow_seconds(), None);
    }

    #[test]
    fn test_parse_from_block_with_sync() {
        let block = [
            0x24, 0x40, // Sync
            0x00, 0x00, // CRC
            0xAB, 0x0F, // ID/Rev
            0x10, 0x00, // Length=16
            0x00, 0x00, 0x00, 0x00, // TOW
            0x00, 0x00, // WNc
        ];

        let header = SbfHeader::parse_from_block(&block).unwrap();
        assert_eq!(header.block_id, 4011);
    }

    #[test]
    fn test_parse_from_block_invalid_sync() {
        let block = [
            0x00, 0x00, // Bad sync
            0x00, 0x00, // CRC
            0xAB, 0x0F, // ID/Rev
            0x10, 0x00, // Length
            0x00, 0x00, 0x00, 0x00, // TOW
            0x00, 0x00, // WNc
        ];

        let result = SbfHeader::parse_from_block(&block);
        assert!(matches!(result, Err(SbfError::InvalidSync)));
    }
}
