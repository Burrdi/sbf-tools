//! CRC-16-CCITT implementation for SBF block validation
//!
//! SBF uses CRC-16-CCITT with polynomial 0x1021 and initial value 0.
//! The CRC covers ID + Length + Body (excludes sync bytes and CRC field itself).

/// CRC-16-CCITT lookup table (polynomial 0x1021)
const CRC_TABLE: [u16; 256] = compute_crc_table();

/// Compute the CRC lookup table at compile time
const fn compute_crc_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut j = 0;
        while j < 8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            j += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Calculate CRC-16-CCITT checksum
///
/// # Arguments
/// * `data` - The data to calculate CRC over
///
/// # Returns
/// The 16-bit CRC value
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        let table_index = ((crc >> 8) ^ (byte as u16)) as usize;
        crc = (crc << 8) ^ CRC_TABLE[table_index];
    }
    crc
}

/// Validate an SBF block's CRC
///
/// # Arguments
/// * `block_data` - Complete block data starting from sync bytes (0x24 0x40)
///
/// # Returns
/// `true` if the CRC is valid, `false` otherwise
///
/// # Block Structure
/// ```text
/// [Sync: 2] [CRC: 2] [ID: 2] [Length: 2] [Body: Length-8]
/// ```
/// CRC is calculated over [ID + Length + Body], i.e., from offset 4 to end
pub fn validate_block(block_data: &[u8]) -> bool {
    if block_data.len() < 8 {
        return false;
    }

    // Check sync bytes
    if block_data[0] != 0x24 || block_data[1] != 0x40 {
        return false;
    }

    // Extract stored CRC (bytes 2-3, little-endian)
    let stored_crc = u16::from_le_bytes([block_data[2], block_data[3]]);

    // Get length from bytes 6-7
    let length = u16::from_le_bytes([block_data[6], block_data[7]]) as usize;

    // Validate we have enough data
    if block_data.len() < length {
        return false;
    }

    // Calculate CRC over ID + Length + Body (offset 4 to length)
    // Note: length includes the 8-byte header, so data is from offset 4 to (4 + length - 4) = length
    let calculated_crc = crc16_ccitt(&block_data[4..length]);

    stored_crc == calculated_crc
}

/// Calculate CRC for block data (ID + Length + Body) and return expected CRC
///
/// # Arguments
/// * `id_length_body` - Block data starting from ID field (excludes sync and CRC)
///
/// # Returns
/// The calculated CRC-16 value
pub fn calculate_block_crc(id_length_body: &[u8]) -> u16 {
    crc16_ccitt(id_length_body)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_empty() {
        assert_eq!(crc16_ccitt(&[]), 0);
    }

    #[test]
    fn test_crc_known_value() {
        // Test vector: "123456789" gives 0x31C3 for CRC-16-CCITT with init=0
        // (0x29B1 is for CRC-16-CCITT-FALSE with init=0xFFFF)
        let data = b"123456789";
        assert_eq!(crc16_ccitt(data), 0x31C3);
    }

    #[test]
    fn test_crc_table_generation() {
        // Verify table was generated correctly
        assert_eq!(CRC_TABLE[0], 0x0000);
        assert_eq!(CRC_TABLE[1], 0x1021);
        assert_eq!(CRC_TABLE[255], 0x1EF0);
    }

    #[test]
    fn test_validate_block_too_short() {
        let short_data = [0x24, 0x40, 0x00, 0x00];
        assert!(!validate_block(&short_data));
    }

    #[test]
    fn test_validate_block_wrong_sync() {
        let bad_sync = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00];
        assert!(!validate_block(&bad_sync));
    }
}
