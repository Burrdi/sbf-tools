//! SBF stream/file reader
//!
//! Provides `SbfReader` for reading SBF blocks from any `Read` source.

use std::collections::VecDeque;
use std::io::Read;

use crate::blocks::SbfBlock;
use crate::crc::validate_block;
use crate::error::{SbfError, SbfResult};
use crate::header::{SbfHeader, SBF_SYNC};

/// Default buffer capacity (64KB)
const DEFAULT_BUFFER_CAPACITY: usize = 65536;

/// Maximum buffer size before trimming (128KB)
const MAX_BUFFER_SIZE: usize = 131072;

/// SBF block reader
///
/// Reads SBF blocks from any source implementing `Read`.
///
/// # Example
///
/// ```no_run
/// use std::fs::File;
/// use sbf_tools::SbfReader;
///
/// let file = File::open("data.sbf").unwrap();
/// let mut reader = SbfReader::new(file);
///
/// while let Some(result) = reader.next() {
///     match result {
///         Ok(block) => println!("Got block: {}", block.name()),
///         Err(e) => eprintln!("Error: {}", e),
///     }
/// }
/// ```
pub struct SbfReader<R: Read> {
    inner: R,
    buffer: VecDeque<u8>,
    /// Whether to validate CRC
    validate_crc: bool,
    /// Statistics
    stats: ReaderStats,
}

/// Reader statistics
#[derive(Debug, Clone, Default)]
pub struct ReaderStats {
    /// Total bytes read from source
    pub bytes_read: u64,
    /// Number of blocks successfully parsed
    pub blocks_parsed: u64,
    /// Number of CRC errors
    pub crc_errors: u64,
    /// Number of parse errors
    pub parse_errors: u64,
    /// Bytes skipped looking for sync
    pub bytes_skipped: u64,
}

impl<R: Read> SbfReader<R> {
    /// Create a new SBF reader
    pub fn new(reader: R) -> Self {
        Self {
            inner: reader,
            buffer: VecDeque::with_capacity(DEFAULT_BUFFER_CAPACITY),
            validate_crc: true,
            stats: ReaderStats::default(),
        }
    }

    /// Create reader with specific buffer capacity
    pub fn with_capacity(reader: R, capacity: usize) -> Self {
        Self {
            inner: reader,
            buffer: VecDeque::with_capacity(capacity),
            validate_crc: true,
            stats: ReaderStats::default(),
        }
    }

    /// Enable or disable CRC validation (default: enabled)
    pub fn validate_crc(mut self, validate: bool) -> Self {
        self.validate_crc = validate;
        self
    }

    /// Get reader statistics
    pub fn stats(&self) -> &ReaderStats {
        &self.stats
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.stats = ReaderStats::default();
    }

    /// Read the next SBF block
    ///
    /// Returns `Ok(Some(block))` if a block was read successfully,
    /// `Ok(None)` if end of stream was reached,
    /// or `Err(e)` if an error occurred.
    pub fn read_block(&mut self) -> SbfResult<Option<SbfBlock>> {
        loop {
            // Try to find sync bytes in buffer
            if let Some(sync_pos) = self.find_sync() {
                // Remove any bytes before sync
                if sync_pos > 0 {
                    self.stats.bytes_skipped += sync_pos as u64;
                    self.buffer.drain(0..sync_pos);
                }

                // Try to parse block
                match self.try_parse_block() {
                    Ok(Some((block, consumed))) => {
                        // Remove consumed bytes
                        self.buffer.drain(0..consumed);
                        self.stats.blocks_parsed += 1;
                        return Ok(Some(block));
                    }
                    Ok(None) => {
                        // Need more data
                        if !self.fill_buffer()? {
                            // EOF reached
                            if self.buffer.len() > 0 {
                                // Partial data at end
                                return Err(SbfError::IncompleteBlock {
                                    needed: 8,
                                    have: self.buffer.len(),
                                });
                            }
                            return Ok(None);
                        }
                    }
                    Err(SbfError::InvalidSync) => {
                        // Skip one byte and try again
                        self.buffer.remove(0);
                        self.stats.bytes_skipped += 1;
                    }
                    Err(SbfError::CrcMismatch { .. }) => {
                        // CRC error - skip sync and continue
                        self.buffer.remove(0);
                        self.stats.crc_errors += 1;
                        self.stats.bytes_skipped += 1;
                    }
                    Err(_e) => {
                        // Other parse error - skip sync and continue
                        self.buffer.remove(0);
                        self.stats.parse_errors += 1;
                        self.stats.bytes_skipped += 1;
                        // Continue to next potential sync
                    }
                }
            } else {
                // No sync found - need more data
                if !self.fill_buffer()? {
                    return Ok(None);
                }
            }

            // Prevent buffer from growing too large
            self.trim_buffer();
        }
    }

    /// Find sync bytes in buffer
    fn find_sync(&self) -> Option<usize> {
        if self.buffer.len() < 2 {
            return None;
        }

        (0..(self.buffer.len() - 1))
            .find(|&i| self.buffer[i] == SBF_SYNC[0] && self.buffer[i + 1] == SBF_SYNC[1])
    }

    /// Try to parse a block from the current buffer position
    fn try_parse_block(&mut self) -> SbfResult<Option<(SbfBlock, usize)>> {
        if self.buffer.len() < 8 {
            return Ok(None);
        }

        let buffer = self.buffer.make_contiguous();

        // Parse header
        let header = SbfHeader::parse(&buffer[2..])?;
        let total_len = header.length as usize;

        if buffer.len() < total_len {
            return Ok(None);
        }

        // Validate CRC if enabled
        if self.validate_crc && !validate_block(&buffer[..total_len]) {
            // Get stored and calculated CRC for error message
            let stored_crc = u16::from_le_bytes([self.buffer[2], self.buffer[3]]);
            return Err(SbfError::CrcMismatch {
                expected: stored_crc,
                actual: 0, // We don't recalculate here
            });
        }

        // Parse block
        let (block, consumed) = SbfBlock::parse(&buffer[..total_len])?;

        Ok(Some((block, consumed)))
    }

    /// Fill buffer from source
    fn fill_buffer(&mut self) -> SbfResult<bool> {
        let mut temp = [0u8; 4096];
        match self.inner.read(&mut temp) {
            Ok(0) => Ok(false), // EOF
            Ok(n) => {
                self.buffer.extend(&temp[..n]);
                self.stats.bytes_read += n as u64;
                Ok(true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(false),
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => self.fill_buffer(),
            Err(e) => Err(SbfError::Io(e)),
        }
    }

    /// Trim buffer if too large
    fn trim_buffer(&mut self) {
        if self.buffer.capacity() > MAX_BUFFER_SIZE && self.buffer.len() < MAX_BUFFER_SIZE / 2 {
            self.buffer.shrink_to_fit();
        }
    }
}

/// Iterator implementation for SbfReader
impl<R: Read> Iterator for SbfReader<R> {
    type Item = SbfResult<SbfBlock>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.read_block() {
            Ok(Some(block)) => Some(Ok(block)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

/// Extension trait for creating SbfReader from Read types
pub trait SbfReadExt: Read + Sized {
    /// Create an SbfReader from this Read source
    fn sbf_blocks(self) -> SbfReader<Self> {
        SbfReader::new(self)
    }
}

impl<R: Read> SbfReadExt for R {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_reader_empty() {
        let data: &[u8] = &[];
        let mut reader = SbfReader::new(Cursor::new(data));

        assert!(reader.read_block().unwrap().is_none());
    }

    #[test]
    fn test_reader_no_sync() {
        let data = [0x00, 0x00, 0x00, 0x00];
        let mut reader = SbfReader::new(Cursor::new(&data[..]));

        assert!(reader.read_block().unwrap().is_none());
    }

    #[test]
    fn test_reader_stats() {
        let data: &[u8] = &[0x00, 0x00];
        let mut reader = SbfReader::new(Cursor::new(data));

        let _ = reader.read_block();

        assert_eq!(reader.stats().bytes_read, 2);
    }

    #[test]
    fn test_sbf_read_ext() {
        let data: &[u8] = &[];
        let reader = Cursor::new(data).sbf_blocks();

        assert!(reader.validate_crc);
    }
}
