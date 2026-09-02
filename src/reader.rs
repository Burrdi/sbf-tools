//! SBF stream/file reader
//!
//! Provides `SbfReader` for reading SBF blocks from any `Read` source.

use std::collections::VecDeque;
use std::io::Read;

use crate::blocks::SbfBlock;
use crate::error::{SbfError, SbfResult};
use crate::header::{SbfHeader, MIN_BLOCK_LENGTH, SBF_SYNC};

/// Default buffer capacity (64KB)
const DEFAULT_BUFFER_CAPACITY: usize = 65536;

/// Maximum buffer size before trimming (128KB)
const MAX_BUFFER_SIZE: usize = 131072;

/// Outcome of a single attempt to pull more bytes from the source.
enum Fill {
    /// At least one byte was appended to the buffer.
    Filled,
    /// The source reported end of stream (`read` returned `Ok(0)`).
    Eof,
    /// A non-blocking source has no data available right now (`WouldBlock`).
    WouldBlock,
}

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

    /// Read the next SBF block.
    ///
    /// Returns:
    /// - `Ok(Some(block))` - a block was parsed successfully.
    /// - `Ok(None)` - end of stream: the source reported EOF and no partial
    ///   block remains in the buffer.
    /// - `Err(SbfError::WouldBlock)` - a non-blocking source has no data
    ///   available yet; call again later. Blocking sources (files, `Cursor`)
    ///   never return this and report EOF instead.
    /// - `Err(SbfError::IncompleteBlock { .. })` - the stream ended in the
    ///   middle of a block (a genuinely truncated final block).
    /// - `Err(e)` - another parse or I/O error.
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
                        // Need more data to complete the block
                        match self.fill_buffer()? {
                            Fill::Filled => {}
                            Fill::WouldBlock => return Err(SbfError::WouldBlock),
                            Fill::Eof => {
                                if !self.buffer.is_empty() {
                                    // Genuinely truncated final block at real EOF
                                    return Err(SbfError::IncompleteBlock {
                                        needed: 8,
                                        have: self.buffer.len(),
                                    });
                                }
                                return Ok(None);
                            }
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
                    Err(error @ SbfError::ParseError(_)) => {
                        // Header, declared length, and CRC have already been
                        // validated by try_parse_block. Consume this complete
                        // known block and expose the typed-parser failure to the
                        // caller instead of silently losing it during resync.
                        let declared_len =
                            u16::from_le_bytes([self.buffer[6], self.buffer[7]]) as usize;
                        self.buffer.drain(0..declared_len);
                        self.stats.parse_errors += 1;
                        return Err(error);
                    }
                    Err(_) => {
                        // Invalid header or another error without a trusted
                        // complete-block boundary: skip sync and resynchronize.
                        self.buffer.remove(0);
                        self.stats.parse_errors += 1;
                        self.stats.bytes_skipped += 1;
                        // Continue to next potential sync
                    }
                }
            } else {
                // No full sync in the buffer: discard the scanned bytes, but keep
                // the last byte in case it is the first half of a sync split
                // across reads (find_sync only scans 0..len-1). This bounds the
                // buffer on garbage/no-sync streams and keeps find_sync O(n).
                let len = self.buffer.len();
                if len > 1 {
                    self.stats.bytes_skipped += (len - 1) as u64;
                    self.buffer.drain(0..len - 1);
                }
                match self.fill_buffer()? {
                    Fill::Filled => {}
                    Fill::WouldBlock => return Err(SbfError::WouldBlock),
                    Fill::Eof => return Ok(None),
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

        // Peek the declared block length (bytes 6..7) via O(1) indexing; this
        // works regardless of where the ring boundary falls.
        let peek_len = u16::from_le_bytes([self.buffer[6], self.buffer[7]]) as usize;
        if peek_len >= MIN_BLOCK_LENGTH as usize && self.buffer.len() < peek_len {
            return Ok(None); // wait for the rest of the block
        }

        // Fast path: if the whole block already lives in the contiguous front
        // slice, parse it in place with no rotation. Only rotate the ring when
        // the block straddles the boundary (or the length is invalid and needs
        // the normal parse path to produce InvalidLength).
        let front_len = self.buffer.as_slices().0.len();
        let contiguous_front = peek_len >= MIN_BLOCK_LENGTH as usize && front_len >= peek_len;
        if !contiguous_front {
            self.buffer.make_contiguous();
        }
        let buffer = self.buffer.as_slices().0;

        // Parse header
        let header = SbfHeader::parse(&buffer[2..])?;
        let total_len = header.length as usize;

        if buffer.len() < total_len {
            return Ok(None);
        }

        // Validate CRC if enabled. `validate_crc` computes the CRC and returns
        // it in the error, so `CrcMismatch.actual` carries the real value.
        if self.validate_crc {
            header.validate_crc(&buffer[..total_len])?;
        }

        // Parse block
        let (block, consumed) = SbfBlock::parse(&buffer[..total_len])?;

        Ok(Some((block, consumed)))
    }

    /// Fill buffer from source.
    ///
    /// Distinguishes end of stream (`Fill::Eof`) from a non-blocking source with
    /// no data available right now (`Fill::WouldBlock`).
    fn fill_buffer(&mut self) -> SbfResult<Fill> {
        let mut temp = [0u8; 4096];
        match self.inner.read(&mut temp) {
            Ok(0) => Ok(Fill::Eof),
            Ok(n) => {
                self.buffer.extend(&temp[..n]);
                self.stats.bytes_read += n as u64;
                Ok(Fill::Filled)
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(Fill::WouldBlock),
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

    #[test]
    fn crc_mismatch_reports_computed_actual() {
        // 16-byte block; corrupt a body byte so the stored CRC no longer matches
        // the recomputed one. Guards that CrcMismatch.actual is the real value.
        let mut block = vec![0u8; 16];
        block[0] = SBF_SYNC[0];
        block[1] = SBF_SYNC[1];
        block[4..6].copy_from_slice(&5922u16.to_le_bytes()); // EndOfMeas id, rev 0
        block[6..8].copy_from_slice(&16u16.to_le_bytes()); // length
        let stored = crate::crc::crc16_ccitt(&block[4..16]);
        block[2..4].copy_from_slice(&stored.to_le_bytes());
        block[12] ^= 0xFF; // corrupt body: computed CRC changes, stored stays
        let computed = crate::crc::crc16_ccitt(&block[4..16]);
        assert_ne!(stored, computed);

        let mut reader = SbfReader::new(Cursor::new(block.clone()));
        while reader.buffer.len() < block.len() {
            match reader.fill_buffer().unwrap() {
                Fill::Filled => {}
                _ => break,
            }
        }

        match reader.try_parse_block() {
            Err(SbfError::CrcMismatch { expected, actual }) => {
                assert_eq!(expected, stored);
                assert_eq!(actual, computed);
                assert_ne!(actual, 0);
            }
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }
}
