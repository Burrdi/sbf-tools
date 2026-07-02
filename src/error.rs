//! SBF error types

use thiserror::Error;

/// Errors that can occur during SBF parsing
#[derive(Error, Debug)]
#[non_exhaustive]
pub enum SbfError {
    /// Invalid sync bytes (expected 0x24 0x40)
    #[error("Invalid sync bytes")]
    InvalidSync,

    /// CRC checksum mismatch
    #[error("CRC mismatch: expected {expected:#06x}, got {actual:#06x}")]
    CrcMismatch { expected: u16, actual: u16 },

    /// Block is incomplete (not enough data)
    #[error("Incomplete block: need {needed} bytes, have {have}")]
    IncompleteBlock { needed: usize, have: usize },

    /// Invalid block length (must be >= 8 and divisible by 4)
    #[error("Invalid block length: {0}")]
    InvalidLength(u16),

    /// Block-specific parsing error
    #[error("Parse error: {0}")]
    ParseError(String),

    /// I/O error during reading
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// No complete block is available yet, but the stream has not ended.
    ///
    /// Returned by [`SbfReader::read_block`](crate::SbfReader::read_block) when a
    /// non-blocking source (for example a serial port in non-blocking mode) has
    /// no data available right now. This is distinct from end of stream: EOF is
    /// reported as `Ok(None)`, whereas `WouldBlock` means the caller should retry
    /// later. Blocking sources (files, `Cursor`) never produce this.
    #[error("Would block: no data available yet")]
    WouldBlock,
}

/// Result type for SBF operations
pub type SbfResult<T> = Result<T, SbfError>;
