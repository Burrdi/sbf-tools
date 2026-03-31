//! SBF error types

use thiserror::Error;

/// Errors that can occur during SBF parsing
#[derive(Error, Debug)]
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

    /// Unknown block ID
    #[error("Unknown block ID: {0}")]
    UnknownBlockId(u16),

    /// Block-specific parsing error
    #[error("Parse error: {0}")]
    ParseError(String),

    /// I/O error during reading
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// End of stream reached
    #[error("End of stream")]
    EndOfStream,
}

/// Result type for SBF operations
pub type SbfResult<T> = Result<T, SbfError>;
