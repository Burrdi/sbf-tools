//! # SBF - Septentrio Binary Format Parser
//!
//! Parse Septentrio Binary Format (SBF) streams from GNSS receivers.
//!
//! ## Features
//!
//! - Read SBF blocks from any `Read` source
//! - Decode measurement, PVT, nav, status, SBAS, INS, and event blocks
//! - Decode `Meas3` epochs with the stateful `Meas3Decoder`
//! - Keep unknown blocks as raw payloads
//! - Parse-only API
//!
//! ## Quick Start
//!
//! ```no_run
//! use std::fs::File;
//! use sbf_tools::{SbfReader, SbfBlock};
//!
//! let file = File::open("data.sbf").unwrap();
//! let reader = SbfReader::new(file);
//!
//! for result in reader {
//!     match result {
//!         Ok(block) => match block {
//!             SbfBlock::PvtGeodetic(pvt) => {
//!                 if let (Some(lat), Some(lon)) = (pvt.latitude_deg(), pvt.longitude_deg()) {
//!                     println!("Position: {:.6}°, {:.6}°", lat, lon);
//!                 }
//!             }
//!             SbfBlock::Dop(dop) => {
//!                 println!("PDOP: {:.2}, HDOP: {:.2}", dop.pdop(), dop.hdop());
//!             }
//!             _ => {}
//!         },
//!         Err(e) => eprintln!("Error: {}", e),
//!     }
//! }
//! ```
//!
//! See `README.md` and the examples under `examples/` for block counting and `Meas3` decoding.

pub mod blocks;
pub mod crc;
pub mod error;
pub mod header;
pub mod reader;
pub mod types;

// Re-export commonly used items at crate root
pub use blocks::{
    // Block IDs
    block_ids,
    block_name,
    fallback_name,
    // Position
    BaseVectorCartBlock,
    BaseVectorGeodBlock,
    BdsAlmBlock,
    BdsCNav2Block,
    BdsCNav3Block,
    // Extended nav / position
    BdsNavBlock,
    BdsRawB1cBlock,
    BdsRawB2aBlock,
    BdsUtcBlock,
    // Status
    ChannelStatusBlock,
    CosmosStatusBlock,
    DiskData,
    DiskStatusBlock,
    DopBlock,
    DynDnsStatusBlock,
    EncapsulatedOutputBlock,
    // Time
    EndOfPvtBlock,
    ExtEventAttEulerBlock,
    ExtEventBaseVectGeodBlock,
    // Navigation
    GalNavBlock,
    GeoRawL5Block,
    GisActionBlock,
    GisDatabaseStatus,
    GisStatusBlock,
    GloNavBlock,
    GpsNavBlock,
    IrnssRawBlock,
    LBandBeamInfo,
    LBandBeamsBlock,
    Meas3BlockSet,
    Meas3Cn0HiResBlock,
    Meas3DecodedEpoch,
    Meas3Decoder,
    Meas3DopplerBlock,
    Meas3Measurement,
    Meas3MpBlock,
    Meas3PpBlock,
    Meas3RangesBlock,
    Meas3Satellite,
    // Measurement
    MeasEpochBlock,
    NtripClientStatusBlock,
    NtripConnectionSlot,
    NtripServerStatusBlock,
    P2ppSession,
    P2ppStatusBlock,
    PosCartBlock,
    PosCovCartesianBlock,
    PosCovGeodeticBlock,
    PosLocalBlock,
    PosProjectedBlock,
    PvtCartesianBlock,
    PvtGeodeticBlock,
    PvtSupportABlock,
    QzsAlmBlock,
    QzsNavBlock,
    ReceiverStatusBlock,
    ReceiverTimeBlock,
    RfBandEntry,
    RfStatusBlock,
    RtcmDatumBlock,
    RxMessageBlock,
    SatVisibilityBlock,
    SatVisibilityInfo,
    SatelliteMeasurement,
    // Main enum
    SbfBlock,
    VelCovCartesianBlock,
    VelCovGeodeticBlock,
};

pub use crc::{calculate_block_crc, crc16_ccitt, validate_block};
pub use error::{SbfError, SbfResult};
pub use header::{SbfHeader, SBF_SYNC};
pub use reader::{ReaderStats, SbfReadExt, SbfReader};
pub use types::{Constellation, PvtError, PvtMode, SatelliteId, SignalType};

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// SBF sync bytes
pub const SYNC_BYTES: [u8; 2] = SBF_SYNC;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        let parts: Vec<_> = VERSION.split('.').collect();
        assert_eq!(parts.len(), 3);
        assert!(parts.iter().all(|part| !part.is_empty()));
    }

    #[test]
    fn test_sync_bytes() {
        assert_eq!(SYNC_BYTES, [0x24, 0x40]);
    }
}
