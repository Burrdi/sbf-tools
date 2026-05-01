//! Attitude blocks (AttEuler, AttCovEuler, AuxAntPositions, EndOfAtt)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::dnu::{f32_or_none, f64_or_none, u8_or_none, F32_DNU};
use super::SbfBlockParse;

#[cfg(test)]
use super::dnu::F64_DNU;

// ============================================================================
// AttEuler Block
// ============================================================================

/// AttEuler block (Block ID 5938)
///
/// Attitude solution in Euler angles.
#[derive(Debug, Clone)]
pub struct AttEulerBlock {
    tow_ms: u32,
    wnc: u16,
    nr_sv: u8,
    error: u8,
    mode: u16,
    datum: u8,
    heading_deg: f32,
    pitch_deg: f32,
    roll_deg: f32,
    pitch_rate_dps: f32,
    roll_rate_dps: f32,
    heading_rate_dps: f32,
}

impl AttEulerBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Number of satellites included in attitude calculations.
    ///
    /// Returns `0` when the SBF `NrSV` field is not available (`255`). Use
    /// [`Self::num_satellites_opt`] to distinguish unavailable from a real zero.
    pub fn num_satellites(&self) -> u8 {
        u8_or_none(self.nr_sv).unwrap_or(0)
    }
    /// Number of satellites included in attitude calculations, or `None` when unavailable.
    pub fn num_satellites_opt(&self) -> Option<u8> {
        u8_or_none(self.nr_sv)
    }
    /// Raw `NrSV` field from the SBF block.
    pub fn num_satellites_raw(&self) -> u8 {
        self.nr_sv
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn mode_raw(&self) -> u16 {
        self.mode
    }
    pub fn datum(&self) -> u8 {
        self.datum
    }

    pub fn heading_deg(&self) -> Option<f32> {
        f32_or_none(self.heading_deg)
    }
    pub fn pitch_deg(&self) -> Option<f32> {
        f32_or_none(self.pitch_deg)
    }
    pub fn roll_deg(&self) -> Option<f32> {
        f32_or_none(self.roll_deg)
    }
    pub fn pitch_rate_dps(&self) -> Option<f32> {
        f32_or_none(self.pitch_rate_dps)
    }
    pub fn roll_rate_dps(&self) -> Option<f32> {
        f32_or_none(self.roll_rate_dps)
    }
    pub fn heading_rate_dps(&self) -> Option<f32> {
        f32_or_none(self.heading_rate_dps)
    }
}

impl SbfBlockParse for AttEulerBlock {
    const BLOCK_ID: u16 = block_ids::ATT_EULER;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 42 {
            return Err(SbfError::ParseError("AttEuler too short".into()));
        }

        // Offsets:
        // 12: NrSV
        // 13: Error
        // 14-15: Mode (u16)
        // 16: Datum
        // 17: Reserved
        // 18-21: Heading (f32)
        // 22-25: Pitch (f32)
        // 26-29: Roll (f32)
        // 30-33: PitchDot (f32)
        // 34-37: RollDot (f32)
        // 38-41: HeadingDot (f32)

        let nr_sv = data[12];
        let error = data[13];
        let mode = u16::from_le_bytes([data[14], data[15]]);
        let datum = data[16];

        let heading_deg = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let pitch_deg = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let roll_deg = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let pitch_rate_dps = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let roll_rate_dps = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let heading_rate_dps = f32::from_le_bytes(data[38..42].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            nr_sv,
            error,
            mode,
            datum,
            heading_deg,
            pitch_deg,
            roll_deg,
            pitch_rate_dps,
            roll_rate_dps,
            heading_rate_dps,
        })
    }
}

// ============================================================================
// AttCovEuler Block
// ============================================================================

/// AttCovEuler block (Block ID 5939)
///
/// Attitude covariance matrix for Euler angles.
#[derive(Debug, Clone)]
pub struct AttCovEulerBlock {
    tow_ms: u32,
    wnc: u16,
    error: u8,
    /// Heading variance (deg^2)
    pub cov_head_head: f32,
    /// Pitch variance (deg^2)
    pub cov_pitch_pitch: f32,
    /// Roll variance (deg^2)
    pub cov_roll_roll: f32,
    /// Heading/Pitch covariance (deg^2)
    pub cov_head_pitch: f32,
    /// Heading/Roll covariance (deg^2)
    pub cov_head_roll: f32,
    /// Pitch/Roll covariance (deg^2)
    pub cov_pitch_roll: f32,
}

impl AttCovEulerBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn error_raw(&self) -> u8 {
        self.error
    }

    pub fn heading_std_deg(&self) -> Option<f32> {
        if self.cov_head_head == F32_DNU || self.cov_head_head < 0.0 {
            None
        } else {
            Some(self.cov_head_head.sqrt())
        }
    }
    pub fn pitch_std_deg(&self) -> Option<f32> {
        if self.cov_pitch_pitch == F32_DNU || self.cov_pitch_pitch < 0.0 {
            None
        } else {
            Some(self.cov_pitch_pitch.sqrt())
        }
    }
    pub fn roll_std_deg(&self) -> Option<f32> {
        if self.cov_roll_roll == F32_DNU || self.cov_roll_roll < 0.0 {
            None
        } else {
            Some(self.cov_roll_roll.sqrt())
        }
    }
}

impl SbfBlockParse for AttCovEulerBlock {
    const BLOCK_ID: u16 = block_ids::ATT_COV_EULER;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 38 {
            return Err(SbfError::ParseError("AttCovEuler too short".into()));
        }

        // Offsets:
        // 12: Reserved
        // 13: Error
        // 14-17: Cov_HeadHead (f32)
        // 18-21: Cov_PitchPitch (f32)
        // 22-25: Cov_RollRoll (f32)
        // 26-29: Cov_HeadPitch (f32)
        // 30-33: Cov_HeadRoll (f32)
        // 34-37: Cov_PitchRoll (f32)

        let error = data[13];
        let cov_head_head = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_pitch_pitch = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_roll_roll = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_head_pitch = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_head_roll = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_pitch_roll = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            error,
            cov_head_head,
            cov_pitch_pitch,
            cov_roll_roll,
            cov_head_pitch,
            cov_head_roll,
            cov_pitch_roll,
        })
    }
}

// ============================================================================
// AuxAntPositions Block
// ============================================================================

// TODO(spec-audit): AuxAntPositions (5942) is not documented in the mosaic-X5
// FW v4.15.1 guide (attitude section and block index only list AttEuler,
// AttCovEuler and EndOfAtt). Keep this legacy parser unchanged until an
// official layout/revision table is available.

/// Auxiliary antenna position info
#[derive(Debug, Clone)]
pub struct AuxAntPosition {
    pub nr_sv: u8,
    pub error: u8,
    pub ambiguity_type: u8,
    pub aux_ant_id: u8,
    d_east_m: f64,
    d_north_m: f64,
    d_up_m: f64,
    east_vel_mps: f64,
    north_vel_mps: f64,
    up_vel_mps: f64,
}

impl AuxAntPosition {
    pub fn d_east_m(&self) -> Option<f64> {
        f64_or_none(self.d_east_m)
    }
    pub fn d_north_m(&self) -> Option<f64> {
        f64_or_none(self.d_north_m)
    }
    pub fn d_up_m(&self) -> Option<f64> {
        f64_or_none(self.d_up_m)
    }
    pub fn velocity_east_mps(&self) -> Option<f64> {
        f64_or_none(self.east_vel_mps)
    }
    pub fn velocity_north_mps(&self) -> Option<f64> {
        f64_or_none(self.north_vel_mps)
    }
    pub fn velocity_up_mps(&self) -> Option<f64> {
        f64_or_none(self.up_vel_mps)
    }
}

/// AuxAntPositions block (Block ID 5942)
#[derive(Debug, Clone)]
pub struct AuxAntPositionsBlock {
    tow_ms: u32,
    wnc: u16,
    pub positions: Vec<AuxAntPosition>,
}

impl AuxAntPositionsBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_positions(&self) -> usize {
        self.positions.len()
    }
}

impl SbfBlockParse for AuxAntPositionsBlock {
    const BLOCK_ID: u16 = block_ids::AUX_ANT_POSITIONS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("AuxAntPositions too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;

        if sb_length < 52 {
            return Err(SbfError::ParseError(
                "AuxAntPositions SBLength too small".into(),
            ));
        }

        let mut positions = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let nr_sv = data[offset];
            let error = data[offset + 1];
            let ambiguity_type = data[offset + 2];
            let aux_ant_id = data[offset + 3];

            let d_east_m = f64::from_le_bytes(data[offset + 4..offset + 12].try_into().unwrap());
            let d_north_m = f64::from_le_bytes(data[offset + 12..offset + 20].try_into().unwrap());
            let d_up_m = f64::from_le_bytes(data[offset + 20..offset + 28].try_into().unwrap());
            let east_vel_mps =
                f64::from_le_bytes(data[offset + 28..offset + 36].try_into().unwrap());
            let north_vel_mps =
                f64::from_le_bytes(data[offset + 36..offset + 44].try_into().unwrap());
            let up_vel_mps = f64::from_le_bytes(data[offset + 44..offset + 52].try_into().unwrap());

            positions.push(AuxAntPosition {
                nr_sv,
                error,
                ambiguity_type,
                aux_ant_id,
                d_east_m,
                d_north_m,
                d_up_m,
                east_vel_mps,
                north_vel_mps,
                up_vel_mps,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            positions,
        })
    }
}

// ============================================================================
// EndOfAtt Block
// ============================================================================

/// EndOfAtt block (Block ID 5943)
#[derive(Debug, Clone)]
pub struct EndOfAttBlock {
    tow_ms: u32,
    wnc: u16,
}

impl EndOfAttBlock {
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

impl SbfBlockParse for EndOfAttBlock {
    const BLOCK_ID: u16 = block_ids::END_OF_ATT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 12 {
            return Err(SbfError::ParseError("EndOfAtt too short".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::SbfHeader;

    fn header_for(block_id: u16, data_len: usize, tow_ms: u32, wnc: u16) -> SbfHeader {
        SbfHeader {
            crc: 0,
            block_id,
            block_rev: 0,
            length: (data_len + 2) as u16,
            tow_ms,
            wnc,
        }
    }

    #[test]
    fn test_att_euler_scaled_accessors() {
        let block = AttEulerBlock {
            tow_ms: 1000,
            wnc: 2000,
            nr_sv: 12,
            error: 0,
            mode: 3,
            datum: 1,
            heading_deg: 45.0,
            pitch_deg: -2.0,
            roll_deg: 1.5,
            pitch_rate_dps: 0.1,
            roll_rate_dps: 0.2,
            heading_rate_dps: -0.3,
        };

        assert!((block.tow_seconds() - 1.0).abs() < 1e-6);
        assert_eq!(block.num_satellites(), 12);
        assert_eq!(block.mode_raw(), 3);
        assert!((block.heading_deg().unwrap() - 45.0).abs() < 1e-6);
        assert!((block.roll_rate_dps().unwrap() - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_att_euler_dnu_handling() {
        let block = AttEulerBlock {
            tow_ms: 0,
            wnc: 0,
            nr_sv: 255,
            error: 0,
            mode: 0,
            datum: 0,
            heading_deg: F32_DNU,
            pitch_deg: F32_DNU,
            roll_deg: 1.0,
            pitch_rate_dps: F32_DNU,
            roll_rate_dps: 0.0,
            heading_rate_dps: F32_DNU,
        };

        assert_eq!(block.num_satellites_raw(), 255);
        assert_eq!(block.num_satellites_opt(), None);
        assert_eq!(block.num_satellites(), 0);
        assert!(block.heading_deg().is_none());
        assert!(block.pitch_deg().is_none());
        assert!(block.roll_deg().is_some());
        assert!(block.pitch_rate_dps().is_none());
        assert!(block.heading_rate_dps().is_none());
    }

    #[test]
    fn test_att_euler_parse() {
        let mut data = vec![0u8; 42];
        data[12] = 8; // NrSV
        data[13] = 1; // Error
        data[14..16].copy_from_slice(&500_u16.to_le_bytes());
        data[16] = 2; // Datum
        data[18..22].copy_from_slice(&10.5_f32.to_le_bytes());
        data[22..26].copy_from_slice(&(-1.25_f32).to_le_bytes());
        data[26..30].copy_from_slice(&0.75_f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.1_f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.2_f32.to_le_bytes());
        data[38..42].copy_from_slice(&0.3_f32.to_le_bytes());

        let header = header_for(block_ids::ATT_EULER, data.len(), 123456, 2048);
        let block = AttEulerBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_satellites(), 8);
        assert_eq!(block.num_satellites_opt(), Some(8));
        assert_eq!(block.num_satellites_raw(), 8);
        assert_eq!(block.error_raw(), 1);
        assert_eq!(block.mode_raw(), 500);
        assert!((block.heading_deg().unwrap() - 10.5).abs() < 1e-6);
        assert!((block.pitch_rate_dps().unwrap() - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_att_cov_euler_std_accessors() {
        let block = AttCovEulerBlock {
            tow_ms: 2000,
            wnc: 100,
            error: 0,
            cov_head_head: 4.0,
            cov_pitch_pitch: 9.0,
            cov_roll_roll: 16.0,
            cov_head_pitch: 0.0,
            cov_head_roll: 0.0,
            cov_pitch_roll: 0.0,
        };

        assert!((block.heading_std_deg().unwrap() - 2.0).abs() < 1e-6);
        assert!((block.pitch_std_deg().unwrap() - 3.0).abs() < 1e-6);
        assert!((block.roll_std_deg().unwrap() - 4.0).abs() < 1e-6);
        assert!((block.tow_seconds() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_att_cov_euler_dnu_handling() {
        let block = AttCovEulerBlock {
            tow_ms: 0,
            wnc: 0,
            error: 0,
            cov_head_head: F32_DNU,
            cov_pitch_pitch: -1.0,
            cov_roll_roll: 1.0,
            cov_head_pitch: 0.0,
            cov_head_roll: 0.0,
            cov_pitch_roll: 0.0,
        };

        assert!(block.heading_std_deg().is_none());
        assert!(block.pitch_std_deg().is_none());
        assert!(block.roll_std_deg().is_some());
    }

    #[test]
    fn test_att_cov_euler_parse() {
        let mut data = vec![0u8; 38];
        data[13] = 2; // Error
        data[14..18].copy_from_slice(&1.0_f32.to_le_bytes());
        data[18..22].copy_from_slice(&4.0_f32.to_le_bytes());
        data[22..26].copy_from_slice(&9.0_f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.1_f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.2_f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.3_f32.to_le_bytes());

        let header = header_for(block_ids::ATT_COV_EULER, data.len(), 654321, 1024);
        let block = AttCovEulerBlock::parse(&header, &data).unwrap();

        assert_eq!(block.error_raw(), 2);
        assert!((block.heading_std_deg().unwrap() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_aux_ant_positions_accessors() {
        let info = AuxAntPosition {
            nr_sv: 7,
            error: 0,
            ambiguity_type: 1,
            aux_ant_id: 2,
            d_east_m: 1.5,
            d_north_m: -2.5,
            d_up_m: 0.5,
            east_vel_mps: 0.1,
            north_vel_mps: -0.2,
            up_vel_mps: 0.0,
        };

        assert_eq!(info.nr_sv, 7);
        assert!((info.d_east_m().unwrap() - 1.5).abs() < 1e-6);
        assert!((info.velocity_north_mps().unwrap() + 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_aux_ant_positions_dnu_handling() {
        let info = AuxAntPosition {
            nr_sv: 0,
            error: 0,
            ambiguity_type: 0,
            aux_ant_id: 0,
            d_east_m: F64_DNU,
            d_north_m: 1.0,
            d_up_m: F64_DNU,
            east_vel_mps: F64_DNU,
            north_vel_mps: 0.0,
            up_vel_mps: F64_DNU,
        };

        assert!(info.d_east_m().is_none());
        assert!(info.d_up_m().is_none());
        assert!(info.velocity_east_mps().is_none());
        assert!(info.velocity_up_mps().is_none());
        assert!(info.velocity_north_mps().is_some());
    }

    #[test]
    fn test_aux_ant_positions_parse() {
        let mut data = vec![0u8; 14 + 52];
        data[12] = 1; // N
        data[13] = 52; // SBLength

        let offset = 14;
        data[offset] = 5; // NrSV
        data[offset + 1] = 1; // Error
        data[offset + 2] = 2; // AmbiguityType
        data[offset + 3] = 3; // AuxAntID
        data[offset + 4..offset + 12].copy_from_slice(&1.0_f64.to_le_bytes());
        data[offset + 12..offset + 20].copy_from_slice(&2.0_f64.to_le_bytes());
        data[offset + 20..offset + 28].copy_from_slice(&3.0_f64.to_le_bytes());
        data[offset + 28..offset + 36].copy_from_slice(&0.1_f64.to_le_bytes());
        data[offset + 36..offset + 44].copy_from_slice(&0.2_f64.to_le_bytes());
        data[offset + 44..offset + 52].copy_from_slice(&0.3_f64.to_le_bytes());

        let header = header_for(block_ids::AUX_ANT_POSITIONS, data.len(), 9999, 55);
        let block = AuxAntPositionsBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_positions(), 1);
        let entry = &block.positions[0];
        assert_eq!(entry.aux_ant_id, 3);
        assert!((entry.d_north_m().unwrap() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_end_of_att_parse() {
        let data = vec![0u8; 14];
        let header = header_for(block_ids::END_OF_ATT, data.len(), 2500, 42);
        let block = EndOfAttBlock::parse(&header, &data).unwrap();

        assert!((block.tow_seconds() - 2.5).abs() < 1e-6);
        assert_eq!(block.wnc(), 42);
    }
}
