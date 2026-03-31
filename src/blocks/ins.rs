//! INS (Integrated Navigation) blocks

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::{PvtError, PvtMode};

use super::block_ids;
use super::dnu::{F32_DNU, F64_DNU, I16_DNU, I32_DNU, U16_DNU};
use super::SbfBlockParse;

// ============================================================================
// IntPVCart Block
// ============================================================================

/// IntPVCart block (Block ID 4060)
///
/// INS position and velocity in Cartesian (ECEF) coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntPvCartBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    info: u16,
    nr_sv: u8,
    nr_ant: u8,
    gnss_pvt_mode: u8,
    datum: u8,
    gnss_age_raw: u16,
    x_m: f64,
    y_m: f64,
    z_m: f64,
    vx_mps: f32,
    vy_mps: f32,
    vz_mps: f32,
    cog_deg: f32,
}

impl IntPvCartBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn info(&self) -> u16 {
        self.info
    }
    pub fn nr_sv(&self) -> u8 {
        self.nr_sv
    }
    pub fn nr_ant(&self) -> u8 {
        self.nr_ant
    }
    pub fn gnss_pvt_mode(&self) -> u8 {
        self.gnss_pvt_mode
    }
    pub fn datum(&self) -> u8 {
        self.datum
    }
    /// GNSS age in seconds (raw × 0.01)
    pub fn gnss_age_seconds(&self) -> Option<f32> {
        if self.gnss_age_raw == U16_DNU {
            None
        } else {
            Some(self.gnss_age_raw as f32 * 0.01)
        }
    }
    pub fn gnss_age_raw(&self) -> u16 {
        self.gnss_age_raw
    }
    pub fn x_m(&self) -> Option<f64> {
        if self.x_m == F64_DNU {
            None
        } else {
            Some(self.x_m)
        }
    }
    pub fn y_m(&self) -> Option<f64> {
        if self.y_m == F64_DNU {
            None
        } else {
            Some(self.y_m)
        }
    }
    pub fn z_m(&self) -> Option<f64> {
        if self.z_m == F64_DNU {
            None
        } else {
            Some(self.z_m)
        }
    }
    pub fn velocity_x_mps(&self) -> Option<f32> {
        if self.vx_mps == F32_DNU {
            None
        } else {
            Some(self.vx_mps)
        }
    }
    pub fn velocity_y_mps(&self) -> Option<f32> {
        if self.vy_mps == F32_DNU {
            None
        } else {
            Some(self.vy_mps)
        }
    }
    pub fn velocity_z_mps(&self) -> Option<f32> {
        if self.vz_mps == F32_DNU {
            None
        } else {
            Some(self.vz_mps)
        }
    }
    pub fn course_over_ground_deg(&self) -> Option<f32> {
        if self.cog_deg == F32_DNU {
            None
        } else {
            Some(self.cog_deg)
        }
    }
}

impl SbfBlockParse for IntPvCartBlock {
    const BLOCK_ID: u16 = block_ids::INT_PV_CART;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Block-specific: Mode, Error, Info, NrSV, NrAnt, GNSSPVTMode, Datum, GNSSage,
        // X, Y, Z (f64 each), Vx, Vy, Vz, COG (f32 each)
        // 12 + 2 + 2 + 1 + 1 + 1 + 1 + 2 + 24 + 16 = 62 bytes min
        const MIN_LEN: usize = 62;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntPVCart too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let info = u16::from_le_bytes([data[14], data[15]]);
        let nr_sv = data[16];
        let nr_ant = data[17];
        let gnss_pvt_mode = data[18];
        let datum = data[19];
        let gnss_age_raw = u16::from_le_bytes([data[20], data[21]]);
        let x_m = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let y_m = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let z_m = f64::from_le_bytes(data[38..46].try_into().unwrap());
        let vx_mps = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let vy_mps = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let vz_mps = f32::from_le_bytes(data[54..58].try_into().unwrap());
        let cog_deg = f32::from_le_bytes(data[58..62].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            info,
            nr_sv,
            nr_ant,
            gnss_pvt_mode,
            datum,
            gnss_age_raw,
            x_m,
            y_m,
            z_m,
            vx_mps,
            vy_mps,
            vz_mps,
            cog_deg,
        })
    }
}

// ============================================================================
// IntPVGeod Block
// ============================================================================

/// IntPVGeod block (Block ID 4061)
///
/// INS position and velocity in geodetic coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntPvGeodBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    info: u16,
    nr_sv: u8,
    nr_ant: u8,
    gnss_pvt_mode: u8,
    datum: u8,
    gnss_age_raw: u16,
    lat_rad: f64,
    long_rad: f64,
    alt_m: f64,
    vn_mps: f32,
    ve_mps: f32,
    vu_mps: f32,
    cog_deg: f32,
}

impl IntPvGeodBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn info(&self) -> u16 {
        self.info
    }
    pub fn nr_sv(&self) -> u8 {
        self.nr_sv
    }
    pub fn nr_ant(&self) -> u8 {
        self.nr_ant
    }
    pub fn gnss_pvt_mode(&self) -> u8 {
        self.gnss_pvt_mode
    }
    pub fn datum(&self) -> u8 {
        self.datum
    }
    pub fn gnss_age_seconds(&self) -> Option<f32> {
        if self.gnss_age_raw == U16_DNU {
            None
        } else {
            Some(self.gnss_age_raw as f32 * 0.01)
        }
    }
    pub fn gnss_age_raw(&self) -> u16 {
        self.gnss_age_raw
    }
    pub fn latitude_deg(&self) -> Option<f64> {
        if self.lat_rad == F64_DNU {
            None
        } else {
            Some(self.lat_rad.to_degrees())
        }
    }
    pub fn longitude_deg(&self) -> Option<f64> {
        if self.long_rad == F64_DNU {
            None
        } else {
            Some(self.long_rad.to_degrees())
        }
    }
    pub fn altitude_m(&self) -> Option<f64> {
        if self.alt_m == F64_DNU {
            None
        } else {
            Some(self.alt_m)
        }
    }
    pub fn velocity_north_mps(&self) -> Option<f32> {
        if self.vn_mps == F32_DNU {
            None
        } else {
            Some(self.vn_mps)
        }
    }
    pub fn velocity_east_mps(&self) -> Option<f32> {
        if self.ve_mps == F32_DNU {
            None
        } else {
            Some(self.ve_mps)
        }
    }
    pub fn velocity_up_mps(&self) -> Option<f32> {
        if self.vu_mps == F32_DNU {
            None
        } else {
            Some(self.vu_mps)
        }
    }
    pub fn course_over_ground_deg(&self) -> Option<f32> {
        if self.cog_deg == F32_DNU {
            None
        } else {
            Some(self.cog_deg)
        }
    }
}

impl SbfBlockParse for IntPvGeodBlock {
    const BLOCK_ID: u16 = block_ids::INT_PV_GEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 62;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntPVGeod too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let info = u16::from_le_bytes([data[14], data[15]]);
        let nr_sv = data[16];
        let nr_ant = data[17];
        let gnss_pvt_mode = data[18];
        let datum = data[19];
        let gnss_age_raw = u16::from_le_bytes([data[20], data[21]]);
        let lat_rad = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let long_rad = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let alt_m = f64::from_le_bytes(data[38..46].try_into().unwrap());
        let vn_mps = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let ve_mps = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let vu_mps = f32::from_le_bytes(data[54..58].try_into().unwrap());
        let cog_deg = f32::from_le_bytes(data[58..62].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            info,
            nr_sv,
            nr_ant,
            gnss_pvt_mode,
            datum,
            gnss_age_raw,
            lat_rad,
            long_rad,
            alt_m,
            vn_mps,
            ve_mps,
            vu_mps,
            cog_deg,
        })
    }
}

// ============================================================================
// IntPVAAGeod Block
// ============================================================================

/// IntPVAAGeod block (Block ID 4045)
///
/// INS position, velocity, and acceleration in geodetic coordinates.
/// Uses scaled integers for compact representation.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntPvaaGeodBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    info: u16,
    gnss_pvt_mode: u8,
    datum: u8,
    gnss_age_raw: u8,
    nr_sv_ant: u8,
    pos_fine: u8,
    lat_raw: i32,
    long_raw: i32,
    alt_raw: i32,
    vn_raw: i32,
    ve_raw: i32,
    vu_raw: i32,
    ax_raw: i16,
    ay_raw: i16,
    az_raw: i16,
    heading_raw: u16,
    pitch_raw: i16,
    roll_raw: i16,
}

impl IntPvaaGeodBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn info(&self) -> u16 {
        self.info
    }
    pub fn gnss_pvt_mode(&self) -> u8 {
        self.gnss_pvt_mode
    }
    pub fn datum(&self) -> u8 {
        self.datum
    }
    /// GNSS age in seconds (raw × 0.1)
    pub fn gnss_age_seconds(&self) -> Option<f32> {
        if self.gnss_age_raw == 255 {
            None
        } else {
            Some(self.gnss_age_raw as f32 * 0.1)
        }
    }
    pub fn gnss_age_raw(&self) -> u8 {
        self.gnss_age_raw
    }
    pub fn nr_sv_ant(&self) -> u8 {
        self.nr_sv_ant
    }
    pub fn pos_fine(&self) -> u8 {
        self.pos_fine
    }
    /// NrSV = NrSVAnt >> 4
    pub fn nr_sv(&self) -> u8 {
        self.nr_sv_ant >> 4
    }
    /// NrAnt = NrSVAnt & 0x0F
    pub fn nr_ant(&self) -> u8 {
        self.nr_sv_ant & 0x0F
    }
    /// Latitude in degrees (Lat × 1e-7)
    pub fn latitude_deg(&self) -> Option<f64> {
        if self.lat_raw == I32_DNU {
            None
        } else {
            Some(self.lat_raw as f64 * 1e-7)
        }
    }
    /// Longitude in degrees (Long × 1e-7)
    pub fn longitude_deg(&self) -> Option<f64> {
        if self.long_raw == I32_DNU {
            None
        } else {
            Some(self.long_raw as f64 * 1e-7)
        }
    }
    /// Altitude in meters (Alt × 1e-3)
    pub fn altitude_m(&self) -> Option<f64> {
        if self.alt_raw == I32_DNU {
            None
        } else {
            Some(self.alt_raw as f64 * 1e-3)
        }
    }
    /// North velocity in m/s (Vn × 1e-3)
    pub fn velocity_north_mps(&self) -> Option<f64> {
        if self.vn_raw == I32_DNU {
            None
        } else {
            Some(self.vn_raw as f64 * 1e-3)
        }
    }
    /// East velocity in m/s (Ve × 1e-3)
    pub fn velocity_east_mps(&self) -> Option<f64> {
        if self.ve_raw == I32_DNU {
            None
        } else {
            Some(self.ve_raw as f64 * 1e-3)
        }
    }
    /// Up velocity in m/s (Vu × 1e-3)
    pub fn velocity_up_mps(&self) -> Option<f64> {
        if self.vu_raw == I32_DNU {
            None
        } else {
            Some(self.vu_raw as f64 * 1e-3)
        }
    }
    /// X acceleration in m/s² (Ax × 0.01)
    pub fn acceleration_x_mps2(&self) -> Option<f64> {
        if self.ax_raw == I16_DNU {
            None
        } else {
            Some(self.ax_raw as f64 * 0.01)
        }
    }
    /// Y acceleration in m/s² (Ay × 0.01)
    pub fn acceleration_y_mps2(&self) -> Option<f64> {
        if self.ay_raw == I16_DNU {
            None
        } else {
            Some(self.ay_raw as f64 * 0.01)
        }
    }
    /// Z acceleration in m/s² (Az × 0.01)
    pub fn acceleration_z_mps2(&self) -> Option<f64> {
        if self.az_raw == I16_DNU {
            None
        } else {
            Some(self.az_raw as f64 * 0.01)
        }
    }
    /// Heading in degrees (× 0.01)
    pub fn heading_deg(&self) -> Option<f64> {
        if self.heading_raw == U16_DNU {
            None
        } else {
            Some(self.heading_raw as f64 * 0.01)
        }
    }
    /// Pitch in degrees (× 0.01)
    pub fn pitch_deg(&self) -> Option<f64> {
        if self.pitch_raw == I16_DNU {
            None
        } else {
            Some(self.pitch_raw as f64 * 0.01)
        }
    }
    /// Roll in degrees (× 0.01)
    pub fn roll_deg(&self) -> Option<f64> {
        if self.roll_raw == I16_DNU {
            None
        } else {
            Some(self.roll_raw as f64 * 0.01)
        }
    }
}

impl SbfBlockParse for IntPvaaGeodBlock {
    const BLOCK_ID: u16 = block_ids::INT_PVA_AGEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Mode:1 Error:1 Info:2 GNSSPVTMode:1 Datum:1 GNSSage:1 NrSVAnt:1 PosFine:1
        // Lat:4 Long:4 Alt:4 Vn:4 Ve:4 Vu:4 Ax:2 Ay:2 Az:2 Heading:2 Pitch:2 Roll:2
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntPVAAGeod too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let info = u16::from_le_bytes([data[14], data[15]]);
        let gnss_pvt_mode = data[16];
        let datum = data[17];
        let gnss_age_raw = data[18];
        let nr_sv_ant = data[19];
        let pos_fine = data[20];
        let lat_raw = i32::from_le_bytes(data[21..25].try_into().unwrap());
        let long_raw = i32::from_le_bytes(data[25..29].try_into().unwrap());
        let alt_raw = i32::from_le_bytes(data[29..33].try_into().unwrap());
        let vn_raw = i32::from_le_bytes(data[33..37].try_into().unwrap());
        let ve_raw = i32::from_le_bytes(data[37..41].try_into().unwrap());
        let vu_raw = i32::from_le_bytes(data[41..45].try_into().unwrap());
        let ax_raw = i16::from_le_bytes(data[45..47].try_into().unwrap());
        let ay_raw = i16::from_le_bytes(data[47..49].try_into().unwrap());
        let az_raw = i16::from_le_bytes(data[49..51].try_into().unwrap());
        let heading_raw = u16::from_le_bytes([data[51], data[52]]);
        let pitch_raw = i16::from_le_bytes(data[53..55].try_into().unwrap());
        let roll_raw = i16::from_le_bytes(data[55..57].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            info,
            gnss_pvt_mode,
            datum,
            gnss_age_raw,
            nr_sv_ant,
            pos_fine,
            lat_raw,
            long_raw,
            alt_raw,
            vn_raw,
            ve_raw,
            vu_raw,
            ax_raw,
            ay_raw,
            az_raw,
            heading_raw,
            pitch_raw,
            roll_raw,
        })
    }
}

// ============================================================================
// IntAttEuler Block
// ============================================================================

/// IntAttEuler block (Block ID 4070)
///
/// INS attitude in Euler angles (heading, pitch, roll).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntAttEulerBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    info: u16,
    nr_sv: u8,
    nr_ant: u8,
    datum: u8,
    gnss_age_raw: u16,
    heading_deg: f32,
    pitch_deg: f32,
    roll_deg: f32,
    pitch_dot_dps: f32,
    roll_dot_dps: f32,
    heading_dot_dps: f32,
}

impl IntAttEulerBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn info(&self) -> u16 {
        self.info
    }
    pub fn nr_sv(&self) -> u8 {
        self.nr_sv
    }
    pub fn nr_ant(&self) -> u8 {
        self.nr_ant
    }
    pub fn datum(&self) -> u8 {
        self.datum
    }
    pub fn gnss_age_seconds(&self) -> Option<f32> {
        if self.gnss_age_raw == U16_DNU {
            None
        } else {
            Some(self.gnss_age_raw as f32 * 0.01)
        }
    }
    pub fn gnss_age_raw(&self) -> u16 {
        self.gnss_age_raw
    }
    pub fn heading_deg(&self) -> Option<f32> {
        if self.heading_deg == F32_DNU {
            None
        } else {
            Some(self.heading_deg)
        }
    }
    pub fn pitch_deg(&self) -> Option<f32> {
        if self.pitch_deg == F32_DNU {
            None
        } else {
            Some(self.pitch_deg)
        }
    }
    pub fn roll_deg(&self) -> Option<f32> {
        if self.roll_deg == F32_DNU {
            None
        } else {
            Some(self.roll_deg)
        }
    }
    pub fn pitch_rate_dps(&self) -> Option<f32> {
        if self.pitch_dot_dps == F32_DNU {
            None
        } else {
            Some(self.pitch_dot_dps)
        }
    }
    pub fn roll_rate_dps(&self) -> Option<f32> {
        if self.roll_dot_dps == F32_DNU {
            None
        } else {
            Some(self.roll_dot_dps)
        }
    }
    pub fn heading_rate_dps(&self) -> Option<f32> {
        if self.heading_dot_dps == F32_DNU {
            None
        } else {
            Some(self.heading_dot_dps)
        }
    }
}

impl SbfBlockParse for IntAttEulerBlock {
    const BLOCK_ID: u16 = block_ids::INT_ATT_EULER;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 45;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntAttEuler too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let info = u16::from_le_bytes([data[14], data[15]]);
        let nr_sv = data[16];
        let nr_ant = data[17];
        let datum = data[18];
        let gnss_age_raw = u16::from_le_bytes([data[19], data[20]]);
        let heading_deg = f32::from_le_bytes(data[21..25].try_into().unwrap());
        let pitch_deg = f32::from_le_bytes(data[25..29].try_into().unwrap());
        let roll_deg = f32::from_le_bytes(data[29..33].try_into().unwrap());
        let pitch_dot_dps = f32::from_le_bytes(data[33..37].try_into().unwrap());
        let roll_dot_dps = f32::from_le_bytes(data[37..41].try_into().unwrap());
        let heading_dot_dps = f32::from_le_bytes(data[41..45].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            info,
            nr_sv,
            nr_ant,
            datum,
            gnss_age_raw,
            heading_deg,
            pitch_deg,
            roll_deg,
            pitch_dot_dps,
            roll_dot_dps,
            heading_dot_dps,
        })
    }
}

// ============================================================================
// IntPosCovCart Block
// ============================================================================

/// IntPosCovCart block (Block ID 4062)
///
/// INS position covariance matrix in Cartesian (ECEF) coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntPosCovCartBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    cov_xx: f32,
    cov_yy: f32,
    cov_zz: f32,
    cov_xy: f32,
    cov_xz: f32,
    cov_yz: f32,
}

impl IntPosCovCartBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn error_raw(&self) -> u8 {
        self.error
    }

    pub fn cov_xx(&self) -> Option<f32> {
        if self.cov_xx == F32_DNU {
            None
        } else {
            Some(self.cov_xx)
        }
    }
    pub fn cov_yy(&self) -> Option<f32> {
        if self.cov_yy == F32_DNU {
            None
        } else {
            Some(self.cov_yy)
        }
    }
    pub fn cov_zz(&self) -> Option<f32> {
        if self.cov_zz == F32_DNU {
            None
        } else {
            Some(self.cov_zz)
        }
    }
    pub fn cov_xy(&self) -> Option<f32> {
        if self.cov_xy == F32_DNU {
            None
        } else {
            Some(self.cov_xy)
        }
    }
    pub fn cov_xz(&self) -> Option<f32> {
        if self.cov_xz == F32_DNU {
            None
        } else {
            Some(self.cov_xz)
        }
    }
    pub fn cov_yz(&self) -> Option<f32> {
        if self.cov_yz == F32_DNU {
            None
        } else {
            Some(self.cov_yz)
        }
    }

    pub fn x_std_m(&self) -> Option<f32> {
        if self.cov_xx == F32_DNU || self.cov_xx < 0.0 {
            None
        } else {
            Some(self.cov_xx.sqrt())
        }
    }
    pub fn y_std_m(&self) -> Option<f32> {
        if self.cov_yy == F32_DNU || self.cov_yy < 0.0 {
            None
        } else {
            Some(self.cov_yy.sqrt())
        }
    }
    pub fn z_std_m(&self) -> Option<f32> {
        if self.cov_zz == F32_DNU || self.cov_zz < 0.0 {
            None
        } else {
            Some(self.cov_zz.sqrt())
        }
    }
}

// ============================================================================
// IntVelCovCart Block
// ============================================================================

/// IntVelCovCart block (Block ID 4063)
///
/// INS velocity covariance matrix in Cartesian (ECEF) coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntVelCovCartBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    cov_vx_vx: f32,
    cov_vy_vy: f32,
    cov_vz_vz: f32,
    cov_vx_vy: f32,
    cov_vx_vz: f32,
    cov_vy_vz: f32,
}

impl IntVelCovCartBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn cov_vx_vx(&self) -> Option<f32> {
        if self.cov_vx_vx == F32_DNU {
            None
        } else {
            Some(self.cov_vx_vx)
        }
    }
    pub fn cov_vy_vy(&self) -> Option<f32> {
        if self.cov_vy_vy == F32_DNU {
            None
        } else {
            Some(self.cov_vy_vy)
        }
    }
    pub fn cov_vz_vz(&self) -> Option<f32> {
        if self.cov_vz_vz == F32_DNU {
            None
        } else {
            Some(self.cov_vz_vz)
        }
    }
    pub fn cov_vx_vy(&self) -> Option<f32> {
        if self.cov_vx_vy == F32_DNU {
            None
        } else {
            Some(self.cov_vx_vy)
        }
    }
    pub fn cov_vx_vz(&self) -> Option<f32> {
        if self.cov_vx_vz == F32_DNU {
            None
        } else {
            Some(self.cov_vx_vz)
        }
    }
    pub fn cov_vy_vz(&self) -> Option<f32> {
        if self.cov_vy_vz == F32_DNU {
            None
        } else {
            Some(self.cov_vy_vz)
        }
    }

    pub fn vx_std_mps(&self) -> Option<f32> {
        if self.cov_vx_vx == F32_DNU || self.cov_vx_vx < 0.0 {
            None
        } else {
            Some(self.cov_vx_vx.sqrt())
        }
    }
    pub fn vy_std_mps(&self) -> Option<f32> {
        if self.cov_vy_vy == F32_DNU || self.cov_vy_vy < 0.0 {
            None
        } else {
            Some(self.cov_vy_vy.sqrt())
        }
    }
    pub fn vz_std_mps(&self) -> Option<f32> {
        if self.cov_vz_vz == F32_DNU || self.cov_vz_vz < 0.0 {
            None
        } else {
            Some(self.cov_vz_vz.sqrt())
        }
    }
}

impl SbfBlockParse for IntVelCovCartBlock {
    const BLOCK_ID: u16 = block_ids::INT_VEL_COV_CART;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 38;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntVelCovCart too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let cov_vx_vx = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_vy_vy = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_vz_vz = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_vx_vy = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_vx_vz = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_vy_vz = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_vx_vx,
            cov_vy_vy,
            cov_vz_vz,
            cov_vx_vy,
            cov_vx_vz,
            cov_vy_vz,
        })
    }
}

// ============================================================================
// IntPosCovGeod Block
// ============================================================================

/// IntPosCovGeod block (Block ID 4064)
///
/// INS position covariance matrix in geodetic coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntPosCovGeodBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    cov_lat_lat: f32,
    cov_lon_lon: f32,
    cov_alt_alt: f32,
    cov_lat_lon: f32,
    cov_lat_alt: f32,
    cov_lon_alt: f32,
}

impl IntPosCovGeodBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn cov_lat_lat(&self) -> Option<f32> {
        if self.cov_lat_lat == F32_DNU {
            None
        } else {
            Some(self.cov_lat_lat)
        }
    }
    pub fn cov_lon_lon(&self) -> Option<f32> {
        if self.cov_lon_lon == F32_DNU {
            None
        } else {
            Some(self.cov_lon_lon)
        }
    }
    pub fn cov_alt_alt(&self) -> Option<f32> {
        if self.cov_alt_alt == F32_DNU {
            None
        } else {
            Some(self.cov_alt_alt)
        }
    }
    pub fn cov_lat_lon(&self) -> Option<f32> {
        if self.cov_lat_lon == F32_DNU {
            None
        } else {
            Some(self.cov_lat_lon)
        }
    }
    pub fn cov_lat_alt(&self) -> Option<f32> {
        if self.cov_lat_alt == F32_DNU {
            None
        } else {
            Some(self.cov_lat_alt)
        }
    }
    pub fn cov_lon_alt(&self) -> Option<f32> {
        if self.cov_lon_alt == F32_DNU {
            None
        } else {
            Some(self.cov_lon_alt)
        }
    }

    pub fn lat_std_m(&self) -> Option<f32> {
        if self.cov_lat_lat == F32_DNU || self.cov_lat_lat < 0.0 {
            None
        } else {
            Some(self.cov_lat_lat.sqrt())
        }
    }
    pub fn lon_std_m(&self) -> Option<f32> {
        if self.cov_lon_lon == F32_DNU || self.cov_lon_lon < 0.0 {
            None
        } else {
            Some(self.cov_lon_lon.sqrt())
        }
    }
    pub fn alt_std_m(&self) -> Option<f32> {
        if self.cov_alt_alt == F32_DNU || self.cov_alt_alt < 0.0 {
            None
        } else {
            Some(self.cov_alt_alt.sqrt())
        }
    }
}

impl SbfBlockParse for IntPosCovGeodBlock {
    const BLOCK_ID: u16 = block_ids::INT_POS_COV_GEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 38;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntPosCovGeod too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let cov_lat_lat = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_lon_lon = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_alt_alt = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_lat_lon = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_lat_alt = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_lon_alt = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_lat_lat,
            cov_lon_lon,
            cov_alt_alt,
            cov_lat_lon,
            cov_lat_alt,
            cov_lon_alt,
        })
    }
}

// ============================================================================
// IntVelCovGeod Block
// ============================================================================

/// IntVelCovGeod block (Block ID 4065)
///
/// INS velocity covariance matrix in geodetic coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntVelCovGeodBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    cov_vn_vn: f32,
    cov_ve_ve: f32,
    cov_vu_vu: f32,
    cov_vn_ve: f32,
    cov_vn_vu: f32,
    cov_ve_vu: f32,
}

impl IntVelCovGeodBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn cov_vn_vn(&self) -> Option<f32> {
        if self.cov_vn_vn == F32_DNU {
            None
        } else {
            Some(self.cov_vn_vn)
        }
    }
    pub fn cov_ve_ve(&self) -> Option<f32> {
        if self.cov_ve_ve == F32_DNU {
            None
        } else {
            Some(self.cov_ve_ve)
        }
    }
    pub fn cov_vu_vu(&self) -> Option<f32> {
        if self.cov_vu_vu == F32_DNU {
            None
        } else {
            Some(self.cov_vu_vu)
        }
    }
    pub fn cov_vn_ve(&self) -> Option<f32> {
        if self.cov_vn_ve == F32_DNU {
            None
        } else {
            Some(self.cov_vn_ve)
        }
    }
    pub fn cov_vn_vu(&self) -> Option<f32> {
        if self.cov_vn_vu == F32_DNU {
            None
        } else {
            Some(self.cov_vn_vu)
        }
    }
    pub fn cov_ve_vu(&self) -> Option<f32> {
        if self.cov_ve_vu == F32_DNU {
            None
        } else {
            Some(self.cov_ve_vu)
        }
    }

    pub fn vn_std_mps(&self) -> Option<f32> {
        if self.cov_vn_vn == F32_DNU || self.cov_vn_vn < 0.0 {
            None
        } else {
            Some(self.cov_vn_vn.sqrt())
        }
    }
    pub fn ve_std_mps(&self) -> Option<f32> {
        if self.cov_ve_ve == F32_DNU || self.cov_ve_ve < 0.0 {
            None
        } else {
            Some(self.cov_ve_ve.sqrt())
        }
    }
    pub fn vu_std_mps(&self) -> Option<f32> {
        if self.cov_vu_vu == F32_DNU || self.cov_vu_vu < 0.0 {
            None
        } else {
            Some(self.cov_vu_vu.sqrt())
        }
    }
}

impl SbfBlockParse for IntVelCovGeodBlock {
    const BLOCK_ID: u16 = block_ids::INT_VEL_COV_GEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 38;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntVelCovGeod too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let cov_vn_vn = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_ve_ve = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_vu_vu = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_vn_ve = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_vn_vu = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_ve_vu = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_vn_vn,
            cov_ve_ve,
            cov_vu_vu,
            cov_vn_ve,
            cov_vn_vu,
            cov_ve_vu,
        })
    }
}

// ============================================================================
// IntAttCovEuler Block
// ============================================================================

/// IntAttCovEuler block (Block ID 4072)
///
/// INS attitude covariance matrix in Euler angles.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct IntAttCovEulerBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    cov_head_head: f32,
    cov_pitch_pitch: f32,
    cov_roll_roll: f32,
    cov_head_pitch: f32,
    cov_head_roll: f32,
    cov_pitch_roll: f32,
}

impl IntAttCovEulerBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn mode_raw(&self) -> u8 {
        self.mode
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn cov_head_head(&self) -> Option<f32> {
        if self.cov_head_head == F32_DNU {
            None
        } else {
            Some(self.cov_head_head)
        }
    }
    pub fn cov_pitch_pitch(&self) -> Option<f32> {
        if self.cov_pitch_pitch == F32_DNU {
            None
        } else {
            Some(self.cov_pitch_pitch)
        }
    }
    pub fn cov_roll_roll(&self) -> Option<f32> {
        if self.cov_roll_roll == F32_DNU {
            None
        } else {
            Some(self.cov_roll_roll)
        }
    }
    pub fn cov_head_pitch(&self) -> Option<f32> {
        if self.cov_head_pitch == F32_DNU {
            None
        } else {
            Some(self.cov_head_pitch)
        }
    }
    pub fn cov_head_roll(&self) -> Option<f32> {
        if self.cov_head_roll == F32_DNU {
            None
        } else {
            Some(self.cov_head_roll)
        }
    }
    pub fn cov_pitch_roll(&self) -> Option<f32> {
        if self.cov_pitch_roll == F32_DNU {
            None
        } else {
            Some(self.cov_pitch_roll)
        }
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

impl SbfBlockParse for IntAttCovEulerBlock {
    const BLOCK_ID: u16 = block_ids::INT_ATT_COV_EULER;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 38;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntAttCovEuler too short".into()));
        }

        let mode = data[12];
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
            mode,
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

impl SbfBlockParse for IntPosCovCartBlock {
    const BLOCK_ID: u16 = block_ids::INT_POS_COV_CART;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 38;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IntPosCovCart too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        let cov_xx = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_yy = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_zz = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_xy = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_xz = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_yz = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_xx,
            cov_yy,
            cov_zz,
            cov_xy,
            cov_xz,
            cov_yz,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::SbfHeader;

    fn header_for(block_id: u16, tow_ms: u32, wnc: u16) -> SbfHeader {
        SbfHeader {
            crc: 0,
            block_id,
            block_rev: 0,
            length: 70,
            tow_ms,
            wnc,
        }
    }

    #[test]
    fn test_int_pv_cart_parse() {
        let mut data = vec![0u8; 70];
        data[0..6].copy_from_slice(&[0, 0, 0, 0, 0, 0]);
        data[6..10].copy_from_slice(&1000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2000u16.to_le_bytes());
        data[12] = 4; // Mode: RTK Fixed
        data[13] = 0; // Error: none
        data[14..16].copy_from_slice(&0u16.to_le_bytes());
        data[16] = 12; // NrSV
        data[17] = 1; // NrAnt
        data[18] = 4;
        data[19] = 0;
        data[20..22].copy_from_slice(&100u16.to_le_bytes()); // GNSSage 1.0s
                                                             // X, Y, Z - use valid ECEF values (not DNU)
        data[22..30].copy_from_slice(&1234567.0f64.to_le_bytes());
        data[30..38].copy_from_slice(&2345678.0f64.to_le_bytes());
        data[38..46].copy_from_slice(&3456789.0f64.to_le_bytes());
        data[46..50].copy_from_slice(&0.1f32.to_le_bytes());
        data[50..54].copy_from_slice(&0.2f32.to_le_bytes());
        data[54..58].copy_from_slice(&0.3f32.to_le_bytes());
        data[58..62].copy_from_slice(&45.0f32.to_le_bytes());

        let header = header_for(block_ids::INT_PV_CART, 1000, 2000);
        let block = IntPvCartBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.wnc(), 2000);
        assert_eq!(block.nr_sv(), 12);
        assert_eq!(block.gnss_age_seconds(), Some(1.0));
    }

    #[test]
    fn test_int_pv_cart_dnu() {
        let mut data = vec![0u8; 70];
        data[6..10].copy_from_slice(&1000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2000u16.to_le_bytes());
        data[20..22].copy_from_slice(&U16_DNU.to_le_bytes());
        data[22..30].copy_from_slice(&F64_DNU.to_le_bytes());
        data[46..50].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_PV_CART, 1000, 2000);
        let block = IntPvCartBlock::parse(&header, &data).unwrap();
        assert!(block.gnss_age_seconds().is_none());
        assert!(block.x_m().is_none());
        assert!(block.velocity_x_mps().is_none());
    }

    #[test]
    fn test_int_pv_geod_parse() {
        let mut data = vec![0u8; 70];
        data[6..10].copy_from_slice(&2000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2100u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..16].copy_from_slice(&0u16.to_le_bytes());
        data[16] = 10;
        data[17] = 1;
        data[18] = 4;
        data[19] = 0;
        data[20..22].copy_from_slice(&50u16.to_le_bytes());
        let lat_rad = 0.5f64;
        let long_rad = 0.3f64;
        data[22..30].copy_from_slice(&lat_rad.to_le_bytes());
        data[30..38].copy_from_slice(&long_rad.to_le_bytes());
        data[38..46].copy_from_slice(&100.0f64.to_le_bytes());
        data[46..50].copy_from_slice(&0.5f32.to_le_bytes());
        data[50..54].copy_from_slice(&0.2f32.to_le_bytes());
        data[54..58].copy_from_slice(&0.1f32.to_le_bytes());
        data[58..62].copy_from_slice(&90.0f32.to_le_bytes());

        let header = header_for(block_ids::INT_PV_GEOD, 2000, 2100);
        let block = IntPvGeodBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 2.0);
        assert_eq!(block.wnc(), 2100);
        assert_eq!(block.nr_sv(), 10);
        assert_eq!(block.gnss_age_seconds(), Some(0.5));
        assert!((block.latitude_deg().unwrap() - lat_rad.to_degrees()).abs() < 1e-6);
        assert!((block.longitude_deg().unwrap() - long_rad.to_degrees()).abs() < 1e-6);
        assert_eq!(block.altitude_m(), Some(100.0));
        assert_eq!(block.course_over_ground_deg(), Some(90.0));
    }

    #[test]
    fn test_int_pvaa_geod_parse() {
        let mut data = vec![0u8; 72];
        data[6..10].copy_from_slice(&3000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2200u16.to_le_bytes());
        data[12] = 4; // mode
        data[13] = 0; // error
        data[14..16].copy_from_slice(&0u16.to_le_bytes());
        data[16] = 4;
        data[17] = 0;
        data[18] = 5; // GNSSage 0.5s
        data[19] = 0x81; // NrSV=8, NrAnt=1
        data[20] = 0; // PosFine
                      // Lat 1e-7 deg: -33.87° = -338700000
        data[21..25].copy_from_slice(&(-338700000i32).to_le_bytes());
        // Long 1e-7 deg: 151.21° = 1512100000
        data[25..29].copy_from_slice(&1512100000i32.to_le_bytes());
        // Alt 1e-3 m: 50.5m = 50500
        data[29..33].copy_from_slice(&50500i32.to_le_bytes());
        // Vn, Ve, Vu 1e-3 m/s: 1.5, 0.2, 0.1
        data[33..37].copy_from_slice(&1500i32.to_le_bytes());
        data[37..41].copy_from_slice(&200i32.to_le_bytes());
        data[41..45].copy_from_slice(&100i32.to_le_bytes());
        // Ax, Ay, Az 0.01 m/s²: 0.1, 0, 9.8
        data[45..47].copy_from_slice(&10i16.to_le_bytes());
        data[47..49].copy_from_slice(&0i16.to_le_bytes());
        data[49..51].copy_from_slice(&980i16.to_le_bytes());
        // Heading 0.01 deg: 45° = 4500
        data[51..53].copy_from_slice(&4500u16.to_le_bytes());
        // Pitch, Roll 0.01 deg: 2°, -1°
        data[53..55].copy_from_slice(&200i16.to_le_bytes());
        data[55..57].copy_from_slice(&(-100i16).to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::INT_PVA_AGEOD,
            block_rev: 0,
            length: 72,
            tow_ms: 3000,
            wnc: 2200,
        };
        let block = IntPvaaGeodBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 3.0);
        assert_eq!(block.wnc(), 2200);
        assert_eq!(block.nr_sv(), 8);
        assert_eq!(block.nr_ant(), 1);
        assert_eq!(block.gnss_age_seconds(), Some(0.5));
        assert!((block.latitude_deg().unwrap() - (-33.87)).abs() < 1e-6);
        assert!((block.longitude_deg().unwrap() - 151.21).abs() < 1e-6);
        assert!((block.altitude_m().unwrap() - 50.5).abs() < 1e-6);
        assert!((block.velocity_north_mps().unwrap() - 1.5).abs() < 1e-6);
        assert!((block.velocity_east_mps().unwrap() - 0.2).abs() < 1e-6);
        assert!((block.velocity_up_mps().unwrap() - 0.1).abs() < 1e-6);
        assert!((block.acceleration_z_mps2().unwrap() - 9.8).abs() < 0.01);
        assert!((block.heading_deg().unwrap() - 45.0).abs() < 1e-6);
        assert!((block.pitch_deg().unwrap() - 2.0).abs() < 1e-6);
        assert!((block.roll_deg().unwrap() - (-1.0)).abs() < 1e-6);
    }

    #[test]
    fn test_int_pvaa_geod_dnu() {
        let mut data = vec![0u8; 72];
        data[6..10].copy_from_slice(&1000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2000u16.to_le_bytes());
        data[18] = 255; // GNSSage DNU
        data[21..25].copy_from_slice(&I32_DNU.to_le_bytes());
        data[45..47].copy_from_slice(&I16_DNU.to_le_bytes());
        data[51..53].copy_from_slice(&U16_DNU.to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::INT_PVA_AGEOD,
            block_rev: 0,
            length: 72,
            tow_ms: 1000,
            wnc: 2000,
        };
        let block = IntPvaaGeodBlock::parse(&header, &data).unwrap();
        assert!(block.gnss_age_seconds().is_none());
        assert!(block.latitude_deg().is_none());
        assert!(block.acceleration_x_mps2().is_none());
        assert!(block.heading_deg().is_none());
    }

    #[test]
    fn test_int_pv_geod_dnu() {
        let mut data = vec![0u8; 70];
        data[6..10].copy_from_slice(&2000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2100u16.to_le_bytes());
        data[20..22].copy_from_slice(&U16_DNU.to_le_bytes());
        data[22..30].copy_from_slice(&F64_DNU.to_le_bytes());
        data[46..50].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_PV_GEOD, 2000, 2100);
        let block = IntPvGeodBlock::parse(&header, &data).unwrap();
        assert!(block.gnss_age_seconds().is_none());
        assert!(block.latitude_deg().is_none());
        assert!(block.velocity_north_mps().is_none());
    }

    #[test]
    fn test_int_att_euler_parse() {
        let mut data = vec![0u8; 60];
        data[6..10].copy_from_slice(&3000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2200u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..16].copy_from_slice(&0u16.to_le_bytes());
        data[16] = 8;
        data[17] = 1;
        data[18] = 0;
        data[19..21].copy_from_slice(&25u16.to_le_bytes());
        data[21..25].copy_from_slice(&180.0f32.to_le_bytes());
        data[25..29].copy_from_slice(&5.0f32.to_le_bytes());
        data[29..33].copy_from_slice(&(-2.0f32).to_le_bytes());
        data[33..37].copy_from_slice(&0.1f32.to_le_bytes());
        data[37..41].copy_from_slice(&0.05f32.to_le_bytes());
        data[41..45].copy_from_slice(&1.5f32.to_le_bytes());

        let header = header_for(block_ids::INT_ATT_EULER, 3000, 2200);
        let block = IntAttEulerBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 3.0);
        assert_eq!(block.nr_sv(), 8);
        assert_eq!(block.gnss_age_raw(), 25);
        assert_eq!(block.gnss_age_seconds(), Some(0.25));
        assert_eq!(block.heading_deg(), Some(180.0));
        assert_eq!(block.pitch_deg(), Some(5.0));
        assert_eq!(block.roll_deg(), Some(-2.0));
        assert_eq!(block.heading_rate_dps(), Some(1.5));
    }

    #[test]
    fn test_int_att_euler_dnu() {
        let mut data = vec![0u8; 60];
        data[6..10].copy_from_slice(&3000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2200u16.to_le_bytes());
        data[19..21].copy_from_slice(&U16_DNU.to_le_bytes());
        data[21..25].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_ATT_EULER, 3000, 2200);
        let block = IntAttEulerBlock::parse(&header, &data).unwrap();
        assert_eq!(block.gnss_age_raw(), U16_DNU);
        assert!(block.gnss_age_seconds().is_none());
        assert!(block.heading_deg().is_none());
    }

    #[test]
    fn test_int_pos_cov_cart_parse() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..18].copy_from_slice(&1.0f32.to_le_bytes());
        data[18..22].copy_from_slice(&2.0f32.to_le_bytes());
        data[22..26].copy_from_slice(&3.0f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.1f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.2f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.3f32.to_le_bytes());

        let header = header_for(block_ids::INT_POS_COV_CART, 4000, 2300);
        let block = IntPosCovCartBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.0);
        assert_eq!(block.cov_xx(), Some(1.0));
        assert_eq!(block.cov_yy(), Some(2.0));
        assert_eq!(block.x_std_m(), Some(1.0));
        assert!((block.y_std_m().unwrap() - 2.0_f32.sqrt()).abs() < 1e-5);
    }

    #[test]
    fn test_int_pos_cov_cart_dnu() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[14..18].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_POS_COV_CART, 4000, 2300);
        let block = IntPosCovCartBlock::parse(&header, &data).unwrap();
        assert!(block.cov_xx().is_none());
        assert!(block.x_std_m().is_none());
    }

    #[test]
    fn test_int_vel_cov_cart_parse() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4100u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..18].copy_from_slice(&0.01f32.to_le_bytes());
        data[18..22].copy_from_slice(&0.02f32.to_le_bytes());
        data[22..26].copy_from_slice(&0.03f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.001f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.002f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.003f32.to_le_bytes());

        let header = header_for(block_ids::INT_VEL_COV_CART, 4100, 2300);
        let block = IntVelCovCartBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.1);
        assert_eq!(block.cov_vx_vx(), Some(0.01));
        assert_eq!(block.vx_std_mps(), Some(0.1));
    }

    #[test]
    fn test_int_vel_cov_cart_dnu() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4100u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[14..18].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_VEL_COV_CART, 4100, 2300);
        let block = IntVelCovCartBlock::parse(&header, &data).unwrap();
        assert!(block.cov_vx_vx().is_none());
        assert!(block.vx_std_mps().is_none());
    }

    #[test]
    fn test_int_pos_cov_geod_parse() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4200u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..18].copy_from_slice(&1.0f32.to_le_bytes());
        data[18..22].copy_from_slice(&2.0f32.to_le_bytes());
        data[22..26].copy_from_slice(&3.0f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.1f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.2f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.3f32.to_le_bytes());

        let header = header_for(block_ids::INT_POS_COV_GEOD, 4200, 2300);
        let block = IntPosCovGeodBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.2);
        assert_eq!(block.cov_lat_lat(), Some(1.0));
        assert_eq!(block.lat_std_m(), Some(1.0));
    }

    #[test]
    fn test_int_vel_cov_geod_parse() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4300u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..18].copy_from_slice(&0.04f32.to_le_bytes());
        data[18..22].copy_from_slice(&0.09f32.to_le_bytes());
        data[22..26].copy_from_slice(&0.16f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.01f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.02f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.03f32.to_le_bytes());

        let header = header_for(block_ids::INT_VEL_COV_GEOD, 4300, 2300);
        let block = IntVelCovGeodBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.3);
        assert_eq!(block.cov_vn_vn(), Some(0.04));
        assert_eq!(block.vn_std_mps(), Some(0.2));
    }

    #[test]
    fn test_int_att_cov_euler_parse() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4400u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[12] = 4;
        data[13] = 0;
        data[14..18].copy_from_slice(&4.0f32.to_le_bytes());
        data[18..22].copy_from_slice(&1.0f32.to_le_bytes());
        data[22..26].copy_from_slice(&1.0f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.5f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.5f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.25f32.to_le_bytes());

        let header = header_for(block_ids::INT_ATT_COV_EULER, 4400, 2300);
        let block = IntAttCovEulerBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.4);
        assert_eq!(block.cov_head_head(), Some(4.0));
        assert_eq!(block.heading_std_deg(), Some(2.0));
    }

    #[test]
    fn test_int_att_cov_euler_dnu() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&4400u32.to_le_bytes());
        data[10..12].copy_from_slice(&2300u16.to_le_bytes());
        data[14..18].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::INT_ATT_COV_EULER, 4400, 2300);
        let block = IntAttCovEulerBlock::parse(&header, &data).unwrap();
        assert!(block.cov_head_head().is_none());
        assert!(block.heading_std_deg().is_none());
    }
}
