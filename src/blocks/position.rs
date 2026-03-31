//! Position blocks (PVTGeodetic, PVTCartesian, DOP, covariance)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::{PvtError, PvtMode};

use super::block_ids;
use super::dnu::{F32_DNU, F64_DNU, I16_DNU, U16_DNU};
use super::SbfBlockParse;

// ============================================================================
// Constants
// ============================================================================

// ============================================================================
// PVTGeodetic Block
// ============================================================================

/// PVTGeodetic_v2 block (Block ID 4007)
///
/// Position, velocity, and time in geodetic coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct PvtGeodeticBlock {
    /// Time of week in milliseconds
    tow_ms: u32,
    /// GPS week number
    wnc: u16,
    /// PVT mode
    mode: u8,
    /// Error code
    error: u8,
    /// Latitude in radians
    latitude_rad: f64,
    /// Longitude in radians
    longitude_rad: f64,
    /// Ellipsoidal height in meters
    height_m: f64,
    /// Geoid undulation in meters
    undulation_m: f32,
    /// North velocity in m/s
    vn_mps: f32,
    /// East velocity in m/s
    ve_mps: f32,
    /// Up velocity in m/s
    vu_mps: f32,
    /// Course over ground in degrees
    cog_deg: f32,
    /// Receiver clock bias in ms
    rx_clk_bias_ms: f64,
    /// Receiver clock drift in ppm
    rx_clk_drift_ppm: f32,
    /// Time system
    pub time_system: u8,
    /// Datum
    pub datum: u8,
    /// Number of satellites used
    nr_sv: u8,
    /// WAAS correction info
    pub wa_corr_info: u8,
    /// Reference station ID
    pub reference_id: u16,
    /// Mean correction age (raw, multiply by 0.01 for seconds)
    mean_corr_age_raw: u16,
    /// Signal usage info
    pub signal_info: u32,
    /// Alert flag
    pub alert_flag: u8,
    /// Number of base stations
    pub nr_bases: u8,
    /// PPP info
    pub ppp_info: u16,
    /// Latency (raw)
    latency_raw: u16,
    /// Horizontal accuracy (raw, multiply by 0.01 for meters)
    h_accuracy_raw: u16,
    /// Vertical accuracy (raw, multiply by 0.01 for meters)
    v_accuracy_raw: u16,
}

impl PvtGeodeticBlock {
    // Time accessors
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    // Mode/error accessors
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
    pub fn has_fix(&self) -> bool {
        self.mode().has_fix() && self.error().is_ok()
    }

    // Position accessors (scaled)
    pub fn latitude_deg(&self) -> Option<f64> {
        if self.latitude_rad == F64_DNU {
            None
        } else {
            Some(self.latitude_rad.to_degrees())
        }
    }
    pub fn longitude_deg(&self) -> Option<f64> {
        if self.longitude_rad == F64_DNU {
            None
        } else {
            Some(self.longitude_rad.to_degrees())
        }
    }
    pub fn height_m(&self) -> Option<f64> {
        if self.height_m == F64_DNU {
            None
        } else {
            Some(self.height_m)
        }
    }
    pub fn undulation_m(&self) -> Option<f32> {
        if self.undulation_m == F32_DNU {
            None
        } else {
            Some(self.undulation_m)
        }
    }

    // Position accessors (raw)
    pub fn latitude_rad(&self) -> f64 {
        self.latitude_rad
    }
    pub fn longitude_rad(&self) -> f64 {
        self.longitude_rad
    }

    // Velocity accessors
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

    // Clock accessors
    pub fn clock_bias_ms(&self) -> Option<f64> {
        if self.rx_clk_bias_ms == F64_DNU {
            None
        } else {
            Some(self.rx_clk_bias_ms)
        }
    }
    pub fn clock_drift_ppm(&self) -> Option<f32> {
        if self.rx_clk_drift_ppm == F32_DNU {
            None
        } else {
            Some(self.rx_clk_drift_ppm)
        }
    }

    // Satellite count
    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }

    // Accuracy (scaled)
    pub fn h_accuracy_m(&self) -> Option<f32> {
        if self.h_accuracy_raw == U16_DNU {
            None
        } else {
            Some(self.h_accuracy_raw as f32 * 0.01)
        }
    }
    pub fn v_accuracy_m(&self) -> Option<f32> {
        if self.v_accuracy_raw == U16_DNU {
            None
        } else {
            Some(self.v_accuracy_raw as f32 * 0.01)
        }
    }

    // Accuracy (raw)
    pub fn h_accuracy_raw(&self) -> u16 {
        self.h_accuracy_raw
    }
    pub fn v_accuracy_raw(&self) -> u16 {
        self.v_accuracy_raw
    }

    // Correction age
    pub fn mean_corr_age_seconds(&self) -> Option<f32> {
        if self.mean_corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.mean_corr_age_raw as f32 * 0.01)
        }
    }
}

impl SbfBlockParse for PvtGeodeticBlock {
    const BLOCK_ID: u16 = block_ids::PVT_GEODETIC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 83 {
            return Err(SbfError::ParseError("PVTGeodetic too short".into()));
        }

        // Offsets from data start (after sync):
        // 0-1: CRC, 2-3: ID, 4-5: Length
        // 6-9: TOW, 10-11: WNc
        // 12: Mode, 13: Error
        // 14-21: Latitude (f64)
        // 22-29: Longitude (f64)
        // 30-37: Height (f64)
        // 38-41: Undulation (f32)
        // 42-45: Vn (f32)
        // 46-49: Ve (f32)
        // 50-53: Vu (f32)
        // 54-57: COG (f32)
        // 58-65: RxClkBias (f64)
        // 66-69: RxClkDrift (f32)
        // 70: TimeSystem
        // 71: Datum
        // 72: NrSV
        // 73: WACorrInfo
        // 74-75: ReferenceID
        // 76-77: MeanCorrAge
        // 78-81: SignalInfo
        // 82: AlertFlag

        let mode = data[12];
        let error = data[13];

        let latitude_rad = f64::from_le_bytes(data[14..22].try_into().unwrap());
        let longitude_rad = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let height_m = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let undulation_m = f32::from_le_bytes(data[38..42].try_into().unwrap());

        let vn_mps = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let ve_mps = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let vu_mps = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let cog_deg = f32::from_le_bytes(data[54..58].try_into().unwrap());

        let rx_clk_bias_ms = f64::from_le_bytes(data[58..66].try_into().unwrap());
        let rx_clk_drift_ppm = f32::from_le_bytes(data[66..70].try_into().unwrap());

        let time_system = data[70];
        let datum = data[71];
        let nr_sv = data[72];
        let wa_corr_info = data[73];
        let reference_id = u16::from_le_bytes([data[74], data[75]]);
        let mean_corr_age_raw = u16::from_le_bytes([data[76], data[77]]);
        let signal_info = u32::from_le_bytes(data[78..82].try_into().unwrap());
        let alert_flag = data[82];

        // Rev 1+ fields
        let (nr_bases, ppp_info, latency_raw, h_accuracy_raw, v_accuracy_raw) =
            if header.block_rev >= 1 && data.len() >= 92 {
                (
                    data[83],
                    u16::from_le_bytes([data[84], data[85]]),
                    u16::from_le_bytes([data[86], data[87]]),
                    u16::from_le_bytes([data[88], data[89]]),
                    u16::from_le_bytes([data[90], data[91]]),
                )
            } else {
                (0, 0, 0, U16_DNU, U16_DNU)
            };

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            latitude_rad,
            longitude_rad,
            height_m,
            undulation_m,
            vn_mps,
            ve_mps,
            vu_mps,
            cog_deg,
            rx_clk_bias_ms,
            rx_clk_drift_ppm,
            time_system,
            datum,
            nr_sv,
            wa_corr_info,
            reference_id,
            mean_corr_age_raw,
            signal_info,
            alert_flag,
            nr_bases,
            ppp_info,
            latency_raw,
            h_accuracy_raw,
            v_accuracy_raw,
        })
    }
}

// ============================================================================
// PVTCartesian Block
// ============================================================================

/// PVTCartesian_v2 block (Block ID 4006)
///
/// Position, velocity, and time in ECEF Cartesian coordinates.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct PvtCartesianBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    x_m: f64,
    y_m: f64,
    z_m: f64,
    undulation_m: f32,
    vx_mps: f32,
    vy_mps: f32,
    vz_mps: f32,
    cog_deg: f32,
    rx_clk_bias_ms: f64,
    rx_clk_drift_ppm: f32,
    pub time_system: u8,
    pub datum: u8,
    nr_sv: u8,
    pub wa_corr_info: u8,
    pub reference_id: u16,
    mean_corr_age_raw: u16,
    pub signal_info: u32,
    pub alert_flag: u8,
    pub nr_bases: u8,
}

impl PvtCartesianBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }
    pub fn has_fix(&self) -> bool {
        self.mode().has_fix() && self.error().is_ok()
    }

    // ECEF position
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

    // ECEF velocity
    pub fn vx_mps(&self) -> Option<f32> {
        if self.vx_mps == F32_DNU {
            None
        } else {
            Some(self.vx_mps)
        }
    }
    pub fn vy_mps(&self) -> Option<f32> {
        if self.vy_mps == F32_DNU {
            None
        } else {
            Some(self.vy_mps)
        }
    }
    pub fn vz_mps(&self) -> Option<f32> {
        if self.vz_mps == F32_DNU {
            None
        } else {
            Some(self.vz_mps)
        }
    }

    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }
}

impl SbfBlockParse for PvtCartesianBlock {
    const BLOCK_ID: u16 = block_ids::PVT_CARTESIAN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 83 {
            return Err(SbfError::ParseError("PVTCartesian too short".into()));
        }

        let mode = data[12];
        let error = data[13];

        let x_m = f64::from_le_bytes(data[14..22].try_into().unwrap());
        let y_m = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let z_m = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let undulation_m = f32::from_le_bytes(data[38..42].try_into().unwrap());

        let vx_mps = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let vy_mps = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let vz_mps = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let cog_deg = f32::from_le_bytes(data[54..58].try_into().unwrap());

        let rx_clk_bias_ms = f64::from_le_bytes(data[58..66].try_into().unwrap());
        let rx_clk_drift_ppm = f32::from_le_bytes(data[66..70].try_into().unwrap());

        let time_system = data[70];
        let datum = data[71];
        let nr_sv = data[72];
        let wa_corr_info = data[73];
        let reference_id = u16::from_le_bytes([data[74], data[75]]);
        let mean_corr_age_raw = u16::from_le_bytes([data[76], data[77]]);
        let signal_info = u32::from_le_bytes(data[78..82].try_into().unwrap());
        let alert_flag = data[82];

        let nr_bases = if header.block_rev >= 1 && data.len() >= 84 {
            data[83]
        } else {
            0
        };

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            x_m,
            y_m,
            z_m,
            undulation_m,
            vx_mps,
            vy_mps,
            vz_mps,
            cog_deg,
            rx_clk_bias_ms,
            rx_clk_drift_ppm,
            time_system,
            datum,
            nr_sv,
            wa_corr_info,
            reference_id,
            mean_corr_age_raw,
            signal_info,
            alert_flag,
            nr_bases,
        })
    }
}

// ============================================================================
// DOP Block
// ============================================================================

/// DOP_v2 block (Block ID 4001)
///
/// Dilution of Precision values.
#[derive(Debug, Clone)]
pub struct DopBlock {
    tow_ms: u32,
    wnc: u16,
    nr_sv: u8,
    pdop_raw: u16,
    tdop_raw: u16,
    hdop_raw: u16,
    vdop_raw: u16,
    hpl_m: f32,
    vpl_m: f32,
}

impl DopBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }

    // Scaled DOP values (multiply by 0.01)
    pub fn pdop(&self) -> f32 {
        self.pdop_raw as f32 * 0.01
    }
    pub fn tdop(&self) -> f32 {
        self.tdop_raw as f32 * 0.01
    }
    pub fn hdop(&self) -> f32 {
        self.hdop_raw as f32 * 0.01
    }
    pub fn vdop(&self) -> f32 {
        self.vdop_raw as f32 * 0.01
    }
    /// GDOP computed as sqrt(PDOP^2 + TDOP^2)
    pub fn gdop(&self) -> f32 {
        let pdop = self.pdop();
        let tdop = self.tdop();
        (pdop * pdop + tdop * tdop).sqrt()
    }

    // Raw DOP values
    pub fn pdop_raw(&self) -> u16 {
        self.pdop_raw
    }
    pub fn tdop_raw(&self) -> u16 {
        self.tdop_raw
    }
    pub fn hdop_raw(&self) -> u16 {
        self.hdop_raw
    }
    pub fn vdop_raw(&self) -> u16 {
        self.vdop_raw
    }

    // Protection levels
    pub fn hpl_m(&self) -> Option<f32> {
        if self.hpl_m == F32_DNU {
            None
        } else {
            Some(self.hpl_m)
        }
    }
    pub fn vpl_m(&self) -> Option<f32> {
        if self.vpl_m == F32_DNU {
            None
        } else {
            Some(self.vpl_m)
        }
    }
}

impl SbfBlockParse for DopBlock {
    const BLOCK_ID: u16 = block_ids::DOP;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 22 {
            return Err(SbfError::ParseError("DOP block too short".into()));
        }

        // Offsets:
        // 12: NrSV
        // 13: Reserved
        // 14-15: PDOP
        // 16-17: TDOP
        // 18-19: HDOP
        // 20-21: VDOP
        // 22-25: HPL (f32)
        // 26-29: VPL (f32)

        let nr_sv = data[12];
        let pdop_raw = u16::from_le_bytes([data[14], data[15]]);
        let tdop_raw = u16::from_le_bytes([data[16], data[17]]);
        let hdop_raw = u16::from_le_bytes([data[18], data[19]]);
        let vdop_raw = u16::from_le_bytes([data[20], data[21]]);

        let (hpl_m, vpl_m) = if data.len() >= 30 {
            (
                f32::from_le_bytes(data[22..26].try_into().unwrap()),
                f32::from_le_bytes(data[26..30].try_into().unwrap()),
            )
        } else {
            (F32_DNU, F32_DNU)
        };

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            nr_sv,
            pdop_raw,
            tdop_raw,
            hdop_raw,
            vdop_raw,
            hpl_m,
            vpl_m,
        })
    }
}

// ============================================================================
// PosCart Block
// ============================================================================

/// PosCart block (Block ID 4044)
///
/// Position solution in ECEF Cartesian coordinates with base vector and covariance.
#[derive(Debug, Clone)]
pub struct PosCartBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    x_m: f64,
    y_m: f64,
    z_m: f64,
    base_x_m: f64,
    base_y_m: f64,
    base_z_m: f64,
    /// Position covariance (m^2)
    pub cov_xx: f32,
    pub cov_yy: f32,
    pub cov_zz: f32,
    pub cov_xy: f32,
    pub cov_xz: f32,
    pub cov_yz: f32,
    pdop_raw: u16,
    hdop_raw: u16,
    vdop_raw: u16,
    pub misc: u8,
    pub alert_flag: u8,
    pub datum: u8,
    nr_sv: u8,
    pub wa_corr_info: u8,
    pub reference_id: u16,
    mean_corr_age_raw: u16,
    pub signal_info: u32,
}

impl PosCartBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
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
    pub fn base_to_rover_x_m(&self) -> Option<f64> {
        if self.base_x_m == F64_DNU {
            None
        } else {
            Some(self.base_x_m)
        }
    }
    pub fn base_to_rover_y_m(&self) -> Option<f64> {
        if self.base_y_m == F64_DNU {
            None
        } else {
            Some(self.base_y_m)
        }
    }
    pub fn base_to_rover_z_m(&self) -> Option<f64> {
        if self.base_z_m == F64_DNU {
            None
        } else {
            Some(self.base_z_m)
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

    pub fn pdop(&self) -> Option<f32> {
        if self.pdop_raw == 0 {
            None
        } else {
            Some(self.pdop_raw as f32 * 0.01)
        }
    }
    pub fn hdop(&self) -> Option<f32> {
        if self.hdop_raw == 0 {
            None
        } else {
            Some(self.hdop_raw as f32 * 0.01)
        }
    }
    pub fn vdop(&self) -> Option<f32> {
        if self.vdop_raw == 0 {
            None
        } else {
            Some(self.vdop_raw as f32 * 0.01)
        }
    }

    pub fn pdop_raw(&self) -> u16 {
        self.pdop_raw
    }
    pub fn hdop_raw(&self) -> u16 {
        self.hdop_raw
    }
    pub fn vdop_raw(&self) -> u16 {
        self.vdop_raw
    }

    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }

    pub fn mean_corr_age_seconds(&self) -> Option<f32> {
        if self.mean_corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.mean_corr_age_raw as f32 * 0.01)
        }
    }
}

impl SbfBlockParse for PosCartBlock {
    const BLOCK_ID: u16 = block_ids::POS_CART;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 106 {
            return Err(SbfError::ParseError("PosCart too short".into()));
        }

        let mode = data[12];
        let error = data[13];
        // Byte 14 is reserved.

        let x_m = f64::from_le_bytes(data[15..23].try_into().unwrap());
        let y_m = f64::from_le_bytes(data[23..31].try_into().unwrap());
        let z_m = f64::from_le_bytes(data[31..39].try_into().unwrap());

        let base_x_m = f64::from_le_bytes(data[39..47].try_into().unwrap());
        let base_y_m = f64::from_le_bytes(data[47..55].try_into().unwrap());
        let base_z_m = f64::from_le_bytes(data[55..63].try_into().unwrap());

        let cov_xx = f32::from_le_bytes(data[63..67].try_into().unwrap());
        let cov_yy = f32::from_le_bytes(data[67..71].try_into().unwrap());
        let cov_zz = f32::from_le_bytes(data[71..75].try_into().unwrap());
        let cov_xy = f32::from_le_bytes(data[75..79].try_into().unwrap());
        let cov_xz = f32::from_le_bytes(data[79..83].try_into().unwrap());
        let cov_yz = f32::from_le_bytes(data[83..87].try_into().unwrap());

        let pdop_raw = u16::from_le_bytes([data[87], data[88]]);
        let hdop_raw = u16::from_le_bytes([data[89], data[90]]);
        let vdop_raw = u16::from_le_bytes([data[91], data[92]]);

        let misc = data[93];
        let alert_flag = data[94];
        let datum = data[95];
        let nr_sv = data[96];
        let wa_corr_info = data[97];
        let reference_id = u16::from_le_bytes([data[98], data[99]]);
        let mean_corr_age_raw = u16::from_le_bytes([data[100], data[101]]);
        let signal_info = u32::from_le_bytes(data[102..106].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            x_m,
            y_m,
            z_m,
            base_x_m,
            base_y_m,
            base_z_m,
            cov_xx,
            cov_yy,
            cov_zz,
            cov_xy,
            cov_xz,
            cov_yz,
            pdop_raw,
            hdop_raw,
            vdop_raw,
            misc,
            alert_flag,
            datum,
            nr_sv,
            wa_corr_info,
            reference_id,
            mean_corr_age_raw,
            signal_info,
        })
    }
}

// ============================================================================
// PVTSatCartesian Block
// ============================================================================

/// Per-satellite ECEF position and velocity data.
#[derive(Debug, Clone)]
pub struct PvtSatCartesianSatPos {
    pub svid: u8,
    pub freq_nr: u8,
    pub iode: u16,
    x_m: f64,
    y_m: f64,
    z_m: f64,
    vx_mps: f32,
    vy_mps: f32,
    vz_mps: f32,
}

impl PvtSatCartesianSatPos {
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

    pub fn vx_mps(&self) -> Option<f32> {
        if self.vx_mps == F32_DNU {
            None
        } else {
            Some(self.vx_mps)
        }
    }

    pub fn vy_mps(&self) -> Option<f32> {
        if self.vy_mps == F32_DNU {
            None
        } else {
            Some(self.vy_mps)
        }
    }

    pub fn vz_mps(&self) -> Option<f32> {
        if self.vz_mps == F32_DNU {
            None
        } else {
            Some(self.vz_mps)
        }
    }
}

/// PVTSatCartesian block (Block ID 4008).
#[derive(Debug, Clone)]
pub struct PvtSatCartesianBlock {
    tow_ms: u32,
    wnc: u16,
    pub satellites: Vec<PvtSatCartesianSatPos>,
}

impl PvtSatCartesianBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }

    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_satellites(&self) -> usize {
        self.satellites.len()
    }
}

impl SbfBlockParse for PvtSatCartesianBlock {
    const BLOCK_ID: u16 = block_ids::PVT_SAT_CARTESIAN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("PVTSatCartesian too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;
        if sb_length < 40 {
            return Err(SbfError::ParseError(
                "PVTSatCartesian SBLength too small".into(),
            ));
        }

        let mut satellites = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let svid = data[offset];
            let freq_nr = data[offset + 1];
            let iode = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let x_m = f64::from_le_bytes(data[offset + 4..offset + 12].try_into().unwrap());
            let y_m = f64::from_le_bytes(data[offset + 12..offset + 20].try_into().unwrap());
            let z_m = f64::from_le_bytes(data[offset + 20..offset + 28].try_into().unwrap());
            let vx_mps = f32::from_le_bytes(data[offset + 28..offset + 32].try_into().unwrap());
            let vy_mps = f32::from_le_bytes(data[offset + 32..offset + 36].try_into().unwrap());
            let vz_mps = f32::from_le_bytes(data[offset + 36..offset + 40].try_into().unwrap());

            satellites.push(PvtSatCartesianSatPos {
                svid,
                freq_nr,
                iode,
                x_m,
                y_m,
                z_m,
                vx_mps,
                vy_mps,
                vz_mps,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            satellites,
        })
    }
}

// ============================================================================
// PVTResiduals_v2 Block
// ============================================================================

/// Residual entry for a single measurement component.
#[derive(Debug, Clone)]
pub struct PvtResidualsV2ResidualInfo {
    e_i_m: f32,
    w_i_raw: u16,
    mdb_raw: u16,
}

impl PvtResidualsV2ResidualInfo {
    pub fn residual_m(&self) -> Option<f32> {
        if self.e_i_m == F32_DNU {
            None
        } else {
            Some(self.e_i_m)
        }
    }

    pub fn weight(&self) -> Option<u16> {
        if self.w_i_raw == U16_DNU {
            None
        } else {
            Some(self.w_i_raw)
        }
    }

    pub fn mdb(&self) -> Option<u16> {
        if self.mdb_raw == U16_DNU {
            None
        } else {
            Some(self.mdb_raw)
        }
    }
}

/// Per-signal residual metadata and nested residual entries.
#[derive(Debug, Clone)]
pub struct PvtResidualsV2SatSignalInfo {
    pub svid: u8,
    pub freq_nr: u8,
    pub signal_type: u8,
    pub ref_svid: u8,
    pub ref_freq_nr: u8,
    pub meas_info: u8,
    pub iode: u16,
    corr_age_raw: u16,
    pub reference_id: u16,
    pub residuals: Vec<PvtResidualsV2ResidualInfo>,
}

impl PvtResidualsV2SatSignalInfo {
    pub fn corr_age_seconds(&self) -> Option<f32> {
        if self.corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.corr_age_raw as f32 * 0.01)
        }
    }

    pub fn expected_residual_count(&self) -> usize {
        residual_count_from_meas_info(self.meas_info)
    }
}

/// PVTResiduals_v2 block (Block ID 4009).
#[derive(Debug, Clone)]
pub struct PvtResidualsV2Block {
    tow_ms: u32,
    wnc: u16,
    pub sat_signal_info: Vec<PvtResidualsV2SatSignalInfo>,
}

impl PvtResidualsV2Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }

    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_sat_signals(&self) -> usize {
        self.sat_signal_info.len()
    }
}

fn residual_count_from_meas_info(meas_info: u8) -> usize {
    ((meas_info & (1 << 2) != 0) as usize)
        + ((meas_info & (1 << 3) != 0) as usize)
        + ((meas_info & (1 << 4) != 0) as usize)
}

impl SbfBlockParse for PvtResidualsV2Block {
    const BLOCK_ID: u16 = block_ids::PVT_RESIDUALS_V2;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 15 {
            return Err(SbfError::ParseError("PVTResiduals_v2 too short".into()));
        }

        let n = data[12] as usize;
        let sb1_length = data[13] as usize;
        let sb2_length = data[14] as usize;

        if sb1_length < 12 {
            return Err(SbfError::ParseError(
                "PVTResiduals_v2 SB1Length too small".into(),
            ));
        }
        if sb2_length < 8 {
            return Err(SbfError::ParseError(
                "PVTResiduals_v2 SB2Length too small".into(),
            ));
        }

        let mut sat_signal_info = Vec::new();
        let mut offset = 15;

        for _ in 0..n {
            if offset + sb1_length > data.len() {
                break;
            }

            let svid = data[offset];
            let freq_nr = data[offset + 1];
            let signal_type = data[offset + 2];
            let ref_svid = data[offset + 3];
            let ref_freq_nr = data[offset + 4];
            let meas_info = data[offset + 5];
            let iode = u16::from_le_bytes([data[offset + 6], data[offset + 7]]);
            let corr_age_raw = u16::from_le_bytes([data[offset + 8], data[offset + 9]]);
            let reference_id = u16::from_le_bytes([data[offset + 10], data[offset + 11]]);
            offset += sb1_length;

            let residual_count = residual_count_from_meas_info(meas_info);
            let mut residuals = Vec::new();
            for _ in 0..residual_count {
                if offset + sb2_length > data.len() {
                    break;
                }

                let e_i_m = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
                let w_i_raw = u16::from_le_bytes([data[offset + 4], data[offset + 5]]);
                let mdb_raw = u16::from_le_bytes([data[offset + 6], data[offset + 7]]);
                residuals.push(PvtResidualsV2ResidualInfo {
                    e_i_m,
                    w_i_raw,
                    mdb_raw,
                });

                offset += sb2_length;
            }

            sat_signal_info.push(PvtResidualsV2SatSignalInfo {
                svid,
                freq_nr,
                signal_type,
                ref_svid,
                ref_freq_nr,
                meas_info,
                iode,
                corr_age_raw,
                reference_id,
                residuals,
            });
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            sat_signal_info,
        })
    }
}

// ============================================================================
// RAIMStatistics_v2 Block
// ============================================================================

/// RAIM integrity statistics.
#[derive(Debug, Clone)]
pub struct RaimStatisticsV2Block {
    tow_ms: u32,
    wnc: u16,
    pub integrity_flag: u8,
    herl_position_m: f32,
    verl_position_m: f32,
    herl_velocity_mps: f32,
    verl_velocity_mps: f32,
    overall_model: u16,
}

impl RaimStatisticsV2Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }

    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn herl_position_m(&self) -> Option<f32> {
        if self.herl_position_m == F32_DNU {
            None
        } else {
            Some(self.herl_position_m)
        }
    }

    pub fn verl_position_m(&self) -> Option<f32> {
        if self.verl_position_m == F32_DNU {
            None
        } else {
            Some(self.verl_position_m)
        }
    }

    pub fn herl_velocity_mps(&self) -> Option<f32> {
        if self.herl_velocity_mps == F32_DNU {
            None
        } else {
            Some(self.herl_velocity_mps)
        }
    }

    pub fn verl_velocity_mps(&self) -> Option<f32> {
        if self.verl_velocity_mps == F32_DNU {
            None
        } else {
            Some(self.verl_velocity_mps)
        }
    }

    pub fn overall_model(&self) -> Option<u16> {
        if self.overall_model == U16_DNU {
            None
        } else {
            Some(self.overall_model)
        }
    }
}

impl SbfBlockParse for RaimStatisticsV2Block {
    const BLOCK_ID: u16 = block_ids::RAIM_STATISTICS_V2;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 34 {
            return Err(SbfError::ParseError("RAIMStatistics_v2 too short".into()));
        }

        let integrity_flag = data[12];
        let herl_position_m = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let verl_position_m = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let herl_velocity_mps = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let verl_velocity_mps = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let overall_model = u16::from_le_bytes([data[30], data[31]]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            integrity_flag,
            herl_position_m,
            verl_position_m,
            herl_velocity_mps,
            verl_velocity_mps,
            overall_model,
        })
    }
}

// ============================================================================
// BaseVectorCart Block
// ============================================================================

/// Base vector info in ECEF Cartesian coordinates
#[derive(Debug, Clone)]
pub struct BaseVectorCartInfo {
    pub nr_sv: u8,
    error: u8,
    mode: u8,
    pub misc: u8,
    dx_m: f64,
    dy_m: f64,
    dz_m: f64,
    dvx_mps: f32,
    dvy_mps: f32,
    dvz_mps: f32,
    azimuth_raw: u16,
    elevation_raw: i16,
    pub reference_id: u16,
    corr_age_raw: u16,
    pub signal_info: u32,
}

impl BaseVectorCartInfo {
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn dx_m(&self) -> Option<f64> {
        if self.dx_m == F64_DNU {
            None
        } else {
            Some(self.dx_m)
        }
    }
    pub fn dy_m(&self) -> Option<f64> {
        if self.dy_m == F64_DNU {
            None
        } else {
            Some(self.dy_m)
        }
    }
    pub fn dz_m(&self) -> Option<f64> {
        if self.dz_m == F64_DNU {
            None
        } else {
            Some(self.dz_m)
        }
    }
    pub fn dvx_mps(&self) -> Option<f32> {
        if self.dvx_mps == F32_DNU {
            None
        } else {
            Some(self.dvx_mps)
        }
    }
    pub fn dvy_mps(&self) -> Option<f32> {
        if self.dvy_mps == F32_DNU {
            None
        } else {
            Some(self.dvy_mps)
        }
    }
    pub fn dvz_mps(&self) -> Option<f32> {
        if self.dvz_mps == F32_DNU {
            None
        } else {
            Some(self.dvz_mps)
        }
    }

    pub fn azimuth_deg(&self) -> Option<f64> {
        if self.azimuth_raw == U16_DNU {
            None
        } else {
            Some(self.azimuth_raw as f64 * 0.01)
        }
    }
    pub fn elevation_deg(&self) -> Option<f64> {
        if self.elevation_raw == I16_DNU {
            None
        } else {
            Some(self.elevation_raw as f64 * 0.01)
        }
    }

    pub fn corr_age_seconds(&self) -> Option<f32> {
        if self.corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.corr_age_raw as f32 * 0.01)
        }
    }
}

/// BaseVectorCart block (Block ID 4043)
#[derive(Debug, Clone)]
pub struct BaseVectorCartBlock {
    tow_ms: u32,
    wnc: u16,
    pub vectors: Vec<BaseVectorCartInfo>,
}

impl BaseVectorCartBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_vectors(&self) -> usize {
        self.vectors.len()
    }
}

impl SbfBlockParse for BaseVectorCartBlock {
    const BLOCK_ID: u16 = block_ids::BASE_VECTOR_CART;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("BaseVectorCart too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;

        if sb_length < 52 {
            return Err(SbfError::ParseError(
                "BaseVectorCart SBLength too small".into(),
            ));
        }

        let mut vectors = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let nr_sv = data[offset];
            let error = data[offset + 1];
            let mode = data[offset + 2];
            let misc = data[offset + 3];

            let dx_m = f64::from_le_bytes(data[offset + 4..offset + 12].try_into().unwrap());
            let dy_m = f64::from_le_bytes(data[offset + 12..offset + 20].try_into().unwrap());
            let dz_m = f64::from_le_bytes(data[offset + 20..offset + 28].try_into().unwrap());

            let dvx_mps = f32::from_le_bytes(data[offset + 28..offset + 32].try_into().unwrap());
            let dvy_mps = f32::from_le_bytes(data[offset + 32..offset + 36].try_into().unwrap());
            let dvz_mps = f32::from_le_bytes(data[offset + 36..offset + 40].try_into().unwrap());

            let azimuth_raw = u16::from_le_bytes([data[offset + 40], data[offset + 41]]);
            let elevation_raw = i16::from_le_bytes([data[offset + 42], data[offset + 43]]);
            let reference_id = u16::from_le_bytes([data[offset + 44], data[offset + 45]]);
            let corr_age_raw = u16::from_le_bytes([data[offset + 46], data[offset + 47]]);
            let signal_info =
                u32::from_le_bytes(data[offset + 48..offset + 52].try_into().unwrap());

            vectors.push(BaseVectorCartInfo {
                nr_sv,
                error,
                mode,
                misc,
                dx_m,
                dy_m,
                dz_m,
                dvx_mps,
                dvy_mps,
                dvz_mps,
                azimuth_raw,
                elevation_raw,
                reference_id,
                corr_age_raw,
                signal_info,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            vectors,
        })
    }
}

// ============================================================================
// BaseVectorGeod Block
// ============================================================================

/// Base vector info in local geodetic coordinates
#[derive(Debug, Clone)]
pub struct BaseVectorGeodInfo {
    pub nr_sv: u8,
    error: u8,
    mode: u8,
    pub misc: u8,
    de_m: f64,
    dn_m: f64,
    du_m: f64,
    dve_mps: f32,
    dvn_mps: f32,
    dvu_mps: f32,
    azimuth_raw: u16,
    elevation_raw: i16,
    pub reference_id: u16,
    corr_age_raw: u16,
    pub signal_info: u32,
}

impl BaseVectorGeodInfo {
    pub fn mode(&self) -> PvtMode {
        PvtMode::from_mode_byte(self.mode)
    }
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    pub fn de_m(&self) -> Option<f64> {
        if self.de_m == F64_DNU {
            None
        } else {
            Some(self.de_m)
        }
    }
    pub fn dn_m(&self) -> Option<f64> {
        if self.dn_m == F64_DNU {
            None
        } else {
            Some(self.dn_m)
        }
    }
    pub fn du_m(&self) -> Option<f64> {
        if self.du_m == F64_DNU {
            None
        } else {
            Some(self.du_m)
        }
    }
    pub fn dve_mps(&self) -> Option<f32> {
        if self.dve_mps == F32_DNU {
            None
        } else {
            Some(self.dve_mps)
        }
    }
    pub fn dvn_mps(&self) -> Option<f32> {
        if self.dvn_mps == F32_DNU {
            None
        } else {
            Some(self.dvn_mps)
        }
    }
    pub fn dvu_mps(&self) -> Option<f32> {
        if self.dvu_mps == F32_DNU {
            None
        } else {
            Some(self.dvu_mps)
        }
    }

    pub fn azimuth_deg(&self) -> Option<f64> {
        if self.azimuth_raw == U16_DNU {
            None
        } else {
            Some(self.azimuth_raw as f64 * 0.01)
        }
    }
    pub fn elevation_deg(&self) -> Option<f64> {
        if self.elevation_raw == I16_DNU {
            None
        } else {
            Some(self.elevation_raw as f64 * 0.01)
        }
    }

    pub fn corr_age_seconds(&self) -> Option<f32> {
        if self.corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.corr_age_raw as f32 * 0.01)
        }
    }
}

/// BaseVectorGeod block (Block ID 4028)
#[derive(Debug, Clone)]
pub struct BaseVectorGeodBlock {
    tow_ms: u32,
    wnc: u16,
    pub vectors: Vec<BaseVectorGeodInfo>,
}

impl BaseVectorGeodBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_vectors(&self) -> usize {
        self.vectors.len()
    }
}

impl SbfBlockParse for BaseVectorGeodBlock {
    const BLOCK_ID: u16 = block_ids::BASE_VECTOR_GEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("BaseVectorGeod too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;

        if sb_length < 52 {
            return Err(SbfError::ParseError(
                "BaseVectorGeod SBLength too small".into(),
            ));
        }

        let mut vectors = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let nr_sv = data[offset];
            let error = data[offset + 1];
            let mode = data[offset + 2];
            let misc = data[offset + 3];

            let de_m = f64::from_le_bytes(data[offset + 4..offset + 12].try_into().unwrap());
            let dn_m = f64::from_le_bytes(data[offset + 12..offset + 20].try_into().unwrap());
            let du_m = f64::from_le_bytes(data[offset + 20..offset + 28].try_into().unwrap());

            let dve_mps = f32::from_le_bytes(data[offset + 28..offset + 32].try_into().unwrap());
            let dvn_mps = f32::from_le_bytes(data[offset + 32..offset + 36].try_into().unwrap());
            let dvu_mps = f32::from_le_bytes(data[offset + 36..offset + 40].try_into().unwrap());

            let azimuth_raw = u16::from_le_bytes([data[offset + 40], data[offset + 41]]);
            let elevation_raw = i16::from_le_bytes([data[offset + 42], data[offset + 43]]);
            let reference_id = u16::from_le_bytes([data[offset + 44], data[offset + 45]]);
            let corr_age_raw = u16::from_le_bytes([data[offset + 46], data[offset + 47]]);
            let signal_info =
                u32::from_le_bytes(data[offset + 48..offset + 52].try_into().unwrap());

            vectors.push(BaseVectorGeodInfo {
                nr_sv,
                error,
                mode,
                misc,
                de_m,
                dn_m,
                du_m,
                dve_mps,
                dvn_mps,
                dvu_mps,
                azimuth_raw,
                elevation_raw,
                reference_id,
                corr_age_raw,
                signal_info,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            vectors,
        })
    }
}

// ============================================================================
// GEOCorrections Block
// ============================================================================

/// SBAS GEO satellite correction sub-block.
#[derive(Debug, Clone)]
pub struct GeoCorrectionsSatCorr {
    pub svid: u8,
    pub iode: u8,
    prc_m: f32,
    corr_age_fc_s: f32,
    delta_x_m: f32,
    delta_y_m: f32,
    delta_z_m: f32,
    delta_clock_m: f32,
    corr_age_lt_s: f32,
    iono_pp_lat_rad: f32,
    iono_pp_lon_rad: f32,
    slant_iono_m: f32,
    corr_age_iono_s: f32,
    var_flt_m2: f32,
    var_uire_m2: f32,
    var_air_m2: f32,
    var_tropo_m2: f32,
}

impl GeoCorrectionsSatCorr {
    pub fn prc_m(&self) -> Option<f32> {
        if self.prc_m == F32_DNU {
            None
        } else {
            Some(self.prc_m)
        }
    }
    pub fn corr_age_fc_seconds(&self) -> Option<f32> {
        if self.corr_age_fc_s == F32_DNU {
            None
        } else {
            Some(self.corr_age_fc_s)
        }
    }
    pub fn delta_x_m(&self) -> Option<f32> {
        if self.delta_x_m == F32_DNU {
            None
        } else {
            Some(self.delta_x_m)
        }
    }
    pub fn delta_y_m(&self) -> Option<f32> {
        if self.delta_y_m == F32_DNU {
            None
        } else {
            Some(self.delta_y_m)
        }
    }
    pub fn delta_z_m(&self) -> Option<f32> {
        if self.delta_z_m == F32_DNU {
            None
        } else {
            Some(self.delta_z_m)
        }
    }
    pub fn delta_clock_m(&self) -> Option<f32> {
        if self.delta_clock_m == F32_DNU {
            None
        } else {
            Some(self.delta_clock_m)
        }
    }
    pub fn corr_age_lt_seconds(&self) -> Option<f32> {
        if self.corr_age_lt_s == F32_DNU {
            None
        } else {
            Some(self.corr_age_lt_s)
        }
    }
    pub fn iono_pp_lat_rad(&self) -> Option<f32> {
        if self.iono_pp_lat_rad == F32_DNU {
            None
        } else {
            Some(self.iono_pp_lat_rad)
        }
    }
    pub fn iono_pp_lon_rad(&self) -> Option<f32> {
        if self.iono_pp_lon_rad == F32_DNU {
            None
        } else {
            Some(self.iono_pp_lon_rad)
        }
    }
    pub fn slant_iono_m(&self) -> Option<f32> {
        if self.slant_iono_m == F32_DNU {
            None
        } else {
            Some(self.slant_iono_m)
        }
    }
    pub fn corr_age_iono_seconds(&self) -> Option<f32> {
        if self.corr_age_iono_s == F32_DNU {
            None
        } else {
            Some(self.corr_age_iono_s)
        }
    }
    pub fn var_flt_m2(&self) -> Option<f32> {
        if self.var_flt_m2 == F32_DNU || self.var_flt_m2 < 0.0 {
            None
        } else {
            Some(self.var_flt_m2)
        }
    }
    pub fn var_uire_m2(&self) -> Option<f32> {
        if self.var_uire_m2 == F32_DNU || self.var_uire_m2 < 0.0 {
            None
        } else {
            Some(self.var_uire_m2)
        }
    }
    pub fn var_air_m2(&self) -> Option<f32> {
        if self.var_air_m2 == F32_DNU || self.var_air_m2 < 0.0 {
            None
        } else {
            Some(self.var_air_m2)
        }
    }
    pub fn var_tropo_m2(&self) -> Option<f32> {
        if self.var_tropo_m2 == F32_DNU || self.var_tropo_m2 < 0.0 {
            None
        } else {
            Some(self.var_tropo_m2)
        }
    }
}

/// GEOCorrections block (Block ID 5935).
///
/// SBAS GEO satellite corrections: fast, long-term, ionosphere, variances.
#[derive(Debug, Clone)]
pub struct GeoCorrectionsBlock {
    tow_ms: u32,
    wnc: u16,
    pub sat_corrections: Vec<GeoCorrectionsSatCorr>,
}

impl GeoCorrectionsBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_satellites(&self) -> usize {
        self.sat_corrections.len()
    }
}

impl SbfBlockParse for GeoCorrectionsBlock {
    const BLOCK_ID: u16 = block_ids::GEO_CORRECTIONS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("GEOCorrections too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;
        // SatCorr: SVID(1) + IODE(1) + 15×f32(60) = 62 bytes
        const SAT_CORR_MIN: usize = 62;
        if sb_length < SAT_CORR_MIN {
            return Err(SbfError::ParseError(
                "GEOCorrections SBLength too small".into(),
            ));
        }

        let mut sat_corrections = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let svid = data[offset];
            let iode = data[offset + 1];
            let prc_m = f32::from_le_bytes(data[offset + 2..offset + 6].try_into().unwrap());
            let corr_age_fc_s =
                f32::from_le_bytes(data[offset + 6..offset + 10].try_into().unwrap());
            let delta_x_m = f32::from_le_bytes(data[offset + 10..offset + 14].try_into().unwrap());
            let delta_y_m = f32::from_le_bytes(data[offset + 14..offset + 18].try_into().unwrap());
            let delta_z_m = f32::from_le_bytes(data[offset + 18..offset + 22].try_into().unwrap());
            let delta_clock_m =
                f32::from_le_bytes(data[offset + 22..offset + 26].try_into().unwrap());
            let corr_age_lt_s =
                f32::from_le_bytes(data[offset + 26..offset + 30].try_into().unwrap());
            let iono_pp_lat_rad =
                f32::from_le_bytes(data[offset + 30..offset + 34].try_into().unwrap());
            let iono_pp_lon_rad =
                f32::from_le_bytes(data[offset + 34..offset + 38].try_into().unwrap());
            let slant_iono_m =
                f32::from_le_bytes(data[offset + 38..offset + 42].try_into().unwrap());
            let corr_age_iono_s =
                f32::from_le_bytes(data[offset + 42..offset + 46].try_into().unwrap());
            let var_flt_m2 = f32::from_le_bytes(data[offset + 46..offset + 50].try_into().unwrap());
            let var_uire_m2 =
                f32::from_le_bytes(data[offset + 50..offset + 54].try_into().unwrap());
            let var_air_m2 = f32::from_le_bytes(data[offset + 54..offset + 58].try_into().unwrap());
            let var_tropo_m2 =
                f32::from_le_bytes(data[offset + 58..offset + 62].try_into().unwrap());

            sat_corrections.push(GeoCorrectionsSatCorr {
                svid,
                iode,
                prc_m,
                corr_age_fc_s,
                delta_x_m,
                delta_y_m,
                delta_z_m,
                delta_clock_m,
                corr_age_lt_s,
                iono_pp_lat_rad,
                iono_pp_lon_rad,
                slant_iono_m,
                corr_age_iono_s,
                var_flt_m2,
                var_uire_m2,
                var_air_m2,
                var_tropo_m2,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            sat_corrections,
        })
    }
}

// ============================================================================
// BaseStation Block
// ============================================================================

/// BaseStation block (Block ID 5949).
///
/// Base station ECEF coordinates for differential correction.
#[derive(Debug, Clone)]
pub struct BaseStationBlock {
    tow_ms: u32,
    wnc: u16,
    pub base_station_id: u16,
    pub base_type: u8,
    pub source: u8,
    pub datum: u8,
    x_m: f64,
    y_m: f64,
    z_m: f64,
}

impl BaseStationBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
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
}

impl SbfBlockParse for BaseStationBlock {
    const BLOCK_ID: u16 = block_ids::BASE_STATION;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 42 {
            return Err(SbfError::ParseError("BaseStation too short".into()));
        }

        let base_station_id = u16::from_le_bytes([data[12], data[13]]);
        let base_type = data[14];
        let source = data[15];
        let datum = data[16];
        // Byte 17 is reserved.
        let x_m = f64::from_le_bytes(data[18..26].try_into().unwrap());
        let y_m = f64::from_le_bytes(data[26..34].try_into().unwrap());
        let z_m = f64::from_le_bytes(data[34..42].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            base_station_id,
            base_type,
            source,
            datum,
            x_m,
            y_m,
            z_m,
        })
    }
}

// ============================================================================
// PosCovCartesian Block
// ============================================================================

/// PosCovCartesian block (Block ID 5905)
///
/// Position covariance matrix in ECEF Cartesian coordinates.
#[derive(Debug, Clone)]
pub struct PosCovCartesianBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    /// X position variance (m^2)
    pub cov_xx: f32,
    /// Y position variance (m^2)
    pub cov_yy: f32,
    /// Z position variance (m^2)
    pub cov_zz: f32,
    /// Clock bias variance (m^2)
    pub cov_bb: f32,
    /// X-Y covariance
    pub cov_xy: f32,
    /// X-Z covariance
    pub cov_xz: f32,
    /// X-Bias covariance
    pub cov_xb: f32,
    /// Y-Z covariance
    pub cov_yz: f32,
    /// Y-Bias covariance
    pub cov_yb: f32,
    /// Z-Bias covariance
    pub cov_zb: f32,
}

impl PosCovCartesianBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    /// Get X position standard deviation in meters
    pub fn x_std_m(&self) -> Option<f32> {
        if self.cov_xx == F32_DNU || self.cov_xx < 0.0 {
            None
        } else {
            Some(self.cov_xx.sqrt())
        }
    }

    /// Get Y position standard deviation in meters
    pub fn y_std_m(&self) -> Option<f32> {
        if self.cov_yy == F32_DNU || self.cov_yy < 0.0 {
            None
        } else {
            Some(self.cov_yy.sqrt())
        }
    }

    /// Get Z position standard deviation in meters
    pub fn z_std_m(&self) -> Option<f32> {
        if self.cov_zz == F32_DNU || self.cov_zz < 0.0 {
            None
        } else {
            Some(self.cov_zz.sqrt())
        }
    }

    /// Get clock bias standard deviation in meters
    pub fn clock_std_m(&self) -> Option<f32> {
        if self.cov_bb == F32_DNU || self.cov_bb < 0.0 {
            None
        } else {
            Some(self.cov_bb.sqrt())
        }
    }
}

// ============================================================================
// PVTSupport Block
// ============================================================================

/// PVTSupport block (Block ID 4076)
///
/// Internal PVT support parameters. Per SBF spec, contains TOW and WNc.
/// Full payload layout is not in public domain; this parses the common header.
#[derive(Debug, Clone)]
pub struct PvtSupportBlock {
    tow_ms: u32,
    wnc: u16,
}

impl PvtSupportBlock {
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

impl SbfBlockParse for PvtSupportBlock {
    const BLOCK_ID: u16 = block_ids::PVT_SUPPORT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 12 {
            return Err(SbfError::ParseError("PVTSupport too short".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
        })
    }
}

/// PVTSupportA block (Block ID 4079).
///
/// The public reference guide does not document the payload layout. This preserves the raw
/// payload bytes after the time header.
#[derive(Debug, Clone)]
pub struct PvtSupportABlock {
    tow_ms: u32,
    wnc: u16,
    payload: Vec<u8>,
}

impl PvtSupportABlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }
}

impl SbfBlockParse for PvtSupportABlock {
    const BLOCK_ID: u16 = block_ids::PVT_SUPPORT_A;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 12 || data.len() < data_len {
            return Err(SbfError::ParseError("PVTSupportA too short".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            payload: data[12..data_len].to_vec(),
        })
    }
}

impl SbfBlockParse for PosCovCartesianBlock {
    const BLOCK_ID: u16 = block_ids::POS_COV_CARTESIAN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 54 {
            return Err(SbfError::ParseError("PosCovCartesian too short".into()));
        }

        let mode = data[12];
        let error = data[13];

        let cov_xx = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_yy = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_zz = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_bb = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_xy = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_xz = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let cov_xb = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let cov_yz = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let cov_yb = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let cov_zb = f32::from_le_bytes(data[50..54].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_xx,
            cov_yy,
            cov_zz,
            cov_bb,
            cov_xy,
            cov_xz,
            cov_xb,
            cov_yz,
            cov_yb,
            cov_zb,
        })
    }
}

// ============================================================================
// PosCovGeodetic Block
// ============================================================================

/// PosCovGeodetic block (Block ID 5906)
///
/// Position covariance matrix in geodetic coordinates.
#[derive(Debug, Clone)]
pub struct PosCovGeodeticBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    /// Latitude variance (m^2)
    pub cov_lat_lat: f32,
    /// Longitude variance (m^2)
    pub cov_lon_lon: f32,
    /// Height variance (m^2)
    pub cov_h_h: f32,
    /// Clock bias variance (m^2)
    pub cov_b_b: f32,
    /// Lat-Lon covariance
    pub cov_lat_lon: f32,
    /// Lat-Height covariance
    pub cov_lat_h: f32,
    /// Lat-Bias covariance
    pub cov_lat_b: f32,
    /// Lon-Height covariance
    pub cov_lon_h: f32,
    /// Lon-Bias covariance
    pub cov_lon_b: f32,
    /// Height-Bias covariance
    pub cov_h_b: f32,
}

impl PosCovGeodeticBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    /// Get latitude standard deviation in meters
    pub fn lat_std_m(&self) -> Option<f32> {
        if self.cov_lat_lat == F32_DNU || self.cov_lat_lat < 0.0 {
            None
        } else {
            Some(self.cov_lat_lat.sqrt())
        }
    }

    /// Get longitude standard deviation in meters
    pub fn lon_std_m(&self) -> Option<f32> {
        if self.cov_lon_lon == F32_DNU || self.cov_lon_lon < 0.0 {
            None
        } else {
            Some(self.cov_lon_lon.sqrt())
        }
    }

    /// Get height standard deviation in meters
    pub fn height_std_m(&self) -> Option<f32> {
        if self.cov_h_h == F32_DNU || self.cov_h_h < 0.0 {
            None
        } else {
            Some(self.cov_h_h.sqrt())
        }
    }
}

impl SbfBlockParse for PosCovGeodeticBlock {
    const BLOCK_ID: u16 = block_ids::POS_COV_GEODETIC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 54 {
            return Err(SbfError::ParseError("PosCovGeodetic too short".into()));
        }

        let mode = data[12];
        let error = data[13];

        let cov_lat_lat = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_lon_lon = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_h_h = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_b_b = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_lat_lon = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_lat_h = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let cov_lat_b = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let cov_lon_h = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let cov_lon_b = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let cov_h_b = f32::from_le_bytes(data[50..54].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_lat_lat,
            cov_lon_lon,
            cov_h_h,
            cov_b_b,
            cov_lat_lon,
            cov_lat_h,
            cov_lat_b,
            cov_lon_h,
            cov_lon_b,
            cov_h_b,
        })
    }
}

// ============================================================================
// VelCovGeodetic Block
// ============================================================================

/// VelCovGeodetic block (Block ID 5908)
///
/// Velocity covariance matrix in geodetic coordinates.
#[derive(Debug, Clone)]
pub struct VelCovGeodeticBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    /// North velocity variance (m^2/s^2)
    pub cov_vn_vn: f32,
    /// East velocity variance (m^2/s^2)
    pub cov_ve_ve: f32,
    /// Up velocity variance (m^2/s^2)
    pub cov_vu_vu: f32,
    /// Clock drift variance
    pub cov_dt_dt: f32,
    /// Vn-Ve covariance
    pub cov_vn_ve: f32,
    /// Vn-Vu covariance
    pub cov_vn_vu: f32,
    /// Vn-Dt covariance
    pub cov_vn_dt: f32,
    /// Ve-Vu covariance
    pub cov_ve_vu: f32,
    /// Ve-Dt covariance
    pub cov_ve_dt: f32,
    /// Vu-Dt covariance
    pub cov_vu_dt: f32,
}

impl VelCovGeodeticBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    /// Get north velocity standard deviation in m/s
    pub fn vn_std_mps(&self) -> Option<f32> {
        if self.cov_vn_vn == F32_DNU || self.cov_vn_vn < 0.0 {
            None
        } else {
            Some(self.cov_vn_vn.sqrt())
        }
    }

    /// Get east velocity standard deviation in m/s
    pub fn ve_std_mps(&self) -> Option<f32> {
        if self.cov_ve_ve == F32_DNU || self.cov_ve_ve < 0.0 {
            None
        } else {
            Some(self.cov_ve_ve.sqrt())
        }
    }

    /// Get up velocity standard deviation in m/s
    pub fn vu_std_mps(&self) -> Option<f32> {
        if self.cov_vu_vu == F32_DNU || self.cov_vu_vu < 0.0 {
            None
        } else {
            Some(self.cov_vu_vu.sqrt())
        }
    }
}

impl SbfBlockParse for VelCovGeodeticBlock {
    const BLOCK_ID: u16 = block_ids::VEL_COV_GEODETIC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 54 {
            return Err(SbfError::ParseError("VelCovGeodetic too short".into()));
        }

        let mode = data[12];
        let error = data[13];

        let cov_vn_vn = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_ve_ve = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_vu_vu = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_dt_dt = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_vn_ve = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_vn_vu = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let cov_vn_dt = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let cov_ve_vu = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let cov_ve_dt = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let cov_vu_dt = f32::from_le_bytes(data[50..54].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_vn_vn,
            cov_ve_ve,
            cov_vu_vu,
            cov_dt_dt,
            cov_vn_ve,
            cov_vn_vu,
            cov_vn_dt,
            cov_ve_vu,
            cov_ve_dt,
            cov_vu_dt,
        })
    }
}

// ============================================================================
// VelCovCartesian Block
// ============================================================================

/// VelCovCartesian block (Block ID 5907)
///
/// Velocity covariance matrix in ECEF Cartesian coordinates.
#[derive(Debug, Clone)]
pub struct VelCovCartesianBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    /// X velocity variance (m^2/s^2)
    pub cov_vx_vx: f32,
    /// Y velocity variance (m^2/s^2)
    pub cov_vy_vy: f32,
    /// Z velocity variance (m^2/s^2)
    pub cov_vz_vz: f32,
    /// Clock drift variance
    pub cov_dt_dt: f32,
    /// Vx-Vy covariance
    pub cov_vx_vy: f32,
    /// Vx-Vz covariance
    pub cov_vx_vz: f32,
    /// Vx-Dt covariance
    pub cov_vx_dt: f32,
    /// Vy-Vz covariance
    pub cov_vy_vz: f32,
    /// Vy-Dt covariance
    pub cov_vy_dt: f32,
    /// Vz-Dt covariance
    pub cov_vz_dt: f32,
}

impl VelCovCartesianBlock {
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
    pub fn error(&self) -> PvtError {
        PvtError::from_error_byte(self.error)
    }

    /// Get X velocity standard deviation in m/s
    pub fn vx_std_mps(&self) -> Option<f32> {
        if self.cov_vx_vx == F32_DNU || self.cov_vx_vx < 0.0 {
            None
        } else {
            Some(self.cov_vx_vx.sqrt())
        }
    }

    /// Get Y velocity standard deviation in m/s
    pub fn vy_std_mps(&self) -> Option<f32> {
        if self.cov_vy_vy == F32_DNU || self.cov_vy_vy < 0.0 {
            None
        } else {
            Some(self.cov_vy_vy.sqrt())
        }
    }

    /// Get Z velocity standard deviation in m/s
    pub fn vz_std_mps(&self) -> Option<f32> {
        if self.cov_vz_vz == F32_DNU || self.cov_vz_vz < 0.0 {
            None
        } else {
            Some(self.cov_vz_vz.sqrt())
        }
    }

    /// Get clock drift standard deviation
    pub fn clock_drift_std(&self) -> Option<f32> {
        if self.cov_dt_dt == F32_DNU || self.cov_dt_dt < 0.0 {
            None
        } else {
            Some(self.cov_dt_dt.sqrt())
        }
    }
}

impl SbfBlockParse for VelCovCartesianBlock {
    const BLOCK_ID: u16 = block_ids::VEL_COV_CARTESIAN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 54 {
            return Err(SbfError::ParseError("VelCovCartesian too short".into()));
        }

        let mode = data[12];
        let error = data[13];

        let cov_vx_vx = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let cov_vy_vy = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let cov_vz_vz = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let cov_dt_dt = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let cov_vx_vy = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let cov_vx_vz = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let cov_vx_dt = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let cov_vy_vz = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let cov_vy_dt = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let cov_vz_dt = f32::from_le_bytes(data[50..54].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode,
            error,
            cov_vx_vx,
            cov_vy_vy,
            cov_vz_vz,
            cov_dt_dt,
            cov_vx_vy,
            cov_vx_vz,
            cov_vx_dt,
            cov_vy_vz,
            cov_vy_dt,
            cov_vz_dt,
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
    fn test_pvt_sat_cartesian_accessors() {
        let sat = PvtSatCartesianSatPos {
            svid: 12,
            freq_nr: 1,
            iode: 22,
            x_m: 10.0,
            y_m: 20.0,
            z_m: 30.0,
            vx_mps: 1.0,
            vy_mps: 2.0,
            vz_mps: 3.0,
        };
        let block = PvtSatCartesianBlock {
            tow_ms: 2500,
            wnc: 2345,
            satellites: vec![sat],
        };

        assert!((block.tow_seconds() - 2.5).abs() < 1e-6);
        assert_eq!(block.num_satellites(), 1);
        let sat = &block.satellites[0];
        assert!((sat.x_m().unwrap() - 10.0).abs() < 1e-6);
        assert!((sat.vz_mps().unwrap() - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_pvt_sat_cartesian_dnu_handling() {
        let sat = PvtSatCartesianSatPos {
            svid: 1,
            freq_nr: 0,
            iode: 0,
            x_m: F64_DNU,
            y_m: 1.0,
            z_m: F64_DNU,
            vx_mps: F32_DNU,
            vy_mps: 0.5,
            vz_mps: F32_DNU,
        };

        assert!(sat.x_m().is_none());
        assert!(sat.y_m().is_some());
        assert!(sat.z_m().is_none());
        assert!(sat.vx_mps().is_none());
        assert!(sat.vy_mps().is_some());
        assert!(sat.vz_mps().is_none());
    }

    #[test]
    fn test_pvt_sat_cartesian_parse() {
        let mut data = vec![0u8; 14 + 40];
        data[12] = 1;
        data[13] = 40;

        let offset = 14;
        let iode = 512_u16;
        let x = 11.0_f64;
        let y = 22.0_f64;
        let z = 33.0_f64;
        let vx = 0.1_f32;
        let vy = 0.2_f32;
        let vz = 0.3_f32;

        data[offset] = 31;
        data[offset + 1] = 2;
        data[offset + 2..offset + 4].copy_from_slice(&iode.to_le_bytes());
        data[offset + 4..offset + 12].copy_from_slice(&x.to_le_bytes());
        data[offset + 12..offset + 20].copy_from_slice(&y.to_le_bytes());
        data[offset + 20..offset + 28].copy_from_slice(&z.to_le_bytes());
        data[offset + 28..offset + 32].copy_from_slice(&vx.to_le_bytes());
        data[offset + 32..offset + 36].copy_from_slice(&vy.to_le_bytes());
        data[offset + 36..offset + 40].copy_from_slice(&vz.to_le_bytes());

        let header = header_for(block_ids::PVT_SAT_CARTESIAN, data.len(), 123000, 2201);
        let block = PvtSatCartesianBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 123000);
        assert_eq!(block.wnc(), 2201);
        assert_eq!(block.num_satellites(), 1);
        let sat = &block.satellites[0];
        assert_eq!(sat.svid, 31);
        assert_eq!(sat.iode, iode);
        assert!((sat.x_m().unwrap() - x).abs() < 1e-6);
        assert!((sat.vy_mps().unwrap() - vy).abs() < 1e-6);
    }

    #[test]
    fn test_pvt_residuals_v2_accessors() {
        let residual = PvtResidualsV2ResidualInfo {
            e_i_m: 0.25,
            w_i_raw: 120,
            mdb_raw: 42,
        };
        let sat = PvtResidualsV2SatSignalInfo {
            svid: 7,
            freq_nr: 1,
            signal_type: 17,
            ref_svid: 33,
            ref_freq_nr: 0,
            meas_info: (1 << 2) | (1 << 4),
            iode: 300,
            corr_age_raw: 150,
            reference_id: 12,
            residuals: vec![residual],
        };
        let block = PvtResidualsV2Block {
            tow_ms: 2000,
            wnc: 100,
            sat_signal_info: vec![sat],
        };

        assert!((block.tow_seconds() - 2.0).abs() < 1e-6);
        assert_eq!(block.num_sat_signals(), 1);
        let sat = &block.sat_signal_info[0];
        assert!((sat.corr_age_seconds().unwrap() - 1.5).abs() < 1e-6);
        assert_eq!(sat.expected_residual_count(), 2);
        assert!((sat.residuals[0].residual_m().unwrap() - 0.25).abs() < 1e-6);
        assert_eq!(sat.residuals[0].weight().unwrap(), 120);
    }

    #[test]
    fn test_pvt_residuals_v2_dnu_handling() {
        let residual = PvtResidualsV2ResidualInfo {
            e_i_m: F32_DNU,
            w_i_raw: U16_DNU,
            mdb_raw: U16_DNU,
        };
        let sat = PvtResidualsV2SatSignalInfo {
            svid: 0,
            freq_nr: 0,
            signal_type: 0,
            ref_svid: 0,
            ref_freq_nr: 0,
            meas_info: 0,
            iode: 0,
            corr_age_raw: U16_DNU,
            reference_id: 0,
            residuals: vec![residual],
        };

        assert!(sat.corr_age_seconds().is_none());
        assert!(sat.residuals[0].residual_m().is_none());
        assert!(sat.residuals[0].weight().is_none());
        assert!(sat.residuals[0].mdb().is_none());
    }

    #[test]
    fn test_pvt_residuals_v2_parse() {
        let mut data = vec![0u8; 15 + 12 + (2 * 8)];
        data[12] = 1; // N
        data[13] = 12; // SB1Length
        data[14] = 8; // SB2Length

        // One SatSignalInfo with two residual entries (MeasInfo bits 2 and 4).
        let mut offset = 15;
        data[offset] = 8; // SVID
        data[offset + 1] = 1; // FreqNr
        data[offset + 2] = 17; // Type
        data[offset + 3] = 33; // RefSVID
        data[offset + 4] = 2; // RefFreqNr
        data[offset + 5] = (1 << 2) | (1 << 4); // MeasInfo
        data[offset + 6..offset + 8].copy_from_slice(&0x1234_u16.to_le_bytes()); // IODE
        data[offset + 8..offset + 10].copy_from_slice(&250_u16.to_le_bytes()); // CorrAge
        data[offset + 10..offset + 12].copy_from_slice(&77_u16.to_le_bytes()); // ReferenceID
        offset += 12;

        let e1 = 0.5_f32;
        let e2 = -0.25_f32;
        data[offset..offset + 4].copy_from_slice(&e1.to_le_bytes());
        data[offset + 4..offset + 6].copy_from_slice(&100_u16.to_le_bytes());
        data[offset + 6..offset + 8].copy_from_slice(&200_u16.to_le_bytes());
        offset += 8;

        data[offset..offset + 4].copy_from_slice(&e2.to_le_bytes());
        data[offset + 4..offset + 6].copy_from_slice(&101_u16.to_le_bytes());
        data[offset + 6..offset + 8].copy_from_slice(&201_u16.to_le_bytes());

        let header = header_for(block_ids::PVT_RESIDUALS_V2, data.len(), 654321, 2222);
        let block = PvtResidualsV2Block::parse(&header, &data).unwrap();

        assert_eq!(block.num_sat_signals(), 1);
        let sat = &block.sat_signal_info[0];
        assert_eq!(sat.svid, 8);
        assert_eq!(sat.expected_residual_count(), 2);
        assert_eq!(sat.residuals.len(), 2);
        assert!((sat.corr_age_seconds().unwrap() - 2.5).abs() < 1e-6);
        assert!((sat.residuals[0].residual_m().unwrap() - e1).abs() < 1e-6);
        assert!((sat.residuals[1].residual_m().unwrap() - e2).abs() < 1e-6);
    }

    #[test]
    fn test_raim_statistics_v2_accessors() {
        let block = RaimStatisticsV2Block {
            tow_ms: 3000,
            wnc: 123,
            integrity_flag: 2,
            herl_position_m: 5.0,
            verl_position_m: 6.0,
            herl_velocity_mps: 0.7,
            verl_velocity_mps: 0.8,
            overall_model: 42,
        };

        assert!((block.tow_seconds() - 3.0).abs() < 1e-6);
        assert_eq!(block.integrity_flag, 2);
        assert!((block.herl_position_m().unwrap() - 5.0).abs() < 1e-6);
        assert!((block.verl_velocity_mps().unwrap() - 0.8).abs() < 1e-6);
        assert_eq!(block.overall_model().unwrap(), 42);
    }

    #[test]
    fn test_raim_statistics_v2_dnu_handling() {
        let block = RaimStatisticsV2Block {
            tow_ms: 0,
            wnc: 0,
            integrity_flag: 0,
            herl_position_m: F32_DNU,
            verl_position_m: F32_DNU,
            herl_velocity_mps: F32_DNU,
            verl_velocity_mps: F32_DNU,
            overall_model: U16_DNU,
        };

        assert!(block.herl_position_m().is_none());
        assert!(block.verl_position_m().is_none());
        assert!(block.herl_velocity_mps().is_none());
        assert!(block.verl_velocity_mps().is_none());
        assert!(block.overall_model().is_none());
    }

    #[test]
    fn test_raim_statistics_v2_parse() {
        let mut data = vec![0u8; 34];
        data[12] = 3; // IntegrityFlag
        data[13] = 0; // Reserved

        let herl_position = 7.5_f32;
        let verl_position = 8.5_f32;
        let herl_velocity = 0.9_f32;
        let verl_velocity = 1.1_f32;
        let overall_model = 321_u16;

        data[14..18].copy_from_slice(&herl_position.to_le_bytes());
        data[18..22].copy_from_slice(&verl_position.to_le_bytes());
        data[22..26].copy_from_slice(&herl_velocity.to_le_bytes());
        data[26..30].copy_from_slice(&verl_velocity.to_le_bytes());
        data[30..32].copy_from_slice(&overall_model.to_le_bytes());

        let header = header_for(block_ids::RAIM_STATISTICS_V2, data.len(), 111222, 3333);
        let block = RaimStatisticsV2Block::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 111222);
        assert_eq!(block.wnc(), 3333);
        assert_eq!(block.integrity_flag, 3);
        assert!((block.herl_position_m().unwrap() - herl_position).abs() < 1e-6);
        assert!((block.verl_position_m().unwrap() - verl_position).abs() < 1e-6);
        assert!((block.herl_velocity_mps().unwrap() - herl_velocity).abs() < 1e-6);
        assert!((block.verl_velocity_mps().unwrap() - verl_velocity).abs() < 1e-6);
        assert_eq!(block.overall_model().unwrap(), overall_model);
    }

    #[test]
    fn test_dop_scaling() {
        let dop = DopBlock {
            tow_ms: 0,
            wnc: 0,
            nr_sv: 10,
            pdop_raw: 150, // 1.50
            tdop_raw: 100, // 1.00
            hdop_raw: 120, // 1.20
            vdop_raw: 200, // 2.00
            hpl_m: F32_DNU,
            vpl_m: F32_DNU,
        };

        assert!((dop.pdop() - 1.50).abs() < 0.001);
        assert!((dop.hdop() - 1.20).abs() < 0.001);
        assert!((dop.gdop() - (1.50_f32.powi(2) + 1.0_f32.powi(2)).sqrt()).abs() < 0.001);
    }

    #[test]
    fn test_pos_cov_cartesian_std_accessors() {
        // Variance of 4.0 m^2 should give std dev of 2.0 m
        let block = PosCovCartesianBlock {
            tow_ms: 100000,
            wnc: 2300,
            mode: 4, // RTK Fixed
            error: 0,
            cov_xx: 4.0,
            cov_yy: 9.0,
            cov_zz: 16.0,
            cov_bb: 25.0,
            cov_xy: 1.0,
            cov_xz: 2.0,
            cov_xb: 3.0,
            cov_yz: 4.0,
            cov_yb: 5.0,
            cov_zb: 6.0,
        };

        assert!((block.x_std_m().unwrap() - 2.0).abs() < 0.001);
        assert!((block.y_std_m().unwrap() - 3.0).abs() < 0.001);
        assert!((block.z_std_m().unwrap() - 4.0).abs() < 0.001);
        assert!((block.clock_std_m().unwrap() - 5.0).abs() < 0.001);
        assert!((block.tow_seconds() - 100.0).abs() < 0.001);
    }

    #[test]
    fn test_pos_cov_cartesian_dnu_handling() {
        let block = PosCovCartesianBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 0,
            error: 0,
            cov_xx: F32_DNU,
            cov_yy: -1.0, // negative variance
            cov_zz: 4.0,
            cov_bb: F32_DNU,
            cov_xy: 0.0,
            cov_xz: 0.0,
            cov_xb: 0.0,
            cov_yz: 0.0,
            cov_yb: 0.0,
            cov_zb: 0.0,
        };

        assert!(block.x_std_m().is_none()); // DNU
        assert!(block.y_std_m().is_none()); // negative
        assert!(block.z_std_m().is_some()); // valid
        assert!(block.clock_std_m().is_none()); // DNU
    }

    #[test]
    fn test_pos_cov_cartesian_parse() {
        // Build synthetic block data
        // Format: CRC(2) + ID(2) + Length(2) + TOW(4) + WNc(2) + Mode(1) + Error(1) + 10×f32
        let mut data = vec![0u8; 54];

        // Skip CRC (0-1), ID (2-3), Length (4-5)
        // TOW at offset 6-9 (already handled by header)
        // WNc at offset 10-11 (already handled by header)
        // Mode at offset 12
        data[12] = 4; // RTK Fixed
                      // Error at offset 13
        data[13] = 0;

        // Covariance values starting at offset 14
        let cov_xx: f32 = 1.0;
        let cov_yy: f32 = 4.0;
        let cov_zz: f32 = 9.0;
        let cov_bb: f32 = 16.0;
        let cov_xy: f32 = 0.5;
        let cov_xz: f32 = 0.6;
        let cov_xb: f32 = 0.7;
        let cov_yz: f32 = 0.8;
        let cov_yb: f32 = 0.9;
        let cov_zb: f32 = 1.1;

        data[14..18].copy_from_slice(&cov_xx.to_le_bytes());
        data[18..22].copy_from_slice(&cov_yy.to_le_bytes());
        data[22..26].copy_from_slice(&cov_zz.to_le_bytes());
        data[26..30].copy_from_slice(&cov_bb.to_le_bytes());
        data[30..34].copy_from_slice(&cov_xy.to_le_bytes());
        data[34..38].copy_from_slice(&cov_xz.to_le_bytes());
        data[38..42].copy_from_slice(&cov_xb.to_le_bytes());
        data[42..46].copy_from_slice(&cov_yz.to_le_bytes());
        data[46..50].copy_from_slice(&cov_yb.to_le_bytes());
        data[50..54].copy_from_slice(&cov_zb.to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::POS_COV_CARTESIAN,
            block_rev: 0,
            length: 56, // 2 sync + 54 data
            tow_ms: 123456,
            wnc: 2300,
        };

        let block = PosCovCartesianBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 123456);
        assert_eq!(block.wnc(), 2300);
        assert!((block.x_std_m().unwrap() - 1.0).abs() < 0.001);
        assert!((block.y_std_m().unwrap() - 2.0).abs() < 0.001);
        assert!((block.z_std_m().unwrap() - 3.0).abs() < 0.001);
        assert!((block.clock_std_m().unwrap() - 4.0).abs() < 0.001);
        assert!((block.cov_xy - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_vel_cov_cartesian_std_accessors() {
        // Variance of 0.04 m²/s² should give std dev of 0.2 m/s
        let block = VelCovCartesianBlock {
            tow_ms: 200000,
            wnc: 2300,
            mode: 4,
            error: 0,
            cov_vx_vx: 0.04,
            cov_vy_vy: 0.09,
            cov_vz_vz: 0.16,
            cov_dt_dt: 0.25,
            cov_vx_vy: 0.01,
            cov_vx_vz: 0.02,
            cov_vx_dt: 0.03,
            cov_vy_vz: 0.04,
            cov_vy_dt: 0.05,
            cov_vz_dt: 0.06,
        };

        assert!((block.vx_std_mps().unwrap() - 0.2).abs() < 0.001);
        assert!((block.vy_std_mps().unwrap() - 0.3).abs() < 0.001);
        assert!((block.vz_std_mps().unwrap() - 0.4).abs() < 0.001);
        assert!((block.clock_drift_std().unwrap() - 0.5).abs() < 0.001);
        assert!((block.tow_seconds() - 200.0).abs() < 0.001);
    }

    #[test]
    fn test_vel_cov_cartesian_dnu_handling() {
        let block = VelCovCartesianBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 0,
            error: 0,
            cov_vx_vx: F32_DNU,
            cov_vy_vy: -0.01, // negative variance
            cov_vz_vz: 0.04,
            cov_dt_dt: F32_DNU,
            cov_vx_vy: 0.0,
            cov_vx_vz: 0.0,
            cov_vx_dt: 0.0,
            cov_vy_vz: 0.0,
            cov_vy_dt: 0.0,
            cov_vz_dt: 0.0,
        };

        assert!(block.vx_std_mps().is_none()); // DNU
        assert!(block.vy_std_mps().is_none()); // negative
        assert!(block.vz_std_mps().is_some()); // valid
        assert!(block.clock_drift_std().is_none()); // DNU
    }

    #[test]
    fn test_vel_cov_cartesian_parse() {
        // Build synthetic block data
        let mut data = vec![0u8; 54];

        data[12] = 4; // Mode: RTK Fixed
        data[13] = 0; // Error: none

        let cov_vx_vx: f32 = 0.01;
        let cov_vy_vy: f32 = 0.04;
        let cov_vz_vz: f32 = 0.09;
        let cov_dt_dt: f32 = 0.16;
        let cov_vx_vy: f32 = 0.001;
        let cov_vx_vz: f32 = 0.002;
        let cov_vx_dt: f32 = 0.003;
        let cov_vy_vz: f32 = 0.004;
        let cov_vy_dt: f32 = 0.005;
        let cov_vz_dt: f32 = 0.006;

        data[14..18].copy_from_slice(&cov_vx_vx.to_le_bytes());
        data[18..22].copy_from_slice(&cov_vy_vy.to_le_bytes());
        data[22..26].copy_from_slice(&cov_vz_vz.to_le_bytes());
        data[26..30].copy_from_slice(&cov_dt_dt.to_le_bytes());
        data[30..34].copy_from_slice(&cov_vx_vy.to_le_bytes());
        data[34..38].copy_from_slice(&cov_vx_vz.to_le_bytes());
        data[38..42].copy_from_slice(&cov_vx_dt.to_le_bytes());
        data[42..46].copy_from_slice(&cov_vy_vz.to_le_bytes());
        data[46..50].copy_from_slice(&cov_vy_dt.to_le_bytes());
        data[50..54].copy_from_slice(&cov_vz_dt.to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::VEL_COV_CARTESIAN,
            block_rev: 0,
            length: 56,
            tow_ms: 345678,
            wnc: 2301,
        };

        let block = VelCovCartesianBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 345678);
        assert_eq!(block.wnc(), 2301);
        assert!((block.vx_std_mps().unwrap() - 0.1).abs() < 0.001);
        assert!((block.vy_std_mps().unwrap() - 0.2).abs() < 0.001);
        assert!((block.vz_std_mps().unwrap() - 0.3).abs() < 0.001);
        assert!((block.clock_drift_std().unwrap() - 0.4).abs() < 0.001);
        assert!((block.cov_vx_vy - 0.001).abs() < 0.0001);
    }

    #[test]
    fn test_pos_cart_scaled_accessors() {
        let block = PosCartBlock {
            tow_ms: 5000,
            wnc: 2000,
            mode: 4,
            error: 0,
            x_m: 10.0,
            y_m: 20.0,
            z_m: 30.0,
            base_x_m: 1.0,
            base_y_m: 2.0,
            base_z_m: 3.0,
            cov_xx: 4.0,
            cov_yy: 9.0,
            cov_zz: 16.0,
            cov_xy: 0.0,
            cov_xz: 0.0,
            cov_yz: 0.0,
            pdop_raw: 200,
            hdop_raw: 150,
            vdop_raw: 250,
            misc: 0,
            alert_flag: 0,
            datum: 0,
            nr_sv: 12,
            wa_corr_info: 1,
            reference_id: 10,
            mean_corr_age_raw: 150,
            signal_info: 0,
        };

        assert!((block.pdop().unwrap() - 2.0).abs() < 1e-6);
        assert!((block.hdop().unwrap() - 1.5).abs() < 1e-6);
        assert!((block.vdop().unwrap() - 2.5).abs() < 1e-6);
        assert!((block.mean_corr_age_seconds().unwrap() - 1.5).abs() < 1e-6);
        assert!((block.x_std_m().unwrap() - 2.0).abs() < 1e-6);
        assert!((block.tow_seconds() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_pos_cart_dnu_handling() {
        let block = PosCartBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 0,
            error: 0,
            x_m: F64_DNU,
            y_m: 1.0,
            z_m: 1.0,
            base_x_m: 1.0,
            base_y_m: 1.0,
            base_z_m: 1.0,
            cov_xx: F32_DNU,
            cov_yy: -1.0,
            cov_zz: 4.0,
            cov_xy: 0.0,
            cov_xz: 0.0,
            cov_yz: 0.0,
            pdop_raw: 0,
            hdop_raw: 100,
            vdop_raw: 0,
            misc: 0,
            alert_flag: 0,
            datum: 0,
            nr_sv: 0,
            wa_corr_info: 0,
            reference_id: 0,
            mean_corr_age_raw: U16_DNU,
            signal_info: 0,
        };

        assert!(block.x_m().is_none());
        assert!(block.x_std_m().is_none());
        assert!(block.y_std_m().is_none());
        assert!(block.vdop().is_none());
        assert!(block.mean_corr_age_seconds().is_none());
    }

    #[test]
    fn test_pos_cart_parse() {
        let mut data = vec![0u8; 106];
        data[12] = 8;
        data[13] = 0;
        data[14] = 0; // Reserved

        let x_m = 123.0_f64;
        let y_m = 456.0_f64;
        let z_m = 789.0_f64;
        let base_x = 10.0_f64;
        let base_y = 20.0_f64;
        let base_z = 30.0_f64;
        let cov_xx = 1.0_f32;
        let cov_yy = 4.0_f32;
        let cov_zz = 9.0_f32;
        let cov_xy = 0.1_f32;
        let cov_xz = 0.2_f32;
        let cov_yz = 0.3_f32;
        let pdop_raw = 250_u16;
        let hdop_raw = 150_u16;
        let vdop_raw = 350_u16;
        let mean_corr_age_raw = 120_u16;
        let signal_info = 0x12345678_u32;

        data[15..23].copy_from_slice(&x_m.to_le_bytes());
        data[23..31].copy_from_slice(&y_m.to_le_bytes());
        data[31..39].copy_from_slice(&z_m.to_le_bytes());
        data[39..47].copy_from_slice(&base_x.to_le_bytes());
        data[47..55].copy_from_slice(&base_y.to_le_bytes());
        data[55..63].copy_from_slice(&base_z.to_le_bytes());
        data[63..67].copy_from_slice(&cov_xx.to_le_bytes());
        data[67..71].copy_from_slice(&cov_yy.to_le_bytes());
        data[71..75].copy_from_slice(&cov_zz.to_le_bytes());
        data[75..79].copy_from_slice(&cov_xy.to_le_bytes());
        data[79..83].copy_from_slice(&cov_xz.to_le_bytes());
        data[83..87].copy_from_slice(&cov_yz.to_le_bytes());
        data[87..89].copy_from_slice(&pdop_raw.to_le_bytes());
        data[89..91].copy_from_slice(&hdop_raw.to_le_bytes());
        data[91..93].copy_from_slice(&vdop_raw.to_le_bytes());
        data[93] = 0;
        data[94] = 0;
        data[95] = 1;
        data[96] = 8;
        data[97] = 2;
        data[98..100].copy_from_slice(&55_u16.to_le_bytes());
        data[100..102].copy_from_slice(&mean_corr_age_raw.to_le_bytes());
        data[102..106].copy_from_slice(&signal_info.to_le_bytes());

        let header = header_for(block_ids::POS_CART, data.len(), 123456, 2222);
        let block = PosCartBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 123456);
        assert_eq!(block.wnc(), 2222);
        assert!((block.x_m().unwrap() - x_m).abs() < 1e-6);
        assert!((block.pdop().unwrap() - 2.5).abs() < 1e-6);
        assert_eq!(block.reference_id, 55);
        assert_eq!(block.signal_info, signal_info);
    }

    #[test]
    fn test_base_vector_cart_scaled_accessors() {
        let info = BaseVectorCartInfo {
            nr_sv: 12,
            error: 0,
            mode: 8,
            misc: 0,
            dx_m: 1.0,
            dy_m: 2.0,
            dz_m: 3.0,
            dvx_mps: 0.1,
            dvy_mps: 0.2,
            dvz_mps: 0.3,
            azimuth_raw: 12345,
            elevation_raw: 250,
            reference_id: 7,
            corr_age_raw: 150,
            signal_info: 0,
        };

        assert!((info.azimuth_deg().unwrap() - 123.45).abs() < 1e-2);
        assert!((info.elevation_deg().unwrap() - 2.5).abs() < 1e-6);
        assert!((info.corr_age_seconds().unwrap() - 1.5).abs() < 1e-6);
        assert!((info.dvx_mps().unwrap() - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_base_vector_cart_dnu_handling() {
        let info = BaseVectorCartInfo {
            nr_sv: 0,
            error: 0,
            mode: 0,
            misc: 0,
            dx_m: F64_DNU,
            dy_m: 1.0,
            dz_m: 1.0,
            dvx_mps: F32_DNU,
            dvy_mps: 0.0,
            dvz_mps: 0.0,
            azimuth_raw: U16_DNU,
            elevation_raw: I16_DNU,
            reference_id: 0,
            corr_age_raw: U16_DNU,
            signal_info: 0,
        };

        assert!(info.dx_m().is_none());
        assert!(info.dvx_mps().is_none());
        assert!(info.azimuth_deg().is_none());
        assert!(info.elevation_deg().is_none());
        assert!(info.corr_age_seconds().is_none());
    }

    #[test]
    fn test_base_vector_cart_parse() {
        let mut data = vec![0u8; 14 + 52];
        data[12] = 1;
        data[13] = 52;

        let offset = 14;
        data[offset] = 10;
        data[offset + 1] = 0;
        data[offset + 2] = 8;
        data[offset + 3] = 0;

        let dx_m = 1.5_f64;
        let dy_m = 2.5_f64;
        let dz_m = 3.5_f64;
        let dvx_mps = 0.25_f32;
        let dvy_mps = 0.5_f32;
        let dvz_mps = 0.75_f32;
        let azimuth_raw = 9000_u16;
        let elevation_raw = 450_i16;
        let reference_id = 22_u16;
        let corr_age_raw = 80_u16;
        let signal_info = 0x87654321_u32;

        data[offset + 4..offset + 12].copy_from_slice(&dx_m.to_le_bytes());
        data[offset + 12..offset + 20].copy_from_slice(&dy_m.to_le_bytes());
        data[offset + 20..offset + 28].copy_from_slice(&dz_m.to_le_bytes());
        data[offset + 28..offset + 32].copy_from_slice(&dvx_mps.to_le_bytes());
        data[offset + 32..offset + 36].copy_from_slice(&dvy_mps.to_le_bytes());
        data[offset + 36..offset + 40].copy_from_slice(&dvz_mps.to_le_bytes());
        data[offset + 40..offset + 42].copy_from_slice(&azimuth_raw.to_le_bytes());
        data[offset + 42..offset + 44].copy_from_slice(&elevation_raw.to_le_bytes());
        data[offset + 44..offset + 46].copy_from_slice(&reference_id.to_le_bytes());
        data[offset + 46..offset + 48].copy_from_slice(&corr_age_raw.to_le_bytes());
        data[offset + 48..offset + 52].copy_from_slice(&signal_info.to_le_bytes());

        let header = header_for(block_ids::BASE_VECTOR_CART, data.len(), 999, 7);
        let block = BaseVectorCartBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_vectors(), 1);
        let info = &block.vectors[0];
        assert_eq!(info.nr_sv, 10);
        assert_eq!(info.reference_id, reference_id);
        assert!((info.dx_m().unwrap() - dx_m).abs() < 1e-6);
        assert!((info.azimuth_deg().unwrap() - 90.0).abs() < 1e-6);
        assert!((info.corr_age_seconds().unwrap() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_base_vector_geod_scaled_accessors() {
        let info = BaseVectorGeodInfo {
            nr_sv: 8,
            error: 0,
            mode: 9,
            misc: 0,
            de_m: 1.0,
            dn_m: 2.0,
            du_m: 3.0,
            dve_mps: 0.1,
            dvn_mps: 0.2,
            dvu_mps: 0.3,
            azimuth_raw: 18000,
            elevation_raw: 100,
            reference_id: 9,
            corr_age_raw: 200,
            signal_info: 0,
        };

        assert!((info.azimuth_deg().unwrap() - 180.0).abs() < 1e-6);
        assert!((info.elevation_deg().unwrap() - 1.0).abs() < 1e-6);
        assert!((info.corr_age_seconds().unwrap() - 2.0).abs() < 1e-6);
        assert!((info.dvn_mps().unwrap() - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_base_vector_geod_dnu_handling() {
        let info = BaseVectorGeodInfo {
            nr_sv: 0,
            error: 0,
            mode: 0,
            misc: 0,
            de_m: F64_DNU,
            dn_m: 1.0,
            du_m: 1.0,
            dve_mps: F32_DNU,
            dvn_mps: 0.0,
            dvu_mps: 0.0,
            azimuth_raw: U16_DNU,
            elevation_raw: I16_DNU,
            reference_id: 0,
            corr_age_raw: U16_DNU,
            signal_info: 0,
        };

        assert!(info.de_m().is_none());
        assert!(info.dve_mps().is_none());
        assert!(info.azimuth_deg().is_none());
        assert!(info.elevation_deg().is_none());
        assert!(info.corr_age_seconds().is_none());
    }

    #[test]
    fn test_base_vector_geod_parse() {
        let mut data = vec![0u8; 14 + 52];
        data[12] = 1;
        data[13] = 52;

        let offset = 14;
        data[offset] = 6;
        data[offset + 1] = 0;
        data[offset + 2] = 8;
        data[offset + 3] = 0;

        let de_m = 4.5_f64;
        let dn_m = 5.5_f64;
        let du_m = 6.5_f64;
        let dve_mps = 0.15_f32;
        let dvn_mps = 0.25_f32;
        let dvu_mps = 0.35_f32;
        let azimuth_raw = 27000_u16;
        let elevation_raw = 300_i16;
        let reference_id = 33_u16;
        let corr_age_raw = 90_u16;
        let signal_info = 0x12340000_u32;

        data[offset + 4..offset + 12].copy_from_slice(&de_m.to_le_bytes());
        data[offset + 12..offset + 20].copy_from_slice(&dn_m.to_le_bytes());
        data[offset + 20..offset + 28].copy_from_slice(&du_m.to_le_bytes());
        data[offset + 28..offset + 32].copy_from_slice(&dve_mps.to_le_bytes());
        data[offset + 32..offset + 36].copy_from_slice(&dvn_mps.to_le_bytes());
        data[offset + 36..offset + 40].copy_from_slice(&dvu_mps.to_le_bytes());
        data[offset + 40..offset + 42].copy_from_slice(&azimuth_raw.to_le_bytes());
        data[offset + 42..offset + 44].copy_from_slice(&elevation_raw.to_le_bytes());
        data[offset + 44..offset + 46].copy_from_slice(&reference_id.to_le_bytes());
        data[offset + 46..offset + 48].copy_from_slice(&corr_age_raw.to_le_bytes());
        data[offset + 48..offset + 52].copy_from_slice(&signal_info.to_le_bytes());

        let header = header_for(block_ids::BASE_VECTOR_GEOD, data.len(), 777, 5);
        let block = BaseVectorGeodBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_vectors(), 1);
        let info = &block.vectors[0];
        assert_eq!(info.nr_sv, 6);
        assert_eq!(info.reference_id, reference_id);
        assert!((info.de_m().unwrap() - de_m).abs() < 1e-6);
        assert!((info.azimuth_deg().unwrap() - 270.0).abs() < 1e-6);
        assert!((info.corr_age_seconds().unwrap() - 0.9).abs() < 1e-6);
    }

    #[test]
    fn test_geo_corrections_accessors() {
        let corr = GeoCorrectionsSatCorr {
            svid: 131,
            iode: 5,
            prc_m: 2.5,
            corr_age_fc_s: 1.2,
            delta_x_m: 0.1,
            delta_y_m: 0.2,
            delta_z_m: 0.3,
            delta_clock_m: 0.01,
            corr_age_lt_s: 120.0,
            iono_pp_lat_rad: 0.5,
            iono_pp_lon_rad: -0.3,
            slant_iono_m: 0.8,
            corr_age_iono_s: 60.0,
            var_flt_m2: 0.25,
            var_uire_m2: 0.5,
            var_air_m2: 1.0,
            var_tropo_m2: 0.1,
        };
        let block = GeoCorrectionsBlock {
            tow_ms: 5000,
            wnc: 2100,
            sat_corrections: vec![corr],
        };

        assert!((block.tow_seconds() - 5.0).abs() < 1e-6);
        assert_eq!(block.num_satellites(), 1);
        let c = &block.sat_corrections[0];
        assert_eq!(c.svid, 131);
        assert!((c.prc_m().unwrap() - 2.5).abs() < 1e-6);
        assert!((c.corr_age_fc_seconds().unwrap() - 1.2).abs() < 1e-6);
        assert!((c.delta_x_m().unwrap() - 0.1).abs() < 1e-6);
        assert!((c.slant_iono_m().unwrap() - 0.8).abs() < 1e-6);
        assert!((c.var_flt_m2().unwrap() - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_geo_corrections_dnu_handling() {
        let corr = GeoCorrectionsSatCorr {
            svid: 0,
            iode: 0,
            prc_m: F32_DNU,
            corr_age_fc_s: F32_DNU,
            delta_x_m: 0.0,
            delta_y_m: F32_DNU,
            delta_z_m: 0.0,
            delta_clock_m: F32_DNU,
            corr_age_lt_s: F32_DNU,
            iono_pp_lat_rad: F32_DNU,
            iono_pp_lon_rad: 0.0,
            slant_iono_m: F32_DNU,
            corr_age_iono_s: F32_DNU,
            var_flt_m2: F32_DNU,
            var_uire_m2: -1.0,
            var_air_m2: 0.0,
            var_tropo_m2: F32_DNU,
        };

        assert!(corr.prc_m().is_none());
        assert!(corr.corr_age_fc_seconds().is_none());
        assert!(corr.delta_y_m().is_none());
        assert!(corr.var_flt_m2().is_none());
        assert!(corr.var_uire_m2().is_none());
    }

    #[test]
    fn test_geo_corrections_parse() {
        let sb_len = 62usize;
        let mut data = vec![0u8; 14 + sb_len];
        data[12] = 1;
        data[13] = sb_len as u8;

        let offset = 14;
        data[offset] = 132;
        data[offset + 1] = 7;
        let prc = 3.1_f32;
        let delta_x = 0.5_f32;
        data[offset + 2..offset + 6].copy_from_slice(&prc.to_le_bytes());
        data[offset + 10..offset + 14].copy_from_slice(&delta_x.to_le_bytes());

        let header = header_for(block_ids::GEO_CORRECTIONS, data.len(), 88888, 2200);
        let block = GeoCorrectionsBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 88888);
        assert_eq!(block.wnc(), 2200);
        assert_eq!(block.num_satellites(), 1);
        let c = &block.sat_corrections[0];
        assert_eq!(c.svid, 132);
        assert_eq!(c.iode, 7);
        assert!((c.prc_m().unwrap() - prc).abs() < 1e-6);
        assert!((c.delta_x_m().unwrap() - delta_x).abs() < 1e-6);
    }

    #[test]
    fn test_base_station_accessors() {
        let block = BaseStationBlock {
            tow_ms: 10000,
            wnc: 2300,
            base_station_id: 42,
            base_type: 1,
            source: 2,
            datum: 0,
            x_m: 4e6,
            y_m: 3e6,
            z_m: -5e6,
        };

        assert!((block.tow_seconds() - 10.0).abs() < 1e-6);
        assert_eq!(block.base_station_id, 42);
        assert!((block.x_m().unwrap() - 4e6).abs() < 1.0);
        assert!((block.y_m().unwrap() - 3e6).abs() < 1.0);
        assert!((block.z_m().unwrap() - (-5e6)).abs() < 1.0);
    }

    #[test]
    fn test_base_station_dnu_handling() {
        let block = BaseStationBlock {
            tow_ms: 0,
            wnc: 0,
            base_station_id: 0,
            base_type: 0,
            source: 0,
            datum: 0,
            x_m: F64_DNU,
            y_m: 1.0,
            z_m: F64_DNU,
        };

        assert!(block.x_m().is_none());
        assert!(block.y_m().is_some());
        assert!(block.z_m().is_none());
    }

    #[test]
    fn test_base_station_parse() {
        let mut data = vec![0u8; 42];
        let base_id = 100_u16;
        let x_m = 4.0e6_f64;
        let y_m = 3.0e6_f64;
        let z_m = -5.5e6_f64;

        data[12..14].copy_from_slice(&base_id.to_le_bytes());
        data[14] = 2;
        data[15] = 1;
        data[16] = 0;
        data[17] = 0; // Reserved
        data[18..26].copy_from_slice(&x_m.to_le_bytes());
        data[26..34].copy_from_slice(&y_m.to_le_bytes());
        data[34..42].copy_from_slice(&z_m.to_le_bytes());

        let header = header_for(block_ids::BASE_STATION, data.len(), 123456, 2345);
        let block = BaseStationBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 123456);
        assert_eq!(block.wnc(), 2345);
        assert_eq!(block.base_station_id, base_id);
        assert_eq!(block.base_type, 2);
        assert!((block.x_m().unwrap() - x_m).abs() < 0.01);
        assert!((block.y_m().unwrap() - y_m).abs() < 0.01);
        assert!((block.z_m().unwrap() - z_m).abs() < 0.01);
    }

    #[test]
    fn test_pvt_support_parse_and_accessors() {
        let data = vec![0u8; 12];
        let header = header_for(block_ids::PVT_SUPPORT, data.len(), 5000, 2100);
        let block = PvtSupportBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 5000);
        assert_eq!(block.wnc(), 2100);
        assert!((block.tow_seconds() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_pvt_support_too_short() {
        let data = vec![0u8; 8];
        let header = header_for(block_ids::PVT_SUPPORT, data.len(), 0, 0);
        let result = PvtSupportBlock::parse(&header, &data);
        assert!(result.is_err());
    }
}
