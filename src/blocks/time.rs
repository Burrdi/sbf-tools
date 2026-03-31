//! Time blocks (ReceiverTime, xPPSOffset, ExtEvent, ExtEventPVT, EndOfPVT)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::{PvtError, PvtMode};

use super::position::BaseVectorGeodBlock;
use super::block_ids;
use super::dnu::{f32_or_none, f64_or_none, U16_DNU};
use super::SbfBlockParse;

#[cfg(test)]
use super::dnu::{F32_DNU, F64_DNU};

// ============================================================================
// Constants
// ============================================================================

// ============================================================================
// ReceiverTime Block
// ============================================================================

/// ReceiverTime block (Block ID 5914)
///
/// UTC time from the receiver.
#[derive(Debug, Clone)]
pub struct ReceiverTimeBlock {
    tow_ms: u32,
    wnc: u16,
    /// UTC year
    pub utc_year: i16,
    /// UTC month (1-12)
    pub utc_month: u8,
    /// UTC day (1-31)
    pub utc_day: u8,
    /// UTC hour (0-23)
    pub utc_hour: u8,
    /// UTC minute (0-59)
    pub utc_minute: u8,
    /// UTC second (0-60, 60 for leap second)
    pub utc_second: u8,
    /// Leap seconds (TAI - UTC)
    pub delta_ls: i8,
    /// Synchronization level
    pub sync_level: u8,
}

impl ReceiverTimeBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get UTC time as a formatted string (ISO 8601)
    pub fn utc_string(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            self.utc_year,
            self.utc_month,
            self.utc_day,
            self.utc_hour,
            self.utc_minute,
            self.utc_second
        )
    }

    /// Check if time is valid
    pub fn is_valid(&self) -> bool {
        self.utc_year >= 2000
            && self.utc_month >= 1
            && self.utc_month <= 12
            && self.utc_day >= 1
            && self.utc_day <= 31
            && self.utc_hour <= 23
            && self.utc_minute <= 59
            && self.utc_second <= 60
    }

    /// Get sync level description
    pub fn sync_level_desc(&self) -> &'static str {
        match self.sync_level {
            0 => "Not synchronized",
            1 => "Approximate time",
            2 => "Coarse time",
            3 => "Fine time (PVT)",
            4 => "Fine time (PPS)",
            _ => "Unknown",
        }
    }

    /// Check if time is fully synchronized
    pub fn is_synchronized(&self) -> bool {
        self.sync_level >= 3
    }
}

impl SbfBlockParse for ReceiverTimeBlock {
    const BLOCK_ID: u16 = block_ids::RECEIVER_TIME;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 20 {
            return Err(SbfError::ParseError("ReceiverTime too short".into()));
        }

        // Offsets:
        // 12: UTCYear (i1, 2-digit year 0..99, or -128 if unavailable)
        // 13: UTCMonth
        // 14: UTCDay
        // 15: UTCHour
        // 16: UTCMin
        // 17: UTCSec
        // 18: DeltaLS
        // 19: SyncLevel

        let utc_year_raw = data[12] as i8;
        let utc_year = if utc_year_raw == i8::MIN {
            i8::MIN as i16
        } else {
            2000 + utc_year_raw as i16
        };
        let utc_month = data[13];
        let utc_day = data[14];
        let utc_hour = data[15];
        let utc_minute = data[16];
        let utc_second = data[17];
        let delta_ls = data[18] as i8;
        let sync_level = data[19];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            utc_year,
            utc_month,
            utc_day,
            utc_hour,
            utc_minute,
            utc_second,
            delta_ls,
            sync_level,
        })
    }
}

// ============================================================================
// xPPSOffset Block
// ============================================================================

/// xPPSOffset block (Block ID 5911)
///
/// PPS offset and synchronization details.
#[derive(Debug, Clone)]
pub struct PpsOffsetBlock {
    tow_ms: u32,
    wnc: u16,
    /// Age of synchronization (s)
    pub sync_age: u8,
    /// Time scale code
    pub timescale: u8,
    /// PPS offset (ns)
    offset_ns: f32,
}

impl PpsOffsetBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Offset in nanoseconds (None if DNU)
    pub fn offset_ns(&self) -> Option<f32> {
        f32_or_none(self.offset_ns)
    }

    /// Offset in seconds (None if DNU)
    pub fn offset_seconds(&self) -> Option<f64> {
        self.offset_ns().map(|value| value as f64 * 1e-9)
    }
}

impl SbfBlockParse for PpsOffsetBlock {
    const BLOCK_ID: u16 = block_ids::PPS_OFFSET;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 18 {
            return Err(SbfError::ParseError("xPPSOffset too short".into()));
        }

        let sync_age = data[12];
        let timescale = data[13];
        let offset_ns = f32::from_le_bytes(data[14..18].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            sync_age,
            timescale,
            offset_ns,
        })
    }
}

// ============================================================================
// ExtEvent Block
// ============================================================================

/// ExtEvent block (Block ID 5924)
///
/// External event timing information.
#[derive(Debug, Clone)]
pub struct ExtEventBlock {
    tow_ms: u32,
    wnc: u16,
    /// Event source
    pub source: u8,
    /// Edge polarity
    pub polarity: u8,
    /// Event offset from TOW in seconds (ExtEvent 5924 uses 1 s units).
    offset_s: f32,
    /// Clock bias in seconds (ExtEvent 5924 uses 1 s units).
    rx_clk_bias_s: f64,
    /// PVT solution age
    pub pvt_age: u16,
}

impl ExtEventBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Event offset in seconds (None if DNU)
    pub fn offset_seconds(&self) -> Option<f64> {
        f32_or_none(self.offset_s).map(|value| value as f64)
    }

    /// Event offset in nanoseconds (legacy convenience conversion)
    pub fn offset_ns(&self) -> Option<f32> {
        self.offset_seconds().map(|value| (value * 1e9) as f32)
    }

    /// Receiver clock bias in seconds (None if DNU)
    pub fn rx_clk_bias_seconds(&self) -> Option<f64> {
        f64_or_none(self.rx_clk_bias_s)
    }

    /// Receiver clock bias in ms (None if DNU)
    ///
    /// Kept for compatibility with existing ms-based callers.
    pub fn rx_clk_bias_ms(&self) -> Option<f64> {
        self.rx_clk_bias_seconds().map(|value| value * 1e3)
    }
}

impl SbfBlockParse for ExtEventBlock {
    const BLOCK_ID: u16 = block_ids::EXT_EVENT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 28 {
            return Err(SbfError::ParseError("ExtEvent too short".into()));
        }

        let source = data[12];
        let polarity = data[13];
        let offset_s = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let rx_clk_bias_s = f64::from_le_bytes(data[18..26].try_into().unwrap());
        let pvt_age = u16::from_le_bytes([data[26], data[27]]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            source,
            polarity,
            offset_s,
            rx_clk_bias_s,
            pvt_age,
        })
    }
}

// ============================================================================
// ExtEventPVT Blocks
// ============================================================================

/// ExtEventPVTCartesian block (Block ID 4037)
///
/// External event timing with PVT solution in ECEF coordinates.
#[derive(Debug, Clone)]
pub struct ExtEventPvtCartesianBlock {
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

impl ExtEventPvtCartesianBlock {
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
    pub fn has_fix(&self) -> bool {
        self.mode().has_fix() && self.error().is_ok()
    }

    // ECEF position
    pub fn x_m(&self) -> Option<f64> {
        f64_or_none(self.x_m)
    }
    pub fn y_m(&self) -> Option<f64> {
        f64_or_none(self.y_m)
    }
    pub fn z_m(&self) -> Option<f64> {
        f64_or_none(self.z_m)
    }
    pub fn undulation_m(&self) -> Option<f32> {
        f32_or_none(self.undulation_m)
    }

    // Velocity
    pub fn vx_mps(&self) -> Option<f32> {
        f32_or_none(self.vx_mps)
    }
    pub fn vy_mps(&self) -> Option<f32> {
        f32_or_none(self.vy_mps)
    }
    pub fn vz_mps(&self) -> Option<f32> {
        f32_or_none(self.vz_mps)
    }
    pub fn course_over_ground_deg(&self) -> Option<f32> {
        f32_or_none(self.cog_deg)
    }

    // Clock
    pub fn clock_bias_ms(&self) -> Option<f64> {
        f64_or_none(self.rx_clk_bias_ms)
    }
    pub fn clock_drift_ppm(&self) -> Option<f32> {
        f32_or_none(self.rx_clk_drift_ppm)
    }

    // Correction age
    pub fn mean_corr_age_seconds(&self) -> Option<f32> {
        if self.mean_corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.mean_corr_age_raw as f32 * 0.01)
        }
    }
    pub fn mean_corr_age_raw(&self) -> u16 {
        self.mean_corr_age_raw
    }

    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }
}

impl SbfBlockParse for ExtEventPvtCartesianBlock {
    const BLOCK_ID: u16 = block_ids::EXT_EVENT_PVT_CARTESIAN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 84 {
            return Err(SbfError::ParseError(
                "ExtEventPVTCartesian too short".into(),
            ));
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
        let nr_bases = data[83];

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

/// ExtEventPVTGeodetic block (Block ID 4038)
///
/// External event timing with PVT solution in geodetic coordinates.
#[derive(Debug, Clone)]
pub struct ExtEventPvtGeodeticBlock {
    tow_ms: u32,
    wnc: u16,
    mode: u8,
    error: u8,
    latitude_rad: f64,
    longitude_rad: f64,
    height_m: f64,
    undulation_m: f32,
    vn_mps: f32,
    ve_mps: f32,
    vu_mps: f32,
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

impl ExtEventPvtGeodeticBlock {
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
    pub fn has_fix(&self) -> bool {
        self.mode().has_fix() && self.error().is_ok()
    }

    // Position
    pub fn latitude_deg(&self) -> Option<f64> {
        f64_or_none(self.latitude_rad).map(|value| value.to_degrees())
    }
    pub fn longitude_deg(&self) -> Option<f64> {
        f64_or_none(self.longitude_rad).map(|value| value.to_degrees())
    }
    pub fn latitude_rad(&self) -> f64 {
        self.latitude_rad
    }
    pub fn longitude_rad(&self) -> f64 {
        self.longitude_rad
    }
    pub fn height_m(&self) -> Option<f64> {
        f64_or_none(self.height_m)
    }
    pub fn undulation_m(&self) -> Option<f32> {
        f32_or_none(self.undulation_m)
    }

    // Velocity
    pub fn velocity_north_mps(&self) -> Option<f32> {
        f32_or_none(self.vn_mps)
    }
    pub fn velocity_east_mps(&self) -> Option<f32> {
        f32_or_none(self.ve_mps)
    }
    pub fn velocity_up_mps(&self) -> Option<f32> {
        f32_or_none(self.vu_mps)
    }
    pub fn course_over_ground_deg(&self) -> Option<f32> {
        f32_or_none(self.cog_deg)
    }

    // Clock
    pub fn clock_bias_ms(&self) -> Option<f64> {
        f64_or_none(self.rx_clk_bias_ms)
    }
    pub fn clock_drift_ppm(&self) -> Option<f32> {
        f32_or_none(self.rx_clk_drift_ppm)
    }

    // Correction age
    pub fn mean_corr_age_seconds(&self) -> Option<f32> {
        if self.mean_corr_age_raw == U16_DNU {
            None
        } else {
            Some(self.mean_corr_age_raw as f32 * 0.01)
        }
    }
    pub fn mean_corr_age_raw(&self) -> u16 {
        self.mean_corr_age_raw
    }

    pub fn num_satellites(&self) -> u8 {
        self.nr_sv
    }
}

impl SbfBlockParse for ExtEventPvtGeodeticBlock {
    const BLOCK_ID: u16 = block_ids::EXT_EVENT_PVT_GEODETIC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 84 {
            return Err(SbfError::ParseError("ExtEventPVTGeodetic too short".into()));
        }

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
        let nr_bases = data[83];

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
        })
    }
}

// ============================================================================
// ExtEventBaseVectGeod / ExtEventAttEuler
// ============================================================================

/// ExtEventBaseVectGeod (4217) — base vectors at event time.
#[derive(Debug, Clone)]
pub struct ExtEventBaseVectGeodBlock(pub BaseVectorGeodBlock);

impl std::ops::Deref for ExtEventBaseVectGeodBlock {
    type Target = BaseVectorGeodBlock;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SbfBlockParse for ExtEventBaseVectGeodBlock {
    const BLOCK_ID: u16 = block_ids::EXT_EVENT_BASE_VECT_GEOD;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        Ok(Self(BaseVectorGeodBlock::parse(header, data)?))
    }
}

/// ExtEventAttEuler (4237) — attitude at event time.
#[derive(Debug, Clone)]
pub struct ExtEventAttEulerBlock {
    tow_ms: u32,
    wnc: u16,
    nr_sv: u8,
    error: u8,
    mode: u16,
    heading_deg: f32,
    pitch_deg: f32,
    roll_deg: f32,
    pitch_rate_dps: f32,
    roll_rate_dps: f32,
    heading_rate_dps: f32,
}

impl ExtEventAttEulerBlock {
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
    pub fn error_raw(&self) -> u8 {
        self.error
    }
    pub fn mode_raw(&self) -> u16 {
        self.mode
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

impl SbfBlockParse for ExtEventAttEulerBlock {
    const BLOCK_ID: u16 = block_ids::EXT_EVENT_ATT_EULER;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 42 {
            return Err(SbfError::ParseError("ExtEventAttEuler too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            nr_sv: data[12],
            error: data[13],
            mode: u16::from_le_bytes(data[14..16].try_into().unwrap()),
            heading_deg: f32::from_le_bytes(data[18..22].try_into().unwrap()),
            pitch_deg: f32::from_le_bytes(data[22..26].try_into().unwrap()),
            roll_deg: f32::from_le_bytes(data[26..30].try_into().unwrap()),
            pitch_rate_dps: f32::from_le_bytes(data[30..34].try_into().unwrap()),
            roll_rate_dps: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            heading_rate_dps: f32::from_le_bytes(data[38..42].try_into().unwrap()),
        })
    }
}

// ============================================================================
// EndOfPVT Block
// ============================================================================

/// EndOfPVT block (Block ID 5921)
///
/// Marker indicating end of PVT-related blocks for current epoch.
#[derive(Debug, Clone)]
pub struct EndOfPvtBlock {
    tow_ms: u32,
    wnc: u16,
}

impl EndOfPvtBlock {
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

impl SbfBlockParse for EndOfPvtBlock {
    const BLOCK_ID: u16 = block_ids::END_OF_PVT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Minimal block - just TOW and WNc from header
        if data.len() < 12 {
            return Err(SbfError::ParseError("EndOfPVT too short".into()));
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
    fn test_receiver_time_string() {
        let time = ReceiverTimeBlock {
            tow_ms: 0,
            wnc: 0,
            utc_year: 2024,
            utc_month: 3,
            utc_day: 15,
            utc_hour: 12,
            utc_minute: 30,
            utc_second: 45,
            delta_ls: 18,
            sync_level: 4,
        };

        assert_eq!(time.utc_string(), "2024-03-15T12:30:45Z");
        assert!(time.is_valid());
        assert!(time.is_synchronized());
    }

    #[test]
    fn test_receiver_time_validation() {
        let invalid_time = ReceiverTimeBlock {
            tow_ms: 0,
            wnc: 0,
            utc_year: 1999, // Invalid
            utc_month: 13,  // Invalid
            utc_day: 1,
            utc_hour: 0,
            utc_minute: 0,
            utc_second: 0,
            delta_ls: 0,
            sync_level: 0,
        };

        assert!(!invalid_time.is_valid());
        assert!(!invalid_time.is_synchronized());
    }

    #[test]
    fn test_sync_levels() {
        let time = ReceiverTimeBlock {
            tow_ms: 0,
            wnc: 0,
            utc_year: 2024,
            utc_month: 1,
            utc_day: 1,
            utc_hour: 0,
            utc_minute: 0,
            utc_second: 0,
            delta_ls: 18,
            sync_level: 0,
        };
        assert_eq!(time.sync_level_desc(), "Not synchronized");

        let time_sync = ReceiverTimeBlock {
            sync_level: 4,
            ..time
        };
        assert_eq!(time_sync.sync_level_desc(), "Fine time (PPS)");
    }

    #[test]
    fn test_pps_offset_accessors() {
        let block = PpsOffsetBlock {
            tow_ms: 1500,
            wnc: 2100,
            sync_age: 3,
            timescale: 1,
            offset_ns: F32_DNU,
        };

        assert!((block.tow_seconds() - 1.5).abs() < 1e-6);
        assert!(block.offset_ns().is_none());
        assert!(block.offset_seconds().is_none());
    }

    #[test]
    fn test_pps_offset_parse() {
        let mut data = vec![0u8; 18];
        data[12] = 2;
        data[13] = 1;
        data[14..18].copy_from_slice(&2500.0_f32.to_le_bytes());

        let header = header_for(block_ids::PPS_OFFSET, data.len(), 5000, 2200);
        let block = PpsOffsetBlock::parse(&header, &data).unwrap();

        assert_eq!(block.sync_age, 2);
        assert_eq!(block.timescale, 1);
        assert!((block.offset_ns().unwrap() - 2500.0).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_accessors() {
        let block = ExtEventBlock {
            tow_ms: 2500,
            wnc: 2300,
            source: 1,
            polarity: 0,
            offset_s: F32_DNU,
            rx_clk_bias_s: F64_DNU,
            pvt_age: 15,
        };

        assert!((block.tow_seconds() - 2.5).abs() < 1e-6);
        assert!(block.offset_seconds().is_none());
        assert!(block.offset_ns().is_none());
        assert!(block.rx_clk_bias_seconds().is_none());
        assert!(block.rx_clk_bias_ms().is_none());
    }

    #[test]
    fn test_ext_event_parse() {
        let mut data = vec![0u8; 28];
        data[12] = 2;
        data[13] = 1;
        data[14..18].copy_from_slice(&0.125_f32.to_le_bytes());
        data[18..26].copy_from_slice(&(-0.25_f64).to_le_bytes());
        data[26..28].copy_from_slice(&20_u16.to_le_bytes());

        let header = header_for(block_ids::EXT_EVENT, data.len(), 6000, 2400);
        let block = ExtEventBlock::parse(&header, &data).unwrap();

        assert_eq!(block.source, 2);
        assert_eq!(block.polarity, 1);
        assert_eq!(block.pvt_age, 20);
        assert!((block.offset_seconds().unwrap() - 0.125).abs() < 1e-9);
        assert!((block.offset_ns().unwrap() - 125000000.0).abs() < 1.0);
        assert!((block.rx_clk_bias_seconds().unwrap() + 0.25).abs() < 1e-12);
        assert!((block.rx_clk_bias_ms().unwrap() + 250.0).abs() < 1e-9);
    }

    #[test]
    fn test_ext_event_pvt_cartesian_scaled() {
        let block = ExtEventPvtCartesianBlock {
            tow_ms: 1000,
            wnc: 2000,
            mode: 4,
            error: 0,
            x_m: 1.0,
            y_m: 2.0,
            z_m: 3.0,
            undulation_m: 4.0,
            vx_mps: 0.1,
            vy_mps: 0.2,
            vz_mps: 0.3,
            cog_deg: 45.0,
            rx_clk_bias_ms: 0.0,
            rx_clk_drift_ppm: 0.0,
            time_system: 1,
            datum: 0,
            nr_sv: 8,
            wa_corr_info: 0,
            reference_id: 12,
            mean_corr_age_raw: 250,
            signal_info: 0,
            alert_flag: 0,
            nr_bases: 0,
        };

        assert!((block.mean_corr_age_seconds().unwrap() - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_pvt_cartesian_dnu() {
        let block = ExtEventPvtCartesianBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 0,
            error: 0,
            x_m: F64_DNU,
            y_m: 0.0,
            z_m: 0.0,
            undulation_m: F32_DNU,
            vx_mps: 0.0,
            vy_mps: 0.0,
            vz_mps: 0.0,
            cog_deg: 0.0,
            rx_clk_bias_ms: 0.0,
            rx_clk_drift_ppm: 0.0,
            time_system: 0,
            datum: 0,
            nr_sv: 0,
            wa_corr_info: 0,
            reference_id: 0,
            mean_corr_age_raw: U16_DNU,
            signal_info: 0,
            alert_flag: 0,
            nr_bases: 0,
        };

        assert!(block.x_m().is_none());
        assert!(block.undulation_m().is_none());
        assert!(block.mean_corr_age_seconds().is_none());
    }

    #[test]
    fn test_ext_event_pvt_cartesian_parse() {
        let mut data = vec![0u8; 84];
        data[12] = 3;
        data[13] = 1;
        data[14..22].copy_from_slice(&1.5_f64.to_le_bytes());
        data[22..30].copy_from_slice(&2.5_f64.to_le_bytes());
        data[30..38].copy_from_slice(&3.5_f64.to_le_bytes());
        data[38..42].copy_from_slice(&(-1.25_f32).to_le_bytes());
        data[42..46].copy_from_slice(&0.1_f32.to_le_bytes());
        data[46..50].copy_from_slice(&0.2_f32.to_le_bytes());
        data[50..54].copy_from_slice(&0.3_f32.to_le_bytes());
        data[54..58].copy_from_slice(&90.0_f32.to_le_bytes());
        data[58..66].copy_from_slice(&(-0.25_f64).to_le_bytes());
        data[66..70].copy_from_slice(&0.5_f32.to_le_bytes());
        data[70] = 2;
        data[71] = 1;
        data[72] = 7;
        data[73] = 3;
        data[74..76].copy_from_slice(&123_u16.to_le_bytes());
        data[76..78].copy_from_slice(&200_u16.to_le_bytes());
        data[78..82].copy_from_slice(&0xAABBCCDD_u32.to_le_bytes());
        data[82] = 1;
        data[83] = 2;

        let header = header_for(block_ids::EXT_EVENT_PVT_CARTESIAN, data.len(), 9000, 2200);
        let block = ExtEventPvtCartesianBlock::parse(&header, &data).unwrap();

        assert_eq!(block.mode_raw(), 3);
        assert_eq!(block.error_raw(), 1);
        assert_eq!(block.reference_id, 123);
        assert_eq!(block.num_satellites(), 7);
        assert!((block.x_m().unwrap() - 1.5).abs() < 1e-6);
        assert!((block.mean_corr_age_seconds().unwrap() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_pvt_geodetic_scaled() {
        let block = ExtEventPvtGeodeticBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 4,
            error: 0,
            latitude_rad: 1.0,
            longitude_rad: -0.5,
            height_m: 10.0,
            undulation_m: 1.0,
            vn_mps: 0.0,
            ve_mps: 0.0,
            vu_mps: 0.0,
            cog_deg: 0.0,
            rx_clk_bias_ms: 0.0,
            rx_clk_drift_ppm: 0.0,
            time_system: 0,
            datum: 0,
            nr_sv: 0,
            wa_corr_info: 0,
            reference_id: 0,
            mean_corr_age_raw: 150,
            signal_info: 0,
            alert_flag: 0,
            nr_bases: 0,
        };

        assert!((block.latitude_deg().unwrap() - 57.2958).abs() < 1e-3);
        assert!((block.longitude_deg().unwrap() + 28.6479).abs() < 1e-3);
        assert!((block.mean_corr_age_seconds().unwrap() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_pvt_geodetic_dnu() {
        let block = ExtEventPvtGeodeticBlock {
            tow_ms: 0,
            wnc: 0,
            mode: 0,
            error: 0,
            latitude_rad: F64_DNU,
            longitude_rad: 0.0,
            height_m: F64_DNU,
            undulation_m: F32_DNU,
            vn_mps: 0.0,
            ve_mps: 0.0,
            vu_mps: 0.0,
            cog_deg: 0.0,
            rx_clk_bias_ms: 0.0,
            rx_clk_drift_ppm: 0.0,
            time_system: 0,
            datum: 0,
            nr_sv: 0,
            wa_corr_info: 0,
            reference_id: 0,
            mean_corr_age_raw: U16_DNU,
            signal_info: 0,
            alert_flag: 0,
            nr_bases: 0,
        };

        assert!(block.latitude_deg().is_none());
        assert!(block.height_m().is_none());
        assert!(block.undulation_m().is_none());
        assert!(block.mean_corr_age_seconds().is_none());
    }

    #[test]
    fn test_ext_event_pvt_geodetic_parse() {
        let mut data = vec![0u8; 84];
        data[12] = 2;
        data[13] = 0;
        data[14..22].copy_from_slice(&0.5_f64.to_le_bytes());
        data[22..30].copy_from_slice(&1.0_f64.to_le_bytes());
        data[30..38].copy_from_slice(&50.0_f64.to_le_bytes());
        data[38..42].copy_from_slice(&2.5_f32.to_le_bytes());
        data[42..46].copy_from_slice(&(-0.1_f32).to_le_bytes());
        data[46..50].copy_from_slice(&0.2_f32.to_le_bytes());
        data[50..54].copy_from_slice(&0.3_f32.to_le_bytes());
        data[54..58].copy_from_slice(&120.0_f32.to_le_bytes());
        data[58..66].copy_from_slice(&1.25_f64.to_le_bytes());
        data[66..70].copy_from_slice(&(-0.75_f32).to_le_bytes());
        data[70] = 1;
        data[71] = 2;
        data[72] = 9;
        data[73] = 4;
        data[74..76].copy_from_slice(&321_u16.to_le_bytes());
        data[76..78].copy_from_slice(&100_u16.to_le_bytes());
        data[78..82].copy_from_slice(&0x01020304_u32.to_le_bytes());
        data[82] = 0;
        data[83] = 1;

        let header = header_for(block_ids::EXT_EVENT_PVT_GEODETIC, data.len(), 9100, 2300);
        let block = ExtEventPvtGeodeticBlock::parse(&header, &data).unwrap();

        assert_eq!(block.mode_raw(), 2);
        assert_eq!(block.reference_id, 321);
        assert_eq!(block.num_satellites(), 9);
        assert!((block.latitude_deg().unwrap() - 28.6479).abs() < 1e-3);
        assert!((block.height_m().unwrap() - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_base_vect_geod_parse() {
        let mut data = vec![0u8; 14 + 52];
        data[12] = 1;
        data[13] = 52;
        data[14] = 7;
        data[15] = 0;
        data[16] = 4;
        data[17] = 0;
        data[18..26].copy_from_slice(&1.5f64.to_le_bytes());
        data[26..34].copy_from_slice(&2.5f64.to_le_bytes());
        data[34..42].copy_from_slice(&3.5f64.to_le_bytes());
        data[54..56].copy_from_slice(&18000u16.to_le_bytes());
        data[56..58].copy_from_slice(&2500i16.to_le_bytes());
        data[58..60].copy_from_slice(&42u16.to_le_bytes());
        data[60..62].copy_from_slice(&300u16.to_le_bytes());
        data[62..66].copy_from_slice(&0x11223344u32.to_le_bytes());

        let header = header_for(block_ids::EXT_EVENT_BASE_VECT_GEOD, data.len(), 12000, 2400);
        let block = ExtEventBaseVectGeodBlock::parse(&header, &data).unwrap();
        assert_eq!(block.num_vectors(), 1);
        assert_eq!(block.vectors[0].reference_id, 42);
        assert!((block.vectors[0].de_m().unwrap() - 1.5).abs() < 1e-6);
        assert!((block.vectors[0].azimuth_deg().unwrap() - 180.0).abs() < 1e-6);
        assert!((block.vectors[0].corr_age_seconds().unwrap() - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_ext_event_att_euler_parse() {
        let mut data = vec![0u8; 42];
        data[12] = 8;
        data[13] = 1;
        data[14..16].copy_from_slice(&4u16.to_le_bytes());
        data[18..22].copy_from_slice(&45.0f32.to_le_bytes());
        data[22..26].copy_from_slice(&(-2.5f32).to_le_bytes());
        data[26..30].copy_from_slice(&1.25f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.5f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.25f32.to_le_bytes());
        data[38..42].copy_from_slice(&(-0.75f32).to_le_bytes());

        let header = header_for(block_ids::EXT_EVENT_ATT_EULER, data.len(), 13000, 2500);
        let block = ExtEventAttEulerBlock::parse(&header, &data).unwrap();
        assert_eq!(block.num_satellites(), 8);
        assert_eq!(block.error_raw(), 1);
        assert_eq!(block.mode_raw(), 4);
        assert_eq!(block.heading_deg(), Some(45.0));
        assert_eq!(block.pitch_deg(), Some(-2.5));
        assert_eq!(block.roll_deg(), Some(1.25));
        assert_eq!(block.heading_rate_dps(), Some(-0.75));
    }
}
