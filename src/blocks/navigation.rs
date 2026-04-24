//! Navigation message blocks (GPSNav, GALNav, GLONav)
//!
//! These blocks contain decoded ephemeris data for satellite orbit computation.

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::dnu::{f32_or_none, f64_or_none};
use super::SbfBlockParse;

#[cfg(test)]
use super::dnu::{F32_DNU, F64_DNU};

// ============================================================================
// GPSNav Block
// ============================================================================

/// GPSNav block (Block ID 5891)
///
/// Decoded GPS navigation message (ephemeris).
#[derive(Debug, Clone)]
pub struct GpsNavBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN number (1-32)
    pub prn: u8,
    /// GPS week number from ephemeris
    pub wn: u16,
    /// C/A or P code on L2
    pub ca_or_p_on_l2: u8,
    /// User Range Accuracy index
    pub ura: u8,
    /// Satellite health
    pub health: u8,
    /// L2 data flag
    pub l2_data_flag: u8,
    /// Issue of Data Clock
    pub iodc: u16,
    /// Issue of Data Ephemeris (subframe 2)
    pub iode2: u8,
    /// Issue of Data Ephemeris (subframe 3)
    pub iode3: u8,
    /// Fit interval flag
    pub fit_int_flag: u8,
    /// Group delay (seconds)
    pub t_gd: f32,
    /// Clock reference time (seconds)
    pub t_oc: u32,
    /// Clock drift rate (s/s^2)
    pub a_f2: f32,
    /// Clock drift (s/s)
    pub a_f1: f32,
    /// Clock bias (s)
    pub a_f0: f32,
    /// Sine harmonic radius correction (m)
    pub c_rs: f32,
    /// Mean motion difference (rad/s)
    pub delta_n: f32,
    /// Mean anomaly at reference time (rad)
    pub m_0: f64,
    /// Cosine harmonic latitude correction (rad)
    pub c_uc: f32,
    /// Eccentricity
    pub e: f64,
    /// Sine harmonic latitude correction (rad)
    pub c_us: f32,
    /// Square root of semi-major axis (m^0.5)
    pub sqrt_a: f64,
    /// Ephemeris reference time (seconds)
    pub t_oe: u32,
    /// Cosine harmonic inclination correction (rad)
    pub c_ic: f32,
    /// Right ascension at reference time (rad)
    pub omega_0: f64,
    /// Sine harmonic inclination correction (rad)
    pub c_is: f32,
    /// Inclination angle at reference time (rad)
    pub i_0: f64,
    /// Cosine harmonic radius correction (m)
    pub c_rc: f32,
    /// Argument of perigee (rad)
    pub omega: f64,
    /// Rate of right ascension (rad/s)
    pub omega_dot: f32,
    /// Rate of inclination (rad/s)
    pub i_dot: f32,
    /// Week number of t_oc
    pub wnt_oc: u16,
    /// Week number of t_oe
    pub wnt_oe: u16,
}

impl GpsNavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Check if ephemeris is healthy
    pub fn is_healthy(&self) -> bool {
        self.health == 0
    }

    /// Get semi-major axis in meters
    pub fn semi_major_axis_m(&self) -> f64 {
        self.sqrt_a * self.sqrt_a
    }

    /// Check if IODE values are consistent
    pub fn iode_consistent(&self) -> bool {
        self.iode2 == self.iode3
    }
}

impl SbfBlockParse for GpsNavBlock {
    const BLOCK_ID: u16 = block_ids::GPS_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 140 {
            return Err(SbfError::ParseError("GPSNav too short".into()));
        }

        // Offsets from data start (after sync):
        // 12: PRN
        // 13: Reserved
        // 14-15: WN
        // 16: CAorPonL2
        // 17: URA
        // 18: health
        // 19: L2DataFlag
        // 20-21: IODC
        // 22: IODE2
        // 23: IODE3
        // 24: FitIntFlg
        // 25: Reserved
        // 26-29: T_gd (f32)
        // 30-33: T_oc (u32)
        // 34-37: A_f2 (f32)
        // 38-41: A_f1 (f32)
        // 42-45: A_f0 (f32)
        // 46-49: C_rs (f32)
        // 50-53: DELTA_N (f32)
        // 54-61: M_0 (f64)
        // 62-65: C_uc (f32)
        // 66-73: E (f64)
        // 74-77: C_us (f32)
        // 78-85: SQRT_A (f64)
        // 86-89: T_oe (u32)
        // 90-93: C_ic (f32)
        // 94-101: OMEGA_0 (f64)
        // 102-105: C_is (f32)
        // 106-113: I_0 (f64)
        // 114-117: C_rc (f32)
        // 118-125: omega (f64)
        // 126-129: OMEGADOT (f32)
        // 130-133: IDOT (f32)
        // 134-135: WNt_oc
        // 136-137: WNt_oe

        let prn = data[12];
        let wn = u16::from_le_bytes([data[14], data[15]]);
        let ca_or_p_on_l2 = data[16];
        let ura = data[17];
        let health = data[18];
        let l2_data_flag = data[19];
        let iodc = u16::from_le_bytes([data[20], data[21]]);
        let iode2 = data[22];
        let iode3 = data[23];
        let fit_int_flag = data[24];

        let t_gd = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let t_oc = u32::from_le_bytes(data[30..34].try_into().unwrap());
        let a_f2 = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let a_f1 = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let a_f0 = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let c_rs = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let delta_n = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let m_0 = f64::from_le_bytes(data[54..62].try_into().unwrap());
        let c_uc = f32::from_le_bytes(data[62..66].try_into().unwrap());
        let e = f64::from_le_bytes(data[66..74].try_into().unwrap());
        let c_us = f32::from_le_bytes(data[74..78].try_into().unwrap());
        let sqrt_a = f64::from_le_bytes(data[78..86].try_into().unwrap());
        let t_oe = u32::from_le_bytes(data[86..90].try_into().unwrap());
        let c_ic = f32::from_le_bytes(data[90..94].try_into().unwrap());
        let omega_0 = f64::from_le_bytes(data[94..102].try_into().unwrap());
        let c_is = f32::from_le_bytes(data[102..106].try_into().unwrap());
        let i_0 = f64::from_le_bytes(data[106..114].try_into().unwrap());
        let c_rc = f32::from_le_bytes(data[114..118].try_into().unwrap());
        let omega = f64::from_le_bytes(data[118..126].try_into().unwrap());
        let omega_dot = f32::from_le_bytes(data[126..130].try_into().unwrap());
        let i_dot = f32::from_le_bytes(data[130..134].try_into().unwrap());
        let wnt_oc = u16::from_le_bytes([data[134], data[135]]);
        let wnt_oe = u16::from_le_bytes([data[136], data[137]]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            wn,
            ca_or_p_on_l2,
            ura,
            health,
            l2_data_flag,
            iodc,
            iode2,
            iode3,
            fit_int_flag,
            t_gd,
            t_oc,
            a_f2,
            a_f1,
            a_f0,
            c_rs,
            delta_n,
            m_0,
            c_uc,
            e,
            c_us,
            sqrt_a,
            t_oe,
            c_ic,
            omega_0,
            c_is,
            i_0,
            c_rc,
            omega,
            omega_dot,
            i_dot,
            wnt_oc,
            wnt_oe,
        })
    }
}

// ============================================================================
// GALNav Block
// ============================================================================

/// GALNav block (Block ID 4002)
///
/// Decoded Galileo navigation message (ephemeris).
#[derive(Debug, Clone)]
pub struct GalNavBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Signal source (F/NAV or I/NAV)
    pub source: u8,
    /// Square root of semi-major axis (m^0.5)
    pub sqrt_a: f64,
    /// Mean anomaly at reference time (rad)
    pub m_0: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination at reference time (rad)
    pub i_0: f64,
    /// Argument of perigee (rad)
    pub omega: f64,
    /// Right ascension at reference time (rad)
    pub omega_0: f64,
    /// Rate of right ascension (rad/s)
    pub omega_dot: f32,
    /// Rate of inclination (rad/s)
    pub i_dot: f32,
    /// Mean motion difference (rad/s)
    pub delta_n: f32,
    /// Cosine harmonic latitude correction (rad)
    pub c_uc: f32,
    /// Sine harmonic latitude correction (rad)
    pub c_us: f32,
    /// Cosine harmonic radius correction (m)
    pub c_rc: f32,
    /// Sine harmonic radius correction (m)
    pub c_rs: f32,
    /// Cosine harmonic inclination correction (rad)
    pub c_ic: f32,
    /// Sine harmonic inclination correction (rad)
    pub c_is: f32,
    /// Ephemeris reference time (seconds)
    pub t_oe: u32,
    /// Clock reference time (seconds)
    pub t_oc: u32,
    /// Clock drift rate (s/s^2)
    pub a_f2: f32,
    /// Clock drift (s/s)
    pub a_f1: f32,
    /// Clock bias (s)
    pub a_f0: f64,
    /// Week number of t_oc
    pub wnt_oc: u16,
    /// Week number of t_oe
    pub wnt_oe: u16,
    /// Issue of Data Navigation
    pub iod_nav: u16,
    /// Health/OS/SOL flags
    pub health_os_sol: u16,
    /// Signal-In-Space Accuracy (L1/E5a)
    pub sisa_l1e5a: u8,
    /// Signal-In-Space Accuracy (L1/E5b)
    pub sisa_l1e5b: u8,
    /// Broadcast Group Delay (L1/E5a)
    pub bgd_l1e5a: f32,
    /// Broadcast Group Delay (L1/E5b)
    pub bgd_l1e5b: f32,
}

impl GalNavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get PRN number (1-36)
    pub fn prn(&self) -> u8 {
        if self.svid >= 71 && self.svid <= 106 {
            self.svid - 70
        } else {
            self.svid
        }
    }

    /// Check if from F/NAV
    pub fn is_fnav(&self) -> bool {
        self.source == 16
    }

    /// Check if from I/NAV
    pub fn is_inav(&self) -> bool {
        self.source == 2
    }

    /// Get semi-major axis in meters
    pub fn semi_major_axis_m(&self) -> f64 {
        self.sqrt_a * self.sqrt_a
    }
}

impl SbfBlockParse for GalNavBlock {
    const BLOCK_ID: u16 = block_ids::GAL_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 144 {
            return Err(SbfError::ParseError("GALNav too short".into()));
        }

        let svid = data[12];
        let source = data[13];

        let sqrt_a = f64::from_le_bytes(data[14..22].try_into().unwrap());
        let m_0 = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let e = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let i_0 = f64::from_le_bytes(data[38..46].try_into().unwrap());
        let omega = f64::from_le_bytes(data[46..54].try_into().unwrap());
        let omega_0 = f64::from_le_bytes(data[54..62].try_into().unwrap());
        let omega_dot = f32::from_le_bytes(data[62..66].try_into().unwrap());
        let i_dot = f32::from_le_bytes(data[66..70].try_into().unwrap());
        let delta_n = f32::from_le_bytes(data[70..74].try_into().unwrap());
        let c_uc = f32::from_le_bytes(data[74..78].try_into().unwrap());
        let c_us = f32::from_le_bytes(data[78..82].try_into().unwrap());
        let c_rc = f32::from_le_bytes(data[82..86].try_into().unwrap());
        let c_rs = f32::from_le_bytes(data[86..90].try_into().unwrap());
        let c_ic = f32::from_le_bytes(data[90..94].try_into().unwrap());
        let c_is = f32::from_le_bytes(data[94..98].try_into().unwrap());
        let t_oe = u32::from_le_bytes(data[98..102].try_into().unwrap());
        let t_oc = u32::from_le_bytes(data[102..106].try_into().unwrap());
        let a_f2 = f32::from_le_bytes(data[106..110].try_into().unwrap());
        let a_f1 = f32::from_le_bytes(data[110..114].try_into().unwrap());
        let a_f0 = f64::from_le_bytes(data[114..122].try_into().unwrap());
        let wnt_oc = u16::from_le_bytes([data[122], data[123]]);
        let wnt_oe = u16::from_le_bytes([data[124], data[125]]);
        let iod_nav = u16::from_le_bytes([data[126], data[127]]);
        let health_os_sol = u16::from_le_bytes([data[128], data[129]]);
        let sisa_l1e5a = data[130];
        let sisa_l1e5b = data[131];
        let bgd_l1e5a = f32::from_le_bytes(data[132..136].try_into().unwrap());
        let bgd_l1e5b = f32::from_le_bytes(data[136..140].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            sqrt_a,
            m_0,
            e,
            i_0,
            omega,
            omega_0,
            omega_dot,
            i_dot,
            delta_n,
            c_uc,
            c_us,
            c_rc,
            c_rs,
            c_ic,
            c_is,
            t_oe,
            t_oc,
            a_f2,
            a_f1,
            a_f0,
            wnt_oc,
            wnt_oe,
            iod_nav,
            health_os_sol,
            sisa_l1e5a,
            sisa_l1e5b,
            bgd_l1e5a,
            bgd_l1e5b,
        })
    }
}

// ============================================================================
// GLONav Block
// ============================================================================

/// GLONav block (Block ID 4004)
///
/// Decoded GLONASS navigation message (ephemeris).
/// GLONASS uses a different ephemeris model based on position/velocity/acceleration.
#[derive(Debug, Clone)]
pub struct GloNavBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (38-61 for GLONASS)
    pub svid: u8,
    /// Frequency number (-7 to +6)
    pub freq_nr: i8,
    /// X position (km)
    pub x_km: f64,
    /// Y position (km)
    pub y_km: f64,
    /// Z position (km)
    pub z_km: f64,
    /// X velocity (km/s)
    pub dx_kmps: f32,
    /// Y velocity (km/s)
    pub dy_kmps: f32,
    /// Z velocity (km/s)
    pub dz_kmps: f32,
    /// X acceleration (km/s^2)
    pub ddx_kmps2: f32,
    /// Y acceleration (km/s^2)
    pub ddy_kmps2: f32,
    /// Z acceleration (km/s^2)
    pub ddz_kmps2: f32,
    /// Frequency bias (gamma)
    pub gamma: f32,
    /// Clock bias (tau, seconds)
    pub tau: f32,
    /// Time difference L1-L2 (dtau, seconds)
    pub dtau: f32,
    /// Ephemeris reference time
    pub t_oe: u32,
    /// Ephemeris reference week
    pub wn_toe: u16,
    /// Time interval P1
    pub p1: u8,
    /// Odd/even flag P2
    pub p2: u8,
    /// Age of data E
    pub e_age: u8,
    /// Health flag B
    pub b_health: u8,
    /// Frame time tb (15-minute intervals)
    pub tb: u16,
    /// Satellite type M
    pub m_type: u8,
    /// P1/P2 mode P
    pub p_mode: u8,
    /// Health flag l
    pub l_health: u8,
    /// Data updated flag P4
    pub p4: u8,
    /// Day number N_T
    pub n_t: u16,
    /// Accuracy F_T
    pub f_t: u16,
}

impl GloNavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get slot number (1-24)
    pub fn slot(&self) -> u8 {
        if self.svid >= 38 && self.svid <= 61 {
            self.svid - 37
        } else {
            self.svid
        }
    }

    /// Get position in meters
    pub fn position_m(&self) -> (f64, f64, f64) {
        (self.x_km * 1000.0, self.y_km * 1000.0, self.z_km * 1000.0)
    }

    /// Get velocity in m/s
    pub fn velocity_mps(&self) -> (f64, f64, f64) {
        (
            self.dx_kmps as f64 * 1000.0,
            self.dy_kmps as f64 * 1000.0,
            self.dz_kmps as f64 * 1000.0,
        )
    }

    /// Get acceleration in m/s^2
    pub fn acceleration_mps2(&self) -> (f64, f64, f64) {
        (
            self.ddx_kmps2 as f64 * 1000.0,
            self.ddy_kmps2 as f64 * 1000.0,
            self.ddz_kmps2 as f64 * 1000.0,
        )
    }

    /// Check if satellite is healthy
    pub fn is_healthy(&self) -> bool {
        self.b_health == 0 && self.l_health == 0
    }
}

impl SbfBlockParse for GloNavBlock {
    const BLOCK_ID: u16 = block_ids::GLO_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 96 {
            return Err(SbfError::ParseError("GLONav too short".into()));
        }

        let svid = data[12];
        let freq_nr = data[13] as i8;

        let x_km = f64::from_le_bytes(data[14..22].try_into().unwrap());
        let y_km = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let z_km = f64::from_le_bytes(data[30..38].try_into().unwrap());
        let dx_kmps = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let dy_kmps = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let dz_kmps = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let ddx_kmps2 = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let ddy_kmps2 = f32::from_le_bytes(data[54..58].try_into().unwrap());
        let ddz_kmps2 = f32::from_le_bytes(data[58..62].try_into().unwrap());
        let gamma = f32::from_le_bytes(data[62..66].try_into().unwrap());
        let tau = f32::from_le_bytes(data[66..70].try_into().unwrap());
        let dtau = f32::from_le_bytes(data[70..74].try_into().unwrap());
        let t_oe = u32::from_le_bytes(data[74..78].try_into().unwrap());
        let wn_toe = u16::from_le_bytes([data[78], data[79]]);
        let p1 = data[80];
        let p2 = data[81];
        let e_age = data[82];
        let b_health = data[83];
        let tb = u16::from_le_bytes([data[84], data[85]]);
        let m_type = data[86];
        let p_mode = data[87];
        let l_health = data[88];
        let p4 = data[89];
        let n_t = u16::from_le_bytes([data[90], data[91]]);
        let f_t = u16::from_le_bytes([data[92], data[93]]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            freq_nr,
            x_km,
            y_km,
            z_km,
            dx_kmps,
            dy_kmps,
            dz_kmps,
            ddx_kmps2,
            ddy_kmps2,
            ddz_kmps2,
            gamma,
            tau,
            dtau,
            t_oe,
            wn_toe,
            p1,
            p2,
            e_age,
            b_health,
            tb,
            m_type,
            p_mode,
            l_health,
            p4,
            n_t,
            f_t,
        })
    }
}

// ============================================================================
// GPSAlm Block
// ============================================================================

/// GPSAlm block (Block ID 5892)
///
/// Decoded GPS almanac data.
#[derive(Debug, Clone)]
pub struct GpsAlmBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN number (1-32)
    pub prn: u8,
    /// Eccentricity
    pub e: f32,
    /// Almanac reference time (s)
    pub t_oa: u32,
    /// Inclination offset (rad)
    pub delta_i: f32,
    /// Rate of right ascension (rad/s)
    pub omega_dot: f32,
    /// Square root of semi-major axis (m^0.5)
    pub sqrt_a: f32,
    /// Right ascension (rad)
    pub omega_0: f32,
    /// Argument of perigee (rad)
    pub omega: f32,
    /// Mean anomaly (rad)
    pub m_0: f32,
    /// Clock drift (s/s)
    pub a_f1: f32,
    /// Clock bias (s)
    pub a_f0: f32,
    /// Almanac week
    pub wn_a: u8,
    /// Anti-spoofing config
    pub as_config: u8,
    /// Health (8-bit)
    pub health8: u8,
    /// Health (6-bit)
    pub health6: u8,
}

impl GpsAlmBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Eccentricity (None if DNU)
    pub fn eccentricity(&self) -> Option<f32> {
        f32_or_none(self.e)
    }

    /// Semi-major axis in meters (None if DNU)
    pub fn semi_major_axis_m(&self) -> Option<f32> {
        f32_or_none(self.sqrt_a).map(|value| value * value)
    }

    /// Clock bias in seconds (None if DNU)
    pub fn clock_bias_s(&self) -> Option<f32> {
        f32_or_none(self.a_f0)
    }
}

impl SbfBlockParse for GpsAlmBlock {
    const BLOCK_ID: u16 = block_ids::GPS_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 57 {
            return Err(SbfError::ParseError("GPSAlm too short".into()));
        }

        let prn = data[12];
        let e = f32::from_le_bytes(data[13..17].try_into().unwrap());
        let t_oa = u32::from_le_bytes(data[17..21].try_into().unwrap());
        let delta_i = f32::from_le_bytes(data[21..25].try_into().unwrap());
        let omega_dot = f32::from_le_bytes(data[25..29].try_into().unwrap());
        let sqrt_a = f32::from_le_bytes(data[29..33].try_into().unwrap());
        let omega_0 = f32::from_le_bytes(data[33..37].try_into().unwrap());
        let omega = f32::from_le_bytes(data[37..41].try_into().unwrap());
        let m_0 = f32::from_le_bytes(data[41..45].try_into().unwrap());
        let a_f1 = f32::from_le_bytes(data[45..49].try_into().unwrap());
        let a_f0 = f32::from_le_bytes(data[49..53].try_into().unwrap());
        let wn_a = data[53];
        let as_config = data[54];
        let health8 = data[55];
        let health6 = data[56];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            e,
            t_oa,
            delta_i,
            omega_dot,
            sqrt_a,
            omega_0,
            omega,
            m_0,
            a_f1,
            a_f0,
            wn_a,
            as_config,
            health8,
            health6,
        })
    }
}

// ============================================================================
// GPSIon Block
// ============================================================================

/// GPSIon block (Block ID 5893)
///
/// GPS ionosphere parameters.
#[derive(Debug, Clone)]
pub struct GpsIonBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN number (1-32)
    pub prn: u8,
    /// Ionosphere alpha parameter (s)
    pub alpha_0: f32,
    /// Ionosphere alpha parameter (s/semi-circle)
    pub alpha_1: f32,
    /// Ionosphere alpha parameter (s/semi-circle^2)
    pub alpha_2: f32,
    /// Ionosphere alpha parameter (s/semi-circle^3)
    pub alpha_3: f32,
    /// Ionosphere beta parameter (s)
    pub beta_0: f32,
    /// Ionosphere beta parameter (s/semi-circle)
    pub beta_1: f32,
    /// Ionosphere beta parameter (s/semi-circle^2)
    pub beta_2: f32,
    /// Ionosphere beta parameter (s/semi-circle^3)
    pub beta_3: f32,
}

impl GpsIonBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn alpha_0(&self) -> Option<f32> {
        f32_or_none(self.alpha_0)
    }

    pub fn beta_0(&self) -> Option<f32> {
        f32_or_none(self.beta_0)
    }
}

impl SbfBlockParse for GpsIonBlock {
    const BLOCK_ID: u16 = block_ids::GPS_ION;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 45 {
            return Err(SbfError::ParseError("GPSIon too short".into()));
        }

        let prn = data[12];
        let alpha_0 = f32::from_le_bytes(data[13..17].try_into().unwrap());
        let alpha_1 = f32::from_le_bytes(data[17..21].try_into().unwrap());
        let alpha_2 = f32::from_le_bytes(data[21..25].try_into().unwrap());
        let alpha_3 = f32::from_le_bytes(data[25..29].try_into().unwrap());
        let beta_0 = f32::from_le_bytes(data[29..33].try_into().unwrap());
        let beta_1 = f32::from_le_bytes(data[33..37].try_into().unwrap());
        let beta_2 = f32::from_le_bytes(data[37..41].try_into().unwrap());
        let beta_3 = f32::from_le_bytes(data[41..45].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            alpha_0,
            alpha_1,
            alpha_2,
            alpha_3,
            beta_0,
            beta_1,
            beta_2,
            beta_3,
        })
    }
}

// ============================================================================
// GPSUtc Block
// ============================================================================

/// GPSUtc block (Block ID 5894)
///
/// GPS to UTC conversion parameters.
#[derive(Debug, Clone)]
pub struct GpsUtcBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN number (1-32)
    pub prn: u8,
    /// UTC drift (s/s)
    pub a_1: f32,
    /// UTC bias (s)
    pub a_0: f64,
    /// Reference time (s)
    pub t_ot: u32,
    /// Reference week
    pub wn_t: u8,
    /// Current leap seconds
    pub delta_t_ls: i8,
    /// Week of future leap second
    pub wn_lsf: u8,
    /// Day of future leap second
    pub dn: u8,
    /// Future leap seconds
    pub delta_t_lsf: i8,
}

impl GpsUtcBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn utc_bias_s(&self) -> Option<f64> {
        f64_or_none(self.a_0)
    }

    pub fn utc_drift_s_per_s(&self) -> Option<f32> {
        f32_or_none(self.a_1)
    }
}

impl SbfBlockParse for GpsUtcBlock {
    const BLOCK_ID: u16 = block_ids::GPS_UTC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 34 {
            return Err(SbfError::ParseError("GPSUtc too short".into()));
        }

        let prn = data[12];
        let a_1 = f32::from_le_bytes(data[13..17].try_into().unwrap());
        let a_0 = f64::from_le_bytes(data[17..25].try_into().unwrap());
        let t_ot = u32::from_le_bytes(data[25..29].try_into().unwrap());
        let wn_t = data[29];
        let delta_t_ls = data[30] as i8;
        let wn_lsf = data[31];
        let dn = data[32];
        let delta_t_lsf = data[33] as i8;

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            a_1,
            a_0,
            t_ot,
            wn_t,
            delta_t_ls,
            wn_lsf,
            dn,
            delta_t_lsf,
        })
    }
}

// ============================================================================
// GLOAlm Block
// ============================================================================

/// GLOAlm block (Block ID 4005)
///
/// GLONASS almanac data.
#[derive(Debug, Clone)]
pub struct GloAlmBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (38-61 for GLONASS)
    pub svid: u8,
    /// Frequency number (-7 to +6)
    pub freq_nr: i8,
    /// Eccentricity
    pub epsilon: f32,
    /// Almanac reference time
    pub t_oa: u32,
    /// Inclination correction (rad)
    pub delta_i: f32,
    /// Longitude of ascending node (rad)
    pub lambda: f32,
    /// Time of ascending node (s)
    pub t_ln: f32,
    /// Argument of perigee (rad)
    pub omega: f32,
    /// Orbit period correction (s/orbit)
    pub delta_t: f32,
    /// Orbit period rate (s/orbit^2)
    pub d_delta_t: f32,
    /// Clock bias (s)
    pub tau: f32,
    /// Almanac week
    pub wn_a: u8,
    /// Health flag
    pub c: u8,
    /// Day number
    pub n: u16,
    /// Satellite type
    pub m_type: u8,
    /// 4-year interval
    pub n_4: u8,
}

impl GloAlmBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get slot number (1-24)
    pub fn slot(&self) -> u8 {
        if self.svid >= 38 && self.svid <= 61 {
            self.svid - 37
        } else {
            self.svid
        }
    }

    pub fn eccentricity(&self) -> Option<f32> {
        f32_or_none(self.epsilon)
    }

    pub fn clock_bias_s(&self) -> Option<f32> {
        f32_or_none(self.tau)
    }
}

impl SbfBlockParse for GloAlmBlock {
    const BLOCK_ID: u16 = block_ids::GLO_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 56 {
            return Err(SbfError::ParseError("GLOAlm too short".into()));
        }

        let svid = data[12];
        let freq_nr = data[13] as i8;
        let epsilon = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let t_oa = u32::from_le_bytes(data[18..22].try_into().unwrap());
        let delta_i = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let lambda = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let t_ln = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let omega = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let delta_t = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let d_delta_t = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let tau = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let wn_a = data[50];
        let c = data[51];
        let n = u16::from_le_bytes([data[52], data[53]]);
        let m_type = data[54];
        let n_4 = data[55];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            freq_nr,
            epsilon,
            t_oa,
            delta_i,
            lambda,
            t_ln,
            omega,
            delta_t,
            d_delta_t,
            tau,
            wn_a,
            c,
            n,
            m_type,
            n_4,
        })
    }
}

// ============================================================================
// GLOTime Block
// ============================================================================

/// GLOTime block (Block ID 4036)
///
/// GLONASS time parameters.
#[derive(Debug, Clone)]
pub struct GloTimeBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (38-61 for GLONASS)
    pub svid: u8,
    /// Frequency number (-7 to +6)
    pub freq_nr: i8,
    /// 4-year interval number
    pub n_4: u8,
    /// Notification of leap second
    pub kp: u8,
    /// Day number
    pub n: u16,
    /// GPS-GLONASS time offset (s)
    pub tau_gps: f32,
    /// GLONASS time scale correction (s)
    pub tau_c: f64,
    /// UT1-UTC coefficient
    pub b1: f32,
    /// UT1-UTC rate
    pub b2: f32,
}

impl GloTimeBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get slot number (1-24)
    pub fn slot(&self) -> u8 {
        if self.svid >= 38 && self.svid <= 61 {
            self.svid - 37
        } else {
            self.svid
        }
    }

    pub fn gps_glonass_offset_s(&self) -> Option<f32> {
        f32_or_none(self.tau_gps)
    }

    pub fn time_scale_correction_s(&self) -> Option<f64> {
        f64_or_none(self.tau_c)
    }
}

impl SbfBlockParse for GloTimeBlock {
    const BLOCK_ID: u16 = block_ids::GLO_TIME;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 38 {
            return Err(SbfError::ParseError("GLOTime too short".into()));
        }

        let svid = data[12];
        let freq_nr = data[13] as i8;
        let n_4 = data[14];
        let kp = data[15];
        let n = u16::from_le_bytes([data[16], data[17]]);
        let tau_gps = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let tau_c = f64::from_le_bytes(data[22..30].try_into().unwrap());
        let b1 = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let b2 = f32::from_le_bytes(data[34..38].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            freq_nr,
            n_4,
            kp,
            n,
            tau_gps,
            tau_c,
            b1,
            b2,
        })
    }
}

// ============================================================================
// GALAlm Block
// ============================================================================

/// GALAlm block (Block ID 4003)
///
/// Galileo almanac data.
#[derive(Debug, Clone)]
pub struct GalAlmBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Signal source (F/NAV or I/NAV)
    pub source: u8,
    /// Eccentricity
    pub e: f32,
    /// Almanac reference time (s)
    pub t_oa: u32,
    /// Inclination offset (rad)
    pub delta_i: f32,
    /// Rate of right ascension (rad/s)
    pub omega_dot: f32,
    /// Semi-major axis delta (m^0.5)
    pub delta_sqrt_a: f32,
    /// Right ascension (rad)
    pub omega_0: f32,
    /// Argument of perigee (rad)
    pub omega: f32,
    /// Mean anomaly (rad)
    pub m_0: f32,
    /// Clock drift (s/s)
    pub a_f1: f32,
    /// Clock bias (s)
    pub a_f0: f32,
    /// Almanac week
    pub wn_a: u8,
    /// Almanac SVID
    pub svid_a: u8,
    /// Health flags
    pub health: u16,
    /// Issue of Data Almanac
    pub ioda: u8,
}

impl GalAlmBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get PRN number (1-36)
    pub fn prn(&self) -> u8 {
        if self.svid >= 71 && self.svid <= 106 {
            self.svid - 70
        } else {
            self.svid
        }
    }

    pub fn eccentricity(&self) -> Option<f32> {
        f32_or_none(self.e)
    }

    pub fn delta_sqrt_a(&self) -> Option<f32> {
        f32_or_none(self.delta_sqrt_a)
    }
}

impl SbfBlockParse for GalAlmBlock {
    const BLOCK_ID: u16 = block_ids::GAL_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 59 {
            return Err(SbfError::ParseError("GALAlm too short".into()));
        }

        let svid = data[12];
        let source = data[13];
        let e = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let t_oa = u32::from_le_bytes(data[18..22].try_into().unwrap());
        let delta_i = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let omega_dot = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let delta_sqrt_a = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let omega_0 = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let omega = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let m_0 = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let a_f1 = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let a_f0 = f32::from_le_bytes(data[50..54].try_into().unwrap());
        let wn_a = data[54];
        let svid_a = data[55];
        let health = u16::from_le_bytes([data[56], data[57]]);
        let ioda = data[58];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            e,
            t_oa,
            delta_i,
            omega_dot,
            delta_sqrt_a,
            omega_0,
            omega,
            m_0,
            a_f1,
            a_f0,
            wn_a,
            svid_a,
            health,
            ioda,
        })
    }
}

// ============================================================================
// GALIon Block
// ============================================================================

/// GALIon block (Block ID 4030)
///
/// Galileo ionosphere parameters.
#[derive(Debug, Clone)]
pub struct GalIonBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Signal source (F/NAV or I/NAV)
    pub source: u8,
    /// Ionosphere coefficient (sfu)
    pub a_i0: f32,
    /// Ionosphere coefficient (sfu/degree)
    pub a_i1: f32,
    /// Ionosphere coefficient (sfu/degree^2)
    pub a_i2: f32,
    /// Ionosphere storm flags
    pub storm_flags: u8,
}

impl GalIonBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Check if from F/NAV
    pub fn is_fnav(&self) -> bool {
        self.source == 16
    }

    /// Check if from I/NAV
    pub fn is_inav(&self) -> bool {
        self.source == 2
    }

    pub fn a_i0(&self) -> Option<f32> {
        f32_or_none(self.a_i0)
    }
}

impl SbfBlockParse for GalIonBlock {
    const BLOCK_ID: u16 = block_ids::GAL_ION;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 27 {
            return Err(SbfError::ParseError("GALIon too short".into()));
        }

        let svid = data[12];
        let source = data[13];
        let a_i0 = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let a_i1 = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let a_i2 = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let storm_flags = data[26];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            a_i0,
            a_i1,
            a_i2,
            storm_flags,
        })
    }
}

// ============================================================================
// GALUtc Block
// ============================================================================

/// GALUtc block (Block ID 4031)
///
/// Galileo UTC parameters.
#[derive(Debug, Clone)]
pub struct GalUtcBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Signal source (F/NAV or I/NAV)
    pub source: u8,
    /// UTC drift (s/s)
    pub a_1: f32,
    /// UTC bias (s)
    pub a_0: f64,
    /// Reference time (s)
    pub t_ot: u32,
    /// Reference week
    pub wn_ot: u8,
    /// Current leap seconds
    pub delta_t_ls: i8,
    /// Week of future leap second
    pub wn_lsf: u8,
    /// Day of future leap second
    pub dn: u8,
    /// Future leap seconds
    pub delta_t_lsf: i8,
}

impl GalUtcBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get PRN number (1-36)
    pub fn prn(&self) -> u8 {
        if self.svid >= 71 && self.svid <= 106 {
            self.svid - 70
        } else {
            self.svid
        }
    }

    pub fn utc_bias_s(&self) -> Option<f64> {
        f64_or_none(self.a_0)
    }

    pub fn utc_drift_s_per_s(&self) -> Option<f32> {
        f32_or_none(self.a_1)
    }
}

impl SbfBlockParse for GalUtcBlock {
    const BLOCK_ID: u16 = block_ids::GAL_UTC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 35 {
            return Err(SbfError::ParseError("GALUtc too short".into()));
        }

        let svid = data[12];
        let source = data[13];
        let a_1 = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let a_0 = f64::from_le_bytes(data[18..26].try_into().unwrap());
        let t_ot = u32::from_le_bytes(data[26..30].try_into().unwrap());
        let wn_ot = data[30];
        let delta_t_ls = data[31] as i8;
        let wn_lsf = data[32];
        let dn = data[33];
        let delta_t_lsf = data[34] as i8;

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            a_1,
            a_0,
            t_ot,
            wn_ot,
            delta_t_ls,
            wn_lsf,
            dn,
            delta_t_lsf,
        })
    }
}

// ============================================================================
// GALGstGps Block
// ============================================================================

/// GALGstGps block (Block ID 4032)
///
/// Galileo GST to GPS time parameters.
#[derive(Debug, Clone)]
pub struct GalGstGpsBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Signal source (F/NAV or I/NAV)
    pub source: u8,
    /// GST-GPS drift (10^9 ns/s)
    pub a_1g: f32,
    /// GST-GPS offset (10^9 ns)
    pub a_0g: f32,
    /// Reference time (s)
    pub t_og: u32,
    /// Reference week
    pub wn_og: u8,
}

impl GalGstGpsBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get PRN number (1-36)
    pub fn prn(&self) -> u8 {
        if self.svid >= 71 && self.svid <= 106 {
            self.svid - 70
        } else {
            self.svid
        }
    }

    pub fn gst_gps_offset_s(&self) -> Option<f32> {
        f32_or_none(self.a_0g)
    }

    pub fn gst_gps_drift_s_per_s(&self) -> Option<f32> {
        f32_or_none(self.a_1g)
    }
}

impl SbfBlockParse for GalGstGpsBlock {
    const BLOCK_ID: u16 = block_ids::GAL_GST_GPS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 27 {
            return Err(SbfError::ParseError("GALGstGps too short".into()));
        }

        let svid = data[12];
        let source = data[13];
        let a_1g = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let a_0g = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let t_og = u32::from_le_bytes(data[22..26].try_into().unwrap());
        let wn_og = data[26];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            a_1g,
            a_0g,
            t_og,
            wn_og,
        })
    }
}

// ============================================================================
// GALAuthStatus Block
// ============================================================================

/// GALAuthStatus (4245) — Galileo OSNMA authentication status.
#[derive(Debug, Clone)]
pub struct GalAuthStatusBlock {
    tow_ms: u32,
    wnc: u16,
    /// OSNMA status bitfield (see Septentrio SBF reference: Status, progress, time source, …).
    pub osnma_status: u16,
    /// Trusted time delta (seconds).
    pub trusted_time_delta: f32,
    /// Galileo active authentication mask (64 bit).
    pub gal_active_mask: u64,
    /// Galileo authentic mask (64 bit).
    pub gal_authentic_mask: u64,
    /// GPS active authentication mask (64 bit).
    pub gps_active_mask: u64,
    /// GPS authentic mask (64 bit).
    pub gps_authentic_mask: u64,
}

impl GalAuthStatusBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    pub fn trusted_time_delta_s(&self) -> Option<f32> {
        f32_or_none(self.trusted_time_delta)
    }
}

impl SbfBlockParse for GalAuthStatusBlock {
    const BLOCK_ID: u16 = block_ids::GAL_AUTH_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 50 {
            return Err(SbfError::ParseError("GALAuthStatus too short".into()));
        }

        let osnma_status = u16::from_le_bytes([data[12], data[13]]);
        let trusted_time_delta = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let gal_active_mask = u64::from_le_bytes(data[18..26].try_into().unwrap());
        let gal_authentic_mask = u64::from_le_bytes(data[26..34].try_into().unwrap());
        let gps_active_mask = u64::from_le_bytes(data[34..42].try_into().unwrap());
        let gps_authentic_mask = u64::from_le_bytes(data[42..50].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            osnma_status,
            trusted_time_delta,
            gal_active_mask,
            gal_authentic_mask,
            gps_active_mask,
            gps_authentic_mask,
        })
    }
}

// ============================================================================
// GALSARRLM Block
// ============================================================================

/// GALSARRLM block (Block ID 4034)
///
/// Decoded Galileo search-and-rescue return link message (RLM).
#[derive(Debug, Clone)]
pub struct GalSarRlmBlock {
    tow_ms: u32,
    wnc: u16,
    /// SVID (71-106 for Galileo)
    pub svid: u8,
    /// Message source (2=I/NAV, 16=F/NAV)
    pub source: u8,
    /// RLM payload length in bits (typically 80 or 160)
    rlm_length_bits: u8,
    /// RLM payload words, MSB-first within each word.
    rlm_bits_words: Vec<u32>,
}

impl GalSarRlmBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get PRN number (1-36) when SVID is in Galileo range.
    pub fn prn(&self) -> u8 {
        if (71..=106).contains(&self.svid) {
            self.svid - 70
        } else {
            self.svid
        }
    }

    pub fn rlm_length_bits(&self) -> u8 {
        self.rlm_length_bits
    }

    /// Raw 32-bit words containing the RLM payload bits.
    pub fn rlm_bits_words(&self) -> &[u32] {
        &self.rlm_bits_words
    }

    /// Return one RLM bit by index (0-based), with bit 0 as the MSB of word 0.
    pub fn bit(&self, bit_index: usize) -> Option<bool> {
        if bit_index >= self.rlm_length_bits as usize {
            return None;
        }

        let word_index = bit_index / 32;
        let bit_in_word = 31 - (bit_index % 32);
        self.rlm_bits_words
            .get(word_index)
            .map(|word| ((word >> bit_in_word) & 1) != 0)
    }
}

impl SbfBlockParse for GalSarRlmBlock {
    const BLOCK_ID: u16 = block_ids::GAL_SAR_RLM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 18 || data.len() < data_len {
            return Err(SbfError::ParseError("GALSARRLM too short".into()));
        }

        let svid = data[12];
        let source = data[13];
        let rlm_length_bits = data[14];

        let word_count = match rlm_length_bits {
            80 => 3,
            160 => 5,
            0 => 0,
            _ => usize::from(rlm_length_bits).div_ceil(32),
        };
        let required_len = 18 + word_count * 4;
        if data_len < required_len {
            return Err(SbfError::ParseError("GALSARRLM payload too short".into()));
        }

        let mut rlm_bits_words = Vec::with_capacity(word_count);
        let mut offset = 18;
        for _ in 0..word_count {
            rlm_bits_words.push(u32::from_le_bytes(
                data[offset..offset + 4].try_into().unwrap(),
            ));
            offset += 4;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid,
            source,
            rlm_length_bits,
            rlm_bits_words,
        })
    }
}

// ============================================================================
// GPSCNav Block
// ============================================================================

/// GPSCNav block (Block ID 4042)
///
/// Decoded GPS CNAV navigation data from L2C and/or L5 signals.
/// Contains ephemeris from MT10/11 and clock/ISC corrections from MT30.
#[derive(Debug, Clone)]
pub struct GpsCNavBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN number (1-32)
    pub prn: u8,
    /// Flags bit field (alert, integrity, L2C phasing, L2C/L5 used)
    pub flags: u8,
    /// Week number (13 bits from MT10)
    pub wn: u16,
    /// L1/L2/L5 signal health (3 bits from MT10)
    pub health: u8,
    /// Elevation-Dependent accuracy index (URA_ED)
    pub ura_ed: i8,
    /// Data predict time of week (seconds)
    pub t_op: u32,
    /// Ephemeris reference time (seconds)
    pub t_oe: u32,
    /// Semi-major axis (m)
    pub a: f64,
    /// Change rate in semi-major axis (m/s)
    pub a_dot: f64,
    /// Mean motion difference (semi-circles/s)
    pub delta_n: f32,
    /// Rate of mean motion difference (semi-circles/s^2)
    pub delta_n_dot: f32,
    /// Mean anomaly at reference time (semi-circles)
    pub m_0: f64,
    /// Eccentricity
    pub e: f64,
    /// Argument of perigee (semi-circles)
    pub omega: f64,
    /// Right ascension at reference time (semi-circles)
    pub omega_0: f64,
    /// Rate of right ascension (semi-circles/s)
    pub omega_dot: f64,
    /// Inclination angle at reference time (semi-circles)
    pub i_0: f64,
    /// Rate of inclination (semi-circles/s)
    pub i_dot: f32,
    /// Sine harmonic inclination correction (rad)
    pub c_is: f32,
    /// Cosine harmonic inclination correction (rad)
    pub c_ic: f32,
    /// Sine harmonic radius correction (m)
    pub c_rs: f32,
    /// Cosine harmonic radius correction (m)
    pub c_rc: f32,
    /// Sine harmonic latitude correction (rad)
    pub c_us: f32,
    /// Cosine harmonic latitude correction (rad)
    pub c_uc: f32,
    /// Clock reference time (seconds)
    pub t_oc: u32,
    /// Non-Elevation-Dependent accuracy index 0
    pub ura_ned0: i8,
    /// Non-Elevation-Dependent accuracy change index
    pub ura_ned1: u8,
    /// Non-Elevation-Dependent accuracy change rate index
    pub ura_ned2: u8,
    /// Week number associated with t_op (modulo 256)
    pub wn_op: u8,
    /// Clock drift rate (s/s^2)
    pub a_f2: f32,
    /// Clock drift (s/s)
    pub a_f1: f32,
    /// Clock bias (s)
    pub a_f0: f64,
    /// Group delay differential (s)
    pub t_gd: f32,
    /// Inter-Signal Correction for L1C/A (s)
    pub isc_l1ca: f32,
    /// Inter-Signal Correction for L2C (s)
    pub isc_l2c: f32,
    /// Inter-Signal Correction for L5I (s)
    pub isc_l5i5: f32,
    /// Inter-Signal Correction for L5Q (s)
    pub isc_l5q5: f32,
}

impl GpsCNavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Check if alert bit is set
    pub fn is_alert(&self) -> bool {
        (self.flags & 0x01) != 0
    }

    /// Check if L2C was used for decoding
    pub fn l2c_used(&self) -> bool {
        (self.flags & 0x40) != 0
    }

    /// Check if L5 was used for decoding
    pub fn l5_used(&self) -> bool {
        (self.flags & 0x80) != 0
    }

    /// Check if satellite is healthy
    pub fn is_healthy(&self) -> bool {
        self.health == 0
    }

    /// Group delay (None if DNU)
    pub fn group_delay_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd)
    }

    /// ISC L1C/A (None if DNU)
    pub fn isc_l1ca_s(&self) -> Option<f32> {
        f32_or_none(self.isc_l1ca)
    }

    /// ISC L2C (None if DNU)
    pub fn isc_l2c_s(&self) -> Option<f32> {
        f32_or_none(self.isc_l2c)
    }

    /// ISC L5I5 (None if DNU)
    pub fn isc_l5i5_s(&self) -> Option<f32> {
        f32_or_none(self.isc_l5i5)
    }

    /// ISC L5Q5 (None if DNU)
    pub fn isc_l5q5_s(&self) -> Option<f32> {
        f32_or_none(self.isc_l5q5)
    }
}

impl SbfBlockParse for GpsCNavBlock {
    const BLOCK_ID: u16 = block_ids::GPS_CNAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Minimum size: header (12) + fields up to ISC_L5Q5 (170 bytes)
        if data.len() < 170 {
            return Err(SbfError::ParseError("GPSCNav too short".into()));
        }

        // Offsets from data start (after sync):
        // 12: PRNidx (u1)
        // 13: Flags (u1)
        // 14-15: WN (u2)
        // 16: Health (u1)
        // 17: URA_ED (i1)
        // 18-21: t_op (u4)
        // 22-25: t_oe (u4)
        // 26-33: A (f8)
        // 34-41: A_DOT (f8)
        // 42-45: DELTA_N (f4)
        // 46-49: DELTA_N_DOT (f4)
        // 50-57: M_0 (f8)
        // 58-65: e (f8)
        // 66-73: omega (f8)
        // 74-81: OMEGA_0 (f8)
        // 82-89: OMEGADOT (f8)
        // 90-97: i_0 (f8)
        // 98-101: IDOT (f4)
        // 102-105: C_is (f4)
        // 106-109: C_ic (f4)
        // 110-113: C_rs (f4)
        // 114-117: C_rc (f4)
        // 118-121: C_us (f4)
        // 122-125: C_uc (f4)
        // 126-129: t_oc (u4)
        // 130: URA_NED0 (i1)
        // 131: URA_NED1 (u1)
        // 132: URA_NED2 (u1)
        // 133: WN_op (u1)
        // 134-137: a_f2 (f4)
        // 138-141: a_f1 (f4)
        // 142-149: a_f0 (f8)
        // 150-153: T_gd (f4)
        // 154-157: ISC_L1CA (f4)
        // 158-161: ISC_L2C (f4)
        // 162-165: ISC_L5I5 (f4)
        // 166-169: ISC_L5Q5 (f4)

        let prn = data[12];
        let flags = data[13];
        let wn = u16::from_le_bytes([data[14], data[15]]);
        let health = data[16];
        let ura_ed = data[17] as i8;
        let t_op = u32::from_le_bytes(data[18..22].try_into().unwrap());
        let t_oe = u32::from_le_bytes(data[22..26].try_into().unwrap());
        let a = f64::from_le_bytes(data[26..34].try_into().unwrap());
        let a_dot = f64::from_le_bytes(data[34..42].try_into().unwrap());
        let delta_n = f32::from_le_bytes(data[42..46].try_into().unwrap());
        let delta_n_dot = f32::from_le_bytes(data[46..50].try_into().unwrap());
        let m_0 = f64::from_le_bytes(data[50..58].try_into().unwrap());
        let e = f64::from_le_bytes(data[58..66].try_into().unwrap());
        let omega = f64::from_le_bytes(data[66..74].try_into().unwrap());
        let omega_0 = f64::from_le_bytes(data[74..82].try_into().unwrap());
        let omega_dot = f64::from_le_bytes(data[82..90].try_into().unwrap());
        let i_0 = f64::from_le_bytes(data[90..98].try_into().unwrap());
        let i_dot = f32::from_le_bytes(data[98..102].try_into().unwrap());
        let c_is = f32::from_le_bytes(data[102..106].try_into().unwrap());
        let c_ic = f32::from_le_bytes(data[106..110].try_into().unwrap());
        let c_rs = f32::from_le_bytes(data[110..114].try_into().unwrap());
        let c_rc = f32::from_le_bytes(data[114..118].try_into().unwrap());
        let c_us = f32::from_le_bytes(data[118..122].try_into().unwrap());
        let c_uc = f32::from_le_bytes(data[122..126].try_into().unwrap());
        let t_oc = u32::from_le_bytes(data[126..130].try_into().unwrap());
        let ura_ned0 = data[130] as i8;
        let ura_ned1 = data[131];
        let ura_ned2 = data[132];
        let wn_op = data[133];
        let a_f2 = f32::from_le_bytes(data[134..138].try_into().unwrap());
        let a_f1 = f32::from_le_bytes(data[138..142].try_into().unwrap());
        let a_f0 = f64::from_le_bytes(data[142..150].try_into().unwrap());
        let t_gd = f32::from_le_bytes(data[150..154].try_into().unwrap());
        let isc_l1ca = f32::from_le_bytes(data[154..158].try_into().unwrap());
        let isc_l2c = f32::from_le_bytes(data[158..162].try_into().unwrap());
        let isc_l5i5 = f32::from_le_bytes(data[162..166].try_into().unwrap());
        let isc_l5q5 = f32::from_le_bytes(data[166..170].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            flags,
            wn,
            health,
            ura_ed,
            t_op,
            t_oe,
            a,
            a_dot,
            delta_n,
            delta_n_dot,
            m_0,
            e,
            omega,
            omega_0,
            omega_dot,
            i_0,
            i_dot,
            c_is,
            c_ic,
            c_rs,
            c_rc,
            c_us,
            c_uc,
            t_oc,
            ura_ned0,
            ura_ned1,
            ura_ned2,
            wn_op,
            a_f2,
            a_f1,
            a_f0,
            t_gd,
            isc_l1ca,
            isc_l2c,
            isc_l5i5,
            isc_l5q5,
        })
    }
}

// ============================================================================
// BDSIon Block
// ============================================================================

/// BDSIon block (Block ID 4120)
///
/// BeiDou ionosphere parameters (Klobuchar coefficients) from D1/D2 nav message.
#[derive(Debug, Clone)]
pub struct BdsIonBlock {
    tow_ms: u32,
    wnc: u16,
    /// PRN of the BeiDou satellite (SVID, see 4.1.9)
    pub prn: u8,
    /// Vertical delay coefficient 0 (s)
    pub alpha_0: f32,
    /// Vertical delay coefficient 1 (s/semi-circle)
    pub alpha_1: f32,
    /// Vertical delay coefficient 2 (s/semi-circle^2)
    pub alpha_2: f32,
    /// Vertical delay coefficient 3 (s/semi-circle^3)
    pub alpha_3: f32,
    /// Model period coefficient 0 (s)
    pub beta_0: f32,
    /// Model period coefficient 1 (s/semi-circle)
    pub beta_1: f32,
    /// Model period coefficient 2 (s/semi-circle^2)
    pub beta_2: f32,
    /// Model period coefficient 3 (s/semi-circle^3)
    pub beta_3: f32,
}

impl BdsIonBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn alpha_0(&self) -> Option<f32> {
        f32_or_none(self.alpha_0)
    }

    pub fn beta_0(&self) -> Option<f32> {
        f32_or_none(self.beta_0)
    }
}

impl SbfBlockParse for BdsIonBlock {
    const BLOCK_ID: u16 = block_ids::BDS_ION;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Header (12) + PRN (1) + Reserved (1) + 8 x f32 (32) = 46 bytes minimum
        if data.len() < 46 {
            return Err(SbfError::ParseError("BDSIon too short".into()));
        }

        // Offsets:
        // 12: PRN (u1)
        // 13: Reserved (u1)
        // 14-17: alpha_0 (f4)
        // 18-21: alpha_1 (f4)
        // 22-25: alpha_2 (f4)
        // 26-29: alpha_3 (f4)
        // 30-33: beta_0 (f4)
        // 34-37: beta_1 (f4)
        // 38-41: beta_2 (f4)
        // 42-45: beta_3 (f4)

        let prn = data[12];
        // data[13] is reserved
        let alpha_0 = f32::from_le_bytes(data[14..18].try_into().unwrap());
        let alpha_1 = f32::from_le_bytes(data[18..22].try_into().unwrap());
        let alpha_2 = f32::from_le_bytes(data[22..26].try_into().unwrap());
        let alpha_3 = f32::from_le_bytes(data[26..30].try_into().unwrap());
        let beta_0 = f32::from_le_bytes(data[30..34].try_into().unwrap());
        let beta_1 = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let beta_2 = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let beta_3 = f32::from_le_bytes(data[42..46].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            alpha_0,
            alpha_1,
            alpha_2,
            alpha_3,
            beta_0,
            beta_1,
            beta_2,
            beta_3,
        })
    }
}

// ============================================================================
// BDSCNav1 Block
// ============================================================================

/// BDSCNav1 block (Block ID 4251)
///
/// BeiDou B-CNAV1 navigation data from B1C signal.
#[derive(Debug, Clone)]
pub struct BdsCNav1Block {
    tow_ms: u32,
    wnc: u16,
    /// PRN index within BeiDou constellation (1 for C01, etc.)
    pub prn_idx: u8,
    /// Flags: bits 0-1 = satellite type (1: GEO, 2: IGSO, 3: MEO)
    pub flags: u8,
    /// Ephemeris reference time (seconds)
    pub t_oe: u32,
    /// Semi-major axis (m)
    pub a: f64,
    /// Change rate in semi-major axis (m/s)
    pub a_dot: f64,
    /// Mean motion difference (semi-circles/s)
    pub delta_n0: f32,
    /// Rate of mean motion difference (semi-circles/s^2)
    pub delta_n0_dot: f32,
    /// Mean anomaly (semi-circles)
    pub m_0: f64,
    /// Eccentricity
    pub e: f64,
    /// Argument of perigee (semi-circles)
    pub omega: f64,
    /// Longitude of ascending node (semi-circles)
    pub omega_0: f64,
    /// Rate of right ascension (semi-circles/s)
    pub omega_dot: f32,
    /// Inclination angle (semi-circles)
    pub i_0: f64,
    /// Rate of inclination (semi-circles/s)
    pub i_dot: f32,
    /// Sine harmonic inclination correction (rad)
    pub c_is: f32,
    /// Cosine harmonic inclination correction (rad)
    pub c_ic: f32,
    /// Sine harmonic radius correction (m)
    pub c_rs: f32,
    /// Cosine harmonic radius correction (m)
    pub c_rc: f32,
    /// Sine harmonic latitude correction (rad)
    pub c_us: f32,
    /// Cosine harmonic latitude correction (rad)
    pub c_uc: f32,
    /// Clock reference time (seconds)
    pub t_oc: u32,
    /// Clock drift rate (s/s^2)
    pub a_2: f32,
    /// Clock drift (s/s)
    pub a_1: f32,
    /// Clock bias (s)
    pub a_0: f64,
    /// Time of week for data prediction (seconds)
    pub t_op: u32,
    /// Satellite orbit radius and clock bias accuracy index
    pub sisai_ocb: u8,
    /// Combined SISAI_oc1 and SISAI_oc2 (bits 0-2: oc2, bits 3-5: oc1)
    pub sisai_oc12: u8,
    /// Satellite orbit along-track and cross-track accuracy index
    pub sisai_oe: u8,
    /// Signal in space monitoring accuracy index
    pub sismai: u8,
    /// Health and integrity flags
    pub health_if: u8,
    /// Issue of Data Ephemeris
    pub iode: u8,
    /// Issue of Data Clock
    pub iodc: u16,
    /// Group delay between B1C data and pilot (s)
    pub isc_b1cd: f32,
    /// Group delay of B1C pilot (s)
    pub t_gd_b1cp: f32,
    /// Group delay of B2a pilot (s)
    pub t_gd_b2ap: f32,
}

impl BdsCNav1Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get satellite type (1: GEO, 2: IGSO, 3: MEO)
    pub fn satellite_type(&self) -> u8 {
        self.flags & 0x03
    }

    /// Check if satellite is GEO
    pub fn is_geo(&self) -> bool {
        self.satellite_type() == 1
    }

    /// Check if satellite is IGSO
    pub fn is_igso(&self) -> bool {
        self.satellite_type() == 2
    }

    /// Check if satellite is MEO
    pub fn is_meo(&self) -> bool {
        self.satellite_type() == 3
    }

    /// Check if satellite is healthy (bits 6-7 of health_if == 0)
    pub fn is_healthy(&self) -> bool {
        (self.health_if & 0xC0) == 0
    }

    /// ISC B1Cd (None if DNU)
    pub fn isc_b1cd_s(&self) -> Option<f32> {
        f32_or_none(self.isc_b1cd)
    }

    /// T_GD B1Cp (None if DNU)
    pub fn t_gd_b1cp_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd_b1cp)
    }

    /// T_GD B2ap (None if DNU)
    pub fn t_gd_b2ap_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd_b2ap)
    }
}

impl SbfBlockParse for BdsCNav1Block {
    const BLOCK_ID: u16 = block_ids::BDS_CNAV1;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Calculate minimum size based on field layout
        // Header fields (12) + PRNidx(1) + Flags(1) + t_oe(4) + A(8) + A_DOT(8) +
        // DELTA_n0(4) + DELTA_n0_DOT(4) + M_0(8) + e(8) + omega(8) + OMEGA_0(8) +
        // OMEGADOT(4) + i_0(8) + IDOT(4) + C_is(4) + C_ic(4) + C_rs(4) + C_rc(4) +
        // C_us(4) + C_uc(4) + t_oc(4) + a_2(4) + a_1(4) + a_0(8) + t_op(4) +
        // SISAI_ocb(1) + SISAI_oc12(1) + SISAI_oe(1) + SISMAI(1) + HealthIF(1) +
        // IODE(1) + IODC(2) + ISC_B1Cd(4) + T_GDB1Cp(4) + T_GDB2ap(4)
        // = 12 + 1 + 1 + 4 + 8 + 8 + 4 + 4 + 8 + 8 + 8 + 8 + 4 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 8 + 4 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 4 + 4 + 4 = 158 bytes
        if data.len() < 158 {
            return Err(SbfError::ParseError("BDSCNav1 too short".into()));
        }

        // Offsets:
        // 12: PRNidx (u1)
        // 13: Flags (u1)
        // 14-17: t_oe (u4)
        // 18-25: A (f8)
        // 26-33: A_DOT (f8)
        // 34-37: DELTA_n0 (f4)
        // 38-41: DELTA_n0_DOT (f4)
        // 42-49: M_0 (f8)
        // 50-57: e (f8)
        // 58-65: omega (f8)
        // 66-73: OMEGA_0 (f8)
        // 74-77: OMEGADOT (f4)
        // 78-85: i_0 (f8)
        // 86-89: IDOT (f4)
        // 90-93: C_is (f4)
        // 94-97: C_ic (f4)
        // 98-101: C_rs (f4)
        // 102-105: C_rc (f4)
        // 106-109: C_us (f4)
        // 110-113: C_uc (f4)
        // 114-117: t_oc (u4)
        // 118-121: a_2 (f4)
        // 122-125: a_1 (f4)
        // 126-133: a_0 (f8)
        // 134-137: t_op (u4)
        // 138: SISAI_ocb (u1)
        // 139: SISAI_oc12 (u1)
        // 140: SISAI_oe (u1)
        // 141: SISMAI (u1)
        // 142: HealthIF (u1)
        // 143: IODE (u1)
        // 144-145: IODC (u2)
        // 146-149: ISC_B1Cd (f4)
        // 150-153: T_GDB1Cp (f4)
        // 154-157: T_GDB2ap (f4)

        let prn_idx = data[12];
        let flags = data[13];
        let t_oe = u32::from_le_bytes(data[14..18].try_into().unwrap());
        let a = f64::from_le_bytes(data[18..26].try_into().unwrap());
        let a_dot = f64::from_le_bytes(data[26..34].try_into().unwrap());
        let delta_n0 = f32::from_le_bytes(data[34..38].try_into().unwrap());
        let delta_n0_dot = f32::from_le_bytes(data[38..42].try_into().unwrap());
        let m_0 = f64::from_le_bytes(data[42..50].try_into().unwrap());
        let e = f64::from_le_bytes(data[50..58].try_into().unwrap());
        let omega = f64::from_le_bytes(data[58..66].try_into().unwrap());
        let omega_0 = f64::from_le_bytes(data[66..74].try_into().unwrap());
        let omega_dot = f32::from_le_bytes(data[74..78].try_into().unwrap());
        let i_0 = f64::from_le_bytes(data[78..86].try_into().unwrap());
        let i_dot = f32::from_le_bytes(data[86..90].try_into().unwrap());
        let c_is = f32::from_le_bytes(data[90..94].try_into().unwrap());
        let c_ic = f32::from_le_bytes(data[94..98].try_into().unwrap());
        let c_rs = f32::from_le_bytes(data[98..102].try_into().unwrap());
        let c_rc = f32::from_le_bytes(data[102..106].try_into().unwrap());
        let c_us = f32::from_le_bytes(data[106..110].try_into().unwrap());
        let c_uc = f32::from_le_bytes(data[110..114].try_into().unwrap());
        let t_oc = u32::from_le_bytes(data[114..118].try_into().unwrap());
        let a_2 = f32::from_le_bytes(data[118..122].try_into().unwrap());
        let a_1 = f32::from_le_bytes(data[122..126].try_into().unwrap());
        let a_0 = f64::from_le_bytes(data[126..134].try_into().unwrap());
        let t_op = u32::from_le_bytes(data[134..138].try_into().unwrap());
        let sisai_ocb = data[138];
        let sisai_oc12 = data[139];
        let sisai_oe = data[140];
        let sismai = data[141];
        let health_if = data[142];
        let iode = data[143];
        let iodc = u16::from_le_bytes([data[144], data[145]]);
        let isc_b1cd = f32::from_le_bytes(data[146..150].try_into().unwrap());
        let t_gd_b1cp = f32::from_le_bytes(data[150..154].try_into().unwrap());
        let t_gd_b2ap = f32::from_le_bytes(data[154..158].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn_idx,
            flags,
            t_oe,
            a,
            a_dot,
            delta_n0,
            delta_n0_dot,
            m_0,
            e,
            omega,
            omega_0,
            omega_dot,
            i_0,
            i_dot,
            c_is,
            c_ic,
            c_rs,
            c_rc,
            c_us,
            c_uc,
            t_oc,
            a_2,
            a_1,
            a_0,
            t_op,
            sisai_ocb,
            sisai_oc12,
            sisai_oe,
            sismai,
            health_if,
            iode,
            iodc,
            isc_b1cd,
            t_gd_b1cp,
            t_gd_b2ap,
        })
    }
}

// ============================================================================
// GPSRawCA Block
// ============================================================================

/// GPSRawCA block (Block ID 4017)
///
/// Raw GPS C/A navigation subframe bits (L1).
#[derive(Debug, Clone)]
pub struct GpsRawCaBlock {
    tow_ms: u32,
    wnc: u16,
    /// Satellite ID (1-32 for GPS)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl GpsRawCaBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    /// Whether CRC check passed
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    /// Raw navigation bits as slice
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GpsRawCaBlock {
    const BLOCK_ID: u16 = block_ids::GPS_RAW_CA;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Header (12) + SVID(1) + CRCPassed(1) + ViterbiCount(1) + Source(1) + FreqNr(1) + NAVBits(40)
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GPSRawCA too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GPSRawL2C Block
// ============================================================================

/// GPSRawL2C block (Block ID 4018)
///
/// Raw GPS L2C navigation frame bits.
#[derive(Debug, Clone)]
pub struct GpsRawL2CBlock {
    tow_ms: u32,
    wnc: u16,
    /// Satellite ID (1-32 for GPS)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl GpsRawL2CBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GpsRawL2CBlock {
    const BLOCK_ID: u16 = block_ids::GPS_RAW_L2C;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GPSRawL2C too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GPSRawL5 Block
// ============================================================================

/// GPSRawL5 block (Block ID 4019)
///
/// Raw GPS L5 navigation frame bits.
#[derive(Debug, Clone)]
pub struct GpsRawL5Block {
    tow_ms: u32,
    wnc: u16,
    /// Satellite ID (1-32 for GPS)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl GpsRawL5Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GpsRawL5Block {
    const BLOCK_ID: u16 = block_ids::GPS_RAW_L5;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GPSRawL5 too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GLORawCA Block
// ============================================================================

/// GLORawCA block (Block ID 4026)
///
/// Raw GLONASS CA navigation string bits.
#[derive(Debug, Clone)]
pub struct GloRawCaBlock {
    tow_ms: u32,
    wnc: u16,
    /// GLONASS slot (38-61)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number (-7 to +6)
    pub freq_nr: u8,
    /// Raw navigation bits (3 × u4 = 12 bytes, 96 bits)
    pub nav_bits: [u8; 12],
}

impl GloRawCaBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 12] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GloRawCaBlock {
    const BLOCK_ID: u16 = block_ids::GLO_RAW_CA;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Header (12) + SVID(1) + CRCPassed(1) + ViterbiCount(1) + Source(1) + FreqNr(1) + NAVBits(12)
        const MIN_LEN: usize = 29;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GLORawCA too short".into()));
        }

        let mut nav_bits = [0u8; 12];
        nav_bits.copy_from_slice(&data[17..29]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GALRawFNAV Block
// ============================================================================

/// GALRawFNAV block (Block ID 4022)
///
/// Raw Galileo F/NAV navigation bits.
#[derive(Debug, Clone)]
pub struct GalRawFnavBlock {
    tow_ms: u32,
    wnc: u16,
    /// Galileo SVID (71-102)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (8 × u4 = 32 bytes, 256 bits)
    pub nav_bits: [u8; 32],
}

impl GalRawFnavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 32] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GalRawFnavBlock {
    const BLOCK_ID: u16 = block_ids::GAL_RAW_FNAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 49;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GALRawFNAV too short".into()));
        }

        let mut nav_bits = [0u8; 32];
        nav_bits.copy_from_slice(&data[17..49]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GALRawINAV Block
// ============================================================================

/// GALRawINAV block (Block ID 4023)
///
/// Raw Galileo I/NAV navigation bits.
#[derive(Debug, Clone)]
pub struct GalRawInavBlock {
    tow_ms: u32,
    wnc: u16,
    /// Galileo SVID (71-102)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (8 × u4 = 32 bytes, 256 bits)
    pub nav_bits: [u8; 32],
}

impl GalRawInavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 32] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GalRawInavBlock {
    const BLOCK_ID: u16 = block_ids::GAL_RAW_INAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 49;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GALRawINAV too short".into()));
        }

        let mut nav_bits = [0u8; 32];
        nav_bits.copy_from_slice(&data[17..49]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GALRawCNAV Block
// ============================================================================

/// GALRawCNAV block (Block ID 4024)
///
/// Raw Galileo CNAV navigation bits.
#[derive(Debug, Clone)]
pub struct GalRawCnavBlock {
    tow_ms: u32,
    wnc: u16,
    /// Galileo SVID (71-102)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (16 × u4 = 64 bytes, 512 bits)
    pub nav_bits: [u8; 64],
}

impl GalRawCnavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 64] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GalRawCnavBlock {
    const BLOCK_ID: u16 = block_ids::GAL_RAW_CNAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 81;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GALRawCNAV too short".into()));
        }

        let mut nav_bits = [0u8; 64];
        nav_bits.copy_from_slice(&data[17..81]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GEORawL1 Block
// ============================================================================

/// GEORawL1 block (Block ID 4020)
///
/// Raw SBAS L1 navigation bits.
#[derive(Debug, Clone)]
pub struct GeoRawL1Block {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN (120-158)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (8 × u4 = 32 bytes, 256 bits)
    pub nav_bits: [u8; 32],
}

impl GeoRawL1Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 32] {
        &self.nav_bits
    }
}

impl SbfBlockParse for GeoRawL1Block {
    const BLOCK_ID: u16 = block_ids::GEO_RAW_L1;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 49;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEORawL1 too short".into()));
        }

        let mut nav_bits = [0u8; 32];
        nav_bits.copy_from_slice(&data[17..49]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// CMPRaw Block
// ============================================================================

/// CMPRaw block (Block ID 4047)
///
/// Raw BeiDou navigation bits.
#[derive(Debug, Clone)]
pub struct CmpRawBlock {
    tow_ms: u32,
    wnc: u16,
    /// BeiDou SVID (141-172)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl CmpRawBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for CmpRawBlock {
    const BLOCK_ID: u16 = block_ids::CMP_RAW;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("CMPRaw too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// QZSRawL1CA Block
// ============================================================================

/// QZSRawL1CA block (Block ID 4066)
///
/// Raw QZSS L1 C/A navigation bits.
#[derive(Debug, Clone)]
pub struct QzsRawL1CaBlock {
    tow_ms: u32,
    wnc: u16,
    /// QZSS SVID (181-187)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl QzsRawL1CaBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for QzsRawL1CaBlock {
    const BLOCK_ID: u16 = block_ids::QZS_RAW_L1CA;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("QZSRawL1CA too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// QZSRawL2C Block
// ============================================================================

/// QZSRawL2C block (Block ID 4067)
///
/// Raw QZSS L2C navigation bits.
#[derive(Debug, Clone)]
pub struct QzsRawL2CBlock {
    tow_ms: u32,
    wnc: u16,
    /// QZSS SVID (181-187)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl QzsRawL2CBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for QzsRawL2CBlock {
    const BLOCK_ID: u16 = block_ids::QZS_RAW_L2C;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("QZSRawL2C too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// QZSRawL5 Block
// ============================================================================

/// QZSRawL5 block (Block ID 4068)
///
/// Raw QZSS L5 navigation bits.
#[derive(Debug, Clone)]
pub struct QzsRawL5Block {
    tow_ms: u32,
    wnc: u16,
    /// QZSS SVID (181-187)
    pub svid: u8,
    /// CRC check: 0=failed, 1=passed
    pub crc_passed: u8,
    /// Viterbi decoder error count
    pub viterbi_count: u8,
    /// Signal source
    pub source: u8,
    /// Frequency number
    pub freq_nr: u8,
    /// Raw navigation bits (10 × u4 = 40 bytes, 300 bits)
    pub nav_bits: [u8; 40],
}

impl QzsRawL5Block {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
    pub fn nav_bits_slice(&self) -> &[u8; 40] {
        &self.nav_bits
    }
}

impl SbfBlockParse for QzsRawL5Block {
    const BLOCK_ID: u16 = block_ids::QZS_RAW_L5;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 57;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("QZSRawL5 too short".into()));
        }

        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[17..57]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            nav_bits,
        })
    }
}

// ============================================================================
// GEOIonoDelay Block
// ============================================================================

/// One ionospheric delay correction entry in `GEOIonoDelay`.
#[derive(Debug, Clone)]
pub struct GeoIonoDelayIdc {
    /// Sequence number in the IGP mask (1..201)
    pub igp_mask_no: u8,
    /// Grid Ionospheric Vertical Error Indicator (0..15)
    pub givei: u8,
    vertical_delay_m_raw: f32,
}

impl GeoIonoDelayIdc {
    /// Vertical delay estimate in meters.
    /// Returns `None` when the receiver marks the value as do-not-use.
    pub fn vertical_delay_m(&self) -> Option<f32> {
        f32_or_none(self.vertical_delay_m_raw)
    }

    /// Raw vertical delay value from the block payload.
    pub fn vertical_delay_m_raw(&self) -> f32 {
        self.vertical_delay_m_raw
    }
}

/// GEOIonoDelay block (Block ID 5933)
///
/// SBAS MT26 ionospheric delay corrections.
#[derive(Debug, Clone)]
pub struct GeoIonoDelayBlock {
    tow_ms: u32,
    wnc: u16,
    /// ID of the SBAS satellite from which MT26 was received
    pub prn: u8,
    /// SBAS band number
    pub band_nbr: u8,
    /// Issue of data ionosphere
    pub iodi: u8,
    /// Ionospheric delay correction entries
    pub idc: Vec<GeoIonoDelayIdc>,
}

impl GeoIonoDelayBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_idc(&self) -> usize {
        self.idc.len()
    }
}

impl SbfBlockParse for GeoIonoDelayBlock {
    const BLOCK_ID: u16 = block_ids::GEO_IONO_DELAY;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Header (12) + PRN (1) + BandNbr (1) + IODI (1) + N (1) + SBLength (1) + Reserved (1)
        if data.len() < 18 {
            return Err(SbfError::ParseError("GEOIonoDelay too short".into()));
        }

        let prn = data[12];
        let band_nbr = data[13];
        let iodi = data[14];
        let n = data[15] as usize;
        let sb_length = data[16] as usize;

        if sb_length < 8 {
            return Err(SbfError::ParseError(
                "GEOIonoDelay SBLength too small".into(),
            ));
        }

        let mut idc = Vec::with_capacity(n);
        let mut offset = 18;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let igp_mask_no = data[offset];
            let givei = data[offset + 1];
            let vertical_delay_m_raw =
                f32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());

            idc.push(GeoIonoDelayIdc {
                igp_mask_no,
                givei,
                vertical_delay_m_raw,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            band_nbr,
            iodi,
            idc,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::SbfBlock;
    use crate::header::{SbfHeader, SBF_SYNC};

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
    fn test_gps_alm_accessors() {
        let block = GpsAlmBlock {
            tow_ms: 1000,
            wnc: 2000,
            prn: 5,
            e: F32_DNU,
            t_oa: 100,
            delta_i: 0.1,
            omega_dot: 0.2,
            sqrt_a: 5153.5,
            omega_0: 1.0,
            omega: 1.1,
            m_0: 0.5,
            a_f1: 0.0,
            a_f0: 0.0,
            wn_a: 10,
            as_config: 1,
            health8: 0,
            health6: 0,
        };

        assert!((block.tow_seconds() - 1.0).abs() < 1e-6);
        assert!(block.eccentricity().is_none());
        assert!((block.semi_major_axis_m().unwrap() - 5153.5_f32.powi(2)).abs() < 1e-3);
    }

    #[test]
    fn test_gps_alm_parse() {
        let mut data = vec![0u8; 57];
        data[12] = 7;
        data[13..17].copy_from_slice(&0.02_f32.to_le_bytes());
        data[17..21].copy_from_slice(&1234_u32.to_le_bytes());
        data[21..25].copy_from_slice(&0.1_f32.to_le_bytes());
        data[25..29].copy_from_slice(&0.2_f32.to_le_bytes());
        data[29..33].copy_from_slice(&5153.8_f32.to_le_bytes());
        data[33..37].copy_from_slice(&1.0_f32.to_le_bytes());
        data[37..41].copy_from_slice(&1.1_f32.to_le_bytes());
        data[41..45].copy_from_slice(&0.5_f32.to_le_bytes());
        data[45..49].copy_from_slice(&0.01_f32.to_le_bytes());
        data[49..53].copy_from_slice(&0.02_f32.to_le_bytes());
        data[53] = 12;
        data[54] = 1;
        data[55] = 0;
        data[56] = 0;

        let header = header_for(block_ids::GPS_ALM, data.len(), 5000, 2001);
        let block = GpsAlmBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 7);
        assert_eq!(block.wnc(), 2001);
        assert_eq!(block.t_oa, 1234);
        assert!((block.eccentricity().unwrap() - 0.02).abs() < 1e-6);
    }

    #[test]
    fn test_gps_ion_accessors() {
        let block = GpsIonBlock {
            tow_ms: 2500,
            wnc: 2100,
            prn: 3,
            alpha_0: F32_DNU,
            alpha_1: 0.1,
            alpha_2: 0.2,
            alpha_3: 0.3,
            beta_0: 1.0,
            beta_1: 2.0,
            beta_2: 3.0,
            beta_3: 4.0,
        };

        assert!((block.tow_seconds() - 2.5).abs() < 1e-6);
        assert!(block.alpha_0().is_none());
        assert!((block.beta_0().unwrap() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_gps_ion_parse() {
        let mut data = vec![0u8; 45];
        data[12] = 8;
        data[13..17].copy_from_slice(&0.1_f32.to_le_bytes());
        data[17..21].copy_from_slice(&0.2_f32.to_le_bytes());
        data[21..25].copy_from_slice(&0.3_f32.to_le_bytes());
        data[25..29].copy_from_slice(&0.4_f32.to_le_bytes());
        data[29..33].copy_from_slice(&1.1_f32.to_le_bytes());
        data[33..37].copy_from_slice(&1.2_f32.to_le_bytes());
        data[37..41].copy_from_slice(&1.3_f32.to_le_bytes());
        data[41..45].copy_from_slice(&1.4_f32.to_le_bytes());

        let header = header_for(block_ids::GPS_ION, data.len(), 8000, 2002);
        let block = GpsIonBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 8);
        assert!((block.alpha_1 - 0.2).abs() < 1e-6);
        assert!((block.beta_3 - 1.4).abs() < 1e-6);
    }

    #[test]
    fn test_gps_utc_accessors() {
        let block = GpsUtcBlock {
            tow_ms: 3000,
            wnc: 2200,
            prn: 1,
            a_1: 0.001,
            a_0: F64_DNU,
            t_ot: 4000,
            wn_t: 12,
            delta_t_ls: 18,
            wn_lsf: 13,
            dn: 2,
            delta_t_lsf: 19,
        };

        assert!((block.tow_seconds() - 3.0).abs() < 1e-6);
        assert!(block.utc_bias_s().is_none());
        assert!((block.utc_drift_s_per_s().unwrap() - 0.001).abs() < 1e-6);
    }

    #[test]
    fn test_gps_utc_parse() {
        let mut data = vec![0u8; 34];
        data[12] = 2;
        data[13..17].copy_from_slice(&0.001_f32.to_le_bytes());
        data[17..25].copy_from_slice(&(-0.5_f64).to_le_bytes());
        data[25..29].copy_from_slice(&12345_u32.to_le_bytes());
        data[29] = 6;
        data[30] = 18u8;
        data[31] = 7;
        data[32] = 4;
        data[33] = 19u8;

        let header = header_for(block_ids::GPS_UTC, data.len(), 9000, 2003);
        let block = GpsUtcBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 2);
        assert_eq!(block.wn_t, 6);
        assert!((block.utc_bias_s().unwrap() + 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_glo_alm_accessors() {
        let block = GloAlmBlock {
            tow_ms: 500,
            wnc: 2300,
            svid: 40,
            freq_nr: -3,
            epsilon: F32_DNU,
            t_oa: 200,
            delta_i: 0.1,
            lambda: 0.2,
            t_ln: 100.0,
            omega: 0.3,
            delta_t: 0.4,
            d_delta_t: 0.5,
            tau: 0.0,
            wn_a: 5,
            c: 0,
            n: 10,
            m_type: 1,
            n_4: 2,
        };

        assert!((block.tow_seconds() - 0.5).abs() < 1e-6);
        assert_eq!(block.slot(), 3);
        assert!(block.eccentricity().is_none());
        assert!(block.clock_bias_s().is_some());
    }

    #[test]
    fn test_glo_alm_parse() {
        let mut data = vec![0u8; 56];
        data[12] = 38;
        data[13] = 250u8;
        data[14..18].copy_from_slice(&0.01_f32.to_le_bytes());
        data[18..22].copy_from_slice(&200_u32.to_le_bytes());
        data[22..26].copy_from_slice(&0.1_f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.2_f32.to_le_bytes());
        data[30..34].copy_from_slice(&300.0_f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.3_f32.to_le_bytes());
        data[38..42].copy_from_slice(&0.4_f32.to_le_bytes());
        data[42..46].copy_from_slice(&0.5_f32.to_le_bytes());
        data[46..50].copy_from_slice(&0.6_f32.to_le_bytes());
        data[50] = 3;
        data[51] = 0;
        data[52..54].copy_from_slice(&15_u16.to_le_bytes());
        data[54] = 1;
        data[55] = 2;

        let header = header_for(block_ids::GLO_ALM, data.len(), 11000, 2301);
        let block = GloAlmBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 38);
        assert_eq!(block.slot(), 1);
        assert_eq!(block.n, 15);
        assert!((block.clock_bias_s().unwrap() - 0.6).abs() < 1e-6);
    }

    #[test]
    fn test_glo_time_accessors() {
        let block = GloTimeBlock {
            tow_ms: 750,
            wnc: 2400,
            svid: 41,
            freq_nr: 1,
            n_4: 3,
            kp: 2,
            n: 12,
            tau_gps: F32_DNU,
            tau_c: 0.2,
            b1: 0.01,
            b2: 0.02,
        };

        assert!((block.tow_seconds() - 0.75).abs() < 1e-6);
        assert_eq!(block.slot(), 4);
        assert!(block.gps_glonass_offset_s().is_none());
        assert!((block.time_scale_correction_s().unwrap() - 0.2).abs() < 1e-9);
    }

    #[test]
    fn test_glo_time_parse() {
        let mut data = vec![0u8; 38];
        data[12] = 39;
        data[13] = 1;
        data[14] = 5;
        data[15] = 1;
        data[16..18].copy_from_slice(&9_u16.to_le_bytes());
        data[18..22].copy_from_slice(&0.123_f32.to_le_bytes());
        data[22..30].copy_from_slice(&(-0.5_f64).to_le_bytes());
        data[30..34].copy_from_slice(&0.01_f32.to_le_bytes());
        data[34..38].copy_from_slice(&0.02_f32.to_le_bytes());

        let header = header_for(block_ids::GLO_TIME, data.len(), 12000, 2401);
        let block = GloTimeBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 39);
        assert_eq!(block.slot(), 2);
        assert_eq!(block.n, 9);
        assert!((block.time_scale_correction_s().unwrap() + 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_gal_alm_accessors() {
        let block = GalAlmBlock {
            tow_ms: 1500,
            wnc: 2500,
            svid: 71,
            source: 1,
            e: F32_DNU,
            t_oa: 100,
            delta_i: 0.1,
            omega_dot: 0.2,
            delta_sqrt_a: 0.3,
            omega_0: 1.0,
            omega: 1.1,
            m_0: 0.5,
            a_f1: 0.01,
            a_f0: 0.02,
            wn_a: 7,
            svid_a: 72,
            health: 0,
            ioda: 4,
        };

        assert!((block.tow_seconds() - 1.5).abs() < 1e-6);
        assert_eq!(block.prn(), 1);
        assert!(block.eccentricity().is_none());
        assert!((block.delta_sqrt_a().unwrap() - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_gal_alm_parse() {
        let mut data = vec![0u8; 59];
        data[12] = 72;
        data[13] = 2;
        data[14..18].copy_from_slice(&0.02_f32.to_le_bytes());
        data[18..22].copy_from_slice(&500_u32.to_le_bytes());
        data[22..26].copy_from_slice(&0.1_f32.to_le_bytes());
        data[26..30].copy_from_slice(&0.2_f32.to_le_bytes());
        data[30..34].copy_from_slice(&0.3_f32.to_le_bytes());
        data[34..38].copy_from_slice(&1.0_f32.to_le_bytes());
        data[38..42].copy_from_slice(&1.1_f32.to_le_bytes());
        data[42..46].copy_from_slice(&0.5_f32.to_le_bytes());
        data[46..50].copy_from_slice(&0.01_f32.to_le_bytes());
        data[50..54].copy_from_slice(&0.02_f32.to_le_bytes());
        data[54] = 9;
        data[55] = 73;
        data[56..58].copy_from_slice(&0x1234_u16.to_le_bytes());
        data[58] = 6;

        let header = header_for(block_ids::GAL_ALM, data.len(), 13000, 2501);
        let block = GalAlmBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 72);
        assert_eq!(block.prn(), 2);
        assert_eq!(block.wn_a, 9);
        assert_eq!(block.health, 0x1234);
    }

    #[test]
    fn test_gal_ion_accessors() {
        let block = GalIonBlock {
            tow_ms: 1600,
            wnc: 2600,
            svid: 75,
            source: 16,
            a_i0: F32_DNU,
            a_i1: 0.1,
            a_i2: 0.2,
            storm_flags: 1,
        };

        assert!((block.tow_seconds() - 1.6).abs() < 1e-6);
        assert!(block.is_fnav());
        assert!(!block.is_inav());
        assert!(block.a_i0().is_none());
    }

    #[test]
    fn test_gal_ion_parse() {
        let mut data = vec![0u8; 27];
        data[12] = 80;
        data[13] = 1;
        data[14..18].copy_from_slice(&0.1_f32.to_le_bytes());
        data[18..22].copy_from_slice(&0.2_f32.to_le_bytes());
        data[22..26].copy_from_slice(&0.3_f32.to_le_bytes());
        data[26] = 2;

        let header = header_for(block_ids::GAL_ION, data.len(), 14000, 2601);
        let block = GalIonBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 80);
        assert!((block.a_i1 - 0.2).abs() < 1e-6);
        assert_eq!(block.storm_flags, 2);
    }

    #[test]
    fn test_gal_utc_accessors() {
        let block = GalUtcBlock {
            tow_ms: 1700,
            wnc: 2700,
            svid: 76,
            source: 1,
            a_1: 0.001,
            a_0: F64_DNU,
            t_ot: 1000,
            wn_ot: 5,
            delta_t_ls: 18,
            wn_lsf: 6,
            dn: 3,
            delta_t_lsf: 19,
        };

        assert!((block.tow_seconds() - 1.7).abs() < 1e-6);
        assert_eq!(block.prn(), 6);
        assert!(block.utc_bias_s().is_none());
        assert!((block.utc_drift_s_per_s().unwrap() - 0.001).abs() < 1e-6);
    }

    #[test]
    fn test_gal_utc_parse() {
        let mut data = vec![0u8; 35];
        data[12] = 74;
        data[13] = 2;
        data[14..18].copy_from_slice(&0.002_f32.to_le_bytes());
        data[18..26].copy_from_slice(&1.25_f64.to_le_bytes());
        data[26..30].copy_from_slice(&800_u32.to_le_bytes());
        data[30] = 4;
        data[31] = 18u8;
        data[32] = 5;
        data[33] = 2;
        data[34] = 19u8;

        let header = header_for(block_ids::GAL_UTC, data.len(), 15000, 2701);
        let block = GalUtcBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 74);
        assert_eq!(block.wn_ot, 4);
        assert!((block.utc_bias_s().unwrap() - 1.25).abs() < 1e-9);
    }

    #[test]
    fn test_gal_gst_gps_accessors() {
        let block = GalGstGpsBlock {
            tow_ms: 1800,
            wnc: 2800,
            svid: 71,
            source: 1,
            a_1g: F32_DNU,
            a_0g: 0.3,
            t_og: 7,
            wn_og: 8,
        };

        assert!((block.tow_seconds() - 1.8).abs() < 1e-6);
        assert_eq!(block.prn(), 1);
        assert!(block.gst_gps_drift_s_per_s().is_none());
        assert!((block.gst_gps_offset_s().unwrap() - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_gal_gst_gps_parse() {
        let mut data = vec![0u8; 27];
        data[12] = 72;
        data[13] = 0;
        data[14..18].copy_from_slice(&0.01_f32.to_le_bytes());
        data[18..22].copy_from_slice(&0.02_f32.to_le_bytes());
        data[22..26].copy_from_slice(&9_u32.to_le_bytes());
        data[26] = 10;

        let header = header_for(block_ids::GAL_GST_GPS, data.len(), 16000, 2801);
        let block = GalGstGpsBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 72);
        assert_eq!(block.t_og, 9);
        assert_eq!(block.wn_og, 10);
    }

    #[test]
    fn test_gps_cnav_parse() {
        let mut data = vec![0u8; 170];
        data[12] = 12;
        data[13] = 0x80;
        data[14..16].copy_from_slice(&2045_u16.to_le_bytes());
        data[17] = (-2_i8) as u8;
        data[18..22].copy_from_slice(&1000_u32.to_le_bytes());
        data[22..26].copy_from_slice(&2000_u32.to_le_bytes());
        data[50..58].copy_from_slice(&0.5_f64.to_le_bytes());
        data[150..154].copy_from_slice(&(-1.25_f32).to_le_bytes());
        data[166..170].copy_from_slice(&0.0002_f32.to_le_bytes());

        let header = header_for(block_ids::GPS_CNAV, data.len(), 17000, 2900);
        let block = GpsCNavBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 12);
        assert_eq!(block.wn, 2045);
        assert_eq!(block.ura_ed, -2);
        assert_eq!(block.t_oe, 2000);
        assert!((block.m_0 - 0.5).abs() < 1e-12);
        assert!((block.t_gd + 1.25).abs() < 1e-6);
        assert!((block.isc_l5q5 - 0.0002).abs() < 1e-9);
    }

    #[test]
    fn test_bds_ion_parse() {
        let mut data = vec![0u8; 46];
        data[12] = 7;
        data[14..18].copy_from_slice(&0.1_f32.to_le_bytes());
        data[30..34].copy_from_slice(&1.1_f32.to_le_bytes());
        data[42..46].copy_from_slice(&4.4_f32.to_le_bytes());

        let header = header_for(block_ids::BDS_ION, data.len(), 18000, 2901);
        let block = BdsIonBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 7);
        assert!((block.alpha_0 - 0.1).abs() < 1e-6);
        assert!((block.beta_0 - 1.1).abs() < 1e-6);
        assert!((block.beta_3 - 4.4).abs() < 1e-6);
    }

    #[test]
    fn test_bds_cnav1_parse() {
        let mut data = vec![0u8; 158];
        data[12] = 3;
        data[13] = 0x02;
        data[14..18].copy_from_slice(&345600_u32.to_le_bytes());
        data[74..78].copy_from_slice(&1.25_f32.to_le_bytes());
        data[143] = 9;
        data[144..146].copy_from_slice(&512_u16.to_le_bytes());
        data[146..150].copy_from_slice(&0.001_f32.to_le_bytes());
        data[154..158].copy_from_slice(&(-0.002_f32).to_le_bytes());

        let header = header_for(block_ids::BDS_CNAV1, data.len(), 19000, 2902);
        let block = BdsCNav1Block::parse(&header, &data).unwrap();

        assert_eq!(block.prn_idx, 3);
        assert_eq!(block.flags & 0x03, 0x02);
        assert_eq!(block.t_oe, 345600);
        assert!((block.omega_dot - 1.25).abs() < 1e-6);
        assert_eq!(block.iode, 9);
        assert_eq!(block.iodc, 512);
        assert!((block.isc_b1cd - 0.001).abs() < 1e-6);
        assert!((block.t_gd_b2ap + 0.002).abs() < 1e-6);
    }

    #[test]
    fn test_geo_iono_delay_accessors() {
        let block = GeoIonoDelayBlock {
            tow_ms: 2500,
            wnc: 2100,
            prn: 120,
            band_nbr: 3,
            iodi: 5,
            idc: vec![GeoIonoDelayIdc {
                igp_mask_no: 10,
                givei: 4,
                vertical_delay_m_raw: F32_DNU,
            }],
        };

        assert!((block.tow_seconds() - 2.5).abs() < 1e-6);
        assert_eq!(block.num_idc(), 1);
        assert!(block.idc[0].vertical_delay_m().is_none());
    }

    #[test]
    fn test_geo_iono_delay_parse() {
        let mut data = vec![0u8; 18 + (2 * 8)];
        data[12] = 120; // PRN
        data[13] = 3; // BandNbr
        data[14] = 5; // IODI
        data[15] = 2; // N
        data[16] = 8; // SBLength
        data[17] = 0; // Reserved

        // IDC #1
        data[18] = 10; // IGPMaskNo
        data[19] = 4; // GIVEI
        data[22..26].copy_from_slice(&12.5_f32.to_le_bytes()); // VerticalDelay

        // IDC #2
        data[26] = 11; // IGPMaskNo
        data[27] = 5; // GIVEI
        data[30..34].copy_from_slice(&F32_DNU.to_le_bytes()); // VerticalDelay DNU

        let header = header_for(block_ids::GEO_IONO_DELAY, data.len(), 4321, 2024);
        let block = GeoIonoDelayBlock::parse(&header, &data).unwrap();

        assert_eq!(block.prn, 120);
        assert_eq!(block.band_nbr, 3);
        assert_eq!(block.iodi, 5);
        assert_eq!(block.num_idc(), 2);
        assert_eq!(block.idc[0].igp_mask_no, 10);
        assert_eq!(block.idc[1].givei, 5);
        assert!((block.idc[0].vertical_delay_m().unwrap() - 12.5).abs() < 1e-6);
        assert!(block.idc[1].vertical_delay_m().is_none());
    }

    #[test]
    fn test_geo_iono_delay_sbf_block_parse() {
        let total_len = 36usize; // sync + header + payload (+padding)
        let mut data = vec![0u8; total_len];
        data[0..2].copy_from_slice(&SBF_SYNC);
        data[2..4].copy_from_slice(&0_u16.to_le_bytes()); // CRC
        data[4..6].copy_from_slice(&block_ids::GEO_IONO_DELAY.to_le_bytes()); // ID/Rev
        data[6..8].copy_from_slice(&(total_len as u16).to_le_bytes()); // Length
        data[8..12].copy_from_slice(&9876_u32.to_le_bytes()); // TOW
        data[12..14].copy_from_slice(&2025_u16.to_le_bytes()); // WNc

        // Payload starts at absolute offset 14 (block_data offset 12)
        data[14] = 120; // PRN
        data[15] = 2; // BandNbr
        data[16] = 7; // IODI
        data[17] = 2; // N
        data[18] = 8; // SBLength
        data[19] = 0; // Reserved

        // IDC #1
        data[20] = 10; // IGPMaskNo
        data[21] = 3; // GIVEI
        data[24..28].copy_from_slice(&8.25_f32.to_le_bytes());

        // IDC #2
        data[28] = 11; // IGPMaskNo
        data[29] = 6; // GIVEI
        data[32..36].copy_from_slice(&9.75_f32.to_le_bytes());

        let (block, used) = SbfBlock::parse(&data).unwrap();
        assert_eq!(used, total_len);
        assert_eq!(block.block_id(), block_ids::GEO_IONO_DELAY);
        match block {
            SbfBlock::GeoIonoDelay(geo) => {
                assert_eq!(geo.tow_ms(), 9876);
                assert_eq!(geo.wnc(), 2025);
                assert_eq!(geo.num_idc(), 2);
                assert!((geo.idc[0].vertical_delay_m().unwrap() - 8.25).abs() < 1e-6);
                assert!((geo.idc[1].vertical_delay_m().unwrap() - 9.75).abs() < 1e-6);
            }
            _ => panic!("Expected GeoIonoDelay block"),
        }
    }

    #[test]
    fn test_gps_raw_ca_parse() {
        let header = header_for(4017, 57, 5000, 2150);
        let mut data = vec![0u8; 57];
        data[12] = 12; // SVID
        data[13] = 1; // CRCPassed
        data[14] = 0; // ViterbiCount
        data[15] = 2; // Source
        data[16] = 0; // FreqNr
        data[17..57].copy_from_slice(&[0xABu8; 40]); // NAVBits

        let block = GpsRawCaBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2150);
        assert_eq!(block.svid, 12);
        assert!(block.crc_ok());
        assert_eq!(block.viterbi_count, 0);
        assert_eq!(block.nav_bits_slice()[0], 0xAB);
    }

    #[test]
    fn test_gps_raw_ca_too_short() {
        let header = header_for(4017, 57, 0, 0);
        let data = [0u8; 50];
        assert!(GpsRawCaBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_gps_raw_l2c_parse() {
        let header = header_for(4018, 57, 6000, 2200);
        let mut data = vec![0u8; 57];
        data[12] = 8;
        data[13] = 1;
        data[17..57].copy_from_slice(&[0xCDu8; 40]);

        let block = GpsRawL2CBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 6.0);
        assert_eq!(block.svid, 8);
        assert!(block.crc_ok());
        assert_eq!(block.nav_bits_slice()[0], 0xCD);
    }

    #[test]
    fn test_gps_raw_l5_parse() {
        let header = header_for(4019, 57, 7000, 2250);
        let mut data = vec![0u8; 57];
        data[12] = 15;
        data[13] = 0;

        let block = GpsRawL5Block::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 7.0);
        assert_eq!(block.svid, 15);
        assert!(!block.crc_ok());
    }

    #[test]
    fn test_glo_raw_ca_parse() {
        let header = header_for(4026, 29, 8000, 2300);
        let mut data = vec![0u8; 29];
        data[12] = 45;
        data[13] = 1;
        data[16] = 3;
        data[17..29].copy_from_slice(&[0x12u8; 12]);

        let block = GloRawCaBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 8.0);
        assert_eq!(block.svid, 45);
        assert_eq!(block.freq_nr, 3);
        assert_eq!(block.nav_bits_slice()[0], 0x12);
    }

    #[test]
    fn test_glo_raw_ca_too_short() {
        let header = header_for(4026, 29, 0, 0);
        let data = [0u8; 25];
        assert!(GloRawCaBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_gal_raw_fnav_parse() {
        let header = header_for(4022, 49, 1000, 2100);
        let mut data = vec![0u8; 49];
        data[12] = 85; // Galileo SVID
        data[13] = 1;
        data[17..49].copy_from_slice(&[0x11u8; 32]);
        let block = GalRawFnavBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.svid, 85);
        assert!(block.crc_ok());
        assert_eq!(block.nav_bits_slice()[0], 0x11);
    }

    #[test]
    fn test_gal_raw_inav_parse() {
        let header = header_for(4023, 49, 2000, 2101);
        let mut data = vec![0u8; 49];
        data[12] = 72;
        data[13] = 0;
        let block = GalRawInavBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 2.0);
        assert!(!block.crc_ok());
    }

    #[test]
    fn test_gal_raw_cnav_parse() {
        let header = header_for(4024, 81, 3000, 2102);
        let mut data = vec![0u8; 81];
        data[12] = 90;
        data[17..81].copy_from_slice(&[0x22u8; 64]);
        let block = GalRawCnavBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 3.0);
        assert_eq!(block.nav_bits_slice()[63], 0x22);
    }

    #[test]
    fn test_geo_raw_l1_parse() {
        let header = header_for(4020, 49, 4000, 2103);
        let mut data = vec![0u8; 49];
        data[12] = 135; // SBAS PRN
        data[13] = 1;
        data[17..49].copy_from_slice(&[0x33u8; 32]);
        let block = GeoRawL1Block::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 4.0);
        assert_eq!(block.svid, 135);
    }

    #[test]
    fn test_cmp_raw_parse() {
        let header = header_for(4047, 57, 5000, 2104);
        let mut data = vec![0u8; 57];
        data[12] = 155; // BeiDou SVID
        data[17..57].copy_from_slice(&[0x44u8; 40]);
        let block = CmpRawBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.nav_bits_slice()[39], 0x44);
    }

    #[test]
    fn test_qzs_raw_l1ca_parse() {
        let header = header_for(4066, 57, 6000, 2105);
        let mut data = vec![0u8; 57];
        data[12] = 183; // QZSS SVID
        data[13] = 1;
        let block = QzsRawL1CaBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 6.0);
        assert_eq!(block.svid, 183);
    }

    #[test]
    fn test_qzs_raw_l2c_parse() {
        let header = header_for(4067, 57, 7000, 2106);
        let mut data = vec![0u8; 57];
        data[12] = 186;
        let block = QzsRawL2CBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 7.0);
    }

    #[test]
    fn test_qzs_raw_l5_parse() {
        let header = header_for(4068, 57, 8000, 2107);
        let mut data = vec![0u8; 57];
        data[12] = 181;
        let block = QzsRawL5Block::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 8.0);
    }

    #[test]
    fn test_gal_raw_fnav_too_short() {
        let header = header_for(4022, 49, 0, 0);
        assert!(GalRawFnavBlock::parse(&header, &[0u8; 40]).is_err());
    }

    #[test]
    fn test_gal_raw_cnav_too_short() {
        let header = header_for(4024, 81, 0, 0);
        assert!(GalRawCnavBlock::parse(&header, &[0u8; 70]).is_err());
    }

    #[test]
    fn test_gal_sar_rlm_accessors() {
        let block = GalSarRlmBlock {
            tow_ms: 4500,
            wnc: 2200,
            svid: 74,
            source: 2,
            rlm_length_bits: 80,
            rlm_bits_words: vec![0x8000_0000, 0, 0x0001_0000],
        };

        assert!((block.tow_seconds() - 4.5).abs() < 1e-6);
        assert_eq!(block.prn(), 4);
        assert_eq!(block.rlm_length_bits(), 80);
        assert_eq!(block.rlm_bits_words().len(), 3);
        assert_eq!(block.bit(0), Some(true));
        assert_eq!(block.bit(1), Some(false));
        assert_eq!(block.bit(79), Some(true));
        assert_eq!(block.bit(80), None);
    }

    #[test]
    fn test_gal_sar_rlm_parse() {
        let mut data = vec![0u8; 30];
        data[12] = 72; // SVID
        data[13] = 16; // Source (F/NAV)
        data[14] = 80; // RLMLength in bits
        data[18..22].copy_from_slice(&0x1234_5678_u32.to_le_bytes());
        data[22..26].copy_from_slice(&0x9abc_def0_u32.to_le_bytes());
        data[26..30].copy_from_slice(&0x0001_0000_u32.to_le_bytes());

        let header = header_for(block_ids::GAL_SAR_RLM, data.len(), 3210, 2048);
        let block = GalSarRlmBlock::parse(&header, &data).unwrap();

        assert_eq!(block.svid, 72);
        assert_eq!(block.source, 16);
        assert_eq!(block.rlm_length_bits(), 80);
        assert_eq!(
            block.rlm_bits_words(),
            &[0x1234_5678, 0x9abc_def0, 0x0001_0000]
        );
        assert_eq!(block.bit(79), Some(true));
    }

    #[test]
    fn test_gal_sar_rlm_too_short() {
        let header = header_for(block_ids::GAL_SAR_RLM, 30, 0, 0);
        assert!(GalSarRlmBlock::parse(&header, &[0u8; 20]).is_err());
    }
}
