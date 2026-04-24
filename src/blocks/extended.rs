//! Additional SBF blocks: extra raw navigation pages, BeiDou/QZSS decoded nav, local/projected position.
//!
//! Layouts follow Septentrio SBF definitions (public reference headers such as PointOneNav `sbfdef.h`).

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::dnu::{f32_or_none, f64_or_none};
use super::navigation::GpsNavBlock;
use super::SbfBlockParse;

// --- Raw navigation (RxChannel + u32 NAVBits array) ---------------------------------

/// GEORawL5 (4021) — SBAS L5 navigation message.
#[derive(Debug, Clone)]
pub struct GeoRawL5Block {
    tow_ms: u32,
    wnc: u16,
    pub svid: u8,
    pub crc_status: u8,
    pub viterbi_count: u8,
    pub source: u8,
    pub freq_nr: u8,
    pub rx_channel: u8,
    /// Raw `NAVBits` payload (`16` × `u32` = 64 bytes).
    pub nav_bits: [u8; 64],
}

impl GeoRawL5Block {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_status != 0
    }
}

impl SbfBlockParse for GeoRawL5Block {
    const BLOCK_ID: u16 = block_ids::GEO_RAW_L5;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 82;
        if data.len() < MIN {
            return Err(SbfError::ParseError("GEORawL5 too short".into()));
        }
        let mut nav_bits = [0u8; 64];
        nav_bits.copy_from_slice(&data[18..82]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_status: data[13],
            viterbi_count: data[14],
            source: data[15],
            freq_nr: data[16],
            rx_channel: data[17],
            nav_bits,
        })
    }
}

/// BDSRawB1C (4218) — BeiDou B1C navigation frame.
#[derive(Debug, Clone)]
pub struct BdsRawB1cBlock {
    tow_ms: u32,
    wnc: u16,
    pub svid: u8,
    pub crc_sf2: u8,
    pub crc_sf3: u8,
    pub source: u8,
    pub reserved: u8,
    pub rx_channel: u8,
    /// `57` × `u32` = 228 bytes.
    pub nav_bits: [u8; 228],
}

impl BdsRawB1cBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
}

impl SbfBlockParse for BdsRawB1cBlock {
    const BLOCK_ID: u16 = block_ids::BDS_RAW_B1C;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 246;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSRawB1C too short".into()));
        }
        let mut nav_bits = [0u8; 228];
        nav_bits.copy_from_slice(&data[18..246]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_sf2: data[13],
            crc_sf3: data[14],
            source: data[15],
            reserved: data[16],
            rx_channel: data[17],
            nav_bits,
        })
    }
}

/// BDSRawB2a (4219) — BeiDou B2a navigation frame.
#[derive(Debug, Clone)]
pub struct BdsRawB2aBlock {
    tow_ms: u32,
    wnc: u16,
    pub svid: u8,
    pub crc_passed: u8,
    pub viterbi_count: u8,
    pub source: u8,
    pub reserved: u8,
    pub rx_channel: u8,
    /// `18` × `u32` = 72 bytes.
    pub nav_bits: [u8; 72],
}

impl BdsRawB2aBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
}

impl SbfBlockParse for BdsRawB2aBlock {
    const BLOCK_ID: u16 = block_ids::BDS_RAW_B2A;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 90;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSRawB2a too short".into()));
        }
        let mut nav_bits = [0u8; 72];
        nav_bits.copy_from_slice(&data[18..90]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            reserved: data[16],
            rx_channel: data[17],
            nav_bits,
        })
    }
}

/// BDSRawB2b (4242) — BeiDou B2b navigation frame.
#[derive(Debug, Clone)]
pub struct BdsRawB2bBlock {
    tow_ms: u32,
    wnc: u16,
    pub svid: u8,
    pub crc_passed: u8,
    pub reserved1: u8,
    pub source: u8,
    pub reserved2: u8,
    pub rx_channel: u8,
    /// `31` × `u32` = 124 bytes.
    pub nav_bits: [u8; 124],
}

impl BdsRawB2bBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
}

impl SbfBlockParse for BdsRawB2bBlock {
    const BLOCK_ID: u16 = block_ids::BDS_RAW_B2B;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 142;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSRawB2b too short".into()));
        }
        let mut nav_bits = [0u8; 124];
        nav_bits.copy_from_slice(&data[18..142]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            reserved1: data[14],
            source: data[15],
            reserved2: data[16],
            rx_channel: data[17],
            nav_bits,
        })
    }
}

/// IRNSSRaw / NAVICRaw (4093) — NavIC/IRNSS subframe.
#[derive(Debug, Clone)]
pub struct IrnssRawBlock {
    tow_ms: u32,
    wnc: u16,
    pub svid: u8,
    pub crc_passed: u8,
    pub viterbi_count: u8,
    pub source: u8,
    pub reserved: u8,
    pub rx_channel: u8,
    /// `10` × `u32` = 40 bytes.
    pub nav_bits: [u8; 40],
}

impl IrnssRawBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn crc_ok(&self) -> bool {
        self.crc_passed != 0
    }
}

impl SbfBlockParse for IrnssRawBlock {
    const BLOCK_ID: u16 = block_ids::NAVIC_RAW;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 58;
        if data.len() < MIN {
            return Err(SbfError::ParseError("IRNSSRaw too short".into()));
        }
        let mut nav_bits = [0u8; 40];
        nav_bits.copy_from_slice(&data[18..58]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            svid: data[12],
            crc_passed: data[13],
            viterbi_count: data[14],
            source: data[15],
            reserved: data[16],
            rx_channel: data[17],
            nav_bits,
        })
    }
}

// --- Position --------------------------------------------------------------------

/// PosLocal (4052) — position in a local datum.
#[derive(Debug, Clone)]
pub struct PosLocalBlock {
    tow_ms: u32,
    wnc: u16,
    pub mode: u8,
    pub error: u8,
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub height_m: f64,
    pub datum: u8,
}

impl PosLocalBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
}

impl SbfBlockParse for PosLocalBlock {
    const BLOCK_ID: u16 = block_ids::POS_LOCAL;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 42;
        if data.len() < MIN {
            return Err(SbfError::ParseError("PosLocal too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode: data[12],
            error: data[13],
            latitude_rad: f64::from_le_bytes(data[14..22].try_into().unwrap()),
            longitude_rad: f64::from_le_bytes(data[22..30].try_into().unwrap()),
            height_m: f64::from_le_bytes(data[30..38].try_into().unwrap()),
            datum: data[38],
        })
    }
}

/// PosProjected (4094) — plane grid coordinates.
#[derive(Debug, Clone)]
pub struct PosProjectedBlock {
    tow_ms: u32,
    wnc: u16,
    pub mode: u8,
    pub error: u8,
    pub northing_m: f64,
    pub easting_m: f64,
    pub height_m: f64,
    pub datum: u8,
}

impl PosProjectedBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
}

impl SbfBlockParse for PosProjectedBlock {
    const BLOCK_ID: u16 = block_ids::POS_PROJECTED;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 42;
        if data.len() < MIN {
            return Err(SbfError::ParseError("PosProjected too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode: data[12],
            error: data[13],
            northing_m: f64::from_le_bytes(data[14..22].try_into().unwrap()),
            easting_m: f64::from_le_bytes(data[22..30].try_into().unwrap()),
            height_m: f64::from_le_bytes(data[30..38].try_into().unwrap()),
            datum: data[38],
        })
    }
}

// --- Decoded BeiDou ephemeris (cmpEph) --------------------------------------------

/// BDSNav (4081) — BeiDou ephemeris and clock (`cmpEph`).
#[derive(Debug, Clone)]
pub struct BdsNavBlock {
    tow_ms: u32,
    wnc: u16,
    pub prn: u8,
    pub wn: u16,
    pub ura: u8,
    pub sat_h1: u8,
    pub iodc: u8,
    pub iode: u8,
    pub t_gd1_s: f32,
    pub t_gd2_s: f32,
    pub t_oc: u32,
    pub a_f2: f32,
    pub a_f1: f32,
    pub a_f0: f32,
    pub c_rs: f32,
    pub delta_n: f32,
    pub m_0: f64,
    pub c_uc: f32,
    pub e: f64,
    pub c_us: f32,
    pub sqrt_a: f64,
    pub t_oe: u32,
    pub c_ic: f32,
    pub omega_0: f64,
    pub c_is: f32,
    pub i_0: f64,
    pub c_rc: f32,
    pub omega: f64,
    pub omega_dot: f32,
    pub i_dot: f32,
    pub wn_t_oc: u16,
    pub wn_t_oe: u16,
}

impl BdsNavBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn t_gd1_s_opt(&self) -> Option<f32> {
        f32_or_none(self.t_gd1_s)
    }
    pub fn t_gd2_s_opt(&self) -> Option<f32> {
        f32_or_none(self.t_gd2_s)
    }
}

impl SbfBlockParse for BdsNavBlock {
    const BLOCK_ID: u16 = block_ids::BDS_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 138 {
            return Err(SbfError::ParseError("BDSNav too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn: data[12],
            wn: u16::from_le_bytes([data[14], data[15]]),
            ura: data[16],
            sat_h1: data[17],
            iodc: data[18],
            iode: data[19],
            t_gd1_s: f32::from_le_bytes(data[22..26].try_into().unwrap()),
            t_gd2_s: f32::from_le_bytes(data[26..30].try_into().unwrap()),
            t_oc: u32::from_le_bytes(data[30..34].try_into().unwrap()),
            a_f2: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            a_f1: f32::from_le_bytes(data[38..42].try_into().unwrap()),
            a_f0: f32::from_le_bytes(data[42..46].try_into().unwrap()),
            c_rs: f32::from_le_bytes(data[46..50].try_into().unwrap()),
            delta_n: f32::from_le_bytes(data[50..54].try_into().unwrap()),
            m_0: f64::from_le_bytes(data[54..62].try_into().unwrap()),
            c_uc: f32::from_le_bytes(data[62..66].try_into().unwrap()),
            e: f64::from_le_bytes(data[66..74].try_into().unwrap()),
            c_us: f32::from_le_bytes(data[74..78].try_into().unwrap()),
            sqrt_a: f64::from_le_bytes(data[78..86].try_into().unwrap()),
            t_oe: u32::from_le_bytes(data[86..90].try_into().unwrap()),
            c_ic: f32::from_le_bytes(data[90..94].try_into().unwrap()),
            omega_0: f64::from_le_bytes(data[94..102].try_into().unwrap()),
            c_is: f32::from_le_bytes(data[102..106].try_into().unwrap()),
            i_0: f64::from_le_bytes(data[106..114].try_into().unwrap()),
            c_rc: f32::from_le_bytes(data[114..118].try_into().unwrap()),
            omega: f64::from_le_bytes(data[118..126].try_into().unwrap()),
            omega_dot: f32::from_le_bytes(data[126..130].try_into().unwrap()),
            i_dot: f32::from_le_bytes(data[130..134].try_into().unwrap()),
            wn_t_oc: u16::from_le_bytes([data[134], data[135]]),
            wn_t_oe: u16::from_le_bytes([data[136], data[137]]),
        })
    }
}

/// QZSNav (4095) — QZSS ephemeris and clock (same binary layout as [`GpsNavBlock`] / `GPSNav`).
#[derive(Debug, Clone)]
pub struct QzsNavBlock(pub GpsNavBlock);

impl std::ops::Deref for QzsNavBlock {
    type Target = GpsNavBlock;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SbfBlockParse for QzsNavBlock {
    const BLOCK_ID: u16 = block_ids::QZS_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        Ok(QzsNavBlock(GpsNavBlock::parse(header, data)?))
    }
}

/// BDSAlm (4119) — BeiDou almanac.
#[derive(Debug, Clone)]
pub struct BdsAlmBlock {
    tow_ms: u32,
    wnc: u16,
    pub prn: u8,
    pub wn_a: u8,
    pub t_oa: u32,
    pub sqrt_a: f32,
    pub e: f32,
    pub omega: f32,
    pub m_0: f32,
    pub omega_0: f32,
    pub omega_dot: f32,
    pub delta_i: f32,
    pub a_f0: f32,
    pub a_f1: f32,
    pub health: u16,
}

impl BdsAlmBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
}

impl SbfBlockParse for BdsAlmBlock {
    const BLOCK_ID: u16 = block_ids::BDS_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 56;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSAlm too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn: data[12],
            wn_a: data[13],
            t_oa: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            sqrt_a: f32::from_le_bytes(data[18..22].try_into().unwrap()),
            e: f32::from_le_bytes(data[22..26].try_into().unwrap()),
            omega: f32::from_le_bytes(data[26..30].try_into().unwrap()),
            m_0: f32::from_le_bytes(data[30..34].try_into().unwrap()),
            omega_0: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            omega_dot: f32::from_le_bytes(data[38..42].try_into().unwrap()),
            delta_i: f32::from_le_bytes(data[42..46].try_into().unwrap()),
            a_f0: f32::from_le_bytes(data[46..50].try_into().unwrap()),
            a_f1: f32::from_le_bytes(data[50..54].try_into().unwrap()),
            health: u16::from_le_bytes(data[54..56].try_into().unwrap()),
        })
    }
}

/// QZSAlm (4116) — QZSS almanac.
#[derive(Debug, Clone)]
pub struct QzsAlmBlock {
    tow_ms: u32,
    wnc: u16,
    pub prn: u8,
    pub e: f32,
    pub t_oa: u32,
    pub delta_i: f32,
    pub omega_dot: f32,
    pub sqrt_a: f32,
    pub omega_0: f32,
    pub omega: f32,
    pub m_0: f32,
    pub a_f1: f32,
    pub a_f0: f32,
    pub wn_a: u8,
    pub health8: u8,
    pub health6: u8,
}

impl QzsAlmBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
}

impl SbfBlockParse for QzsAlmBlock {
    const BLOCK_ID: u16 = block_ids::QZS_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 58;
        if data.len() < MIN {
            return Err(SbfError::ParseError("QZSAlm too short".into()));
        }
        // The public QZSAlm layout includes reserved bytes after `PRN` and `WN_a`.
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn: data[12],
            e: f32::from_le_bytes(data[14..18].try_into().unwrap()),
            t_oa: u32::from_le_bytes(data[18..22].try_into().unwrap()),
            delta_i: f32::from_le_bytes(data[22..26].try_into().unwrap()),
            omega_dot: f32::from_le_bytes(data[26..30].try_into().unwrap()),
            sqrt_a: f32::from_le_bytes(data[30..34].try_into().unwrap()),
            omega_0: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            omega: f32::from_le_bytes(data[38..42].try_into().unwrap()),
            m_0: f32::from_le_bytes(data[42..46].try_into().unwrap()),
            a_f1: f32::from_le_bytes(data[46..50].try_into().unwrap()),
            a_f0: f32::from_le_bytes(data[50..54].try_into().unwrap()),
            wn_a: data[54],
            health8: data[56],
            health6: data[57],
        })
    }
}

/// BDSCNav2 (4252) — BeiDou B-CNAV2 ephemeris from the B2a signal.
#[derive(Debug, Clone)]
pub struct BdsCNav2Block {
    tow_ms: u32,
    wnc: u16,
    pub prn_idx: u8,
    pub flags: u8,
    pub t_oe: u32,
    pub a: f64,
    pub a_dot: f64,
    pub delta_n0: f32,
    pub delta_n0_dot: f32,
    pub m_0: f64,
    pub e: f64,
    pub omega: f64,
    pub omega_0: f64,
    pub omega_dot: f32,
    pub i_0: f64,
    pub i_dot: f32,
    pub c_is: f32,
    pub c_ic: f32,
    pub c_rs: f32,
    pub c_rc: f32,
    pub c_us: f32,
    pub c_uc: f32,
    pub t_oc: u32,
    pub a_2: f32,
    pub a_1: f32,
    pub a_0: f64,
    pub t_op: u32,
    pub sisai_ocb: u8,
    pub sisai_oc12: u8,
    pub sisai_oe: u8,
    pub sismai: u8,
    pub health_if: u8,
    pub iode: u8,
    pub iodc: u16,
    pub isc_b2ad: f32,
    pub t_gd_b2ap: f32,
    pub t_gd_b1cp: f32,
}

impl BdsCNav2Block {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn satellite_type(&self) -> u8 {
        self.flags & 0x03
    }
    pub fn is_healthy(&self) -> bool {
        (self.health_if & 0xC0) == 0
    }
    pub fn isc_b2ad_s(&self) -> Option<f32> {
        f32_or_none(self.isc_b2ad)
    }
    pub fn t_gd_b2ap_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd_b2ap)
    }
    pub fn t_gd_b1cp_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd_b1cp)
    }
}

impl SbfBlockParse for BdsCNav2Block {
    const BLOCK_ID: u16 = block_ids::BDS_CNAV2;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 158;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSCNav2 too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn_idx: data[12],
            flags: data[13],
            t_oe: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            a: f64::from_le_bytes(data[18..26].try_into().unwrap()),
            a_dot: f64::from_le_bytes(data[26..34].try_into().unwrap()),
            delta_n0: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            delta_n0_dot: f32::from_le_bytes(data[38..42].try_into().unwrap()),
            m_0: f64::from_le_bytes(data[42..50].try_into().unwrap()),
            e: f64::from_le_bytes(data[50..58].try_into().unwrap()),
            omega: f64::from_le_bytes(data[58..66].try_into().unwrap()),
            omega_0: f64::from_le_bytes(data[66..74].try_into().unwrap()),
            omega_dot: f32::from_le_bytes(data[74..78].try_into().unwrap()),
            i_0: f64::from_le_bytes(data[78..86].try_into().unwrap()),
            i_dot: f32::from_le_bytes(data[86..90].try_into().unwrap()),
            c_is: f32::from_le_bytes(data[90..94].try_into().unwrap()),
            c_ic: f32::from_le_bytes(data[94..98].try_into().unwrap()),
            c_rs: f32::from_le_bytes(data[98..102].try_into().unwrap()),
            c_rc: f32::from_le_bytes(data[102..106].try_into().unwrap()),
            c_us: f32::from_le_bytes(data[106..110].try_into().unwrap()),
            c_uc: f32::from_le_bytes(data[110..114].try_into().unwrap()),
            t_oc: u32::from_le_bytes(data[114..118].try_into().unwrap()),
            a_2: f32::from_le_bytes(data[118..122].try_into().unwrap()),
            a_1: f32::from_le_bytes(data[122..126].try_into().unwrap()),
            a_0: f64::from_le_bytes(data[126..134].try_into().unwrap()),
            t_op: u32::from_le_bytes(data[134..138].try_into().unwrap()),
            sisai_ocb: data[138],
            sisai_oc12: data[139],
            sisai_oe: data[140],
            sismai: data[141],
            health_if: data[142],
            iode: data[143],
            iodc: u16::from_le_bytes(data[144..146].try_into().unwrap()),
            isc_b2ad: f32::from_le_bytes(data[146..150].try_into().unwrap()),
            t_gd_b2ap: f32::from_le_bytes(data[150..154].try_into().unwrap()),
            t_gd_b1cp: f32::from_le_bytes(data[154..158].try_into().unwrap()),
        })
    }
}

/// BDSCNav3 (4253) — BeiDou B-CNAV3 ephemeris from the B2b_I signal.
#[derive(Debug, Clone)]
pub struct BdsCNav3Block {
    tow_ms: u32,
    wnc: u16,
    pub prn_idx: u8,
    pub flags: u8,
    pub t_oe: u32,
    pub a: f64,
    pub a_dot: f64,
    pub delta_n0: f32,
    pub delta_n0_dot: f32,
    pub m_0: f64,
    pub e: f64,
    pub omega: f64,
    pub omega_0: f64,
    pub omega_dot: f32,
    pub i_0: f64,
    pub i_dot: f32,
    pub c_is: f32,
    pub c_ic: f32,
    pub c_rs: f32,
    pub c_rc: f32,
    pub c_us: f32,
    pub c_uc: f32,
    pub t_oc: u32,
    pub a_2: f32,
    pub a_1: f32,
    pub a_0: f64,
    pub t_op: u32,
    pub sisai_ocb: u8,
    pub sisai_oc12: u8,
    pub sisai_oe: u8,
    pub sismai: u8,
    pub health_if: u8,
    pub reserved: [u8; 3],
    pub t_gd_b2bi: f32,
}

impl BdsCNav3Block {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn satellite_type(&self) -> u8 {
        self.flags & 0x03
    }
    pub fn is_healthy(&self) -> bool {
        (self.health_if & 0xC0) == 0
    }
    pub fn t_gd_b2bi_s(&self) -> Option<f32> {
        f32_or_none(self.t_gd_b2bi)
    }
}

impl SbfBlockParse for BdsCNav3Block {
    const BLOCK_ID: u16 = block_ids::BDS_CNAV3;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 150;
        if data.len() < MIN {
            return Err(SbfError::ParseError("BDSCNav3 too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn_idx: data[12],
            flags: data[13],
            t_oe: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            a: f64::from_le_bytes(data[18..26].try_into().unwrap()),
            a_dot: f64::from_le_bytes(data[26..34].try_into().unwrap()),
            delta_n0: f32::from_le_bytes(data[34..38].try_into().unwrap()),
            delta_n0_dot: f32::from_le_bytes(data[38..42].try_into().unwrap()),
            m_0: f64::from_le_bytes(data[42..50].try_into().unwrap()),
            e: f64::from_le_bytes(data[50..58].try_into().unwrap()),
            omega: f64::from_le_bytes(data[58..66].try_into().unwrap()),
            omega_0: f64::from_le_bytes(data[66..74].try_into().unwrap()),
            omega_dot: f32::from_le_bytes(data[74..78].try_into().unwrap()),
            i_0: f64::from_le_bytes(data[78..86].try_into().unwrap()),
            i_dot: f32::from_le_bytes(data[86..90].try_into().unwrap()),
            c_is: f32::from_le_bytes(data[90..94].try_into().unwrap()),
            c_ic: f32::from_le_bytes(data[94..98].try_into().unwrap()),
            c_rs: f32::from_le_bytes(data[98..102].try_into().unwrap()),
            c_rc: f32::from_le_bytes(data[102..106].try_into().unwrap()),
            c_us: f32::from_le_bytes(data[106..110].try_into().unwrap()),
            c_uc: f32::from_le_bytes(data[110..114].try_into().unwrap()),
            t_oc: u32::from_le_bytes(data[114..118].try_into().unwrap()),
            a_2: f32::from_le_bytes(data[118..122].try_into().unwrap()),
            a_1: f32::from_le_bytes(data[122..126].try_into().unwrap()),
            a_0: f64::from_le_bytes(data[126..134].try_into().unwrap()),
            t_op: u32::from_le_bytes(data[134..138].try_into().unwrap()),
            sisai_ocb: data[138],
            sisai_oc12: data[139],
            sisai_oe: data[140],
            sismai: data[141],
            health_if: data[142],
            reserved: data[143..146].try_into().unwrap(),
            t_gd_b2bi: f32::from_le_bytes(data[146..150].try_into().unwrap()),
        })
    }
}

/// BDSUtc (4121) — BDT-UTC parameters (`cmpUtc`).
#[derive(Debug, Clone)]
pub struct BdsUtcBlock {
    tow_ms: u32,
    wnc: u16,
    pub prn: u8,
    pub a_1: f32,
    pub a_0: f64,
    pub delta_t_ls: i8,
    pub wn_lsf: u8,
    pub dn: u8,
    pub delta_t_lsf: i8,
}

impl BdsUtcBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn a_1_opt(&self) -> Option<f32> {
        f32_or_none(self.a_1)
    }
    pub fn a_0_opt(&self) -> Option<f64> {
        f64_or_none(self.a_0)
    }
}

impl SbfBlockParse for BdsUtcBlock {
    const BLOCK_ID: u16 = block_ids::BDS_UTC;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 30 {
            return Err(SbfError::ParseError("BDSUtc too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn: data[12],
            a_1: f32::from_le_bytes(data[14..18].try_into().unwrap()),
            a_0: f64::from_le_bytes(data[18..26].try_into().unwrap()),
            delta_t_ls: data[26] as i8,
            wn_lsf: data[27],
            dn: data[28],
            delta_t_lsf: data[29] as i8,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pos_local_roundtrip_minimal() {
        let mut d = vec![0u8; 44];
        d[6..10].copy_from_slice(&1000u32.to_le_bytes());
        d[10..12].copy_from_slice(&200u16.to_le_bytes());
        d[12] = 1;
        d[13] = 0;
        d[14..22].copy_from_slice(&1.0f64.to_le_bytes());
        d[22..30].copy_from_slice(&2.0f64.to_le_bytes());
        d[30..38].copy_from_slice(&3.0f64.to_le_bytes());
        d[38] = 5;
        let h = SbfHeader {
            crc: 0,
            block_id: block_ids::POS_LOCAL,
            block_rev: 0,
            length: 46,
            tow_ms: 1000,
            wnc: 200,
        };
        let p = PosLocalBlock::parse(&h, &d).unwrap();
        assert_eq!(p.latitude_rad, 1.0);
        assert_eq!(p.datum, 5);
    }

    #[test]
    fn bds_alm_parse_minimal() {
        let mut d = vec![0u8; 56];
        d[12] = 12;
        d[13] = 34;
        d[14..18].copy_from_slice(&5678u32.to_le_bytes());
        d[18..22].copy_from_slice(&5153.5f32.to_le_bytes());
        d[54..56].copy_from_slice(&0x0123u16.to_le_bytes());
        let h = SbfHeader {
            crc: 0,
            block_id: block_ids::BDS_ALM,
            block_rev: 0,
            length: (d.len() + 2) as u16,
            tow_ms: 1000,
            wnc: 200,
        };
        let block = BdsAlmBlock::parse(&h, &d).unwrap();
        assert_eq!(block.prn, 12);
        assert_eq!(block.wn_a, 34);
        assert_eq!(block.t_oa, 5678);
        assert_eq!(block.health, 0x0123);
    }

    #[test]
    fn qzs_alm_parse_minimal_respects_reserved_bytes() {
        let mut d = vec![0u8; 58];
        d[12] = 3;
        d[13] = 0x7E;
        d[14..18].copy_from_slice(&0.25f32.to_le_bytes());
        d[18..22].copy_from_slice(&3456u32.to_le_bytes());
        d[54] = 77;
        d[55] = 0x5A;
        d[56] = 0xAA;
        d[57] = 0x55;
        let h = SbfHeader {
            crc: 0,
            block_id: block_ids::QZS_ALM,
            block_rev: 0,
            length: (d.len() + 2) as u16,
            tow_ms: 2000,
            wnc: 300,
        };
        let block = QzsAlmBlock::parse(&h, &d).unwrap();
        assert_eq!(block.prn, 3);
        assert!((block.e - 0.25).abs() < 1e-6);
        assert_eq!(block.t_oa, 3456);
        assert_eq!(block.wn_a, 77);
        assert_eq!(block.health8, 0xAA);
        assert_eq!(block.health6, 0x55);
    }

    #[test]
    fn bds_cnav2_parse_minimal() {
        let mut d = vec![0u8; 158];
        d[12] = 21;
        d[13] = 3;
        d[14..18].copy_from_slice(&7200u32.to_le_bytes());
        d[18..26].copy_from_slice(&42_164_000.0f64.to_le_bytes());
        d[134..138].copy_from_slice(&8000u32.to_le_bytes());
        d[142] = 0;
        d[143] = 44;
        d[144..146].copy_from_slice(&0x1122u16.to_le_bytes());
        d[146..150].copy_from_slice(&1.25f32.to_le_bytes());
        d[150..154].copy_from_slice(&2.5f32.to_le_bytes());
        d[154..158].copy_from_slice(&3.75f32.to_le_bytes());
        let h = SbfHeader {
            crc: 0,
            block_id: block_ids::BDS_CNAV2,
            block_rev: 0,
            length: (d.len() + 2) as u16,
            tow_ms: 3000,
            wnc: 400,
        };
        let block = BdsCNav2Block::parse(&h, &d).unwrap();
        assert_eq!(block.prn_idx, 21);
        assert_eq!(block.satellite_type(), 3);
        assert_eq!(block.t_oe, 7200);
        assert_eq!(block.t_op, 8000);
        assert_eq!(block.iode, 44);
        assert_eq!(block.iodc, 0x1122);
        assert_eq!(block.isc_b2ad_s(), Some(1.25));
        assert_eq!(block.t_gd_b2ap_s(), Some(2.5));
        assert_eq!(block.t_gd_b1cp_s(), Some(3.75));
    }

    #[test]
    fn bds_cnav3_parse_minimal() {
        let mut d = vec![0u8; 150];
        d[12] = 7;
        d[13] = 2;
        d[14..18].copy_from_slice(&900u32.to_le_bytes());
        d[138] = 9;
        d[142] = 0;
        d[143..146].copy_from_slice(&[1, 2, 3]);
        d[146..150].copy_from_slice(&(-0.5f32).to_le_bytes());
        let h = SbfHeader {
            crc: 0,
            block_id: block_ids::BDS_CNAV3,
            block_rev: 0,
            length: (d.len() + 2) as u16,
            tow_ms: 4000,
            wnc: 500,
        };
        let block = BdsCNav3Block::parse(&h, &d).unwrap();
        assert_eq!(block.prn_idx, 7);
        assert_eq!(block.satellite_type(), 2);
        assert_eq!(block.t_oe, 900);
        assert_eq!(block.sisai_ocb, 9);
        assert_eq!(block.reserved, [1, 2, 3]);
        assert_eq!(block.t_gd_b2bi_s(), Some(-0.5));
    }
}
