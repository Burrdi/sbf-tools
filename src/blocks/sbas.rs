//! SBAS (Space-Based Augmentation System) blocks
//!
//! GEOMT00, GEOPRNMask, GEOFastCorr and related SBAS message blocks.

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::dnu::{F32_DNU, F64_DNU};
use super::SbfBlockParse;

// ============================================================================
// GEOMT00 Block
// ============================================================================

/// GEOMT00 block (Block ID 5925)
///
/// SBAS MT00 message (null message / test).
#[derive(Debug, Clone)]
pub struct GeoMt00Block {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN (120-158)
    pub prn: u8,
}

impl GeoMt00Block {
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

impl SbfBlockParse for GeoMt00Block {
    const BLOCK_ID: u16 = block_ids::GEO_MT00;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 13 {
            return Err(SbfError::ParseError("GEOMT00 too short".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn: data[12],
        })
    }
}

// ============================================================================
// GEOPRNMask Block
// ============================================================================

/// GEOPRNMask block (Block ID 5926)
///
/// SBAS MT01 PRN mask assignments.
#[derive(Debug, Clone)]
pub struct GeoPrnMaskBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Issue of Data PRN mask
    pub iodp: u8,
    /// Number of PRNs in mask
    pub nbr_prns: u8,
    /// PRN mask array (up to 51 entries)
    pub prn_mask: Vec<u8>,
}

impl GeoPrnMaskBlock {
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

impl SbfBlockParse for GeoPrnMaskBlock {
    const BLOCK_ID: u16 = block_ids::GEO_PRN_MASK;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 16 {
            return Err(SbfError::ParseError("GEOPRNMask too short".into()));
        }

        let prn = data[12];
        let iodp = data[13];
        let nbr_prns = data[14] as usize;

        let prn_mask_len = nbr_prns.min(data.len().saturating_sub(15));
        let prn_mask: Vec<u8> = data[15..15 + prn_mask_len].to_vec();

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iodp,
            nbr_prns: nbr_prns as u8,
            prn_mask,
        })
    }
}

// ============================================================================
// GEOFastCorr Block
// ============================================================================

/// One fast correction entry in `GEOFastCorr`.
#[derive(Debug, Clone)]
pub struct GeoFastCorrEntry {
    /// PRN mask number
    pub prn_mask_no: u8,
    /// User Differential Range Error index
    pub udrei: u8,
    prc_m: f32,
}

impl GeoFastCorrEntry {
    /// Pseudo-Range Correction in meters.
    /// Returns `None` when do-not-use.
    pub fn prc_m(&self) -> Option<f32> {
        if self.prc_m == F32_DNU {
            None
        } else {
            Some(self.prc_m)
        }
    }
    pub fn prc_m_raw(&self) -> f32 {
        self.prc_m
    }
}

/// GEOFastCorr block (Block ID 5927)
///
/// SBAS MT02-05/24 fast corrections.
#[derive(Debug, Clone)]
pub struct GeoFastCorrBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Message type
    pub mt: u8,
    /// Issue of Data PRN mask
    pub iodp: u8,
    /// Issue of Data Fast
    pub iodf: u8,
    /// Number of corrections
    pub n: u8,
    /// Sub-block length
    pub sb_length: u8,
    /// Fast correction entries
    pub corrections: Vec<GeoFastCorrEntry>,
}

impl GeoFastCorrBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_corrections(&self) -> usize {
        self.corrections.len()
    }
}

impl SbfBlockParse for GeoFastCorrBlock {
    const BLOCK_ID: u16 = block_ids::GEO_FAST_CORR;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 20 {
            return Err(SbfError::ParseError("GEOFastCorr too short".into()));
        }

        let prn = data[12];
        let mt = data[13];
        let iodp = data[14];
        let iodf = data[15];
        let n = data[16] as usize;
        let sb_length = data[17] as usize;

        if sb_length < 6 {
            return Err(SbfError::ParseError(
                "GEOFastCorr SBLength too small".into(),
            ));
        }

        let mut corrections = Vec::with_capacity(n);
        let mut offset = 18;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let prn_mask_no = data[offset];
            let udrei = data[offset + 1];
            let prc_m = f32::from_le_bytes(data[offset + 2..offset + 6].try_into().unwrap());

            corrections.push(GeoFastCorrEntry {
                prn_mask_no,
                udrei,
                prc_m,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            mt,
            iodp,
            iodf,
            n: n as u8,
            sb_length: sb_length as u8,
            corrections,
        })
    }
}

// ============================================================================
// GEONav Block
// ============================================================================

/// GEONav block (Block ID 5896)
///
/// SBAS MT09 navigation message with GEO satellite ephemeris.
#[derive(Debug, Clone)]
pub struct GeoNavBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN (120-158)
    pub prn: u8,
    /// Issue of Data Navigation (spare bits)
    pub iodn_spare: u16,
    /// User Range Accuracy
    pub ura: u16,
    /// Reference time (seconds)
    pub t0: u32,
    xg_m: f64,
    yg_m: f64,
    zg_m: f64,
    xgd_mps: f64,
    ygd_mps: f64,
    zgd_mps: f64,
    xgdd_mps2: f64,
    ygdd_mps2: f64,
    zgdd_mps2: f64,
    ag_f0_s: f32,
    ag_f1_sps: f32,
}

impl GeoNavBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn position_x_m(&self) -> Option<f64> {
        if self.xg_m == F64_DNU {
            None
        } else {
            Some(self.xg_m)
        }
    }
    pub fn position_y_m(&self) -> Option<f64> {
        if self.yg_m == F64_DNU {
            None
        } else {
            Some(self.yg_m)
        }
    }
    pub fn position_z_m(&self) -> Option<f64> {
        if self.zg_m == F64_DNU {
            None
        } else {
            Some(self.zg_m)
        }
    }
    pub fn velocity_x_mps(&self) -> Option<f64> {
        if self.xgd_mps == F64_DNU {
            None
        } else {
            Some(self.xgd_mps)
        }
    }
    pub fn velocity_y_mps(&self) -> Option<f64> {
        if self.ygd_mps == F64_DNU {
            None
        } else {
            Some(self.ygd_mps)
        }
    }
    pub fn velocity_z_mps(&self) -> Option<f64> {
        if self.zgd_mps == F64_DNU {
            None
        } else {
            Some(self.zgd_mps)
        }
    }
    pub fn acceleration_x_mps2(&self) -> Option<f64> {
        if self.xgdd_mps2 == F64_DNU {
            None
        } else {
            Some(self.xgdd_mps2)
        }
    }
    pub fn acceleration_y_mps2(&self) -> Option<f64> {
        if self.ygdd_mps2 == F64_DNU {
            None
        } else {
            Some(self.ygdd_mps2)
        }
    }
    pub fn acceleration_z_mps2(&self) -> Option<f64> {
        if self.zgdd_mps2 == F64_DNU {
            None
        } else {
            Some(self.zgdd_mps2)
        }
    }
    pub fn clock_bias_s(&self) -> Option<f32> {
        if self.ag_f0_s == F32_DNU {
            None
        } else {
            Some(self.ag_f0_s)
        }
    }
    pub fn clock_drift_sps(&self) -> Option<f32> {
        if self.ag_f1_sps == F32_DNU {
            None
        } else {
            Some(self.ag_f1_sps)
        }
    }
}

impl SbfBlockParse for GeoNavBlock {
    const BLOCK_ID: u16 = block_ids::GEO_NAV;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 101;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEONav too short".into()));
        }

        let prn = data[12];
        let iodn_spare = u16::from_le_bytes([data[13], data[14]]);
        let ura = u16::from_le_bytes([data[15], data[16]]);
        let t0 = u32::from_le_bytes([data[17], data[18], data[19], data[20]]);
        let xg_m = f64::from_le_bytes(data[21..29].try_into().unwrap());
        let yg_m = f64::from_le_bytes(data[29..37].try_into().unwrap());
        let zg_m = f64::from_le_bytes(data[37..45].try_into().unwrap());
        let xgd_mps = f64::from_le_bytes(data[45..53].try_into().unwrap());
        let ygd_mps = f64::from_le_bytes(data[53..61].try_into().unwrap());
        let zgd_mps = f64::from_le_bytes(data[61..69].try_into().unwrap());
        let xgdd_mps2 = f64::from_le_bytes(data[69..77].try_into().unwrap());
        let ygdd_mps2 = f64::from_le_bytes(data[77..85].try_into().unwrap());
        let zgdd_mps2 = f64::from_le_bytes(data[85..93].try_into().unwrap());
        let ag_f0_s = f32::from_le_bytes(data[93..97].try_into().unwrap());
        let ag_f1_sps = f32::from_le_bytes(data[97..101].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iodn_spare,
            ura,
            t0,
            xg_m,
            yg_m,
            zg_m,
            xgd_mps,
            ygd_mps,
            zgd_mps,
            xgdd_mps2,
            ygdd_mps2,
            zgdd_mps2,
            ag_f0_s,
            ag_f1_sps,
        })
    }
}

// ============================================================================
// GEOIntegrity Block
// ============================================================================

/// GEOIntegrity block (Block ID 5928)
///
/// SBAS MT06 integrity information (UDRE indices).
#[derive(Debug, Clone)]
pub struct GeoIntegrityBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Issue of Data Fast (4 values)
    pub iodf: [u8; 4],
    /// UDRE indices (51 values)
    pub udrei: [u8; 51],
}

impl GeoIntegrityBlock {
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

impl SbfBlockParse for GeoIntegrityBlock {
    const BLOCK_ID: u16 = block_ids::GEO_INTEGRITY;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 68;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOIntegrity too short".into()));
        }

        let prn = data[12];
        let mut iodf = [0u8; 4];
        iodf.copy_from_slice(&data[13..17]);
        let mut udrei = [0u8; 51];
        udrei.copy_from_slice(&data[17..68]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iodf,
            udrei,
        })
    }
}

// ============================================================================
// GEOAlm Block
// ============================================================================

/// GEOAlm block (Block ID 5897)
///
/// SBAS MT17 satellite almanac.
#[derive(Debug, Clone)]
pub struct GeoAlmBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Data ID
    pub data_id: u8,
    /// Health flags
    pub health: u16,
    /// Reference time (seconds)
    pub t0: u32,
    xg_m: f64,
    yg_m: f64,
    zg_m: f64,
    xgd_mps: f64,
    ygd_mps: f64,
    zgd_mps: f64,
}

impl GeoAlmBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn position_x_m(&self) -> Option<f64> {
        if self.xg_m == F64_DNU {
            None
        } else {
            Some(self.xg_m)
        }
    }
    pub fn position_y_m(&self) -> Option<f64> {
        if self.yg_m == F64_DNU {
            None
        } else {
            Some(self.yg_m)
        }
    }
    pub fn position_z_m(&self) -> Option<f64> {
        if self.zg_m == F64_DNU {
            None
        } else {
            Some(self.zg_m)
        }
    }
    pub fn velocity_x_mps(&self) -> Option<f64> {
        if self.xgd_mps == F64_DNU {
            None
        } else {
            Some(self.xgd_mps)
        }
    }
    pub fn velocity_y_mps(&self) -> Option<f64> {
        if self.ygd_mps == F64_DNU {
            None
        } else {
            Some(self.ygd_mps)
        }
    }
    pub fn velocity_z_mps(&self) -> Option<f64> {
        if self.zgd_mps == F64_DNU {
            None
        } else {
            Some(self.zgd_mps)
        }
    }
}

impl SbfBlockParse for GeoAlmBlock {
    const BLOCK_ID: u16 = block_ids::GEO_ALM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 68;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOAlm too short".into()));
        }

        let prn = data[12];
        let data_id = data[13];
        let health = u16::from_le_bytes([data[14], data[15]]);
        let t0 = u32::from_le_bytes([data[16], data[17], data[18], data[19]]);
        let xg_m = f64::from_le_bytes(data[20..28].try_into().unwrap());
        let yg_m = f64::from_le_bytes(data[28..36].try_into().unwrap());
        let zg_m = f64::from_le_bytes(data[36..44].try_into().unwrap());
        let xgd_mps = f64::from_le_bytes(data[44..52].try_into().unwrap());
        let ygd_mps = f64::from_le_bytes(data[52..60].try_into().unwrap());
        let zgd_mps = f64::from_le_bytes(data[60..68].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            data_id,
            health,
            t0,
            xg_m,
            yg_m,
            zg_m,
            xgd_mps,
            ygd_mps,
            zgd_mps,
        })
    }
}

// ============================================================================
// GEOFastCorrDegr Block
// ============================================================================

/// GEOFastCorrDegr block (Block ID 5929)
///
/// SBAS MT07 fast correction degradation (AI indices per PRN mask).
#[derive(Debug, Clone)]
pub struct GeoFastCorrDegrBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Issue of Data PRN mask
    pub iodp: u8,
    /// System latency (seconds)
    pub t_lat: u8,
    /// Degradation factor indices (51 entries)
    pub ai: [u8; 51],
}

impl GeoFastCorrDegrBlock {
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

impl SbfBlockParse for GeoFastCorrDegrBlock {
    const BLOCK_ID: u16 = block_ids::GEO_FAST_CORR_DEGR;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 66;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOFastCorrDegr too short".into()));
        }

        let prn = data[12];
        let iodp = data[13];
        let t_lat = data[14];
        let mut ai = [0u8; 51];
        ai.copy_from_slice(&data[15..66]);

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iodp,
            t_lat,
            ai,
        })
    }
}

// ============================================================================
// GEODegrFactors Block
// ============================================================================

/// GEODegrFactors block (Block ID 5930)
///
/// SBAS MT10 degradation factors for integrity bounds.
#[derive(Debug, Clone)]
pub struct GeoDegrFactorsBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    brrc: f64,
    cltc_lsb: f64,
    cltc_v1: f64,
    pub iltc_v1: u32,
    cltc_v0: f64,
    pub iltc_v0: u32,
    cgeo_lsb: f64,
    cgeo_v: f64,
    pub igeo: u32,
    cer: f32,
    ciono_step: f64,
    pub iiono: u32,
    ciono_ramp: f64,
    pub rss_udre: u8,
    pub rss_iono: u8,
    ccovariance: f64,
}

impl GeoDegrFactorsBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn brrc(&self) -> Option<f64> {
        if self.brrc == F64_DNU {
            None
        } else {
            Some(self.brrc)
        }
    }
    pub fn cltc_lsb(&self) -> Option<f64> {
        if self.cltc_lsb == F64_DNU {
            None
        } else {
            Some(self.cltc_lsb)
        }
    }
    pub fn cltc_v1(&self) -> Option<f64> {
        if self.cltc_v1 == F64_DNU {
            None
        } else {
            Some(self.cltc_v1)
        }
    }
    pub fn cltc_v0(&self) -> Option<f64> {
        if self.cltc_v0 == F64_DNU {
            None
        } else {
            Some(self.cltc_v0)
        }
    }
    pub fn cgeo_lsb(&self) -> Option<f64> {
        if self.cgeo_lsb == F64_DNU {
            None
        } else {
            Some(self.cgeo_lsb)
        }
    }
    pub fn cgeo_v(&self) -> Option<f64> {
        if self.cgeo_v == F64_DNU {
            None
        } else {
            Some(self.cgeo_v)
        }
    }
    pub fn cer(&self) -> Option<f32> {
        if self.cer == F32_DNU {
            None
        } else {
            Some(self.cer)
        }
    }
    pub fn ciono_step(&self) -> Option<f64> {
        if self.ciono_step == F64_DNU {
            None
        } else {
            Some(self.ciono_step)
        }
    }
    pub fn ciono_ramp(&self) -> Option<f64> {
        if self.ciono_ramp == F64_DNU {
            None
        } else {
            Some(self.ciono_ramp)
        }
    }
    pub fn ccovariance(&self) -> Option<f64> {
        if self.ccovariance == F64_DNU {
            None
        } else {
            Some(self.ccovariance)
        }
    }
}

impl SbfBlockParse for GeoDegrFactorsBlock {
    const BLOCK_ID: u16 = block_ids::GEO_DEGR_FACTORS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 107;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEODegrFactors too short".into()));
        }

        let prn = data[12];
        let brrc = f64::from_le_bytes(data[13..21].try_into().unwrap());
        let cltc_lsb = f64::from_le_bytes(data[21..29].try_into().unwrap());
        let cltc_v1 = f64::from_le_bytes(data[29..37].try_into().unwrap());
        let iltc_v1 = u32::from_le_bytes([data[37], data[38], data[39], data[40]]);
        let cltc_v0 = f64::from_le_bytes(data[41..49].try_into().unwrap());
        let iltc_v0 = u32::from_le_bytes([data[49], data[50], data[51], data[52]]);
        let cgeo_lsb = f64::from_le_bytes(data[53..61].try_into().unwrap());
        let cgeo_v = f64::from_le_bytes(data[61..69].try_into().unwrap());
        let igeo = u32::from_le_bytes([data[69], data[70], data[71], data[72]]);
        let cer = f32::from_le_bytes(data[73..77].try_into().unwrap());
        let ciono_step = f64::from_le_bytes(data[77..85].try_into().unwrap());
        let iiono = u32::from_le_bytes([data[85], data[86], data[87], data[88]]);
        let ciono_ramp = f64::from_le_bytes(data[89..97].try_into().unwrap());
        let rss_udre = data[97];
        let rss_iono = data[98];
        let ccovariance = f64::from_le_bytes(data[99..107].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            brrc,
            cltc_lsb,
            cltc_v1,
            iltc_v1,
            cltc_v0,
            iltc_v0,
            cgeo_lsb,
            cgeo_v,
            igeo,
            cer,
            ciono_step,
            iiono,
            ciono_ramp,
            rss_udre,
            rss_iono,
            ccovariance,
        })
    }
}

// ============================================================================
// GEOServiceLevel Block
// ============================================================================

/// One service region in GEOServiceLevel.
#[derive(Debug, Clone)]
pub struct GeoServiceRegion {
    /// Latitude 1 (degrees)
    pub latitude1: i8,
    /// Latitude 2 (degrees)
    pub latitude2: i8,
    /// Longitude 1 (degrees)
    pub longitude1: i16,
    /// Longitude 2 (degrees)
    pub longitude2: i16,
    /// Region shape code
    pub region_shape: u8,
}

/// GEOServiceLevel block (Block ID 5917)
///
/// SBAS MT27 service message with service regions.
#[derive(Debug, Clone)]
pub struct GeoServiceLevelBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Issue of Data Service
    pub iods: u8,
    /// Number of messages
    pub nr_messages: u8,
    /// Message number
    pub message_nr: u8,
    /// Priority code
    pub priority_code: u8,
    /// UDREI delta inside
    pub d_udrei_in: u8,
    /// UDREI delta outside
    pub d_udrei_out: u8,
    /// Number of regions
    pub n: u8,
    /// Sub-block length
    pub sb_length: u8,
    /// Service regions
    pub regions: Vec<GeoServiceRegion>,
}

impl GeoServiceLevelBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_regions(&self) -> usize {
        self.regions.len()
    }
}

impl SbfBlockParse for GeoServiceLevelBlock {
    const BLOCK_ID: u16 = block_ids::GEO_SERVICE_LEVEL;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 21;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOServiceLevel too short".into()));
        }

        let prn = data[12];
        let iods = data[13];
        let nr_messages = data[14];
        let message_nr = data[15];
        let priority_code = data[16];
        let d_udrei_in = data[17];
        let d_udrei_out = data[18];
        let n = data[19] as usize;
        let sb_length = data[20] as usize;

        if sb_length < 7 {
            return Err(SbfError::ParseError(
                "GEOServiceLevel SBLength too small".into(),
            ));
        }

        let mut regions = Vec::with_capacity(n);
        let mut offset = 21;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                return Err(SbfError::ParseError(
                    "GEOServiceLevel data truncated: fewer regions than N".into(),
                ));
            }

            let latitude1 = data[offset] as i8;
            let latitude2 = data[offset + 1] as i8;
            let longitude1 = i16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let longitude2 = i16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            let region_shape = data[offset + 6];

            regions.push(GeoServiceRegion {
                latitude1,
                latitude2,
                longitude1,
                longitude2,
                region_shape,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iods,
            nr_messages,
            message_nr,
            priority_code,
            d_udrei_in,
            d_udrei_out,
            n: n as u8,
            sb_length: sb_length as u8,
            regions,
        })
    }
}

// ============================================================================
// GEONetworkTime Block
// ============================================================================

/// GEONetworkTime block (Block ID 5918)
///
/// SBAS MT12 network time / UTC offset parameters.
#[derive(Debug, Clone)]
pub struct GeoNetworkTimeBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Time offset drift (s/s)
    pub a1: f32,
    /// Time offset (s)
    pub a0: f64,
    /// Reference time (s)
    pub t_ot: u32,
    /// Reference week
    pub wn_t: u8,
    /// Current offset (s)
    pub del_t_1s: i8,
    /// Leap second week
    pub wn_lsf: u8,
    /// Leap second day
    pub dn: u8,
    /// Future offset (s)
    pub del_t_lsf: i8,
    /// UTC standard ID
    pub utc_std_id: u8,
    /// GPS week number
    pub gps_wn: u16,
    /// GPS time of week
    pub gps_tow: u32,
    /// GLONASS indicator
    pub glonass_ind: u8,
}

impl GeoNetworkTimeBlock {
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

impl SbfBlockParse for GeoNetworkTimeBlock {
    const BLOCK_ID: u16 = block_ids::GEO_NETWORK_TIME;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 42;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEONetworkTime too short".into()));
        }

        let prn = data[12];
        let a1 = f32::from_le_bytes(data[13..17].try_into().unwrap());
        let a0 = f64::from_le_bytes(data[17..25].try_into().unwrap());
        let t_ot = u32::from_le_bytes([data[25], data[26], data[27], data[28]]);
        let wn_t = data[29];
        let del_t_1s = data[30] as i8;
        let wn_lsf = data[31];
        let dn = data[32];
        let del_t_lsf = data[33] as i8;
        let utc_std_id = data[34];
        let gps_wn = u16::from_le_bytes([data[35], data[36]]);
        let gps_tow = u32::from_le_bytes([data[37], data[38], data[39], data[40]]);
        let glonass_ind = data[41];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            a1,
            a0,
            t_ot,
            wn_t,
            del_t_1s,
            wn_lsf,
            dn,
            del_t_lsf,
            utc_std_id,
            gps_wn,
            gps_tow,
            glonass_ind,
        })
    }
}

// ============================================================================
// GEOIGPMask Block
// ============================================================================

/// GEOIGPMask block (Block ID 5931)
///
/// SBAS MT18 ionospheric grid point mask.
#[derive(Debug, Clone)]
pub struct GeoIgpMaskBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Number of bands
    pub nbr_bands: u8,
    /// Band number
    pub band_nbr: u8,
    /// Issue of Data Ionosphere
    pub iodi: u8,
    /// Number of IGPs
    pub nbr_igps: u8,
    /// IGP mask array
    pub igp_mask: Vec<u8>,
}

impl GeoIgpMaskBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_igps(&self) -> usize {
        self.igp_mask.len()
    }
}

impl SbfBlockParse for GeoIgpMaskBlock {
    const BLOCK_ID: u16 = block_ids::GEO_IGP_MASK;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 18;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOIGPMask too short".into()));
        }

        let prn = data[12];
        let nbr_bands = data[13];
        let band_nbr = data[14];
        let iodi = data[15];
        let nbr_igps = data[16] as usize;

        let igp_mask_len = nbr_igps.min(data.len().saturating_sub(17));
        let igp_mask: Vec<u8> = data[17..17 + igp_mask_len].to_vec();

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            nbr_bands,
            band_nbr,
            iodi,
            nbr_igps: nbr_igps as u8,
            igp_mask,
        })
    }
}

// ============================================================================
// GEOLongTermCorr Block
// ============================================================================

/// One long-term correction entry in `GeoLongTermCorrBlock`.
#[derive(Debug, Clone)]
pub struct GeoLongTermCorrEntry {
    /// Velocity code present flag
    pub velocity_code: u8,
    /// PRN mask number
    pub prn_mask_no: u8,
    /// Issue of Data PRN mask
    pub iodp: u8,
    /// Issue of Data Ephemeris
    pub iode: u8,
    dx_m: f32,
    dy_m: f32,
    dz_m: f32,
    dx_rate_mps: f32,
    dy_rate_mps: f32,
    dz_rate_mps: f32,
    da_f0_s: f32,
    da_f1_sps: f32,
    /// Reference time (seconds)
    pub t_oe: u32,
}

impl GeoLongTermCorrEntry {
    pub fn dx_m(&self) -> Option<f32> {
        if self.dx_m == F32_DNU {
            None
        } else {
            Some(self.dx_m)
        }
    }
    pub fn dy_m(&self) -> Option<f32> {
        if self.dy_m == F32_DNU {
            None
        } else {
            Some(self.dy_m)
        }
    }
    pub fn dz_m(&self) -> Option<f32> {
        if self.dz_m == F32_DNU {
            None
        } else {
            Some(self.dz_m)
        }
    }
    pub fn dx_rate_mps(&self) -> Option<f32> {
        if self.dx_rate_mps == F32_DNU {
            None
        } else {
            Some(self.dx_rate_mps)
        }
    }
    pub fn dy_rate_mps(&self) -> Option<f32> {
        if self.dy_rate_mps == F32_DNU {
            None
        } else {
            Some(self.dy_rate_mps)
        }
    }
    pub fn dz_rate_mps(&self) -> Option<f32> {
        if self.dz_rate_mps == F32_DNU {
            None
        } else {
            Some(self.dz_rate_mps)
        }
    }
    pub fn da_f0_s(&self) -> Option<f32> {
        if self.da_f0_s == F32_DNU {
            None
        } else {
            Some(self.da_f0_s)
        }
    }
    pub fn da_f1_sps(&self) -> Option<f32> {
        if self.da_f1_sps == F32_DNU {
            None
        } else {
            Some(self.da_f1_sps)
        }
    }
}

/// GEOLongTermCorr block (Block ID 5932)
///
/// SBAS MT24/25 long-term satellite error corrections.
#[derive(Debug, Clone)]
pub struct GeoLongTermCorrBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Number of corrections
    pub n: u8,
    /// Sub-block length
    pub sb_length: u8,
    /// Long-term correction entries
    pub corrections: Vec<GeoLongTermCorrEntry>,
}

impl GeoLongTermCorrBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_corrections(&self) -> usize {
        self.corrections.len()
    }
}

impl SbfBlockParse for GeoLongTermCorrBlock {
    const BLOCK_ID: u16 = block_ids::GEO_LONG_TERM_CORR;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 16;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("GEOLongTermCorr too short".into()));
        }

        let prn = data[12];
        let n = data[13] as usize;
        let sb_length = data[14] as usize;

        if sb_length < 28 {
            return Err(SbfError::ParseError(
                "GEOLongTermCorr SBLength too small".into(),
            ));
        }

        let mut corrections = Vec::with_capacity(n);
        let mut offset = 15;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let velocity_code = data[offset];
            let prn_mask_no = data[offset + 1];
            let iodp = data[offset + 2];
            let iode = data[offset + 3];
            let dx_m = f32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
            let dy_m = f32::from_le_bytes(data[offset + 8..offset + 12].try_into().unwrap());
            let dz_m = f32::from_le_bytes(data[offset + 12..offset + 16].try_into().unwrap());

            let (dx_rate_mps, dy_rate_mps, dz_rate_mps, da_f0_s, da_f1_sps, t_oe) =
                if sb_length >= 40 {
                    (
                        f32::from_le_bytes(data[offset + 16..offset + 20].try_into().unwrap()),
                        f32::from_le_bytes(data[offset + 20..offset + 24].try_into().unwrap()),
                        f32::from_le_bytes(data[offset + 24..offset + 28].try_into().unwrap()),
                        f32::from_le_bytes(data[offset + 28..offset + 32].try_into().unwrap()),
                        f32::from_le_bytes(data[offset + 32..offset + 36].try_into().unwrap()),
                        u32::from_le_bytes([
                            data[offset + 36],
                            data[offset + 37],
                            data[offset + 38],
                            data[offset + 39],
                        ]),
                    )
                } else {
                    let t_oe = u32::from_le_bytes([
                        data[offset + 24],
                        data[offset + 25],
                        data[offset + 26],
                        data[offset + 27],
                    ]);
                    (
                        F32_DNU,
                        F32_DNU,
                        F32_DNU,
                        f32::from_le_bytes(data[offset + 16..offset + 20].try_into().unwrap()),
                        f32::from_le_bytes(data[offset + 20..offset + 24].try_into().unwrap()),
                        t_oe,
                    )
                };

            corrections.push(GeoLongTermCorrEntry {
                velocity_code,
                prn_mask_no,
                iodp,
                iode,
                dx_m,
                dy_m,
                dz_m,
                dx_rate_mps,
                dy_rate_mps,
                dz_rate_mps,
                da_f0_s,
                da_f1_sps,
                t_oe,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            n: n as u8,
            sb_length: sb_length as u8,
            corrections,
        })
    }
}

// ============================================================================
// GEOClockEphCovMatrix Block
// ============================================================================

/// One covariance matrix entry in `GeoClockEphCovMatrixBlock`.
#[derive(Debug, Clone)]
pub struct GeoClockEphCovMatrixEntry {
    /// PRN mask number
    pub prn_mask_no: u8,
    /// Scale exponent
    pub scale_exp: u8,
    /// Covariance elements (E11, E22, E33, E44)
    pub e11: u16,
    pub e22: u16,
    pub e33: u16,
    pub e44: u16,
    /// Off-diagonal elements
    pub e12: i16,
    pub e13: i16,
    pub e14: i16,
    pub e23: i16,
    pub e24: i16,
    pub e34: i16,
}

impl GeoClockEphCovMatrixEntry {
    /// Scale factor: 2^scale_exp
    pub fn scale_factor(&self) -> f64 {
        2f64.powi(self.scale_exp as i32)
    }
}

/// GEOClockEphCovMatrix block (Block ID 5934)
///
/// SBAS MT28 clock-ephemeris covariance matrix.
#[derive(Debug, Clone)]
pub struct GeoClockEphCovMatrixBlock {
    tow_ms: u32,
    wnc: u16,
    /// SBAS PRN
    pub prn: u8,
    /// Issue of Data PRN mask
    pub iodp: u8,
    /// Number of covariance entries
    pub n: u8,
    /// Sub-block length
    pub sb_length: u8,
    /// Covariance matrix entries
    pub entries: Vec<GeoClockEphCovMatrixEntry>,
}

impl GeoClockEphCovMatrixBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_entries(&self) -> usize {
        self.entries.len()
    }
}

impl SbfBlockParse for GeoClockEphCovMatrixBlock {
    const BLOCK_ID: u16 = block_ids::GEO_CLOCK_EPH_COV_MATRIX;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 17;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError(
                "GEOClockEphCovMatrix too short".into(),
            ));
        }

        let prn = data[12];
        let iodp = data[13];
        let n = data[14] as usize;
        let sb_length = data[15] as usize;

        if sb_length < 22 {
            return Err(SbfError::ParseError(
                "GEOClockEphCovMatrix SBLength too small".into(),
            ));
        }

        let mut entries = Vec::with_capacity(n);
        let mut offset = 16;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let prn_mask_no = data[offset];
            let scale_exp = data[offset + 1];
            let e11 = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let e22 = u16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            let e33 = u16::from_le_bytes([data[offset + 6], data[offset + 7]]);
            let e44 = u16::from_le_bytes([data[offset + 8], data[offset + 9]]);
            let e12 = i16::from_le_bytes([data[offset + 10], data[offset + 11]]);
            let e13 = i16::from_le_bytes([data[offset + 12], data[offset + 13]]);
            let e14 = i16::from_le_bytes([data[offset + 14], data[offset + 15]]);
            let e23 = i16::from_le_bytes([data[offset + 16], data[offset + 17]]);
            let e24 = i16::from_le_bytes([data[offset + 18], data[offset + 19]]);
            let e34 = i16::from_le_bytes([data[offset + 20], data[offset + 21]]);

            entries.push(GeoClockEphCovMatrixEntry {
                prn_mask_no,
                scale_exp,
                e11,
                e22,
                e33,
                e44,
                e12,
                e13,
                e14,
                e23,
                e24,
                e34,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            prn,
            iodp,
            n: n as u8,
            sb_length: sb_length as u8,
            entries,
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
            length: 32,
            tow_ms,
            wnc,
        }
    }

    #[test]
    fn test_geo_mt00_parse() {
        let header = header_for(5925, 1000, 2200);
        let mut data = vec![0u8; 13];
        data[12] = 131; // PRN 131

        let block = GeoMt00Block::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.wnc(), 2200);
        assert_eq!(block.prn, 131);
    }

    #[test]
    fn test_geo_mt00_too_short() {
        let header = header_for(5925, 0, 0);
        let data = [0u8; 10];
        assert!(GeoMt00Block::parse(&header, &data).is_err());
    }

    #[test]
    fn test_geo_prn_mask_parse() {
        let header = header_for(5926, 2000, 2100);
        let mut data = vec![0u8; 20];
        data[12] = 120;
        data[13] = 5;
        data[14] = 3; // NbrPRNs = 3
        data[15] = 1;
        data[16] = 2;
        data[17] = 3;

        let block = GeoPrnMaskBlock::parse(&header, &data).unwrap();
        assert_eq!(block.prn, 120);
        assert_eq!(block.iodp, 5);
        assert_eq!(block.nbr_prns, 3);
        assert_eq!(block.prn_mask, vec![1, 2, 3]);
    }

    #[test]
    fn test_geo_fast_corr_parse() {
        let header = header_for(5927, 3000, 2000);
        let mut data = vec![0u8; 30];
        data[12] = 124;
        data[13] = 2; // MT
        data[14] = 1;
        data[15] = 0;
        data[16] = 1; // N = 1
        data[17] = 6; // SBLength = 6
        data[18] = 10; // PRNMaskNo
        data[19] = 2; // UDREI
        data[20..24].copy_from_slice(&2.5_f32.to_le_bytes()); // PRC = 2.5 m

        let block = GeoFastCorrBlock::parse(&header, &data).unwrap();
        assert_eq!(block.prn, 124);
        assert_eq!(block.mt, 2);
        assert_eq!(block.n, 1);
        assert_eq!(block.corrections.len(), 1);
        assert_eq!(block.corrections[0].prn_mask_no, 10);
        assert_eq!(block.corrections[0].udrei, 2);
        assert!((block.corrections[0].prc_m().unwrap() - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_geo_fast_corr_dnu() {
        let header = header_for(5927, 0, 0);
        let mut data = vec![0u8; 30];
        data[16] = 1;
        data[17] = 6;
        data[20..24].copy_from_slice(&F32_DNU.to_le_bytes());

        let block = GeoFastCorrBlock::parse(&header, &data).unwrap();
        assert!(block.corrections[0].prc_m().is_none());
    }

    #[test]
    fn test_geo_nav_parse() {
        let header = header_for(5896, 5000, 2200);
        let mut data = vec![0u8; 101];
        data[12] = 124;
        data[13..15].copy_from_slice(&1u16.to_le_bytes());
        data[15..17].copy_from_slice(&2u16.to_le_bytes());
        data[17..21].copy_from_slice(&100u32.to_le_bytes());
        data[21..29].copy_from_slice(&12345678.0_f64.to_le_bytes());
        data[29..37].copy_from_slice(&23456789.0_f64.to_le_bytes());
        data[37..45].copy_from_slice(&34567890.0_f64.to_le_bytes());
        data[93..97].copy_from_slice(&0.0001_f32.to_le_bytes());
        data[97..101].copy_from_slice(&0.00001_f32.to_le_bytes());

        let block = GeoNavBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2200);
        assert_eq!(block.prn, 124);
        assert_eq!(block.t0, 100);
        assert!((block.position_x_m().unwrap() - 12345678.0).abs() < 1e-6);
        assert!((block.clock_bias_s().unwrap() - 0.0001).abs() < 1e-9);
    }

    #[test]
    fn test_geo_nav_dnu() {
        let header = header_for(5896, 0, 0);
        let mut data = vec![0u8; 101];
        data[21..29].copy_from_slice(&F64_DNU.to_le_bytes());
        data[93..97].copy_from_slice(&F32_DNU.to_le_bytes());

        let block = GeoNavBlock::parse(&header, &data).unwrap();
        assert!(block.position_x_m().is_none());
        assert!(block.clock_bias_s().is_none());
    }

    #[test]
    fn test_geo_integrity_parse() {
        let header = header_for(5928, 1000, 2100);
        let mut data = vec![0u8; 68];
        data[12] = 120;
        data[13..17].copy_from_slice(&[1, 2, 3, 4]);
        data[17..68].fill(5);

        let block = GeoIntegrityBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.prn, 120);
        assert_eq!(block.iodf, [1, 2, 3, 4]);
        assert_eq!(block.udrei[0], 5);
    }

    #[test]
    fn test_geo_alm_parse() {
        let header = header_for(5897, 2000, 2050);
        let mut data = vec![0u8; 68];
        data[12] = 131;
        data[13] = 1;
        data[14..16].copy_from_slice(&0u16.to_le_bytes());
        data[16..20].copy_from_slice(&500u32.to_le_bytes());
        data[20..28].copy_from_slice(&1e7_f64.to_le_bytes());
        data[28..36].copy_from_slice(&2e7_f64.to_le_bytes());
        data[36..44].copy_from_slice(&3e7_f64.to_le_bytes());

        let block = GeoAlmBlock::parse(&header, &data).unwrap();
        assert_eq!(block.prn, 131);
        assert_eq!(block.data_id, 1);
        assert_eq!(block.t0, 500);
        assert!((block.position_x_m().unwrap() - 1e7).abs() < 1.0);
    }

    #[test]
    fn test_geo_network_time_parse() {
        let header = header_for(5918, 3000, 2000);
        let mut data = vec![0u8; 42];
        data[12] = 124;
        data[13..17].copy_from_slice(&0.0001_f32.to_le_bytes());
        data[17..25].copy_from_slice(&0.5_f64.to_le_bytes());
        data[25..29].copy_from_slice(&1000u32.to_le_bytes());
        data[29] = 100;
        data[30] = 18i8 as u8;
        data[35..37].copy_from_slice(&2300u16.to_le_bytes());
        data[37..41].copy_from_slice(&43200000u32.to_le_bytes());

        let block = GeoNetworkTimeBlock::parse(&header, &data).unwrap();
        assert_eq!(block.prn, 124);
        assert_eq!(block.gps_wn, 2300);
        assert_eq!(block.gps_tow, 43200000);
    }

    #[test]
    fn test_geo_fast_corr_degr_parse() {
        let header = header_for(5929, 1000, 2200);
        let mut data = vec![0u8; 66];
        data[12] = 124;
        data[13] = 3;
        data[14] = 5;
        data[15..20].copy_from_slice(&[1, 2, 3, 4, 5]);

        let block = GeoFastCorrDegrBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.prn, 124);
        assert_eq!(block.iodp, 3);
        assert_eq!(block.t_lat, 5);
        assert_eq!(block.ai[0], 1);
        assert_eq!(block.ai[4], 5);
    }

    #[test]
    fn test_geo_fast_corr_degr_too_short() {
        let header = header_for(5929, 0, 0);
        let data = vec![0u8; 50];
        assert!(GeoFastCorrDegrBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_geo_degr_factors_parse() {
        let header = header_for(5930, 2000, 2100);
        let mut data = vec![0u8; 107];
        data[12] = 120;
        data[13..21].copy_from_slice(&1.5_f64.to_le_bytes());
        data[21..29].copy_from_slice(&0.125_f64.to_le_bytes());
        data[37..41].copy_from_slice(&60u32.to_le_bytes());
        data[97] = 1;
        data[98] = 0;

        let block = GeoDegrFactorsBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 2.0);
        assert_eq!(block.prn, 120);
        assert!((block.brrc().unwrap() - 1.5).abs() < 1e-10);
        assert_eq!(block.iltc_v1, 60);
        assert_eq!(block.rss_udre, 1);
    }

    #[test]
    fn test_geo_degr_factors_dnu() {
        let header = header_for(5930, 0, 0);
        let mut data = vec![0u8; 107];
        data[13..21].copy_from_slice(&F64_DNU.to_le_bytes());
        data[73..77].copy_from_slice(&F32_DNU.to_le_bytes());

        let block = GeoDegrFactorsBlock::parse(&header, &data).unwrap();
        assert!(block.brrc().is_none());
        assert!(block.cer().is_none());
    }

    #[test]
    fn test_geo_service_level_parse() {
        let header = header_for(5917, 3000, 2000);
        let mut data = vec![0u8; 28];
        data[12] = 124;
        data[13] = 1;
        data[14] = 2;
        data[15] = 0;
        data[16] = 0;
        data[17] = 1;
        data[18] = 2;
        data[19] = 1;
        data[20] = 7;
        data[21] = 45i8 as u8;
        data[22] = 50i8 as u8;
        data[23..25].copy_from_slice(&(-120i16).to_le_bytes());
        data[25..27].copy_from_slice(&(-115i16).to_le_bytes());
        data[27] = 0;

        let block = GeoServiceLevelBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 3.0);
        assert_eq!(block.prn, 124);
        assert_eq!(block.iods, 1);
        assert_eq!(block.n, 1);
        assert_eq!(block.regions.len(), 1);
        assert_eq!(block.regions[0].latitude1, 45);
        assert_eq!(block.regions[0].latitude2, 50);
        assert_eq!(block.regions[0].longitude1, -120);
    }

    #[test]
    fn test_geo_service_level_too_short() {
        let header = header_for(5917, 0, 0);
        let data = vec![0u8; 15];
        assert!(GeoServiceLevelBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_geo_service_level_truncated_regions() {
        let header = header_for(5917, 0, 0);
        let mut data = vec![0u8; 28];
        data[19] = 2;
        data[20] = 7;
        assert!(GeoServiceLevelBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_geo_igp_mask_parse() {
        let header = header_for(5931, 5000, 2100);
        let mut data = vec![0u8; 22];
        data[12] = 124;
        data[13] = 1;
        data[14] = 0;
        data[15] = 5;
        data[16] = 4;
        data[17] = 0xAB;
        data[18] = 0xCD;
        data[19] = 0xEF;
        data[20] = 0x12;

        let block = GeoIgpMaskBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2100);
        assert_eq!(block.prn, 124);
        assert_eq!(block.nbr_bands, 1);
        assert_eq!(block.band_nbr, 0);
        assert_eq!(block.iodi, 5);
        assert_eq!(block.nbr_igps, 4);
        assert_eq!(block.igp_mask.len(), 4);
        assert_eq!(block.igp_mask[0], 0xAB);
        assert_eq!(block.igp_mask[3], 0x12);
    }

    #[test]
    fn test_geo_igp_mask_too_short() {
        let header = header_for(5931, 0, 0);
        let data = vec![0u8; 15];
        assert!(GeoIgpMaskBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_geo_long_term_corr_parse() {
        let header = header_for(5932, 1000, 2200);
        let mut data = vec![0u8; 56];
        data[12] = 131;
        data[13] = 1;
        data[14] = 40;
        data[15] = 1;
        data[16] = 2;
        data[17] = 3;
        data[18] = 4;
        data[19..23].copy_from_slice(&(1.5f32).to_le_bytes());
        data[23..27].copy_from_slice(&(2.5f32).to_le_bytes());
        data[27..31].copy_from_slice(&(3.5f32).to_le_bytes());
        data[31..35].copy_from_slice(&(0.1f32).to_le_bytes());
        data[35..39].copy_from_slice(&(0.2f32).to_le_bytes());
        data[39..43].copy_from_slice(&(0.3f32).to_le_bytes());
        data[43..47].copy_from_slice(&(1e-6f32).to_le_bytes());
        data[47..51].copy_from_slice(&(1e-10f32).to_le_bytes());
        data[51..55].copy_from_slice(&86400u32.to_le_bytes());

        let block = GeoLongTermCorrBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.prn, 131);
        assert_eq!(block.n, 1);
        assert_eq!(block.corrections.len(), 1);
        let c = &block.corrections[0];
        assert_eq!(c.prn_mask_no, 2);
        assert_eq!(c.iodp, 3);
        assert_eq!(c.iode, 4);
        assert_eq!(c.dx_m(), Some(1.5));
        assert_eq!(c.t_oe, 86400);
    }

    #[test]
    fn test_geo_long_term_corr_dnu() {
        let header = header_for(5932, 0, 0);
        let mut data = vec![0u8; 56];
        data[13] = 1;
        data[14] = 40;
        data[19..23].copy_from_slice(&F32_DNU.to_le_bytes());
        data[23..27].copy_from_slice(&F32_DNU.to_le_bytes());
        data[27..31].copy_from_slice(&F32_DNU.to_le_bytes());

        let block = GeoLongTermCorrBlock::parse(&header, &data).unwrap();
        assert!(block.corrections[0].dx_m().is_none());
        assert!(block.corrections[0].dy_m().is_none());
        assert!(block.corrections[0].dz_m().is_none());
    }

    #[test]
    fn test_geo_clock_eph_cov_matrix_parse() {
        let header = header_for(5934, 2000, 100);
        let mut data = vec![0u8; 40];
        data[12] = 120;
        data[13] = 7;
        data[14] = 1;
        data[15] = 22;
        data[16] = 1;
        data[17] = 3;
        data[18..20].copy_from_slice(&100u16.to_le_bytes());
        data[20..22].copy_from_slice(&200u16.to_le_bytes());
        data[22..24].copy_from_slice(&300u16.to_le_bytes());
        data[24..26].copy_from_slice(&400u16.to_le_bytes());
        data[26..28].copy_from_slice(&(-10i16).to_le_bytes());
        data[28..30].copy_from_slice(&(-20i16).to_le_bytes());
        data[30..32].copy_from_slice(&(-30i16).to_le_bytes());
        data[32..34].copy_from_slice(&(-40i16).to_le_bytes());
        data[34..36].copy_from_slice(&(-50i16).to_le_bytes());
        data[36..38].copy_from_slice(&(-60i16).to_le_bytes());

        let block = GeoClockEphCovMatrixBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 2.0);
        assert_eq!(block.prn, 120);
        assert_eq!(block.iodp, 7);
        assert_eq!(block.n, 1);
        assert_eq!(block.entries.len(), 1);
        let e = &block.entries[0];
        assert_eq!(e.prn_mask_no, 1);
        assert_eq!(e.scale_exp, 3);
        assert_eq!(e.e11, 100);
        assert_eq!(e.e22, 200);
        assert_eq!(e.e34, -60);
        assert_eq!(e.scale_factor(), 8.0);
    }

    #[test]
    fn test_geo_clock_eph_cov_matrix_too_short() {
        let header = header_for(5934, 0, 0);
        let data = vec![0u8; 14];
        assert!(GeoClockEphCovMatrixBlock::parse(&header, &data).is_err());
    }
}
