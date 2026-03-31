//! High-level Meas3 decoding.
//!
//! Septentrio does not publish the packed Meas3 payload layout in the public reference guide.
//! The decoder in this module follows the `sbf2asc` / `sbfread_meas.c` sources shipped with
//! RxTools.

use std::collections::HashMap;

use crate::error::{SbfError, SbfResult};
use crate::types::{SatelliteId, SignalType};

use super::{
    Meas3Cn0HiResBlock, Meas3DopplerBlock, Meas3MpBlock, Meas3PpBlock, Meas3RangesBlock, SbfBlock,
};

const C84: f64 = 299_792_458.0;
const E5_FREQ: f64 = 1_191.795e6;
const E5A_FREQ: f64 = 1_176.45e6;
const E5B_FREQ: f64 = 1_207.14e6;
const E6_FREQ: f64 = 1_278.75e6;
const L1_FREQ: f64 = 1_575.42e6;
const L2_FREQ: f64 = 1_227.60e6;
const L5_FREQ: f64 = 1_176.45e6;
const E2_FREQ: f64 = 1_561.098e6;
const L1_GLO_FREQ: f64 = 1_602.00e6;
const L2_GLO_FREQ: f64 = 1_246.00e6;
const L3_GLO_FREQ: f64 = 1_202.025e6;
const B3_FREQ: f64 = 1_268.52e6;
const S1_FREQ: f64 = 2_492.028e6;

const L1_WAVELENGTH: f64 = C84 / L1_FREQ;
const L2_WAVELENGTH: f64 = C84 / L2_FREQ;
const L3_WAVELENGTH: f64 = C84 / L3_GLO_FREQ;
const L5_WAVELENGTH: f64 = C84 / L5_FREQ;
const E2_WAVELENGTH: f64 = C84 / E2_FREQ;
const E5_WAVELENGTH: f64 = C84 / E5_FREQ;
const E5A_WAVELENGTH: f64 = C84 / E5A_FREQ;
const E5B_WAVELENGTH: f64 = C84 / E5B_FREQ;
const E6_WAVELENGTH: f64 = C84 / E6_FREQ;
const B3_WAVELENGTH: f64 = C84 / B3_FREQ;
const S1_WAVELENGTH: f64 = C84 / S1_FREQ;

const F64_NOTVALID: f64 = -2e10;
const F32_NOTVALID: f32 = -2e10;

const MEAS3_SIG_MAX: usize = 16;
const MEAS3_SAT_MAX: usize = 64;
const MAX_ANTENNAS: usize = 3;

const MEASFLAG_SMOOTHING: u8 = 1 << 0;
const MEASFLAG_HALFCYCLEAMBIGUITY: u8 = 1 << 2;
const MEASFLAG_FIRSTMEAS: u8 = 1 << 3;
const MEASFLAG_APMEINSYNC: u8 = 1 << 5;
const MEASFLAG_VALIDITY: u8 = 1 << 7;

const PR_BASE_M: [f64; 7] = [19e6, 19e6, 22e6, 20e6, 34e6, 34e6, 34e6];
const LOCK_INDICATOR_TO_MS: [u32; 16] = [
    0, 60_000, 30_000, 15_000, 10_000, 5_000, 2_000, 1_000, 500, 200, 100, 50, 40, 20, 10, 0,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Meas3SatSystem {
    Gps = 0,
    Glo = 1,
    Gal = 2,
    Bds = 3,
    Sbas = 4,
    Qzs = 5,
    Irn = 6,
}

impl Meas3SatSystem {
    fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::Gps),
            1 => Some(Self::Glo),
            2 => Some(Self::Gal),
            3 => Some(Self::Bds),
            4 => Some(Self::Sbas),
            5 => Some(Self::Qzs),
            6 => Some(Self::Irn),
            _ => None,
        }
    }

    fn base_svid(self) -> u8 {
        match self {
            Self::Gps => 1,
            Self::Glo => 38,
            Self::Gal => 71,
            Self::Bds => 141,
            Self::Sbas => 120,
            Self::Qzs => 181,
            Self::Irn => 191,
        }
    }
}

#[derive(Debug, Clone)]
struct InternalMeasurement {
    signal_index: u8,
    signal_type: SignalType,
    flags: u8,
    pr_m: f64,
    l_cycles: f64,
    doppler_hz: f32,
    cn0_dbhz: f32,
    raw_cn0_dbhz: u8,
    pll_timer_ms: u32,
    mp_mm: i16,
    carrier_mp_1_512c: i8,
    smoothing_corr_mm: i16,
    lock_count: u8,
}

impl Default for InternalMeasurement {
    fn default() -> Self {
        Self {
            signal_index: 0,
            signal_type: SignalType::Other(u8::MAX),
            flags: 0,
            pr_m: F64_NOTVALID,
            l_cycles: F64_NOTVALID,
            doppler_hz: F32_NOTVALID,
            cn0_dbhz: F32_NOTVALID,
            raw_cn0_dbhz: 0,
            pll_timer_ms: 0,
            mp_mm: 0,
            carrier_mp_1_512c: 0,
            smoothing_corr_mm: 0,
            lock_count: 0,
        }
    }
}

impl InternalMeasurement {
    fn into_public(self) -> Meas3Measurement {
        Meas3Measurement {
            signal_index: self.signal_index,
            signal_type: self.signal_type,
            flags: self.flags,
            pseudorange_m: if self.flags & MEASFLAG_VALIDITY != 0 && self.pr_m != F64_NOTVALID {
                Some(self.pr_m)
            } else {
                None
            },
            carrier_phase_cycles: if self.flags & MEASFLAG_VALIDITY != 0
                && self.l_cycles != F64_NOTVALID
            {
                Some(self.l_cycles)
            } else {
                None
            },
            doppler_hz: if self.flags & MEASFLAG_VALIDITY != 0 && self.doppler_hz != F32_NOTVALID {
                Some(self.doppler_hz)
            } else {
                None
            },
            cn0_dbhz: if self.flags & MEASFLAG_VALIDITY != 0 && self.cn0_dbhz != F32_NOTVALID {
                Some(self.cn0_dbhz)
            } else {
                None
            },
            raw_cn0_dbhz: if self.flags & MEASFLAG_VALIDITY != 0 {
                Some(self.raw_cn0_dbhz)
            } else {
                None
            },
            lock_time_ms: if self.flags & MEASFLAG_VALIDITY != 0 && self.pll_timer_ms != 0 {
                Some(self.pll_timer_ms)
            } else {
                None
            },
            multipath_mm: self.mp_mm,
            carrier_multipath_1_512c: self.carrier_mp_1_512c,
            smoothing_correction_mm: self.smoothing_corr_mm,
            lock_count: if self.lock_count == 0 {
                None
            } else {
                Some(self.lock_count)
            },
        }
    }
}

#[derive(Debug, Clone, Default)]
struct ReferenceSatellite {
    sig_order: Vec<u8>,
    measurements: HashMap<u8, InternalMeasurement>,
    slave_sig_mask: u32,
    pr_rate_64mm_s: i16,
}

#[derive(Debug, Clone, Default)]
struct ReferenceConstellation {
    m3satdata_copy: Vec<u8>,
    satellites: HashMap<u8, ReferenceSatellite>,
}

#[derive(Debug, Clone, Default)]
struct AntennaReferenceEpoch {
    tow_ms: Option<u32>,
    constellations: HashMap<Meas3SatSystem, ReferenceConstellation>,
}

#[derive(Debug, Clone, Copy)]
struct MasterDecodeContext<'a> {
    signal_table: &'a [SignalType; MEAS3_SIG_MAX],
    glonass_fn: i8,
    short_pr_base_m: f64,
    sig_idx_master_short: u8,
    reference_sat: &'a ReferenceSatellite,
    time_since_ref_epoch_ms: u32,
    pr_rate_available: bool,
}

#[derive(Debug, Clone, Copy)]
struct SlaveDecodeContext<'a> {
    signal_table: &'a [SignalType; MEAS3_SIG_MAX],
    sig_idx: u8,
    glonass_fn: i8,
    master: &'a InternalMeasurement,
    master_sig_idx: u8,
    master_ref: Option<&'a InternalMeasurement>,
    slave_ref: Option<&'a InternalMeasurement>,
}

/// A single decoded measurement from a Meas3 epoch.
#[derive(Debug, Clone)]
pub struct Meas3Measurement {
    pub signal_index: u8,
    pub signal_type: SignalType,
    flags: u8,
    pseudorange_m: Option<f64>,
    carrier_phase_cycles: Option<f64>,
    doppler_hz: Option<f32>,
    cn0_dbhz: Option<f32>,
    raw_cn0_dbhz: Option<u8>,
    lock_time_ms: Option<u32>,
    pub multipath_mm: i16,
    pub carrier_multipath_1_512c: i8,
    pub smoothing_correction_mm: i16,
    pub lock_count: Option<u8>,
}

impl Meas3Measurement {
    pub fn pseudorange_m(&self) -> Option<f64> {
        self.pseudorange_m
    }
    pub fn carrier_phase_cycles(&self) -> Option<f64> {
        self.carrier_phase_cycles
    }
    pub fn doppler_hz(&self) -> Option<f32> {
        self.doppler_hz
    }
    pub fn cn0_dbhz(&self) -> Option<f32> {
        self.cn0_dbhz
    }
    pub fn raw_cn0_dbhz(&self) -> Option<u8> {
        self.raw_cn0_dbhz
    }
    pub fn lock_time_ms(&self) -> Option<u32> {
        self.lock_time_ms
    }
    pub fn is_valid(&self) -> bool {
        (self.flags & MEASFLAG_VALIDITY) != 0
    }
    pub fn smoothing_active(&self) -> bool {
        (self.flags & MEASFLAG_SMOOTHING) != 0
    }
    pub fn has_half_cycle_ambiguity(&self) -> bool {
        (self.flags & MEASFLAG_HALFCYCLEAMBIGUITY) != 0
    }
    pub fn is_first_measurement(&self) -> bool {
        (self.flags & MEASFLAG_FIRSTMEAS) != 0
    }
    pub fn apme_in_sync(&self) -> bool {
        (self.flags & MEASFLAG_APMEINSYNC) != 0
    }
    pub fn raw_pseudorange_m(&self) -> Option<f64> {
        self.pseudorange_m.map(|value| {
            value + self.multipath_mm as f64 * 0.001 + self.smoothing_correction_mm as f64 * 0.001
        })
    }
    pub fn raw_carrier_phase_cycles(&self) -> Option<f64> {
        self.carrier_phase_cycles
            .map(|value| value + self.carrier_multipath_1_512c as f64 / 512.0)
    }
}

/// All decoded measurements for one satellite in a Meas3 epoch.
#[derive(Debug, Clone)]
pub struct Meas3Satellite {
    pub sat_id: SatelliteId,
    pub glonass_frequency_number: Option<i8>,
    pub measurements: Vec<Meas3Measurement>,
}

/// One decoded Meas3 epoch for a single antenna.
#[derive(Debug, Clone)]
pub struct Meas3DecodedEpoch {
    tow_ms: u32,
    wnc: u16,
    pub antenna_id: u8,
    pub common_flags: u8,
    pub total_clock_jump_ms: i32,
    pub reference_epoch_interval_ms: u32,
    pub is_reference_epoch: bool,
    pub reference_epoch_contains_pr_rate: bool,
    pub satellites: Vec<Meas3Satellite>,
}

impl Meas3DecodedEpoch {
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
    pub fn num_measurements(&self) -> usize {
        self.satellites
            .iter()
            .map(|sat| sat.measurements.len())
            .sum()
    }
}

/// One same-epoch bundle of Meas3 blocks for a single antenna.
#[derive(Debug, Clone, Default)]
pub struct Meas3BlockSet {
    pub ranges: Option<Meas3RangesBlock>,
    pub cn0_hi_res: Option<Meas3Cn0HiResBlock>,
    pub doppler: Option<Meas3DopplerBlock>,
    pub pp: Option<Meas3PpBlock>,
    pub mp: Option<Meas3MpBlock>,
}

impl Meas3BlockSet {
    pub fn tow_ms(&self) -> Option<u32> {
        self.ranges.as_ref().map(Meas3RangesBlock::tow_ms)
    }

    pub fn wnc(&self) -> Option<u16> {
        self.ranges.as_ref().map(Meas3RangesBlock::wnc)
    }

    pub fn antenna_id(&self) -> Option<u8> {
        self.ranges.as_ref().map(Meas3RangesBlock::antenna_id)
    }

    pub fn insert_block(&mut self, block: &SbfBlock) -> bool {
        match block {
            SbfBlock::Meas3Ranges(value) => {
                self.ranges = Some(value.clone());
                true
            }
            SbfBlock::Meas3Cn0HiRes(value) => {
                self.cn0_hi_res = Some(value.clone());
                true
            }
            SbfBlock::Meas3Doppler(value) => {
                self.doppler = Some(value.clone());
                true
            }
            SbfBlock::Meas3Pp(value) => {
                self.pp = Some(value.clone());
                true
            }
            SbfBlock::Meas3Mp(value) => {
                self.mp = Some(value.clone());
                true
            }
            _ => false,
        }
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

/// Stateful Meas3 decoder.
#[derive(Debug, Clone, Default)]
pub struct Meas3Decoder {
    references: Vec<AntennaReferenceEpoch>,
}

impl Meas3Decoder {
    pub fn new() -> Self {
        Self {
            references: vec![AntennaReferenceEpoch::default(); MAX_ANTENNAS],
        }
    }

    pub fn decode_block_set(&mut self, set: &Meas3BlockSet) -> SbfResult<Meas3DecodedEpoch> {
        let ranges = set
            .ranges
            .as_ref()
            .ok_or_else(|| SbfError::ParseError("Meas3 block set is missing Meas3Ranges".into()))?;
        self.decode(
            ranges,
            set.cn0_hi_res.as_ref(),
            set.doppler.as_ref(),
            set.pp.as_ref(),
            set.mp.as_ref(),
        )
    }

    pub fn decode(
        &mut self,
        ranges: &Meas3RangesBlock,
        cn0_hi_res: Option<&Meas3Cn0HiResBlock>,
        doppler: Option<&Meas3DopplerBlock>,
        pp: Option<&Meas3PpBlock>,
        mp: Option<&Meas3MpBlock>,
    ) -> SbfResult<Meas3DecodedEpoch> {
        if ranges.has_scrambled_measurements() {
            return Err(SbfError::ParseError(
                "Meas3 scrambling is not supported by this crate".into(),
            ));
        }
        if ranges.reserved > 31 {
            return Err(SbfError::ParseError(
                "Meas3Ranges revision marker is not supported".into(),
            ));
        }

        let antenna_id = ranges.antenna_id() as usize;
        if antenna_id >= self.references.len() {
            return Err(SbfError::ParseError(
                "Meas3 antenna index out of range".into(),
            ));
        }

        for block in [
            cn0_hi_res.map(|b| (b.tow_ms(), b.wnc(), b.antenna_id(), "Meas3CN0HiRes")),
            doppler.map(|b| (b.tow_ms(), b.wnc(), b.antenna_id(), "Meas3Doppler")),
            pp.map(|b| (b.tow_ms(), b.wnc(), b.antenna_id(), "Meas3PP")),
            mp.map(|b| (b.tow_ms(), b.wnc(), b.antenna_id(), "Meas3MP")),
        ]
        .into_iter()
        .flatten()
        {
            if block.0 != ranges.tow_ms()
                || block.1 != ranges.wnc()
                || block.2 != ranges.antenna_id()
            {
                return Err(SbfError::ParseError(format!(
                    "{} does not match the Meas3Ranges epoch",
                    block.3
                )));
            }
        }

        let ref_interval_ms = ranges.reference_epoch_interval_ms();
        let is_reference_epoch = ranges.is_reference_epoch();
        let ref_epoch_contains_pr_rate = ranges.reference_epoch_contains_pr_rate();
        let ant_ref = &mut self.references[antenna_id];

        if is_reference_epoch {
            *ant_ref = AntennaReferenceEpoch::default();
            ant_ref.tow_ms = Some(ranges.tow_ms());
        } else {
            let required_ref_tow = (ranges.tow_ms() / ref_interval_ms) * ref_interval_ms;
            if ant_ref.tow_ms != Some(required_ref_tow) {
                return Err(SbfError::ParseError(
                    "Meas3 delta epoch received before its reference epoch".into(),
                ));
            }
        }

        let mut payload = ranges.data.as_slice();
        let mut cn0_idx = 0usize;
        let mut doppler_idx = 0usize;
        let mut pp1_idx = 0usize;
        let mut pp2_idx = 0usize;
        let mut mp_idx = 0usize;
        let mut satellites = Vec::new();

        for index in 0..7 {
            if (ranges.constellations & (1 << index)) == 0 {
                continue;
            }
            let system = Meas3SatSystem::from_index(index).unwrap();
            let (consumed, mut decoded) = Self::decode_constellation(
                payload,
                system,
                ant_ref,
                ranges.tow_ms(),
                ref_interval_ms,
                ref_epoch_contains_pr_rate,
                cn0_hi_res,
                &mut cn0_idx,
                doppler,
                &mut doppler_idx,
                pp,
                &mut pp1_idx,
                &mut pp2_idx,
                mp,
                &mut mp_idx,
                is_reference_epoch,
            )?;
            satellites.append(&mut decoded);
            payload = &payload[consumed..];
        }

        let total_clock_jump_ms = if ranges.cum_clk_jumps >= 128 {
            ranges.cum_clk_jumps as i32 - 256
        } else {
            ranges.cum_clk_jumps as i32
        };

        Ok(Meas3DecodedEpoch {
            tow_ms: ranges.tow_ms(),
            wnc: ranges.wnc(),
            antenna_id: ranges.antenna_id(),
            common_flags: ranges.common_flags,
            total_clock_jump_ms,
            reference_epoch_interval_ms: ref_interval_ms,
            is_reference_epoch,
            reference_epoch_contains_pr_rate: ref_epoch_contains_pr_rate,
            satellites,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn decode_constellation(
        buf: &[u8],
        system: Meas3SatSystem,
        antenna_ref: &mut AntennaReferenceEpoch,
        tow_ms: u32,
        ref_interval_ms: u32,
        ref_epoch_contains_pr_rate: bool,
        cn0_hi_res: Option<&Meas3Cn0HiResBlock>,
        cn0_idx: &mut usize,
        doppler: Option<&Meas3DopplerBlock>,
        doppler_idx: &mut usize,
        pp: Option<&Meas3PpBlock>,
        pp1_idx: &mut usize,
        pp2_idx: &mut usize,
        mp: Option<&Meas3MpBlock>,
        mp_idx: &mut usize,
        is_reference_epoch: bool,
    ) -> SbfResult<(usize, Vec<Meas3Satellite>)> {
        if buf.is_empty() {
            return Err(SbfError::ParseError(
                "Meas3 constellation data is empty".into(),
            ));
        }

        let start = buf;
        let ref_const = antenna_ref.constellations.entry(system).or_default();
        let sat_data_buf = if buf[0] == 0 {
            if ref_const.m3satdata_copy.is_empty() {
                return Err(SbfError::ParseError(
                    "Meas3 delta constellation is missing its reference layout".into(),
                ));
            }
            ref_const.m3satdata_copy.as_slice()
        } else {
            buf
        };

        let mut n = 0usize;
        let mut sat_mask = 0u64;
        let mut nsats = 0usize;
        let mut glo_fn_list = [0u8; 32];

        let bf1 = sat_data_buf[0];
        let mut nb = (bf1 & 0x07) as usize;
        if nb == 7 {
            nb = 8;
        }
        let sig_idx_master_short = (bf1 >> 3) & 0x0f;
        let sig_excluded_present = (bf1 & 0x80) != 0;
        n += 1;

        if sat_data_buf.len() < n + nb {
            return Err(SbfError::ParseError("Meas3 SatMask is truncated".into()));
        }
        for ii in 0..nb {
            let byte = sat_data_buf[n + ii];
            sat_mask |= (byte as u64) << (ii * 8);
            nsats += bit_count(byte) as usize;
        }
        n += nb;

        if system == Meas3SatSystem::Glo {
            let len = nsats.div_ceil(2);
            if sat_data_buf.len() < n + len {
                return Err(SbfError::ParseError(
                    "Meas3 GLO frequency list is truncated".into(),
                ));
            }
            glo_fn_list[..len].copy_from_slice(&sat_data_buf[n..n + len]);
            n += len;
        }

        let bds_long_range = if system == Meas3SatSystem::Bds {
            if sat_data_buf.len() < n + 2 {
                return Err(SbfError::ParseError(
                    "Meas3 BDS long-range flags are truncated".into(),
                ));
            }
            let value = u16::from_le_bytes(sat_data_buf[n..n + 2].try_into().unwrap());
            n += 2;
            value
        } else {
            0
        };

        let sig_excluded = if sig_excluded_present {
            if sat_data_buf.len() <= n {
                return Err(SbfError::ParseError(
                    "Meas3 SigExcluded is truncated".into(),
                ));
            }
            let value = sat_data_buf[n];
            n += 1;
            value
        } else {
            0
        };

        let mut buf = if start[0] == 0 {
            &start[1..]
        } else {
            &start[n..]
        };

        if is_reference_epoch {
            ref_const.m3satdata_copy = start[..n].to_vec();
        }

        let signal_table = prepare_signal_table(system, sig_excluded);
        let mut satellites = Vec::new();
        let mut sat_count = 0usize;

        for sat_idx in 0..MEAS3_SAT_MAX {
            if sat_count >= nsats {
                break;
            }
            if (sat_mask & (1u64 << sat_idx)) == 0 {
                continue;
            }

            let glonass_fn = if system == Meas3SatSystem::Glo {
                let nibble = (glo_fn_list[sat_count / 2] >> (4 * (sat_count % 2))) & 0x0f;
                nibble as i8 - 8
            } else {
                0
            };
            let short_pr_base =
                if system == Meas3SatSystem::Bds && (bds_long_range & (1 << sat_count)) != 0 {
                    34e6
                } else {
                    PR_BASE_M[system as usize]
                };

            let reference_sat = ref_const.satellites.entry(sat_idx as u8).or_default();
            let time_since_ref_epoch_ms = tow_ms % ref_interval_ms;
            let (mut master, master_sig_idx, mut slave_sig_mask, pr_rate_64mm_s, master_size) =
                decode_master(
                    buf,
                    MasterDecodeContext {
                        signal_table: &signal_table,
                        glonass_fn,
                        short_pr_base_m: short_pr_base,
                        sig_idx_master_short,
                        reference_sat,
                        time_since_ref_epoch_ms,
                        pr_rate_available: ref_epoch_contains_pr_rate,
                    },
                )?;
            buf = &buf[master_size..];

            if is_reference_epoch {
                reference_sat.sig_order.clear();
                reference_sat.sig_order.push(master_sig_idx);
                reference_sat.slave_sig_mask = slave_sig_mask;
                reference_sat.pr_rate_64mm_s = pr_rate_64mm_s;
                reference_sat
                    .measurements
                    .insert(master_sig_idx, master.clone());
            }
            if let Some(existing) = reference_sat.measurements.get_mut(&master_sig_idx) {
                if master.pll_timer_ms > existing.pll_timer_ms {
                    existing.pll_timer_ms = master.pll_timer_ms;
                }
            }

            let master_wavelength = wavelength_for_signal(master.signal_type, glonass_fn);
            add_master_doppler(
                &mut master,
                doppler,
                master_wavelength,
                reference_sat.pr_rate_64mm_s,
                doppler_idx,
            );
            add_pp_info(&mut master, pp, pp1_idx, pp2_idx);
            add_mp_info(&mut master, mp, mp_idx);

            let mut master_hi_res_adjust = 0.0f32;
            if let Some(cn0) = cn0_hi_res {
                master_hi_res_adjust = decode_cn0_hires(&cn0.data, cn0_idx)?;
            }

            let sat_id = SatelliteId::from_svid(system.base_svid() + sat_idx as u8)
                .ok_or_else(|| SbfError::ParseError("Meas3 produced an invalid SVID".into()))?;

            let mut current_measurements = Vec::new();
            let mut slave_count = 0usize;
            for sig_idx in 1..MEAS3_SIG_MAX {
                if slave_sig_mask == 0 {
                    break;
                }
                if (slave_sig_mask & (1 << sig_idx)) == 0 {
                    continue;
                }

                let slave_uses_reference = buf
                    .first()
                    .is_some_and(|first| (first & 1) == 0 && (first & 3) != 0);
                let (master_ref, slave_ref) = if slave_uses_reference {
                    let master_ref_idx = *reference_sat.sig_order.first().ok_or_else(|| {
                        SbfError::ParseError(
                            "Meas3 reference epoch is missing the master signal".into(),
                        )
                    })?;
                    let master_ref = reference_sat
                        .measurements
                        .get(&master_ref_idx)
                        .ok_or_else(|| {
                            SbfError::ParseError(
                                "Meas3 reference master measurement is missing".into(),
                            )
                        })?
                        .clone();
                    let slave_ref_idx =
                        *reference_sat
                            .sig_order
                            .get(slave_count + 1)
                            .ok_or_else(|| {
                                SbfError::ParseError(
                                    "Meas3 reference slave measurement is missing".into(),
                                )
                            })?;
                    let slave_ref = reference_sat
                        .measurements
                        .get(&slave_ref_idx)
                        .ok_or_else(|| {
                            SbfError::ParseError(
                                "Meas3 reference slave measurement is missing".into(),
                            )
                        })?
                        .clone();
                    (Some(master_ref), Some(slave_ref))
                } else {
                    (None, None)
                };

                let (mut slave, consumed) = decode_slave(
                    buf,
                    SlaveDecodeContext {
                        signal_table: &signal_table,
                        sig_idx: sig_idx as u8,
                        glonass_fn,
                        master: &master,
                        master_sig_idx,
                        master_ref: master_ref.as_ref(),
                        slave_ref: slave_ref.as_ref(),
                    },
                )?;
                buf = &buf[consumed..];

                let slave_wavelength = wavelength_for_signal(slave.signal_type, glonass_fn);
                add_slave_doppler(
                    &mut slave,
                    &master,
                    doppler,
                    master_wavelength,
                    slave_wavelength,
                    doppler_idx,
                );
                add_pp_info(&mut slave, pp, pp1_idx, pp2_idx);
                add_mp_info(&mut slave, mp, mp_idx);

                if is_reference_epoch {
                    if reference_sat.sig_order.len() <= slave_count + 1 {
                        reference_sat.sig_order.push(sig_idx as u8);
                    }
                    reference_sat
                        .measurements
                        .insert(sig_idx as u8, slave.clone());
                }
                if let Some(existing) = reference_sat.measurements.get_mut(&(sig_idx as u8)) {
                    if slave.pll_timer_ms > existing.pll_timer_ms {
                        existing.pll_timer_ms = slave.pll_timer_ms;
                    }
                }

                if cn0_hi_res.is_some() {
                    slave.cn0_dbhz +=
                        decode_cn0_hires(cn0_hi_res.unwrap().data.as_slice(), cn0_idx)?;
                }

                current_measurements.push(slave.into_public());
                slave_count += 1;
                slave_sig_mask ^= 1 << sig_idx;
            }

            master.cn0_dbhz += master_hi_res_adjust;
            current_measurements.insert(0, master.into_public());

            satellites.push(Meas3Satellite {
                sat_id,
                glonass_frequency_number: if system == Meas3SatSystem::Glo {
                    Some(glonass_fn)
                } else {
                    None
                },
                measurements: current_measurements,
            });

            sat_count += 1;
        }

        let consumed = start.len() - buf.len();
        Ok((consumed, satellites))
    }
}

fn bit_count(value: u8) -> u32 {
    value.count_ones()
}

fn lsb_pos(value: u32) -> u8 {
    if value == 0 {
        32
    } else {
        value.trailing_zeros() as u8
    }
}

fn decode_cn0_hires(data: &[u8], idx: &mut usize) -> SbfResult<f32> {
    let byte = *data
        .get(*idx / 2)
        .ok_or_else(|| SbfError::ParseError("Meas3CN0HiRes payload is truncated".into()))?;
    let nibble = (byte >> ((*idx % 2) * 4)) & 0x0f;
    *idx += 1;
    Ok(nibble as f32 * 0.0625 - 0.5)
}

fn prepare_signal_table(system: Meas3SatSystem, sig_excluded: u8) -> [SignalType; MEAS3_SIG_MAX] {
    let defaults = default_signal_table(system);
    let mut result = [SignalType::Other(u8::MAX); MEAS3_SIG_MAX];
    let mut positions = Vec::with_capacity(MEAS3_SIG_MAX);
    for i in 0..MEAS3_SIG_MAX {
        if i >= 8 || (sig_excluded & (1 << i)) == 0 {
            positions.push(i);
        }
    }
    for (dst, src) in positions.into_iter().enumerate().take(MEAS3_SIG_MAX) {
        result[dst] = defaults[src];
    }
    result
}

fn default_signal_table(system: Meas3SatSystem) -> [SignalType; MEAS3_SIG_MAX] {
    let invalid = SignalType::Other(u8::MAX);
    match system {
        Meas3SatSystem::Gps => [
            SignalType::L1CA,
            SignalType::L2C,
            SignalType::L5,
            SignalType::L1PY,
            SignalType::L2P,
            SignalType::L1C,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Glo => [
            SignalType::G1CA,
            SignalType::G2CA,
            SignalType::G1P,
            SignalType::G2P,
            SignalType::G3,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Gal => [
            SignalType::E1,
            SignalType::E5a,
            SignalType::E5b,
            SignalType::E6,
            SignalType::E5AltBOC,
            SignalType::Other(16),
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Bds => [
            SignalType::B1I,
            SignalType::B2I,
            SignalType::B3I,
            SignalType::B1C,
            SignalType::B2a,
            SignalType::B2b,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Sbas => [
            SignalType::SBASL1,
            SignalType::SBASL5,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Qzs => [
            SignalType::QZSSL1CA,
            SignalType::QZSSL2C,
            SignalType::QZSSL5,
            SignalType::QZSSL6,
            SignalType::QZSSL1C,
            SignalType::QZSSL1S,
            SignalType::QZSSL5S,
            SignalType::QZSSL1CB,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
        Meas3SatSystem::Irn => [
            SignalType::NavICL5,
            SignalType::NavICL1,
            SignalType::Other(36),
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
            invalid,
        ],
    }
}

fn wavelength_for_signal(signal_type: SignalType, glonass_fn: i8) -> f64 {
    match signal_type {
        SignalType::L1CA
        | SignalType::L1PY
        | SignalType::L1C
        | SignalType::E1
        | SignalType::SBASL1
        | SignalType::QZSSL1CA
        | SignalType::QZSSL1CB
        | SignalType::QZSSL1C
        | SignalType::QZSSL1S
        | SignalType::B1C
        | SignalType::NavICL1 => L1_WAVELENGTH,
        SignalType::L2P | SignalType::L2C | SignalType::QZSSL2C => L2_WAVELENGTH,
        SignalType::E5AltBOC => E5_WAVELENGTH,
        SignalType::E5a | SignalType::Other(16) => E5A_WAVELENGTH,
        SignalType::E5b | SignalType::B2I | SignalType::B2b => E5B_WAVELENGTH,
        SignalType::E6 | SignalType::QZSSL6 => E6_WAVELENGTH,
        SignalType::L5
        | SignalType::SBASL5
        | SignalType::QZSSL5
        | SignalType::QZSSL5S
        | SignalType::NavICL5
        | SignalType::B2a => L5_WAVELENGTH,
        SignalType::B1I => E2_WAVELENGTH,
        SignalType::B3I => B3_WAVELENGTH,
        SignalType::G1CA | SignalType::G1P => {
            if (-7..=6).contains(&glonass_fn) {
                C84 / (L1_GLO_FREQ + glonass_fn as f64 * 562_500.0)
            } else {
                F64_NOTVALID
            }
        }
        SignalType::G2CA | SignalType::G2P => {
            if (-7..=6).contains(&glonass_fn) {
                C84 / (L2_GLO_FREQ + glonass_fn as f64 * 437_500.0)
            } else {
                F64_NOTVALID
            }
        }
        SignalType::G3 => L3_WAVELENGTH,
        SignalType::Other(36) => S1_WAVELENGTH,
        SignalType::LBand => L1_WAVELENGTH,
        SignalType::L2PY | SignalType::Other(_) => F64_NOTVALID,
    }
}

fn is_gps_p_code(signal_type: SignalType) -> bool {
    matches!(signal_type, SignalType::L1PY | SignalType::L2P)
}

fn read_u16(bytes: &[u8], offset: usize) -> SbfResult<u16> {
    let slice = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| SbfError::ParseError("Meas3 payload is truncated".into()))?;
    Ok(u16::from_le_bytes(slice.try_into().unwrap()))
}

fn read_u32(bytes: &[u8], offset: usize) -> SbfResult<u32> {
    let slice = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| SbfError::ParseError("Meas3 payload is truncated".into()))?;
    Ok(u32::from_le_bytes(slice.try_into().unwrap()))
}

fn read_i16(bytes: &[u8], offset: usize) -> SbfResult<i16> {
    let slice = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| SbfError::ParseError("Meas3 payload is truncated".into()))?;
    Ok(i16::from_le_bytes(slice.try_into().unwrap()))
}

fn read_u8(bytes: &[u8], offset: usize) -> SbfResult<u8> {
    bytes
        .get(offset)
        .copied()
        .ok_or_else(|| SbfError::ParseError("Meas3 payload is truncated".into()))
}

fn decode_master(
    buf: &[u8],
    ctx: MasterDecodeContext<'_>,
) -> SbfResult<(InternalMeasurement, u8, u32, i16, usize)> {
    let MasterDecodeContext {
        signal_table,
        glonass_fn,
        short_pr_base_m,
        sig_idx_master_short,
        reference_sat,
        time_since_ref_epoch_ms,
        pr_rate_available,
    } = ctx;

    if buf.is_empty() {
        return Err(SbfError::ParseError(
            "Meas3 master sub-block is missing".into(),
        ));
    }

    let mut meas = InternalMeasurement::default();

    if (buf[0] & 1) == 1 {
        if buf.len() < if pr_rate_available { 10 } else { 8 } {
            return Err(SbfError::ParseError(
                "Meas3 master-short is truncated".into(),
            ));
        }
        let bf1 = read_u32(buf, 0)?;
        let pr_lsb = read_u32(buf, 4)?;
        let cmc = (bf1 >> 1) & 0x3ffff;
        let pr_msb = (bf1 >> 19) & 1;
        let lti = ((bf1 >> 20) & 0x7) as usize;
        let cn0 = (bf1 >> 23) & 0x1f;
        let sig_list = (bf1 >> 28) & 0x0f;
        let master_sig_idx = sig_idx_master_short;
        let signal_type = signal_table[master_sig_idx as usize];
        let wavelength = wavelength_for_signal(signal_type, glonass_fn);

        meas.signal_index = master_sig_idx;
        meas.signal_type = signal_type;
        meas.flags = MEASFLAG_VALIDITY;
        meas.pr_m = short_pr_base_m + (pr_lsb as f64 + 4_294_967_296.0 * pr_msb as f64) * 0.001;
        meas.l_cycles = if cmc == 0 {
            F64_NOTVALID
        } else {
            meas.pr_m / wavelength - 131.072 + cmc as f64 * 0.001
        };
        meas.cn0_dbhz = cn0 as f32 + 24.0;
        meas.pll_timer_ms = LOCK_INDICATOR_TO_MS[lti];
        if lti == 0 {
            meas.flags |= MEASFLAG_HALFCYCLEAMBIGUITY;
        }

        let pr_rate_64mm_s = if pr_rate_available {
            read_i16(buf, 8)?
        } else {
            0
        };
        let slave_sig_mask = sig_list << (master_sig_idx + 1);
        Ok((
            meas,
            master_sig_idx,
            slave_sig_mask,
            pr_rate_64mm_s,
            if pr_rate_available { 10 } else { 8 },
        ))
    } else if (buf[0] & 3) == 0 {
        if buf.len() < 10 {
            return Err(SbfError::ParseError(
                "Meas3 master-long is truncated".into(),
            ));
        }
        let bf1 = read_u32(buf, 0)?;
        let pr_lsb = read_u32(buf, 4)?;
        let bf2 = read_u16(buf, 8)?;
        let pr_msb = (bf1 >> 2) & 0x0f;
        let cmc = (bf1 >> 6) & 0x3fffff;
        let lti = ((bf1 >> 28) & 0x0f) as usize;
        let cn0 = (bf2 & 0x3f) as u32;
        let mut sig_mask = ((bf2 >> 6) & 0x1ff) as u32;
        let cont = ((bf2 >> 15) & 1) != 0;
        let pr_rate_offset = 10 + cont as usize;
        let consumed = if pr_rate_available {
            pr_rate_offset + 2
        } else {
            pr_rate_offset
        };
        if buf.len() < consumed {
            return Err(SbfError::ParseError(
                "Meas3 master-long is truncated".into(),
            ));
        }
        if cont {
            let bf3 = read_u8(buf, 10)?;
            sig_mask |= ((bf3 & 0x7f) as u32) << 9;
        }
        if sig_mask == 0 {
            return Err(SbfError::ParseError(
                "Meas3 master-long has an empty signal mask".into(),
            ));
        }
        let master_sig_idx = lsb_pos(sig_mask);
        let signal_type = signal_table[master_sig_idx as usize];
        let wavelength = wavelength_for_signal(signal_type, glonass_fn);

        meas.signal_index = master_sig_idx;
        meas.signal_type = signal_type;
        meas.flags = MEASFLAG_VALIDITY;
        meas.pr_m = (pr_lsb as f64 + 4_294_967_296.0 * pr_msb as f64) * 0.001;
        meas.l_cycles = if cmc == 0 {
            F64_NOTVALID
        } else {
            meas.pr_m / wavelength - 2_097.152 + cmc as f64 * 0.001
        };
        meas.cn0_dbhz = if is_gps_p_code(signal_type) {
            cn0 as f32
        } else {
            cn0 as f32 + 10.0
        };
        meas.pll_timer_ms = LOCK_INDICATOR_TO_MS[lti];
        if lti == 0 {
            meas.flags |= MEASFLAG_HALFCYCLEAMBIGUITY;
        }

        let pr_rate_64mm_s = if pr_rate_available {
            read_i16(buf, pr_rate_offset)?
        } else {
            0
        };
        let slave_sig_mask = sig_mask ^ (1 << master_sig_idx);
        Ok((
            meas,
            master_sig_idx,
            slave_sig_mask,
            pr_rate_64mm_s,
            consumed,
        ))
    } else if (buf[0] & 0x0c) == 0x0c {
        if buf.len() < 5 {
            return Err(SbfError::ParseError(
                "Meas3 master delta-long is truncated".into(),
            ));
        }
        let bf1 = read_u8(buf, 0)?;
        let bf2 = read_u32(buf, 1)?;
        let pr = (((bf1 >> 4) as u32) << 13) | (bf2 & 0x1fff);
        let cn0 = (bf2 >> 13) & 0x07;
        let cmc = bf2 >> 16;
        let master_sig_idx = *reference_sat.sig_order.first().ok_or_else(|| {
            SbfError::ParseError("Meas3 master delta-long is missing reference data".into())
        })?;
        let master_ref = reference_sat
            .measurements
            .get(&master_sig_idx)
            .ok_or_else(|| {
                SbfError::ParseError("Meas3 master delta-long is missing reference data".into())
            })?;
        let signal_type = signal_table[master_sig_idx as usize];
        let wavelength = wavelength_for_signal(signal_type, glonass_fn);

        meas.signal_index = master_sig_idx;
        meas.signal_type = signal_type;
        meas.flags = master_ref.flags & (MEASFLAG_VALIDITY | MEASFLAG_HALFCYCLEAMBIGUITY);
        meas.pll_timer_ms = master_ref.pll_timer_ms;
        meas.pr_m = master_ref.pr_m
            + ((reference_sat.pr_rate_64mm_s as i64 * 64 * time_since_ref_epoch_ms as i64 / 1000)
                as f64)
                * 0.001
            + pr as f64 * 0.001
            - 65.536;
        meas.l_cycles = if cmc == 0 {
            F64_NOTVALID
        } else {
            (meas.pr_m - master_ref.pr_m) / wavelength + master_ref.l_cycles - 32.768
                + cmc as f64 * 0.001
        };
        meas.cn0_dbhz = master_ref.cn0_dbhz - 4.0 + cn0 as f32;
        Ok((meas, master_sig_idx, reference_sat.slave_sig_mask, 0, 5))
    } else {
        if buf.len() < 4 {
            return Err(SbfError::ParseError(
                "Meas3 master delta-short is truncated".into(),
            ));
        }
        let bf1 = read_u32(buf, 0)?;
        let pr = (bf1 >> 4) & 0x3fff;
        let cmc = (bf1 >> 18) & 0x3fff;
        let cn0 = (bf1 >> 2) & 0x03;
        let master_sig_idx = *reference_sat.sig_order.first().ok_or_else(|| {
            SbfError::ParseError("Meas3 master delta-short is missing reference data".into())
        })?;
        let master_ref = reference_sat
            .measurements
            .get(&master_sig_idx)
            .ok_or_else(|| {
                SbfError::ParseError("Meas3 master delta-short is missing reference data".into())
            })?;
        let signal_type = signal_table[master_sig_idx as usize];
        let wavelength = wavelength_for_signal(signal_type, glonass_fn);

        meas.signal_index = master_sig_idx;
        meas.signal_type = signal_type;
        meas.flags = master_ref.flags & (MEASFLAG_VALIDITY | MEASFLAG_HALFCYCLEAMBIGUITY);
        meas.pll_timer_ms = master_ref.pll_timer_ms;
        meas.pr_m = master_ref.pr_m
            + ((reference_sat.pr_rate_64mm_s as i64 * 64 * time_since_ref_epoch_ms as i64 / 1000)
                as f64)
                * 0.001
            + pr as f64 * 0.001
            - 8.192;
        meas.l_cycles = if cmc == 0 {
            F64_NOTVALID
        } else {
            (meas.pr_m - master_ref.pr_m) / wavelength + master_ref.l_cycles - 8.192
                + cmc as f64 * 0.001
        };
        meas.cn0_dbhz = master_ref.cn0_dbhz - 1.0 + cn0 as f32;
        Ok((meas, master_sig_idx, reference_sat.slave_sig_mask, 0, 4))
    }
}

fn decode_slave(
    buf: &[u8],
    ctx: SlaveDecodeContext<'_>,
) -> SbfResult<(InternalMeasurement, usize)> {
    let SlaveDecodeContext {
        signal_table,
        sig_idx,
        glonass_fn,
        master,
        master_sig_idx,
        master_ref,
        slave_ref,
    } = ctx;

    if buf.is_empty() {
        return Err(SbfError::ParseError(
            "Meas3 slave sub-block is missing".into(),
        ));
    }

    let mut meas = InternalMeasurement::default();
    let wavelength_master =
        wavelength_for_signal(signal_table[master_sig_idx as usize], glonass_fn);
    let signal_type = signal_table[sig_idx as usize];
    let wavelength_slave = wavelength_for_signal(signal_type, glonass_fn);

    meas.flags = MEASFLAG_VALIDITY;
    meas.signal_index = sig_idx;
    meas.signal_type = signal_type;

    if (buf[0] & 1) == 1 {
        if buf.len() < 5 {
            return Err(SbfError::ParseError(
                "Meas3 slave-short is truncated".into(),
            ));
        }
        let bf1 = read_u32(buf, 0)?;
        let bf2 = read_u8(buf, 4)?;
        let cmc_res = (bf1 >> 1) & 0xffff;
        let pr_rel = bf1 >> 17;
        let lti = (bf2 & 0x07) as usize;
        let cn0 = (bf2 >> 3) as u32;

        if wavelength_master < wavelength_slave {
            meas.pr_m = master.pr_m + pr_rel as f64 * 0.001 - 10.0;
        } else {
            meas.pr_m = master.pr_m - pr_rel as f64 * 0.001 + 10.0;
        }

        meas.l_cycles = if cmc_res == 0 {
            F64_NOTVALID
        } else {
            meas.pr_m / wavelength_slave
                + (master.l_cycles - master.pr_m / wavelength_master) * wavelength_slave
                    / wavelength_master
                - 32.768
                + cmc_res as f64 * 0.001
        };

        meas.cn0_dbhz = if is_gps_p_code(signal_type) {
            master.cn0_dbhz - 3.0 - cn0 as f32
        } else {
            cn0 as f32 + 24.0
        };
        meas.pll_timer_ms = LOCK_INDICATOR_TO_MS[lti];
        if lti == 0 {
            meas.flags |= MEASFLAG_HALFCYCLEAMBIGUITY;
        }
        Ok((meas, 5))
    } else if (buf[0] & 3) == 0 {
        if buf.len() < 7 {
            return Err(SbfError::ParseError("Meas3 slave-long is truncated".into()));
        }
        let bf1 = read_u32(buf, 0)?;
        let pr_lsb_rel = read_u16(buf, 4)?;
        let bf3 = read_u8(buf, 6)?;
        let cmc = (bf1 >> 2) & 0x3fffff;
        let lti = ((bf1 >> 24) & 0x0f) as usize;
        let pr_msb_rel = (bf1 >> 28) & 0x07;
        let cn0 = (bf3 & 0x3f) as u32;

        meas.pr_m =
            master.pr_m + (pr_msb_rel * 65_536 + pr_lsb_rel as u32) as f64 * 0.001 - 262.144;
        meas.l_cycles = if cmc == 0 {
            F64_NOTVALID
        } else {
            meas.pr_m / wavelength_slave - 2_097.152 + cmc as f64 * 0.001
        };
        meas.cn0_dbhz = if is_gps_p_code(signal_type) {
            cn0 as f32
        } else {
            cn0 as f32 + 10.0
        };
        meas.pll_timer_ms = LOCK_INDICATOR_TO_MS[lti];
        if lti == 0 {
            meas.flags |= MEASFLAG_HALFCYCLEAMBIGUITY;
        }
        Ok((meas, 7))
    } else {
        if buf.len() < 3 {
            return Err(SbfError::ParseError(
                "Meas3 slave delta is truncated".into(),
            ));
        }
        let master_ref = master_ref.ok_or_else(|| {
            SbfError::ParseError("Meas3 slave delta is missing reference data".into())
        })?;
        let slave_ref = slave_ref.ok_or_else(|| {
            SbfError::ParseError("Meas3 slave delta is missing reference data".into())
        })?;
        let bf1 = read_u16(buf, 0)?;
        let d_carrier = read_u8(buf, 2)?;
        let d_pr = ((bf1 >> 2) & 0x0fff) as u32;
        let cn0 = (bf1 >> 14) as u32;

        meas.flags = slave_ref.flags & (MEASFLAG_VALIDITY | MEASFLAG_HALFCYCLEAMBIGUITY);
        meas.l_cycles = slave_ref.l_cycles
            + (master.l_cycles - master_ref.l_cycles) * wavelength_master / wavelength_slave
            - 0.128
            + d_carrier as f64 * 0.001;
        meas.pr_m = slave_ref.pr_m + (meas.l_cycles - slave_ref.l_cycles) * wavelength_slave
            - 2.048
            + d_pr as f64 * 0.001;
        meas.cn0_dbhz = slave_ref.cn0_dbhz - 2.0 + cn0 as f32;
        meas.pll_timer_ms = slave_ref.pll_timer_ms;
        Ok((meas, 3))
    }
}

fn get_pr_rate_mm_s(
    doppler: Option<&Meas3DopplerBlock>,
    idx: &mut usize,
) -> SbfResult<Option<i32>> {
    let Some(doppler) = doppler else {
        return Ok(None);
    };

    let value = read_u32(doppler.data.as_slice(), *idx)?;
    let (abs_prr_mm_s, consumed, magnitude_bits) = if (value & 2) == 0 {
        (((value & 0xff) >> 2) as i32, 1usize, 6u8)
    } else if (value & 6) == 2 {
        (((value & 0xffff) >> 3) as i32, 2usize, 13u8)
    } else if (value & 0x0e) == 6 {
        (((value & 0x00ff_ffff) >> 4) as i32, 3usize, 20u8)
    } else {
        ((value >> 4) as i32, 4usize, 28u8)
    };
    *idx += consumed;

    let negative = (value & 1) != 0;
    // Meas3Doppler uses a width-dependent negative all-ones magnitude as the DNU marker.
    let invalid_abs = (1i32 << magnitude_bits) - 1;
    if negative && abs_prr_mm_s == invalid_abs {
        return Ok(None);
    }

    Ok(Some(if negative {
        -abs_prr_mm_s
    } else {
        abs_prr_mm_s
    }))
}

fn add_master_doppler(
    meas: &mut InternalMeasurement,
    doppler: Option<&Meas3DopplerBlock>,
    wavelength_m: f64,
    ref_pr_rate_64mm_s: i16,
    idx: &mut usize,
) {
    let Ok(Some(prr_mm_s)) = get_pr_rate_mm_s(doppler, idx) else {
        meas.doppler_hz = F32_NOTVALID;
        return;
    };
    meas.doppler_hz =
        -((prr_mm_s + ref_pr_rate_64mm_s as i32 * 64) as f64 * 0.001 / wavelength_m) as f32;
}

fn add_slave_doppler(
    meas: &mut InternalMeasurement,
    master: &InternalMeasurement,
    doppler: Option<&Meas3DopplerBlock>,
    wavelength_master_m: f64,
    wavelength_slave_m: f64,
    idx: &mut usize,
) {
    let Ok(Some(prr_mm_s)) = get_pr_rate_mm_s(doppler, idx) else {
        meas.doppler_hz = F32_NOTVALID;
        return;
    };
    meas.doppler_hz = ((master.doppler_hz as f64 * wavelength_master_m * 1000.0 - prr_mm_s as f64)
        * 0.001
        / wavelength_slave_m) as f32;
}

fn add_pp_info(
    meas: &mut InternalMeasurement,
    pp: Option<&Meas3PpBlock>,
    pp1_idx: &mut usize,
    pp2_idx: &mut usize,
) {
    meas.raw_cn0_dbhz = ((meas.cn0_dbhz as i32 / 2) * 2).max(0) as u8;

    let Some(pp) = pp else {
        return;
    };
    // The first PP payload byte is the section-2 offset in 4-byte units. Older
    // parsing accidentally folded it into `flags`; keep the decoder aligned to
    // the wire format by splitting it out explicitly.
    let Some((section2_offset_units, payload)) = pp.data.split_first() else {
        return;
    };

    let Some(first_lo) = payload.get(*pp1_idx / 8) else {
        return;
    };
    let Some(first_hi) = payload.get(*pp1_idx / 8 + 1) else {
        return;
    };
    let dummy = *first_lo as u16 | (*first_hi as u16) << 8;
    let val = dummy >> (*pp1_idx % 8);
    let start2 = (*section2_offset_units as usize) * 4;

    if (val & 1) != 0 {
        meas.flags |= MEASFLAG_APMEINSYNC;
    }
    meas.lock_count = ((val >> 1) & 0x0f) as u8;
    *pp1_idx += 5;

    if start2 != 0 && payload.get(start2).map(|value| value & 0x0f) == Some(0) {
        if let (Some(lo), Some(hi)) = (
            payload.get(*pp2_idx / 8 + start2 + 2),
            payload.get(*pp2_idx / 8 + start2 + 3),
        ) {
            let dummy = *lo as u16 | (*hi as u16) << 8;
            let val = dummy >> (*pp2_idx % 8);
            if (val & 1) == 0 {
                *pp2_idx += 1;
            } else {
                meas.raw_cn0_dbhz = (((val >> 1) & 0x1f) * 2) as u8;
                if !is_gps_p_code(meas.signal_type) {
                    meas.raw_cn0_dbhz = meas.raw_cn0_dbhz.saturating_add(10);
                }
                *pp2_idx += 6;
            }
        }
    }
}

fn add_mp_info(meas: &mut InternalMeasurement, mp: Option<&Meas3MpBlock>, idx: &mut usize) {
    let Some(mp) = mp else {
        meas.mp_mm = 0;
        meas.carrier_mp_1_512c = 0;
        return;
    };
    let Some(base) = mp.data.get(*idx / 8) else {
        meas.mp_mm = 0;
        meas.carrier_mp_1_512c = 0;
        return;
    };
    let value = *base as u32
        | (mp.data.get(*idx / 8 + 1).copied().unwrap_or(0) as u32) << 8
        | (mp.data.get(*idx / 8 + 2).copied().unwrap_or(0) as u32) << 16
        | (mp.data.get(*idx / 8 + 3).copied().unwrap_or(0) as u32) << 24;
    let value = value >> (*idx % 8);
    match value & 3 {
        0 => {
            meas.mp_mm = 0;
            meas.carrier_mp_1_512c = 0;
            *idx += 2;
        }
        1 | 2 => {
            let code = ((value >> 2) & 0x7f) as i16 * 10;
            let carrier = ((value >> 9) & 0x1f) as i16;
            meas.mp_mm = if (value & 3) == 1 { code } else { -code };
            meas.carrier_mp_1_512c = if carrier < 16 {
                carrier as i8
            } else {
                (carrier - 32) as i8
            };
            *idx += 14;
        }
        _ => {
            let code = ((value >> 2) & 0x7ff) as i16;
            let carrier = ((value >> 13) & 0xff) as i16;
            meas.mp_mm = if code < 1024 {
                code * 10
            } else {
                (code - 2048) * 10
            };
            meas.carrier_mp_1_512c = if carrier < 128 {
                carrier as i8
            } else {
                (carrier - 256) as i8
            };
            *idx += 21;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::block_ids;

    fn build_sbf_block(block_id: u16, body: &[u8]) -> Vec<u8> {
        let block_data_len = 12 + body.len();
        let mut total_len = (2 + block_data_len) as u16;
        while (total_len as usize & 0x03) != 0 {
            total_len += 1;
        }
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;
        data[4..6].copy_from_slice(&block_id.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&1000u32.to_le_bytes());
        data[12..14].copy_from_slice(&2200u16.to_le_bytes());
        data[14..14 + body.len()].copy_from_slice(body);
        data
    }

    fn parse_doppler_block(payload: &[u8]) -> Meas3DopplerBlock {
        let mut body = vec![0u8; 1];
        body.extend_from_slice(payload);
        let raw = build_sbf_block(block_ids::MEAS3_DOPPLER, &body);
        let (block, _) = SbfBlock::parse(&raw).unwrap();
        let SbfBlock::Meas3Doppler(doppler) = block else {
            panic!("expected Meas3Doppler");
        };
        doppler
    }

    fn encode_pr_rate(abs_prr_mm_s: u32, width_bytes: usize, negative: bool) -> Vec<u8> {
        let sign = u32::from(negative);
        let value = match width_bytes {
            1 => (abs_prr_mm_s << 2) | sign,
            2 => (abs_prr_mm_s << 3) | 0b010 | sign,
            3 => (abs_prr_mm_s << 4) | 0b0110 | sign,
            4 => (abs_prr_mm_s << 4) | 0b1110 | sign,
            _ => panic!("unsupported Meas3Doppler width"),
        };
        value.to_le_bytes()[..width_bytes].to_vec()
    }

    #[test]
    fn meas3_decoder_decodes_single_reference_measurement() {
        let constellation_header = [0x01u8, 0x01];
        let bf1 = 1u32 | (1000u32 << 1) | (1u32 << 20) | (10u32 << 23);
        let mut master = Vec::new();
        master.extend_from_slice(&bf1.to_le_bytes());
        master.extend_from_slice(&10_000u32.to_le_bytes());

        let mut body = vec![0u8; 6];
        body[2..4].copy_from_slice(&(1u16).to_le_bytes());
        body.extend_from_slice(&constellation_header);
        body.extend_from_slice(&master);

        let raw = build_sbf_block(block_ids::MEAS3_RANGES, &body);
        let (block, _) = SbfBlock::parse(&raw).unwrap();
        let SbfBlock::Meas3Ranges(ranges) = block else {
            panic!("expected Meas3Ranges");
        };

        let mut decoder = Meas3Decoder::new();
        let epoch = decoder.decode(&ranges, None, None, None, None).unwrap();
        assert_eq!(epoch.antenna_id, 0);
        assert!(epoch.is_reference_epoch);
        assert_eq!(epoch.num_satellites(), 1);
        assert_eq!(epoch.num_measurements(), 1);
        let sat = &epoch.satellites[0];
        assert_eq!(sat.sat_id.to_string(), "G01");
        let meas = &sat.measurements[0];
        assert_eq!(meas.signal_type, SignalType::L1CA);
        assert!((meas.cn0_dbhz().unwrap() - 34.0).abs() < 1e-6);
        assert_eq!(meas.lock_time_ms(), Some(60_000));
        assert!(meas.pseudorange_m().unwrap() > 19_000_000.0);
    }

    #[test]
    fn meas3_decoder_decodes_reference_epoch_with_slave_signal() {
        let constellation_header = [0x01u8, 0x01];
        let master_bf1 = 1u32 | (1000u32 << 1) | (1u32 << 20) | (10u32 << 23) | (1u32 << 28);
        let mut master = Vec::new();
        master.extend_from_slice(&master_bf1.to_le_bytes());
        master.extend_from_slice(&10_000u32.to_le_bytes());

        let slave_bf1 = 1u32 | (1000u32 << 1);
        let slave_bf2 = 1u8 | (10u8 << 3);
        let mut slave = Vec::new();
        slave.extend_from_slice(&slave_bf1.to_le_bytes());
        slave.push(slave_bf2);

        let mut body = vec![0u8; 6];
        body[2..4].copy_from_slice(&(1u16).to_le_bytes());
        body.extend_from_slice(&constellation_header);
        body.extend_from_slice(&master);
        body.extend_from_slice(&slave);

        let raw = build_sbf_block(block_ids::MEAS3_RANGES, &body);
        let (block, _) = SbfBlock::parse(&raw).unwrap();
        let SbfBlock::Meas3Ranges(ranges) = block else {
            panic!("expected Meas3Ranges");
        };

        let mut decoder = Meas3Decoder::new();
        let epoch = decoder.decode(&ranges, None, None, None, None).unwrap();
        assert!(epoch.is_reference_epoch);
        assert_eq!(epoch.num_satellites(), 1);
        assert_eq!(epoch.num_measurements(), 2);

        let sat = &epoch.satellites[0];
        assert_eq!(sat.measurements.len(), 2);
        assert_eq!(sat.measurements[0].signal_type, SignalType::L1CA);
        assert_eq!(sat.measurements[1].signal_type, SignalType::L2C);
        assert!(sat.measurements[1].pseudorange_m().is_some());
    }

    #[test]
    fn decode_master_long_reads_pr_rate_after_continuation_byte() {
        let signal_table = prepare_signal_table(Meas3SatSystem::Gps, 0);
        let reference_sat = ReferenceSatellite::default();
        let bf1 = (1u32 << 6) | (1u32 << 28);
        let pr_rate_64mm_s = 0x1234i16;

        let mut buf = Vec::new();
        buf.extend_from_slice(&bf1.to_le_bytes());
        buf.extend_from_slice(&10_000u32.to_le_bytes());
        buf.extend_from_slice(&((1u16 << 6) | (20u16) | (1u16 << 15)).to_le_bytes());
        buf.push(0x55);
        buf.extend_from_slice(&pr_rate_64mm_s.to_le_bytes());

        let (_, _, _, decoded_pr_rate, consumed) = decode_master(
            &buf,
            MasterDecodeContext {
                signal_table: &signal_table,
                glonass_fn: 0,
                short_pr_base_m: PR_BASE_M[Meas3SatSystem::Gps as usize],
                sig_idx_master_short: 0,
                reference_sat: &reference_sat,
                time_since_ref_epoch_ms: 0,
                pr_rate_available: true,
            },
        )
        .unwrap();

        assert_eq!(decoded_pr_rate, pr_rate_64mm_s);
        assert_eq!(consumed, 13);
    }

    #[test]
    fn decode_master_long_with_cont_requires_full_pr_rate_field() {
        let signal_table = prepare_signal_table(Meas3SatSystem::Gps, 0);
        let reference_sat = ReferenceSatellite::default();
        let bf1 = (1u32 << 6) | (1u32 << 28);

        let mut buf = Vec::new();
        buf.extend_from_slice(&bf1.to_le_bytes());
        buf.extend_from_slice(&10_000u32.to_le_bytes());
        buf.extend_from_slice(&((1u16 << 6) | (20u16) | (1u16 << 15)).to_le_bytes());
        buf.push(0x55);
        buf.push(0x34);

        let err = decode_master(
            &buf,
            MasterDecodeContext {
                signal_table: &signal_table,
                glonass_fn: 0,
                short_pr_base_m: PR_BASE_M[Meas3SatSystem::Gps as usize],
                sig_idx_master_short: 0,
                reference_sat: &reference_sat,
                time_since_ref_epoch_ms: 0,
                pr_rate_available: true,
            },
        )
        .unwrap_err();

        assert!(err.to_string().contains("Meas3 master-long is truncated"));
    }

    #[test]
    fn master_doppler_marks_three_byte_dnu_as_invalid() {
        let doppler = parse_doppler_block(&encode_pr_rate((1 << 20) - 1, 3, true));
        let mut meas = InternalMeasurement::default();
        let mut idx = 0usize;

        add_master_doppler(&mut meas, Some(&doppler), L1_WAVELENGTH, 0, &mut idx);

        assert_eq!(idx, 3);
        assert_eq!(meas.doppler_hz, F32_NOTVALID);
    }

    #[test]
    fn slave_doppler_marks_four_byte_dnu_as_invalid() {
        let doppler = parse_doppler_block(&encode_pr_rate((1 << 28) - 1, 4, true));
        let master = InternalMeasurement {
            doppler_hz: 125.0,
            ..Default::default()
        };
        let mut meas = InternalMeasurement::default();
        let mut idx = 0usize;

        add_slave_doppler(
            &mut meas,
            &master,
            Some(&doppler),
            L1_WAVELENGTH,
            L2_WAVELENGTH,
            &mut idx,
        );

        assert_eq!(idx, 4);
        assert_eq!(meas.doppler_hz, F32_NOTVALID);
    }

    #[test]
    fn meas3_block_set_collects_blocks() {
        let mut set = Meas3BlockSet::default();
        let body = [0u8; 6];
        let raw = build_sbf_block(block_ids::MEAS3_RANGES, &body);
        let (block, _) = SbfBlock::parse(&raw).unwrap();
        assert!(set.insert_block(&block));
        assert_eq!(set.tow_ms(), Some(1000));
    }

    #[test]
    fn meas3_pp_keeps_section2_offset_in_payload() {
        let body = [
            0x05u8, // Flags
            0x01,   // Section-2 offset in 4-byte units
            0x13,   // APMEInSync=1, LockCount=9
            0x00, 0x00, 0x00, 0x00, // start2 marker low nibble == 0
            0x00, 0x21, // section-2 present, raw CN0 field = 16 -> 32 + 10 offset
            0x00,
        ];
        let raw = build_sbf_block(block_ids::MEAS3_PP, &body);
        let (block, _) = SbfBlock::parse(&raw).unwrap();
        let SbfBlock::Meas3Pp(pp) = block else {
            panic!("expected Meas3Pp");
        };

        let mut meas = InternalMeasurement {
            cn0_dbhz: 34.0,
            signal_type: SignalType::L1CA,
            ..Default::default()
        };
        let mut pp1_idx = 0usize;
        let mut pp2_idx = 0usize;

        add_pp_info(&mut meas, Some(&pp), &mut pp1_idx, &mut pp2_idx);

        assert_eq!(pp.flags, 0x05);
        assert_eq!(pp.antenna_id(), 0x05);
        assert_eq!(meas.lock_count, 9);
        assert_eq!(meas.raw_cn0_dbhz, 42);
        assert_ne!(meas.flags & MEASFLAG_APMEINSYNC, 0);
        assert_eq!(pp1_idx, 5);
        assert_eq!(pp2_idx, 6);
    }
}
