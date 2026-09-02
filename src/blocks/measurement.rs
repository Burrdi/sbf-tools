//! Measurement blocks (MeasEpoch_v2)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::{SatelliteId, SignalType};
use std::collections::HashSet;

use super::block_ids;
use super::dnu::{u16_or_none, u8_or_none, I32_DNU, U16_DNU};
use super::SbfBlockParse;

// ============================================================================
// MeasEpoch Type1 Sub-block (raw)
// ============================================================================

/// Raw Type1 sub-block data from MeasEpoch
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeasEpochType1Raw {
    pub rx_channel: u8,
    pub signal_type: u8,
    pub svid: u8,
    pub misc: u8,
    pub code_lsb: u32,
    pub doppler: i32,
    pub carrier_lsb: u16,
    pub carrier_msb: i8,
    pub cn0: u8,
    pub lock_time: u16,
    pub obs_info: u8,
    pub n2: u8,
}

impl MeasEpochType1Raw {
    /// Unsigned MSB of the pseudorange from bits 0-3 of `Misc`.
    pub fn code_msb(&self) -> u8 {
        self.misc & 0x0f
    }

    /// Antenna ID from bits 5-7 of the Type field.
    pub fn antenna_id(&self) -> u8 {
        self.signal_type >> 5
    }

    /// Decoded global SBF signal number.
    pub fn signal_number(&self) -> u8 {
        signal_number(self.signal_type, self.obs_info)
    }
}

/// Raw Type2 sub-block data from MeasEpoch.
///
/// Type2 measurements inherit their receiver channel and satellite from the
/// preceding Type1 sub-block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeasEpochType2Raw {
    pub signal_type: u8,
    pub lock_time: u8,
    pub cn0: u8,
    pub offsets_msb: u8,
    pub carrier_msb: i8,
    pub obs_info: u8,
    pub code_offset_lsb: u16,
    pub carrier_lsb: u16,
    pub doppler_offset_lsb: u16,
}

impl MeasEpochType2Raw {
    /// Antenna ID from bits 5-7 of the Type field.
    pub fn antenna_id(&self) -> u8 {
        self.signal_type >> 5
    }

    /// Decoded global SBF signal number.
    pub fn signal_number(&self) -> u8 {
        signal_number(self.signal_type, self.obs_info)
    }

    /// Signed 3-bit MSB of the pseudorange offset.
    pub fn code_offset_msb(&self) -> i8 {
        sign_extend(self.offsets_msb & 0x07, 3)
    }

    /// Signed 5-bit MSB of the Doppler offset.
    pub fn doppler_offset_msb(&self) -> i8 {
        sign_extend(self.offsets_msb >> 3, 5)
    }
}

/// Original SBF sub-block from which a processed measurement was decoded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeasEpochMeasurementRaw {
    Type1(MeasEpochType1Raw),
    Type2(MeasEpochType2Raw),
}

// ============================================================================
// Satellite Measurement (processed)
// ============================================================================

/// Processed satellite measurement from MeasEpoch
#[derive(Debug, Clone)]
pub struct SatelliteMeasurement {
    /// Satellite ID
    pub sat_id: SatelliteId,
    /// Signal type
    pub signal_type: SignalType,
    /// Receiver tracking channel.
    rx_channel: u8,
    /// Antenna ID from bits 5-7 of the SBF Type field.
    antenna_id: u8,
    /// Decoded global SBF signal number.
    signal_number: u8,
    /// GLONASS frequency number, inherited by Type2 measurements.
    glonass_frequency_number: Option<i8>,
    /// Pseudorange in metres.
    pseudorange_m: Option<f64>,
    /// Carrier phase in cycles.
    carrier_phase_cycles: Option<f64>,
    /// Doppler in Hz, including the Type2 frequency-ratio reconstruction.
    doppler_hz: Option<f64>,
    /// Original Type1 or Type2 wire fields.
    raw: MeasEpochMeasurementRaw,
    /// Raw CN0 value (use cn0_dbhz() for scaling per SBF spec)
    cn0_raw: u8,
    /// Raw lock time
    lock_time_raw: u16,
    /// Whether `lock_time_raw` is available.
    lock_time_valid: bool,
    /// Observation info flags
    pub obs_info: u8,
}

impl SatelliteMeasurement {
    /// Receiver tracking channel from the parent Type1 sub-block.
    pub fn rx_channel(&self) -> u8 {
        self.rx_channel
    }

    /// Antenna ID from bits 5-7 of the SBF Type field.
    pub fn antenna_id(&self) -> u8 {
        self.antenna_id
    }

    /// Decoded global SBF signal number.
    pub fn signal_number(&self) -> u8 {
        self.signal_number
    }

    /// GLONASS frequency number used to determine the carrier frequency.
    pub fn glonass_frequency_number(&self) -> Option<i8> {
        self.glonass_frequency_number
    }

    /// Pseudorange in metres, or `None` for the documented DNU value.
    pub fn pseudorange_m(&self) -> Option<f64> {
        self.pseudorange_m
    }

    /// Carrier phase in cycles, or `None` for DNU or an unknown wavelength.
    pub fn carrier_phase_cycles(&self) -> Option<f64> {
        self.carrier_phase_cycles
    }

    /// Carrier frequency in Hz used to reconstruct the phase.
    pub fn carrier_frequency_hz(&self) -> Option<f64> {
        carrier_frequency_hz(self.signal_type, self.glonass_frequency_number)
    }

    /// Carrier wavelength in metres used to reconstruct the phase.
    pub fn carrier_wavelength_m(&self) -> Option<f64> {
        self.carrier_frequency_hz()
            .map(|frequency_hz| SPEED_OF_LIGHT_M_S / frequency_hz)
    }

    /// Original Type1 or Type2 wire fields.
    pub fn raw(&self) -> &MeasEpochMeasurementRaw {
        &self.raw
    }

    /// Get CN0 in dB-Hz (scaled per SBF Reference Guide)
    pub fn cn0_dbhz(&self) -> f64 {
        self.cn0_dbhz_opt().unwrap_or(0.0)
    }

    /// Get CN0 in dB-Hz, or `None` when the raw C/N0 field is unavailable.
    pub fn cn0_dbhz_opt(&self) -> Option<f64> {
        let cn0_raw = u8_or_none(self.cn0_raw)?;
        let base = cn0_raw as f64 * 0.25;
        match self.signal_type {
            // Signal numbers 1 and 2 (GPS L1P, GPS L2P) have no +10 dB offset
            SignalType::L1PY | SignalType::L2P => Some(base),
            _ => Some(base + 10.0),
        }
    }

    /// Get raw CN0 value
    pub fn cn0_raw(&self) -> u8 {
        self.cn0_raw
    }

    /// Check if CN0 is valid (not 255)
    pub fn cn0_valid(&self) -> bool {
        u8_or_none(self.cn0_raw).is_some()
    }

    /// Get Doppler in Hz (scaled).
    ///
    /// Returns `0.0` when the absolute Doppler field is unavailable. Use
    /// [`Self::doppler_hz_opt`] to distinguish unavailable from a real zero.
    pub fn doppler_hz(&self) -> f64 {
        self.doppler_hz_opt().unwrap_or(0.0)
    }

    /// Get Doppler in Hz (scaled), or `None` when unavailable.
    pub fn doppler_hz_opt(&self) -> Option<f64> {
        self.doppler_hz
    }

    /// Get the raw absolute Type1 Doppler value.
    ///
    /// Type2 carries a Doppler offset instead of an absolute wire value; for
    /// Type2 this legacy accessor returns the documented Type1 DNU sentinel.
    /// Use [`Self::raw`] to inspect the original Type2 offset fields.
    pub fn doppler_raw(&self) -> i32 {
        match &self.raw {
            MeasEpochMeasurementRaw::Type1(raw) => raw.doppler,
            MeasEpochMeasurementRaw::Type2(_) => I32_DNU,
        }
    }

    /// Get lock time in seconds.
    ///
    /// Returns `0.0` when the lock time field is unavailable. Use
    /// [`Self::lock_time_seconds_opt`] to distinguish unavailable from a real zero.
    pub fn lock_time_seconds(&self) -> f64 {
        self.lock_time_seconds_opt().unwrap_or(0.0)
    }

    /// Get lock time in seconds, or `None` when unavailable.
    pub fn lock_time_seconds_opt(&self) -> Option<f64> {
        if self.lock_time_valid {
            Some(self.lock_time_raw as f64)
        } else {
            None
        }
    }

    /// Get raw lock time value
    pub fn lock_time_raw(&self) -> u16 {
        self.lock_time_raw
    }

    /// Check if half-cycle ambiguity is resolved.
    ///
    /// Per SBF, bit 2 is set when a half-cycle ambiguity is present.
    pub fn half_cycle_resolved(&self) -> bool {
        (self.obs_info & 0x04) == 0
    }

    /// Check if smoothing is active.
    ///
    /// Per SBF, bit 0 indicates code smoothing.
    pub fn smoothing_active(&self) -> bool {
        (self.obs_info & 0x01) != 0
    }
}

const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

fn sign_extend(value: u8, bits: u32) -> i8 {
    let shift = 8 - bits;
    ((value << shift) as i8) >> shift
}

fn signal_number(type_field: u8, obs_info: u8) -> u8 {
    let sig_idx = type_field & 0x1f;
    if sig_idx == 31 {
        32 + ((obs_info >> 3) & 0x1f)
    } else {
        sig_idx
    }
}

fn glonass_frequency_number(signal_number: u8, obs_info: u8) -> Option<i8> {
    if (8..=11).contains(&signal_number) {
        Some(((obs_info >> 3) & 0x1f) as i8 - 8)
    } else {
        None
    }
}

fn carrier_frequency_hz(
    signal_type: SignalType,
    glonass_frequency_number: Option<i8>,
) -> Option<f64> {
    use SignalType::*;

    let frequency_hz = match signal_type {
        L1CA | L1C | L1PY | E1 | B1C | QZSSL1CA | QZSSL1C | QZSSL1S | QZSSL1CB | SBASL1
        | NavICL1 => 1_575.42e6,
        L2C | L2P | L2PY | QZSSL2C => 1_227.60e6,
        L5 | E5a | B2a | QZSSL5 | QZSSL5S | SBASL5 | NavICL5 => 1_176.45e6,
        E5b | B2I | B2b => 1_207.14e6,
        E5AltBOC => 1_191.795e6,
        E6 | QZSSL6 => 1_278.75e6,
        B1I => 1_561.098e6,
        B3I => 1_268.52e6,
        G1CA | G1P => {
            let frequency_number = glonass_frequency_number?;
            1_602.0e6 + frequency_number as f64 * 562_500.0
        }
        G2CA | G2P => {
            let frequency_number = glonass_frequency_number?;
            1_246.0e6 + frequency_number as f64 * 437_500.0
        }
        G3 => 1_202.025e6,
        LBand | Other(_) => return None,
    };
    Some(frequency_hz)
}

fn type1_pseudorange_m(raw: &MeasEpochType1Raw) -> Option<f64> {
    let code_msb = raw.misc & 0x0f;
    if code_msb == 0 && raw.code_lsb == 0 {
        return None;
    }

    let millimetres = u64::from(code_msb) * 4_294_967_296 + u64::from(raw.code_lsb);
    Some(millimetres as f64 * 0.001)
}

fn type2_pseudorange_m(parent_pseudorange_m: Option<f64>, raw: &MeasEpochType2Raw) -> Option<f64> {
    let code_offset_msb = raw.code_offset_msb();
    if code_offset_msb == -4 && raw.code_offset_lsb == 0 {
        return None;
    }

    parent_pseudorange_m.map(|parent| {
        parent + (code_offset_msb as f64 * 65_536.0 + f64::from(raw.code_offset_lsb)) * 0.001
    })
}

fn compute_carrier_phase_cycles(
    pseudorange_m: Option<f64>,
    frequency_hz: Option<f64>,
    carrier_msb: i8,
    carrier_lsb: u16,
) -> Option<f64> {
    if carrier_msb == -128 && carrier_lsb == 0 {
        return None;
    }

    Some(
        pseudorange_m? / (SPEED_OF_LIGHT_M_S / frequency_hz?)
            + (carrier_msb as f64 * 65_536.0 + f64::from(carrier_lsb)) * 0.001,
    )
}

fn type2_doppler_hz(
    parent_doppler_hz: Option<f64>,
    parent_frequency_hz: Option<f64>,
    frequency_hz: Option<f64>,
    raw: &MeasEpochType2Raw,
) -> Option<f64> {
    let doppler_offset_msb = raw.doppler_offset_msb();
    if doppler_offset_msb == -16 && raw.doppler_offset_lsb == 0 {
        return None;
    }

    let frequency_ratio = frequency_hz? / parent_frequency_hz?;
    Some(
        parent_doppler_hz? * frequency_ratio
            + (doppler_offset_msb as f64 * 65_536.0 + f64::from(raw.doppler_offset_lsb)) * 0.0001,
    )
}

// ============================================================================
// MeasEpoch Block
// ============================================================================

/// MeasEpoch_v2 block (Block ID 4027)
///
/// Contains satellite measurements including code, carrier, Doppler, and CN0.
#[derive(Debug, Clone)]
pub struct MeasEpochBlock {
    /// Time of week in milliseconds
    tow_ms: u32,
    /// GPS week number
    wnc: u16,
    /// Number of Type1 sub-blocks
    pub n1: u8,
    /// Length of each Type1 sub-block
    pub sb1_length: u8,
    /// Length of each Type2 sub-block
    pub sb2_length: u8,
    /// Common flags
    pub common_flags: u8,
    /// Cumulative clock jumps modulo 256 ms (raw field value).
    pub cum_clk_jumps: u8,
    /// Satellite measurements
    pub measurements: Vec<SatelliteMeasurement>,
}

impl MeasEpochBlock {
    /// Get TOW in seconds
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    /// Get raw TOW in milliseconds
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }

    /// Get week number
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get number of satellites with measurements
    pub fn num_satellites(&self) -> usize {
        self.measurements
            .iter()
            .map(|measurement| &measurement.sat_id)
            .collect::<HashSet<_>>()
            .len()
    }

    /// Get total number of signal measurements, including Type2 signals.
    pub fn num_measurements(&self) -> usize {
        self.measurements.len()
    }

    /// Get measurements for a specific satellite
    pub fn measurements_for_sat(&self, sat_id: &SatelliteId) -> Vec<&SatelliteMeasurement> {
        self.measurements
            .iter()
            .filter(|m| &m.sat_id == sat_id)
            .collect()
    }

    /// Get all valid CN0 measurements
    pub fn valid_cn0_measurements(&self) -> Vec<&SatelliteMeasurement> {
        self.measurements.iter().filter(|m| m.cn0_valid()).collect()
    }

    /// Find the companion MeasExtra channel for a decoded measurement.
    ///
    /// The epoch timestamp is checked before matching antenna, receiver
    /// channel, and global SBF signal number.
    pub fn matching_extra_channel<'a>(
        &self,
        extra: &'a MeasExtraBlock,
        measurement: &SatelliteMeasurement,
    ) -> Option<&'a MeasExtraChannel> {
        if self.wnc != extra.wnc || self.tow_ms != extra.tow_ms {
            return None;
        }

        extra
            .channels
            .iter()
            .find(|channel| channel.matches_measurement(measurement))
    }
}

impl SbfBlockParse for MeasEpochBlock {
    const BLOCK_ID: u16 = block_ids::MEAS_EPOCH;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let full_len = header.length as usize;
        if data.len() < full_len - 2 {
            // -2 for sync bytes not in data
            return Err(SbfError::IncompleteBlock {
                needed: full_len,
                have: data.len() + 2,
            });
        }

        // MeasEpoch structure (offsets from data start, which is after sync):
        // 0-1: CRC, 2-3: ID, 4-5: Length
        // 6-9: TOW, 10-11: WNc
        // 12: N1, 13: SB1Length, 14: SB2Length
        // 15: CommonFlags, 16: CumClkJumps
        // 17: Reserved (Rev 1+)
        // Type1 sub-blocks start at offset 17 (Rev 0) or 18 (Rev 1+)

        if data.len() < 17 {
            return Err(SbfError::ParseError("MeasEpoch too short".into()));
        }

        let n1 = data[12];
        let sb1_length = data[13];
        let sb2_length = data[14];
        let common_flags = data[15];
        let cum_clk_jumps = data[16];

        if n1 > 0 && sb1_length < 20 {
            return Err(SbfError::ParseError(
                "MeasEpoch SB1Length is smaller than the Type1 layout".into(),
            ));
        }

        let sb1_length_usize = sb1_length as usize;
        let sb2_length_usize = sb2_length as usize;

        // Type1 sub-blocks start at offset 17 (Rev 0) or 18 (Rev 1+)
        let mut offset = 17;
        if header.block_rev >= 1 {
            offset += 1; // Reserved byte in Rev 1+
        }

        let mut measurements = Vec::new();

        for _ in 0..n1 {
            if offset + sb1_length_usize > data.len() {
                return Err(SbfError::ParseError(
                    "MeasEpoch SB1 exceeds block length".into(),
                ));
            }

            // Type-1 sub-block structure (20 bytes typical):
            // 0: RxChannel, 1: Type, 2: SVID, 3: Misc
            // 4-7: CodeLSB, 8-11: Doppler, 12-13: CarrierLSB
            // 14: CarrierMSB, 15: CN0, 16-17: LockTime
            // 18: ObsInfo, 19: N2

            let raw_type1 = MeasEpochType1Raw {
                rx_channel: data[offset],
                signal_type: data[offset + 1],
                svid: data[offset + 2],
                misc: data[offset + 3],
                code_lsb: u32::from_le_bytes([
                    data[offset + 4],
                    data[offset + 5],
                    data[offset + 6],
                    data[offset + 7],
                ]),
                doppler: i32::from_le_bytes([
                    data[offset + 8],
                    data[offset + 9],
                    data[offset + 10],
                    data[offset + 11],
                ]),
                carrier_lsb: u16::from_le_bytes([data[offset + 12], data[offset + 13]]),
                carrier_msb: data[offset + 14] as i8,
                cn0: data[offset + 15],
                lock_time: u16::from_le_bytes([data[offset + 16], data[offset + 17]]),
                obs_info: data[offset + 18],
                n2: data[offset + 19],
            };

            let sig_num = signal_number(raw_type1.signal_type, raw_type1.obs_info);
            let signal_type = SignalType::from_signal_number(sig_num);
            let glo_frequency_number = glonass_frequency_number(sig_num, raw_type1.obs_info);
            let frequency_hz = carrier_frequency_hz(signal_type, glo_frequency_number);
            let pseudorange_m = type1_pseudorange_m(&raw_type1);
            let carrier_phase_cycles = compute_carrier_phase_cycles(
                pseudorange_m,
                frequency_hz,
                raw_type1.carrier_msb,
                raw_type1.carrier_lsb,
            );
            let doppler_hz =
                (raw_type1.doppler != I32_DNU).then_some(raw_type1.doppler as f64 * 0.0001);
            let lock_time_valid = raw_type1.lock_time != U16_DNU;
            let sat_id = SatelliteId::from_svid(raw_type1.svid);

            if let Some(sat_id) = &sat_id {
                measurements.push(SatelliteMeasurement {
                    sat_id: sat_id.clone(),
                    signal_type,
                    rx_channel: raw_type1.rx_channel,
                    antenna_id: raw_type1.signal_type >> 5,
                    signal_number: sig_num,
                    glonass_frequency_number: glo_frequency_number,
                    pseudorange_m,
                    carrier_phase_cycles,
                    doppler_hz,
                    raw: MeasEpochMeasurementRaw::Type1(raw_type1.clone()),
                    cn0_raw: raw_type1.cn0,
                    lock_time_raw: raw_type1.lock_time,
                    lock_time_valid,
                    obs_info: raw_type1.obs_info,
                });
            }

            offset += sb1_length_usize;

            // Parse Type2 sub-blocks (additional signals for the same satellite).
            if raw_type1.n2 > 0 {
                if sb2_length_usize < 12 {
                    return Err(SbfError::ParseError(
                        "MeasEpoch SB2Length is smaller than the Type2 layout".into(),
                    ));
                }

                for _ in 0..raw_type1.n2 {
                    if offset + sb2_length_usize > data.len() {
                        return Err(SbfError::ParseError(
                            "MeasEpoch SB2 exceeds block length".into(),
                        ));
                    }

                    // Type-2 sub-block structure:
                    // 0: Type, 1: LockTime (short), 2: CN0
                    // 3: OffsetMSB, 4: CarrierMSB, 5: ObsInfo
                    // 6-7: CodeOffsetLSB, 8-9: CarrierLSB, 10-11: DopplerOffsetLSB

                    let raw_type2 = MeasEpochType2Raw {
                        signal_type: data[offset],
                        lock_time: data[offset + 1],
                        cn0: data[offset + 2],
                        offsets_msb: data[offset + 3],
                        carrier_msb: data[offset + 4] as i8,
                        obs_info: data[offset + 5],
                        code_offset_lsb: u16::from_le_bytes([data[offset + 6], data[offset + 7]]),
                        carrier_lsb: u16::from_le_bytes([data[offset + 8], data[offset + 9]]),
                        doppler_offset_lsb: u16::from_le_bytes([
                            data[offset + 10],
                            data[offset + 11],
                        ]),
                    };

                    let sig_num_2 = signal_number(raw_type2.signal_type, raw_type2.obs_info);
                    let signal_type_2 = SignalType::from_signal_number(sig_num_2);
                    let frequency_hz_2 = carrier_frequency_hz(signal_type_2, glo_frequency_number);
                    let pseudorange_m_2 = type2_pseudorange_m(pseudorange_m, &raw_type2);
                    let carrier_phase_cycles_2 = compute_carrier_phase_cycles(
                        pseudorange_m_2,
                        frequency_hz_2,
                        raw_type2.carrier_msb,
                        raw_type2.carrier_lsb,
                    );
                    let doppler_hz_2 =
                        type2_doppler_hz(doppler_hz, frequency_hz, frequency_hz_2, &raw_type2);
                    let lock_time_2 = u16::from(raw_type2.lock_time);

                    if let Some(sat_id) = &sat_id {
                        measurements.push(SatelliteMeasurement {
                            sat_id: sat_id.clone(),
                            signal_type: signal_type_2,
                            rx_channel: raw_type1.rx_channel,
                            antenna_id: raw_type2.signal_type >> 5,
                            signal_number: sig_num_2,
                            glonass_frequency_number: glo_frequency_number,
                            pseudorange_m: pseudorange_m_2,
                            carrier_phase_cycles: carrier_phase_cycles_2,
                            doppler_hz: doppler_hz_2,
                            raw: MeasEpochMeasurementRaw::Type2(raw_type2.clone()),
                            cn0_raw: raw_type2.cn0,
                            lock_time_raw: lock_time_2,
                            lock_time_valid: raw_type2.lock_time != u8::MAX,
                            obs_info: raw_type2.obs_info,
                        });
                    }

                    offset += sb2_length_usize;
                }
            }
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n1,
            sb1_length,
            sb2_length,
            common_flags,
            cum_clk_jumps,
            measurements,
        })
    }
}

// ============================================================================
// MeasExtra Block
// ============================================================================

/// MeasExtra channel information
#[derive(Debug, Clone)]
pub struct MeasExtraChannel {
    /// Receiver channel
    pub rx_channel: u8,
    /// Signal type (decoded)
    pub signal_type: SignalType,
    /// Raw signal type field
    signal_type_raw: u8,
    /// Decoded global SBF signal number
    signal_number: u8,
    /// Multipath correction (raw, millimeters)
    mp_correction_raw: i16,
    /// Smoothing correction (raw, millimeters)
    smoothing_correction_raw: i16,
    /// Code variance (raw)
    code_var_raw: u16,
    /// Carrier variance (raw)
    carrier_var_raw: u16,
    /// Lock time in seconds (raw)
    lock_time_raw: u16,
    /// Cumulative loss of continuity
    pub cum_loss_cont: u8,
    /// Carrier phase multipath correction (raw, 1/512 cycles) when present
    car_mp_correction_raw: Option<i8>,
    /// Info flags
    pub info: u8,
    /// Misc bitfield when present (rev 3+ sub-block layout)
    misc_raw: Option<u8>,
}

impl MeasExtraChannel {
    /// Whether this channel complements a decoded MeasEpoch measurement.
    pub fn matches_measurement(&self, measurement: &SatelliteMeasurement) -> bool {
        self.rx_channel == measurement.rx_channel
            && self.antenna_id() == measurement.antenna_id
            && self.signal_number == measurement.signal_number
    }

    /// Get raw signal type value
    pub fn signal_type_raw(&self) -> u8 {
        self.signal_type_raw
    }

    /// Get decoded global SBF signal number
    pub fn signal_number(&self) -> u8 {
        self.signal_number
    }

    /// Get antenna ID from the Type field (bits 5-7)
    pub fn antenna_id(&self) -> u8 {
        (self.signal_type_raw >> 5) & 0x07
    }

    /// Multipath correction in meters
    pub fn mp_correction_m(&self) -> f64 {
        self.mp_correction_raw as f64 * 0.001
    }

    /// Smoothing correction in meters
    pub fn smoothing_correction_m(&self) -> f64 {
        self.smoothing_correction_raw as f64 * 0.001
    }

    /// Code variance in m^2
    pub fn code_var_m2(&self) -> f64 {
        self.code_var_m2_opt().unwrap_or(0.0)
    }

    /// Code variance in m^2, or `None` when unavailable.
    pub fn code_var_m2_opt(&self) -> Option<f64> {
        u16_or_none(self.code_var_raw).map(|raw| raw as f64 * 0.0001)
    }

    /// Raw code variance field from the SBF block.
    pub fn code_var_raw(&self) -> u16 {
        self.code_var_raw
    }

    /// Carrier variance in cycles^2
    pub fn carrier_var_cycles2(&self) -> f64 {
        self.carrier_var_cycles2_opt().unwrap_or(0.0)
    }

    /// Carrier variance in cycles^2, or `None` when unavailable.
    pub fn carrier_var_cycles2_opt(&self) -> Option<f64> {
        u16_or_none(self.carrier_var_raw).map(|raw| raw as f64 * 0.000001)
    }

    /// Raw carrier variance field from the SBF block.
    pub fn carrier_var_raw(&self) -> u16 {
        self.carrier_var_raw
    }

    /// Lock time in seconds
    pub fn lock_time_seconds(&self) -> f64 {
        self.lock_time_seconds_opt().unwrap_or(0.0)
    }

    /// Lock time in seconds, or `None` when unavailable.
    pub fn lock_time_seconds_opt(&self) -> Option<f64> {
        u16_or_none(self.lock_time_raw).map(|raw| raw as f64)
    }

    /// Raw lock time value
    pub fn lock_time_raw(&self) -> u16 {
        self.lock_time_raw
    }

    /// Raw carrier multipath correction in units of 1/512 cycles
    pub fn car_mp_correction_raw(&self) -> Option<i8> {
        self.car_mp_correction_raw
    }

    /// Carrier multipath correction in cycles (when present)
    pub fn car_mp_correction_cycles(&self) -> Option<f64> {
        self.car_mp_correction_raw.map(|v| v as f64 / 512.0)
    }

    /// Get raw Misc bitfield (rev 3+)
    pub fn misc_raw(&self) -> Option<u8> {
        self.misc_raw
    }

    /// C/N0 high-resolution extension in dB-Hz offset (rev 3+, bits 0-2)
    pub fn cn0_high_res_dbhz_offset(&self) -> Option<f64> {
        self.misc_raw.map(|misc| (misc & 0x07) as f64 * 0.03125)
    }
}

/// MeasExtra block (Block ID 4000)
///
/// Additional measurement data such as multipath corrections and variances.
#[derive(Debug, Clone)]
pub struct MeasExtraBlock {
    /// Time of week in milliseconds
    tow_ms: u32,
    /// GPS week number
    wnc: u16,
    /// Number of sub-blocks
    pub n: u8,
    /// Sub-block length
    pub sb_length: u8,
    /// Doppler variance factor
    doppler_var_factor: f32,
    /// Channel data
    pub channels: Vec<MeasExtraChannel>,
}

impl MeasExtraBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Doppler variance factor
    pub fn doppler_var_factor(&self) -> f32 {
        self.doppler_var_factor
    }

    /// Number of channels
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }
}

impl SbfBlockParse for MeasExtraBlock {
    const BLOCK_ID: u16 = block_ids::MEAS_EXTRA;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 18 {
            return Err(SbfError::ParseError("MeasExtra too short".into()));
        }

        // Offsets:
        // 12: N
        // 13: SBLength
        // 14-17: DopplerVarFactor (f4)
        let n = data[12];
        let sb_length = data[13];

        if sb_length < 14 {
            return Err(SbfError::ParseError("MeasExtra SBLength too small".into()));
        }

        let doppler_var_factor = f32::from_le_bytes(data[14..18].try_into().unwrap());

        let sb_length_usize = sb_length as usize;
        let mut channels = Vec::new();
        let mut offset = 18;

        for _ in 0..n {
            if offset + sb_length_usize > data.len() {
                return Err(SbfError::ParseError(
                    "MeasExtra sub-block exceeds block length".into(),
                ));
            }

            let rx_channel = data[offset];
            let signal_type_raw = data[offset + 1];
            let mp_correction_raw = i16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let smoothing_correction_raw = i16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            let code_var_raw = u16::from_le_bytes([data[offset + 6], data[offset + 7]]);
            let carrier_var_raw = u16::from_le_bytes([data[offset + 8], data[offset + 9]]);
            let lock_time_raw = u16::from_le_bytes([data[offset + 10], data[offset + 11]]);
            let cum_loss_cont = data[offset + 12];
            let (car_mp_correction_raw, info, misc_raw) = if sb_length_usize >= 16 {
                // Rev 3+ layout includes CarMPCorr, Info, and Misc.
                (
                    Some(data[offset + 13] as i8),
                    data[offset + 14],
                    Some(data[offset + 15]),
                )
            } else if sb_length_usize >= 15 {
                // Intermediate layout includes CarMPCorr and Info.
                (Some(data[offset + 13] as i8), data[offset + 14], None)
            } else {
                // Legacy layout has Info directly after CumLossCont.
                (None, data[offset + 13], None)
            };

            let sig_idx_lo = signal_type_raw & 0x1F;
            let signal_number = if sig_idx_lo == 31 {
                misc_raw
                    .map(|misc| 32 + ((misc >> 3) & 0x1F))
                    .unwrap_or(sig_idx_lo)
            } else {
                sig_idx_lo
            };

            channels.push(MeasExtraChannel {
                rx_channel,
                signal_type: SignalType::from_signal_number(signal_number),
                signal_type_raw,
                signal_number,
                mp_correction_raw,
                smoothing_correction_raw,
                code_var_raw,
                carrier_var_raw,
                lock_time_raw,
                cum_loss_cont,
                car_mp_correction_raw,
                info,
                misc_raw,
            });

            offset += sb_length_usize;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            doppler_var_factor,
            channels,
        })
    }
}

// ============================================================================
// IQCorr Block
// ============================================================================

/// IQ correlation channel sub-block
#[derive(Debug, Clone)]
pub struct IqCorrChannel {
    pub rx_channel: u8,
    pub signal_type: u8,
    pub svid: u8,
    pub corr_iq_msb: u8,
    pub corr_i_lsb: u8,
    pub corr_q_lsb: u8,
    pub carrier_phase_lsb: u16,
}

/// IQCorr block (Block ID 4046)
///
/// Signal-quality metrics from I/Q correlation.
#[derive(Debug, Clone)]
pub struct IqCorrBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    /// Correlation duration in ms
    pub corr_duration: u8,
    pub cum_clk_jumps: i8,
    pub channels: Vec<IqCorrChannel>,
}

impl IqCorrBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }
}

impl SbfBlockParse for IqCorrBlock {
    const BLOCK_ID: u16 = block_ids::IQ_CORR;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        // Header: N, SBLength, CorrDuration, CumClkJumps = 4 bytes at offset 12
        const MIN_HEADER: usize = 16;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("IQCorr too short".into()));
        }

        let n = data[12];
        let sb_length = data[13];
        let corr_duration = data[14];
        let cum_clk_jumps = data[15] as i8;

        if sb_length < 8 {
            return Err(SbfError::ParseError("IQCorr SBLength too small".into()));
        }

        let sb_length_usize = sb_length as usize;
        let mut channels = Vec::new();
        let mut offset = 16;

        for _ in 0..n {
            if offset + sb_length_usize > data.len() {
                return Err(SbfError::ParseError(
                    "IQCorr sub-block exceeds block length".into(),
                ));
            }

            channels.push(IqCorrChannel {
                rx_channel: data[offset],
                signal_type: data[offset + 1],
                svid: data[offset + 2],
                corr_iq_msb: data[offset + 3],
                corr_i_lsb: data[offset + 4],
                corr_q_lsb: data[offset + 5],
                carrier_phase_lsb: u16::from_le_bytes([data[offset + 6], data[offset + 7]]),
            });

            offset += sb_length_usize;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            corr_duration,
            cum_clk_jumps,
            channels,
        })
    }
}

// ============================================================================
// EndOfMeas Block
// ============================================================================

/// EndOfMeas block (Block ID 5922)
///
/// Marker indicating end of measurement blocks for current epoch.
#[derive(Debug, Clone)]
pub struct EndOfMeasBlock {
    tow_ms: u32,
    wnc: u16,
}

impl EndOfMeasBlock {
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

impl SbfBlockParse for EndOfMeasBlock {
    const BLOCK_ID: u16 = block_ids::END_OF_MEAS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 12 {
            return Err(SbfError::ParseError("EndOfMeas too short".into()));
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
    use crate::types::Constellation;

    fn test_measurement(
        signal_type: SignalType,
        cn0_raw: u8,
        doppler_raw: i32,
        doppler_hz: Option<f64>,
        lock_time_raw: u16,
        lock_time_valid: bool,
    ) -> SatelliteMeasurement {
        let raw = MeasEpochType1Raw {
            rx_channel: 1,
            signal_type: 0,
            svid: 1,
            misc: 0,
            code_lsb: 0,
            doppler: doppler_raw,
            carrier_lsb: 0,
            carrier_msb: -128,
            cn0: cn0_raw,
            lock_time: lock_time_raw,
            obs_info: 0,
            n2: 0,
        };

        SatelliteMeasurement {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            signal_type,
            rx_channel: 1,
            antenna_id: 0,
            signal_number: 0,
            glonass_frequency_number: None,
            pseudorange_m: None,
            carrier_phase_cycles: None,
            doppler_hz,
            raw: MeasEpochMeasurementRaw::Type1(raw),
            cn0_raw,
            lock_time_raw,
            lock_time_valid,
            obs_info: 0,
        }
    }

    #[test]
    fn test_satellite_measurement_cn0() {
        let meas = test_measurement(SignalType::L1CA, 160, 1000, Some(0.1), 10, true);

        assert_eq!(meas.cn0_dbhz(), 50.0);
        assert_eq!(meas.cn0_dbhz_opt(), Some(50.0));
        assert!(meas.cn0_valid());
        assert_eq!(meas.doppler_hz_opt(), Some(0.1));
        assert_eq!(meas.lock_time_seconds_opt(), Some(10.0));
    }

    #[test]
    fn test_satellite_measurement_cn0_gps_p_no_offset() {
        // GPS L1P (signal number 1) has no +10 dB offset.
        let meas = test_measurement(SignalType::L1PY, 160, 1000, Some(0.1), 10, true);

        assert_eq!(meas.cn0_dbhz(), 40.0);
    }

    #[test]
    fn test_satellite_measurement_invalid_cn0() {
        let meas = test_measurement(SignalType::L1CA, 255, 0, None, 0, false);

        assert!(!meas.cn0_valid());
        assert_eq!(meas.cn0_dbhz_opt(), None);
        assert_eq!(meas.cn0_dbhz(), 0.0);
    }

    #[test]
    fn test_lock_time_encoding() {
        // Linear encoding (seconds)
        let meas = test_measurement(SignalType::L1CA, 160, 0, Some(0.0), 30, true);
        assert_eq!(meas.lock_time_seconds(), 30.0);
        assert_eq!(meas.lock_time_seconds_opt(), Some(30.0));

        // Larger values are still linear, just clipped by the receiver if too large
        let meas2 = SatelliteMeasurement {
            lock_time_raw: 96,
            ..meas
        };
        assert_eq!(meas2.lock_time_seconds(), 96.0);
    }

    #[test]
    fn test_satellite_measurement_doppler_and_lock_time_dnu() {
        let meas = test_measurement(SignalType::L1CA, 160, I32_DNU, None, U16_DNU, false);

        assert_eq!(meas.doppler_raw(), I32_DNU);
        assert_eq!(meas.doppler_hz_opt(), None);
        assert_eq!(meas.doppler_hz(), 0.0);
        assert_eq!(meas.lock_time_raw(), U16_DNU);
        assert_eq!(meas.lock_time_seconds_opt(), None);
        assert_eq!(meas.lock_time_seconds(), 0.0);
    }

    #[test]
    fn meas_epoch_decodes_type1_and_type2_observables_and_preserves_raw_fields() {
        let mut data = vec![0u8; 50];
        data[12] = 1; // N1
        data[13] = 20; // SB1Length
        data[14] = 12; // SB2Length
        data[17] = 0; // Reserved in revision 1

        let sb1 = 18;
        data[sb1] = 7; // RxChannel
        data[sb1 + 1] = 0; // GPS L1CA, antenna 0
        data[sb1 + 2] = 1; // GPS G01
        data[sb1 + 3] = 4; // CodeMSB
        data[sb1 + 4..sb1 + 8].copy_from_slice(&3_000_000_000_u32.to_le_bytes());
        data[sb1 + 8..sb1 + 12].copy_from_slice(&(-123_456_i32).to_le_bytes());
        data[sb1 + 12..sb1 + 14].copy_from_slice(&12_345_u16.to_le_bytes());
        data[sb1 + 14] = (-2_i8) as u8;
        data[sb1 + 15] = 160;
        data[sb1 + 16..sb1 + 18].copy_from_slice(&30_u16.to_le_bytes());
        data[sb1 + 18] = 0;
        data[sb1 + 19] = 1; // One Type2 signal

        let sb2 = sb1 + 20;
        data[sb2] = 3; // GPS L2C, antenna 0
        data[sb2 + 1] = 5;
        data[sb2 + 2] = 140;
        data[sb2 + 3] = 0x0f; // CodeOffsetMSB=-1, DopplerOffsetMSB=1
        data[sb2 + 4] = 1;
        data[sb2 + 5] = 0;
        data[sb2 + 6..sb2 + 8].copy_from_slice(&1_000_u16.to_le_bytes());
        data[sb2 + 8..sb2 + 10].copy_from_slice(&2_000_u16.to_le_bytes());
        data[sb2 + 10..sb2 + 12].copy_from_slice(&2_000_u16.to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::MEAS_EPOCH,
            block_rev: 1,
            length: 52,
            tow_ms: 123_456,
            wnc: 2_400,
        };

        let epoch = MeasEpochBlock::parse(&header, &data).expect("parse MeasEpoch");
        assert_eq!(epoch.measurements.len(), 2);
        assert_eq!(epoch.num_satellites(), 1);
        assert_eq!(epoch.num_measurements(), 2);

        let primary = &epoch.measurements[0];
        let expected_pr1 = (4_u64 * 4_294_967_296 + 3_000_000_000_u64) as f64 * 0.001;
        let l1_wavelength = SPEED_OF_LIGHT_M_S / 1_575.42e6;
        let expected_l1 = expected_pr1 / l1_wavelength + (-2.0 * 65_536.0 + 12_345.0) * 0.001;
        assert_eq!(primary.sat_id.key(), "G01");
        assert_eq!(primary.signal_type, SignalType::L1CA);
        assert_eq!(primary.rx_channel(), 7);
        assert_eq!(primary.antenna_id(), 0);
        assert_eq!(primary.signal_number(), 0);
        assert!((primary.pseudorange_m().unwrap() - expected_pr1).abs() < 1e-9);
        assert!((primary.carrier_phase_cycles().unwrap() - expected_l1).abs() < 1e-9);
        assert!((primary.doppler_hz_opt().unwrap() + 12.3456).abs() < 1e-12);
        let MeasEpochMeasurementRaw::Type1(raw1) = primary.raw() else {
            panic!("expected Type1 raw data");
        };
        assert_eq!(raw1.code_msb(), 4);
        assert_eq!(raw1.code_lsb, 3_000_000_000);
        assert_eq!(raw1.carrier_msb, -2);
        assert_eq!(raw1.carrier_lsb, 12_345);

        let secondary = &epoch.measurements[1];
        let expected_pr2 = expected_pr1 + (-65_536.0 + 1_000.0) * 0.001;
        let l2_frequency = 1_227.60e6;
        let l2_wavelength = SPEED_OF_LIGHT_M_S / l2_frequency;
        let expected_l2 = expected_pr2 / l2_wavelength + (65_536.0 + 2_000.0) * 0.001;
        let expected_d2 = -12.3456 * (l2_frequency / 1_575.42e6) + (65_536.0 + 2_000.0) * 0.0001;
        assert_eq!(secondary.sat_id.key(), "G01");
        assert_eq!(secondary.signal_type, SignalType::L2C);
        assert_eq!(secondary.rx_channel(), 7);
        assert!((secondary.pseudorange_m().unwrap() - expected_pr2).abs() < 1e-9);
        assert!((secondary.carrier_phase_cycles().unwrap() - expected_l2).abs() < 1e-9);
        assert!((secondary.doppler_hz_opt().unwrap() - expected_d2).abs() < 1e-12);
        let MeasEpochMeasurementRaw::Type2(raw2) = secondary.raw() else {
            panic!("expected Type2 raw data");
        };
        assert_eq!(raw2.code_offset_msb(), -1);
        assert_eq!(raw2.doppler_offset_msb(), 1);
        assert_eq!(raw2.code_offset_lsb, 1_000);
        assert_eq!(raw2.carrier_lsb, 2_000);
        assert_eq!(raw2.doppler_offset_lsb, 2_000);
    }

    #[test]
    fn meas_epoch_handles_dnu_and_glonass_frequency_number() {
        let raw1 = MeasEpochType1Raw {
            rx_channel: 1,
            signal_type: 8,
            svid: 38,
            misc: 0,
            code_lsb: 0,
            doppler: I32_DNU,
            carrier_lsb: 0,
            carrier_msb: -128,
            cn0: u8::MAX,
            lock_time: U16_DNU,
            obs_info: 1 << 3, // FreqNr field 1 means GLONASS frequency number -7.
            n2: 0,
        };
        assert_eq!(type1_pseudorange_m(&raw1), None);
        assert_eq!(glonass_frequency_number(8, raw1.obs_info), Some(-7));
        assert_eq!(
            carrier_frequency_hz(SignalType::G1CA, Some(-7)),
            Some(1_602.0e6 - 7.0 * 562_500.0)
        );
        assert_eq!(
            compute_carrier_phase_cycles(None, Some(1_575.42e6), -128, 0),
            None
        );

        let raw2 = MeasEpochType2Raw {
            signal_type: 3,
            lock_time: u8::MAX,
            cn0: u8::MAX,
            offsets_msb: 0x84, // CodeOffsetMSB=-4, DopplerOffsetMSB=-16.
            carrier_msb: -128,
            obs_info: 0,
            code_offset_lsb: 0,
            carrier_lsb: 0,
            doppler_offset_lsb: 0,
        };
        assert_eq!(raw2.code_offset_msb(), -4);
        assert_eq!(raw2.doppler_offset_msb(), -16);
        assert_eq!(type2_pseudorange_m(Some(20_000_000.0), &raw2), None);
        assert_eq!(
            type2_doppler_hz(Some(1.0), Some(1.0), Some(1.0), &raw2),
            None
        );
    }

    #[test]
    fn meas_epoch_matches_meas_extra_by_epoch_channel_antenna_and_signal() {
        let measurement = test_measurement(SignalType::L1CA, 160, 0, Some(0.0), 30, true);
        let epoch = MeasEpochBlock {
            tow_ms: 10_000,
            wnc: 2_400,
            n1: 1,
            sb1_length: 20,
            sb2_length: 12,
            common_flags: 0,
            cum_clk_jumps: 0,
            measurements: vec![measurement],
        };
        let channel = MeasExtraChannel {
            rx_channel: 1,
            signal_type: SignalType::L1CA,
            signal_type_raw: 0,
            signal_number: 0,
            mp_correction_raw: 0,
            smoothing_correction_raw: 0,
            code_var_raw: 1,
            carrier_var_raw: 1,
            lock_time_raw: 30,
            cum_loss_cont: 0,
            car_mp_correction_raw: None,
            info: 0,
            misc_raw: None,
        };
        let mut extra = MeasExtraBlock {
            tow_ms: 10_000,
            wnc: 2_400,
            n: 1,
            sb_length: 14,
            doppler_var_factor: 1.0,
            channels: vec![channel],
        };

        assert!(epoch
            .matching_extra_channel(&extra, &epoch.measurements[0])
            .is_some());

        extra.tow_ms += 1;
        assert!(epoch
            .matching_extra_channel(&extra, &epoch.measurements[0])
            .is_none());
    }

    #[test]
    fn test_meas_extra_scaling() {
        let channel = MeasExtraChannel {
            rx_channel: 3,
            signal_type: SignalType::L1CA,
            signal_type_raw: 0,
            signal_number: 0,
            mp_correction_raw: 1234,
            smoothing_correction_raw: -500,
            code_var_raw: 200,
            carrier_var_raw: 150,
            lock_time_raw: 45,
            cum_loss_cont: 2,
            car_mp_correction_raw: None,
            info: 1,
            misc_raw: None,
        };

        assert!((channel.mp_correction_m() - 1.234).abs() < 1e-6);
        assert!((channel.smoothing_correction_m() + 0.5).abs() < 1e-6);
        assert!((channel.code_var_m2() - 0.02).abs() < 1e-6);
        assert!((channel.code_var_m2_opt().unwrap() - 0.02).abs() < 1e-6);
        assert_eq!(channel.code_var_raw(), 200);
        assert!((channel.carrier_var_cycles2() - 0.00015).abs() < 1e-9);
        assert!((channel.carrier_var_cycles2_opt().unwrap() - 0.00015).abs() < 1e-9);
        assert_eq!(channel.carrier_var_raw(), 150);
        assert_eq!(channel.lock_time_seconds_opt(), Some(45.0));
        assert_eq!(channel.lock_time_raw(), 45);
        assert_eq!(channel.signal_type_raw(), 0);
        assert_eq!(channel.signal_number(), 0);
        assert_eq!(channel.antenna_id(), 0);
        assert_eq!(channel.car_mp_correction_raw(), None);
        assert_eq!(channel.misc_raw(), None);
    }

    #[test]
    fn test_meas_extra_channel_dnu_handling() {
        let channel = MeasExtraChannel {
            rx_channel: 3,
            signal_type: SignalType::L1CA,
            signal_type_raw: 0,
            signal_number: 0,
            mp_correction_raw: 0,
            smoothing_correction_raw: 0,
            code_var_raw: U16_DNU,
            carrier_var_raw: U16_DNU,
            lock_time_raw: U16_DNU,
            cum_loss_cont: 0,
            car_mp_correction_raw: None,
            info: 0,
            misc_raw: None,
        };

        assert_eq!(channel.code_var_raw(), U16_DNU);
        assert_eq!(channel.code_var_m2_opt(), None);
        assert_eq!(channel.code_var_m2(), 0.0);
        assert_eq!(channel.carrier_var_raw(), U16_DNU);
        assert_eq!(channel.carrier_var_cycles2_opt(), None);
        assert_eq!(channel.carrier_var_cycles2(), 0.0);
        assert_eq!(channel.lock_time_seconds_opt(), None);
        assert_eq!(channel.lock_time_seconds(), 0.0);
    }

    #[test]
    fn test_meas_extra_doppler_factor() {
        let block = MeasExtraBlock {
            tow_ms: 1000,
            wnc: 2000,
            n: 0,
            sb_length: 14,
            doppler_var_factor: 1.5,
            channels: Vec::new(),
        };

        assert_eq!(block.tow_seconds(), 1.0);
        assert_eq!(block.wnc(), 2000);
        assert!((block.doppler_var_factor() - 1.5).abs() < 1e-6);
        assert_eq!(block.num_channels(), 0);
    }

    #[test]
    fn test_iq_corr_parse() {
        let mut data = vec![0u8; 32];
        data[6..10].copy_from_slice(&5000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2100u16.to_le_bytes());
        data[12] = 1; // N
        data[13] = 8; // SBLength
        data[14] = 20; // CorrDuration 20ms
        data[15] = 0; // CumClkJumps
        data[16] = 2; // RxChannel
        data[17] = 0; // Type
        data[18] = 7; // SVID
        data[19] = 10; // CorrIQ_MSB
        data[20] = 5; // CorrI_LSB
        data[21] = 3; // CorrQ_LSB
        data[22..24].copy_from_slice(&1000u16.to_le_bytes()); // CarrierPhaseLSB

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::IQ_CORR,
            block_rev: 0,
            length: 32,
            tow_ms: 5000,
            wnc: 2100,
        };
        let block = IqCorrBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2100);
        assert_eq!(block.n, 1);
        assert_eq!(block.corr_duration, 20);
        assert_eq!(block.num_channels(), 1);
        assert_eq!(block.channels[0].rx_channel, 2);
        assert_eq!(block.channels[0].svid, 7);
        assert_eq!(block.channels[0].carrier_phase_lsb, 1000);
    }

    #[test]
    fn test_end_of_meas_accessors() {
        let end = EndOfMeasBlock {
            tow_ms: 2500,
            wnc: 123,
        };
        assert_eq!(end.tow_ms(), 2500);
        assert_eq!(end.wnc(), 123);
        assert_eq!(end.tow_seconds(), 2.5);
    }

    #[test]
    fn test_meas_extra_parse() {
        let mut data = vec![0u8; 18 + 14];
        data[12] = 1; // N
        data[13] = 14; // SBLength
        data[14..18].copy_from_slice(&1.25_f32.to_le_bytes());

        let offset = 18;
        data[offset] = 5; // RxChannel
        data[offset + 1] = 0; // Signal type (L1CA)
        data[offset + 2..offset + 4].copy_from_slice(&1000_i16.to_le_bytes());
        data[offset + 4..offset + 6].copy_from_slice(&(-200_i16).to_le_bytes());
        data[offset + 6..offset + 8].copy_from_slice(&500_u16.to_le_bytes());
        data[offset + 8..offset + 10].copy_from_slice(&250_u16.to_le_bytes());
        data[offset + 10..offset + 12].copy_from_slice(&60_u16.to_le_bytes());
        data[offset + 12] = 3;
        data[offset + 13] = 0xA5;

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::MEAS_EXTRA,
            block_rev: 0,
            length: (data.len() + 2) as u16,
            tow_ms: 123456,
            wnc: 321,
        };

        let parsed = MeasExtraBlock::parse(&header, &data).expect("parse");
        assert_eq!(parsed.tow_ms(), 123456);
        assert_eq!(parsed.wnc(), 321);
        assert_eq!(parsed.n, 1);
        assert_eq!(parsed.sb_length, 14);
        assert!((parsed.doppler_var_factor() - 1.25).abs() < 1e-6);
        assert_eq!(parsed.num_channels(), 1);

        let ch = &parsed.channels[0];
        assert_eq!(ch.rx_channel, 5);
        assert_eq!(ch.signal_type, SignalType::L1CA);
        assert_eq!(ch.signal_type_raw(), 0);
        assert_eq!(ch.signal_number(), 0);
        assert_eq!(ch.antenna_id(), 0);
        assert!((ch.mp_correction_m() - 1.0).abs() < 1e-6);
        assert!((ch.smoothing_correction_m() + 0.2).abs() < 1e-6);
        assert!((ch.code_var_m2() - 0.05).abs() < 1e-6);
        assert!((ch.carrier_var_cycles2() - 0.00025).abs() < 1e-9);
        assert_eq!(ch.lock_time_raw(), 60);
        assert_eq!(ch.cum_loss_cont, 3);
        assert_eq!(ch.car_mp_correction_raw(), None);
        assert_eq!(ch.info, 0xA5);
        assert_eq!(ch.misc_raw(), None);
    }

    #[test]
    fn test_meas_extra_parse_extended_type_and_misc() {
        let mut data = vec![0u8; 18 + 16];
        data[12] = 1; // N
        data[13] = 16; // SBLength (includes CarMPCorr, Info, Misc)
        data[14..18].copy_from_slice(&2.0_f32.to_le_bytes());

        let offset = 18;
        data[offset] = 7; // RxChannel
        data[offset + 1] = 0x5F; // antenna ID 2 (bits 5-7), SigIdxLo = 31
        data[offset + 2..offset + 4].copy_from_slice(&0_i16.to_le_bytes());
        data[offset + 4..offset + 6].copy_from_slice(&0_i16.to_le_bytes());
        data[offset + 6..offset + 8].copy_from_slice(&100_u16.to_le_bytes());
        data[offset + 8..offset + 10].copy_from_slice(&1024_u16.to_le_bytes());
        data[offset + 10..offset + 12].copy_from_slice(&11_u16.to_le_bytes());
        data[offset + 12] = 9; // CumLossCont
        data[offset + 13] = (-64_i8) as u8; // CarMPCorr
        data[offset + 14] = 0xB4; // Info
        data[offset + 15] = 0x33; // Misc: CN0HighRes=3, SigIdxHi=6 => signal number 38

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::MEAS_EXTRA,
            block_rev: 3,
            length: (data.len() + 2) as u16,
            tow_ms: 500,
            wnc: 2222,
        };

        let parsed = MeasExtraBlock::parse(&header, &data).expect("parse");
        assert_eq!(parsed.num_channels(), 1);

        let ch = &parsed.channels[0];
        assert_eq!(ch.rx_channel, 7);
        assert_eq!(ch.signal_type_raw(), 0x5F);
        assert_eq!(ch.antenna_id(), 2);
        assert_eq!(ch.signal_number(), 38);
        assert_eq!(ch.signal_type, SignalType::QZSSL1CB);
        assert_eq!(ch.cum_loss_cont, 9);
        assert_eq!(ch.info, 0xB4);
        assert_eq!(ch.car_mp_correction_raw(), Some(-64));
        assert_eq!(ch.misc_raw(), Some(0x33));
        assert!((ch.car_mp_correction_cycles().expect("carmp") + 0.125).abs() < 1e-9);
        assert!((ch.cn0_high_res_dbhz_offset().expect("cn0 hi-res") - 0.09375).abs() < 1e-9);
    }

    #[test]
    fn test_end_of_meas_parse() {
        let data = vec![0u8; 12];
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::END_OF_MEAS,
            block_rev: 0,
            length: (data.len() + 2) as u16,
            tow_ms: 1000,
            wnc: 45,
        };

        let parsed = EndOfMeasBlock::parse(&header, &data).expect("parse");
        assert_eq!(parsed.tow_ms(), 1000);
        assert_eq!(parsed.wnc(), 45);
    }
}
