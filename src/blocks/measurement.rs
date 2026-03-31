//! Measurement blocks (MeasEpoch_v2)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::{SatelliteId, SignalType};

use super::block_ids;
use super::SbfBlockParse;

// ============================================================================
// MeasEpoch Type1 Sub-block (raw)
// ============================================================================

/// Raw Type1 sub-block data from MeasEpoch
#[derive(Debug, Clone)]
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
    /// Raw CN0 value (use cn0_dbhz() for scaling per SBF spec)
    cn0_raw: u8,
    /// Raw Doppler value (multiply by 0.0001 for Hz)
    doppler_raw: i32,
    /// Raw lock time
    lock_time_raw: u16,
    /// Observation info flags
    pub obs_info: u8,
}

impl SatelliteMeasurement {
    /// Get CN0 in dB-Hz (scaled per SBF Reference Guide)
    pub fn cn0_dbhz(&self) -> f64 {
        let base = self.cn0_raw as f64 * 0.25;
        match self.signal_type {
            // Signal numbers 1 and 2 (GPS L1P, GPS L2P) have no +10 dB offset
            SignalType::L1PY | SignalType::L2P => base,
            _ => base + 10.0,
        }
    }

    /// Get raw CN0 value
    pub fn cn0_raw(&self) -> u8 {
        self.cn0_raw
    }

    /// Check if CN0 is valid (not 255)
    pub fn cn0_valid(&self) -> bool {
        self.cn0_raw != 255
    }

    /// Get Doppler in Hz (scaled)
    pub fn doppler_hz(&self) -> f64 {
        self.doppler_raw as f64 * 0.0001
    }

    /// Get raw Doppler value
    pub fn doppler_raw(&self) -> i32 {
        self.doppler_raw
    }

    /// Get lock time in seconds
    ///
    /// Lock time is encoded in seconds and clipped at 65534 seconds.
    pub fn lock_time_seconds(&self) -> f64 {
        self.lock_time_raw as f64
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

        if sb1_length == 0 {
            return Err(SbfError::ParseError("MeasEpoch SB1Length is zero".into()));
        }

        let sb1_length_usize = sb1_length as usize;
        let sb2_length_usize = sb2_length as usize;

        // Type1 sub-blocks start at offset 17 (Rev 0) or 18 (Rev 1+)
        let mut offset = 17;
        if header.block_rev >= 1 {
            offset += 1; // Reserved byte in Rev 1+
        }

        let mut measurements = Vec::new();

        // Helper to extract signal number from type field
        let signal_number = |type_field: u8, obs_info: u8| -> u8 {
            let sig_idx = type_field & 0x1F;
            if sig_idx == 31 {
                32 + ((obs_info >> 3) & 0x1F)
            } else {
                sig_idx
            }
        };

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

            let svid = data[offset + 2];
            let type_field = data[offset + 1];

            let doppler = if sb1_length_usize > 11 {
                i32::from_le_bytes([
                    data[offset + 8],
                    data[offset + 9],
                    data[offset + 10],
                    data[offset + 11],
                ])
            } else {
                0
            };

            let cn0_raw = if sb1_length_usize > 15 {
                data[offset + 15]
            } else {
                255
            };

            let lock_time = if sb1_length_usize > 17 {
                u16::from_le_bytes([data[offset + 16], data[offset + 17]])
            } else {
                0
            };

            let obs_info = if sb1_length_usize > 18 {
                data[offset + 18]
            } else {
                0
            };

            let n2 = if sb1_length_usize > 19 {
                data[offset + 19]
            } else {
                0
            };

            // Parse primary signal measurement
            if let Some(sat_id) = SatelliteId::from_svid(svid) {
                let sig_num = signal_number(type_field, obs_info);
                let signal_type = SignalType::from_signal_number(sig_num);

                measurements.push(SatelliteMeasurement {
                    sat_id: sat_id.clone(),
                    signal_type,
                    cn0_raw,
                    doppler_raw: doppler,
                    lock_time_raw: lock_time,
                    obs_info,
                });

                offset += sb1_length_usize;

                // Parse Type2 sub-blocks (additional signals for same satellite)
                if sb2_length_usize > 0 {
                    for _ in 0..n2 {
                        if offset + sb2_length_usize > data.len() {
                            return Err(SbfError::ParseError(
                                "MeasEpoch SB2 exceeds block length".into(),
                            ));
                        }

                        // Type-2 sub-block structure:
                        // 0: Type, 1: LockTime (short), 2: CN0
                        // 3: OffsetMSB, 4: CarrierMSB, 5: ObsInfo
                        // 6-7: CodeOffsetLSB, 8-9: CarrierLSB, 10-11: DopplerOffsetLSB

                        let type2_field = data[offset];
                        let cn0_raw_2 = if sb2_length_usize > 2 {
                            data[offset + 2]
                        } else {
                            255
                        };
                        let lock_time_2 = if sb2_length_usize > 1 {
                            data[offset + 1] as u16
                        } else {
                            0
                        };
                        let obs_info_2 = if sb2_length_usize > 5 {
                            data[offset + 5]
                        } else {
                            0
                        };

                        let sig_num_2 = signal_number(type2_field, obs_info_2);
                        let signal_type_2 = SignalType::from_signal_number(sig_num_2);

                        measurements.push(SatelliteMeasurement {
                            sat_id: sat_id.clone(),
                            signal_type: signal_type_2,
                            cn0_raw: cn0_raw_2,
                            doppler_raw: 0, // Type2 has offset, not absolute
                            lock_time_raw: lock_time_2,
                            obs_info: obs_info_2,
                        });

                        offset += sb2_length_usize;
                    }
                }
            } else {
                // Skip invalid SVID
                offset += sb1_length_usize;
                if sb2_length_usize > 0 {
                    let n2_skip = if sb1_length_usize > 19 {
                        data[offset - sb1_length_usize + 19]
                    } else {
                        0
                    };
                    let skip_bytes =
                        sb2_length_usize
                            .checked_mul(n2_skip as usize)
                            .ok_or_else(|| {
                                SbfError::ParseError("MeasEpoch SB2 length overflow".into())
                            })?;
                    if offset + skip_bytes > data.len() {
                        return Err(SbfError::ParseError(
                            "MeasEpoch SB2 exceeds block length".into(),
                        ));
                    }
                    offset += skip_bytes;
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
        self.code_var_raw as f64 * 0.0001
    }

    /// Carrier variance in cycles^2
    pub fn carrier_var_cycles2(&self) -> f64 {
        self.carrier_var_raw as f64 * 0.000001
    }

    /// Lock time in seconds
    pub fn lock_time_seconds(&self) -> f64 {
        self.lock_time_raw as f64
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

    #[test]
    fn test_satellite_measurement_cn0() {
        let meas = SatelliteMeasurement {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            signal_type: SignalType::L1CA,
            cn0_raw: 160, // 160 * 0.25 + 10 = 50 dB-Hz
            doppler_raw: 1000,
            lock_time_raw: 10,
            obs_info: 0,
        };

        assert_eq!(meas.cn0_dbhz(), 50.0);
        assert!(meas.cn0_valid());
    }

    #[test]
    fn test_satellite_measurement_cn0_gps_p_no_offset() {
        let meas = SatelliteMeasurement {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            signal_type: SignalType::L1PY, // GPS L1P (signal number 1)
            cn0_raw: 160,                  // 160 * 0.25 = 40 dB-Hz (no +10 dB)
            doppler_raw: 1000,
            lock_time_raw: 10,
            obs_info: 0,
        };

        assert_eq!(meas.cn0_dbhz(), 40.0);
    }

    #[test]
    fn test_satellite_measurement_invalid_cn0() {
        let meas = SatelliteMeasurement {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            signal_type: SignalType::L1CA,
            cn0_raw: 255,
            doppler_raw: 0,
            lock_time_raw: 0,
            obs_info: 0,
        };

        assert!(!meas.cn0_valid());
    }

    #[test]
    fn test_lock_time_encoding() {
        // Linear encoding (seconds)
        let meas = SatelliteMeasurement {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            signal_type: SignalType::L1CA,
            cn0_raw: 160,
            doppler_raw: 0,
            lock_time_raw: 30,
            obs_info: 0,
        };
        assert_eq!(meas.lock_time_seconds(), 30.0);

        // Larger values are still linear, just clipped by the receiver if too large
        let meas2 = SatelliteMeasurement {
            lock_time_raw: 96,
            ..meas
        };
        assert_eq!(meas2.lock_time_seconds(), 96.0);
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
        assert!((channel.carrier_var_cycles2() - 0.00015).abs() < 1e-9);
        assert_eq!(channel.lock_time_raw(), 45);
        assert_eq!(channel.signal_type_raw(), 0);
        assert_eq!(channel.signal_number(), 0);
        assert_eq!(channel.antenna_id(), 0);
        assert_eq!(channel.car_mp_correction_raw(), None);
        assert_eq!(channel.misc_raw(), None);
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
