//! Status and miscellaneous blocks

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;
use crate::types::SatelliteId;

use super::block_ids;
use super::dnu::{f32_or_none, f64_or_none, u16_or_none, I16_DNU, I8_DNU, U16_DNU};
use super::SbfBlockParse;

#[cfg(test)]
use super::dnu::{F32_DNU, F64_DNU};

// ============================================================================
// Constants
// ============================================================================

fn trim_trailing_nuls(bytes: &[u8]) -> &[u8] {
    let end = bytes
        .iter()
        .rposition(|&byte| byte != 0)
        .map(|idx| idx + 1)
        .unwrap_or(0);
    &bytes[..end]
}

fn format_ip_bytes(bytes: &[u8; 16]) -> String {
    if bytes[0..12].iter().all(|&b| b == 0) {
        format!("{}.{}.{}.{}", bytes[12], bytes[13], bytes[14], bytes[15])
    } else {
        bytes.chunks(2)
            .map(|c| format!("{:02x}{:02x}", c[0], c[1]))
            .collect::<Vec<_>>()
            .join(":")
    }
}

// ============================================================================
// ReceiverStatus Block
// ============================================================================

/// AGC (Automatic Gain Control) data for each frontend
#[derive(Debug, Clone)]
pub struct AgcData {
    /// Frontend identifier
    pub frontend_id: u8,
    /// AGC gain in dB
    pub gain_db: i8,
    /// Sample variance
    pub sample_var: u8,
    /// Blanking statistics
    pub blanking_stat: u8,
}

/// ReceiverStatus block
///
/// Receiver status and health information.
#[derive(Debug, Clone)]
pub struct ReceiverStatusBlock {
    tow_ms: u32,
    wnc: u16,
    /// CPU load percentage
    pub cpu_load: u8,
    /// External error code
    pub ext_error: u8,
    /// Receiver uptime in seconds
    pub uptime_s: u32,
    /// Receiver state bitfield
    pub rx_state: u32,
    /// Receiver error bitfield
    pub rx_error: u32,
    /// Command cyclic counter (Rev 1+)
    cmd_count: Option<u8>,
    /// Raw temperature byte with +100 deg C offset (Rev 1+)
    temperature_raw: Option<u8>,
    /// AGC data per frontend
    pub agc_data: Vec<AgcData>,
}

impl ReceiverStatusBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Check if receiver has any errors
    pub fn has_errors(&self) -> bool {
        self.rx_error != 0 || self.ext_error != 0
    }

    /// Command cyclic counter, when present.
    pub fn cmd_count(&self) -> Option<u8> {
        self.cmd_count
    }

    /// Raw temperature byte, when present.
    pub fn temperature_raw(&self) -> Option<u8> {
        self.temperature_raw
    }

    /// Receiver temperature in deg C, when present and valid.
    pub fn temperature_celsius(&self) -> Option<i16> {
        self.temperature_raw.and_then(|raw| {
            if raw == 0 {
                None
            } else {
                Some(raw as i16 - 100)
            }
        })
    }

    /// Get uptime as Duration-like struct (hours, minutes, seconds)
    pub fn uptime_hms(&self) -> (u32, u8, u8) {
        let hours = self.uptime_s / 3600;
        let minutes = ((self.uptime_s % 3600) / 60) as u8;
        let seconds = (self.uptime_s % 60) as u8;
        (hours, minutes, seconds)
    }
}

impl SbfBlockParse for ReceiverStatusBlock {
    const BLOCK_ID: u16 = block_ids::RECEIVER_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let min_len = if header.block_rev >= 1 { 30 } else { 26 };
        if data.len() < min_len {
            return Err(SbfError::ParseError("ReceiverStatus too short".into()));
        }

        // Offsets:
        // 12: CPULoad
        // 13: ExtError
        // 14-17: UpTime (u32)
        // 18-21: RxState (u32)
        // 22-25: RxError (u32)
        // 26: N (number of AGC sub-blocks)
        // 27: SBLength

        let cpu_load = data[12];
        let ext_error = data[13];
        let uptime_s = u32::from_le_bytes(data[14..18].try_into().unwrap());
        let rx_state = u32::from_le_bytes(data[18..22].try_into().unwrap());
        let rx_error = u32::from_le_bytes(data[22..26].try_into().unwrap());

        let mut cmd_count = None;
        let mut temperature_raw = None;
        let mut agc_data = Vec::new();

        // Rev 1+ layout (documented in mosaic-X5 FW v4.15.1):
        // 26: N, 27: SBLength, 28: CmdCount, 29: Temperature, 30+: AGCState sub-blocks
        if header.block_rev >= 1 {
            let n = data[26] as usize;
            let sb_length = data[27] as usize;
            cmd_count = Some(data[28]);
            temperature_raw = Some(data[29]);

            if n > 0 && sb_length < 4 {
                return Err(SbfError::ParseError(
                    "ReceiverStatus SBLength too small".into(),
                ));
            }

            if sb_length >= 4 {
                let mut offset = 30;
                for _ in 0..n {
                    if offset + sb_length > data.len() {
                        break;
                    }

                    agc_data.push(AgcData {
                        frontend_id: data[offset],
                        gain_db: data[offset + 1] as i8,
                        sample_var: data[offset + 2],
                        blanking_stat: data[offset + 3],
                    });

                    offset += sb_length;
                }
            }
        } else if data.len() >= 28 {
            // Legacy fallback for pre-Rev1 payloads that include N/SBLength
            // directly followed by AGC sub-blocks.
            let n = data[26] as usize;
            let sb_length = data[27] as usize;

            if n > 0 && sb_length < 4 {
                return Err(SbfError::ParseError(
                    "ReceiverStatus SBLength too small".into(),
                ));
            }

            if sb_length >= 4 {
                let mut offset = 28;
                for _ in 0..n {
                    if offset + sb_length > data.len() {
                        break;
                    }

                    agc_data.push(AgcData {
                        frontend_id: data[offset],
                        gain_db: data[offset + 1] as i8,
                        sample_var: data[offset + 2],
                        blanking_stat: data[offset + 3],
                    });

                    offset += sb_length;
                }
            }
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            cpu_load,
            ext_error,
            uptime_s,
            rx_state,
            rx_error,
            cmd_count,
            temperature_raw,
            agc_data,
        })
    }
}

// ============================================================================
// ChannelStatus Block
// ============================================================================

/// Channel state information for a tracking channel
#[derive(Debug, Clone)]
pub struct ChannelState {
    pub antenna: u8,
    pub tracking_status: u16,
    pub pvt_status: u16,
    pub pvt_info: u16,
}

/// Satellite tracking information
#[derive(Debug, Clone)]
pub struct ChannelSatInfo {
    /// Satellite ID
    pub sat_id: SatelliteId,
    /// Frequency number (for GLONASS, raw field value)
    pub freq_nr: u8,
    /// Azimuth in degrees (raw value, 0..359)
    azimuth_raw: u16,
    /// Rise/set indicator
    pub rise_set: u8,
    /// Elevation in degrees (raw)
    elevation_raw: i8,
    /// Health status
    pub health_status: u16,
    /// Channel states
    pub states: Vec<ChannelState>,
}

impl ChannelSatInfo {
    /// Get azimuth in degrees
    pub fn azimuth_deg(&self) -> f64 {
        self.azimuth_raw as f64
    }

    /// Get elevation in degrees
    pub fn elevation_deg(&self) -> f64 {
        self.elevation_raw as f64
    }

    /// Check if satellite is rising
    pub fn is_rising(&self) -> bool {
        self.rise_set == 1
    }

    /// Check if satellite is setting
    pub fn is_setting(&self) -> bool {
        self.rise_set == 0
    }

    /// Check if rise/set state is unknown
    pub fn is_rise_set_unknown(&self) -> bool {
        self.rise_set == 3
    }
}

/// ChannelStatus block (Block ID 4013)
///
/// Detailed tracking status per channel.
#[derive(Debug, Clone)]
pub struct ChannelStatusBlock {
    tow_ms: u32,
    wnc: u16,
    /// Satellite tracking info
    pub satellites: Vec<ChannelSatInfo>,
}

impl ChannelStatusBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get number of tracked satellites
    pub fn num_satellites(&self) -> usize {
        self.satellites.len()
    }
}

impl SbfBlockParse for ChannelStatusBlock {
    const BLOCK_ID: u16 = block_ids::CHANNEL_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 18 {
            return Err(SbfError::ParseError("ChannelStatus too short".into()));
        }

        // Offsets:
        // 12: N1 (number of satellites)
        // 13: SB1Length
        // 14: SB2Length
        // 15-17: Reserved

        let n1 = data[12] as usize;
        let sb1_length = data[13] as usize;
        let sb2_length = data[14] as usize;

        if sb1_length < 12 {
            return Err(SbfError::ParseError(
                "ChannelStatus SB1Length too small".into(),
            ));
        }

        let mut satellites = Vec::new();
        let mut offset = 18;

        for _ in 0..n1 {
            if offset + sb1_length > data.len() {
                break;
            }

            // Sub-block 1 structure:
            // 0: SVID
            // 1: FreqNr
            // 2-3: SVIDFull
            // 4-5: Azimuth_RiseSet (packed)
            // 6-7: HealthStatus
            // 8: Elevation
            // 9: N2 (number of channel states)
            // 10: RxChannel
            // 11: Reserved

            let svid = data[offset];
            let freq_nr = data[offset + 1];
            let svid_full = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let az_rise_set = u16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            let health_status = u16::from_le_bytes([data[offset + 6], data[offset + 7]]);
            let elevation_raw = data[offset + 8] as i8;
            let n2 = data[offset + 9] as usize;

            let azimuth_raw = az_rise_set & 0x01FF;
            let rise_set = ((az_rise_set >> 14) & 0x03) as u8;

            offset += sb1_length;

            // Parse channel states (sub-block 2).
            // Byte 1 in each sub-block is reserved and must be skipped.
            let mut states = Vec::new();
            for _ in 0..n2 {
                if offset + sb2_length > data.len() {
                    break;
                }

                if sb2_length >= 8 {
                    states.push(ChannelState {
                        antenna: data[offset],
                        tracking_status: u16::from_le_bytes([data[offset + 2], data[offset + 3]]),
                        pvt_status: u16::from_le_bytes([data[offset + 4], data[offset + 5]]),
                        pvt_info: u16::from_le_bytes([data[offset + 6], data[offset + 7]]),
                    });
                }

                offset += sb2_length;
            }

            let sat_id = if svid != 0 {
                SatelliteId::from_svid(svid)
            } else if svid_full <= u8::MAX as u16 {
                SatelliteId::from_svid(svid_full as u8)
            } else {
                None
            };

            if let Some(sat_id) = sat_id {
                satellites.push(ChannelSatInfo {
                    sat_id,
                    freq_nr,
                    azimuth_raw,
                    rise_set,
                    elevation_raw,
                    health_status,
                    states,
                });
            }
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            satellites,
        })
    }
}

// ============================================================================
// SatVisibility Block
// ============================================================================

/// Satellite visibility information
#[derive(Debug, Clone)]
pub struct SatVisibilityInfo {
    /// Satellite ID
    pub sat_id: SatelliteId,
    /// Frequency number (for GLONASS, raw field value)
    pub freq_nr: u8,
    /// Azimuth (raw, multiply by 0.01 for degrees)
    azimuth_raw: u16,
    /// Elevation (raw, multiply by 0.01 for degrees)
    elevation_raw: i16,
    /// Rise/set indicator (0=setting, 1=rising)
    pub rise_set: u8,
    /// Satellite info flags
    pub satellite_info: u8,
}

impl SatVisibilityInfo {
    /// Get azimuth in degrees
    pub fn azimuth_deg(&self) -> Option<f64> {
        if self.azimuth_raw == 65535 {
            None
        } else {
            Some(self.azimuth_raw as f64 * 0.01)
        }
    }

    /// Get raw azimuth
    pub fn azimuth_raw(&self) -> u16 {
        self.azimuth_raw
    }

    /// Get elevation in degrees
    pub fn elevation_deg(&self) -> Option<f64> {
        if self.elevation_raw == -32768 {
            None
        } else {
            Some(self.elevation_raw as f64 * 0.01)
        }
    }

    /// Get raw elevation
    pub fn elevation_raw(&self) -> i16 {
        self.elevation_raw
    }

    /// Check if satellite is rising
    pub fn is_rising(&self) -> bool {
        self.rise_set == 1
    }

    /// Check if rise/set state is unknown
    pub fn is_rise_set_unknown(&self) -> bool {
        self.rise_set == 255
    }

    /// Check if satellite is above horizon
    pub fn is_above_horizon(&self) -> bool {
        self.elevation_deg().map(|e| e > 0.0).unwrap_or(false)
    }
}

/// SatVisibility block (Block ID 4012)
///
/// Satellite visibility (azimuth, elevation) information.
#[derive(Debug, Clone)]
pub struct SatVisibilityBlock {
    tow_ms: u32,
    wnc: u16,
    /// Satellite visibility data
    pub satellites: Vec<SatVisibilityInfo>,
}

impl SatVisibilityBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Get number of visible satellites
    pub fn num_satellites(&self) -> usize {
        self.satellites.len()
    }

    /// Get satellites above a minimum elevation
    pub fn above_elevation(&self, min_elevation_deg: f64) -> Vec<&SatVisibilityInfo> {
        self.satellites
            .iter()
            .filter(|s| {
                s.elevation_deg()
                    .map(|e| e >= min_elevation_deg)
                    .unwrap_or(false)
            })
            .collect()
    }

    /// Get visibility info for a specific satellite
    pub fn get_satellite(&self, sat_id: &SatelliteId) -> Option<&SatVisibilityInfo> {
        self.satellites.iter().find(|s| &s.sat_id == sat_id)
    }
}

impl SbfBlockParse for SatVisibilityBlock {
    const BLOCK_ID: u16 = block_ids::SAT_VISIBILITY;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("SatVisibility too short".into()));
        }

        // Offsets:
        // 12: N (number of satellites)
        // 13: SBLength (sub-block length)

        let n = data[12] as usize;
        let sb_length = data[13] as usize;

        if sb_length < 8 {
            return Err(SbfError::ParseError(
                "SatVisibility SBLength too small".into(),
            ));
        }

        let mut satellites = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            // Sub-block structure:
            // 0: SVID
            // 1: FreqNr
            // 2-3: Azimuth (u16 * 0.01)
            // 4-5: Elevation (i16 * 0.01)
            // 6: RiseSet
            // 7: SatelliteInfo

            let svid = data[offset];
            let freq_nr = data[offset + 1];
            let azimuth_raw = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let elevation_raw = i16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            let rise_set = data[offset + 6];
            let satellite_info = data[offset + 7];

            if let Some(sat_id) = SatelliteId::from_svid(svid) {
                satellites.push(SatVisibilityInfo {
                    sat_id,
                    freq_nr,
                    azimuth_raw,
                    elevation_raw,
                    rise_set,
                    satellite_info,
                });
            }

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
// QualityInd Block
// ============================================================================

/// QualityInd block (Block ID 4082)
///
/// Quality indicator values (raw indicator list).
#[derive(Debug, Clone)]
pub struct QualityIndBlock {
    tow_ms: u32,
    wnc: u16,
    /// Raw quality indicators (u16 values)
    pub indicators: Vec<u16>,
}

impl QualityIndBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_indicators(&self) -> usize {
        self.indicators.len()
    }
}

impl SbfBlockParse for QualityIndBlock {
    const BLOCK_ID: u16 = block_ids::QUALITY_IND;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("QualityInd too short".into()));
        }

        // Offsets:
        // 12: N (number of indicators)
        // 13: Reserved
        // 14+: N * u16 indicators
        let n = data[12] as usize;
        if n > 40 {
            return Err(SbfError::ParseError(
                "QualityInd too many indicators".into(),
            ));
        }

        let required_len = 14 + (n * 2);
        if data.len() < required_len {
            return Err(SbfError::ParseError("QualityInd too short".into()));
        }

        let mut indicators = Vec::with_capacity(n);
        let mut offset = 14;
        for _ in 0..n {
            let value = u16::from_le_bytes([data[offset], data[offset + 1]]);
            indicators.push(value);
            offset += 2;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            indicators,
        })
    }
}

// ============================================================================
// InputLink Block
// ============================================================================

/// Input link statistics entry
#[derive(Debug, Clone)]
pub struct InputLinkStats {
    /// Connection descriptor
    pub connection_descriptor: u8,
    /// Link type
    pub link_type: u8,
    /// Age of last message (raw, seconds)
    age_last_message_raw: u16,
    /// Bytes received
    pub bytes_received: u32,
    /// Bytes accepted
    pub bytes_accepted: u32,
    /// Messages received
    pub messages_received: u32,
    /// Messages accepted
    pub messages_accepted: u32,
}

impl InputLinkStats {
    pub fn age_last_message_s(&self) -> Option<u16> {
        u16_or_none(self.age_last_message_raw)
    }

    pub fn age_last_message_raw(&self) -> u16 {
        self.age_last_message_raw
    }
}

/// InputLink block (Block ID 4090)
#[derive(Debug, Clone)]
pub struct InputLinkBlock {
    tow_ms: u32,
    wnc: u16,
    /// Input link statistics
    pub inputs: Vec<InputLinkStats>,
}

impl InputLinkBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_links(&self) -> usize {
        self.inputs.len()
    }
}

impl SbfBlockParse for InputLinkBlock {
    const BLOCK_ID: u16 = block_ids::INPUT_LINK;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 14 {
            return Err(SbfError::ParseError("InputLink too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;

        if sb_length < 20 {
            return Err(SbfError::ParseError("InputLink SBLength too small".into()));
        }

        let mut inputs = Vec::new();
        let mut offset = 14;

        for _ in 0..n {
            if offset + sb_length > data.len() {
                break;
            }

            let connection_descriptor = data[offset];
            let link_type = data[offset + 1];
            let age_last_message_raw = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let bytes_received =
                u32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
            let bytes_accepted =
                u32::from_le_bytes(data[offset + 8..offset + 12].try_into().unwrap());
            let messages_received =
                u32::from_le_bytes(data[offset + 12..offset + 16].try_into().unwrap());
            let messages_accepted =
                u32::from_le_bytes(data[offset + 16..offset + 20].try_into().unwrap());

            inputs.push(InputLinkStats {
                connection_descriptor,
                link_type,
                age_last_message_raw,
                bytes_received,
                bytes_accepted,
                messages_received,
                messages_accepted,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            inputs,
        })
    }
}

// ============================================================================
// OutputLink Block
// ============================================================================

/// Output type entry
#[derive(Debug, Clone)]
pub struct OutputType {
    /// Output type
    pub output_type: u8,
    /// Percentage of output bandwidth
    pub percentage: u8,
}

/// Output link statistics entry
#[derive(Debug, Clone)]
pub struct OutputLinkStats {
    /// Connection descriptor
    pub connection_descriptor: u8,
    /// Allowed rate (raw, kbyte/s)
    allowed_rate_raw: u16,
    /// Bytes produced
    pub bytes_produced: u32,
    /// Bytes sent
    pub bytes_sent: u32,
    /// Number of clients
    pub nr_clients: u8,
    /// Output types
    pub output_types: Vec<OutputType>,
}

impl OutputLinkStats {
    /// Allowed rate in kbyte/s (as provided by the block).
    pub fn allowed_rate_kbytes_per_s(&self) -> u16 {
        self.allowed_rate_raw
    }

    /// Allowed rate in bytes/s (decimal kilobytes).
    pub fn allowed_rate_bytes_per_s(&self) -> u32 {
        self.allowed_rate_raw as u32 * 1000
    }

    /// Legacy accessor kept for compatibility.
    ///
    /// The name is historical; it returns the raw `AllowedRate` value (kbyte/s),
    /// wrapped in `Some` because this field has no documented DNU sentinel.
    pub fn allowed_rate_bps(&self) -> Option<u16> {
        Some(self.allowed_rate_raw)
    }

    pub fn allowed_rate_raw(&self) -> u16 {
        self.allowed_rate_raw
    }
}

/// OutputLink block (Block ID 4091)
#[derive(Debug, Clone)]
pub struct OutputLinkBlock {
    tow_ms: u32,
    wnc: u16,
    /// Output link statistics
    pub outputs: Vec<OutputLinkStats>,
}

impl OutputLinkBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_links(&self) -> usize {
        self.outputs.len()
    }
}

impl SbfBlockParse for OutputLinkBlock {
    const BLOCK_ID: u16 = block_ids::OUTPUT_LINK;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 18 {
            return Err(SbfError::ParseError("OutputLink too short".into()));
        }

        let n1 = data[12] as usize;
        let sb1_length = data[13] as usize;
        let sb2_length = data[14] as usize;

        if sb1_length < 13 {
            return Err(SbfError::ParseError(
                "OutputLink SB1Length too small".into(),
            ));
        }

        let mut outputs = Vec::new();
        // Bytes 15..17 are reserved.
        let mut offset = 18;

        for _ in 0..n1 {
            if offset + sb1_length > data.len() {
                break;
            }

            let connection_descriptor = data[offset];
            let n2 = data[offset + 1] as usize;
            let allowed_rate_raw = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let bytes_produced =
                u32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
            let bytes_sent = u32::from_le_bytes(data[offset + 8..offset + 12].try_into().unwrap());
            let nr_clients = data[offset + 12];

            offset += sb1_length;

            let mut output_types = Vec::new();
            if sb2_length >= 2 {
                for _ in 0..n2 {
                    if offset + sb2_length > data.len() {
                        break;
                    }

                    output_types.push(OutputType {
                        output_type: data[offset],
                        percentage: data[offset + 1],
                    });

                    offset += sb2_length;
                }
            }

            outputs.push(OutputLinkStats {
                connection_descriptor,
                allowed_rate_raw,
                bytes_produced,
                bytes_sent,
                nr_clients,
                output_types,
            });
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            outputs,
        })
    }
}

// ============================================================================
// IPStatus Block
// ============================================================================

/// IPStatus block (Block ID 4058)
///
/// Ethernet IP address, gateway, MAC address, and netmask status.
#[derive(Debug, Clone)]
pub struct IpStatusBlock {
    tow_ms: u32,
    wnc: u16,
    /// MAC address (6 bytes)
    pub mac_address: [u8; 6],
    /// IP address (16 bytes, supports IPv4/IPv6)
    pub ip_address: [u8; 16],
    /// Gateway address (16 bytes)
    pub gateway: [u8; 16],
    /// Network mask prefix length (e.g. 24 for /24)
    pub netmask_prefix: u8,
}

impl IpStatusBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Format MAC address as colon-separated hex (e.g. "AA:BB:CC:DD:EE:FF")
    pub fn mac_address_string(&self) -> String {
        self.mac_address
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(":")
    }

    /// Format IPv4 address if first 12 bytes are zero (IPv4-mapped)
    pub fn ip_address_string(&self) -> String {
        if self.ip_address[0..12].iter().all(|&b| b == 0) {
            format!(
                "{}.{}.{}.{}",
                self.ip_address[12], self.ip_address[13], self.ip_address[14], self.ip_address[15]
            )
        } else {
            // IPv6 or other - format as hex groups
            self.ip_address
                .chunks(2)
                .map(|c| format!("{:02x}{:02x}", c[0], c[1]))
                .collect::<Vec<_>>()
                .join(":")
        }
    }

    /// Format gateway address (same logic as IP)
    pub fn gateway_string(&self) -> String {
        if self.gateway[0..12].iter().all(|&b| b == 0) {
            format!(
                "{}.{}.{}.{}",
                self.gateway[12], self.gateway[13], self.gateway[14], self.gateway[15]
            )
        } else {
            self.gateway
                .chunks(2)
                .map(|c| format!("{:02x}{:02x}", c[0], c[1]))
                .collect::<Vec<_>>()
                .join(":")
        }
    }
}

impl SbfBlockParse for IpStatusBlock {
    const BLOCK_ID: u16 = block_ids::IP_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 51; // 12 + 6 + 16 + 16 + 1
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("IPStatus too short".into()));
        }

        let mut mac_address = [0u8; 6];
        mac_address.copy_from_slice(&data[12..18]);
        let mut ip_address = [0u8; 16];
        ip_address.copy_from_slice(&data[18..34]);
        let mut gateway = [0u8; 16];
        gateway.copy_from_slice(&data[34..50]);
        let netmask_prefix = data[50];

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mac_address,
            ip_address,
            gateway,
            netmask_prefix,
        })
    }
}

// ============================================================================
// LBandTrackerStatus Block
// ============================================================================

/// One L-band tracker entry from `LBandTrackerStatus`.
#[derive(Debug, Clone)]
pub struct LBandTrackerData {
    /// Nominal beam frequency (Hz)
    pub frequency_hz: u32,
    /// Beam baudrate (baud)
    pub baudrate: u16,
    /// Beam service ID
    pub service_id: u16,
    freq_offset_hz_raw: f32,
    cn0_raw: u16,
    avg_power_raw: i16,
    agc_gain_db_raw: i8,
    /// Current operation mode
    pub mode: u8,
    /// Current tracker status (0=Idle, 1=Search, 2=FrameSearch, 3=Locked)
    pub status: u8,
    /// Satellite ID (Rev 2+)
    pub svid: Option<u8>,
    /// Lock time in seconds (Rev 1+)
    pub lock_time_s: Option<u16>,
    /// Source tracker module (Rev 3+)
    pub source: Option<u8>,
}

impl LBandTrackerData {
    /// Demodulator frequency offset in Hz.
    pub fn freq_offset_hz(&self) -> Option<f32> {
        f32_or_none(self.freq_offset_hz_raw)
    }

    pub fn freq_offset_hz_raw(&self) -> f32 {
        self.freq_offset_hz_raw
    }

    /// C/N0 in dB-Hz (raw * 0.01).
    pub fn cn0_dbhz(&self) -> Option<f32> {
        if self.cn0_raw == 0 {
            None
        } else {
            Some(self.cn0_raw as f32 * 0.01)
        }
    }

    pub fn cn0_raw(&self) -> u16 {
        self.cn0_raw
    }

    /// Average power in dB (raw * 0.01).
    pub fn avg_power_db(&self) -> Option<f32> {
        if self.avg_power_raw == I16_DNU {
            None
        } else {
            Some(self.avg_power_raw as f32 * 0.01)
        }
    }

    pub fn avg_power_raw(&self) -> i16 {
        self.avg_power_raw
    }

    /// L-band AGC gain in dB.
    pub fn agc_gain_db(&self) -> Option<i8> {
        if self.agc_gain_db_raw == I8_DNU {
            None
        } else {
            Some(self.agc_gain_db_raw)
        }
    }

    pub fn agc_gain_db_raw(&self) -> i8 {
        self.agc_gain_db_raw
    }
}

/// LBandTrackerStatus block (Block ID 4201)
///
/// General L-band tracker status and per-tracker demodulator information.
#[derive(Debug, Clone)]
pub struct LBandTrackerStatusBlock {
    tow_ms: u32,
    wnc: u16,
    /// Number of `TrackData` sub-blocks
    pub n: u8,
    /// Length of one `TrackData` sub-block
    pub sb_length: u8,
    /// Parsed tracker entries
    pub trackers: Vec<LBandTrackerData>,
}

impl LBandTrackerStatusBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn num_trackers(&self) -> usize {
        self.trackers.len()
    }
}

impl SbfBlockParse for LBandTrackerStatusBlock {
    const BLOCK_ID: u16 = block_ids::LBAND_TRACKER_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 14 || data.len() < data_len {
            return Err(SbfError::ParseError("LBandTrackerStatus too short".into()));
        }

        let n = data[12] as usize;
        let sb_length = data[13] as usize;
        let required_sb_len = 19
            + if header.block_rev >= 2 { 1 } else { 0 }
            + if header.block_rev >= 1 { 2 } else { 0 }
            + if header.block_rev >= 3 { 1 } else { 0 };
        if n > 0 && sb_length < required_sb_len {
            return Err(SbfError::ParseError(
                "LBandTrackerStatus SBLength too small".into(),
            ));
        }

        let mut trackers = Vec::with_capacity(n);
        let mut offset = 14usize;

        for _ in 0..n {
            if offset + sb_length > data_len {
                break;
            }

            let entry = &data[offset..offset + sb_length];
            let frequency_hz = u32::from_le_bytes(entry[0..4].try_into().unwrap());
            let baudrate = u16::from_le_bytes(entry[4..6].try_into().unwrap());
            let service_id = u16::from_le_bytes(entry[6..8].try_into().unwrap());
            let freq_offset_hz_raw = f32::from_le_bytes(entry[8..12].try_into().unwrap());
            let cn0_raw = u16::from_le_bytes(entry[12..14].try_into().unwrap());
            let avg_power_raw = i16::from_le_bytes(entry[14..16].try_into().unwrap());
            let agc_gain_db_raw = entry[16] as i8;
            let mode = entry[17];
            let status = entry[18];

            let mut cursor = 19usize;
            let svid = if header.block_rev >= 2 {
                let value = entry[cursor];
                cursor += 1;
                Some(value)
            } else {
                None
            };
            let lock_time_s = if header.block_rev >= 1 {
                let value = u16::from_le_bytes(entry[cursor..cursor + 2].try_into().unwrap());
                cursor += 2;
                Some(value)
            } else {
                None
            };
            let source = if header.block_rev >= 3 {
                Some(entry[cursor])
            } else {
                None
            };

            trackers.push(LBandTrackerData {
                frequency_hz,
                baudrate,
                service_id,
                freq_offset_hz_raw,
                cn0_raw,
                avg_power_raw,
                agc_gain_db_raw,
                mode,
                status,
                svid,
                lock_time_s,
                source,
            });

            offset += sb_length;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n: n as u8,
            sb_length: sb_length as u8,
            trackers,
        })
    }
}

// ============================================================================
// ReceiverSetup Block
// ============================================================================

fn field_text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(trim_trailing_nuls(bytes)).into_owned()
}

/// ReceiverSetup block (Block ID 5902)
///
/// Receiver installation metadata and RINEX header information.
#[derive(Debug, Clone)]
pub struct ReceiverSetupBlock {
    tow_ms: u32,
    wnc: u16,
    marker_name: Vec<u8>,
    marker_number: Vec<u8>,
    observer: Vec<u8>,
    agency: Vec<u8>,
    rx_serial_number: Vec<u8>,
    rx_name: Vec<u8>,
    rx_version: Vec<u8>,
    ant_serial_nbr: Vec<u8>,
    ant_type: Vec<u8>,
    delta_h_m: f32,
    delta_e_m: f32,
    delta_n_m: f32,
    marker_type: Option<Vec<u8>>,
    gnss_fw_version: Option<Vec<u8>>,
    product_name: Option<Vec<u8>>,
    latitude_rad: Option<f64>,
    longitude_rad: Option<f64>,
    height_m: Option<f32>,
    station_code: Option<Vec<u8>>,
    monument_idx: Option<u8>,
    receiver_idx: Option<u8>,
    country_code: Option<Vec<u8>>,
}

impl ReceiverSetupBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn marker_name(&self) -> &[u8] {
        &self.marker_name
    }
    pub fn marker_name_lossy(&self) -> String {
        field_text(&self.marker_name)
    }

    pub fn marker_number(&self) -> &[u8] {
        &self.marker_number
    }
    pub fn marker_number_lossy(&self) -> String {
        field_text(&self.marker_number)
    }

    pub fn observer(&self) -> &[u8] {
        &self.observer
    }
    pub fn observer_lossy(&self) -> String {
        field_text(&self.observer)
    }

    pub fn agency(&self) -> &[u8] {
        &self.agency
    }
    pub fn agency_lossy(&self) -> String {
        field_text(&self.agency)
    }

    pub fn rx_serial_number(&self) -> &[u8] {
        &self.rx_serial_number
    }
    pub fn rx_serial_number_lossy(&self) -> String {
        field_text(&self.rx_serial_number)
    }

    pub fn rx_name(&self) -> &[u8] {
        &self.rx_name
    }
    pub fn rx_name_lossy(&self) -> String {
        field_text(&self.rx_name)
    }

    pub fn rx_version(&self) -> &[u8] {
        &self.rx_version
    }
    pub fn rx_version_lossy(&self) -> String {
        field_text(&self.rx_version)
    }

    pub fn ant_serial_number(&self) -> &[u8] {
        &self.ant_serial_nbr
    }
    pub fn ant_serial_number_lossy(&self) -> String {
        field_text(&self.ant_serial_nbr)
    }

    pub fn ant_type(&self) -> &[u8] {
        &self.ant_type
    }
    pub fn ant_type_lossy(&self) -> String {
        field_text(&self.ant_type)
    }

    pub fn delta_h_m(&self) -> f32 {
        self.delta_h_m
    }
    pub fn delta_e_m(&self) -> f32 {
        self.delta_e_m
    }
    pub fn delta_n_m(&self) -> f32 {
        self.delta_n_m
    }

    pub fn marker_type_lossy(&self) -> Option<String> {
        self.marker_type.as_deref().map(field_text)
    }
    pub fn gnss_fw_version_lossy(&self) -> Option<String> {
        self.gnss_fw_version.as_deref().map(field_text)
    }
    pub fn product_name_lossy(&self) -> Option<String> {
        self.product_name.as_deref().map(field_text)
    }

    pub fn latitude_rad(&self) -> Option<f64> {
        self.latitude_rad.and_then(f64_or_none)
    }
    pub fn longitude_rad(&self) -> Option<f64> {
        self.longitude_rad.and_then(f64_or_none)
    }
    pub fn latitude_deg(&self) -> Option<f64> {
        self.latitude_rad().map(f64::to_degrees)
    }
    pub fn longitude_deg(&self) -> Option<f64> {
        self.longitude_rad().map(f64::to_degrees)
    }
    pub fn height_m(&self) -> Option<f32> {
        self.height_m.and_then(f32_or_none)
    }

    pub fn station_code_lossy(&self) -> Option<String> {
        self.station_code.as_deref().map(field_text)
    }
    pub fn monument_idx(&self) -> Option<u8> {
        self.monument_idx
    }
    pub fn receiver_idx(&self) -> Option<u8> {
        self.receiver_idx
    }
    pub fn country_code_lossy(&self) -> Option<String> {
        self.country_code.as_deref().map(field_text)
    }
}

impl SbfBlockParse for ReceiverSetupBlock {
    const BLOCK_ID: u16 = block_ids::RECEIVER_SETUP;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const REV0_LEN: usize = 266;
        const REV1_LEN: usize = 286;
        const REV2_LEN: usize = 326;
        const REV3_LEN: usize = 386;
        const REV4_LEN: usize = 422;

        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data.len() < data_len {
            return Err(SbfError::ParseError("ReceiverSetup too short".into()));
        }

        let required_len = match header.block_rev {
            0 => REV0_LEN,
            1 => REV1_LEN,
            2 => REV2_LEN,
            3 => REV3_LEN,
            _ => REV4_LEN,
        };
        if data_len < required_len {
            return Err(SbfError::ParseError("ReceiverSetup too short".into()));
        }

        let marker_name = data[14..74].to_vec();
        let marker_number = data[74..94].to_vec();
        let observer = data[94..114].to_vec();
        let agency = data[114..154].to_vec();
        let rx_serial_number = data[154..174].to_vec();
        let rx_name = data[174..194].to_vec();
        let rx_version = data[194..214].to_vec();
        let ant_serial_nbr = data[214..234].to_vec();
        let ant_type = data[234..254].to_vec();
        let delta_h_m = f32::from_le_bytes(data[254..258].try_into().unwrap());
        let delta_e_m = f32::from_le_bytes(data[258..262].try_into().unwrap());
        let delta_n_m = f32::from_le_bytes(data[262..266].try_into().unwrap());

        let marker_type = if header.block_rev >= 1 {
            Some(data[266..286].to_vec())
        } else {
            None
        };
        let gnss_fw_version = if header.block_rev >= 2 {
            Some(data[286..326].to_vec())
        } else {
            None
        };
        let (product_name, latitude_rad, longitude_rad, height_m) = if header.block_rev >= 3 {
            (
                Some(data[326..366].to_vec()),
                Some(f64::from_le_bytes(data[366..374].try_into().unwrap())),
                Some(f64::from_le_bytes(data[374..382].try_into().unwrap())),
                Some(f32::from_le_bytes(data[382..386].try_into().unwrap())),
            )
        } else {
            (None, None, None, None)
        };
        let (station_code, monument_idx, receiver_idx, country_code) = if header.block_rev >= 4 {
            (
                Some(data[386..396].to_vec()),
                Some(data[396]),
                Some(data[397]),
                Some(data[398..401].to_vec()),
            )
        } else {
            (None, None, None, None)
        };

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            marker_name,
            marker_number,
            observer,
            agency,
            rx_serial_number,
            rx_name,
            rx_version,
            ant_serial_nbr,
            ant_type,
            delta_h_m,
            delta_e_m,
            delta_n_m,
            marker_type,
            gnss_fw_version,
            product_name,
            latitude_rad,
            longitude_rad,
            height_m,
            station_code,
            monument_idx,
            receiver_idx,
            country_code,
        })
    }
}

// ============================================================================
// BBSamples Block
// ============================================================================

/// One complex baseband sample from `BBSamples`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BBSample {
    raw: u16,
}

impl BBSample {
    pub fn raw(&self) -> u16 {
        self.raw
    }

    /// In-phase component (I), stored in the upper byte.
    pub fn i(&self) -> i8 {
        (self.raw >> 8) as u8 as i8
    }

    /// Quadrature component (Q), stored in the lower byte.
    pub fn q(&self) -> i8 {
        self.raw as u8 as i8
    }
}

/// BBSamples block (Block ID 4040)
///
/// Complex baseband samples for RF monitoring and spectral analysis.
#[derive(Debug, Clone)]
pub struct BBSamplesBlock {
    tow_ms: u32,
    wnc: u16,
    n: u16,
    info: u8,
    sample_freq_hz: u32,
    lo_freq_hz: u32,
    samples: Vec<BBSample>,
    tow_delta_s: f32,
}

impl BBSamplesBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn num_samples(&self) -> u16 {
        self.n
    }
    pub fn info_raw(&self) -> u8 {
        self.info
    }
    pub fn antenna_id(&self) -> u8 {
        self.info & 0x07
    }
    pub fn sample_freq_hz(&self) -> u32 {
        self.sample_freq_hz
    }
    pub fn lo_freq_hz(&self) -> u32 {
        self.lo_freq_hz
    }
    pub fn samples(&self) -> &[BBSample] {
        &self.samples
    }
    pub fn sample_iq(&self, index: usize) -> Option<(i8, i8)> {
        self.samples
            .get(index)
            .map(|sample| (sample.i(), sample.q()))
    }
    pub fn tow_delta_seconds(&self) -> Option<f32> {
        f32_or_none(self.tow_delta_s)
    }
    pub fn first_sample_time_seconds(&self) -> Option<f64> {
        self.tow_delta_seconds()
            .map(|tow_delta_s| self.tow_seconds() + tow_delta_s as f64)
    }
}

impl SbfBlockParse for BBSamplesBlock {
    const BLOCK_ID: u16 = block_ids::BB_SAMPLES;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 30 || data.len() < data_len {
            return Err(SbfError::ParseError("BBSamples too short".into()));
        }

        let n = u16::from_le_bytes(data[12..14].try_into().unwrap()) as usize;
        let info = data[14];
        let sample_freq_hz = u32::from_le_bytes(data[18..22].try_into().unwrap());
        let lo_freq_hz = u32::from_le_bytes(data[22..26].try_into().unwrap());

        let samples_len = n
            .checked_mul(2)
            .ok_or_else(|| SbfError::ParseError("BBSamples sample count overflow".into()))?;
        let samples_end = 26usize
            .checked_add(samples_len)
            .ok_or_else(|| SbfError::ParseError("BBSamples sample data overflow".into()))?;
        let tow_delta_end = samples_end
            .checked_add(4)
            .ok_or_else(|| SbfError::ParseError("BBSamples TOWDelta overflow".into()))?;
        if tow_delta_end > data_len {
            return Err(SbfError::ParseError(
                "BBSamples sample data exceeds block length".into(),
            ));
        }

        let mut samples = Vec::with_capacity(n);
        for chunk in data[26..samples_end].chunks_exact(2) {
            samples.push(BBSample {
                raw: u16::from_le_bytes(chunk.try_into().unwrap()),
            });
        }
        let tow_delta_s = f32::from_le_bytes(data[samples_end..tow_delta_end].try_into().unwrap());

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n: n as u16,
            info,
            sample_freq_hz,
            lo_freq_hz,
            samples,
            tow_delta_s,
        })
    }
}

// ============================================================================
// ASCIIIn Block
// ============================================================================

/// ASCIIIn block (Block ID 4075)
///
/// Raw ASCII line received on a configured receiver input port.
#[derive(Debug, Clone)]
pub struct ASCIIInBlock {
    tow_ms: u32,
    wnc: u16,
    cd: u8,
    string_len: u16,
    sensor_model: Vec<u8>,
    sensor_type: Vec<u8>,
    ascii_string: Vec<u8>,
}

impl ASCIIInBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    pub fn connection_descriptor(&self) -> u8 {
        self.cd
    }
    pub fn string_len(&self) -> u16 {
        self.string_len
    }
    pub fn sensor_model(&self) -> &[u8] {
        &self.sensor_model
    }
    pub fn sensor_model_lossy(&self) -> String {
        field_text(&self.sensor_model)
    }
    pub fn sensor_type(&self) -> &[u8] {
        &self.sensor_type
    }
    pub fn sensor_type_lossy(&self) -> String {
        field_text(&self.sensor_type)
    }
    pub fn ascii_string(&self) -> &[u8] {
        &self.ascii_string
    }
    pub fn ascii_text_lossy(&self) -> String {
        String::from_utf8_lossy(&self.ascii_string).into_owned()
    }
}

impl SbfBlockParse for ASCIIInBlock {
    const BLOCK_ID: u16 = block_ids::ASCII_IN;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 78 || data.len() < data_len {
            return Err(SbfError::ParseError("ASCIIIn too short".into()));
        }

        let cd = data[12];
        let string_len = u16::from_le_bytes(data[16..18].try_into().unwrap()) as usize;
        let sensor_model = data[18..38].to_vec();
        let sensor_type = data[38..58].to_vec();
        let string_start = 78usize;
        let string_end = string_start
            .checked_add(string_len)
            .ok_or_else(|| SbfError::ParseError("ASCIIIn string length overflow".into()))?;
        if string_end > data_len {
            return Err(SbfError::ParseError("ASCIIIn string exceeds block".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            cd,
            string_len: string_len as u16,
            sensor_model,
            sensor_type,
            ascii_string: data[string_start..string_end].to_vec(),
        })
    }
}

// ============================================================================
// Commands Block
// ============================================================================

/// Commands block (Block ID 4015)
///
/// Logs command data entered by the user.
#[derive(Debug, Clone)]
pub struct CommandsBlock {
    tow_ms: u32,
    wnc: u16,
    cmd_data: Vec<u8>,
}

impl CommandsBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn cmd_data(&self) -> &[u8] {
        &self.cmd_data
    }

    /// Best-effort UTF-8 text view of command data.
    pub fn cmd_text_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.cmd_data)).into_owned()
    }
}

impl SbfBlockParse for CommandsBlock {
    const BLOCK_ID: u16 = block_ids::COMMANDS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 14 || data.len() < data_len {
            return Err(SbfError::ParseError("Commands too short".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            cmd_data: data[14..data_len].to_vec(),
        })
    }
}

// ============================================================================
// NTRIP Client / Server status (4053 / 4122)
// ============================================================================

/// One NTRIP connection slot (`NTRIPClientConnection` in SBF; server block uses the same layout).
#[derive(Debug, Clone)]
pub struct NtripConnectionSlot {
    pub cd_index: u8,
    pub status: u8,
    pub error_code: u8,
}

fn parse_ntrip_connection_status(data: &[u8]) -> SbfResult<(u8, u8, Vec<NtripConnectionSlot>)> {
    const MIN_HEADER: usize = 14;
    const MIN_SB_LENGTH: usize = 4;
    if data.len() < MIN_HEADER {
        return Err(SbfError::ParseError("NTRIP status too short".into()));
    }
    let n = data[12];
    let sb_length = data[13];
    let sb_length_usize = sb_length as usize;
    if sb_length_usize < MIN_SB_LENGTH {
        return Err(SbfError::ParseError(
            "NTRIP status SBLength too small".into(),
        ));
    }

    let required_len = MIN_HEADER + n as usize * sb_length_usize;
    if required_len > data.len() {
        return Err(SbfError::ParseError(
            "NTRIP status sub-blocks exceed block length".into(),
        ));
    }

    let mut connections = Vec::with_capacity(n as usize);
    let mut off = MIN_HEADER;
    for _ in 0..n as usize {
        connections.push(NtripConnectionSlot {
            cd_index: data[off],
            status: data[off + 1],
            error_code: data[off + 2],
        });
        off += sb_length_usize;
    }
    Ok((n, sb_length, connections))
}

/// NTRIP client connection status (Block ID 4053).
#[derive(Debug, Clone)]
pub struct NtripClientStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub connections: Vec<NtripConnectionSlot>,
}

impl NtripClientStatusBlock {
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

impl SbfBlockParse for NtripClientStatusBlock {
    const BLOCK_ID: u16 = block_ids::NTRIP_CLIENT_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let (n, sb_length, connections) = parse_ntrip_connection_status(data)?;
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            connections,
        })
    }
}

/// NTRIP server connection status (Block ID 4122).
///
/// Uses the same binary layout as the client status block (common Septentrio pairing). If a
/// firmware revision differs, compare with the SBF reference for your receiver.
#[derive(Debug, Clone)]
pub struct NtripServerStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub connections: Vec<NtripConnectionSlot>,
}

impl NtripServerStatusBlock {
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

impl SbfBlockParse for NtripServerStatusBlock {
    const BLOCK_ID: u16 = block_ids::NTRIP_SERVER_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let (n, sb_length, connections) = parse_ntrip_connection_status(data)?;
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            connections,
        })
    }
}

// ============================================================================
// RFStatus (4092)
// ============================================================================

/// One RF mitigation band entry (`RFBand` in SBF).
#[derive(Debug, Clone)]
pub struct RfBandEntry {
    pub frequency_hz: u32,
    pub bandwidth: u16,
    pub info: u8,
}

/// RF interference mitigation status (Block ID 4092).
#[derive(Debug, Clone)]
pub struct RfStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub reserved: u32,
    pub bands: Vec<RfBandEntry>,
}

impl RfStatusBlock {
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

impl SbfBlockParse for RfStatusBlock {
    const BLOCK_ID: u16 = block_ids::RF_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_HEADER: usize = 18;
        const MIN_SB_LENGTH: usize = 8;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("RFStatus too short".into()));
        }
        let n = data[12];
        let sb_length = data[13];
        let sb_length_usize = sb_length as usize;
        if sb_length_usize < MIN_SB_LENGTH {
            return Err(SbfError::ParseError("RFStatus SBLength too small".into()));
        }

        let required_len = MIN_HEADER + n as usize * sb_length_usize;
        if required_len > data.len() {
            return Err(SbfError::ParseError(
                "RFStatus sub-blocks exceed block length".into(),
            ));
        }

        let reserved = u32::from_le_bytes(data[14..18].try_into().unwrap());
        let mut bands = Vec::with_capacity(n as usize);
        let mut off = MIN_HEADER;
        for _ in 0..n as usize {
            bands.push(RfBandEntry {
                frequency_hz: u32::from_le_bytes(data[off..off + 4].try_into().unwrap()),
                bandwidth: u16::from_le_bytes(data[off + 4..off + 6].try_into().unwrap()),
                info: data[off + 6],
            });
            off += sb_length_usize;
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            reserved,
            bands,
        })
    }
}

// ============================================================================
// Comment Block
// ============================================================================

/// Comment block (Block ID 5936)
///
/// User comment string entered with `setObserverComment`.
#[derive(Debug, Clone)]
pub struct CommentBlock {
    tow_ms: u32,
    wnc: u16,
    comment_len: u16,
    comment_data: Vec<u8>,
}

impl CommentBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn comment_len(&self) -> u16 {
        self.comment_len
    }
    pub fn comment_data(&self) -> &[u8] {
        &self.comment_data
    }

    /// Best-effort UTF-8 text view of the comment bytes.
    pub fn comment_text_lossy(&self) -> String {
        String::from_utf8_lossy(&self.comment_data).into_owned()
    }
}

impl SbfBlockParse for CommentBlock {
    const BLOCK_ID: u16 = block_ids::COMMENT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 14 || data.len() < data_len {
            return Err(SbfError::ParseError("Comment too short".into()));
        }

        let comment_len = u16::from_le_bytes([data[12], data[13]]) as usize;
        let comment_end = 14 + comment_len;
        if comment_end > data_len {
            return Err(SbfError::ParseError("Comment length exceeds block".into()));
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            comment_len: comment_len as u16,
            comment_data: data[14..comment_end].to_vec(),
        })
    }
}

// ============================================================================
// RTCMDatum / LBandBeams / DynDNSStatus / DiskStatus / P2PPStatus
// ============================================================================

/// RTCM datum information from the correction provider (4049).
#[derive(Debug, Clone)]
pub struct RtcmDatumBlock {
    tow_ms: u32,
    wnc: u16,
    source_crs: [u8; 32],
    target_crs: [u8; 32],
    pub datum: u8,
    pub height_type: u8,
    pub quality_ind: u8,
}

impl RtcmDatumBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn source_crs_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.source_crs)).into_owned()
    }
    pub fn target_crs_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.target_crs)).into_owned()
    }
}

impl SbfBlockParse for RtcmDatumBlock {
    const BLOCK_ID: u16 = block_ids::RTCM_DATUM;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN: usize = 79;
        if data.len() < MIN {
            return Err(SbfError::ParseError("RTCMDatum too short".into()));
        }
        let mut source_crs = [0u8; 32];
        source_crs.copy_from_slice(&data[12..44]);
        let mut target_crs = [0u8; 32];
        target_crs.copy_from_slice(&data[44..76]);
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            source_crs,
            target_crs,
            datum: data[76],
            height_type: data[77],
            quality_ind: data[78],
        })
    }
}

/// One `BeamInfo` entry from `LBandBeams`.
#[derive(Debug, Clone)]
pub struct LBandBeamInfo {
    pub svid: u8,
    sat_name: [u8; 9],
    sat_longitude_raw: i16,
    pub beam_freq_hz: u32,
}

impl LBandBeamInfo {
    pub fn sat_name_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.sat_name)).into_owned()
    }
    pub fn sat_longitude_deg(&self) -> Option<f64> {
        if self.sat_longitude_raw == I16_DNU {
            None
        } else {
            Some(self.sat_longitude_raw as f64 * 0.01)
        }
    }
}

/// L-band beam list (4204).
#[derive(Debug, Clone)]
pub struct LBandBeamsBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub beams: Vec<LBandBeamInfo>,
}

impl LBandBeamsBlock {
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

impl SbfBlockParse for LBandBeamsBlock {
    const BLOCK_ID: u16 = block_ids::LBAND_BEAMS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_HEADER: usize = 14;
        const MIN_SB: usize = 16;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("LBandBeams too short".into()));
        }
        let n = data[12];
        let sb_length = data[13];
        let sb_length_usize = sb_length as usize;
        if sb_length_usize < MIN_SB {
            return Err(SbfError::ParseError("LBandBeams SBLength too small".into()));
        }
        let required_len = MIN_HEADER + n as usize * sb_length_usize;
        if required_len > data.len() {
            return Err(SbfError::ParseError(
                "LBandBeams sub-blocks exceed block length".into(),
            ));
        }
        let mut beams = Vec::with_capacity(n as usize);
        let mut off = MIN_HEADER;
        for _ in 0..n as usize {
            let mut sat_name = [0u8; 9];
            sat_name.copy_from_slice(&data[off + 1..off + 10]);
            beams.push(LBandBeamInfo {
                svid: data[off],
                sat_name,
                sat_longitude_raw: i16::from_le_bytes(data[off + 10..off + 12].try_into().unwrap()),
                beam_freq_hz: u32::from_le_bytes(data[off + 12..off + 16].try_into().unwrap()),
            });
            off += sb_length_usize;
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            beams,
        })
    }
}

/// DynDNS status (4105).
#[derive(Debug, Clone)]
pub struct DynDnsStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub status: u8,
    pub error_code: u8,
    pub ip_address: [u8; 16],
    pub ipv6_address: Option<[u8; 16]>,
}

impl DynDnsStatusBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn ip_address_string(&self) -> String {
        format_ip_bytes(&self.ip_address)
    }
    pub fn ipv6_address_string(&self) -> Option<String> {
        self.ipv6_address.as_ref().map(format_ip_bytes)
    }
}

impl SbfBlockParse for DynDnsStatusBlock {
    const BLOCK_ID: u16 = block_ids::DYN_DNS_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_V0: usize = 14;
        const MIN_V1: usize = 30;
        const MIN_V2: usize = 46;
        if data.len() < MIN_V0 {
            return Err(SbfError::ParseError("DynDNSStatus too short".into()));
        }
        let mut ip_address = [0u8; 16];
        if data.len() >= MIN_V1 {
            ip_address.copy_from_slice(&data[14..30]);
        }
        let ipv6_address = if header.block_rev >= 2 && data.len() >= MIN_V2 {
            let mut addr = [0u8; 16];
            addr.copy_from_slice(&data[30..46]);
            Some(addr)
        } else {
            None
        };
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            status: data[12],
            error_code: data[13],
            ip_address,
            ipv6_address,
        })
    }
}

/// One `DiskData` entry from `DiskStatus`.
#[derive(Debug, Clone)]
pub struct DiskData {
    pub disk_id: u8,
    pub status: u8,
    pub disk_usage_msb: u16,
    pub disk_usage_lsb: u32,
    pub disk_size_mb: u32,
    pub create_delete_count: u8,
    pub error: Option<u8>,
}

impl DiskData {
    pub fn disk_usage_bytes(&self) -> Option<u64> {
        if self.disk_usage_msb == U16_DNU && self.disk_usage_lsb == u32::MAX {
            None
        } else {
            Some(((self.disk_usage_msb as u64) << 32) | self.disk_usage_lsb as u64)
        }
    }
}

/// Disk usage and status (4059).
#[derive(Debug, Clone)]
pub struct DiskStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub reserved: [u8; 4],
    pub disks: Vec<DiskData>,
}

impl DiskStatusBlock {
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

impl SbfBlockParse for DiskStatusBlock {
    const BLOCK_ID: u16 = block_ids::DISK_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_HEADER: usize = 18;
        const MIN_SB: usize = 13;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("DiskStatus too short".into()));
        }
        let n = data[12];
        let sb_length = data[13];
        let sb_length_usize = sb_length as usize;
        if sb_length_usize < MIN_SB {
            return Err(SbfError::ParseError("DiskStatus SBLength too small".into()));
        }
        let required_len = MIN_HEADER + n as usize * sb_length_usize;
        if required_len > data.len() {
            return Err(SbfError::ParseError(
                "DiskStatus sub-blocks exceed block length".into(),
            ));
        }
        let mut reserved = [0u8; 4];
        reserved.copy_from_slice(&data[14..18]);
        let mut disks = Vec::with_capacity(n as usize);
        let mut off = MIN_HEADER;
        for _ in 0..n as usize {
            disks.push(DiskData {
                disk_id: data[off],
                status: data[off + 1],
                disk_usage_msb: u16::from_le_bytes(data[off + 2..off + 4].try_into().unwrap()),
                disk_usage_lsb: u32::from_le_bytes(data[off + 4..off + 8].try_into().unwrap()),
                disk_size_mb: u32::from_le_bytes(data[off + 8..off + 12].try_into().unwrap()),
                create_delete_count: data[off + 12],
                error: if header.block_rev >= 1 && sb_length_usize >= 14 {
                    Some(data[off + 13])
                } else {
                    None
                },
            });
            off += sb_length_usize;
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            reserved,
            disks,
        })
    }
}

/// One `P2PPSession` entry from `P2PPStatus`.
#[derive(Debug, Clone)]
pub struct P2ppSession {
    pub session_id: u8,
    pub port: u8,
    pub status: u8,
    pub error_code: u8,
}

/// P2PP session status (4238).
#[derive(Debug, Clone)]
pub struct P2ppStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub sessions: Vec<P2ppSession>,
}

impl P2ppStatusBlock {
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

impl SbfBlockParse for P2ppStatusBlock {
    const BLOCK_ID: u16 = block_ids::P2PP_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_HEADER: usize = 14;
        const MIN_SB: usize = 4;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("P2PPStatus too short".into()));
        }
        let n = data[12];
        let sb_length = data[13];
        let sb_length_usize = sb_length as usize;
        if sb_length_usize < MIN_SB {
            return Err(SbfError::ParseError("P2PPStatus SBLength too small".into()));
        }
        let required_len = MIN_HEADER + n as usize * sb_length_usize;
        if required_len > data.len() {
            return Err(SbfError::ParseError(
                "P2PPStatus sub-blocks exceed block length".into(),
            ));
        }
        let mut sessions = Vec::with_capacity(n as usize);
        let mut off = MIN_HEADER;
        for _ in 0..n as usize {
            sessions.push(P2ppSession {
                session_id: data[off],
                port: data[off + 1],
                status: data[off + 2],
                error_code: data[off + 3],
            });
            off += sb_length_usize;
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            sessions,
        })
    }
}

/// Cosmos service status (4243).
#[derive(Debug, Clone)]
pub struct CosmosStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub status: u8,
}

impl CosmosStatusBlock {
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

impl SbfBlockParse for CosmosStatusBlock {
    const BLOCK_ID: u16 = block_ids::COSMOS_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        if data.len() < 13 {
            return Err(SbfError::ParseError("CosmosStatus too short".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            status: data[12],
        })
    }
}

// ============================================================================
// RxMessage / EncapsulatedOutput / GIS
// ============================================================================

/// Receiver activity log entry (4103).
#[derive(Debug, Clone)]
pub struct RxMessageBlock {
    tow_ms: u32,
    wnc: u16,
    pub message_type: u8,
    pub severity: u8,
    pub message_id: u32,
    pub string_len: u16,
    message: Vec<u8>,
}

impl RxMessageBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn message(&self) -> &[u8] {
        &self.message
    }
    pub fn message_text_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.message)).into_owned()
    }
}

impl SbfBlockParse for RxMessageBlock {
    const BLOCK_ID: u16 = block_ids::RX_MESSAGE;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 22 || data.len() < data_len {
            return Err(SbfError::ParseError("RxMessage too short".into()));
        }
        let string_len = u16::from_le_bytes(data[18..20].try_into().unwrap()) as usize;
        let end = 22 + string_len;
        if end > data_len {
            return Err(SbfError::ParseError("RxMessage length exceeds block".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            message_type: data[12],
            severity: data[13],
            message_id: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            string_len: string_len as u16,
            message: data[22..end].to_vec(),
        })
    }
}

/// Encapsulated non-SBF output message (4097).
#[derive(Debug, Clone)]
pub struct EncapsulatedOutputBlock {
    tow_ms: u32,
    wnc: u16,
    pub mode: u8,
    pub reserved_id: u16,
    payload: Vec<u8>,
}

impl EncapsulatedOutputBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }
}

impl SbfBlockParse for EncapsulatedOutputBlock {
    const BLOCK_ID: u16 = block_ids::ENCAPSULATED_OUTPUT;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 18 || data.len() < data_len {
            return Err(SbfError::ParseError("EncapsulatedOutput too short".into()));
        }
        let payload_len = u16::from_le_bytes(data[14..16].try_into().unwrap()) as usize;
        let end = 18 + payload_len;
        if end > data_len {
            return Err(SbfError::ParseError(
                "EncapsulatedOutput payload exceeds block".into(),
            ));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            mode: data[12],
            reserved_id: u16::from_le_bytes(data[16..18].try_into().unwrap()),
            payload: data[18..end].to_vec(),
        })
    }
}

/// GIS action entry (4106).
#[derive(Debug, Clone)]
pub struct GisActionBlock {
    tow_ms: u32,
    wnc: u16,
    pub comment_len: u16,
    pub item_id_msb: u32,
    pub item_id_lsb: u32,
    pub action: u8,
    pub trigger: u8,
    pub database: u8,
    comment: Vec<u8>,
}

impl GisActionBlock {
    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }
    pub fn wnc(&self) -> u16 {
        self.wnc
    }
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }
    pub fn comment(&self) -> &[u8] {
        &self.comment
    }
    pub fn comment_text_lossy(&self) -> String {
        String::from_utf8_lossy(trim_trailing_nuls(&self.comment)).into_owned()
    }
}

impl SbfBlockParse for GisActionBlock {
    const BLOCK_ID: u16 = block_ids::GIS_ACTION;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let block_len = header.length as usize;
        let data_len = block_len.saturating_sub(2);
        if data_len < 26 || data.len() < data_len {
            return Err(SbfError::ParseError("GISAction too short".into()));
        }
        let comment_len = u16::from_le_bytes(data[12..14].try_into().unwrap()) as usize;
        let end = 26 + comment_len;
        if end > data_len {
            return Err(SbfError::ParseError("GISAction comment exceeds block".into()));
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            comment_len: comment_len as u16,
            item_id_msb: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            item_id_lsb: u32::from_le_bytes(data[18..22].try_into().unwrap()),
            action: data[22],
            trigger: data[23],
            database: data[24],
            comment: data[26..end].to_vec(),
        })
    }
}

/// One `DatabaseStatus` entry from `GISStatus`.
#[derive(Debug, Clone)]
pub struct GisDatabaseStatus {
    pub database: u8,
    pub online_status: u8,
    pub error: u8,
    pub nr_items: u32,
    pub nr_not_sync: u32,
}

/// GIS database status (4107).
#[derive(Debug, Clone)]
pub struct GisStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub databases: Vec<GisDatabaseStatus>,
}

impl GisStatusBlock {
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

impl SbfBlockParse for GisStatusBlock {
    const BLOCK_ID: u16 = block_ids::GIS_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_HEADER: usize = 14;
        const MIN_SB: usize = 12;
        if data.len() < MIN_HEADER {
            return Err(SbfError::ParseError("GISStatus too short".into()));
        }
        let n = data[12];
        let sb_length = data[13];
        let sb_length_usize = sb_length as usize;
        if sb_length_usize < MIN_SB {
            return Err(SbfError::ParseError("GISStatus SBLength too small".into()));
        }
        let required_len = MIN_HEADER + n as usize * sb_length_usize;
        if required_len > data.len() {
            return Err(SbfError::ParseError(
                "GISStatus sub-blocks exceed block length".into(),
            ));
        }
        let mut databases = Vec::with_capacity(n as usize);
        let mut off = MIN_HEADER;
        for _ in 0..n as usize {
            databases.push(GisDatabaseStatus {
                database: data[off],
                online_status: data[off + 1],
                error: data[off + 2],
                nr_items: u32::from_le_bytes(data[off + 4..off + 8].try_into().unwrap()),
                nr_not_sync: u32::from_le_bytes(data[off + 8..off + 12].try_into().unwrap()),
            });
            off += sb_length_usize;
        }
        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            databases,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::SbfBlock;
    use crate::header::{SbfHeader, SBF_SYNC};
    use crate::types::Constellation;

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
    fn test_sat_visibility_elevation() {
        let info = SatVisibilityInfo {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            freq_nr: 0,
            azimuth_raw: 18000,  // 180.00 degrees
            elevation_raw: 4500, // 45.00 degrees
            rise_set: 1,
            satellite_info: 0,
        };

        assert!((info.azimuth_deg().unwrap() - 180.0).abs() < 0.01);
        assert!((info.elevation_deg().unwrap() - 45.0).abs() < 0.01);
        assert!(info.is_rising());
        assert!(info.is_above_horizon());
    }

    #[test]
    fn ntrip_client_status_parse_min() {
        let mut data = vec![0u8; 22];
        data[12] = 2;
        data[13] = 4;
        data[14] = 10;
        data[15] = 20;
        data[16] = 30;
        data[18] = 11;
        data[19] = 21;
        data[20] = 31;
        let header = header_for(block_ids::NTRIP_CLIENT_STATUS, data.len(), 1000, 100);
        let b = NtripClientStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(b.n, 2);
        assert_eq!(b.sb_length, 4);
        assert_eq!(b.connections.len(), 2);
        assert_eq!(b.connections[0].cd_index, 10);
        assert_eq!(b.connections[0].status, 20);
        assert_eq!(b.connections[0].error_code, 30);
        assert_eq!(b.connections[1].cd_index, 11);
        assert_eq!(b.connections[1].status, 21);
        assert_eq!(b.connections[1].error_code, 31);
    }

    #[test]
    fn ntrip_server_status_respects_n() {
        let mut data = vec![0u8; 18];
        data[12] = 1;
        data[13] = 4;
        data[14] = 9;
        data[15] = 8;
        data[16] = 7;
        let header = header_for(block_ids::NTRIP_SERVER_STATUS, data.len(), 1500, 101);
        let b = NtripServerStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(b.n, 1);
        assert_eq!(b.connections.len(), 1);
        assert_eq!(b.connections[0].cd_index, 9);
        assert_eq!(b.connections[0].status, 8);
        assert_eq!(b.connections[0].error_code, 7);
    }

    #[test]
    fn rf_status_parse_min() {
        let mut data = vec![0u8; 34];
        data[12] = 2;
        data[13] = 8;
        data[14..18].copy_from_slice(&0x01020304u32.to_le_bytes());
        data[18..22].copy_from_slice(&1_575_420_000u32.to_le_bytes());
        data[22..24].copy_from_slice(&2000u16.to_le_bytes());
        data[24] = 42;
        data[26..30].copy_from_slice(&1_227_600_000u32.to_le_bytes());
        data[30..32].copy_from_slice(&1000u16.to_le_bytes());
        data[32] = 24;
        let header = header_for(block_ids::RF_STATUS, data.len(), 500, 200);
        let b = RfStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(b.n, 2);
        assert_eq!(b.sb_length, 8);
        assert_eq!(b.reserved, 0x0102_0304);
        assert_eq!(b.bands.len(), 2);
        assert_eq!(b.bands[0].frequency_hz, 1_575_420_000);
        assert_eq!(b.bands[0].bandwidth, 2000);
        assert_eq!(b.bands[0].info, 42);
        assert_eq!(b.bands[1].frequency_hz, 1_227_600_000);
        assert_eq!(b.bands[1].bandwidth, 1000);
        assert_eq!(b.bands[1].info, 24);
    }

    #[test]
    fn test_sat_visibility_invalid() {
        let info = SatVisibilityInfo {
            sat_id: SatelliteId::new(Constellation::GPS, 1),
            freq_nr: 0,
            azimuth_raw: 65535,
            elevation_raw: -32768,
            rise_set: 0,
            satellite_info: 0,
        };

        assert!(info.azimuth_deg().is_none());
        assert!(info.elevation_deg().is_none());
    }

    #[test]
    fn test_receiver_status_uptime() {
        let status = ReceiverStatusBlock {
            tow_ms: 0,
            wnc: 0,
            cpu_load: 50,
            ext_error: 0,
            uptime_s: 3661, // 1 hour, 1 minute, 1 second
            rx_state: 0,
            rx_error: 0,
            cmd_count: None,
            temperature_raw: None,
            agc_data: vec![],
        };

        let (h, m, s) = status.uptime_hms();
        assert_eq!(h, 1);
        assert_eq!(m, 1);
        assert_eq!(s, 1);
    }

    #[test]
    fn test_receiver_status_parse_rev1_with_agc() {
        let mut data = vec![0u8; 30 + 4];
        data[12] = 75; // CPULoad
        data[13] = 2; // ExtError
        data[14..18].copy_from_slice(&120_u32.to_le_bytes()); // UpTime
        data[18..22].copy_from_slice(&0x0001_0002_u32.to_le_bytes()); // RxState
        data[22..26].copy_from_slice(&0x0000_0200_u32.to_le_bytes()); // RxError
        data[26] = 1; // N
        data[27] = 4; // SBLength
        data[28] = 11; // CmdCount
        data[29] = 123; // Temperature raw => 23 degC
        data[30] = 9; // FrontEndID
        data[31] = (-12_i8) as u8; // Gain
        data[32] = 100; // SampleVar
        data[33] = 3; // BlankingStat

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::RECEIVER_STATUS,
            block_rev: 1,
            length: (data.len() + 2) as u16,
            tow_ms: 4321,
            wnc: 2045,
        };

        let block = ReceiverStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.cpu_load, 75);
        assert_eq!(block.cmd_count(), Some(11));
        assert_eq!(block.temperature_raw(), Some(123));
        assert_eq!(block.temperature_celsius(), Some(23));
        assert_eq!(block.agc_data.len(), 1);
        assert_eq!(block.agc_data[0].frontend_id, 9);
        assert_eq!(block.agc_data[0].gain_db, -12);
    }

    #[test]
    fn test_receiver_status_parse_rev1_min_length_enforced() {
        let data = vec![0u8; 26];
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::RECEIVER_STATUS,
            block_rev: 1,
            length: (data.len() + 2) as u16,
            tow_ms: 0,
            wnc: 0,
        };

        let err = ReceiverStatusBlock::parse(&header, &data).unwrap_err();
        assert!(matches!(err, SbfError::ParseError(_)));
    }

    #[test]
    fn test_input_link_accessors() {
        let stats = InputLinkStats {
            connection_descriptor: 1,
            link_type: 2,
            age_last_message_raw: U16_DNU,
            bytes_received: 100,
            bytes_accepted: 90,
            messages_received: 10,
            messages_accepted: 9,
        };
        let block = InputLinkBlock {
            tow_ms: 2000,
            wnc: 3000,
            inputs: vec![stats],
        };

        assert!((block.tow_seconds() - 2.0).abs() < 1e-6);
        assert!(block.inputs[0].age_last_message_s().is_none());
    }

    #[test]
    fn test_input_link_parse() {
        let mut data = vec![0u8; 14 + 20];
        data[12] = 1; // N
        data[13] = 20; // SBLength

        let offset = 14;
        data[offset] = 3; // CD
        data[offset + 1] = 4; // Type
        data[offset + 2..offset + 4].copy_from_slice(&120_u16.to_le_bytes());
        data[offset + 4..offset + 8].copy_from_slice(&1000_u32.to_le_bytes());
        data[offset + 8..offset + 12].copy_from_slice(&900_u32.to_le_bytes());
        data[offset + 12..offset + 16].copy_from_slice(&10_u32.to_le_bytes());
        data[offset + 16..offset + 20].copy_from_slice(&9_u32.to_le_bytes());

        let header = header_for(block_ids::INPUT_LINK, data.len(), 123456, 2222);
        let block = InputLinkBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_links(), 1);
        let entry = &block.inputs[0];
        assert_eq!(entry.connection_descriptor, 3);
        assert_eq!(entry.link_type, 4);
        assert_eq!(entry.age_last_message_raw(), 120);
        assert_eq!(entry.bytes_received, 1000);
        assert_eq!(entry.messages_accepted, 9);
    }

    #[test]
    fn test_quality_ind_parse() {
        let mut data = vec![0u8; 14 + 6];
        data[12] = 3; // N
        data[13] = 0; // Reserved

        data[14..16].copy_from_slice(&0x1234_u16.to_le_bytes());
        data[16..18].copy_from_slice(&0x5678_u16.to_le_bytes());
        data[18..20].copy_from_slice(&0x9abc_u16.to_le_bytes());

        let header = header_for(block_ids::QUALITY_IND, data.len(), 1000, 2000);
        let block = QualityIndBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_indicators(), 3);
        assert_eq!(block.indicators[0], 0x1234);
        assert_eq!(block.indicators[1], 0x5678);
        assert_eq!(block.indicators[2], 0x9abc);
    }

    #[test]
    fn test_quality_ind_sbf_block_parse() {
        let indicators = [0x1111_u16, 0x2222_u16];
        let n = indicators.len();
        let total_len = 16 + (n * 2); // includes sync bytes
        assert_eq!(total_len % 4, 0);

        let mut data = vec![0u8; total_len];
        data[0..2].copy_from_slice(&SBF_SYNC);
        data[2..4].copy_from_slice(&0_u16.to_le_bytes()); // CRC (unused)
        data[4..6].copy_from_slice(&block_ids::QUALITY_IND.to_le_bytes()); // ID/Rev
        data[6..8].copy_from_slice(&(total_len as u16).to_le_bytes()); // Length
        data[8..12].copy_from_slice(&1000_u32.to_le_bytes()); // TOW
        data[12..14].copy_from_slice(&2000_u16.to_le_bytes()); // WNc
        data[14] = n as u8;
        data[15] = 0; // Reserved

        let mut offset = 16;
        for value in indicators {
            data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
            offset += 2;
        }

        let (block, used) = SbfBlock::parse(&data).unwrap();
        assert_eq!(used, total_len);
        assert_eq!(block.block_id(), block_ids::QUALITY_IND);
        match block {
            SbfBlock::QualityInd(quality) => {
                assert_eq!(quality.wnc(), 2000);
                assert_eq!(quality.tow_ms(), 1000);
                assert_eq!(quality.indicators, indicators);
            }
            _ => panic!("Expected QualityInd block"),
        }
    }

    #[test]
    fn test_output_link_accessors() {
        let stats = OutputLinkStats {
            connection_descriptor: 2,
            allowed_rate_raw: 500,
            bytes_produced: 2000,
            bytes_sent: 1900,
            nr_clients: 1,
            output_types: vec![OutputType {
                output_type: 10,
                percentage: 50,
            }],
        };
        let block = OutputLinkBlock {
            tow_ms: 3000,
            wnc: 4000,
            outputs: vec![stats],
        };

        assert!((block.tow_seconds() - 3.0).abs() < 1e-6);
        assert_eq!(block.outputs[0].allowed_rate_kbytes_per_s(), 500);
        assert_eq!(block.outputs[0].allowed_rate_bytes_per_s(), 500_000);
        assert_eq!(block.outputs[0].allowed_rate_bps(), Some(500));
    }

    #[test]
    fn test_ip_status_parse() {
        let mut data = vec![0u8; 51];
        data[6..10].copy_from_slice(&5000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2400u16.to_le_bytes());
        data[12..18].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        data[18..34].copy_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 168, 1, 100]);
        data[34..50].copy_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 168, 1, 1]);
        data[50] = 24;

        let header = header_for(block_ids::IP_STATUS, 51, 5000, 2400);
        let block = IpStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2400);
        assert_eq!(block.mac_address, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        assert_eq!(block.mac_address_string(), "AA:BB:CC:DD:EE:FF");
        assert_eq!(block.ip_address_string(), "192.168.1.100");
        assert_eq!(block.gateway_string(), "192.168.1.1");
        assert_eq!(block.netmask_prefix, 24);
    }

    #[test]
    fn test_output_link_parse() {
        let mut data = vec![0u8; 18 + 13 + 2];
        data[12] = 1; // N1
        data[13] = 13; // SB1Length
        data[14] = 2; // SB2Length

        // 15..17 reserved
        let offset = 18;
        data[offset] = 1; // CD
        data[offset + 1] = 1; // N2
        data[offset + 2..offset + 4].copy_from_slice(&500_u16.to_le_bytes());
        data[offset + 4..offset + 8].copy_from_slice(&2000_u32.to_le_bytes());
        data[offset + 8..offset + 12].copy_from_slice(&1800_u32.to_le_bytes());
        data[offset + 12] = 2; // NrClients

        let type_offset = offset + 13;
        data[type_offset] = 7; // Output type
        data[type_offset + 1] = 80; // Percentage

        let header = header_for(block_ids::OUTPUT_LINK, data.len(), 654321, 3333);
        let block = OutputLinkBlock::parse(&header, &data).unwrap();

        assert_eq!(block.num_links(), 1);
        let entry = &block.outputs[0];
        assert_eq!(entry.connection_descriptor, 1);
        assert_eq!(entry.allowed_rate_raw(), 500);
        assert_eq!(entry.nr_clients, 2);
        assert_eq!(entry.output_types.len(), 1);
        assert_eq!(entry.output_types[0].output_type, 7);
        assert_eq!(entry.output_types[0].percentage, 80);
    }

    #[test]
    fn test_tracking_status_sbf_block_parse() {
        // 2 sync + 12 header + 3 base fields + 3 reserved + 12 sat info + 8 state info = 40
        let total_len = 40usize;
        let mut data = vec![0u8; total_len];

        data[0..2].copy_from_slice(&SBF_SYNC);
        data[2..4].copy_from_slice(&0_u16.to_le_bytes()); // CRC (unused)
        data[4..6].copy_from_slice(&block_ids::TRACKING_STATUS.to_le_bytes()); // ID/Rev
        data[6..8].copy_from_slice(&(total_len as u16).to_le_bytes()); // Length
        data[8..12].copy_from_slice(&12345_u32.to_le_bytes()); // TOW
        data[12..14].copy_from_slice(&2045_u16.to_le_bytes()); // WNc

        // Block payload starts at absolute offset 14 (offset 12 in parse() data slice)
        data[14] = 1; // N1
        data[15] = 12; // SB1Length
        data[16] = 8; // SB2Length
        data[17] = 0; // Reserved
        data[18] = 0; // Reserved
        data[19] = 0; // Reserved

        let sb1 = 20;
        data[sb1] = 5; // SVID
        data[sb1 + 1] = 0; // FreqNr
        data[sb1 + 2..sb1 + 4].copy_from_slice(&0_u16.to_le_bytes()); // SVIDFull
        let az_rise_set = (1_u16 << 14) | 180_u16; // rise=1, azimuth=180 deg
        data[sb1 + 4..sb1 + 6].copy_from_slice(&az_rise_set.to_le_bytes());
        data[sb1 + 6..sb1 + 8].copy_from_slice(&0_u16.to_le_bytes()); // HealthStatus
        data[sb1 + 8] = 45; // Elevation
        data[sb1 + 9] = 1; // N2
        data[sb1 + 10] = 3; // RxChannel
        data[sb1 + 11] = 0; // Reserved

        let sb2 = sb1 + 12;
        data[sb2] = 0; // Antenna
        data[sb2 + 1] = 0; // Reserved
        data[sb2 + 2..sb2 + 4].copy_from_slice(&3_u16.to_le_bytes()); // TrackingStatus
        data[sb2 + 4..sb2 + 6].copy_from_slice(&2_u16.to_le_bytes()); // PVTStatus
        data[sb2 + 6..sb2 + 8].copy_from_slice(&0x1234_u16.to_le_bytes()); // PVTInfo

        let (block, used) = SbfBlock::parse(&data).unwrap();
        assert_eq!(used, total_len);
        assert_eq!(block.block_id(), block_ids::TRACKING_STATUS);
        match block {
            SbfBlock::TrackingStatus(tracking) => {
                assert_eq!(tracking.tow_ms(), 12345);
                assert_eq!(tracking.wnc(), 2045);
                assert_eq!(tracking.num_satellites(), 1);
                assert_eq!(tracking.satellites[0].states.len(), 1);
                assert_eq!(tracking.satellites[0].states[0].tracking_status, 3);
                assert_eq!(tracking.satellites[0].states[0].pvt_status, 2);
                assert_eq!(tracking.satellites[0].states[0].pvt_info, 0x1234);
            }
            _ => panic!("Expected TrackingStatus block"),
        }
    }

    #[test]
    fn test_lband_tracker_data_accessors() {
        let entry = LBandTrackerData {
            frequency_hz: 1_545_000_000,
            baudrate: 1200,
            service_id: 7,
            freq_offset_hz_raw: F32_DNU,
            cn0_raw: 0,
            avg_power_raw: I16_DNU,
            agc_gain_db_raw: I8_DNU,
            mode: 0,
            status: 1,
            svid: None,
            lock_time_s: None,
            source: None,
        };

        assert!(entry.freq_offset_hz().is_none());
        assert!(entry.cn0_dbhz().is_none());
        assert!(entry.avg_power_db().is_none());
        assert!(entry.agc_gain_db().is_none());
    }

    #[test]
    fn test_lband_tracker_status_parse_rev3() {
        let sb_length = 24usize;
        let mut data = vec![0u8; 14 + sb_length];
        data[12] = 1; // N
        data[13] = sb_length as u8; // SBLength

        let offset = 14;
        data[offset..offset + 4].copy_from_slice(&1_545_000_000_u32.to_le_bytes()); // Frequency
        data[offset + 4..offset + 6].copy_from_slice(&1200_u16.to_le_bytes()); // Baudrate
        data[offset + 6..offset + 8].copy_from_slice(&42_u16.to_le_bytes()); // ServiceID
        data[offset + 8..offset + 12].copy_from_slice(&12.5_f32.to_le_bytes()); // FreqOffset
        data[offset + 12..offset + 14].copy_from_slice(&4550_u16.to_le_bytes()); // CN0 (45.50 dB-Hz)
        data[offset + 14..offset + 16].copy_from_slice(&(-123_i16).to_le_bytes()); // AvgPower
        data[offset + 16] = (-7_i8) as u8; // AGCGain
        data[offset + 17] = 0; // Mode
        data[offset + 18] = 3; // Status (Locked)
        data[offset + 19] = 110; // SVID (Rev2+)
        data[offset + 20..offset + 22].copy_from_slice(&360_u16.to_le_bytes()); // LockTime (Rev1+)
        data[offset + 22] = 2; // Source (Rev3+)

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::LBAND_TRACKER_STATUS,
            block_rev: 3,
            length: (data.len() + 2) as u16,
            tow_ms: 7777,
            wnc: 2099,
        };

        let block = LBandTrackerStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.n, 1);
        assert_eq!(block.sb_length, sb_length as u8);
        assert_eq!(block.num_trackers(), 1);

        let entry = &block.trackers[0];
        assert_eq!(entry.frequency_hz, 1_545_000_000);
        assert_eq!(entry.baudrate, 1200);
        assert_eq!(entry.service_id, 42);
        assert!((entry.freq_offset_hz().unwrap() - 12.5).abs() < 1e-6);
        assert!((entry.cn0_dbhz().unwrap() - 45.5).abs() < 1e-6);
        assert!((entry.avg_power_db().unwrap() + 1.23).abs() < 1e-6);
        assert_eq!(entry.agc_gain_db(), Some(-7));
        assert_eq!(entry.svid, Some(110));
        assert_eq!(entry.lock_time_s, Some(360));
        assert_eq!(entry.source, Some(2));
    }

    #[test]
    fn test_receiver_setup_parse_rev4() {
        let mut data = vec![0u8; 422];
        data[14..18].copy_from_slice(b"TEST");
        data[74..78].copy_from_slice(b"1234");
        data[94..102].copy_from_slice(b"Observer");
        data[114..120].copy_from_slice(b"Agency");
        data[154..160].copy_from_slice(b"RX1234");
        data[174..183].copy_from_slice(b"mosaic-X5");
        data[194..200].copy_from_slice(b"4.15.1");
        data[214..220].copy_from_slice(b"ANT123");
        data[234..242].copy_from_slice(b"ANT-TYPE");
        data[254..258].copy_from_slice(&1.25_f32.to_le_bytes());
        data[258..262].copy_from_slice(&(-0.5_f32).to_le_bytes());
        data[262..266].copy_from_slice(&0.75_f32.to_le_bytes());
        data[266..274].copy_from_slice(b"GEODETIC");
        data[286..293].copy_from_slice(b"GNSS_FW");
        data[326..335].copy_from_slice(b"mosaic-X5");
        data[366..374].copy_from_slice(&0.5_f64.to_le_bytes());
        data[374..382].copy_from_slice(&1.0_f64.to_le_bytes());
        data[382..386].copy_from_slice(&123.25_f32.to_le_bytes());
        data[386..390].copy_from_slice(b"TST1");
        data[396] = 1;
        data[397] = 2;
        data[398..401].copy_from_slice(b"BEL");

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::RECEIVER_SETUP,
            block_rev: 4,
            length: (data.len() + 2) as u16,
            tow_ms: 60_000,
            wnc: 2300,
        };
        let block = ReceiverSetupBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_seconds(), 60.0);
        assert_eq!(block.marker_name_lossy(), "TEST");
        assert_eq!(block.marker_number_lossy(), "1234");
        assert_eq!(block.observer_lossy(), "Observer");
        assert_eq!(block.agency_lossy(), "Agency");
        assert_eq!(block.rx_serial_number_lossy(), "RX1234");
        assert_eq!(block.rx_name_lossy(), "mosaic-X5");
        assert_eq!(block.rx_version_lossy(), "4.15.1");
        assert_eq!(block.ant_serial_number_lossy(), "ANT123");
        assert_eq!(block.ant_type_lossy(), "ANT-TYPE");
        assert_eq!(block.delta_h_m(), 1.25);
        assert_eq!(block.delta_e_m(), -0.5);
        assert_eq!(block.delta_n_m(), 0.75);
        assert_eq!(block.marker_type_lossy().as_deref(), Some("GEODETIC"));
        assert_eq!(block.gnss_fw_version_lossy().as_deref(), Some("GNSS_FW"));
        assert_eq!(block.product_name_lossy().as_deref(), Some("mosaic-X5"));
        assert!((block.latitude_deg().unwrap() - 28.64788975654116).abs() < 1e-9);
        assert!((block.longitude_deg().unwrap() - 57.29577951308232).abs() < 1e-9);
        assert_eq!(block.height_m(), Some(123.25));
        assert_eq!(block.station_code_lossy().as_deref(), Some("TST1"));
        assert_eq!(block.monument_idx(), Some(1));
        assert_eq!(block.receiver_idx(), Some(2));
        assert_eq!(block.country_code_lossy().as_deref(), Some("BEL"));
    }

    #[test]
    fn test_receiver_setup_dnu_reference_position() {
        let mut data = vec![0u8; 386];
        data[366..374].copy_from_slice(&F64_DNU.to_le_bytes());
        data[374..382].copy_from_slice(&F64_DNU.to_le_bytes());
        data[382..386].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::RECEIVER_SETUP,
            block_rev: 3,
            length: (data.len() + 2) as u16,
            tow_ms: 1,
            wnc: 2,
        };
        let block = ReceiverSetupBlock::parse(&header, &data).unwrap();

        assert!(block.latitude_deg().is_none());
        assert!(block.longitude_deg().is_none());
        assert!(block.height_m().is_none());
    }

    #[test]
    fn test_bb_samples_parse() {
        let mut data = vec![0u8; 34];
        data[12..14].copy_from_slice(&2_u16.to_le_bytes());
        data[14] = 2;
        data[18..22].copy_from_slice(&40_000_000_u32.to_le_bytes());
        data[22..26].copy_from_slice(&1_575_420_000_u32.to_le_bytes());
        data[26..28].copy_from_slice(&0xFF02_u16.to_le_bytes());
        data[28..30].copy_from_slice(&0x7F80_u16.to_le_bytes());
        data[30..34].copy_from_slice(&0.125_f32.to_le_bytes());

        let header = header_for(block_ids::BB_SAMPLES, data.len(), 2000, 2100);
        let block = BBSamplesBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_seconds(), 2.0);
        assert_eq!(block.num_samples(), 2);
        assert_eq!(block.antenna_id(), 2);
        assert_eq!(block.sample_freq_hz(), 40_000_000);
        assert_eq!(block.lo_freq_hz(), 1_575_420_000);
        assert_eq!(block.samples()[0].raw(), 0xFF02);
        assert_eq!(block.sample_iq(0), Some((-1, 2)));
        assert_eq!(block.sample_iq(1), Some((127, -128)));
        assert_eq!(block.tow_delta_seconds(), Some(0.125));
        assert_eq!(block.first_sample_time_seconds(), Some(2.125));
    }

    #[test]
    fn test_bb_samples_tow_delta_dnu() {
        let mut data = vec![0u8; 30];
        data[12..14].copy_from_slice(&0_u16.to_le_bytes());
        data[30 - 4..30].copy_from_slice(&F32_DNU.to_le_bytes());

        let header = header_for(block_ids::BB_SAMPLES, data.len(), 0, 0);
        let block = BBSamplesBlock::parse(&header, &data).unwrap();

        assert!(block.tow_delta_seconds().is_none());
        assert!(block.first_sample_time_seconds().is_none());
    }

    #[test]
    fn test_ascii_in_parse() {
        let mut data = vec![0u8; 86];
        data[12] = 33;
        data[16..18].copy_from_slice(&5_u16.to_le_bytes());
        data[18..22].copy_from_slice(b"MET4");
        data[38..40].copy_from_slice(b"WX");
        data[78..83].copy_from_slice(b"hello");

        let header = header_for(block_ids::ASCII_IN, data.len(), 9000, 2200);
        let block = ASCIIInBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_seconds(), 9.0);
        assert_eq!(block.connection_descriptor(), 33);
        assert_eq!(block.string_len(), 5);
        assert_eq!(block.sensor_model_lossy(), "MET4");
        assert_eq!(block.sensor_type_lossy(), "WX");
        assert_eq!(block.ascii_string(), b"hello");
        assert_eq!(block.ascii_text_lossy(), "hello");
    }

    #[test]
    fn test_ascii_in_rejects_overlong_string() {
        let mut data = vec![0u8; 82];
        data[16..18].copy_from_slice(&8_u16.to_le_bytes());

        let header = header_for(block_ids::ASCII_IN, data.len(), 0, 0);
        assert!(ASCIIInBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_commands_parse() {
        let mut data = vec![0u8; 20];
        data[14..20].copy_from_slice(&[b's', b'e', b't', 0, 0, 0]);

        let header = header_for(block_ids::COMMANDS, data.len(), 1234, 2040);
        let block = CommandsBlock::parse(&header, &data).unwrap();

        assert_eq!(block.tow_ms(), 1234);
        assert_eq!(block.wnc(), 2040);
        assert_eq!(block.cmd_data(), &[b's', b'e', b't', 0, 0, 0]);
        assert_eq!(block.cmd_text_lossy(), "set");
    }

    #[test]
    fn test_comment_parse() {
        let mut data = vec![0u8; 20];
        data[12..14].copy_from_slice(&5_u16.to_le_bytes()); // CommentLn
        data[14..19].copy_from_slice(b"hello");
        data[19] = 0; // Padding

        let header = header_for(block_ids::COMMENT, data.len(), 4321, 2055);
        let block = CommentBlock::parse(&header, &data).unwrap();

        assert_eq!(block.comment_len(), 5);
        assert_eq!(block.comment_data(), b"hello");
        assert_eq!(block.comment_text_lossy(), "hello");
    }

    #[test]
    fn test_comment_parse_rejects_too_long_length() {
        let mut data = vec![0u8; 16];
        data[12..14].copy_from_slice(&8_u16.to_le_bytes()); // CommentLn exceeds payload

        let header = header_for(block_ids::COMMENT, data.len(), 0, 0);
        assert!(CommentBlock::parse(&header, &data).is_err());
    }

    #[test]
    fn test_rtcm_datum_parse() {
        let mut data = vec![0u8; 79];
        data[12..20].copy_from_slice(b"WGS84\0\0\0");
        data[44..53].copy_from_slice(b"ETRS89\0\0\0");
        data[76] = 19;
        data[77] = 2;
        data[78] = 0xA5;
        let header = header_for(block_ids::RTCM_DATUM, data.len(), 100, 200);
        let block = RtcmDatumBlock::parse(&header, &data).unwrap();
        assert_eq!(block.source_crs_lossy(), "WGS84");
        assert_eq!(block.target_crs_lossy(), "ETRS89");
        assert_eq!(block.datum, 19);
        assert_eq!(block.height_type, 2);
        assert_eq!(block.quality_ind, 0xA5);
    }

    #[test]
    fn test_lband_beams_parse() {
        let mut data = vec![0u8; 14 + 16];
        data[12] = 1;
        data[13] = 16;
        data[14] = 110;
        data[15..24].copy_from_slice(b"AORE\0\0\0\0\0");
        data[24..26].copy_from_slice(&(-1550i16).to_le_bytes());
        data[26..30].copy_from_slice(&1_539_982_500u32.to_le_bytes());
        let header = header_for(block_ids::LBAND_BEAMS, data.len(), 200, 300);
        let block = LBandBeamsBlock::parse(&header, &data).unwrap();
        assert_eq!(block.beams.len(), 1);
        assert_eq!(block.beams[0].svid, 110);
        assert_eq!(block.beams[0].sat_name_lossy(), "AORE");
        assert!((block.beams[0].sat_longitude_deg().unwrap() + 15.5).abs() < 1e-6);
        assert_eq!(block.beams[0].beam_freq_hz, 1_539_982_500);
    }

    #[test]
    fn test_dyndns_status_parse_rev2() {
        let mut data = vec![0u8; 46];
        data[12] = 2;
        data[13] = 0;
        data[14..30].copy_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 168, 1, 10]);
        data[30..46].copy_from_slice(&[0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::DYN_DNS_STATUS,
            block_rev: 2,
            length: (data.len() + 2) as u16,
            tow_ms: 1,
            wnc: 2,
        };
        let block = DynDnsStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.status, 2);
        assert_eq!(block.ip_address_string(), "192.168.1.10");
        assert!(block.ipv6_address.is_some());
    }

    #[test]
    fn test_disk_status_parse_rev1() {
        let mut data = vec![0u8; 18 + 16];
        data[12] = 1;
        data[13] = 16;
        data[18] = 1;
        data[19] = 0b0010_1111;
        data[20..22].copy_from_slice(&1u16.to_le_bytes());
        data[22..26].copy_from_slice(&2u32.to_le_bytes());
        data[26..30].copy_from_slice(&4096u32.to_le_bytes());
        data[30] = 9;
        data[31] = 4;
        let header = SbfHeader {
            crc: 0,
            block_id: block_ids::DISK_STATUS,
            block_rev: 1,
            length: (data.len() + 2) as u16,
            tow_ms: 10,
            wnc: 20,
        };
        let block = DiskStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.disks.len(), 1);
        assert_eq!(block.disks[0].disk_id, 1);
        assert_eq!(block.disks[0].disk_usage_bytes(), Some((1u64 << 32) | 2));
        assert_eq!(block.disks[0].disk_size_mb, 4096);
        assert_eq!(block.disks[0].error, Some(4));
    }

    #[test]
    fn test_p2pp_status_parse() {
        let mut data = vec![0u8; 14 + 4];
        data[12] = 1;
        data[13] = 4;
        data[14] = 2;
        data[15] = 1;
        data[16] = 0b0000_0100;
        data[17] = 9;
        let header = header_for(block_ids::P2PP_STATUS, data.len(), 30, 40);
        let block = P2ppStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.sessions.len(), 1);
        assert_eq!(block.sessions[0].session_id, 2);
        assert_eq!(block.sessions[0].port, 1);
        assert_eq!(block.sessions[0].status, 0b0000_0100);
        assert_eq!(block.sessions[0].error_code, 9);
    }

    #[test]
    fn test_rx_message_parse() {
        let mut data = vec![0u8; 22 + 6];
        data[12] = 4;
        data[13] = 2;
        data[14..18].copy_from_slice(&77u32.to_le_bytes());
        data[18..20].copy_from_slice(&6u16.to_le_bytes());
        data[22..28].copy_from_slice(b"boot!\0");
        let header = header_for(block_ids::RX_MESSAGE, data.len(), 50, 60);
        let block = RxMessageBlock::parse(&header, &data).unwrap();
        assert_eq!(block.message_type, 4);
        assert_eq!(block.severity, 2);
        assert_eq!(block.message_id, 77);
        assert_eq!(block.message_text_lossy(), "boot!");
    }

    #[test]
    fn test_encapsulated_output_parse() {
        let mut data = vec![0u8; 18 + 5];
        data[12] = 4;
        data[14..16].copy_from_slice(&5u16.to_le_bytes());
        data[16..18].copy_from_slice(&99u16.to_le_bytes());
        data[18..23].copy_from_slice(b"$GGA\n");
        let header = header_for(block_ids::ENCAPSULATED_OUTPUT, data.len(), 70, 80);
        let block = EncapsulatedOutputBlock::parse(&header, &data).unwrap();
        assert_eq!(block.mode, 4);
        assert_eq!(block.reserved_id, 99);
        assert_eq!(block.payload(), b"$GGA\n");
    }

    #[test]
    fn test_gis_and_cosmos_parse() {
        let mut action = vec![0u8; 26 + 4];
        action[12..14].copy_from_slice(&4u16.to_le_bytes());
        action[14..18].copy_from_slice(&1u32.to_le_bytes());
        action[18..22].copy_from_slice(&2u32.to_le_bytes());
        action[22] = 3;
        action[23] = 4;
        action[24] = 5;
        action[26..30].copy_from_slice(b"note");
        let action_header = header_for(block_ids::GIS_ACTION, action.len(), 90, 91);
        let action_block = GisActionBlock::parse(&action_header, &action).unwrap();
        assert_eq!(action_block.item_id_msb, 1);
        assert_eq!(action_block.item_id_lsb, 2);
        assert_eq!(action_block.comment_text_lossy(), "note");

        let mut gis = vec![0u8; 14 + 12];
        gis[12] = 1;
        gis[13] = 12;
        gis[14] = 7;
        gis[15] = 1;
        gis[16] = 2;
        gis[18..22].copy_from_slice(&123u32.to_le_bytes());
        gis[22..26].copy_from_slice(&9u32.to_le_bytes());
        let gis_header = header_for(block_ids::GIS_STATUS, gis.len(), 92, 93);
        let gis_block = GisStatusBlock::parse(&gis_header, &gis).unwrap();
        assert_eq!(gis_block.databases.len(), 1);
        assert_eq!(gis_block.databases[0].nr_items, 123);

        let mut cosmos = vec![0u8; 13];
        cosmos[12] = 6;
        let cosmos_header = header_for(block_ids::COSMOS_STATUS, cosmos.len(), 94, 95);
        let cosmos_block = CosmosStatusBlock::parse(&cosmos_header, &cosmos).unwrap();
        assert_eq!(cosmos_block.status, 6);
    }
}
