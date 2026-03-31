//! External sensor blocks (ExtSensorMeas, ExtSensorStatus, ExtSensorSetup)

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::dnu::f64_or_none;
use super::SbfBlockParse;

#[cfg(test)]
use super::dnu::F64_DNU;

// ============================================================================
// ExtSensorMeas Block
// ============================================================================

/// Single measurement from an external sensor (ExtSensorMeas sub-block)
#[derive(Debug, Clone)]
pub struct ExtSensorMeasSet {
    pub source: u8,
    pub sensor_model: u8,
    pub meas_type: u8,
    pub obs_info: u8,
    x: f64,
    y: f64,
    z: f64,
}

impl ExtSensorMeasSet {
    pub fn x_m(&self) -> Option<f64> {
        f64_or_none(self.x)
    }
    pub fn y_m(&self) -> Option<f64> {
        f64_or_none(self.y)
    }
    pub fn z_m(&self) -> Option<f64> {
        f64_or_none(self.z)
    }
}

/// ExtSensorMeas block (Block ID 4050)
///
/// External sensor measurements (accelerometer, gyro, etc.).
#[derive(Debug, Clone)]
pub struct ExtSensorMeasBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub meas_sets: Vec<ExtSensorMeasSet>,
}

impl ExtSensorMeasBlock {
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

impl SbfBlockParse for ExtSensorMeasBlock {
    const BLOCK_ID: u16 = block_ids::EXT_SENSOR_MEAS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 16;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("ExtSensorMeas too short".into()));
        }

        let n = data[12];
        let sb_length = data[13];

        if sb_length < 28 {
            return Err(SbfError::ParseError(
                "ExtSensorMeas SBLength too small for MeasSet".into(),
            ));
        }

        let mut meas_sets = Vec::with_capacity(n as usize);
        let mut offset = 14usize;

        for _ in 0..n {
            if offset + 28 > data.len() {
                return Err(SbfError::ParseError(
                    "ExtSensorMeas sub-block exceeds block length".into(),
                ));
            }

            let source = data[offset];
            let sensor_model = data[offset + 1];
            let meas_type = data[offset + 2];
            let obs_info = data[offset + 3];
            let x = f64::from_le_bytes(data[offset + 4..offset + 12].try_into().unwrap());
            let y = f64::from_le_bytes(data[offset + 12..offset + 20].try_into().unwrap());
            let z = f64::from_le_bytes(data[offset + 20..offset + 28].try_into().unwrap());

            meas_sets.push(ExtSensorMeasSet {
                source,
                sensor_model,
                meas_type,
                obs_info,
                x,
                y,
                z,
            });

            offset += sb_length as usize;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            meas_sets,
        })
    }
}

// ============================================================================
// ExtSensorStatus Block
// ============================================================================

/// ExtSensorStatus block (Block ID 4056)
///
/// External sensor status/health information.
#[derive(Debug, Clone)]
pub struct ExtSensorStatusBlock {
    tow_ms: u32,
    wnc: u16,
    pub source: u8,
    pub sensor_model: u8,
    pub status_type: u8,
    pub status_bits: Vec<u8>,
}

impl ExtSensorStatusBlock {
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

impl SbfBlockParse for ExtSensorStatusBlock {
    const BLOCK_ID: u16 = block_ids::EXT_SENSOR_STATUS;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 15;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("ExtSensorStatus too short".into()));
        }

        let source = data[12];
        let sensor_model = data[13];
        let status_type = data[14];
        let status_bits = data[15..].to_vec();

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            source,
            sensor_model,
            status_type,
            status_bits,
        })
    }
}

// ============================================================================
// ExtSensorSetup Block
// ============================================================================

/// Single sensor setup entry (ExtSensorSetup sub-block)
#[derive(Debug, Clone)]
pub struct ExtSensorSetupEntry {
    pub source: u8,
    pub sensor_model: u8,
    pub meas_type: u16,
}

/// ExtSensorSetup block (Block ID 4057)
///
/// External sensor configuration/setup.
#[derive(Debug, Clone)]
pub struct ExtSensorSetupBlock {
    tow_ms: u32,
    wnc: u16,
    pub n: u8,
    pub sb_length: u8,
    pub sensors: Vec<ExtSensorSetupEntry>,
}

impl ExtSensorSetupBlock {
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

impl SbfBlockParse for ExtSensorSetupBlock {
    const BLOCK_ID: u16 = block_ids::EXT_SENSOR_SETUP;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        const MIN_LEN: usize = 16;
        if data.len() < MIN_LEN {
            return Err(SbfError::ParseError("ExtSensorSetup too short".into()));
        }

        let n = data[12];
        let sb_length = data[13];

        if sb_length < 4 {
            return Err(SbfError::ParseError(
                "ExtSensorSetup SBLength too small for OneSensor".into(),
            ));
        }

        let mut sensors = Vec::with_capacity(n as usize);
        let mut offset = 14usize;

        for _ in 0..n {
            if offset + 4 > data.len() {
                return Err(SbfError::ParseError(
                    "ExtSensorSetup sub-block exceeds block length".into(),
                ));
            }

            let source = data[offset];
            let sensor_model = data[offset + 1];
            let meas_type = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);

            sensors.push(ExtSensorSetupEntry {
                source,
                sensor_model,
                meas_type,
            });

            offset += sb_length as usize;
        }

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            n,
            sb_length,
            sensors,
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
            length: 100,
            tow_ms,
            wnc,
        }
    }

    #[test]
    fn test_ext_sensor_meas_parse() {
        let mut data = vec![0u8; 70];
        data[6..10].copy_from_slice(&5000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2400u16.to_le_bytes());
        data[12] = 2; // N
        data[13] = 28; // SBLength
        data[14] = 1; // Source
        data[15] = 2; // SensorModel
        data[16] = 3; // Type
        data[17] = 0; // ObsInfo
        data[18..26].copy_from_slice(&1.5f64.to_le_bytes());
        data[26..34].copy_from_slice(&2.5f64.to_le_bytes());
        data[34..42].copy_from_slice(&3.5f64.to_le_bytes());
        data[42] = 2; // Source
        data[43] = 3; // SensorModel
        data[44] = 4; // Type
        data[45] = 0; // ObsInfo
        data[46..54].copy_from_slice(&4.0f64.to_le_bytes());
        data[54..62].copy_from_slice(&5.0f64.to_le_bytes());
        data[62..70].copy_from_slice(&6.0f64.to_le_bytes());

        let header = header_for(block_ids::EXT_SENSOR_MEAS, 5000, 2400);
        let block = ExtSensorMeasBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 5.0);
        assert_eq!(block.wnc(), 2400);
        assert_eq!(block.n, 2);
        assert_eq!(block.meas_sets.len(), 2);
        assert_eq!(block.meas_sets[0].source, 1);
        assert_eq!(block.meas_sets[0].x_m(), Some(1.5));
        assert_eq!(block.meas_sets[0].y_m(), Some(2.5));
        assert_eq!(block.meas_sets[0].z_m(), Some(3.5));
        assert_eq!(block.meas_sets[1].x_m(), Some(4.0));
    }

    #[test]
    fn test_ext_sensor_meas_dnu() {
        let mut data = vec![0u8; 50];
        data[6..10].copy_from_slice(&5000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2400u16.to_le_bytes());
        data[12] = 1;
        data[13] = 28;
        data[18..26].copy_from_slice(&F64_DNU.to_le_bytes());

        let header = header_for(block_ids::EXT_SENSOR_MEAS, 5000, 2400);
        let block = ExtSensorMeasBlock::parse(&header, &data).unwrap();
        assert!(block.meas_sets[0].x_m().is_none());
    }

    #[test]
    fn test_ext_sensor_status_parse() {
        let mut data = vec![0u8; 20];
        data[6..10].copy_from_slice(&6000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2500u16.to_le_bytes());
        data[12] = 1; // Source
        data[13] = 2; // SensorModel
        data[14] = 0; // StatusType
        data[15..20].copy_from_slice(&[0x01, 0x02, 0x03, 0x04, 0x05]);

        let header = header_for(block_ids::EXT_SENSOR_STATUS, 6000, 2500);
        let block = ExtSensorStatusBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 6.0);
        assert_eq!(block.source, 1);
        assert_eq!(block.sensor_model, 2);
        assert_eq!(block.status_bits.len(), 5);
    }

    #[test]
    fn test_ext_sensor_setup_parse() {
        let mut data = vec![0u8; 30];
        data[6..10].copy_from_slice(&7000u32.to_le_bytes());
        data[10..12].copy_from_slice(&2600u16.to_le_bytes());
        data[12] = 2; // N
        data[13] = 4; // SBLength
        data[14] = 1;
        data[15] = 2;
        data[16..18].copy_from_slice(&0x0102u16.to_le_bytes());
        data[18] = 2;
        data[19] = 3;
        data[20..22].copy_from_slice(&0x0304u16.to_le_bytes());

        let header = header_for(block_ids::EXT_SENSOR_SETUP, 7000, 2600);
        let block = ExtSensorSetupBlock::parse(&header, &data).unwrap();
        assert_eq!(block.tow_seconds(), 7.0);
        assert_eq!(block.n, 2);
        assert_eq!(block.sensors.len(), 2);
        assert_eq!(block.sensors[0].source, 1);
        assert_eq!(block.sensors[0].meas_type, 0x0102);
        assert_eq!(block.sensors[1].sensor_model, 3);
    }
}
