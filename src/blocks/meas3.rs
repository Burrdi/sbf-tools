//! Meas3 measurement blocks (4109–4113)
//!
//! Layout follows Septentrio SBF definitions: `Meas3Ranges` holds packed `M3SatData`
//! sub-blocks; the extension blocks share a `Flags` field (antenna index in bits 0–2)
//! followed by opaque payload bytes tied to the paired `Meas3Ranges` epoch.

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

use super::block_ids;
use super::SbfBlockParse;

// ============================================================================
// Meas3Ranges (4109)
// ============================================================================

/// Meas3Ranges block — code, phase, and CN0 (packed satellite data).
#[derive(Debug, Clone)]
pub struct Meas3RangesBlock {
    tow_ms: u32,
    wnc: u16,
    pub common_flags: u8,
    pub cum_clk_jumps: u8,
    pub constellations: u16,
    pub misc: u8,
    pub reserved: u8,
    /// Raw `Data` field: packed [`M3SatData`](https://github.com/PointOneNav/polaris/blob/master/third_party/septentrio/sbfdef.h) bytes.
    pub data: Vec<u8>,
}

impl Meas3RangesBlock {
    pub fn tow_seconds(&self) -> f64 {
        self.tow_ms as f64 * 0.001
    }

    pub fn tow_ms(&self) -> u32 {
        self.tow_ms
    }

    pub fn wnc(&self) -> u16 {
        self.wnc
    }

    /// Antenna index from `Misc` bits 0–2 (Septentrio convention).
    pub fn antenna_id(&self) -> u8 {
        self.misc & 0x07
    }

    pub fn reference_epoch_interval_ms(&self) -> u32 {
        match self.misc >> 4 {
            0 => 1,
            1 => 500,
            2 => 1000,
            3 => 2000,
            4 => 5000,
            5 => 10_000,
            6 => 15_000,
            7 => 30_000,
            8 => 60_000,
            9 => 120_000,
            _ => 1,
        }
    }

    pub fn is_reference_epoch(&self) -> bool {
        self.tow_ms.checked_rem(self.reference_epoch_interval_ms()) == Some(0)
    }

    pub fn reference_epoch_contains_pr_rate(&self) -> bool {
        (self.misc & 0x08) != 0
    }

    pub fn has_scrambled_measurements(&self) -> bool {
        (self.common_flags & 0x80) != 0
    }
}

impl SbfBlockParse for Meas3RangesBlock {
    const BLOCK_ID: u16 = block_ids::MEAS3_RANGES;

    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
        let full_len = header.length as usize;
        if data.len() < full_len.saturating_sub(2) {
            return Err(SbfError::IncompleteBlock {
                needed: full_len,
                have: data.len() + 2,
            });
        }
        if data.len() < 18 {
            return Err(SbfError::ParseError("Meas3Ranges too short".into()));
        }

        let common_flags = data[12];
        let cum_clk_jumps = data[13];
        let constellations = u16::from_le_bytes([data[14], data[15]]);
        let misc = data[16];
        let reserved = data[17];
        let payload = data[18..].to_vec();

        Ok(Self {
            tow_ms: header.tow_ms,
            wnc: header.wnc,
            common_flags,
            cum_clk_jumps,
            constellations,
            misc,
            reserved,
            data: payload,
        })
    }
}

// ============================================================================
// Meas3 extension blocks (4110–4113): Flags + payload
// ============================================================================

macro_rules! impl_meas3_ext {
    (
        $name:ident,
        $block_id:expr,
        $doc:literal
    ) => {
        #[doc = $doc]
        #[derive(Debug, Clone)]
        pub struct $name {
            tow_ms: u32,
            wnc: u16,
            pub flags: u8,
            pub data: Vec<u8>,
        }

        impl $name {
            pub fn tow_seconds(&self) -> f64 {
                self.tow_ms as f64 * 0.001
            }

            pub fn tow_ms(&self) -> u32 {
                self.tow_ms
            }

            pub fn wnc(&self) -> u16 {
                self.wnc
            }

            /// Antenna index from `Flags` bits 0–2.
            pub fn antenna_id(&self) -> u8 {
                (self.flags & 0x07) as u8
            }
        }

        impl SbfBlockParse for $name {
            const BLOCK_ID: u16 = $block_id;

            fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self> {
                let full_len = header.length as usize;
                if data.len() < full_len.saturating_sub(2) {
                    return Err(SbfError::IncompleteBlock {
                        needed: full_len,
                        have: data.len() + 2,
                    });
                }
                if data.len() < 13 {
                    return Err(SbfError::ParseError(
                        concat!(stringify!($name), " too short").into(),
                    ));
                }

                let flags = data[12];
                let payload = data[13..].to_vec();

                Ok(Self {
                    tow_ms: header.tow_ms,
                    wnc: header.wnc,
                    flags,
                    data: payload,
                })
            }
        }
    };
}

impl_meas3_ext!(
    Meas3Cn0HiResBlock,
    block_ids::MEAS3_CN0_HI_RES,
    "Meas3CN0HiRes — fractional C/N0 extension (paired with Meas3Ranges)."
);

impl_meas3_ext!(
    Meas3DopplerBlock,
    block_ids::MEAS3_DOPPLER,
    "Meas3Doppler — Doppler extension (paired with Meas3Ranges)."
);

impl_meas3_ext!(
    Meas3PpBlock,
    block_ids::MEAS3_PP,
    "Meas3PP — post-processing flags extension (paired with Meas3Ranges)."
);

impl_meas3_ext!(
    Meas3MpBlock,
    block_ids::MEAS3_MP,
    "Meas3MP — multipath correction extension (paired with Meas3Ranges)."
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::SbfBlock;

    /// Full SBF block: `[sync][CRC][ID][Len][TOW][WNc][body...]`. `body` starts at index 14.
    fn build_block(block_id: u16, block_rev: u8, body: &[u8]) -> Vec<u8> {
        let block_data_len = 12 + body.len();
        let mut total_len = (2 + block_data_len) as u16;
        while (total_len as usize & 0x03) != 0 {
            total_len += 1;
        }
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;

        let id_rev = block_id | ((block_rev as u16 & 0x07) << 13);
        data[4..6].copy_from_slice(&id_rev.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&123_456u32.to_le_bytes());
        data[12..14].copy_from_slice(&2150u16.to_le_bytes());
        data[14..14 + body.len()].copy_from_slice(body);
        data
    }

    #[test]
    fn meas3_ranges_parses_header_and_payload() {
        // 6-byte Meas3 fixed header + 4-byte Data so block length stays 4-byte aligned without extra padding bytes.
        let body = [
            0u8, 0, 0, 0, // CommonFlags, CumClkJumps, Constellations u16
            0x03, 0xAA, // misc, reserved
            0x01, 0x02, 0x03, 0x04, // payload
        ];
        let raw = build_block(block_ids::MEAS3_RANGES, 0, &body);

        let (block, consumed) = SbfBlock::parse(&raw).unwrap();
        assert_eq!(consumed, raw.len());
        match block {
            SbfBlock::Meas3Ranges(m) => {
                assert_eq!(m.tow_ms(), 123_456);
                assert_eq!(m.wnc(), 2150);
                assert_eq!(m.common_flags, 0);
                assert_eq!(m.cum_clk_jumps, 0);
                assert_eq!(m.constellations, 0);
                assert_eq!(m.misc, 0x03);
                assert_eq!(m.reserved, 0xAA);
                assert_eq!(m.antenna_id(), 0x03);
                assert_eq!(m.data, vec![0x01, 0x02, 0x03, 0x04]);
            }
            other => panic!("expected Meas3Ranges, got {:?}", other),
        }
    }

    #[test]
    fn meas3_cn0_hi_res_parses_flags_and_bytes() {
        // Use a 5-byte payload so the block stays 4-byte aligned without trailing SBF padding.
        let mut ext = vec![0u8; 1 + 5];
        ext[0] = 0x05;
        ext[1..].copy_from_slice(&[0xAAu8, 10, 20, 30, 40]);
        let raw = build_block(block_ids::MEAS3_CN0_HI_RES, 1, &ext);

        let (block, _) = SbfBlock::parse(&raw).unwrap();
        match block {
            SbfBlock::Meas3Cn0HiRes(m) => {
                assert_eq!(m.flags, 0x05);
                assert_eq!(m.antenna_id(), 0x05);
                assert_eq!(m.data, vec![0xAA, 10, 20, 30, 40]);
            }
            other => panic!("expected Meas3Cn0HiRes, got {:?}", other),
        }
    }

    #[test]
    fn meas3_doppler_pp_mp_parse() {
        for (id, label) in [
            (block_ids::MEAS3_DOPPLER, "doppler"),
            (block_ids::MEAS3_PP, "pp"),
            (block_ids::MEAS3_MP, "mp"),
        ] {
            // Use a 5-byte payload so the block stays 4-byte aligned without trailing SBF padding.
            let mut ext = vec![0u8; 1 + 5];
            ext[0] = 0x02;
            ext[1..].copy_from_slice(&[0xCCu8, 1, 2, 3, 4]);
            let raw = build_block(id, 0, &ext);

            let (block, _) = SbfBlock::parse(&raw).unwrap();
            match (label, block) {
                ("doppler", SbfBlock::Meas3Doppler(m)) => {
                    assert_eq!(m.antenna_id(), 2);
                    assert_eq!(m.data, vec![0xCC, 1, 2, 3, 4]);
                }
                ("pp", SbfBlock::Meas3Pp(m)) => {
                    assert_eq!(m.data, vec![0xCC, 1, 2, 3, 4]);
                }
                ("mp", SbfBlock::Meas3Mp(m)) => {
                    assert_eq!(m.data, vec![0xCC, 1, 2, 3, 4]);
                }
                (_, other) => panic!("{}: unexpected {:?}", label, other),
            }
        }
    }
}
