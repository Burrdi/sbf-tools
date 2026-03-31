//! Common SBF types for GNSS data representation

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// GNSS constellation identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Constellation {
    GPS,
    GLONASS,
    Galileo,
    BeiDou,
    QZSS,
    SBAS,
    NavIC,
    Unknown(u8),
}

impl Constellation {
    /// Convert GNSS ID (from SBF GNSSId field) to Constellation
    ///
    /// Per SBF Reference Guide:
    /// - 0: GPS
    /// - 1: SBAS
    /// - 2: Galileo
    /// - 3: BeiDou
    /// - 4: IMES (not supported)
    /// - 5: QZSS
    /// - 6: GLONASS
    /// - 7: NavIC/IRNSS
    pub fn from_gnss_id(id: u8) -> Self {
        match id {
            0 => Constellation::GPS,
            1 => Constellation::SBAS,
            2 => Constellation::Galileo,
            3 => Constellation::BeiDou,
            5 => Constellation::QZSS,
            6 => Constellation::GLONASS,
            7 => Constellation::NavIC,
            _ => Constellation::Unknown(id),
        }
    }

    /// Short name for the constellation
    pub fn short_name(&self) -> &'static str {
        match self {
            Constellation::GPS => "GPS",
            Constellation::GLONASS => "GLO",
            Constellation::Galileo => "GAL",
            Constellation::BeiDou => "BDS",
            Constellation::QZSS => "QZS",
            Constellation::SBAS => "SBA",
            Constellation::NavIC => "NAV",
            Constellation::Unknown(_) => "UNK",
        }
    }

    /// Single character prefix for satellite IDs
    pub fn prefix(&self) -> char {
        match self {
            Constellation::GPS => 'G',
            Constellation::GLONASS => 'R',
            Constellation::Galileo => 'E',
            Constellation::BeiDou => 'C',
            Constellation::QZSS => 'J',
            Constellation::SBAS => 'S',
            Constellation::NavIC => 'I',
            Constellation::Unknown(_) => 'X',
        }
    }
}

impl std::fmt::Display for Constellation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.short_name())
    }
}

/// Satellite identifier (constellation + PRN)
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SatelliteId {
    pub constellation: Constellation,
    pub prn: u8,
}

impl SatelliteId {
    /// Create a new satellite ID
    pub fn new(constellation: Constellation, prn: u8) -> Self {
        Self { constellation, prn }
    }

    /// Decode SVID into constellation + PRN per SBF spec.
    ///
    /// SVID ranges per SBF Reference Guide:
    /// - 1-37: GPS
    /// - 38-61: GLONASS (slot number = SVID - 37)
    /// - 62: GLONASS (slot number 0)
    /// - 63-68: GLONASS (slot number = SVID - 38)
    /// - 71-106: Galileo (PRN = SVID - 70)
    /// - 107-119: L-Band (MSS) satellites
    /// - 120-140: SBAS (PRN = SVID - 100)
    /// - 141-180: BeiDou (PRN = SVID - 140)
    /// - 181-190: QZSS (PRN = SVID - 180)
    /// - 191-197: NavIC/IRNSS (PRN = SVID - 190)
    /// - 198-215: SBAS (PRN = SVID - 157)
    /// - 216-222: NavIC/IRNSS (PRN = SVID - 208)
    /// - 223-245: BeiDou (PRN = SVID - 182)
    pub fn from_svid(svid: u8) -> Option<Self> {
        if svid == 0 {
            return None;
        }

        let (constellation, prn) = match svid {
            1..=37 => (Constellation::GPS, svid),
            38..=61 => (Constellation::GLONASS, svid - 37),
            62 => (Constellation::GLONASS, 0),
            63..=68 => (Constellation::GLONASS, svid - 38),
            71..=106 => (Constellation::Galileo, svid - 70),
            120..=140 => (Constellation::SBAS, svid - 100),
            141..=180 => (Constellation::BeiDou, svid - 140),
            181..=190 => (Constellation::QZSS, svid - 180),
            191..=197 => (Constellation::NavIC, svid - 190),
            198..=215 => (Constellation::SBAS, svid - 157),
            216..=222 => (Constellation::NavIC, svid - 208),
            223..=245 => (Constellation::BeiDou, svid - 182),
            _ => (Constellation::Unknown(svid), svid),
        };

        Some(Self { constellation, prn })
    }

    /// Convert back to SVID
    pub fn to_svid(&self) -> u8 {
        match self.constellation {
            Constellation::GPS => self.prn,
            Constellation::GLONASS => {
                if self.prn == 0 {
                    62
                } else if self.prn <= 24 {
                    self.prn + 37
                } else {
                    self.prn + 38
                }
            }
            Constellation::Galileo => self.prn + 70,
            Constellation::SBAS => {
                if self.prn <= 40 {
                    self.prn + 100
                } else {
                    self.prn + 157
                }
            }
            Constellation::BeiDou => {
                if self.prn <= 40 {
                    self.prn + 140
                } else {
                    self.prn + 182
                }
            }
            Constellation::QZSS => self.prn + 180,
            Constellation::NavIC => {
                if self.prn <= 7 {
                    self.prn + 190
                } else {
                    self.prn + 208
                }
            }
            Constellation::Unknown(svid) => svid,
        }
    }

    /// Generate a short key like "G01", "E05", "R24"
    pub fn key(&self) -> String {
        format!("{}{:02}", self.constellation.prefix(), self.prn)
    }
}

impl std::fmt::Display for SatelliteId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.key())
    }
}

/// Signal type for tracking
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SignalType {
    // GPS signals
    L1CA,
    L1C,
    L1PY,
    L2C,
    L2P,
    L2PY,
    L5,
    // Galileo signals
    E1,
    E5a,
    E5b,
    E5AltBOC,
    E6,
    // GLONASS signals
    G1CA,
    G1P,
    G2CA,
    G2P,
    G3,
    // BeiDou signals
    B1I,
    B1C,
    B2I,
    B2a,
    B2b,
    B3I,
    // QZSS signals
    QZSSL1CA,
    QZSSL2C,
    QZSSL5,
    QZSSL1C,
    QZSSL1S,
    QZSSL1CB,
    QZSSL5S,
    QZSSL6,
    // SBAS signals
    SBASL1,
    SBASL5,
    // MSS signals
    LBand,
    // NavIC signals
    NavICL5,
    NavICL1,
    // Generic/fallback
    Other(u8),
}

impl SignalType {
    /// Decode signal type from global SBF signal number (per SBF Reference Guide section 4.1.10)
    pub fn from_signal_number(sig_num: u8) -> Self {
        match sig_num {
            // GPS signals (0-5)
            0 => SignalType::L1CA,
            1 => SignalType::L1PY,
            2 => SignalType::L2P,
            3 => SignalType::L2C,
            4 => SignalType::L5,
            5 => SignalType::L1C,
            // QZSS signals (6-7)
            6 => SignalType::QZSSL1CA,
            7 => SignalType::QZSSL2C,
            // GLONASS signals (8-12)
            8 => SignalType::G1CA,
            9 => SignalType::G1P,
            10 => SignalType::G2P,
            11 => SignalType::G2CA,
            12 => SignalType::G3,
            // BeiDou signals (13-14)
            13 => SignalType::B1C,
            14 => SignalType::B2a,
            // NavIC (15)
            15 => SignalType::NavICL5,
            // Galileo signals (17, 19-22)
            17 => SignalType::E1,
            19 => SignalType::E6,
            20 => SignalType::E5a,
            21 => SignalType::E5b,
            22 => SignalType::E5AltBOC,
            // MSS signals (23)
            23 => SignalType::LBand,
            // SBAS signals (24-25)
            24 => SignalType::SBASL1,
            25 => SignalType::SBASL5,
            // QZSS signals (26-27)
            26 => SignalType::QZSSL5,
            27 => SignalType::QZSSL6,
            // BeiDou signals (28-30)
            28 => SignalType::B1I,
            29 => SignalType::B2I,
            30 => SignalType::B3I,
            // QZSS signals (32-33)
            32 => SignalType::QZSSL1C,
            33 => SignalType::QZSSL1S,
            // BeiDou signal (34)
            34 => SignalType::B2b,
            // NavIC (37)
            37 => SignalType::NavICL1,
            // QZSS signals (38-39)
            38 => SignalType::QZSSL1CB,
            39 => SignalType::QZSSL5S,
            _ => SignalType::Other(sig_num),
        }
    }

    /// Decode signal type from constellation-specific signal number
    pub fn from_signal_number_and_constellation(
        sig_num: u8,
        constellation: &Constellation,
    ) -> Self {
        match constellation {
            Constellation::GPS => match sig_num {
                0 => SignalType::L1CA,
                1 => SignalType::L1PY,
                2 => SignalType::L2P,
                3 => SignalType::L2C,
                4 => SignalType::L5,
                5 => SignalType::L1C,
                _ => SignalType::Other(sig_num),
            },
            Constellation::GLONASS => match sig_num {
                0 => SignalType::G1CA,
                1 => SignalType::G1P,
                2 => SignalType::G2P,
                3 => SignalType::G2CA,
                4 => SignalType::G3,
                _ => SignalType::Other(sig_num),
            },
            Constellation::Galileo => match sig_num {
                0 => SignalType::E1,
                1 => SignalType::E5a,
                2 => SignalType::E5b,
                3 => SignalType::E5AltBOC,
                4 => SignalType::E6,
                _ => SignalType::Other(sig_num),
            },
            Constellation::BeiDou => match sig_num {
                0 => SignalType::B1I,
                1 => SignalType::B2I,
                2 => SignalType::B3I,
                3 => SignalType::B1C,
                4 => SignalType::B2a,
                5 => SignalType::B2b,
                _ => SignalType::Other(sig_num),
            },
            Constellation::QZSS => match sig_num {
                0 => SignalType::QZSSL1CA,
                1 => SignalType::QZSSL1S,
                2 => SignalType::L2P,
                3 => SignalType::QZSSL2C,
                4 => SignalType::QZSSL5,
                5 => SignalType::QZSSL1C,
                _ => SignalType::Other(sig_num),
            },
            Constellation::SBAS => match sig_num {
                0 => SignalType::SBASL1,
                1 => SignalType::SBASL5,
                _ => SignalType::Other(sig_num),
            },
            Constellation::NavIC => match sig_num {
                0 => SignalType::NavICL5,
                1 => SignalType::NavICL1,
                _ => SignalType::Other(sig_num),
            },
            Constellation::Unknown(_) => SignalType::Other(sig_num),
        }
    }

    /// Short name suitable for display
    pub fn name(&self) -> &'static str {
        match self {
            SignalType::L1CA => "L1CA",
            SignalType::L1C => "L1C",
            SignalType::L1PY => "L1PY",
            SignalType::L2C => "L2C",
            SignalType::L2P => "L2P",
            SignalType::L2PY => "L2PY",
            SignalType::L5 => "L5",
            SignalType::E1 => "E1",
            SignalType::E5a => "E5a",
            SignalType::E5b => "E5b",
            SignalType::E5AltBOC => "E5",
            SignalType::E6 => "E6",
            SignalType::G1CA => "G1",
            SignalType::G1P => "G1P",
            SignalType::G2CA => "G2",
            SignalType::G2P => "G2P",
            SignalType::G3 => "G3",
            SignalType::B1I => "B1I",
            SignalType::B1C => "B1C",
            SignalType::B2I => "B2I",
            SignalType::B2a => "B2a",
            SignalType::B2b => "B2b",
            SignalType::B3I => "B3I",
            SignalType::QZSSL1CA => "L1",
            SignalType::QZSSL2C => "L2C",
            SignalType::QZSSL5 => "L5",
            SignalType::QZSSL1C => "L1C",
            SignalType::QZSSL1S => "L1S",
            SignalType::QZSSL1CB => "L1CB",
            SignalType::QZSSL5S => "L5S",
            SignalType::QZSSL6 => "L6",
            SignalType::SBASL1 => "L1",
            SignalType::SBASL5 => "L5",
            SignalType::LBand => "LBand",
            SignalType::NavICL5 => "L5",
            SignalType::NavICL1 => "L1",
            SignalType::Other(_) => "?",
        }
    }

    /// Get the frequency band category
    pub fn band(&self) -> &'static str {
        match self {
            SignalType::L1CA | SignalType::L1C | SignalType::L1PY => "L1",
            SignalType::L2C | SignalType::L2P | SignalType::L2PY => "L2",
            SignalType::L5 => "L5",
            SignalType::E1 => "L1",
            SignalType::E5a => "L5",
            SignalType::E5b => "E5b",
            SignalType::E5AltBOC => "E5",
            SignalType::E6 => "E6",
            SignalType::G1CA | SignalType::G1P => "G1",
            SignalType::G2CA | SignalType::G2P => "G2",
            SignalType::G3 => "G3",
            SignalType::B1I | SignalType::B1C => "B1",
            SignalType::B2I | SignalType::B2a | SignalType::B2b => "B2",
            SignalType::B3I => "B3",
            SignalType::QZSSL1CA
            | SignalType::QZSSL1C
            | SignalType::QZSSL1S
            | SignalType::QZSSL1CB => "L1",
            SignalType::QZSSL2C => "L2",
            SignalType::QZSSL5 | SignalType::QZSSL5S => "L5",
            SignalType::QZSSL6 => "L6",
            SignalType::SBASL1 => "L1",
            SignalType::SBASL5 => "L5",
            SignalType::LBand => "LBand",
            SignalType::NavICL5 => "L5",
            SignalType::NavICL1 => "L1",
            SignalType::Other(_) => "?",
        }
    }
}

impl std::fmt::Display for SignalType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// PVT solution mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum PvtMode {
    NoFix,
    StandAlone,
    Dgps,
    Fixed,
    FixedPpp,
    Float,
    FloatPpp,
    Sbas,
    RtkFixed,
    RtkFloat,
    MovingBaseRtkFixed,
    MovingBaseRtkFloat,
    Ppp,
    Unknown(u8),
}

impl PvtMode {
    /// Decode PVT mode from mode byte (bits 0-3)
    pub fn from_mode_byte(mode: u8) -> Self {
        match mode & 0x0F {
            0 => PvtMode::NoFix,
            1 => PvtMode::StandAlone,
            2 => PvtMode::Dgps,
            3 => PvtMode::Fixed,
            4 => PvtMode::FixedPpp,
            5 => PvtMode::Float,
            6 => PvtMode::FloatPpp,
            7 => PvtMode::Sbas,
            8 => PvtMode::RtkFixed,
            9 => PvtMode::RtkFloat,
            10 => PvtMode::MovingBaseRtkFixed,
            11 => PvtMode::MovingBaseRtkFloat,
            12 => PvtMode::Ppp,
            n => PvtMode::Unknown(n),
        }
    }

    /// Check if this is a valid fix (2D or 3D)
    pub fn has_fix(&self) -> bool {
        !matches!(self, PvtMode::NoFix | PvtMode::Unknown(_))
    }

    /// Check if this is an RTK solution
    pub fn is_rtk(&self) -> bool {
        matches!(
            self,
            PvtMode::RtkFixed
                | PvtMode::RtkFloat
                | PvtMode::MovingBaseRtkFixed
                | PvtMode::MovingBaseRtkFloat
        )
    }

    /// Check if this is a PPP solution
    pub fn is_ppp(&self) -> bool {
        matches!(self, PvtMode::Ppp | PvtMode::FixedPpp | PvtMode::FloatPpp)
    }
}

impl std::fmt::Display for PvtMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            PvtMode::NoFix => "No Fix",
            PvtMode::StandAlone => "Stand-Alone",
            PvtMode::Dgps => "DGPS",
            PvtMode::Fixed => "Fixed",
            PvtMode::FixedPpp => "Fixed PPP",
            PvtMode::Float => "Float",
            PvtMode::FloatPpp => "Float PPP",
            PvtMode::Sbas => "SBAS",
            PvtMode::RtkFixed => "RTK Fixed",
            PvtMode::RtkFloat => "RTK Float",
            PvtMode::MovingBaseRtkFixed => "Moving Base RTK Fixed",
            PvtMode::MovingBaseRtkFloat => "Moving Base RTK Float",
            PvtMode::Ppp => "PPP",
            PvtMode::Unknown(_) => "Unknown",
        };
        write!(f, "{}", s)
    }
}

/// PVT error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum PvtError {
    None,
    NotEnoughMeasurements,
    NotEnoughEphemerides,
    DopTooLarge,
    ResidualsTooLarge,
    NoCovergence,
    NotEnoughMeasurementsAfterRejection,
    PositionProhibited,
    NotEnoughDiffCorr,
    BaseStationCoordinatesUnavailable,
    Unknown(u8),
}

impl PvtError {
    /// Decode PVT error from error byte
    pub fn from_error_byte(error: u8) -> Self {
        match error {
            0 => PvtError::None,
            1 => PvtError::NotEnoughMeasurements,
            2 => PvtError::NotEnoughEphemerides,
            3 => PvtError::DopTooLarge,
            4 => PvtError::ResidualsTooLarge,
            5 => PvtError::NoCovergence,
            6 => PvtError::NotEnoughMeasurementsAfterRejection,
            7 => PvtError::PositionProhibited,
            8 => PvtError::NotEnoughDiffCorr,
            9 => PvtError::BaseStationCoordinatesUnavailable,
            n => PvtError::Unknown(n),
        }
    }

    /// Check if there's no error
    pub fn is_ok(&self) -> bool {
        matches!(self, PvtError::None)
    }
}

impl std::fmt::Display for PvtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            PvtError::None => "None",
            PvtError::NotEnoughMeasurements => "Not enough measurements",
            PvtError::NotEnoughEphemerides => "Not enough ephemerides",
            PvtError::DopTooLarge => "DOP too large",
            PvtError::ResidualsTooLarge => "Residuals too large",
            PvtError::NoCovergence => "No convergence",
            PvtError::NotEnoughMeasurementsAfterRejection => {
                "Not enough measurements after rejection"
            }
            PvtError::PositionProhibited => "Position prohibited",
            PvtError::NotEnoughDiffCorr => "Not enough differential corrections",
            PvtError::BaseStationCoordinatesUnavailable => "Base station coordinates unavailable",
            PvtError::Unknown(_) => "Unknown",
        };
        write!(f, "{}", s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_satellite_id_from_svid() {
        // GPS
        assert_eq!(
            SatelliteId::from_svid(1),
            Some(SatelliteId::new(Constellation::GPS, 1))
        );
        assert_eq!(
            SatelliteId::from_svid(32),
            Some(SatelliteId::new(Constellation::GPS, 32))
        );

        // GLONASS
        assert_eq!(
            SatelliteId::from_svid(38),
            Some(SatelliteId::new(Constellation::GLONASS, 1))
        );
        assert_eq!(
            SatelliteId::from_svid(61),
            Some(SatelliteId::new(Constellation::GLONASS, 24))
        );

        // Galileo
        assert_eq!(
            SatelliteId::from_svid(71),
            Some(SatelliteId::new(Constellation::Galileo, 1))
        );
        assert_eq!(
            SatelliteId::from_svid(106),
            Some(SatelliteId::new(Constellation::Galileo, 36))
        );

        // BeiDou
        assert_eq!(
            SatelliteId::from_svid(141),
            Some(SatelliteId::new(Constellation::BeiDou, 1))
        );

        // QZSS
        assert_eq!(
            SatelliteId::from_svid(181),
            Some(SatelliteId::new(Constellation::QZSS, 1))
        );

        // NavIC/IRNSS (extended range)
        assert_eq!(
            SatelliteId::from_svid(216),
            Some(SatelliteId::new(Constellation::NavIC, 8))
        );
        assert_eq!(
            SatelliteId::from_svid(222),
            Some(SatelliteId::new(Constellation::NavIC, 14))
        );

        // SBAS
        assert_eq!(
            SatelliteId::from_svid(120),
            Some(SatelliteId::new(Constellation::SBAS, 20))
        );

        // Invalid
        assert_eq!(SatelliteId::from_svid(0), None);
    }

    #[test]
    fn test_satellite_id_key() {
        let gps = SatelliteId::new(Constellation::GPS, 1);
        assert_eq!(gps.key(), "G01");

        let gal = SatelliteId::new(Constellation::Galileo, 12);
        assert_eq!(gal.key(), "E12");

        let glo = SatelliteId::new(Constellation::GLONASS, 5);
        assert_eq!(glo.key(), "R05");
    }

    #[test]
    fn test_satellite_id_to_svid_navic_ranges() {
        let navic_legacy = SatelliteId::new(Constellation::NavIC, 7);
        assert_eq!(navic_legacy.to_svid(), 197);

        let navic_extended = SatelliteId::new(Constellation::NavIC, 8);
        assert_eq!(navic_extended.to_svid(), 216);
    }

    #[test]
    fn test_signal_type() {
        assert_eq!(SignalType::from_signal_number(0), SignalType::L1CA);
        assert_eq!(SignalType::from_signal_number(17), SignalType::E1);
        assert_eq!(SignalType::from_signal_number(28), SignalType::B1I);
        assert_eq!(SignalType::from_signal_number(23), SignalType::LBand);
        assert_eq!(SignalType::from_signal_number(38), SignalType::QZSSL1CB);
        assert_eq!(SignalType::from_signal_number(39), SignalType::QZSSL5S);
    }

    #[test]
    fn test_pvt_mode() {
        assert!(!PvtMode::NoFix.has_fix());
        assert!(PvtMode::StandAlone.has_fix());
        assert!(PvtMode::RtkFixed.is_rtk());
        assert!(PvtMode::Ppp.is_ppp());
    }
}
