//! SBF block definitions and parsing
//!
//! This module contains all supported SBF block types and the main `SbfBlock` enum.

mod attitude;
pub mod catalog;
mod differential;
mod dnu;
mod extended;
mod external;
mod ins;
mod meas3;
mod meas3_decoder;
mod measurement;
mod navigation;
mod position;
mod sbas;
mod status;
mod time;

pub use attitude::*;
pub use differential::*;
pub use extended::*;
pub use external::*;
pub use ins::*;
pub use meas3::*;
pub use meas3_decoder::*;
pub use measurement::*;
pub use navigation::*;
pub use position::*;
pub use sbas::*;
pub use status::*;
pub use time::*;

pub use catalog::fallback_name;

use crate::error::{SbfError, SbfResult};
use crate::header::SbfHeader;

// ============================================================================
// Block IDs
// ============================================================================

/// Block ID constants for commonly used SBF blocks
pub mod block_ids {
    // Measurement blocks
    pub const MEAS_EPOCH: u16 = 4027;
    pub const MEAS_EXTRA: u16 = 4000;
    pub const END_OF_MEAS: u16 = 5922;
    pub const IQ_CORR: u16 = 4046;
    pub const MEAS3_RANGES: u16 = 4109;
    pub const MEAS3_CN0_HI_RES: u16 = 4110;
    pub const MEAS3_DOPPLER: u16 = 4111;
    pub const MEAS3_PP: u16 = 4112;
    pub const MEAS3_MP: u16 = 4113;

    // Position blocks
    pub const PVT_CARTESIAN: u16 = 4006;
    pub const PVT_GEODETIC: u16 = 4007;
    pub const DOP: u16 = 4001;
    pub const DOP_LEGACY: u16 = 5909;
    pub const POS_CART: u16 = 4044;
    pub const PVT_SAT_CARTESIAN: u16 = 4008;
    pub const PVT_RESIDUALS_V2: u16 = 4009;
    pub const RAIM_STATISTICS_V2: u16 = 4011;
    pub const BASE_VECTOR_CART: u16 = 4043;
    pub const BASE_VECTOR_GEOD: u16 = 4028;
    pub const POS_COV_CARTESIAN: u16 = 5905;
    pub const POS_COV_GEODETIC: u16 = 5906;
    pub const VEL_COV_CARTESIAN: u16 = 5907;
    pub const VEL_COV_GEODETIC: u16 = 5908;
    pub const GEO_CORRECTIONS: u16 = 5935;
    pub const BASE_STATION: u16 = 5949;
    pub const DIFF_CORR_IN: u16 = 5919;
    pub const END_OF_PVT: u16 = 5921;
    pub const PVT_SUPPORT: u16 = 4076;
    pub const PVT_SUPPORT_A: u16 = 4079;
    pub const FUGRO_DDS: u16 = 4211;
    pub const POS_LOCAL: u16 = 4052;
    pub const POS_PROJECTED: u16 = 4094;

    // INS blocks
    pub const INT_PV_CART: u16 = 4060;
    pub const INT_PV_GEOD: u16 = 4061;
    pub const INT_PVA_AGEOD: u16 = 4045;
    pub const INT_POS_COV_CART: u16 = 4062;
    pub const INT_VEL_COV_CART: u16 = 4063;
    pub const INT_POS_COV_GEOD: u16 = 4064;
    pub const INT_VEL_COV_GEOD: u16 = 4065;
    pub const INT_ATT_EULER: u16 = 4070;
    pub const INT_ATT_COV_EULER: u16 = 4072;

    // Status blocks

    // Attitude blocks
    pub const ATT_EULER: u16 = 5938;
    pub const ATT_COV_EULER: u16 = 5939;
    pub const AUX_ANT_POSITIONS: u16 = 5942;
    pub const END_OF_ATT: u16 = 5943;

    // Navigation message blocks
    pub const GPS_NAV: u16 = 5891;
    pub const GPS_ALM: u16 = 5892;
    pub const GPS_ION: u16 = 5893;
    pub const GPS_UTC: u16 = 5894;
    pub const GPS_CNAV: u16 = 4042;
    pub const GPS_RAW_CA: u16 = 4017;
    pub const GPS_RAW_L2C: u16 = 4018;
    pub const GPS_RAW_L5: u16 = 4019;
    pub const GEO_RAW_L1: u16 = 4020;
    pub const GAL_RAW_FNAV: u16 = 4022;
    pub const GAL_RAW_INAV: u16 = 4023;
    pub const GAL_RAW_CNAV: u16 = 4024;
    pub const GLO_RAW_CA: u16 = 4026;
    pub const CMP_RAW: u16 = 4047;
    pub const QZS_RAW_L1CA: u16 = 4066;
    pub const QZS_RAW_L2C: u16 = 4067;
    pub const QZS_RAW_L5: u16 = 4068;
    pub const GLO_NAV: u16 = 4004;
    pub const GLO_ALM: u16 = 4005;
    pub const GLO_TIME: u16 = 4036;
    pub const GAL_NAV: u16 = 4002;
    pub const GAL_ALM: u16 = 4003;
    pub const GAL_ION: u16 = 4030;
    pub const GAL_UTC: u16 = 4031;
    pub const GAL_GST_GPS: u16 = 4032;
    pub const GAL_SAR_RLM: u16 = 4034;
    pub const BDS_ION: u16 = 4120;
    pub const BDS_NAV: u16 = 4081;
    pub const BDS_ALM: u16 = 4119;
    pub const BDS_UTC: u16 = 4121;
    pub const BDS_CNAV1: u16 = 4251;
    pub const BDS_CNAV2: u16 = 4252;
    pub const BDS_CNAV3: u16 = 4253;
    pub const QZS_NAV: u16 = 4095;
    pub const QZS_ALM: u16 = 4116;
    pub const GEO_RAW_L5: u16 = 4021;
    pub const BDS_RAW_B1C: u16 = 4218;
    pub const BDS_RAW_B2A: u16 = 4219;
    /// BeiDou B2b navigation frame
    pub const BDS_RAW_B2B: u16 = 4242;
    /// Galileo OSNMA authentication status
    pub const GAL_AUTH_STATUS: u16 = 4245;
    pub const NAVIC_RAW: u16 = 4093;
    pub const GEO_IONO_DELAY: u16 = 5933;
    pub const GEO_MT00: u16 = 5925;
    pub const GEO_PRN_MASK: u16 = 5926;
    pub const GEO_FAST_CORR: u16 = 5927;
    pub const GEO_FAST_CORR_DEGR: u16 = 5929;
    pub const GEO_DEGR_FACTORS: u16 = 5930;
    pub const GEO_SERVICE_LEVEL: u16 = 5917;
    pub const GEO_NAV: u16 = 5896;
    pub const GEO_INTEGRITY: u16 = 5928;
    pub const GEO_ALM: u16 = 5897;
    pub const GEO_NETWORK_TIME: u16 = 5918;
    pub const GEO_IGP_MASK: u16 = 5931;
    pub const GEO_LONG_TERM_CORR: u16 = 5932;
    pub const GEO_CLOCK_EPH_COV_MATRIX: u16 = 5934;

    // Status blocks
    // TODO(spec-audit): v4.15.1 documents ReceiverStatus as 4014.
    // Legacy ReceiverStatus IDs (for example 5913 in older references) remain unverified.
    pub const RECEIVER_STATUS: u16 = 4014;
    pub const TRACKING_STATUS: u16 = 5912;
    pub const CHANNEL_STATUS: u16 = 4013;
    pub const SAT_VISIBILITY: u16 = 4012;
    pub const QUALITY_IND: u16 = 4082;
    pub const INPUT_LINK: u16 = 4090;
    pub const OUTPUT_LINK: u16 = 4091;
    pub const IP_STATUS: u16 = 4058;
    pub const LBAND_TRACKER_STATUS: u16 = 4201;
    pub const LBAND_BEAMS: u16 = 4204;
    pub const EXT_SENSOR_MEAS: u16 = 4050;
    pub const EXT_SENSOR_STATUS: u16 = 4056;
    pub const EXT_SENSOR_SETUP: u16 = 4057;
    pub const COMMANDS: u16 = 4015;
    pub const COMMENT: u16 = 5936;
    pub const RECEIVER_SETUP: u16 = 5902;
    pub const BB_SAMPLES: u16 = 4040;
    pub const ASCII_IN: u16 = 4075;
    pub const RX_MESSAGE: u16 = 4103;
    pub const ENCAPSULATED_OUTPUT: u16 = 4097;
    pub const GIS_ACTION: u16 = 4106;
    pub const GIS_STATUS: u16 = 4107;
    pub const DYN_DNS_STATUS: u16 = 4105;
    pub const DISK_STATUS: u16 = 4059;
    pub const NTRIP_CLIENT_STATUS: u16 = 4053;
    pub const NTRIP_SERVER_STATUS: u16 = 4122;
    pub const RF_STATUS: u16 = 4092;
    pub const P2PP_STATUS: u16 = 4238;
    pub const COSMOS_STATUS: u16 = 4243;
    pub const RTCM_DATUM: u16 = 4049;

    // Time blocks
    pub const RECEIVER_TIME: u16 = 5914;
    pub const PPS_OFFSET: u16 = 5911;
    pub const EXT_EVENT: u16 = 5924;
    pub const EXT_EVENT_PVT_CARTESIAN: u16 = 4037;
    pub const EXT_EVENT_PVT_GEODETIC: u16 = 4038;
    pub const EXT_EVENT_BASE_VECT_GEOD: u16 = 4217;
    pub const EXT_EVENT_ATT_EULER: u16 = 4237;
}

// ============================================================================
// Block Name Lookup
// ============================================================================

/// Get a human-readable name for a block ID
pub fn block_name(id: u16) -> &'static str {
    match id {
        block_ids::MEAS_EPOCH => "MeasEpoch",
        block_ids::MEAS_EXTRA => "MeasExtra",
        block_ids::IQ_CORR => "IQCorr",
        block_ids::END_OF_MEAS => "EndOfMeas",
        block_ids::MEAS3_RANGES => "Meas3Ranges",
        block_ids::MEAS3_CN0_HI_RES => "Meas3CN0HiRes",
        block_ids::MEAS3_DOPPLER => "Meas3Doppler",
        block_ids::MEAS3_PP => "Meas3PP",
        block_ids::MEAS3_MP => "Meas3MP",
        block_ids::PVT_CARTESIAN => "PVTCartesian",
        block_ids::PVT_GEODETIC => "PVTGeodetic",
        block_ids::DOP => "DOP",
        block_ids::DOP_LEGACY => "DOP",
        block_ids::POS_CART => "PosCart",
        block_ids::PVT_SAT_CARTESIAN => "PVTSatCartesian",
        block_ids::PVT_RESIDUALS_V2 => "PVTResiduals_v2",
        block_ids::RAIM_STATISTICS_V2 => "RAIMStatistics_v2",
        block_ids::BASE_VECTOR_CART => "BaseVectorCart",
        block_ids::BASE_VECTOR_GEOD => "BaseVectorGeod",
        block_ids::POS_COV_CARTESIAN => "PosCovCartesian",
        block_ids::POS_COV_GEODETIC => "PosCovGeodetic",
        block_ids::VEL_COV_CARTESIAN => "VelCovCartesian",
        block_ids::VEL_COV_GEODETIC => "VelCovGeodetic",
        block_ids::GEO_CORRECTIONS => "GEOCorrections",
        block_ids::BASE_STATION => "BaseStation",
        block_ids::DIFF_CORR_IN => "DiffCorrIn",
        block_ids::END_OF_PVT => "EndOfPVT",
        block_ids::PVT_SUPPORT => "PVTSupport",
        block_ids::PVT_SUPPORT_A => "PVTSupportA",
        block_ids::FUGRO_DDS => "FugroDDS",
        block_ids::INT_PV_CART => "IntPVCart",
        block_ids::INT_PV_GEOD => "IntPVGeod",
        block_ids::INT_PVA_AGEOD => "IntPVAAGeod",
        block_ids::INT_ATT_EULER => "IntAttEuler",
        block_ids::INT_POS_COV_CART => "IntPosCovCart",
        block_ids::INT_VEL_COV_CART => "IntVelCovCart",
        block_ids::INT_POS_COV_GEOD => "IntPosCovGeod",
        block_ids::INT_VEL_COV_GEOD => "IntVelCovGeod",
        block_ids::INT_ATT_COV_EULER => "IntAttCovEuler",
        block_ids::IP_STATUS => "IPStatus",
        block_ids::EXT_SENSOR_MEAS => "ExtSensorMeas",
        block_ids::EXT_SENSOR_STATUS => "ExtSensorStatus",
        block_ids::EXT_SENSOR_SETUP => "ExtSensorSetup",
        block_ids::ATT_EULER => "AttEuler",
        block_ids::ATT_COV_EULER => "AttCovEuler",
        block_ids::AUX_ANT_POSITIONS => "AuxAntPositions",
        block_ids::END_OF_ATT => "EndOfAtt",
        block_ids::GPS_NAV => "GPSNav",
        block_ids::GPS_ALM => "GPSAlm",
        block_ids::GPS_ION => "GPSIon",
        block_ids::GPS_UTC => "GPSUtc",
        block_ids::GPS_CNAV => "GPSCNav",
        block_ids::GLO_NAV => "GLONav",
        block_ids::GLO_ALM => "GLOAlm",
        block_ids::GLO_TIME => "GLOTime",
        block_ids::GAL_NAV => "GALNav",
        block_ids::GAL_ALM => "GALAlm",
        block_ids::GAL_ION => "GALIon",
        block_ids::GAL_UTC => "GALUtc",
        block_ids::GAL_GST_GPS => "GALGstGps",
        block_ids::GAL_SAR_RLM => "GALSARRLM",
        block_ids::BDS_ION => "BDSIon",
        block_ids::BDS_CNAV1 => "BDSCNav1",
        block_ids::GEO_IONO_DELAY => "GEOIonoDelay",
        block_ids::GEO_MT00 => "GEOMT00",
        block_ids::GEO_PRN_MASK => "GEOPRNMask",
        block_ids::GEO_FAST_CORR => "GEOFastCorr",
        block_ids::GEO_FAST_CORR_DEGR => "GEOFastCorrDegr",
        block_ids::GEO_DEGR_FACTORS => "GEODegrFactors",
        block_ids::GEO_SERVICE_LEVEL => "GEOServiceLevel",
        block_ids::GEO_NAV => "GEONav",
        block_ids::GEO_INTEGRITY => "GEOIntegrity",
        block_ids::GEO_ALM => "GEOAlm",
        block_ids::GEO_NETWORK_TIME => "GEONetworkTime",
        block_ids::GEO_IGP_MASK => "GEOIGPMask",
        block_ids::GEO_LONG_TERM_CORR => "GEOLongTermCorr",
        block_ids::GEO_CLOCK_EPH_COV_MATRIX => "GEOClockEphCovMatrix",
        block_ids::GPS_RAW_CA => "GPSRawCA",
        block_ids::GPS_RAW_L2C => "GPSRawL2C",
        block_ids::GPS_RAW_L5 => "GPSRawL5",
        block_ids::GAL_RAW_FNAV => "GALRawFNAV",
        block_ids::GAL_RAW_INAV => "GALRawINAV",
        block_ids::GAL_RAW_CNAV => "GALRawCNAV",
        block_ids::GEO_RAW_L1 => "GEORawL1",
        block_ids::GLO_RAW_CA => "GLORawCA",
        block_ids::CMP_RAW => "CMPRaw",
        block_ids::QZS_RAW_L1CA => "QZSRawL1CA",
        block_ids::QZS_RAW_L2C => "QZSRawL2C",
        block_ids::QZS_RAW_L5 => "QZSRawL5",
        block_ids::RECEIVER_STATUS => "ReceiverStatus",
        block_ids::TRACKING_STATUS => "TrackingStatus",
        block_ids::CHANNEL_STATUS => "ChannelStatus",
        block_ids::SAT_VISIBILITY => "SatVisibility",
        block_ids::QUALITY_IND => "QualityInd",
        block_ids::INPUT_LINK => "InputLink",
        block_ids::OUTPUT_LINK => "OutputLink",
        block_ids::LBAND_TRACKER_STATUS => "LBandTrackerStatus",
        block_ids::COMMANDS => "Commands",
        block_ids::COMMENT => "Comment",
        block_ids::RECEIVER_SETUP => "ReceiverSetup",
        block_ids::BB_SAMPLES => "BBSamples",
        block_ids::ASCII_IN => "ASCIIIn",
        block_ids::NTRIP_CLIENT_STATUS => "NTRIPClientStatus",
        block_ids::NTRIP_SERVER_STATUS => "NTRIPServerStatus",
        block_ids::RF_STATUS => "RFStatus",
        block_ids::RECEIVER_TIME => "ReceiverTime",
        block_ids::PPS_OFFSET => "xPPSOffset",
        block_ids::EXT_EVENT => "ExtEvent",
        block_ids::EXT_EVENT_PVT_CARTESIAN => "ExtEventPVTCartesian",
        block_ids::EXT_EVENT_PVT_GEODETIC => "ExtEventPVTGeodetic",
        block_ids::EXT_EVENT_BASE_VECT_GEOD => "ExtEventBaseVectGeod",
        block_ids::EXT_EVENT_ATT_EULER => "ExtEventAttEuler",
        block_ids::POS_LOCAL => "PosLocal",
        block_ids::POS_PROJECTED => "PosProjected",
        block_ids::BDS_NAV => "BDSNav",
        block_ids::BDS_ALM => "BDSAlm",
        block_ids::BDS_UTC => "BDSUtc",
        block_ids::BDS_CNAV2 => "BDSCNav2",
        block_ids::BDS_CNAV3 => "BDSCNav3",
        block_ids::QZS_NAV => "QZSNav",
        block_ids::QZS_ALM => "QZSAlm",
        block_ids::GEO_RAW_L5 => "GEORawL5",
        block_ids::BDS_RAW_B1C => "BDSRawB1C",
        block_ids::BDS_RAW_B2A => "BDSRawB2a",
        block_ids::BDS_RAW_B2B => "BDSRawB2b",
        block_ids::GAL_AUTH_STATUS => "GALAuthStatus",
        block_ids::NAVIC_RAW => "NAVICRaw",
        block_ids::RTCM_DATUM => "RTCMDatum",
        block_ids::LBAND_BEAMS => "LBandBeams",
        block_ids::DYN_DNS_STATUS => "DynDNSStatus",
        block_ids::DISK_STATUS => "DiskStatus",
        block_ids::P2PP_STATUS => "P2PPStatus",
        block_ids::COSMOS_STATUS => "CosmosStatus",
        block_ids::RX_MESSAGE => "RxMessage",
        block_ids::ENCAPSULATED_OUTPUT => "EncapsulatedOutput",
        block_ids::GIS_ACTION => "GISAction",
        block_ids::GIS_STATUS => "GISStatus",
        _ => catalog::fallback_name(id),
    }
}

/// IDs that are known/catalogued but intentionally parsed as opaque payloads.
pub fn is_known_opaque_id(id: u16) -> bool {
    matches!(id, block_ids::FUGRO_DDS)
}

// ============================================================================
// Block Trait
// ============================================================================

/// Trait for SBF block parsing
pub trait SbfBlockParse: Sized {
    /// The block ID for this block type
    const BLOCK_ID: u16;

    /// Parse block from data (starting after sync bytes)
    ///
    /// # Arguments
    /// * `header` - Parsed block header
    /// * `data` - Block data starting from CRC field (after sync bytes)
    ///
    /// # Returns
    /// Parsed block or error
    fn parse(header: &SbfHeader, data: &[u8]) -> SbfResult<Self>;
}

// ============================================================================
// Main Block Enum
// ============================================================================

/// Parsed SBF block variants.
///
/// This enum is [`non_exhaustive`]: new block types may appear in any minor release.
/// Code outside this crate that matches on [`SbfBlock`](Self) should include a wildcard
/// pattern (for example `_ =>` or a catch-all) so it keeps compiling as variants are added.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub enum SbfBlock {
    // Measurement blocks
    MeasEpoch(MeasEpochBlock),
    MeasExtra(MeasExtraBlock),
    EndOfMeas(EndOfMeasBlock),
    Meas3Ranges(Meas3RangesBlock),
    Meas3Cn0HiRes(Meas3Cn0HiResBlock),
    Meas3Doppler(Meas3DopplerBlock),
    Meas3Pp(Meas3PpBlock),
    Meas3Mp(Meas3MpBlock),

    // Position blocks
    PvtGeodetic(PvtGeodeticBlock),
    PvtCartesian(PvtCartesianBlock),
    Dop(DopBlock),
    PosCart(PosCartBlock),
    PvtSatCartesian(PvtSatCartesianBlock),
    PvtResidualsV2(PvtResidualsV2Block),
    RaimStatisticsV2(RaimStatisticsV2Block),
    BaseVectorCart(BaseVectorCartBlock),
    BaseVectorGeod(BaseVectorGeodBlock),
    PosCovCartesian(PosCovCartesianBlock),
    PosCovGeodetic(PosCovGeodeticBlock),
    VelCovCartesian(VelCovCartesianBlock),
    VelCovGeodetic(VelCovGeodeticBlock),
    GeoCorrections(GeoCorrectionsBlock),
    BaseStation(BaseStationBlock),
    DiffCorrIn(DiffCorrInBlock),
    PvtSupport(PvtSupportBlock),
    PvtSupportA(PvtSupportABlock),
    PosLocal(PosLocalBlock),
    PosProjected(PosProjectedBlock),

    // Attitude blocks
    AttEuler(AttEulerBlock),
    AttCovEuler(AttCovEulerBlock),
    AuxAntPositions(AuxAntPositionsBlock),
    EndOfAtt(EndOfAttBlock),

    // Navigation blocks
    GpsNav(GpsNavBlock),
    GpsAlm(GpsAlmBlock),
    GpsIon(GpsIonBlock),
    GpsUtc(GpsUtcBlock),
    GpsCNav(GpsCNavBlock),
    GalNav(GalNavBlock),
    GalAlm(GalAlmBlock),
    GalIon(GalIonBlock),
    GalUtc(GalUtcBlock),
    GalGstGps(GalGstGpsBlock),
    GalAuthStatus(GalAuthStatusBlock),
    GalSarRlm(GalSarRlmBlock),
    GloNav(GloNavBlock),
    GloAlm(GloAlmBlock),
    GloTime(GloTimeBlock),
    BdsIon(BdsIonBlock),
    BdsNav(BdsNavBlock),
    BdsAlm(BdsAlmBlock),
    BdsUtc(BdsUtcBlock),
    BdsCNav1(BdsCNav1Block),
    BdsCNav2(BdsCNav2Block),
    BdsCNav3(BdsCNav3Block),
    QzsNav(QzsNavBlock),
    QzsAlm(QzsAlmBlock),
    GeoIonoDelay(GeoIonoDelayBlock),
    GpsRawCa(GpsRawCaBlock),
    GpsRawL2C(GpsRawL2CBlock),
    GpsRawL5(GpsRawL5Block),
    GalRawFnav(GalRawFnavBlock),
    GalRawInav(GalRawInavBlock),
    GalRawCnav(GalRawCnavBlock),
    GeoRawL1(GeoRawL1Block),
    GeoRawL5(GeoRawL5Block),
    GloRawCa(GloRawCaBlock),
    CmpRaw(CmpRawBlock),
    BdsRawB1c(BdsRawB1cBlock),
    BdsRawB2a(BdsRawB2aBlock),
    BdsRawB2b(BdsRawB2bBlock),
    IrnssRaw(IrnssRawBlock),
    QzsRawL1Ca(QzsRawL1CaBlock),
    QzsRawL2C(QzsRawL2CBlock),
    QzsRawL5(QzsRawL5Block),
    GeoMt00(GeoMt00Block),
    GeoPrnMask(GeoPrnMaskBlock),
    GeoFastCorr(GeoFastCorrBlock),
    GeoFastCorrDegr(GeoFastCorrDegrBlock),
    GeoDegrFactors(GeoDegrFactorsBlock),
    GeoServiceLevel(GeoServiceLevelBlock),
    GeoNav(GeoNavBlock),
    GeoIntegrity(GeoIntegrityBlock),
    GeoAlm(GeoAlmBlock),
    GeoNetworkTime(GeoNetworkTimeBlock),
    GeoIgpMask(GeoIgpMaskBlock),
    GeoLongTermCorr(GeoLongTermCorrBlock),
    GeoClockEphCovMatrix(GeoClockEphCovMatrixBlock),

    // Status blocks
    ReceiverStatus(ReceiverStatusBlock),
    TrackingStatus(ChannelStatusBlock),
    ChannelStatus(ChannelStatusBlock),
    SatVisibility(SatVisibilityBlock),
    QualityInd(QualityIndBlock),
    InputLink(InputLinkBlock),
    OutputLink(OutputLinkBlock),
    LBandTrackerStatus(LBandTrackerStatusBlock),
    Commands(CommandsBlock),
    Comment(CommentBlock),
    ReceiverSetup(ReceiverSetupBlock),
    BBSamples(BBSamplesBlock),
    ASCIIIn(ASCIIInBlock),
    NtripClientStatus(NtripClientStatusBlock),
    NtripServerStatus(NtripServerStatusBlock),
    RfStatus(RfStatusBlock),
    RtcmDatum(RtcmDatumBlock),
    LBandBeams(LBandBeamsBlock),
    DynDnsStatus(DynDnsStatusBlock),
    DiskStatus(DiskStatusBlock),
    P2ppStatus(P2ppStatusBlock),
    CosmosStatus(CosmosStatusBlock),
    RxMessage(RxMessageBlock),
    EncapsulatedOutput(EncapsulatedOutputBlock),
    GisAction(GisActionBlock),
    GisStatus(GisStatusBlock),

    // Time blocks
    ReceiverTime(ReceiverTimeBlock),
    PpsOffset(PpsOffsetBlock),
    ExtEvent(ExtEventBlock),
    ExtEventPvtCartesian(ExtEventPvtCartesianBlock),
    ExtEventPvtGeodetic(ExtEventPvtGeodeticBlock),
    ExtEventBaseVectGeod(ExtEventBaseVectGeodBlock),
    ExtEventAttEuler(ExtEventAttEulerBlock),
    EndOfPvt(EndOfPvtBlock),
    IntPvCart(IntPvCartBlock),
    IntPvGeod(IntPvGeodBlock),
    IntPvaaGeod(IntPvaaGeodBlock),
    IntAttEuler(IntAttEulerBlock),
    IntPosCovCart(IntPosCovCartBlock),
    IntVelCovCart(IntVelCovCartBlock),
    IntPosCovGeod(IntPosCovGeodBlock),
    IntVelCovGeod(IntVelCovGeodBlock),
    IntAttCovEuler(IntAttCovEulerBlock),
    IpStatus(IpStatusBlock),
    IqCorr(IqCorrBlock),
    ExtSensorMeas(ExtSensorMeasBlock),
    ExtSensorStatus(ExtSensorStatusBlock),
    ExtSensorSetup(ExtSensorSetupBlock),

    /// Known block ID with intentionally opaque payload bytes.
    KnownOpaque {
        id: u16,
        rev: u8,
        /// Raw payload bytes (bytes after the 8-byte SBF header).
        data: Vec<u8>,
    },

    /// Unknown block (stores raw payload bytes).
    Unknown {
        id: u16,
        rev: u8,
        /// Raw payload bytes (bytes after the 8-byte SBF header).
        data: Vec<u8>,
    },
}

impl SbfBlock {
    /// Parse a block from complete block data (including sync bytes)
    ///
    /// # Arguments
    /// * `data` - Complete block data starting with sync bytes (0x24 0x40)
    ///
    /// # Returns
    /// Tuple of (parsed block, bytes consumed)
    pub fn parse(data: &[u8]) -> SbfResult<(Self, usize)> {
        // Verify sync bytes
        if data.len() < 2 || data[0] != 0x24 || data[1] != 0x40 {
            return Err(SbfError::InvalidSync);
        }

        // Parse header
        let header = SbfHeader::parse(&data[2..])?;
        let total_len = header.length as usize;

        // Verify we have enough data
        if data.len() < total_len {
            return Err(SbfError::IncompleteBlock {
                needed: total_len,
                have: data.len(),
            });
        }

        // Data starting from CRC (after sync)
        let block_data = &data[2..];

        // Parse based on block ID
        let block = match header.block_id {
            block_ids::MEAS_EPOCH => {
                SbfBlock::MeasEpoch(MeasEpochBlock::parse(&header, block_data)?)
            }
            block_ids::MEAS_EXTRA => {
                SbfBlock::MeasExtra(MeasExtraBlock::parse(&header, block_data)?)
            }
            block_ids::END_OF_MEAS => {
                SbfBlock::EndOfMeas(EndOfMeasBlock::parse(&header, block_data)?)
            }
            block_ids::MEAS3_RANGES => {
                SbfBlock::Meas3Ranges(Meas3RangesBlock::parse(&header, block_data)?)
            }
            block_ids::MEAS3_CN0_HI_RES => {
                SbfBlock::Meas3Cn0HiRes(Meas3Cn0HiResBlock::parse(&header, block_data)?)
            }
            block_ids::MEAS3_DOPPLER => {
                SbfBlock::Meas3Doppler(Meas3DopplerBlock::parse(&header, block_data)?)
            }
            block_ids::MEAS3_PP => SbfBlock::Meas3Pp(Meas3PpBlock::parse(&header, block_data)?),
            block_ids::MEAS3_MP => SbfBlock::Meas3Mp(Meas3MpBlock::parse(&header, block_data)?),
            block_ids::PVT_GEODETIC => {
                SbfBlock::PvtGeodetic(PvtGeodeticBlock::parse(&header, block_data)?)
            }
            block_ids::PVT_CARTESIAN => {
                SbfBlock::PvtCartesian(PvtCartesianBlock::parse(&header, block_data)?)
            }
            block_ids::DOP | block_ids::DOP_LEGACY => {
                SbfBlock::Dop(DopBlock::parse(&header, block_data)?)
            }
            block_ids::POS_CART => SbfBlock::PosCart(PosCartBlock::parse(&header, block_data)?),
            block_ids::PVT_SAT_CARTESIAN => {
                SbfBlock::PvtSatCartesian(PvtSatCartesianBlock::parse(&header, block_data)?)
            }
            block_ids::PVT_RESIDUALS_V2 => {
                SbfBlock::PvtResidualsV2(PvtResidualsV2Block::parse(&header, block_data)?)
            }
            block_ids::RAIM_STATISTICS_V2 => {
                SbfBlock::RaimStatisticsV2(RaimStatisticsV2Block::parse(&header, block_data)?)
            }
            block_ids::BASE_VECTOR_CART => {
                SbfBlock::BaseVectorCart(BaseVectorCartBlock::parse(&header, block_data)?)
            }
            block_ids::BASE_VECTOR_GEOD => {
                SbfBlock::BaseVectorGeod(BaseVectorGeodBlock::parse(&header, block_data)?)
            }
            block_ids::POS_COV_CARTESIAN => {
                SbfBlock::PosCovCartesian(PosCovCartesianBlock::parse(&header, block_data)?)
            }
            block_ids::POS_COV_GEODETIC => {
                SbfBlock::PosCovGeodetic(PosCovGeodeticBlock::parse(&header, block_data)?)
            }
            block_ids::VEL_COV_CARTESIAN => {
                SbfBlock::VelCovCartesian(VelCovCartesianBlock::parse(&header, block_data)?)
            }
            block_ids::VEL_COV_GEODETIC => {
                SbfBlock::VelCovGeodetic(VelCovGeodeticBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_CORRECTIONS => {
                SbfBlock::GeoCorrections(GeoCorrectionsBlock::parse(&header, block_data)?)
            }
            block_ids::BASE_STATION => {
                SbfBlock::BaseStation(BaseStationBlock::parse(&header, block_data)?)
            }
            block_ids::DIFF_CORR_IN => {
                SbfBlock::DiffCorrIn(DiffCorrInBlock::parse(&header, block_data)?)
            }
            block_ids::PVT_SUPPORT => {
                SbfBlock::PvtSupport(PvtSupportBlock::parse(&header, block_data)?)
            }
            block_ids::PVT_SUPPORT_A => {
                SbfBlock::PvtSupportA(PvtSupportABlock::parse(&header, block_data)?)
            }
            block_ids::POS_LOCAL => SbfBlock::PosLocal(PosLocalBlock::parse(&header, block_data)?),
            block_ids::POS_PROJECTED => {
                SbfBlock::PosProjected(PosProjectedBlock::parse(&header, block_data)?)
            }
            block_ids::INT_PVA_AGEOD => {
                SbfBlock::IntPvaaGeod(IntPvaaGeodBlock::parse(&header, block_data)?)
            }
            block_ids::ATT_EULER => SbfBlock::AttEuler(AttEulerBlock::parse(&header, block_data)?),
            block_ids::ATT_COV_EULER => {
                SbfBlock::AttCovEuler(AttCovEulerBlock::parse(&header, block_data)?)
            }
            block_ids::AUX_ANT_POSITIONS => {
                SbfBlock::AuxAntPositions(AuxAntPositionsBlock::parse(&header, block_data)?)
            }
            block_ids::END_OF_ATT => SbfBlock::EndOfAtt(EndOfAttBlock::parse(&header, block_data)?),
            block_ids::GPS_NAV => SbfBlock::GpsNav(GpsNavBlock::parse(&header, block_data)?),
            block_ids::GPS_ALM => SbfBlock::GpsAlm(GpsAlmBlock::parse(&header, block_data)?),
            block_ids::GPS_ION => SbfBlock::GpsIon(GpsIonBlock::parse(&header, block_data)?),
            block_ids::GPS_UTC => SbfBlock::GpsUtc(GpsUtcBlock::parse(&header, block_data)?),
            block_ids::GPS_CNAV => SbfBlock::GpsCNav(GpsCNavBlock::parse(&header, block_data)?),
            block_ids::GAL_NAV => SbfBlock::GalNav(GalNavBlock::parse(&header, block_data)?),
            block_ids::GAL_ALM => SbfBlock::GalAlm(GalAlmBlock::parse(&header, block_data)?),
            block_ids::GAL_ION => SbfBlock::GalIon(GalIonBlock::parse(&header, block_data)?),
            block_ids::GAL_UTC => SbfBlock::GalUtc(GalUtcBlock::parse(&header, block_data)?),
            block_ids::GAL_GST_GPS => {
                SbfBlock::GalGstGps(GalGstGpsBlock::parse(&header, block_data)?)
            }
            block_ids::GAL_AUTH_STATUS => {
                SbfBlock::GalAuthStatus(GalAuthStatusBlock::parse(&header, block_data)?)
            }
            block_ids::GAL_SAR_RLM => {
                SbfBlock::GalSarRlm(GalSarRlmBlock::parse(&header, block_data)?)
            }
            block_ids::GLO_NAV => SbfBlock::GloNav(GloNavBlock::parse(&header, block_data)?),
            block_ids::GLO_ALM => SbfBlock::GloAlm(GloAlmBlock::parse(&header, block_data)?),
            block_ids::GLO_TIME => SbfBlock::GloTime(GloTimeBlock::parse(&header, block_data)?),
            block_ids::BDS_ION => SbfBlock::BdsIon(BdsIonBlock::parse(&header, block_data)?),
            block_ids::BDS_NAV => SbfBlock::BdsNav(BdsNavBlock::parse(&header, block_data)?),
            block_ids::BDS_ALM => SbfBlock::BdsAlm(BdsAlmBlock::parse(&header, block_data)?),
            block_ids::BDS_UTC => SbfBlock::BdsUtc(BdsUtcBlock::parse(&header, block_data)?),
            block_ids::BDS_CNAV1 => SbfBlock::BdsCNav1(BdsCNav1Block::parse(&header, block_data)?),
            block_ids::BDS_CNAV2 => SbfBlock::BdsCNav2(BdsCNav2Block::parse(&header, block_data)?),
            block_ids::BDS_CNAV3 => SbfBlock::BdsCNav3(BdsCNav3Block::parse(&header, block_data)?),
            block_ids::QZS_NAV => SbfBlock::QzsNav(QzsNavBlock::parse(&header, block_data)?),
            block_ids::QZS_ALM => SbfBlock::QzsAlm(QzsAlmBlock::parse(&header, block_data)?),
            block_ids::GEO_IONO_DELAY => {
                SbfBlock::GeoIonoDelay(GeoIonoDelayBlock::parse(&header, block_data)?)
            }
            block_ids::GPS_RAW_CA => SbfBlock::GpsRawCa(GpsRawCaBlock::parse(&header, block_data)?),
            block_ids::GPS_RAW_L2C => {
                SbfBlock::GpsRawL2C(GpsRawL2CBlock::parse(&header, block_data)?)
            }
            block_ids::GPS_RAW_L5 => SbfBlock::GpsRawL5(GpsRawL5Block::parse(&header, block_data)?),
            block_ids::GAL_RAW_FNAV => {
                SbfBlock::GalRawFnav(GalRawFnavBlock::parse(&header, block_data)?)
            }
            block_ids::GAL_RAW_INAV => {
                SbfBlock::GalRawInav(GalRawInavBlock::parse(&header, block_data)?)
            }
            block_ids::GAL_RAW_CNAV => {
                SbfBlock::GalRawCnav(GalRawCnavBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_RAW_L1 => SbfBlock::GeoRawL1(GeoRawL1Block::parse(&header, block_data)?),
            block_ids::GEO_RAW_L5 => SbfBlock::GeoRawL5(GeoRawL5Block::parse(&header, block_data)?),
            block_ids::GLO_RAW_CA => SbfBlock::GloRawCa(GloRawCaBlock::parse(&header, block_data)?),
            block_ids::CMP_RAW => SbfBlock::CmpRaw(CmpRawBlock::parse(&header, block_data)?),
            block_ids::BDS_RAW_B1C => {
                SbfBlock::BdsRawB1c(BdsRawB1cBlock::parse(&header, block_data)?)
            }
            block_ids::BDS_RAW_B2A => {
                SbfBlock::BdsRawB2a(BdsRawB2aBlock::parse(&header, block_data)?)
            }
            block_ids::BDS_RAW_B2B => {
                SbfBlock::BdsRawB2b(BdsRawB2bBlock::parse(&header, block_data)?)
            }
            block_ids::NAVIC_RAW => SbfBlock::IrnssRaw(IrnssRawBlock::parse(&header, block_data)?),
            block_ids::QZS_RAW_L1CA => {
                SbfBlock::QzsRawL1Ca(QzsRawL1CaBlock::parse(&header, block_data)?)
            }
            block_ids::QZS_RAW_L2C => {
                SbfBlock::QzsRawL2C(QzsRawL2CBlock::parse(&header, block_data)?)
            }
            block_ids::QZS_RAW_L5 => SbfBlock::QzsRawL5(QzsRawL5Block::parse(&header, block_data)?),
            block_ids::GEO_MT00 => SbfBlock::GeoMt00(GeoMt00Block::parse(&header, block_data)?),
            block_ids::GEO_PRN_MASK => {
                SbfBlock::GeoPrnMask(GeoPrnMaskBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_FAST_CORR => {
                SbfBlock::GeoFastCorr(GeoFastCorrBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_FAST_CORR_DEGR => {
                SbfBlock::GeoFastCorrDegr(GeoFastCorrDegrBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_DEGR_FACTORS => {
                SbfBlock::GeoDegrFactors(GeoDegrFactorsBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_SERVICE_LEVEL => {
                SbfBlock::GeoServiceLevel(GeoServiceLevelBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_NAV => SbfBlock::GeoNav(GeoNavBlock::parse(&header, block_data)?),
            block_ids::GEO_INTEGRITY => {
                SbfBlock::GeoIntegrity(GeoIntegrityBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_ALM => SbfBlock::GeoAlm(GeoAlmBlock::parse(&header, block_data)?),
            block_ids::GEO_NETWORK_TIME => {
                SbfBlock::GeoNetworkTime(GeoNetworkTimeBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_IGP_MASK => {
                SbfBlock::GeoIgpMask(GeoIgpMaskBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_LONG_TERM_CORR => {
                SbfBlock::GeoLongTermCorr(GeoLongTermCorrBlock::parse(&header, block_data)?)
            }
            block_ids::GEO_CLOCK_EPH_COV_MATRIX => SbfBlock::GeoClockEphCovMatrix(
                GeoClockEphCovMatrixBlock::parse(&header, block_data)?,
            ),
            block_ids::RECEIVER_STATUS => {
                SbfBlock::ReceiverStatus(ReceiverStatusBlock::parse(&header, block_data)?)
            }
            block_ids::TRACKING_STATUS => {
                SbfBlock::TrackingStatus(ChannelStatusBlock::parse(&header, block_data)?)
            }
            block_ids::CHANNEL_STATUS => {
                SbfBlock::ChannelStatus(ChannelStatusBlock::parse(&header, block_data)?)
            }
            block_ids::SAT_VISIBILITY => {
                SbfBlock::SatVisibility(SatVisibilityBlock::parse(&header, block_data)?)
            }
            block_ids::QUALITY_IND => {
                SbfBlock::QualityInd(QualityIndBlock::parse(&header, block_data)?)
            }
            block_ids::INPUT_LINK => {
                SbfBlock::InputLink(InputLinkBlock::parse(&header, block_data)?)
            }
            block_ids::OUTPUT_LINK => {
                SbfBlock::OutputLink(OutputLinkBlock::parse(&header, block_data)?)
            }
            block_ids::LBAND_TRACKER_STATUS => {
                SbfBlock::LBandTrackerStatus(LBandTrackerStatusBlock::parse(&header, block_data)?)
            }
            block_ids::COMMANDS => SbfBlock::Commands(CommandsBlock::parse(&header, block_data)?),
            block_ids::COMMENT => SbfBlock::Comment(CommentBlock::parse(&header, block_data)?),
            block_ids::RECEIVER_SETUP => {
                SbfBlock::ReceiverSetup(ReceiverSetupBlock::parse(&header, block_data)?)
            }
            block_ids::BB_SAMPLES => {
                SbfBlock::BBSamples(BBSamplesBlock::parse(&header, block_data)?)
            }
            block_ids::ASCII_IN => SbfBlock::ASCIIIn(ASCIIInBlock::parse(&header, block_data)?),
            block_ids::NTRIP_CLIENT_STATUS => {
                SbfBlock::NtripClientStatus(NtripClientStatusBlock::parse(&header, block_data)?)
            }
            block_ids::NTRIP_SERVER_STATUS => {
                SbfBlock::NtripServerStatus(NtripServerStatusBlock::parse(&header, block_data)?)
            }
            block_ids::RF_STATUS => SbfBlock::RfStatus(RfStatusBlock::parse(&header, block_data)?),
            block_ids::RTCM_DATUM => {
                SbfBlock::RtcmDatum(RtcmDatumBlock::parse(&header, block_data)?)
            }
            block_ids::LBAND_BEAMS => {
                SbfBlock::LBandBeams(LBandBeamsBlock::parse(&header, block_data)?)
            }
            block_ids::DYN_DNS_STATUS => {
                SbfBlock::DynDnsStatus(DynDnsStatusBlock::parse(&header, block_data)?)
            }
            block_ids::DISK_STATUS => {
                SbfBlock::DiskStatus(DiskStatusBlock::parse(&header, block_data)?)
            }
            block_ids::P2PP_STATUS => {
                SbfBlock::P2ppStatus(P2ppStatusBlock::parse(&header, block_data)?)
            }
            block_ids::COSMOS_STATUS => {
                SbfBlock::CosmosStatus(CosmosStatusBlock::parse(&header, block_data)?)
            }
            block_ids::RX_MESSAGE => {
                SbfBlock::RxMessage(RxMessageBlock::parse(&header, block_data)?)
            }
            block_ids::ENCAPSULATED_OUTPUT => {
                SbfBlock::EncapsulatedOutput(EncapsulatedOutputBlock::parse(&header, block_data)?)
            }
            block_ids::GIS_ACTION => {
                SbfBlock::GisAction(GisActionBlock::parse(&header, block_data)?)
            }
            block_ids::GIS_STATUS => {
                SbfBlock::GisStatus(GisStatusBlock::parse(&header, block_data)?)
            }
            block_ids::RECEIVER_TIME => {
                SbfBlock::ReceiverTime(ReceiverTimeBlock::parse(&header, block_data)?)
            }
            block_ids::PPS_OFFSET => {
                SbfBlock::PpsOffset(PpsOffsetBlock::parse(&header, block_data)?)
            }
            block_ids::EXT_EVENT => SbfBlock::ExtEvent(ExtEventBlock::parse(&header, block_data)?),
            block_ids::EXT_EVENT_PVT_CARTESIAN => SbfBlock::ExtEventPvtCartesian(
                ExtEventPvtCartesianBlock::parse(&header, block_data)?,
            ),
            block_ids::EXT_EVENT_PVT_GEODETIC => {
                SbfBlock::ExtEventPvtGeodetic(ExtEventPvtGeodeticBlock::parse(&header, block_data)?)
            }
            block_ids::EXT_EVENT_BASE_VECT_GEOD => SbfBlock::ExtEventBaseVectGeod(
                ExtEventBaseVectGeodBlock::parse(&header, block_data)?,
            ),
            block_ids::EXT_EVENT_ATT_EULER => {
                SbfBlock::ExtEventAttEuler(ExtEventAttEulerBlock::parse(&header, block_data)?)
            }
            block_ids::END_OF_PVT => SbfBlock::EndOfPvt(EndOfPvtBlock::parse(&header, block_data)?),
            block_ids::INT_PV_CART => {
                SbfBlock::IntPvCart(IntPvCartBlock::parse(&header, block_data)?)
            }
            block_ids::INT_PV_GEOD => {
                SbfBlock::IntPvGeod(IntPvGeodBlock::parse(&header, block_data)?)
            }
            block_ids::INT_ATT_EULER => {
                SbfBlock::IntAttEuler(IntAttEulerBlock::parse(&header, block_data)?)
            }
            block_ids::INT_POS_COV_CART => {
                SbfBlock::IntPosCovCart(IntPosCovCartBlock::parse(&header, block_data)?)
            }
            block_ids::INT_VEL_COV_CART => {
                SbfBlock::IntVelCovCart(IntVelCovCartBlock::parse(&header, block_data)?)
            }
            block_ids::INT_POS_COV_GEOD => {
                SbfBlock::IntPosCovGeod(IntPosCovGeodBlock::parse(&header, block_data)?)
            }
            block_ids::INT_VEL_COV_GEOD => {
                SbfBlock::IntVelCovGeod(IntVelCovGeodBlock::parse(&header, block_data)?)
            }
            block_ids::INT_ATT_COV_EULER => {
                SbfBlock::IntAttCovEuler(IntAttCovEulerBlock::parse(&header, block_data)?)
            }
            block_ids::IP_STATUS => SbfBlock::IpStatus(IpStatusBlock::parse(&header, block_data)?),
            block_ids::IQ_CORR => SbfBlock::IqCorr(IqCorrBlock::parse(&header, block_data)?),
            block_ids::EXT_SENSOR_MEAS => {
                SbfBlock::ExtSensorMeas(ExtSensorMeasBlock::parse(&header, block_data)?)
            }
            block_ids::EXT_SENSOR_STATUS => {
                SbfBlock::ExtSensorStatus(ExtSensorStatusBlock::parse(&header, block_data)?)
            }
            block_ids::EXT_SENSOR_SETUP => {
                SbfBlock::ExtSensorSetup(ExtSensorSetupBlock::parse(&header, block_data)?)
            }
            id if is_known_opaque_id(id) => SbfBlock::KnownOpaque {
                id,
                rev: header.block_rev,
                data: data[8..total_len].to_vec(),
            },
            _ => SbfBlock::Unknown {
                id: header.block_id,
                rev: header.block_rev,
                data: data[8..total_len].to_vec(),
            },
        };

        Ok((block, total_len))
    }

    /// Get the block ID
    pub fn block_id(&self) -> u16 {
        match self {
            SbfBlock::MeasEpoch(_) => block_ids::MEAS_EPOCH,
            SbfBlock::MeasExtra(_) => block_ids::MEAS_EXTRA,
            SbfBlock::EndOfMeas(_) => block_ids::END_OF_MEAS,
            SbfBlock::Meas3Ranges(_) => block_ids::MEAS3_RANGES,
            SbfBlock::Meas3Cn0HiRes(_) => block_ids::MEAS3_CN0_HI_RES,
            SbfBlock::Meas3Doppler(_) => block_ids::MEAS3_DOPPLER,
            SbfBlock::Meas3Pp(_) => block_ids::MEAS3_PP,
            SbfBlock::Meas3Mp(_) => block_ids::MEAS3_MP,
            SbfBlock::PvtGeodetic(_) => block_ids::PVT_GEODETIC,
            SbfBlock::PvtCartesian(_) => block_ids::PVT_CARTESIAN,
            SbfBlock::Dop(_) => block_ids::DOP,
            SbfBlock::PosCart(_) => block_ids::POS_CART,
            SbfBlock::PvtSatCartesian(_) => block_ids::PVT_SAT_CARTESIAN,
            SbfBlock::PvtResidualsV2(_) => block_ids::PVT_RESIDUALS_V2,
            SbfBlock::RaimStatisticsV2(_) => block_ids::RAIM_STATISTICS_V2,
            SbfBlock::BaseVectorCart(_) => block_ids::BASE_VECTOR_CART,
            SbfBlock::BaseVectorGeod(_) => block_ids::BASE_VECTOR_GEOD,
            SbfBlock::PosCovCartesian(_) => block_ids::POS_COV_CARTESIAN,
            SbfBlock::PosCovGeodetic(_) => block_ids::POS_COV_GEODETIC,
            SbfBlock::VelCovCartesian(_) => block_ids::VEL_COV_CARTESIAN,
            SbfBlock::VelCovGeodetic(_) => block_ids::VEL_COV_GEODETIC,
            SbfBlock::GeoCorrections(_) => block_ids::GEO_CORRECTIONS,
            SbfBlock::BaseStation(_) => block_ids::BASE_STATION,
            SbfBlock::DiffCorrIn(_) => block_ids::DIFF_CORR_IN,
            SbfBlock::PvtSupport(_) => block_ids::PVT_SUPPORT,
            SbfBlock::PvtSupportA(_) => block_ids::PVT_SUPPORT_A,
            SbfBlock::PosLocal(_) => block_ids::POS_LOCAL,
            SbfBlock::PosProjected(_) => block_ids::POS_PROJECTED,
            SbfBlock::IntPvaaGeod(_) => block_ids::INT_PVA_AGEOD,
            SbfBlock::AttEuler(_) => block_ids::ATT_EULER,
            SbfBlock::AttCovEuler(_) => block_ids::ATT_COV_EULER,
            SbfBlock::AuxAntPositions(_) => block_ids::AUX_ANT_POSITIONS,
            SbfBlock::EndOfAtt(_) => block_ids::END_OF_ATT,
            SbfBlock::GpsNav(_) => block_ids::GPS_NAV,
            SbfBlock::GpsAlm(_) => block_ids::GPS_ALM,
            SbfBlock::GpsIon(_) => block_ids::GPS_ION,
            SbfBlock::GpsUtc(_) => block_ids::GPS_UTC,
            SbfBlock::GpsCNav(_) => block_ids::GPS_CNAV,
            SbfBlock::GalNav(_) => block_ids::GAL_NAV,
            SbfBlock::GalAlm(_) => block_ids::GAL_ALM,
            SbfBlock::GalIon(_) => block_ids::GAL_ION,
            SbfBlock::GalUtc(_) => block_ids::GAL_UTC,
            SbfBlock::GalGstGps(_) => block_ids::GAL_GST_GPS,
            SbfBlock::GalAuthStatus(_) => block_ids::GAL_AUTH_STATUS,
            SbfBlock::GalSarRlm(_) => block_ids::GAL_SAR_RLM,
            SbfBlock::GloNav(_) => block_ids::GLO_NAV,
            SbfBlock::GloAlm(_) => block_ids::GLO_ALM,
            SbfBlock::GloTime(_) => block_ids::GLO_TIME,
            SbfBlock::BdsIon(_) => block_ids::BDS_ION,
            SbfBlock::BdsNav(_) => block_ids::BDS_NAV,
            SbfBlock::BdsAlm(_) => block_ids::BDS_ALM,
            SbfBlock::BdsUtc(_) => block_ids::BDS_UTC,
            SbfBlock::BdsCNav1(_) => block_ids::BDS_CNAV1,
            SbfBlock::BdsCNav2(_) => block_ids::BDS_CNAV2,
            SbfBlock::BdsCNav3(_) => block_ids::BDS_CNAV3,
            SbfBlock::QzsNav(_) => block_ids::QZS_NAV,
            SbfBlock::QzsAlm(_) => block_ids::QZS_ALM,
            SbfBlock::GeoIonoDelay(_) => block_ids::GEO_IONO_DELAY,
            SbfBlock::GpsRawCa(_) => block_ids::GPS_RAW_CA,
            SbfBlock::GpsRawL2C(_) => block_ids::GPS_RAW_L2C,
            SbfBlock::GpsRawL5(_) => block_ids::GPS_RAW_L5,
            SbfBlock::GalRawFnav(_) => block_ids::GAL_RAW_FNAV,
            SbfBlock::GalRawInav(_) => block_ids::GAL_RAW_INAV,
            SbfBlock::GalRawCnav(_) => block_ids::GAL_RAW_CNAV,
            SbfBlock::GeoRawL1(_) => block_ids::GEO_RAW_L1,
            SbfBlock::GeoRawL5(_) => block_ids::GEO_RAW_L5,
            SbfBlock::GloRawCa(_) => block_ids::GLO_RAW_CA,
            SbfBlock::CmpRaw(_) => block_ids::CMP_RAW,
            SbfBlock::BdsRawB1c(_) => block_ids::BDS_RAW_B1C,
            SbfBlock::BdsRawB2a(_) => block_ids::BDS_RAW_B2A,
            SbfBlock::BdsRawB2b(_) => block_ids::BDS_RAW_B2B,
            SbfBlock::IrnssRaw(_) => block_ids::NAVIC_RAW,
            SbfBlock::QzsRawL1Ca(_) => block_ids::QZS_RAW_L1CA,
            SbfBlock::QzsRawL2C(_) => block_ids::QZS_RAW_L2C,
            SbfBlock::QzsRawL5(_) => block_ids::QZS_RAW_L5,
            SbfBlock::GeoMt00(_) => block_ids::GEO_MT00,
            SbfBlock::GeoPrnMask(_) => block_ids::GEO_PRN_MASK,
            SbfBlock::GeoFastCorr(_) => block_ids::GEO_FAST_CORR,
            SbfBlock::GeoFastCorrDegr(_) => block_ids::GEO_FAST_CORR_DEGR,
            SbfBlock::GeoDegrFactors(_) => block_ids::GEO_DEGR_FACTORS,
            SbfBlock::GeoServiceLevel(_) => block_ids::GEO_SERVICE_LEVEL,
            SbfBlock::GeoNav(_) => block_ids::GEO_NAV,
            SbfBlock::GeoIntegrity(_) => block_ids::GEO_INTEGRITY,
            SbfBlock::GeoAlm(_) => block_ids::GEO_ALM,
            SbfBlock::GeoNetworkTime(_) => block_ids::GEO_NETWORK_TIME,
            SbfBlock::GeoIgpMask(_) => block_ids::GEO_IGP_MASK,
            SbfBlock::GeoLongTermCorr(_) => block_ids::GEO_LONG_TERM_CORR,
            SbfBlock::GeoClockEphCovMatrix(_) => block_ids::GEO_CLOCK_EPH_COV_MATRIX,
            SbfBlock::ReceiverStatus(_) => block_ids::RECEIVER_STATUS,
            SbfBlock::TrackingStatus(_) => block_ids::TRACKING_STATUS,
            SbfBlock::ChannelStatus(_) => block_ids::CHANNEL_STATUS,
            SbfBlock::SatVisibility(_) => block_ids::SAT_VISIBILITY,
            SbfBlock::QualityInd(_) => block_ids::QUALITY_IND,
            SbfBlock::InputLink(_) => block_ids::INPUT_LINK,
            SbfBlock::OutputLink(_) => block_ids::OUTPUT_LINK,
            SbfBlock::LBandTrackerStatus(_) => block_ids::LBAND_TRACKER_STATUS,
            SbfBlock::Commands(_) => block_ids::COMMANDS,
            SbfBlock::Comment(_) => block_ids::COMMENT,
            SbfBlock::ReceiverSetup(_) => block_ids::RECEIVER_SETUP,
            SbfBlock::BBSamples(_) => block_ids::BB_SAMPLES,
            SbfBlock::ASCIIIn(_) => block_ids::ASCII_IN,
            SbfBlock::NtripClientStatus(_) => block_ids::NTRIP_CLIENT_STATUS,
            SbfBlock::NtripServerStatus(_) => block_ids::NTRIP_SERVER_STATUS,
            SbfBlock::RfStatus(_) => block_ids::RF_STATUS,
            SbfBlock::RtcmDatum(_) => block_ids::RTCM_DATUM,
            SbfBlock::LBandBeams(_) => block_ids::LBAND_BEAMS,
            SbfBlock::DynDnsStatus(_) => block_ids::DYN_DNS_STATUS,
            SbfBlock::DiskStatus(_) => block_ids::DISK_STATUS,
            SbfBlock::P2ppStatus(_) => block_ids::P2PP_STATUS,
            SbfBlock::CosmosStatus(_) => block_ids::COSMOS_STATUS,
            SbfBlock::RxMessage(_) => block_ids::RX_MESSAGE,
            SbfBlock::EncapsulatedOutput(_) => block_ids::ENCAPSULATED_OUTPUT,
            SbfBlock::GisAction(_) => block_ids::GIS_ACTION,
            SbfBlock::GisStatus(_) => block_ids::GIS_STATUS,
            SbfBlock::ReceiverTime(_) => block_ids::RECEIVER_TIME,
            SbfBlock::PpsOffset(_) => block_ids::PPS_OFFSET,
            SbfBlock::ExtEvent(_) => block_ids::EXT_EVENT,
            SbfBlock::ExtEventPvtCartesian(_) => block_ids::EXT_EVENT_PVT_CARTESIAN,
            SbfBlock::ExtEventPvtGeodetic(_) => block_ids::EXT_EVENT_PVT_GEODETIC,
            SbfBlock::ExtEventBaseVectGeod(_) => block_ids::EXT_EVENT_BASE_VECT_GEOD,
            SbfBlock::ExtEventAttEuler(_) => block_ids::EXT_EVENT_ATT_EULER,
            SbfBlock::EndOfPvt(_) => block_ids::END_OF_PVT,
            SbfBlock::IntPvCart(_) => block_ids::INT_PV_CART,
            SbfBlock::IntPvGeod(_) => block_ids::INT_PV_GEOD,
            SbfBlock::IntAttEuler(_) => block_ids::INT_ATT_EULER,
            SbfBlock::IntPosCovCart(_) => block_ids::INT_POS_COV_CART,
            SbfBlock::IntVelCovCart(_) => block_ids::INT_VEL_COV_CART,
            SbfBlock::IntPosCovGeod(_) => block_ids::INT_POS_COV_GEOD,
            SbfBlock::IntVelCovGeod(_) => block_ids::INT_VEL_COV_GEOD,
            SbfBlock::IntAttCovEuler(_) => block_ids::INT_ATT_COV_EULER,
            SbfBlock::IpStatus(_) => block_ids::IP_STATUS,
            SbfBlock::IqCorr(_) => block_ids::IQ_CORR,
            SbfBlock::ExtSensorMeas(_) => block_ids::EXT_SENSOR_MEAS,
            SbfBlock::ExtSensorStatus(_) => block_ids::EXT_SENSOR_STATUS,
            SbfBlock::ExtSensorSetup(_) => block_ids::EXT_SENSOR_SETUP,
            SbfBlock::KnownOpaque { id, .. } => *id,
            SbfBlock::Unknown { id, .. } => *id,
        }
    }

    /// Get the block name
    pub fn name(&self) -> &'static str {
        block_name(self.block_id())
    }

    /// Get raw payload bytes for unsupported blocks.
    ///
    /// Returns `Some(&[u8])` for `KnownOpaque` and `Unknown` variants, otherwise `None`.
    pub fn unsupported_payload(&self) -> Option<&[u8]> {
        match self {
            SbfBlock::KnownOpaque { data, .. } | SbfBlock::Unknown { data, .. } => Some(data),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_min_block(block_id: u16, block_rev: u8) -> Vec<u8> {
        let total_len = 16u16;
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;

        let id_rev = block_id | ((block_rev as u16 & 0x07) << 13);
        data[4..6].copy_from_slice(&id_rev.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&123_456u32.to_le_bytes());
        data[12..14].copy_from_slice(&2150u16.to_le_bytes());
        data[14] = 0xAA;
        data[15] = 0x55;
        data
    }

    #[test]
    fn test_meas3_ranges_min_block_parse() {
        // Minimum valid Meas3Ranges: block_data length 18 (fixed header through Reserved, empty Data).
        let total_len = 20u16;
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;
        let id_rev = block_ids::MEAS3_RANGES | ((2u16 & 0x07) << 13);
        data[4..6].copy_from_slice(&id_rev.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&123_456u32.to_le_bytes());
        data[12..14].copy_from_slice(&2150u16.to_le_bytes());

        let (block, consumed) = SbfBlock::parse(&data).unwrap();
        assert_eq!(consumed, usize::from(total_len));
        match block {
            SbfBlock::Meas3Ranges(m) => {
                assert_eq!(m.tow_ms(), 123_456);
                assert_eq!(m.wnc(), 2150);
                assert_eq!(m.data, Vec::<u8>::new());
            }
            other => panic!("expected Meas3Ranges, got {:?}", other),
        }
    }

    #[test]
    fn test_pvt_support_parse() {
        let data = build_min_block(block_ids::PVT_SUPPORT, 1);
        let (block, consumed) = SbfBlock::parse(&data).unwrap();

        assert_eq!(consumed, 16);
        assert_eq!(block.block_id(), block_ids::PVT_SUPPORT);
        match block {
            SbfBlock::PvtSupport(pvt) => {
                assert_eq!(pvt.tow_ms(), 123_456);
                assert_eq!(pvt.wnc(), 2150);
                assert!((pvt.tow_seconds() - 123.456).abs() < 1e-6);
            }
            other => panic!("expected PvtSupport, got {:?}", other),
        }
    }

    #[test]
    fn test_bds_raw_b2b_min_block_parse() {
        let total_len: u16 = 144; // 2 sync + 142 block bytes (B2b min)
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;
        let id_rev = block_ids::BDS_RAW_B2B;
        data[4..6].copy_from_slice(&id_rev.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&1_000u32.to_le_bytes());
        data[12..14].copy_from_slice(&200u16.to_le_bytes());
        data[14] = 5;
        data[15] = 1;

        let (block, consumed) = SbfBlock::parse(&data).unwrap();
        assert_eq!(consumed, total_len as usize);
        match block {
            SbfBlock::BdsRawB2b(b) => {
                assert_eq!(b.tow_ms(), 1_000);
                assert_eq!(b.wnc(), 200);
                assert_eq!(b.svid, 5);
            }
            other => panic!("expected BdsRawB2b, got {:?}", other),
        }
    }

    #[test]
    fn test_gal_auth_status_min_block_parse() {
        let total_len: u16 = 52;
        let mut data = vec![0u8; total_len as usize];
        data[0] = 0x24;
        data[1] = 0x40;
        let id_rev = block_ids::GAL_AUTH_STATUS;
        data[4..6].copy_from_slice(&id_rev.to_le_bytes());
        data[6..8].copy_from_slice(&total_len.to_le_bytes());
        data[8..12].copy_from_slice(&2_000u32.to_le_bytes());
        data[12..14].copy_from_slice(&201u16.to_le_bytes());
        data[14..16].copy_from_slice(&0xABCDu16.to_le_bytes());
        data[16..20].copy_from_slice(&1.5f32.to_le_bytes());
        for i in 0..8usize {
            data[20 + i] = 0x10 + i as u8;
            data[28 + i] = 0x20 + i as u8;
            data[36 + i] = 0x30 + i as u8;
            data[44 + i] = 0x40 + i as u8;
        }

        let (block, consumed) = SbfBlock::parse(&data).unwrap();
        assert_eq!(consumed, total_len as usize);
        match block {
            SbfBlock::GalAuthStatus(a) => {
                assert_eq!(a.tow_ms(), 2_000);
                assert_eq!(a.wnc(), 201);
                assert_eq!(a.osnma_status, 0xABCD);
                assert!((a.trusted_time_delta - 1.5).abs() < 1e-6);
            }
            other => panic!("expected GalAuthStatus, got {:?}", other),
        }
    }

    #[test]
    fn test_unknown_id_stays_unknown() {
        let data = build_min_block(4999, 0);
        let (block, _) = SbfBlock::parse(&data).unwrap();

        match block {
            SbfBlock::Unknown { id, rev, data } => {
                assert_eq!(id, 4999);
                assert_eq!(rev, 0);
                assert_eq!(data, vec![0x40, 0xE2, 0x01, 0x00, 0x66, 0x08, 0xAA, 0x55]);
            }
            other => panic!("expected Unknown, got {:?}", other),
        }
    }

    #[test]
    fn test_unsupported_payload_helper() {
        let known_data = build_min_block(block_ids::FUGRO_DDS, 0);
        let (known, _) = SbfBlock::parse(&known_data).unwrap();
        assert_eq!(
            known.unsupported_payload(),
            Some([0x40, 0xE2, 0x01, 0x00, 0x66, 0x08, 0xAA, 0x55].as_slice())
        );

        let unknown_data = build_min_block(4998, 1);
        let (unknown, _) = SbfBlock::parse(&unknown_data).unwrap();
        assert_eq!(
            unknown.unsupported_payload(),
            Some([0x40, 0xE2, 0x01, 0x00, 0x66, 0x08, 0xAA, 0x55].as_slice())
        );
    }
}
