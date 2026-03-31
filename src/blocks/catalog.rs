//! Human-readable names for SBF block IDs without dedicated [`super::SbfBlock`] variants.

/// Names for IDs not listed in the primary [`super::block_name`] match.
pub fn fallback_name(id: u16) -> &'static str {
    match id {
        4049 => "RTCMDatum",
        4059 => "DiskStatus",
        4097 => "EncapsulatedOutput",
        4103 => "RxMessage",
        4105 => "DynDNSStatus",
        4106 => "GISAction",
        4107 => "GISStatus",
        4116 => "QZSAlm",
        4119 => "BDSAlm",
        4204 => "LBandBeams",
        4217 => "ExtEventBaseVectGeod",
        4237 => "ExtEventAttEuler",
        4238 => "P2PPStatus",
        4243 => "CosmosStatus",
        4252 => "BDSCNav2",
        4253 => "BDSCNav3",
        5913 => "ReceiverStatus",
        _ => "Unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fallback_names() {
        assert_eq!(fallback_name(4049), "RTCMDatum");
        assert_eq!(fallback_name(4253), "BDSCNav3");
        assert_eq!(fallback_name(0xFFFF), "Unknown");
    }
}
