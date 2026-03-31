//! Shared do-not-use sentinels for SBF fields.

/// Do-not-use value for `f32` fields.
pub(super) const F32_DNU: f32 = -2e10;

/// Do-not-use value for `f64` fields.
pub(super) const F64_DNU: f64 = -2e10;

/// Do-not-use value for `u16` fields.
pub(super) const U16_DNU: u16 = 65535;

/// Do-not-use value for `i16` fields.
pub(super) const I16_DNU: i16 = -32768;

/// Do-not-use value for `i32` fields.
pub(super) const I32_DNU: i32 = -2147483648;

/// Do-not-use value for `i8` fields.
pub(super) const I8_DNU: i8 = -128;

pub(super) fn f32_or_none(value: f32) -> Option<f32> {
    if value == F32_DNU {
        None
    } else {
        Some(value)
    }
}

pub(super) fn f64_or_none(value: f64) -> Option<f64> {
    if value == F64_DNU {
        None
    } else {
        Some(value)
    }
}

pub(super) fn u16_or_none(value: u16) -> Option<u16> {
    if value == U16_DNU {
        None
    } else {
        Some(value)
    }
}
