use std::env;
use std::error::Error;
use std::fs::File;

use sbf_tools::{MeasEpochBlock, MeasExtraBlock, SbfBlock, SbfReader};

#[derive(Default)]
struct EpochBundle {
    epoch: Option<MeasEpochBlock>,
    extra: Option<MeasExtraBlock>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args()
        .nth(1)
        .ok_or("usage: cargo run --example decode_meas_epoch -- <path-to-file.sbf>")?;
    let mut reader = SbfReader::new(File::open(path)?);
    let mut current_key: Option<(u16, u32)> = None;
    let mut bundle = EpochBundle::default();

    loop {
        let block = match reader.read_block() {
            Ok(Some(block)) => block,
            Ok(None) => break,
            Err(error) => {
                eprintln!("SBF parse error: {error}");
                continue;
            }
        };

        let key = match &block {
            SbfBlock::MeasEpoch(value) => Some((value.wnc(), value.tow_ms())),
            SbfBlock::MeasExtra(value) => Some((value.wnc(), value.tow_ms())),
            _ => None,
        };

        if let Some(key) = key {
            if current_key.is_some() && current_key != Some(key) {
                emit_bundle(&mut bundle);
            }
            current_key = Some(key);
        }

        match block {
            SbfBlock::MeasEpoch(value) => bundle.epoch = Some(value),
            SbfBlock::MeasExtra(value) => bundle.extra = Some(value),
            SbfBlock::EndOfMeas(_) => {
                emit_bundle(&mut bundle);
                current_key = None;
            }
            _ => {}
        }
    }

    emit_bundle(&mut bundle);
    Ok(())
}

fn emit_bundle(bundle: &mut EpochBundle) {
    let Some(epoch) = bundle.epoch.take() else {
        bundle.extra = None;
        return;
    };

    for measurement in &epoch.measurements {
        let extra = bundle
            .extra
            .as_ref()
            .and_then(|value| epoch.matching_extra_channel(value, measurement));

        let refined_cn0_dbhz = measurement.cn0_dbhz_opt().map(|cn0| {
            cn0 + extra
                .and_then(|channel| channel.cn0_high_res_dbhz_offset())
                .unwrap_or(0.0)
        });

        println!(
            "WNc={} TOW={} satellite={} signal={} antenna={} channel={} PR={:?}m L={:?}cycles D={:?}Hz CN0={:?}dB-Hz raw={:?}",
            epoch.wnc(),
            epoch.tow_ms(),
            measurement.sat_id,
            measurement.signal_type,
            measurement.antenna_id(),
            measurement.rx_channel(),
            measurement.pseudorange_m(),
            measurement.carrier_phase_cycles(),
            measurement.doppler_hz_opt(),
            refined_cn0_dbhz,
            measurement.raw(),
        );
    }

    bundle.extra = None;
}
