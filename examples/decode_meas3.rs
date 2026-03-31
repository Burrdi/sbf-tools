use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs::File;

use sbf_tools::{Meas3BlockSet, Meas3Decoder, SbfBlock, SbfReader};

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args()
        .nth(1)
        .ok_or("usage: cargo run --example decode_meas3 -- <path-to-file.sbf>")?;

    let file = File::open(&path)?;
    let reader = SbfReader::new(file);

    let mut decoder = Meas3Decoder::new();
    let mut current_tow: Option<u32> = None;
    let mut bundles: HashMap<u8, Meas3BlockSet> = HashMap::new();

    for block in reader {
        let block = block?;

        if let Some((tow_ms, antenna_id)) = meas3_epoch_key(&block) {
            if current_tow != Some(tow_ms) {
                flush_bundles(&mut bundles, &mut decoder)?;
                current_tow = Some(tow_ms);
            }
            bundles.entry(antenna_id).or_default().insert_block(&block);
            continue;
        }

        if matches!(block, SbfBlock::EndOfMeas(_)) {
            flush_bundles(&mut bundles, &mut decoder)?;
            current_tow = None;
        }
    }

    flush_bundles(&mut bundles, &mut decoder)?;
    Ok(())
}

fn meas3_epoch_key(block: &SbfBlock) -> Option<(u32, u8)> {
    match block {
        SbfBlock::Meas3Ranges(b) => Some((b.tow_ms(), b.antenna_id())),
        SbfBlock::Meas3Cn0HiRes(b) => Some((b.tow_ms(), b.antenna_id())),
        SbfBlock::Meas3Doppler(b) => Some((b.tow_ms(), b.antenna_id())),
        SbfBlock::Meas3Pp(b) => Some((b.tow_ms(), b.antenna_id())),
        SbfBlock::Meas3Mp(b) => Some((b.tow_ms(), b.antenna_id())),
        _ => None,
    }
}

fn flush_bundles(
    bundles: &mut HashMap<u8, Meas3BlockSet>,
    decoder: &mut Meas3Decoder,
) -> Result<(), Box<dyn Error>> {
    let mut antenna_ids: Vec<u8> = bundles.keys().copied().collect();
    antenna_ids.sort_unstable();

    for antenna_id in antenna_ids {
        if let Some(block_set) = bundles.get(&antenna_id) {
            if block_set.ranges.is_none() {
                continue;
            }

            let epoch = decoder.decode_block_set(block_set)?;
            println!(
                "TOW {} WNc {} antenna {}: {} satellites, {} measurements",
                epoch.tow_ms(),
                epoch.wnc(),
                epoch.antenna_id,
                epoch.num_satellites(),
                epoch.num_measurements()
            );

            for satellite in epoch.satellites.iter().take(4) {
                let signal_summary = satellite
                    .measurements
                    .iter()
                    .map(|meas| match meas.cn0_dbhz() {
                        Some(cn0) => format!("{}:{cn0:.2}dB-Hz", meas.signal_type),
                        None => meas.signal_type.to_string(),
                    })
                    .collect::<Vec<_>>()
                    .join(", ");
                println!("  {}  {}", satellite.sat_id, signal_summary);
            }
        }
    }

    bundles.clear();
    Ok(())
}
