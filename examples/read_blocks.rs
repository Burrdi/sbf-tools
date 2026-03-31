use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::File;

use sbf_tools::SbfReader;

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args()
        .nth(1)
        .ok_or("usage: cargo run --example read_blocks -- <path-to-file.sbf>")?;

    let file = File::open(&path)?;
    let reader = SbfReader::new(file);
    let mut counts = BTreeMap::<&'static str, u64>::new();

    for block in reader {
        let block = block?;
        *counts.entry(block.name()).or_default() += 1;
    }

    for (name, count) in counts {
        println!("{count:>8}  {name}");
    }

    Ok(())
}
