# SBF Tools

[![crates.io](https://img.shields.io/crates/v/sbf-tools.svg)](https://crates.io/crates/sbf-tools)
[![docs.rs](https://docs.rs/sbf-tools/badge.svg)](https://docs.rs/sbf-tools)

Rust parser for Septentrio Binary Format.

This crate reads existing SBF logs and streams, turns the common
blocks into typed Rust values, and keeps anything it does not understand as raw bytes instead of
pretending otherwise.

## Scope

- Reads SBF blocks from any `Read` source.
- Parses measurement, PVT, attitude, timing, navigation, SBAS, INS, status, and mosaic-era
  support blocks.
- Includes a stateful `Meas3Decoder` built from Septentrio's `sbf2asc` sources in RxTools.
- Keeps unknown block payloads as raw bytes.
- Parse-only. This crate does not write SBF.
- `SbfBlock` is `#[non_exhaustive]`: new block variants may appear in any minor release. If you
  `match` on `SbfBlock` outside this crate, include a wildcard arm so your code keeps compiling
  as support expands.

The `serde` feature currently adds derives for the shared value types in `types.rs`. It does not
cover every block struct yet.

## Installation

The Cargo package is `sbf-tools`, and the Rust crate name is `sbf_tools`:

```toml
[dependencies]
sbf_tools = { package = "sbf-tools", version = "0.2.0" }
```

With the optional `serde` feature:

```toml
[dependencies]
sbf_tools = { package = "sbf-tools", version = "0.2.0", features = ["serde"] }
```

## Quick start

```rust
use std::fs::File;

use sbf_tools::{SbfBlock, SbfReader};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = File::open("data.sbf")?;

    for block in SbfReader::new(file) {
        match block? {
            SbfBlock::PvtGeodetic(pvt) => {
                if let (Some(lat), Some(lon), Some(height)) =
                    (pvt.latitude_deg(), pvt.longitude_deg(), pvt.height_m())
                {
                    println!("PVT {lat:.6} {lon:.6} {height:.2} m");
                }
            }
            SbfBlock::ReceiverTime(time) => {
                if time.is_synchronized() {
                    println!("UTC {}", time.utc_string());
                }
            }
            other => {
                println!("{}", other.name());
            }
        }
    }

    Ok(())
}
```

## Reader API

The main entry point is `SbfReader<R: Read>`. It is both an iterator and a lower-level block
reader.

```rust
use std::fs::File;

use sbf_tools::{SbfReadExt, SbfReader};

let file = File::open("data.sbf")?;

// Plain reader
let mut reader = SbfReader::new(file);

// Or through the extension trait
let file = File::open("data.sbf")?;
let reader_from_ext = file.sbf_blocks();

// Read block-by-block yourself
let _ = reader.read_block()?;

// Stats are kept on the reader
let stats = reader.stats();
println!("blocks parsed: {}", stats.blocks_parsed);
println!("crc errors: {}", stats.crc_errors);

// Clear stats if you want to reuse the reader statefully
reader.reset_stats();
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Examples

Count blocks in an SBF file:

```bash
cargo run --example read_blocks -- data.sbf
```

Decode `Meas3` epochs:

```bash
cargo run --example decode_meas3 -- data.sbf
```

## Supported blocks

The list below is the block surface that this crate parses. For some blocks, the payload is
fully decoded into fields; for a few support blocks, the crate keeps a named raw payload because
the public Septentrio reference guide does not publish the internal layout.

### Measurement blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 4027 | `MeasEpoch` | Code, carrier, Doppler, CN0, lock time |
| 4000 | `MeasExtra` | Extra measurement data, variances, multipath, smoothing |
| 4046 | `IQCorr` | Post-correlation I/Q values |
| 4109 | `Meas3Ranges` | Packed Meas3 code, carrier, CN0 block |
| 4110 | `Meas3CN0HiRes` | Fractional CN0 companion block |
| 4111 | `Meas3Doppler` | Doppler companion block |
| 4112 | `Meas3PP` | Post-processing companion block |
| 4113 | `Meas3MP` | Multipath companion block |
| 5922 | `EndOfMeas` | End of measurement epoch marker |

### Position, PVT, and correction blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 4007 | `PVTGeodetic` | Latitude, longitude, height, velocity, fix mode |
| 4006 | `PVTCartesian` | ECEF position and velocity |
| 4001 | `DOP` | Parsed together with legacy `5909` |
| 5909 | `DOP` | Legacy DOP block |
| 4052 | `PosLocal` | Local datum coordinates |
| 4094 | `PosProjected` | Projected grid coordinates |
| 4044 | `PosCart` | Cartesian position with base vector terms |
| 4008 | `PVTSatCartesian` | Per-satellite ECEF position and velocity |
| 4009 | `PVTResiduals_v2` | Per-signal residuals |
| 4011 | `RAIMStatistics_v2` | RAIM integrity and test statistics |
| 4043 | `BaseVectorCart` | Relative vector in ECEF |
| 4028 | `BaseVectorGeod` | Relative vector in ENU/geodetic form |
| 5905 | `PosCovCartesian` | Position covariance in ECEF |
| 5906 | `PosCovGeodetic` | Position covariance in geodetic frame |
| 5907 | `VelCovCartesian` | Velocity covariance in ECEF |
| 5908 | `VelCovGeodetic` | Velocity covariance in geodetic frame |
| 5935 | `GEOCorrections` | GEO correction summary |
| 5919 | `DiffCorrIn` | Incoming RTCM/CMR/SPARTN correction payload |
| 5949 | `BaseStation` | Base station ECEF coordinates |
| 4049 | `RTCMDatum` | Source/target CRS names and quality flags |
| 4076 | `PVTSupport` | Public part only |
| 4079 | `PVTSupportA` | Named raw payload wrapper |
| 5921 | `EndOfPVT` | End of PVT epoch marker |

### Attitude and event blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 5938 | `AttEuler` | Heading, pitch, roll |
| 5939 | `AttCovEuler` | Attitude covariance |
| 5942 | `AuxAntPositions` | Auxiliary antenna offsets |
| 5943 | `EndOfAtt` | End of attitude epoch marker |
| 5914 | `ReceiverTime` | UTC date/time and sync level |
| 5911 | `xPPSOffset` | PPS offset and sync age |
| 5924 | `ExtEvent` | Event timing and receiver clock bias |
| 4037 | `ExtEventPVTCartesian` | Event-time PVT in ECEF |
| 4038 | `ExtEventPVTGeodetic` | Event-time PVT in geodetic form |
| 4217 | `ExtEventBaseVectGeod` | Event-time ENU base vectors |
| 4237 | `ExtEventAttEuler` | Event-time attitude |

### Status, receiver, and miscellaneous blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 4014 | `ReceiverStatus` | CPU load, receiver state, AGC data |
| 5912 | `TrackingStatus` | Parsed with the same layout as `ChannelStatus` |
| 4013 | `ChannelStatus` | Channel allocation and tracking state |
| 4012 | `SatVisibility` | Azimuth and elevation |
| 4090 | `InputLink` | Input link counters |
| 4091 | `OutputLink` | Output link counters |
| 4058 | `IPStatus` | MAC, IP, gateway, netmask |
| 4082 | `QualityInd` | Receiver quality indicators |
| 4105 | `DynDNSStatus` | DynDNS registration state |
| 4059 | `DiskStatus` | Disk usage, flags, disk errors |
| 4092 | `RFStatus` | Interference and spoofing flags |
| 4053 | `NTRIPClientStatus` | Client sessions |
| 4122 | `NTRIPServerStatus` | Server sessions |
| 4201 | `LBandTrackerStatus` | L-band tracker state |
| 4204 | `LBandBeams` | L-band beam names, longitude, frequency |
| 4238 | `P2PPStatus` | Point-to-point session state |
| 4243 | `CosmosStatus` | Cosmos service status |
| 5902 | `ReceiverSetup` | Station metadata and RINEX header data |
| 4103 | `RxMessage` | Receiver activity log message |
| 4015 | `Commands` | Raw command payload |
| 5936 | `Comment` | Observer comment string |
| 4040 | `BBSamples` | Complex baseband samples |
| 4075 | `ASCIIIn` | Received ASCII line |
| 4097 | `EncapsulatedOutput` | Embedded RTCM/CMR/NMEA/ASCII output |
| 4106 | `GISAction` | PinPoint GIS action |
| 4107 | `GISStatus` | PinPoint GIS database status |
| 4211 | `FugroDDS` | Kept as `KnownOpaque` |

### Navigation blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 5891 | `GPSNav` | GPS ephemeris |
| 5892 | `GPSAlm` | GPS almanac |
| 5893 | `GPSIon` | GPS ionosphere |
| 5894 | `GPSUtc` | GPS-UTC |
| 4042 | `GPSCNav` | GPS CNAV ephemeris |
| 4017 | `GPSRawCA` | Raw GPS C/A subframe |
| 4018 | `GPSRawL2C` | Raw GPS L2C frame |
| 4019 | `GPSRawL5` | Raw GPS L5 frame |
| 4002 | `GALNav` | Galileo ephemeris |
| 4003 | `GALAlm` | Galileo almanac |
| 4030 | `GALIon` | Galileo ionosphere |
| 4031 | `GALUtc` | Galileo UTC parameters |
| 4032 | `GALGstGps` | Galileo GST-GPS relationship |
| 4245 | `GALAuthStatus` | Galileo OSNMA authentication status |
| 4034 | `GALSARRLM` | Galileo SAR return-link message |
| 4022 | `GALRawFNAV` | Raw Galileo F/NAV |
| 4023 | `GALRawINAV` | Raw Galileo I/NAV |
| 4024 | `GALRawCNAV` | Raw Galileo CNAV |
| 4004 | `GLONav` | GLONASS ephemeris |
| 4005 | `GLOAlm` | GLONASS almanac |
| 4036 | `GLOTime` | GLONASS time parameters |
| 4026 | `GLORawCA` | Raw GLONASS CA string |
| 4081 | `BDSNav` | BeiDou ephemeris |
| 4119 | `BDSAlm` | BeiDou almanac |
| 4120 | `BDSIon` | BeiDou ionosphere |
| 4121 | `BDSUtc` | BeiDou UTC parameters |
| 4251 | `BDSCNav1` | BeiDou B-CNAV1 |
| 4252 | `BDSCNav2` | BeiDou B-CNAV2 |
| 4253 | `BDSCNav3` | BeiDou B-CNAV3 |
| 4218 | `BDSRawB1C` | Raw BeiDou B1C |
| 4219 | `BDSRawB2a` | Raw BeiDou B2a |
| 4242 | `BDSRawB2b` | Raw BeiDou B2b |
| 4047 | `CMPRaw` | Raw BeiDou nav bits |
| 4095 | `QZSNav` | QZSS ephemeris |
| 4116 | `QZSAlm` | QZSS almanac |
| 4066 | `QZSRawL1CA` | Raw QZSS L1 C/A |
| 4067 | `QZSRawL2C` | Raw QZSS L2C |
| 4068 | `QZSRawL5` | Raw QZSS L5 |
| 4093 | `NAVICRaw` | Raw NavIC/IRNSS |
| 4020 | `GEORawL1` | Raw SBAS L1 |
| 4021 | `GEORawL5` | Raw SBAS L5 |
| 5933 | `GEOIonoDelay` | SBAS MT26 ionospheric delay |

### SBAS blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 5925 | `GEOMT00` | SBAS MT00 |
| 5926 | `GEOPRNMask` | SBAS MT01 |
| 5927 | `GEOFastCorr` | SBAS fast corrections |
| 5929 | `GEOFastCorrDegr` | SBAS fast correction degradation |
| 5930 | `GEODegrFactors` | SBAS degradation factors |
| 5917 | `GEOServiceLevel` | SBAS service message |
| 5896 | `GEONav` | SBAS navigation |
| 5928 | `GEOIntegrity` | SBAS integrity |
| 5897 | `GEOAlm` | SBAS almanac |
| 5918 | `GEONetworkTime` | SBAS network time |
| 5931 | `GEOIGPMask` | SBAS ionospheric grid mask |
| 5932 | `GEOLongTermCorr` | SBAS long-term corrections |
| 5934 | `GEOClockEphCovMatrix` | SBAS covariance matrix |

### INS and external sensor blocks

| Block ID | Name | Notes |
|----------|------|-------|
| 4060 | `IntPVCart` | INS/GNSS position and velocity in ECEF |
| 4061 | `IntPVGeod` | INS/GNSS position and velocity in geodetic form |
| 4062 | `IntPosCovCart` | INS position covariance in ECEF |
| 4063 | `IntVelCovCart` | INS velocity covariance in ECEF |
| 4064 | `IntPosCovGeod` | INS position covariance in geodetic frame |
| 4065 | `IntVelCovGeod` | INS velocity covariance in geodetic frame |
| 4072 | `IntAttCovEuler` | INS attitude covariance |
| 4045 | `IntPVAAGeod` | INS position, velocity, acceleration, attitude |
| 4070 | `IntAttEuler` | INS attitude |
| 4050 | `ExtSensorMeas` | External sensor measurements |
| 4056 | `ExtSensorStatus` | External sensor status |
| 4057 | `ExtSensorSetup` | External sensor configuration |

Anything else falls back to:

- `SbfBlock::KnownOpaque { id, rev, data }` for known but still raw payloads such as `FugroDDS`
- `SbfBlock::Unknown { id, rev, data }` for anything else

`block_name(id)` and `fallback_name(id)` are there if you need stable naming for IDs you are not
decoding yet.

## Meas3

`Meas3Ranges` is only one piece of a complete Meas3 epoch. To recover code, carrier, C/N0,
Doppler, lock time, and the companion post-processing and multipath fields, collect the matching
Meas3 blocks for one epoch and feed them to `Meas3Decoder`.

```rust
use sbf_tools::{Meas3BlockSet, Meas3Decoder};

let mut decoder = Meas3Decoder::new();
let mut block_set = Meas3BlockSet::default();

// Insert the matching Meas3 blocks for one TOW / antenna.
// block_set.insert_block(&block);

let decoded = decoder.decode_block_set(&block_set)?;
println!(
    "antenna {}: {} satellites, {} measurements",
    decoded.antenna_id,
    decoded.num_satellites(),
    decoded.num_measurements()
);
# Ok::<(), sbf_tools::SbfError>(())
```

The public mosaic reference guide does not publish the packed `Meas3` payload layout. The decoder
in this crate follows the C implementation shipped with Septentrio RxTools.

Some practical points:

- A `Meas3` epoch is keyed by TOW, WNc, and antenna.
- `Meas3Ranges` is required. The other Meas3 blocks are companions.
- Delta epochs depend on an earlier reference epoch.
- If a delta epoch arrives before its reference epoch, decoding fails for that epoch.
- Scrambled Meas3 payloads are not supported.

## Block parsing notes

- SBF sync bytes are `0x24 0x40`.
- CRC validation is enabled by default. If a CRC fails, `SbfReader` counts it, skips forward, and
  keeps scanning for the next valid sync.
- Variable-length blocks such as `MeasEpoch`, `MeasExtra`, `ChannelStatus`, `InputLink`,
  `OutputLink`, `LBandTrackerStatus`, `LBandBeams`, `DiskStatus`, and `P2PPStatus` are parsed
  from their sub-block length fields.
- Padding bytes at the end of a block are ignored.
- Revisions are handled where the layout actually changes. For example, blocks such as
  `ReceiverStatus`, `DynDNSStatus`, and `DiskStatus` expose revision-specific fields when the
  data is present.
- When the receiver uses a documented do-not-use value, accessors return `None` instead of the raw
  sentinel where that makes sense.
- Some blocks are public in name but not in field layout. `PVTSupportA` is kept as a named raw
  payload wrapper for that reason.
- `block_name(id)` returns the main name table, and `fallback_name(id)` covers additional
  catalogued IDs that may not yet have a typed block.

## Raw vs scaled values

Not every block exposes both raw and scaled accessors, but the crate tries to do so where the SBF
format uses compact integer storage with a published scale factor.

```rust
use sbf_tools::DopBlock;

fn example(dop: &DopBlock) {
    let pdop_raw: u16 = dop.pdop_raw();
    let pdop: f32 = dop.pdop();

    println!("raw={pdop_raw}, scaled={pdop}");
}
```

A few common patterns in the API:

- `DopBlock` exposes raw DOP integers and scaled floating-point values.
- Visibility and base-vector blocks keep raw azimuth/elevation fields internally and expose degree
  accessors such as `azimuth_deg()` and `elevation_deg()`.
- PVT, covariance, and attitude blocks expose unit-ready accessors like `latitude_deg()`,
  `height_m()`, `heading_deg()`, `corr_age_seconds()`, or standard deviations derived from
  covariance fields.
- `MeasEpoch` and `Meas3` surface measurements in physical units directly through helpers such as
  `cn0_dbhz()`, `doppler_hz()`, `pseudorange_m()`, and `carrier_phase_cycles()`.

## Common scaling factors

The same conversions show up repeatedly across SBF blocks. The exact accessor name varies by block,
but these are the ones you will run into most often:

- TOW: milliseconds in the wire format, seconds through `tow_seconds()`
- DOP values: `raw * 0.01`
- Azimuth and elevation in visibility/vector blocks: `raw * 0.01` degrees
- Accuracy and correction age fields in many PVT-related blocks: `raw * 0.01`
- `MeasEpoch` Doppler: `raw * 0.0001` Hz
- `MeasEpoch` CN0: `raw * 0.25`, with a `+10 dB` offset for most signals and no offset for
  GPS `L1P/L2P`
- `Meas3Ranges` CN0: integer dB-Hz, with fractional refinement from `Meas3CN0HiRes`
- `BBSamples` I/Q words: packed signed 8-bit `I` and `Q` values inside each `u16`

The main thing to keep in mind is that there is no single global scaling rule for SBF. Use the
block-specific accessor if it exists.

## Configuration options

The reader surface is intentionally small:

```rust
use std::fs::File;

use sbf_tools::{SbfReadExt, SbfReader};

let file = File::open("data.sbf")?;

let mut reader = SbfReader::with_capacity(file, 128 * 1024)
    .validate_crc(false);

while let Some(block) = reader.next() {
    let _block = block?;
}

println!("bytes read: {}", reader.stats().bytes_read);
println!("blocks parsed: {}", reader.stats().blocks_parsed);
println!("crc errors: {}", reader.stats().crc_errors);
println!("parse errors: {}", reader.stats().parse_errors);
println!("bytes skipped: {}", reader.stats().bytes_skipped);

reader.reset_stats();

let file = File::open("data.sbf")?;
let _same_reader = file.sbf_blocks();
# Ok::<(), Box<dyn std::error::Error>>(())
```

What those options mean in practice:

- `SbfReader::new(reader)` uses the default buffer sizing.
- `SbfReader::with_capacity(reader, capacity)` lets you start with a larger buffer if you are
  working with large or bursty inputs.
- `.validate_crc(false)` skips CRC checks. That can be useful for trusted files when you care more
  about throughput than corruption detection.
- `read_block()` gives explicit control over `Ok(Some(block))`, `Ok(None)`, and parse errors.
- The iterator interface is the simplest option for ordinary file and stream processing.

## Other details worth knowing

- The crate reads blocks. It does not configure receivers, manage ports, or write SBF back out.
- Unknown blocks are preserved rather than dropped, so you can still archive or inspect payloads
  even when the crate does not decode them.
- The examples in `examples/` are intended as working starting points, not a complete CLI.
- The Cargo package name is `sbf-tools`; the Rust crate name is `sbf_tools`.

## License

MIT
