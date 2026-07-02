use std::io::{self, Cursor, Read};

use sbf_tools::{block_ids, calculate_block_crc, SbfBlock, SbfError, SbfReadExt};

const UNKNOWN_BLOCK_ID: u16 = 8190;
const TEST_WNC: u16 = 2200;

struct ChunkedReader {
    inner: Cursor<Vec<u8>>,
    chunk_size: usize,
}

impl ChunkedReader {
    fn new(data: Vec<u8>, chunk_size: usize) -> Self {
        Self {
            inner: Cursor::new(data),
            chunk_size,
        }
    }
}

impl Read for ChunkedReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let max = buf.len().min(self.chunk_size);
        self.inner.read(&mut buf[..max])
    }
}

fn build_block(block_id: u16, block_rev: u8, tow_ms: u32, wnc: u16, body: &[u8]) -> Vec<u8> {
    let mut total_len = 8 + 6 + body.len();
    while (total_len & 0x03) != 0 {
        total_len += 1;
    }

    let mut block = vec![0u8; total_len];
    let id_rev = block_id | ((u16::from(block_rev) & 0x07) << 13);

    block[0] = 0x24;
    block[1] = 0x40;
    block[4..6].copy_from_slice(&id_rev.to_le_bytes());
    block[6..8].copy_from_slice(&(total_len as u16).to_le_bytes());
    block[8..12].copy_from_slice(&tow_ms.to_le_bytes());
    block[12..14].copy_from_slice(&wnc.to_le_bytes());
    block[14..14 + body.len()].copy_from_slice(body);

    let crc = calculate_block_crc(&block[4..total_len]);
    block[2..4].copy_from_slice(&crc.to_le_bytes());
    block
}

fn with_bad_crc(mut block: Vec<u8>) -> Vec<u8> {
    block[2] ^= 0xff;
    block
}

#[test]
fn reader_resynchronizes_after_crc_error_and_preserves_unknown_block() {
    let receiver_time = build_block(
        block_ids::RECEIVER_TIME,
        0,
        1_000,
        TEST_WNC,
        &[24, 3, 31, 23, 59, 58, 18u8, 4],
    );
    let corrupted_end_of_meas =
        with_bad_crc(build_block(block_ids::END_OF_MEAS, 0, 1_001, TEST_WNC, &[]));
    let unknown = build_block(UNKNOWN_BLOCK_ID, 1, 1_002, TEST_WNC, &[0xde, 0xad, 0xbe]);
    let end_of_meas = build_block(block_ids::END_OF_MEAS, 0, 1_003, TEST_WNC, &[]);

    let mut stream = vec![0xaa, 0xbb, 0xcc];
    stream.extend_from_slice(&receiver_time);
    stream.extend_from_slice(&corrupted_end_of_meas);
    stream.extend_from_slice(&unknown);
    stream.extend_from_slice(&end_of_meas);

    let mut reader = ChunkedReader::new(stream.clone(), 5).sbf_blocks();

    let first = reader.read_block().unwrap().unwrap();
    let SbfBlock::ReceiverTime(time) = first else {
        panic!("expected ReceiverTime");
    };
    assert_eq!(time.tow_ms(), 1_000);
    assert_eq!(time.wnc(), TEST_WNC);
    assert_eq!(time.utc_string(), "2024-03-31T23:59:58Z");
    assert!(time.is_synchronized());

    let second = reader.read_block().unwrap().unwrap();
    let SbfBlock::Unknown { id, rev, data } = second else {
        panic!("expected Unknown block");
    };
    assert_eq!(id, UNKNOWN_BLOCK_ID);
    assert_eq!(rev, 1);
    assert_eq!(data, unknown[8..].to_vec());

    let third = reader.read_block().unwrap().unwrap();
    let SbfBlock::EndOfMeas(block) = third else {
        panic!("expected EndOfMeas");
    };
    assert_eq!(block.tow_ms(), 1_003);
    assert_eq!(block.wnc(), TEST_WNC);

    assert!(reader.read_block().unwrap().is_none());

    let stats = reader.stats();
    assert_eq!(stats.bytes_read, stream.len() as u64);
    assert_eq!(stats.blocks_parsed, 3);
    assert_eq!(stats.crc_errors, 1);
    assert_eq!(stats.parse_errors, 0);
    assert!(stats.bytes_skipped >= 4);
}

#[test]
fn reader_reports_incomplete_final_block() {
    let receiver_time = build_block(
        block_ids::RECEIVER_TIME,
        0,
        2_000,
        TEST_WNC,
        &[24, 4, 1, 0, 0, 0, 18u8, 3],
    );
    let mut truncated_end_of_meas = build_block(block_ids::END_OF_MEAS, 0, 2_001, TEST_WNC, &[]);
    truncated_end_of_meas.truncate(truncated_end_of_meas.len() - 3);

    let mut stream = receiver_time;
    stream.extend_from_slice(&truncated_end_of_meas);

    let mut reader = ChunkedReader::new(stream, 4).sbf_blocks();

    let first = reader.read_block().unwrap().unwrap();
    assert!(matches!(first, SbfBlock::ReceiverTime(_)));

    let err = reader.read_block().unwrap_err();
    let SbfError::IncompleteBlock { needed, have } = err else {
        panic!("expected IncompleteBlock");
    };
    assert_eq!(needed, 8);
    assert!(have >= 2);
}

/// A `Read` that replays a scripted sequence of results, including transient
/// `WouldBlock` errors, to emulate a non-blocking source.
struct FlakyReader {
    chunks: std::collections::VecDeque<io::Result<Vec<u8>>>,
}

impl FlakyReader {
    fn new(chunks: Vec<io::Result<Vec<u8>>>) -> Self {
        Self {
            chunks: chunks.into_iter().collect(),
        }
    }
}

impl Read for FlakyReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.chunks.pop_front() {
            Some(Ok(data)) => {
                let n = data.len().min(buf.len());
                buf[..n].copy_from_slice(&data[..n]);
                if n < data.len() {
                    self.chunks.push_front(Ok(data[n..].to_vec()));
                }
                Ok(n)
            }
            Some(Err(e)) => Err(e),
            None => Ok(0),
        }
    }
}

#[test]
fn reader_signals_would_block_then_resumes() {
    let block = build_block(
        block_ids::RECEIVER_TIME,
        0,
        5_000,
        TEST_WNC,
        &[24, 3, 31, 23, 59, 58, 18u8, 4],
    );
    let mid = block.len() / 2;
    let first = block[..mid].to_vec();
    let second = block[mid..].to_vec();

    let mut reader = FlakyReader::new(vec![
        Err(io::Error::from(io::ErrorKind::WouldBlock)),
        Ok(first),
        Err(io::Error::from(io::ErrorKind::WouldBlock)),
        Ok(second),
    ])
    .sbf_blocks();

    // No data available yet -> WouldBlock, not end of stream.
    assert!(matches!(reader.read_block(), Err(SbfError::WouldBlock)));
    // Partial block buffered, then WouldBlock again mid-block.
    assert!(matches!(reader.read_block(), Err(SbfError::WouldBlock)));
    // Remaining bytes arrive -> the block completes.
    let parsed = reader.read_block().unwrap().unwrap();
    assert!(matches!(parsed, SbfBlock::ReceiverTime(_)));
    // Source now returns Ok(0) -> genuine end of stream.
    assert!(reader.read_block().unwrap().is_none());
}

#[test]
fn reader_skips_bounded_garbage_with_no_sync() {
    const GARBAGE: usize = 10_000;
    // 0x00 bytes contain no 0x24 0x40 sync, so the reader must traverse the whole
    // run via the no-sync branch without losing the following block.
    let mut stream = vec![0u8; GARBAGE];
    let block = build_block(block_ids::END_OF_MEAS, 0, 7_000, TEST_WNC, &[]);
    stream.extend_from_slice(&block);

    let mut reader = Cursor::new(stream).sbf_blocks();
    let parsed = reader.read_block().unwrap().unwrap();
    assert!(matches!(parsed, SbfBlock::EndOfMeas(_)));
    assert!(reader.read_block().unwrap().is_none());

    let stats = reader.stats();
    assert_eq!(stats.blocks_parsed, 1);
    assert_eq!(stats.bytes_skipped, GARBAGE as u64);
}
