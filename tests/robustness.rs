//! Decoder robustness: any byte input must return `Ok` or `Err` —
//! never panic, never allocate unboundedly. This is the poor man's fuzz
//! pass that runs on every CI build; see `fuzz/` for the libFuzzer
//! harness that explores far deeper.

use depthpack::{EncodeOptions, MAX_PIXELS, decode, encode};

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 11
    }
}

fn sample_blob() -> Vec<u8> {
    let (w, h) = (96u32, 48u32);
    let depth: Vec<u16> = (0..w * h)
        .map(|i| {
            if i % 11 == 0 {
                0
            } else {
                1_500 + (i % 700) as u16
            }
        })
        .collect();
    encode(&depth, w, h, &EncodeOptions::default()).unwrap()
}

#[test]
fn every_truncation_is_handled() {
    let blob = sample_blob();
    for len in 0..blob.len() {
        let _ = decode(&blob[..len]); // must not panic
    }
}

#[test]
fn bit_flips_are_handled() {
    let blob = sample_blob();
    let mut rng = Lcg(99);
    for _ in 0..4_000 {
        let mut m = blob.clone();
        let byte = (rng.next() as usize) % m.len();
        let bit = (rng.next() as u8) % 8;
        m[byte] ^= 1 << bit;
        let _ = decode(&m); // must not panic
    }
}

#[test]
fn multi_byte_corruption_is_handled() {
    let blob = sample_blob();
    let mut rng = Lcg(7_777);
    for _ in 0..1_000 {
        let mut m = blob.clone();
        for _ in 0..1 + rng.next() % 16 {
            let i = (rng.next() as usize) % m.len();
            m[i] = rng.next() as u8;
        }
        let _ = decode(&m);
    }
}

#[test]
fn random_garbage_is_rejected() {
    let mut rng = Lcg(31_337);
    for _ in 0..2_000 {
        let len = (rng.next() as usize) % 512;
        let garbage: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
        assert!(decode(&garbage).is_err());
    }
}

/// A forged header demanding a huge allocation must be rejected by the
/// MAX_PIXELS guard, not attempted.
#[test]
fn oversized_dimensions_are_rejected_cheaply() {
    let mut blob = sample_blob();
    blob[6..10].copy_from_slice(&u32::MAX.to_le_bytes()); // width
    blob[10..14].copy_from_slice(&u32::MAX.to_le_bytes()); // height
    assert!(decode(&blob).is_err());
    assert!(u64::from(u32::MAX) * u64::from(u32::MAX) > MAX_PIXELS);
}

/// n_valid inflated past the mask's popcount must be caught before the
/// residual stage tries to read it.
#[test]
fn inflated_n_valid_is_rejected() {
    let mut blob = sample_blob();
    let n_valid = u32::from_le_bytes([blob[14], blob[15], blob[16], blob[17]]);
    blob[14..18].copy_from_slice(&(n_valid + 1).to_le_bytes());
    assert!(decode(&blob).is_err());
}

/// A mask section length pointing past the end of the blob.
#[test]
fn overrunning_mask_length_is_rejected() {
    let mut blob = sample_blob();
    let total_len = blob.len() as u32;
    blob[42..46].copy_from_slice(&total_len.to_le_bytes());
    assert!(decode(&blob).is_err());
}

/// Wrong version / codec bytes must be refused, not misparsed.
#[test]
fn unknown_version_and_codec_are_rejected() {
    let blob = sample_blob();
    let mut v = blob.clone();
    v[4] = 99; // unsupported version
    assert!(decode(&v).is_err());
    let mut c = blob;
    c[5] = 1; // unknown codec
    assert!(decode(&c).is_err());
}
