//! Golden decode vector: a blob encoded by depthpack 0.1.0 (pure-Rust
//! encoder, scale 0.001 m, unit "m") that every future version of the
//! DECODER must keep decoding to the same lattice and physical values.
//! This is the format's backward-compatibility contract — encoder output
//! may evolve (zstd levels, backend changes), decoded results may not.
//!
//! Regenerate only for a deliberate, versioned format change:
//! `cargo run --example gen_golden`.

use depthpack::{decode, decode_header, decode_scaled};

const BLOB_HEX: &str = "4450434b010010000000080000006d000000fca9f1d24d62503f00000000000000006d000000000000002100000028b52ffd0438a500000c01000000dfffdfffdfffffffffffffffffff00211c489928b52ffd0438a50200fc01000800000000000000000001d40e0e0e0e1c0e0e0e0e0e0e0e0e0e040e8a0e0e15a8100357da3b1014e27d108a81041b5a0e5f2b822712c0874452e092dc0538b9c8520b6422915005bec8cddd00e00600328d57814b";

/// The 16×8 source image (millimetre counts): row 0 is nodata sky, a
/// nodata slit at x=5 for the first rows, a smooth gradient elsewhere.
fn expected_counts() -> Vec<u16> {
    (0..16u32 * 8)
        .map(|i| {
            let (x, y) = (i % 16, i / 16);
            if y == 0 || (x == 5 && y < 4) {
                0
            } else {
                (1000 + x * 7 + y * 130) as u16
            }
        })
        .collect()
}

fn blob() -> Vec<u8> {
    (0..BLOB_HEX.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&BLOB_HEX[i..i + 2], 16).unwrap())
        .collect()
}

#[test]
fn golden_lattice_decodes_identically_forever() {
    let img = decode(&blob()).unwrap();
    assert_eq!(img.width, 16);
    assert_eq!(img.height, 8);
    assert_eq!(img.values, expected_counts());
    assert_eq!(img.scale, 0.001);
    assert_eq!(img.unit, "m");
}

#[test]
fn golden_scaled_decodes_to_metres() {
    let scaled = decode_scaled(&blob()).unwrap();
    assert_eq!(scaled.unit, "m");
    for (i, (&c, &phys)) in expected_counts().iter().zip(&scaled.values).enumerate() {
        if c == 0 {
            assert!(phys.is_nan(), "nodata px {i}");
        } else {
            assert!((phys as f64 - c as f64 * 0.001).abs() < 1e-6, "px {i}");
        }
    }
}

#[test]
fn golden_header_fields() {
    let hdr = decode_header(&blob()).unwrap();
    assert_eq!(hdr.width, 16);
    assert_eq!(hdr.height, 8);
    assert_eq!(hdr.n_valid, 16 * 8 - 16 - 3); // minus sky row, minus slit rows y=1..4
    assert_eq!(hdr.scale, 0.001);
    assert_eq!(hdr.offset, 0.0);
    assert_eq!(hdr.unit, "m");
}
