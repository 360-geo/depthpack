//! Roundtrip contracts: the lattice is always bit-exact (including
//! adversarial noise, which exercises the wrapping-residual path), the
//! scale/offset/unit metadata survives, decode_scaled applies the
//! mapping with NaN nodata, and shape edge cases hold.

use depthpack::{
    DepthImage, EncodeOptions, decode, decode_header, decode_into, decode_scaled, encode,
};

/// Deterministic LCG so failures reproduce without a rand dep.
struct Lcg(u64);
impl Lcg {
    fn next_u16(&mut self) -> u16 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) as u16
    }
}

fn opts() -> EncodeOptions {
    EncodeOptions {
        scale: 0.001,
        unit: "m".into(),
        ..Default::default()
    }
}

/// A plausible depth map (millimetre counts): smooth surface + nodata
/// sky + speckle holes.
fn synthetic_surface(w: u32, h: u32, seed: u64) -> Vec<u16> {
    let mut rng = Lcg(seed);
    (0..w * h)
        .map(|i| {
            let (x, y) = (i % w, i / w);
            let speckle = rng.next_u16().is_multiple_of(19);
            if y < h / 3 || speckle {
                0
            } else {
                let base = 2_000 + x * 3 + (y - h / 3) * 11;
                (base % 30_000 + 500) as u16
            }
        })
        .collect()
}

#[test]
fn lattice_is_bit_exact_on_surface() {
    let (w, h) = (160u32, 90u32);
    let counts = synthetic_surface(w, h, 42);
    let blob = encode(&counts, w, h, &opts()).unwrap();
    let img = decode(&blob).unwrap();
    assert_eq!(
        img,
        DepthImage {
            width: w,
            height: h,
            values: counts,
            scale: 0.001,
            offset: 0.0,
            unit: "m".into(),
        }
    );
}

#[test]
fn lattice_is_bit_exact_on_adversarial_noise() {
    // Uniform u16 noise maximizes residual magnitude — this is the input
    // class that breaks a non-wrapping zigzag implementation.
    let (w, h) = (127u32, 63u32);
    let mut rng = Lcg(7);
    let counts: Vec<u16> = (0..w * h).map(|_| rng.next_u16()).collect();
    let blob = encode(&counts, w, h, &opts()).unwrap();
    assert_eq!(decode(&blob).unwrap().values, counts);
}

#[test]
fn lattice_is_bit_exact_on_extremes() {
    // Alternating min/max valid values adjacent to nodata.
    let (w, h) = (64u32, 4u32);
    let counts: Vec<u16> = (0..w * h)
        .map(|i| match i % 4 {
            0 => 1,
            1 => 65_535,
            2 => 0,
            _ => 32_768,
        })
        .collect();
    let blob = encode(&counts, w, h, &opts()).unwrap();
    assert_eq!(decode(&blob).unwrap().values, counts);
}

#[test]
fn decode_scaled_applies_mapping_with_nan_nodata() {
    let (w, h) = (8u32, 4u32);
    // count 0 = nodata; others map through scale=0.001, offset=0.25.
    let counts: Vec<u16> = (0..w * h)
        .map(|i| if i % 5 == 0 { 0 } else { (i * 100) as u16 })
        .collect();
    let o = EncodeOptions {
        scale: 0.001,
        offset: 0.25,
        unit: "m".into(),
        ..Default::default()
    };
    let blob = encode(&counts, w, h, &o).unwrap();
    let scaled = decode_scaled(&blob).unwrap();

    assert_eq!(scaled.unit, "m");
    for (i, (&c, &phys)) in counts.iter().zip(&scaled.values).enumerate() {
        if c == 0 {
            assert!(phys.is_nan(), "nodata px {i} must be NaN, got {phys}");
        } else {
            let want = c as f64 * 0.001 + 0.25;
            assert!(
                (phys as f64 - want).abs() < 1e-6,
                "px {i}: {phys} vs {want}"
            );
        }
    }
}

#[test]
fn shape_edge_cases_roundtrip() {
    for (w, h) in [(1u32, 1u32), (1, 100), (100, 1), (7, 3), (8, 8)] {
        let mut rng = Lcg(u64::from(w) << 32 | u64::from(h));
        let counts: Vec<u16> = (0..w * h).map(|_| rng.next_u16() % 3_000).collect();
        let blob = encode(&counts, w, h, &opts()).unwrap();
        assert_eq!(decode(&blob).unwrap().values, counts, "{w}x{h}");
    }
}

#[test]
fn all_nodata_and_all_valid_roundtrip() {
    let (w, h) = (33u32, 17u32);
    let zeros = vec![0u16; (w * h) as usize];
    let blob = encode(&zeros, w, h, &opts()).unwrap();
    assert_eq!(decode(&blob).unwrap().values, zeros);
    assert_eq!(decode_header(&blob).unwrap().n_valid, 0);
    // decode_scaled: every pixel NaN.
    assert!(
        decode_scaled(&blob)
            .unwrap()
            .values
            .iter()
            .all(|v| v.is_nan())
    );

    let full = vec![1_234u16; (w * h) as usize];
    let blob = encode(&full, w, h, &opts()).unwrap();
    assert_eq!(decode(&blob).unwrap().values, full);
    assert_eq!(decode_header(&blob).unwrap().n_valid, w * h);
}

#[test]
fn header_reports_scale_offset_unit() {
    let (w, h) = (50u32, 20u32);
    let counts = synthetic_surface(w, h, 5);
    let o = EncodeOptions {
        scale: 0.005,
        offset: -1.0,
        unit: "ftUS".into(),
        ..Default::default()
    };
    let blob = encode(&counts, w, h, &o).unwrap();
    let hdr = decode_header(&blob).unwrap();
    assert_eq!((hdr.width, hdr.height), (w, h));
    assert_eq!(hdr.scale, 0.005);
    assert_eq!(hdr.offset, -1.0);
    assert_eq!(hdr.unit, "ftUS");
}

#[test]
fn decode_into_rejects_wrong_buffer_size() {
    let (w, h) = (10u32, 10u32);
    let counts = vec![100u16; 100];
    let blob = encode(&counts, w, h, &opts()).unwrap();
    let mut small = vec![0u16; 99];
    assert!(decode_into(&blob, &mut small).is_err());
    let mut right = vec![0u16; 100];
    decode_into(&blob, &mut right).unwrap();
    assert_eq!(right, counts);
}

#[test]
fn encode_rejects_bad_arguments() {
    assert!(encode(&[0u16; 10], 3, 3, &opts()).is_err()); // len mismatch
    assert!(encode(&[], 0, 5, &opts()).is_err()); // zero dim
    // scale 0 / non-finite, over-long unit.
    let bad_scale = EncodeOptions {
        scale: 0.0,
        ..Default::default()
    };
    assert!(encode(&[0u16; 25], 5, 5, &bad_scale).is_err());
    let bad_unit = EncodeOptions {
        unit: "toolongunit".into(),
        ..Default::default()
    };
    assert!(encode(&[0u16; 25], 5, 5, &bad_unit).is_err());
}
