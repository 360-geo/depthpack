//! Encode/decode demo on a synthetic depth pano. Prints sizes and
//! timings for a few quantization steps.
//!
//! Run: cargo run --release --example roundtrip [--features zstd-c]

use std::time::Instant;

use depthpack::{EncodeOptions, decode_scaled, encode};

fn main() {
    let (w, h) = (3600u32, 1800u32);

    // Synthetic street scene: nodata sky, smooth ground, a few walls.
    let depth_mm: Vec<u16> = (0..w * h)
        .map(|i| {
            let (x, y) = (i % w, i / w);
            if y < h / 2 {
                0 // sky
            } else {
                let ground = 30_000 - (y - h / 2) * 30;
                let wall = if (x / 600) % 2 == 0 {
                    8_000 + x % 600
                } else {
                    ground
                };
                ground.min(wall).max(500) as u16
            }
        })
        .collect();

    // Quantize the mm field to `step`-mm bins (round, clamp valid to >= 1),
    // and set scale so decode_scaled yields metres. step 1 = lossless mm.
    for step in [1u16, 2, 5, 10] {
        let counts: Vec<u16> = depth_mm
            .iter()
            .map(|&mm| {
                if mm == 0 {
                    0
                } else {
                    ((mm as u32 + step as u32 / 2) / step as u32).clamp(1, 65_535) as u16
                }
            })
            .collect();
        let opts = EncodeOptions {
            scale: step as f64 * 1e-3, // metres per count
            unit: "m".into(),
            ..Default::default()
        };

        let t = Instant::now();
        let blob = encode(&counts, w, h, &opts).unwrap();
        let enc_s = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let scaled = decode_scaled(&blob).unwrap();
        let dec_s = t.elapsed().as_secs_f64();

        // Reconstruction error against the original mm field, in mm.
        let max_err = depth_mm
            .iter()
            .zip(&scaled.values)
            .filter(|&(&mm, _)| mm != 0)
            .map(|(&mm, &m)| ((mm as f64) - m as f64 * 1000.0).abs())
            .fold(0.0f64, f64::max);

        println!(
            "step {step:>2} mm: {:>7.2} KB  encode {:>5.1} ms  decode {:>5.1} ms  max err {max_err:.1} mm",
            blob.len() as f64 / 1e3,
            enc_s * 1e3,
            dec_s * 1e3,
        );
    }
}
