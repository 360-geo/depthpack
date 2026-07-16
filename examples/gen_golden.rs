//! One-off generator for tests/golden.rs (not shipped as an example).
fn main() {
    let (w, h) = (16u32, 8u32);
    let depth: Vec<u16> = (0..w * h)
        .map(|i| {
            let (x, y) = (i % w, i / w);
            if y == 0 || (x == 5 && y < 4) {
                0
            } else {
                (1000 + x * 7 + y * 130) as u16
            }
        })
        .collect();
    let opts = depthpack::EncodeOptions {
        scale: 0.001,
        unit: "m".into(),
        ..Default::default()
    };
    let blob = depthpack::encode(&depth, w, h, &opts).unwrap();
    print!("BLOB=");
    for b in &blob {
        print!("{b:02x}");
    }
    println!();
    print!("PIXELS=");
    for d in &depth {
        print!("{d},");
    }
    println!();
}
