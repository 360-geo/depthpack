//! Fuzz the decoder: any input must return Ok/Err without panicking or
//! blowing up memory. Run with `cargo +nightly fuzz run decode`.
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = depthpack::decode(data);
});
