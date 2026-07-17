# depthpack — repo guide

Single-crate Rust library: a compact lossless / bounded-error codec for
16-bit depth maps (`u16` lattice, `0` = nodata) built from LiDAR projected
into equirectangular frames. MED (LOCO-I / JPEG-LS median edge detector)
prediction + zigzag residuals + a 1-bpp validity mask, both zstd-compressed.
Pure Rust and `wasm32`-clean by default; the `zstd-c` feature swaps in C
zstd for the encoder only. Read `README.md` for the full pitch, benchmark
numbers, and the on-disk format table.

## Layout

- `src/lib.rs` — the whole codec (~580 lines). Public API: `encode`,
  `decode` / `decode_into`, `decode_scaled` / `decode_scaled_into`,
  `decode_header`; types `EncodeOptions`, `Header`, `DepthImage`,
  `ScaledImage`, `Error`; format constants (`MAGIC`, `VERSION`,
  `HEADER_LEN`, `UNIT_LEN`, `MAX_PIXELS`). Internal helpers: `predict`,
  `zigzag`/`unzigzag`, `zstd_compress` (cfg-split by feature),
  `zstd_to_exact`.
- `tests/roundtrip.rs` — property/roundtrip coverage.
- `tests/robustness.rs` — malformed-input decoding must error, never panic.
- `tests/golden.rs` — a fixed blob decodes identically forever (format
  stability). Regenerate the blob with `cargo run --example gen_golden`
  and paste the `BLOB=` output back into the test.
- `examples/roundtrip.rs` — runnable usage demo.
- `examples/gen_golden.rs` — one-off generator for the golden test (not a
  real example; keep in sync with `tests/golden.rs`).
- `fuzz/` — `cargo-fuzz` harness; `fuzz_targets/decode.rs` fuzzes the
  decoder against arbitrary bytes.

## Invariants — do not break without a format version bump

- **Bit-exact lattice.** The codec never quantizes. Precision is the
  caller's choice of `scale` + pre-quantization. `decode` must reproduce
  the input `u16` lattice exactly.
- **`unit` is opaque.** Stored and returned verbatim (ASCII, ≤ `UNIT_LEN`
  bytes, null-padded). depthpack never converts between units.
- **Decoder never panics on malformed input.** Every section is validated
  against the header (dimension cap `MAX_PIXELS`, mask popcount vs
  `n_valid`, exact decompressed lengths) and returns `Error`. Any new
  decode path must uphold this — `tests/robustness.rs` + the fuzz target
  guard it.
- **Format is v1**, 46-byte little-endian header, two zstd frames (mask,
  then byte-plane-split zigzag residuals). Residuals use wrapping `u16`
  arithmetic so reconstruction is exact for any value/prediction pair.
  Changing the byte layout, `codec` id, or header means bumping `VERSION`
  and updating the format table in `README.md`.
- **Decode stays pure Rust** regardless of features — only the encoder's
  entropy stage is cfg-split for `zstd-c`. Keep `wasm32-unknown-unknown`
  building for both encode and decode on the default (no-feature) build.

## Commands

```sh
cargo test                        # default (pure Rust) features
cargo test --features zstd-c      # C-zstd encoder path
cargo build --release --target wasm32-unknown-unknown   # must stay green
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo clippy --all-targets --features zstd-c -- -D warnings
cargo run --example gen_golden    # regen golden blob after a format change
```

CI (`.github/workflows/ci.yaml`) runs exactly the above on push/PR. Clippy
is `-D warnings` — keep it clean. When touching the format, run *both*
feature sets and the wasm build locally before pushing.

## Releasing

See the **Releasing** section in `README.md`. In short: bump `version` in
`Cargo.toml`, then create a GitHub release `vX.Y.Z`. `publish.yaml`
publishes to crates.io via trusted publishing (OIDC) — no token needed.
