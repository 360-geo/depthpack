# depthpack

[![CI](https://github.com/360-geo/depthpack/actions/workflows/ci.yaml/badge.svg)](https://github.com/360-geo/depthpack/actions/workflows/ci.yaml)
[![Crates.io](https://img.shields.io/crates/v/depthpack.svg)](https://crates.io/crates/depthpack)
[![docs.rs](https://docs.rs/depthpack/badge.svg)](https://docs.rs/depthpack)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

Compact codec for 16-bit raster fields with nodata, as produced by
projecting LiDAR point clouds into panoramic (equirectangular) camera
frames. Pure Rust, `wasm32` compatible, dependency-light.

A blob stores an integer lattice of `u16` counts (`0` = nodata) plus a
physical mapping `value = count·scale + offset` and an opaque `unit`
label — so `decode_scaled` hands you metres (or any unit) as `f32`
directly. The encoder predicts each valid count from its causal
neighbours with the LOCO-I / JPEG-LS *median edge detector* (MED) and
compresses the residuals plus a 1-bpp validity mask with zstd. Depth maps
built from triangulated point clouds are mostly smooth surfaces, so the
residuals are near zero — which is why this outperforms both generic
image codecs and raster codecs like LERC on this data, on size *and*
speed.

## Numbers

Measured on a real 7200×3600 street-level depth pano (26 MP, 61% nodata,
Apple M-series, best of 3). All rows roundtrip-verified. The lattice here
is a millimetre grid (our pipeline's choice — the format is unit-neutral);
a coarser lattice is a smaller `scale`, and "err" is the reconstruction
error it implies. "Lossless" means the lattice roundtrips bit-exact.

| encoding                          | encode | decode | size    | err   |
|-----------------------------------|--------|--------|---------|-------|
| **depthpack, 1 mm lattice**       | 0.09 s | 123 ms | 5.38 MB | exact |
| **depthpack, 2 mm lattice**       | 0.11 s | 120 ms | 4.59 MB | 1 mm  |
| **depthpack, 10 mm lattice**      | 0.10 s | 102 ms | 2.94 MB | 5 mm  |
| WebP lossless (libwebp q75)       | 8.71 s | —      | 5.42 MB | exact |
| LERC (lerc-rs, tol 5 mm)          | 0.13 s | 63 ms  | 5.86 MB | 5 mm  |
| LERC (lerc-rs, lossless)          | 0.13 s | 63 ms  | 9.81 MB | exact |

Lossless, depthpack matches WebP's size at ~90× the encode speed. At a
5 mm error bound it is half the size of LERC. Encode rows use the
`zstd-c` feature; the default pure-Rust build encodes ~2.6× slower and
~27% larger (ruzstd's encoder compresses less densely than C zstd), and
decodes identically.

## Quick start

```toml
[dependencies]
depthpack = "0.1"
```

```rust
use depthpack::{encode, decode, decode_scaled, EncodeOptions};

// Row-major u16 lattice counts, 0 = nodata. Here: millimetres.
let counts: Vec<u16> = vec![2_500; 3600 * 1800];

// scale/offset/unit describe the physical meaning: value = count*scale + offset.
// A millimetre lattice read as metres is scale = 0.001.
let opts = EncodeOptions { scale: 0.001, unit: "m".into(), ..Default::default() };
let blob = encode(&counts, 3600, 1800, &opts)?;

// Raw lattice roundtrips bit-exact…
assert_eq!(decode(&blob)?.values, counts);

// …or get physical f32 straight out (NaN at nodata) — no conversion needed.
let scaled = decode_scaled(&blob)?;   // scaled.values in metres, scaled.unit == "m"
# Ok::<(), depthpack::Error>(())
```

The codec stores the lattice bit-exact and never quantizes — **you** pick
the precision by choosing `scale` and quantizing before encoding
(a 1 mm lattice read as metres is `scale = 0.001`; a 5 mm lattice is
`scale = 0.005`, error ≤ 2.5 mm). depthpack applies `scale`/`offset` on
decode but **never interprets `unit`**: a US survey-foot producer stores
foot counts with `unit = "ftUS"`, and no unit conversion ever happens
inside this crate.

## Reading a blob

Two representations, each with an allocating and a decode-into form. The
scaled forms are what a consumer usually wants — physical values, ready
to use, with no post-decode arithmetic:

| function | returns | nodata |
|----------|---------|--------|
| `decode` / `decode_into` | raw `u16` lattice counts | `0` |
| `decode_scaled` / `decode_scaled_into` | physical `f32` = `count·scale + offset` | `NaN` |
| `decode_header` | dimensions + `scale`, `offset`, `unit` (no pixels) | — |

`unit` is returned verbatim so the caller knows what the values *are*
(`"m"`, `"ftUS"`, …); depthpack never converts between units. The
`*_into` forms write into a caller-provided buffer of exactly
`width × height`, avoiding the output allocation — handy for streaming a
decoded field straight into a GPU texture:

```rust
let header = depthpack::decode_header(&blob)?;
let mut depth_m = vec![0f32; (header.width * header.height) as usize];
depthpack::decode_scaled_into(&blob, &mut depth_m)?; // physical f32, NaN = nodata
# Ok::<(), depthpack::Error>(())
```

## Builds

The crate is **pure Rust end to end by default** — encode *and* decode
compile for `wasm32-unknown-unknown` unchanged, with no C toolchain and
no JS interop, so a browser viewer decodes with the same code path.

For native bulk encoding, enable the `zstd-c` feature to swap the
encoder's entropy stage to the C `zstd` bindings (~2.6× faster encode,
~27% smaller output, honours `zstd_level`). Decoding stays pure Rust
regardless:

```toml
[dependencies]
depthpack = { version = "0.1", features = ["zstd-c"] }
```

## Format (v1)

Little-endian, 46-byte header, two zstd frames:

```text
offset  size  field
0       4     magic "DPCK"
4       1     version (1)
5       1     codec (0 = MED)
6       4     width (u32)
10      4     height (u32)
14      4     n_valid (u32) — number of valid pixels
18      8     scale (f64)   — physical = count*scale + offset
26      8     offset (f64)
34      8     unit (ASCII, null-padded; opaque, never interpreted)
42      4     mask_len (u32) — compressed length of the mask frame
46      *     zstd frame: validity mask, 1 bpp, row-major, LSB-first
46+*    *     zstd frame: MED residuals, zigzag, byte-plane split
              (all high bytes, then all low bytes)
```

Residuals are computed in wrapping u16 arithmetic and zigzag-mapped as
i16, so reconstruction is exact for *any* value/prediction pair, not
just the small residuals smooth data produces. The decoder validates
every section against the header (dimension cap, mask popcount vs
`n_valid`, exact decompressed lengths) and never panics on malformed
input — see `tests/robustness.rs` and the `fuzz/` harness.

## Non-goals

- Not a general image codec: it assumes `u16` + `0`-as-nodata and wins
  by exploiting smoothness. For photos, use an image codec.
- No ecosystem tooling (GDAL etc. cannot read it). The format is ~120
  lines to decode; keep the debug tools next to it.

## Related

- [360-geo/copc](https://github.com/360-geo/copc) — streaming COPC
  reader used by the pipelines that produce these depth maps.
- [LERC](https://github.com/Esri/lerc) — the raster codec we
  benchmarked against; excellent general-purpose choice when you don't
  control both ends of the pipe.

## Releasing

Releases are published to [crates.io](https://crates.io/crates/depthpack)
by `.github/workflows/publish.yaml`, which runs on a published GitHub
release and authenticates with crates.io via **trusted publishing**
(OIDC) — no API token required.

To cut a release:

1. Bump `version` in `Cargo.toml` (follow [SemVer](https://semver.org/))
   and, if the on-disk format changed, the format table above and
   `VERSION` in `src/lib.rs`. Commit and merge to `main`.
2. Make sure CI is green on `main` (fmt, clippy, tests, wasm build).
3. Create the release, tagging the version with a leading `v`:

   ```sh
   gh release create v0.1.0 --title v0.1.0 --notes "…" --target main
   ```

   The workflow derives the crate version from the tag (strips the `v`),
   so the tag must match `Cargo.toml`.
4. Watch the publish run and confirm the new version is live:

   ```sh
   gh run watch "$(gh run list --workflow=publish.yaml -L1 --json databaseId -q '.[0].databaseId')" --exit-status
   ```

Trusted publishing is configured under the crate's *Settings → Trusted
Publishing* on crates.io (repository `360-geo/depthpack`, workflow
`publish.yaml`). The very first `0.1.0` release predated that setup and
was published once with a bootstrap token; every release since uses OIDC.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms
or conditions.
