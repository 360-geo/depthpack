//! # depthpack
//!
//! Compact codec for 16-bit raster fields with nodata — built for depth
//! maps produced by projecting LiDAR into panoramic (equirectangular)
//! camera frames.
//!
//! A depthpack blob stores an **integer lattice** of `u16` counts
//! (`0` = nodata) plus a physical mapping `value = count · scale +
//! offset` and an opaque `unit` label. The encoder predicts each valid
//! count from its causal neighbours with the LOCO-I / JPEG-LS *median
//! edge detector* (MED) and compresses the zigzag residuals plus a
//! 1-bpp validity mask with zstd. Smooth surfaces — the dominant content
//! of a depth map built from a triangulated point cloud — leave
//! near-zero residuals, which is why this beats generic image codecs on
//! both size and speed for this data.
//!
//! The lattice roundtrips **bit-exact**; the codec never quantizes. Pick
//! your precision by choosing `scale` and quantizing before you encode
//! (e.g. millimetres → `scale = 0.001`, `unit = "m"`; a 5 mm lattice →
//! `scale = 0.005`). depthpack applies `scale`/`offset` on
//! [`decode_scaled`] but **never interprets `unit`** — it carries the
//! label verbatim, so a survey-foot producer stores foot counts with
//! `unit = "ftUS"` and no unit conversion happens anywhere in this crate.
//!
//! ## Feature flags
//!
//! The crate is **pure Rust by default** — both encode and decode
//! compile for `wasm32-unknown-unknown` unchanged (zstd via [`ruzstd`]).
//!
//! - `zstd-c` *(optional)* — swap the encoder's entropy stage to the C
//!   `zstd` bindings. Measured on a real 7200×3600 depth map: ~2.6×
//!   faster encode and ~27% smaller output than the pure-Rust encoder
//!   (ruzstd's `Fastest` compresses less densely than C zstd level 1),
//!   and it honours [`EncodeOptions::zstd_level`]. Recommended for
//!   native bulk pipelines; decoding always stays pure Rust either way.
//!
//! ## Example
//!
//! ```
//! let (w, h) = (64u32, 32u32);
//! // A smooth surface (millimetre counts) with a nodata hole.
//! let counts: Vec<u16> = (0..w * h)
//!     .map(|i| if i % 97 == 0 { 0 } else { 2500 + (i % w) as u16 })
//!     .collect();
//!
//! // Millimetre lattice interpreted as metres.
//! let opts = depthpack::EncodeOptions { scale: 0.001, unit: "m".into(), ..Default::default() };
//! let blob = depthpack::encode(&counts, w, h, &opts).unwrap();
//!
//! // Raw lattice roundtrips bit-exact…
//! let img = depthpack::decode(&blob).unwrap();
//! assert_eq!(img.values, counts);
//!
//! // …and decode_scaled applies scale, NaN for nodata.
//! let scaled = depthpack::decode_scaled(&blob).unwrap();
//! assert_eq!(scaled.unit, "m");
//! assert!((scaled.values[1] - 2.501).abs() < 1e-6); // count 2501 · 0.001 m
//! assert!(scaled.values[0].is_nan()); // nodata
//! ```

use std::io::Read;

/// Magic bytes at the start of every blob.
pub const MAGIC: [u8; 4] = *b"DPCK";
/// Container version this library reads and writes.
pub const VERSION: u8 = 1;
/// Fixed header length in bytes.
pub const HEADER_LEN: usize = 46;
/// Maximum length of the [`unit`](EncodeOptions::unit) label in bytes.
pub const UNIT_LEN: usize = 8;
/// Upper bound on `width × height` accepted by the decoder, so a
/// malformed header cannot request an arbitrarily large allocation.
/// 2²⁷ px comfortably covers 7200×3600 panoramas with a wide margin.
pub const MAX_PIXELS: u64 = 1 << 27;

/// Codec identifier stored in the header. Only MED is defined for v1.
const CODEC_MED: u8 = 0;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Errors returned by [`encode`] and the decode functions.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// Input slice length does not match `width × height`.
    #[error("buffer has {got} values, expected width*height = {expected}")]
    BadDimensions { got: usize, expected: u64 },
    /// Width or height is zero, or the pixel count exceeds [`MAX_PIXELS`].
    #[error("unsupported dimensions {width}x{height}")]
    UnsupportedDimensions { width: u32, height: u32 },
    /// `scale` must be finite and non-zero; `offset` must be finite.
    #[error("scale must be finite and non-zero, offset finite")]
    BadScale,
    /// The `unit` label is longer than [`UNIT_LEN`] bytes or not ASCII.
    #[error("unit label must be <= {UNIT_LEN} ASCII bytes")]
    BadUnit,
    /// The blob is not a depthpack container (bad magic or too short).
    #[error("not a depthpack blob")]
    NotDepthpack,
    /// The container version or codec id is not supported by this build.
    #[error("unsupported version {version} / codec {codec}")]
    Unsupported { version: u8, codec: u8 },
    /// The blob is structurally invalid: truncated sections, section
    /// lengths that disagree with the header, residual counts that
    /// disagree with the mask, or undecodable zstd frames.
    #[error("corrupt blob: {0}")]
    Corrupt(&'static str),
}

/// Options for [`encode`].
#[derive(Debug, Clone)]
pub struct EncodeOptions {
    /// Physical units per lattice count: `value = count · scale + offset`.
    /// A millimetre lattice read as metres is `scale = 0.001`. Must be
    /// finite and non-zero. Default `1.0` (counts are their own value).
    pub scale: f64,
    /// Physical offset added after scaling. Depth is a range from the
    /// camera, so this is normally `0.0`. Must be finite. Nodata (`0`)
    /// decodes to `NaN` regardless of `offset`.
    pub offset: f64,
    /// Opaque unit label, up to [`UNIT_LEN`] ASCII bytes (e.g. `"m"`,
    /// `"ftUS"`). Stored and returned verbatim; depthpack never converts
    /// between units. Default empty (raw counts, no physical unit).
    pub unit: String,
    /// zstd compression level for the mask and residual streams.
    /// Level 1 is the measured sweet spot: higher levels buy ~3% size
    /// for ~3× the encode time on real depth maps.
    ///
    /// Only honoured with the `zstd-c` feature; the default pure-Rust
    /// encoder always compresses at its `Fastest` (≈ level 1) setting.
    pub zstd_level: i32,
}

impl Default for EncodeOptions {
    fn default() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
            unit: String::new(),
            zstd_level: 1,
        }
    }
}

/// Parsed container header. Obtain with [`decode_header`].
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Number of valid (non-nodata) pixels.
    pub n_valid: u32,
    /// Physical units per lattice count (`value = count · scale + offset`).
    pub scale: f64,
    /// Physical offset (see [`EncodeOptions::offset`]).
    pub offset: f64,
    /// Opaque unit label (see [`EncodeOptions::unit`]).
    pub unit: String,
}

/// A decoded raster: the raw integer lattice plus its physical mapping.
/// `values` is row-major, `0` = nodata.
#[derive(Debug, Clone, PartialEq)]
pub struct DepthImage {
    pub width: u32,
    pub height: u32,
    /// Raw lattice counts, `0` = nodata.
    pub values: Vec<u16>,
    pub scale: f64,
    pub offset: f64,
    pub unit: String,
}

/// A decoded raster in physical units: `count · scale + offset`, with
/// `NaN` at nodata pixels. Returned by [`decode_scaled`].
#[derive(Debug, Clone, PartialEq)]
pub struct ScaledImage {
    pub width: u32,
    pub height: u32,
    /// Physical values, row-major, `NaN` = nodata.
    pub values: Vec<f32>,
    pub unit: String,
}

// ---------------------------------------------------------------------------
// Shared primitives
// ---------------------------------------------------------------------------

/// MED / LOCO-I predictor over the valid lattice. `0` marks an absent
/// (nodata or out-of-image) neighbour; with no valid neighbour the
/// previous decoded value continues the scan context.
#[inline(always)]
fn predict(l: u16, u: u16, ul: u16, prev: u16) -> u16 {
    match (l != 0, u != 0) {
        (true, true) => {
            if ul != 0 {
                let (l, u, ul) = (l as i32, u as i32, ul as i32);
                let mx = l.max(u);
                let mn = l.min(u);
                (if ul >= mx {
                    mn
                } else if ul <= mn {
                    mx
                } else {
                    l + u - ul
                }) as u16
            } else {
                ((l as u32 + u as u32) / 2) as u16
            }
        }
        (true, false) => l,
        (false, true) => u,
        (false, false) => prev,
    }
}

/// Zigzag-map a wrapped (mod 2¹⁶) residual so small magnitudes of either
/// sign get small codes. Exact for the full i16 range.
#[inline(always)]
fn zigzag(d: i16) -> u16 {
    (((d as i32) << 1) ^ ((d as i32) >> 31)) as u16
}

#[inline(always)]
fn unzigzag(z: u16) -> i16 {
    (((z >> 1) as i32) ^ -((z & 1) as i32)) as i16
}

fn validate_dims(width: u32, height: u32) -> Result<usize, Error> {
    let n = width as u64 * height as u64;
    if width == 0 || height == 0 || n > MAX_PIXELS {
        return Err(Error::UnsupportedDimensions { width, height });
    }
    Ok(n as usize)
}

/// Encode a `unit` string into a fixed [`UNIT_LEN`]-byte, null-padded
/// field. Rejects non-ASCII or over-length labels.
fn encode_unit(unit: &str) -> Result<[u8; UNIT_LEN], Error> {
    let bytes = unit.as_bytes();
    if bytes.len() > UNIT_LEN || !unit.is_ascii() {
        return Err(Error::BadUnit);
    }
    let mut field = [0u8; UNIT_LEN];
    field[..bytes.len()].copy_from_slice(bytes);
    Ok(field)
}

/// Decode a null-padded [`UNIT_LEN`]-byte field back to a `String`.
fn decode_unit(field: &[u8]) -> Result<String, Error> {
    let end = field.iter().position(|&b| b == 0).unwrap_or(field.len());
    // The label is written ASCII; anything else is a corrupt header.
    if !field[..end].is_ascii() || field[end..].iter().any(|&b| b != 0) {
        return Err(Error::Corrupt("unit label not ascii / not null-padded"));
    }
    Ok(String::from_utf8_lossy(&field[..end]).into_owned())
}

// ---------------------------------------------------------------------------
// Encode
// ---------------------------------------------------------------------------

#[cfg(feature = "zstd-c")]
fn zstd_compress(buf: &[u8], level: i32) -> Result<Vec<u8>, Error> {
    zstd::bulk::compress(buf, level).map_err(|_| Error::Corrupt("zstd compress"))
}

#[cfg(not(feature = "zstd-c"))]
fn zstd_compress(buf: &[u8], _level: i32) -> Result<Vec<u8>, Error> {
    Ok(ruzstd::encoding::compress_to_vec(
        buf,
        ruzstd::encoding::CompressionLevel::Fastest,
    ))
}

/// Encode a row-major lattice of `u16` counts (`0` = nodata).
///
/// The caller owns quantization: `count = round((value − offset) /
/// scale)`, clamped to `1..=65535` for valid samples so a valid value
/// never collapses onto the nodata sentinel. The returned blob is
/// self-describing; decode it with [`decode`] or [`decode_scaled`].
pub fn encode(
    counts: &[u16],
    width: u32,
    height: u32,
    opts: &EncodeOptions,
) -> Result<Vec<u8>, Error> {
    let n = validate_dims(width, height)?;
    if counts.len() != n {
        return Err(Error::BadDimensions {
            got: counts.len(),
            expected: n as u64,
        });
    }
    if !opts.scale.is_finite() || opts.scale == 0.0 || !opts.offset.is_finite() {
        return Err(Error::BadScale);
    }
    let unit = encode_unit(&opts.unit)?;

    // Validity mask, 1 bpp, LSB-first within each byte.
    let mut mask = vec![0u8; n.div_ceil(8)];
    let mut n_valid: u32 = 0;
    for (i, &v) in counts.iter().enumerate() {
        if v != 0 {
            mask[i / 8] |= 1 << (i % 8);
            n_valid += 1;
        }
    }

    // MED prediction over valid pixels, zigzag residuals.
    let (w, h) = (width as usize, height as usize);
    let mut res: Vec<u16> = Vec::with_capacity(n_valid as usize);
    let mut prev: u16 = 0;
    for y in 0..h {
        let row = y * w;
        for x in 0..w {
            let i = row + x;
            let v = counts[i];
            if v == 0 {
                continue;
            }
            let l = if x > 0 { counts[i - 1] } else { 0 };
            let u = if y > 0 { counts[i - w] } else { 0 };
            let ul = if x > 0 && y > 0 { counts[i - w - 1] } else { 0 };
            // Residual in wrapping u16 space, reinterpreted as i16: the
            // decoder adds it back with wrapping_add, so reconstruction
            // is exact for any (value, prediction) pair — not just the
            // small residuals smooth data produces.
            res.push(zigzag(v.wrapping_sub(predict(l, u, ul, prev)) as i16));
            prev = v;
        }
    }

    // Byte-plane split: on smooth data the high plane is almost all
    // zeros, which zstd's RLE path erases nearly for free.
    let m = res.len();
    let mut planes = Vec::with_capacity(m * 2);
    planes.extend(res.iter().map(|&v| (v >> 8) as u8));
    planes.extend(res.iter().map(|&v| (v & 0xFF) as u8));

    let zm = zstd_compress(&mask, opts.zstd_level)?;
    let zr = zstd_compress(&planes, opts.zstd_level)?;

    let mut out = Vec::with_capacity(HEADER_LEN + zm.len() + zr.len());
    out.extend_from_slice(&MAGIC);
    out.push(VERSION);
    out.push(CODEC_MED);
    out.extend_from_slice(&width.to_le_bytes());
    out.extend_from_slice(&height.to_le_bytes());
    out.extend_from_slice(&n_valid.to_le_bytes());
    out.extend_from_slice(&opts.scale.to_le_bytes());
    out.extend_from_slice(&opts.offset.to_le_bytes());
    out.extend_from_slice(&unit);
    out.extend_from_slice(&(zm.len() as u32).to_le_bytes());
    out.extend_from_slice(&zm);
    out.extend_from_slice(&zr);
    Ok(out)
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// Parse and validate the fixed header without decoding pixel data.
pub fn decode_header(blob: &[u8]) -> Result<Header, Error> {
    if blob.len() < HEADER_LEN || blob[0..4] != MAGIC {
        return Err(Error::NotDepthpack);
    }
    let version = blob[4];
    let codec = blob[5];
    if version != VERSION || codec != CODEC_MED {
        return Err(Error::Unsupported { version, codec });
    }
    let width = u32::from_le_bytes(blob[6..10].try_into().unwrap());
    let height = u32::from_le_bytes(blob[10..14].try_into().unwrap());
    let n_valid = u32::from_le_bytes(blob[14..18].try_into().unwrap());
    let scale = f64::from_le_bytes(blob[18..26].try_into().unwrap());
    let offset = f64::from_le_bytes(blob[26..34].try_into().unwrap());
    let unit = decode_unit(&blob[34..42])?;
    if !scale.is_finite() || scale == 0.0 || !offset.is_finite() {
        return Err(Error::Corrupt("non-finite or zero scale/offset"));
    }
    let n = validate_dims(width, height)?;
    if n_valid as usize > n {
        return Err(Error::Corrupt("n_valid exceeds pixel count"));
    }
    Ok(Header {
        width,
        height,
        n_valid,
        scale,
        offset,
        unit,
    })
}

/// Decompress one zstd section into an exactly-`expected`-byte buffer.
/// Anything shorter, longer, or undecodable is an error — the expected
/// size comes from the validated header, so memory stays bounded no
/// matter what the frame itself claims.
fn zstd_to_exact(src: &[u8], expected: usize, what: &'static str) -> Result<Vec<u8>, Error> {
    let mut dec = ruzstd::decoding::StreamingDecoder::new(src).map_err(|_| Error::Corrupt(what))?;
    let mut out = vec![0u8; expected];
    dec.read_exact(&mut out).map_err(|_| Error::Corrupt(what))?;
    // The frame must end exactly here.
    let mut probe = [0u8; 1];
    match dec.read(&mut probe) {
        Ok(0) => Ok(out),
        _ => Err(Error::Corrupt(what)),
    }
}

/// Decode a depthpack blob into a fresh [`DepthImage`] (raw lattice +
/// physical mapping).
pub fn decode(blob: &[u8]) -> Result<DepthImage, Error> {
    let hdr = decode_header(blob)?;
    let n = hdr.width as usize * hdr.height as usize;
    let mut values = vec![0u16; n];
    decode_into(blob, &mut values)?;
    Ok(DepthImage {
        width: hdr.width,
        height: hdr.height,
        values,
        scale: hdr.scale,
        offset: hdr.offset,
        unit: hdr.unit,
    })
}

/// Decode into a caller-provided lattice buffer of exactly `width ×
/// height` counts, avoiding the output allocation. Nodata pixels are
/// written as `0`.
pub fn decode_into(blob: &[u8], out: &mut [u16]) -> Result<Header, Error> {
    let hdr = decode_header(blob)?;
    let (w, h) = (hdr.width as usize, hdr.height as usize);
    let n = w * h;
    if out.len() != n {
        return Err(Error::BadDimensions {
            got: out.len(),
            expected: n as u64,
        });
    }

    let mask_zlen = u32::from_le_bytes(blob[42..46].try_into().unwrap()) as usize;
    let body = &blob[HEADER_LEN..];
    if mask_zlen > body.len() {
        return Err(Error::Corrupt("mask section overruns blob"));
    }
    let mask = zstd_to_exact(&body[..mask_zlen], n.div_ceil(8), "mask")?;

    // The mask's population count must match the residual count claimed
    // by the header — this is what makes the reconstruction loop below
    // infallible. Trailing bits past `n` must be zero.
    let mut popcount: u64 = mask.iter().map(|b| b.count_ones() as u64).sum();
    let tail_bits = mask.len() * 8 - n;
    if tail_bits > 0 {
        let tail = mask[mask.len() - 1] >> (8 - tail_bits);
        if tail != 0 {
            return Err(Error::Corrupt("mask has bits past the last pixel"));
        }
        popcount -= tail.count_ones() as u64;
    }
    if popcount != hdr.n_valid as u64 {
        return Err(Error::Corrupt("mask popcount disagrees with n_valid"));
    }

    let m = hdr.n_valid as usize;
    let planes = zstd_to_exact(&body[mask_zlen..], m * 2, "residuals")?;

    // Reconstruct: same scan order and predictor as the encoder.
    out.fill(0);
    let mut k = 0usize;
    let mut prev: u16 = 0;
    for y in 0..h {
        let row = y * w;
        for x in 0..w {
            let i = row + x;
            if mask[i / 8] & (1 << (i % 8)) == 0 {
                continue;
            }
            let l = if x > 0 { out[i - 1] } else { 0 };
            let u = if y > 0 { out[i - w] } else { 0 };
            let ul = if x > 0 && y > 0 { out[i - w - 1] } else { 0 };
            let p = predict(l, u, ul, prev);
            let z = ((planes[k] as u16) << 8) | planes[m + k] as u16;
            let v = p.wrapping_add(unzigzag(z) as u16);
            out[i] = v;
            prev = v;
            k += 1;
        }
    }
    Ok(hdr)
}

/// Decode a blob directly to physical values (`count · scale + offset`),
/// with `NaN` at nodata pixels. depthpack applies the scale but never
/// the unit — the returned [`ScaledImage::unit`] is the stored label.
pub fn decode_scaled(blob: &[u8]) -> Result<ScaledImage, Error> {
    let hdr = decode_header(blob)?;
    let n = hdr.width as usize * hdr.height as usize;
    let mut values = vec![0f32; n];
    decode_scaled_into(blob, &mut values)?;
    Ok(ScaledImage {
        width: hdr.width,
        height: hdr.height,
        values,
        unit: hdr.unit,
    })
}

/// Decode physical values into a caller-provided `f32` buffer of exactly
/// `width × height` (`NaN` = nodata). Uses an internal `u16` scratch for
/// the lattice reconstruction, then applies `scale`/`offset`.
pub fn decode_scaled_into(blob: &[u8], out: &mut [f32]) -> Result<Header, Error> {
    let hdr = decode_header(blob)?;
    let n = hdr.width as usize * hdr.height as usize;
    if out.len() != n {
        return Err(Error::BadDimensions {
            got: out.len(),
            expected: n as u64,
        });
    }
    let mut lattice = vec![0u16; n];
    decode_into(blob, &mut lattice)?;
    let (scale, offset) = (hdr.scale, hdr.offset);
    for (dst, &c) in out.iter_mut().zip(lattice.iter()) {
        *dst = if c == 0 {
            f32::NAN
        } else {
            (c as f64 * scale + offset) as f32
        };
    }
    Ok(hdr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zigzag_roundtrips_full_i16_range() {
        for d in i16::MIN..=i16::MAX {
            assert_eq!(unzigzag(zigzag(d)), d, "d={d}");
        }
    }

    /// Wrapping residual + wrapping reconstruction is exact for every
    /// (prediction, value) pair, by construction.
    #[test]
    fn wrapping_residual_reconstructs_any_pair() {
        for p in [0u16, 1, 2, 255, 256, 32_767, 32_768, 65_534, 65_535] {
            for v in [0u16, 1, 2, 255, 256, 32_767, 32_768, 65_534, 65_535] {
                let d = v.wrapping_sub(p) as i16;
                assert_eq!(p.wrapping_add(unzigzag(zigzag(d)) as u16), v, "p={p} v={v}");
            }
        }
    }

    #[test]
    fn unit_field_roundtrips() {
        for u in ["", "m", "ftUS", "12345678"] {
            assert_eq!(decode_unit(&encode_unit(u).unwrap()).unwrap(), u);
        }
        assert!(matches!(encode_unit("123456789"), Err(Error::BadUnit))); // 9 > 8
        assert!(matches!(encode_unit("mé"), Err(Error::BadUnit))); // non-ascii
    }

    #[test]
    fn header_rejects_garbage() {
        assert!(matches!(decode_header(b"nope"), Err(Error::NotDepthpack)));
        assert!(matches!(
            decode_header(&[0u8; HEADER_LEN]),
            Err(Error::NotDepthpack)
        ));
    }
}
