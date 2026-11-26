//! Minimal WAV writer (16-bit PCM), for offline renders and headless capture.

use std::io::{self, Write};
use std::path::Path;

/// Writes interleaved `samples` as 16-bit PCM.
pub fn write_wav(path: &Path, samples: &[f32], channels: u16, sample_rate: u32) -> io::Result<()> {
    let mut f = io::BufWriter::new(std::fs::File::create(path)?);
    write_wav_to(&mut f, samples, channels, sample_rate)?;
    f.flush()
}

pub fn write_wav_to<W: Write>(
    w: &mut W,
    samples: &[f32],
    channels: u16,
    sample_rate: u32,
) -> io::Result<()> {
    let data_len = (samples.len() * 2) as u32;
    let block_align = channels * 2;
    w.write_all(b"RIFF")?;
    w.write_all(&(36 + data_len).to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&1u16.to_le_bytes())?;
    w.write_all(&channels.to_le_bytes())?;
    w.write_all(&sample_rate.to_le_bytes())?;
    w.write_all(&(sample_rate * block_align as u32).to_le_bytes())?;
    w.write_all(&block_align.to_le_bytes())?;
    w.write_all(&16u16.to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data_len.to_le_bytes())?;
    for &s in samples {
        let v = (s.clamp(-1.0, 1.0) * 32767.0).round() as i16;
        w.write_all(&v.to_le_bytes())?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn header_is_44_bytes() {
        let mut buf = Vec::new();
        super::write_wav_to(&mut buf, &[0.0, 0.5, -0.5, 1.0], 2, 48_000).unwrap();
        assert_eq!(buf.len(), 44 + 8);
        assert_eq!(&buf[0..4], b"RIFF");
    }
}
