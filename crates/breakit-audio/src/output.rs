//! Getting the mixer's samples out: the sound card, a WAV file, or nowhere.
//!
//! The game holds an [`AudioHandle`] (cheap to clone, `Send`) and sends
//! commands through it; the mixer lives on the audio thread inside an
//! [`AudioOutput`]. Keep the `AudioOutput` alive for as long as sound should
//! play. If there is no sound device (a headless container, CI) the handle
//! simply drops commands.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::{Command, Mixer};

/// Sends commands to the audio thread. A default handle is disconnected and
/// ignores everything.
#[derive(Clone, Debug, Default)]
pub struct AudioHandle {
    tx: Option<Sender<Command>>,
}

impl AudioHandle {
    pub fn send(&self, cmd: Command) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(cmd);
        }
    }

    pub fn is_connected(&self) -> bool {
        self.tx.is_some()
    }
}

/// Applies every queued command to the mixer. Never blocks.
fn drain(rx: &Receiver<Command>, mixer: &mut Mixer) {
    // Bounded so a flood cannot stall a callback.
    for _ in 0..512 {
        match rx.try_recv() {
            Ok(cmd) => mixer.handle(cmd),
            Err(_) => break,
        }
    }
}

/// Owns whatever is producing sound. Drop it to stop.
pub struct AudioOutput {
    #[cfg(feature = "device")]
    _stream: Option<cpal::Stream>,
    recorder: Option<Recorder>,
    pub sample_rate: u32,
    pub description: String,
}

struct Recorder {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for Recorder {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl AudioOutput {
    /// Opens the default output device. Fails if there is none.
    #[cfg(feature = "device")]
    pub fn open_device() -> Result<(AudioOutput, AudioHandle), String> {
        use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
        use cpal::SampleFormat;

        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .ok_or("no audio output device")?;
        let supported = device.default_output_config().map_err(|e| e.to_string())?;
        let format = supported.sample_format();
        let config: cpal::StreamConfig = supported.config();
        let sr = config.sample_rate;
        let channels = config.channels as usize;
        let (tx, rx) = mpsc::channel();
        let mixer = Mixer::new(sr as f32);

        fn build<T>(
            device: &cpal::Device,
            config: &cpal::StreamConfig,
            mut mixer: Mixer,
            rx: Receiver<Command>,
            channels: usize,
        ) -> Result<cpal::Stream, cpal::BuildStreamError>
        where
            T: cpal::SizedSample + cpal::FromSample<f32>,
        {
            let mut scratch: Vec<f32> = Vec::new();
            device.build_output_stream::<T, _, _>(
                config,
                move |data: &mut [T], _| {
                    drain(&rx, &mut mixer);
                    scratch.resize(data.len(), 0.0);
                    mixer.render(&mut scratch, channels);
                    for (d, s) in data.iter_mut().zip(&scratch) {
                        *d = T::from_sample(*s);
                    }
                },
                |e| eprintln!("breakit-audio: stream error: {e}"),
                None,
            )
        }

        let stream = match format {
            SampleFormat::F32 => build::<f32>(&device, &config, mixer, rx, channels),
            SampleFormat::I16 => build::<i16>(&device, &config, mixer, rx, channels),
            SampleFormat::U16 => build::<u16>(&device, &config, mixer, rx, channels),
            SampleFormat::I32 => build::<i32>(&device, &config, mixer, rx, channels),
            other => return Err(format!("unsupported sample format {other:?}")),
        }
        .map_err(|e| e.to_string())?;
        stream.play().map_err(|e| e.to_string())?;
        let description = format!("{sr} Hz, {channels} ch, {format:?}");
        Ok((
            AudioOutput {
                _stream: Some(stream),
                recorder: None,
                sample_rate: sr,
                description,
            },
            AudioHandle { tx: Some(tx) },
        ))
    }

    /// Runs the mixer on a thread at real-time pace and writes everything it
    /// plays to `path` (16-bit stereo WAV) when the output is dropped. For
    /// headless runs where there is no device to listen on.
    pub fn record_to(path: impl Into<PathBuf>, sample_rate: u32) -> (AudioOutput, AudioHandle) {
        let path = path.into();
        let (tx, rx) = mpsc::channel::<Command>();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let thread = std::thread::Builder::new()
            .name("breakit-audio-recorder".into())
            .spawn(move || {
                let mut mixer = Mixer::new(sample_rate as f32);
                let chunk = (sample_rate / 100) as usize; // 10 ms
                let mut buf = vec![0.0f32; chunk * 2];
                let mut all: Vec<f32> = Vec::new();
                let start = Instant::now();
                let mut rendered = 0u64;
                while !stop2.load(Ordering::Relaxed) {
                    drain(&rx, &mut mixer);
                    mixer.render(&mut buf, 2);
                    all.extend_from_slice(&buf);
                    rendered += chunk as u64;
                    let due = Duration::from_secs_f64(rendered as f64 / sample_rate as f64);
                    if let Some(wait) = due.checked_sub(start.elapsed()) {
                        std::thread::sleep(wait);
                    }
                }
                if let Err(e) = crate::wav::write_wav(&path, &all, 2, sample_rate) {
                    eprintln!("breakit-audio: could not write {}: {e}", path.display());
                }
            })
            .expect("spawn audio recorder");
        (
            AudioOutput {
                #[cfg(feature = "device")]
                _stream: None,
                recorder: Some(Recorder {
                    stop,
                    thread: Some(thread),
                }),
                sample_rate,
                description: "WAV recorder".into(),
            },
            AudioHandle { tx: Some(tx) },
        )
    }

    /// No sound at all; the handle drops commands.
    pub fn silent() -> (AudioOutput, AudioHandle) {
        (
            AudioOutput {
                #[cfg(feature = "device")]
                _stream: None,
                recorder: None,
                sample_rate: 48_000,
                description: "silent".into(),
            },
            AudioHandle::default(),
        )
    }

    /// Stops a recorder and writes its file. Devices just stop.
    pub fn finish(mut self) {
        self.recorder.take();
    }
}
