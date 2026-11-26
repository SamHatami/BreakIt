//! What a playtest build needs beyond the game itself: a version string,
//! a log file, and panics written to that log.
//!
//! Release builds on Windows have no console (see `main.rs`), so the log
//! file is the only place errors and panics end up. It is `breakit.log`
//! in a `logs` folder next to the executable; the previous run is kept as
//! `breakit.prev.log`. If that folder cannot be written (the game was
//! unpacked somewhere read-only) it falls back to the system temp folder,
//! `%TEMP%\BreakIt\logs` on Windows. `BREAKIT_LOG_DIR` overrides both.

use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

use bevy::app::App;
use bevy::log::{tracing_subscriber, BoxedLayer, LogPlugin};

/// `0.1.0`, or the release name CI passed in (`v0.1.0`, `nightly-2026-11-26`).
pub fn version() -> &'static str {
    option_env!("BREAKIT_VERSION").unwrap_or(concat!("v", env!("CARGO_PKG_VERSION"), "-dev"))
}

/// Short commit hash CI built from, or `local`.
pub fn commit() -> &'static str {
    option_env!("BREAKIT_COMMIT").unwrap_or("local")
}

/// One line for the window title, the read-out and bug reports.
pub fn version_line() -> String {
    format!("BreakIt {} ({})", version(), commit())
}

/// The log file this run writes to, once [`log_plugin`] has opened it.
pub fn log_path() -> Option<&'static Path> {
    LOG_PATH.get().map(PathBuf::as_path)
}

static LOG_PATH: OnceLock<PathBuf> = OnceLock::new();

/// Bevy's log plugin with an extra layer that writes to the log file.
/// Console output stays as it was.
pub fn log_plugin() -> LogPlugin {
    LogPlugin {
        custom_layer: file_layer,
        ..Default::default()
    }
}

fn file_layer(_app: &mut App) -> Option<BoxedLayer> {
    let (path, mut file) = open_log()?;
    let _ = writeln!(
        file,
        "{}\nos: {} {}\n",
        version_line(),
        std::env::consts::OS,
        std::env::consts::ARCH
    );
    let _ = LOG_PATH.set(path);
    install_panic_hook();
    let layer = tracing_subscriber::fmt::layer()
        .with_ansi(false)
        .with_writer(Mutex::new(file));
    Some(Box::new(layer))
}

fn open_log() -> Option<(PathBuf, File)> {
    let mut dirs = Vec::new();
    if let Some(dir) = std::env::var_os("BREAKIT_LOG_DIR") {
        dirs.push(PathBuf::from(dir));
    }
    if let Some(exe_dir) = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(Path::to_path_buf))
    {
        dirs.push(exe_dir.join("logs"));
    }
    dirs.push(std::env::temp_dir().join("BreakIt").join("logs"));
    dirs.into_iter().find_map(|dir| {
        fs::create_dir_all(&dir).ok()?;
        let path = dir.join("breakit.log");
        if path.exists() {
            let _ = fs::rename(&path, dir.join("breakit.prev.log"));
        }
        // Append mode, so the panic hook's writes and the layer's never
        // overwrite each other.
        File::create(&path).ok()?;
        let file = OpenOptions::new().append(true).open(&path).ok()?;
        Some((path, file))
    })
}

/// Append panics, with a backtrace, to the log file before the default
/// hook runs: without a console the default hook's output is lost.
fn install_panic_hook() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if let Some(path) = log_path() {
            if let Ok(mut f) = OpenOptions::new().append(true).open(path) {
                let backtrace = std::backtrace::Backtrace::force_capture();
                let _ = writeln!(f, "\nPANIC in {}: {info}\n{backtrace}", version_line());
            }
        }
        previous(info);
    }));
}
