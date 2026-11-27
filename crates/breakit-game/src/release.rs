//! What a playtest build needs beyond the game itself: a version string,
//! a log file, and panics written to that log.
//!
//! Release builds on Windows have no console (see `main.rs`), so the log
//! file is the only place errors and panics end up. It is `breakit.log`
//! in a `logs` folder next to the executable; the previous run is kept as
//! `breakit.prev.log`. If that folder cannot be written (the game was
//! unpacked somewhere read-only) it falls back to the system temp folder,
//! `%TEMP%\BreakIt\logs` on Windows. `BREAKIT_LOG_DIR` overrides both.
//!
//! CI builds also look once, in the background, for a newer build on the
//! GitHub Releases page (see [`readout_line`]). Nightly builds compare
//! against the `nightly` release's commit, numbered builds against the
//! latest numbered release. It shells out to `curl` (part of Windows 10 and
//! later), so the game needs no HTTP library. Local builds never check;
//! `BREAKIT_NO_UPDATE_CHECK=1` turns it off.

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

/// The read-out's first line: [`version_line`], plus a note once the
/// background check has found a newer build. The first call starts the
/// check.
pub fn readout_line() -> String {
    match update_notice() {
        Some(note) => format!("{}\n{note}", version_line()),
        None => version_line(),
    }
}

static UPDATE: Mutex<Option<String>> = Mutex::new(None);
static UPDATE_CHECK: std::sync::Once = std::sync::Once::new();

/// "Update available: ..." once the check found one.
pub fn update_notice() -> Option<String> {
    UPDATE_CHECK.call_once(start_update_check);
    UPDATE.lock().ok()?.clone()
}

const REPO_API: &str = "https://api.github.com/repos/SamHatami/BreakIt/releases";

fn start_update_check() {
    if option_env!("BREAKIT_COMMIT").is_none()
        || std::env::var_os("BREAKIT_NO_UPDATE_CHECK").is_some()
    {
        return;
    }
    let nightly = version().starts_with("nightly");
    let url = if nightly {
        format!("{REPO_API}/tags/nightly")
    } else {
        format!("{REPO_API}/latest")
    };
    let spawned = std::thread::Builder::new()
        .name("update-check".into())
        .spawn(move || {
            let Some(body) = fetch(&url) else {
                bevy::log::info!("update check: no answer from {url}");
                return;
            };
            let note = if nightly {
                newer_nightly(&body, commit())
            } else {
                newer_release(&body, version())
            };
            bevy::log::info!(
                "update check: {}",
                note.as_deref().unwrap_or("this is the newest build")
            );
            if let (Some(note), Ok(mut slot)) = (note, UPDATE.lock()) {
                *slot = Some(note);
            }
        });
    if let Err(e) = spawned {
        bevy::log::warn!("update check: could not start: {e}");
    }
}

/// GET `url` with curl; `None` on any failure.
fn fetch(url: &str) -> Option<String> {
    let mut cmd = std::process::Command::new("curl");
    cmd.args([
        "-fsSL",
        "--max-time",
        "10",
        "-H",
        "Accept: application/vnd.github+json",
        "-H",
        "User-Agent: BreakIt-update-check",
        url,
    ]);
    #[cfg(windows)]
    {
        // No console window flashing up from the windowed game.
        use std::os::windows::process::CommandExt;
        const CREATE_NO_WINDOW: u32 = 0x0800_0000;
        cmd.creation_flags(CREATE_NO_WINDOW);
    }
    let out = cmd.output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).into_owned())
}

/// The `nightly` release is named "BreakIt nightly (abc1234)". A different
/// commit than ours means main has moved on.
fn newer_nightly(body: &str, ours: &str) -> Option<String> {
    let name = json_string(body, "name")?;
    let theirs = name.rsplit_once('(')?.1.strip_suffix(')')?;
    (theirs != ours && !theirs.is_empty())
        .then(|| format!("Update available: nightly ({theirs}), see the Releases page"))
}

/// The latest numbered release, by tag.
fn newer_release(body: &str, ours: &str) -> Option<String> {
    let tag = json_string(body, "tag_name")?;
    (tag != ours && tag.starts_with('v'))
        .then(|| format!("Update available: {tag}, see the Releases page"))
}

/// The first string value of `"key"` in a JSON text. Enough for the few
/// top-level fields read here; not a JSON parser.
fn json_string(body: &str, key: &str) -> Option<String> {
    let pat = format!("\"{key}\"");
    let after = &body[body.find(&pat)? + pat.len()..];
    let after = after.trim_start().strip_prefix(':')?.trim_start();
    let mut chars = after.strip_prefix('"')?.chars();
    let mut out = String::new();
    while let Some(c) = chars.next() {
        match c {
            '"' => return Some(out),
            '\\' => out.push(chars.next()?),
            c => out.push(c),
        }
    }
    None
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

#[cfg(test)]
mod tests {
    use super::*;

    const NIGHTLY: &str = r#"{"url": "x", "author": {"login": "github-actions[bot]"},
        "tag_name": "nightly", "name": "BreakIt nightly (49bba27)",
        "assets": [{"name": "BreakIt-nightly-windows-x64.zip"}]}"#;

    #[test]
    fn reads_fields() {
        assert_eq!(json_string(NIGHTLY, "tag_name").as_deref(), Some("nightly"));
        assert_eq!(
            json_string(NIGHTLY, "name").as_deref(),
            Some("BreakIt nightly (49bba27)")
        );
        assert_eq!(
            json_string(r#"{"name":"a\"b"}"#, "name").as_deref(),
            Some("a\"b")
        );
        assert_eq!(json_string("{}", "name"), None);
    }

    #[test]
    fn nightly_compares_commits() {
        assert_eq!(newer_nightly(NIGHTLY, "49bba27"), None);
        assert!(newer_nightly(NIGHTLY, "1234567")
            .unwrap()
            .contains("49bba27"));
    }

    #[test]
    fn release_compares_tags() {
        let body = r#"{"tag_name": "v0.2.0", "name": "BreakIt v0.2.0"}"#;
        assert_eq!(newer_release(body, "v0.2.0"), None);
        assert!(newer_release(body, "v0.1.0").unwrap().contains("v0.2.0"));
    }
}
