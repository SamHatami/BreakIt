//! Indexes the recordings under `samples/` so that dropping a WAV file into
//! a timbre's folder is all it takes to use it. Writes `sample_index.rs`:
//! one `(folder, bytes)` entry per file, folders relative to `samples/`.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect(&path, out);
        } else if path
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("wav"))
        {
            out.push(path);
        }
    }
}

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let samples = root.join("samples");
    println!("cargo:rerun-if-changed={}", samples.display());
    let mut files = Vec::new();
    collect(&samples, &mut files);
    files.sort();
    let mut code = String::from("pub static FILES: &[(&str, &[u8])] = &[\n");
    for file in &files {
        let folder = file
            .parent()
            .and_then(|p| p.strip_prefix(&samples).ok())
            .map(|p| p.to_string_lossy().replace('\\', "/"))
            .unwrap_or_default();
        println!("cargo:rerun-if-changed={}", file.display());
        let _ = writeln!(
            code,
            "    ({folder:?}, include_bytes!({:?})),",
            file.to_string_lossy()
        );
    }
    code.push_str("];\n");
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("out dir")).join("sample_index.rs");
    std::fs::write(out, code).expect("write sample index");
}
