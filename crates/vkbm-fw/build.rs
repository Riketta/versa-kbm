//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.

use std::{
    env,
    fs,
    io,
    path::PathBuf,
    process,
};

fn main() {
    if let Err(e) = run() {
        eprintln!("vkbm-fw build script failed: {e}");
        process::exit(1);
    }
}

fn run() -> io::Result<()> {
    let out_dir = env::var_os("OUT_DIR")
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "OUT_DIR is not set"))?;
    let out = PathBuf::from(out_dir);

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path. Re-run whenever it changes so a memory
    // layout edit reliably relinks the firmware.
    fs::write(out.join("memory.x"), include_bytes!("memory.x"))?;
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    Ok(())
}
