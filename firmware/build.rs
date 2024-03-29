use std::env;
use std::fs;
use std::io::Write;
use std::path::PathBuf;

use vergen::{vergen, Config};

fn main() {
	// Put the linker script somewhere the linker can find it
	let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
	fs::File::create(out_dir.join("memory.x"))
		.unwrap()
		.write_all(if cfg!(feature = "bootloader") {
			include_bytes!("memory-bootloader.x")
		}
		else {
			include_bytes!("memory-nobootloader.x")
		})
		.unwrap();
	println!("cargo:rustc-link-search={}", out_dir.display());
	println!("cargo:rerun-if-changed=memory-nobootloader.x");
	println!("cargo:rerun-if-changed=memory-bootloader.x");

	vergen(Config::default()).unwrap();
}
