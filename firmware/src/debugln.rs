macro_rules! debugln {
	($dst:expr, $($arg:tt)*) => {{
		if cfg!(feature = "debugprint_basic") {
			writeln!($dst, $($arg)*).ok();
		}
	}}
}

macro_rules! debug {
	($dst:expr, $($arg:tt)*) => {{
		if cfg!(feature = "debugprint_basic") {
			write!($dst, $($arg)*).ok();
		}
	}}
}

pub(crate) use debugln;
pub(crate) use debug;
