#[allow(unused_macros)]
macro_rules! debug {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		use core::fmt::Write;
		use stm32f1xx_hal::{serial, stm32};

		if cfg!(feature = "debugprint_basic") {
			#[allow(unused_unsafe)]
			let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
			write!(tx, $($arg)*).ok();
		}
	}}
}
#[allow(unused_macros)]
macro_rules! debugln {
	($($arg:tt)*) => {{
		use core::mem::MaybeUninit;
		use stm32f1xx_hal::{serial, stm32};
		use core::fmt::Write;

		if cfg!(feature = "debugprint_basic") {
			#[allow(unused_unsafe)]
			let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
			writeln!(tx, $($arg)*).ok();
		}
	}}
}

pub(crate) use debug;
pub(crate) use debugln;
