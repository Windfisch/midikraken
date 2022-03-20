use crate::machine::reset_to_bootloader;

pub struct BootloaderSysexStatemachine {
	index: usize
}

impl BootloaderSysexStatemachine {
	pub fn new() -> BootloaderSysexStatemachine {
		BootloaderSysexStatemachine { index: 0 }
	}

	pub fn push(&mut self, byte: u8) {
		let sysex = [0xF0u8, 0x00, 0x37, 0x64, 0x00, 0x00, 0x7f, 0x1e, 0x3a, 0x62];

		if byte == sysex[self.index] {
			self.index += 1;

			if self.index == sysex.len() {
				unsafe {
					reset_to_bootloader();
				}
			}
		}
		else {
			self.index = 0;
		}
	}
}
