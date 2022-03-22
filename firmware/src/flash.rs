use crate::debugln::*;
use crate::preset::*;
use simple_flash_store::FlashStore;
use stm32f1xx_hal::flash::{FlashSize, SectorSize};

const SETTINGS_BASE: usize = (1 << 17) - 2 * 2048;

#[derive(Debug)]
pub enum SaveError {
	BufferTooSmall,
	NoSpaceLeft,
	CorruptData,
}

// consumes 4kb on the stack
pub fn save_preset_to_flash(
	preset_idx: u8,
	preset: &Preset,
	flash_store: &mut MyFlashStore,
	tx: &mut impl core::fmt::Write,
) -> Result<(), SaveError> {
	let mut buffer = [0; 1024];
	debugln!(tx, "saving preset {}", preset_idx);
	let len = serialize_preset(&mut None, preset);
	if len > 1024 {
		return Err(SaveError::BufferTooSmall);
	}
	debugln!(tx, "  -> serializing {} bytes", len);
	serialize_preset(&mut Some(&mut buffer[0..len]), preset);

	debugln!(tx, "  -> writing to flash");
	for i in 0..len {
		debug!(tx, "{:02X} ", buffer[i]);
	}
	debugln!(tx, "");
	let result = match flash_store.write_file(preset_idx, &buffer[0..len]) {
		Ok(()) => Ok(()),
		Err(FlashStoreError::CorruptData) => Err(SaveError::CorruptData),
		Err(FlashStoreError::NoSpaceLeft) => Err(SaveError::NoSpaceLeft),
		_ => {
			debugln!(tx, " -> cannot happen");
			unreachable!()
		}
	};
	debugln!(tx, "  -> {:?}", result);
	return result;
}

// consumes 1kb on the stack
pub fn read_preset_from_flash(
	preset_idx: u8,
	flash_store: &mut MyFlashStore,
	tx: &mut impl core::fmt::Write,
) -> Result<Preset, SettingsError> {
	debugln!(tx, "loading preset {}", preset_idx);
	let mut buffer = [0; 1024];

	match flash_store.read_file(preset_idx, &mut buffer) {
		Ok(data) => {
			debugln!(tx, "  -> Found file of length {}", data.len());
			for i in 0..data.len() {
				debug!(tx, "{:02X} ", data[i]);
			}
			debugln!(tx, "");
			let preset = parse_preset(data)?;
			debugln!(tx, "  -> Done");
			Ok(preset)
		}
		Err(FlashStoreError::NotFound) => {
			debugln!(tx, "  -> Not found");
			Ok(Preset::new())
		}
		Err(FlashStoreError::BufferTooSmall) => Err(SettingsError), // cannot happen, we are never writing files that large
		Err(FlashStoreError::CorruptData) => Err(SettingsError),
		Err(_) => unreachable!(),
	}
}

use simple_flash_store::*;

pub struct FlashAdapter {
	flash: stm32f1xx_hal::flash::Parts,
}

impl FlashAdapter {
	pub fn new(flash: stm32f1xx_hal::flash::Parts) -> FlashAdapter {
		FlashAdapter { flash }
	}
}

pub type MyFlashStore = FlashStore<FlashAdapter, 2048>;

impl FlashTrait for FlashAdapter {
	const SIZE: usize = 2048;
	const PAGE_SIZE: usize = 2048; // Some chips have 1k, others have 2k. We're being pessimistic here, which is why this size differs from FlashWriter's sector size.
	const WORD_SIZE: usize = 4;
	const ERASED_VALUE: u8 = 0xFF;

	fn erase_page(&mut self, page: usize) -> Result<(), FlashAccessError> {
		// Some chips have 1k, others have 2k. We're being pessimistic here again, which is why this size differs from PAGE_SIZE.
		let mut writer = self.flash.writer(SectorSize::Sz1K, FlashSize::Sz128K);
		writer
			.erase((SETTINGS_BASE + page) as u32, Self::PAGE_SIZE)
			.unwrap();
		Ok(())
	}

	fn read(&mut self, address: usize, data: &mut [u8]) -> Result<(), FlashAccessError> {
		let writer = self.flash.writer(SectorSize::Sz1K, FlashSize::Sz128K);
		data.copy_from_slice(
			writer
				.read((SETTINGS_BASE + address) as u32, data.len())
				.unwrap(),
		);
		Ok(())
	}

	fn write(&mut self, address: usize, data: &[u8]) -> Result<(), FlashAccessError> {
		let mut writer = self.flash.writer(SectorSize::Sz1K, FlashSize::Sz128K);
		if data.len() % 2 == 0 {
			writer
				.write((SETTINGS_BASE + address) as u32, data)
				.unwrap();
		}
		else {
			writer
				.write((SETTINGS_BASE + address) as u32, &data[0..(data.len() - 1)])
				.unwrap();
			writer
				.write(
					(SETTINGS_BASE + address + data.len() - 1) as u32,
					&[data[data.len() - 1], 0],
				)
				.unwrap();
		}
		Ok(())
	}
}
