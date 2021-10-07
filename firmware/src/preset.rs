#[derive(Copy,Clone,PartialEq)]
pub enum EventRouteMode {
	None,
	Keyboard,
	Controller,
	Both
}

impl EventRouteMode {
	pub fn from_u8(val: u8) -> Result<EventRouteMode,()> {
		match val {
			0 => Ok(EventRouteMode::None),
			1 => Ok(EventRouteMode::Keyboard),
			2 => Ok(EventRouteMode::Controller),
			3 => Ok(EventRouteMode::Both),
			_ => Err(())
		}
	}

	pub fn to_u8(&self) -> u8 {
		match self {
			EventRouteMode::None => 0,
			EventRouteMode::Keyboard => 1,
			EventRouteMode::Controller => 2,
			EventRouteMode::Both => 3
		}
	}
}

pub type EventRoutingTable = [[EventRouteMode; 8]; 8];
pub type ClockRoutingTable = [[u8; 8]; 8];

#[derive(Clone, Copy)]
pub struct Preset {
	pub event_routing_table: EventRoutingTable,
	pub clock_routing_table: ClockRoutingTable,
	pub mode_mask: u32
}

impl Preset {
	pub const fn new() -> Preset {
		Preset {
			event_routing_table: [[EventRouteMode::None; 8]; 8],
			clock_routing_table: [[0; 8]; 8],
			mode_mask: 0
		}
	}
}

use core::ops::{Range, RangeFrom};
fn slice(data: &[u8], range: Range<usize>) -> Result<&[u8], SettingsError> {
	if range.start < data.len() && range.end < data.len() {
		Ok(&data[range])
	}
	else {
		Err(SettingsError)
	}
}
fn slice_from(data: &[u8], offset: usize) -> Result<&[u8], SettingsError> {
	if offset < data.len() {
		Ok(&data[offset..])
	}
	else {
		Err(SettingsError)
	}
}

pub fn parse_preset(data: &[u8]) -> Result<Preset, SettingsError> {
	let mut preset = Preset::new();
	let mut offset = 0;

	offset += parse_routing_matrix(&mut preset, slice_from(data, offset)?)?;
	offset += parse_trs_mode(&mut preset, slice_from(data, offset)?)?;

	Ok(preset)
}

fn parse_routing_matrix(preset: &mut Preset, data: &[u8]) -> Result<usize, SettingsError> {
	const LEN_ENTRY: usize = 7;

	if data.len() < 1 {
		return Err(SettingsError);
	}

	let n_entries = data[0] as usize;

	for chunk in slice(data, 1..1+LEN_ENTRY*n_entries)?.chunks_exact(LEN_ENTRY) {
		let x = chunk[0] & 0x0F;
		let y = (chunk[0] & 0xF0) >> 4;
		let clock_divisor = chunk[1] & 0x0F;
		let event_routing = (chunk[1] & 0xF0) >> 4;
		let _channel_mask = &chunk[2..=3];
		let _map_channel = chunk[4];
		let _note_split = chunk[5];
		let _transpose = chunk[6];

		preset.clock_routing_table[x as usize][y as usize] = clock_divisor;
		preset.event_routing_table[x as usize][y as usize] = EventRouteMode::from_u8(event_routing)?;
	}

	Ok(1 + LEN_ENTRY * n_entries)
}

fn parse_trs_mode(preset: &mut Preset, data: &[u8]) -> Result<usize, SettingsError> {
	use core::convert::TryInto;
	preset.mode_mask = u32::from_le_bytes(slice(data, 0..4)?.try_into().unwrap());
	Ok(4)
}

/** It is expected that if data_opt == Some(data), then data must be large enough to hold the whole
  * serialized data. Call this function with data_opt == None first to find out the required size. */
pub fn serialize_preset(data_opt: &mut Option<&mut [u8]>, preset: &Preset) -> usize {
	let mut bytes_written = 0;

	bytes_written += serialize_routing(data_opt, preset);
	bytes_written += serialize_trs_mode(data_opt, preset);

	return bytes_written;
}

fn serialize_routing(data_opt: &mut Option<&mut [u8]>, preset: &Preset) -> usize {
	let mut n_entries = 0;
	assert!(preset.event_routing_table.len() == preset.clock_routing_table.len());
	assert!(preset.event_routing_table[0].len() == preset.clock_routing_table[0].len());
	for x in 0..preset.event_routing_table.len() {
		for y in 0..preset.event_routing_table[0].len() {
			if preset.event_routing_table[x][y] != EventRouteMode::None || preset.clock_routing_table[x][y] != 0 {
				if let Some(ref mut data) = data_opt {
					let offset = 1 + 7 * n_entries;
					 data[offset+0] = ((x as u8) & 0x0F) | (((y as u8) & 0x0F) << 4);
					 data[offset+1] = preset.clock_routing_table[x][y] | (preset.event_routing_table[x][y].to_u8() << 4);
					 data[(offset+2)..=(offset+6)].fill(0);
				}
				n_entries += 1;
			}
		}
	}
	assert!(n_entries < 256); // FIXME
	if let Some(ref mut data) = data_opt {
		data[0] = n_entries as u8;
	}

	return 1 + 7 * n_entries;
}

fn serialize_trs_mode(data_opt: &mut Option<&mut [u8]>, preset: &Preset) -> usize {
	if let Some(ref mut data) = data_opt {
		data[0..4].copy_from_slice(&preset.mode_mask.to_be_bytes());
	}
	return 4;
}

#[derive(Debug)]
pub struct SettingsError;
impl From<core::array::TryFromSliceError> for SettingsError {
	fn from(_: core::array::TryFromSliceError) -> SettingsError { SettingsError{} }
}
impl From<stm32f1xx_hal::flash::Error> for SettingsError {
	fn from(_: stm32f1xx_hal::flash::Error) -> SettingsError { SettingsError{} }
}
impl From<()> for SettingsError {
	fn from(_: ()) -> SettingsError { SettingsError{} }
}
