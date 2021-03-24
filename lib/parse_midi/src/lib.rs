#![no_std]

fn is_realtime(byte: u8) -> bool {
	return byte & 0xF8 == 0xF8;
}
fn is_system_common(byte: u8) -> bool {
	return byte & 0xF8 == 0xF0 && byte != 0xF0;
}
fn is_start_of_sysex(byte: u8) -> bool {
	return byte == 0xF0;
}
fn is_channel(byte: u8) -> bool {
	return is_status(byte) && byte & 0xF0 != 0xF0;
}
fn is_status(byte: u8) -> bool {
	return byte & 0x80 != 0;
}
fn status_length(status: u8) -> u8 {
	assert!(is_status(status));
	let high = status & 0xF0;
	let low = status & 0x0F;
	if high == 0x80 || high == 0x90 || high == 0xA0 || high == 0xB0 || high ==0xE0 { return 3; }
	if high == 0xC0 || high == 0xD0 { return 2; }
	if high == 0xF0 {
		if low == 1 || low == 3 { return 2; }
		if low == 4 || low == 5 { return 3; } // not really defined in the standard but let's handle these nicely.
		if low == 2 { return 3; }
		if low == 6 || low == 7 { return 1; }
	}
	if status == 0xF0 { // sysex
		return 3;
	}
	return 1;
}

fn cin_byte(status: u8, datalen: u8, sysex_ends: bool) -> u8 {
	if is_start_of_sysex(status) && sysex_ends {
		match datalen {
			1 => 0x5,
			2 => 0x6,
			3 => 0x7,
			_ => panic!()
		}
	}
	else if is_start_of_sysex(status) && !sysex_ends {
		assert!(datalen == 3);
		0x4
	}
	else if is_system_common(status) {
		match datalen {
			1 => 0xF,
			2 => 0x2,
			3 => 0x3,
			_ => panic!()
		}
	}
	else if is_realtime(status) {
		assert!(datalen == 1);
		0xF
	}
	else if is_channel(status) {
		(status & 0xF0) >> 4
	}
	else {
		panic!()
	}
}

pub fn payload_length(firstbyte: u8) -> u8 {
	match firstbyte & 0x0F {
		0x0 | 0x1 => 0, // not in the standard
		0x5 | 0xF => 1,
		0x2 | 0x6 | 0xC | 0xD => 2,
		0x3 | 0x4 | 0x7 | 0x8 | 0x9 | 0xA | 0xB | 0xE => 3,
		_ => unreachable!()
	}
}

pub struct MidiToUsbParser {
	status: u8,
	data: [u8; 3],
	datalen: u8
}

impl MidiToUsbParser {
	pub fn new() -> MidiToUsbParser {
		MidiToUsbParser {
			status: 0,
			data: [0; 3],
			datalen: 0
		}
	}
	pub fn push(&mut self, byte: u8) -> Option<[u8; 4]> {
		let mut result = None;

		if is_status(byte) {
			debug_assert!(is_status(self.status) || self.status == 0);
			debug_assert!(!is_realtime(self.status));
			if is_realtime(byte) || byte == 0xF6 { // hack: tune request (0xF6) is handled like realtime messages for simplicity
				return Some([0x0F, byte, 0, 0]);
			}
			else {
				if self.status == 0xF0 { // a sysex has ended
					debug_assert!(self.datalen < 3);
					self.data[self.datalen as usize] = 0xF7; // insert end-of-sysex, regardless of whether this was an EOX or a different status byte
					self.datalen += 1;
					result = Some([cin_byte(self.status, self.datalen, true), self.data[0], self.data[1], self.data[2]]);
				}

				self.status = byte;
				self.data = [0; 3];
				self.datalen = 1;
				self.data[0] = byte;

				if self.status == 0xF7 { // "end-of-sysex" is not a valid running status
					self.status = 0;
				}
			}
		}
		else {
			if self.datalen == 0 && self.status != 0xF0 { // running status and not receiving a sysex?
				self.data[0] = self.status;
				self.datalen = 1;
			}
			self.data[self.datalen as usize] = byte;
			self.datalen += 1;
		}
		
		if self.status == 0 { // initial condition
			self.datalen = 0;
			self.data = [0; 3];
			return result;
		}

		debug_assert!(status_length(self.status) >= 2);
		if self.datalen >= status_length(self.status) {
			result = Some([cin_byte(self.status, self.datalen, false), self.data[0], self.data[1], self.data[2]]);
			self.datalen = 0;
			self.data = [0; 3];
		}
		return result;
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use rand_core::RngCore;

	#[test]
	fn withstands_garbage_input() {
		let mut rng = rand_pcg::Pcg32::new(0xcafef00dd15ea5e5, 0xa02bdbf7bb3c0a7);
		for _ in 0..10000 {
			let mut parser = MidiToUsbParser::new();
			for _ in 0..100 {
				parser.push(rng.next_u32() as u8);
			}
			// check if it's recovering
			parser.push(0xB0);
			assert!(parser.push(0x13) == None);
			assert!(parser.push(0x37) == Some([0x0B, 0xB0, 0x13, 0x37]));
		}
	}
	
	#[test]
	fn withstands_zero_input() {
		let mut parser = MidiToUsbParser::new();
		for _ in 0..100 {
			assert!(parser.push(0) == None);
		}
		// check if it's recovering
		assert!(parser.push(0xB0) == None);
		assert!(parser.push(0x13) == None);
		assert!(parser.push(0x37) == Some([0x0B, 0xB0, 0x13, 0x37]));
	}
	
	#[test]
	fn voice_messages_and_running_status() {
		let mut parser = MidiToUsbParser::new();
		let input = [
			0x84, 42, 52, 43, 53, // note off
			0x93, 13, 37, 11, 11, // note on
			0xA0, 12, 34, 56, 78, 9, 10, // poly aftertouch
			0xB0, 0, 0, 47, 11,  // control change
			0xC0, 64, 65, 66, // program change
			0xD0, 1, 2, 3, 4, // mono aftertouch
			0xE0, 11, 22, 33, 44, // pitch bend
		];

		let desired_output = [
			[0x08, 0x84, 42, 52],
			[0x08, 0x84, 43, 53],
			[0x09, 0x93, 13, 37],
			[0x09, 0x93, 11, 11],
			[0x0A, 0xA0, 12, 34],
			[0x0A, 0xA0, 56, 78],
			[0x0A, 0xA0, 9, 10],
			[0x0B, 0xB0, 0, 0],
			[0x0B, 0xB0, 47, 11],
			[0x0C, 0xC0, 64, 0],
			[0x0C, 0xC0, 65, 0],
			[0x0C, 0xC0, 66, 0],
			[0x0D, 0xD0, 1, 0],
			[0x0D, 0xD0, 2, 0],
			[0x0D, 0xD0, 3, 0],
			[0x0D, 0xD0, 4, 0],
			[0x0E, 0xE0, 11, 22],
			[0x0E, 0xE0, 33, 44],
		];

		for (actual, desired) in input.iter()
			.filter_map(|byte| parser.push(*byte))
			.zip(desired_output.iter())
		{
			assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
		}
	}

	#[test]
	fn realtime_works_and_does_not_interfere_with_running_status() {
		let mut parser = MidiToUsbParser::new();

		for msg in &[0xF8, 0xFA, 0xFB, 0xFC, 0xFE, 0xFF] {
			let input = [
				*msg, 0x84, *msg, 11, 11, *msg, 22, 22, *msg, 33, *msg, *msg, 33, *msg, // note off
			];

			let desired_output = [
				[0x0F, *msg, 0, 0],
				[0x0F, *msg, 0, 0],
				[0x08, 0x84, 11, 11],
				[0x0F, *msg, 0, 0],
				[0x08, 0x84, 22, 22],
				[0x0F, *msg, 0, 0],
				[0x0F, *msg, 0, 0],
				[0x0F, *msg, 0, 0],
				[0x08, 0x84, 33, 33],
				[0x0F, *msg, 0, 0],
			];

			for (actual, desired) in input.iter()
				.filter_map(|byte| parser.push(*byte))
				.zip(desired_output.iter())
			{
				assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
			}
		}
	}

	#[test]
	fn sysex() {
		let mut parser = MidiToUsbParser::new();

		let input = [
			0xF0, 0xF7,
			0xF0, 1, 0xF7,
			0xF0, 1, 2, 0xF7,
			0xF0, 1, 2, 3, 0xF7,
			0xF0, 1, 2, 3, 4, 0xF7,
			0xF0, 1, 2, 3, 4, 5, 0xF7,
			0xF0, 1, 2, 3, 4, 5, 6, 0xF7,
			0xF0, 1, 2, 3, 4, 5, 6, 7, 0xF7,
			0xF0, 1, 2, 3, 4, 5, 6, 7, 8, 0xF7,
			0xF0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xF7,
		];

		let desired_output = [
			[0x06, 0xF0, 0xF7, 0],
			[0x07, 0xF0, 1, 0xF7],
			[0x04, 0xF0, 1, 2], [0x05, 0xF7, 0, 0],
			[0x04, 0xF0, 1, 2], [0x06, 3, 0xF7, 0],
			[0x04, 0xF0, 1, 2], [0x07, 3, 4, 0xF7],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x05, 0xF7, 0, 0],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x06, 6, 0xF7, 0],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x07, 6, 7, 0xF7],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x04, 6, 7, 8], [0x05, 0xF7, 0, 0],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x04, 6, 7, 8], [0x06, 9, 0xF7, 0],
		];

		for (actual, desired) in input.iter()
			.filter_map(|byte| parser.push(*byte))
			.zip(desired_output.iter())
		{
			assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
		}
	}

	#[test]
	fn sysex_with_realtime() {
		let mut parser = MidiToUsbParser::new();

		let input = [
			0xF0, 0xF8, 0xF7,
			0xF0, 1, 0xF8, 2, 0xF7,
			0xF0, 1, 2, 0xF8, 3, 4, 5, 6, 0xF8, 7, 8, 9, 0xF7,
		];

		let desired_output = [
			[0x0F, 0xF8, 0, 0], [0x06, 0xF0, 0xF7, 0],
			[0x0F, 0xF8, 0, 0], [0x04, 0xF0, 1, 2], [0x05, 0xF7, 0, 0],
			[0x04, 0xF0, 1, 2], [0x0F, 0xF8, 0, 0], [0x04, 3, 4, 5], [0x0F, 0xF8, 0, 0], [0x04, 6, 7, 8], [0x06, 9, 0xF7, 0],
		];

		for (actual, desired) in input.iter()
			.filter_map(|byte| parser.push(*byte))
			.zip(desired_output.iter())
		{
			assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
		}
	}

	#[test]
	fn sysex_may_end_with_other_status() {
		let mut parser = MidiToUsbParser::new();

		let input = [
			0xF0,
			0xF0, 1,
			0xF0, 1, 2,
			0xF0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
			0xB0, 13, 37,
		];

		let desired_output = [
			[0x06, 0xF0, 0xF7, 0],
			[0x07, 0xF0, 1, 0xF7],
			[0x04, 0xF0, 1, 2], [0x05, 0xF7, 0, 0],
			[0x04, 0xF0, 1, 2], [0x04, 3, 4, 5], [0x04, 6, 7, 8], [0x06, 9, 0xF7, 0],
			[0x0B, 0xB0, 13, 37]
		];

		for (actual, desired) in input.iter()
			.filter_map(|byte| parser.push(*byte))
			.zip(desired_output.iter())
		{
			assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
		}
	}

	#[test]
	fn system_common() {
		let mut parser = MidiToUsbParser::new();
		let input = [
			0xF1, 42, // midi time code quarter frame
			0xF2, 13, 37, // song position pointer
			0xF3, 66, // song select
			0xF6, // tune request
			0xF6, // tune request
			0xF3, 67, // song select
		];

		let desired_output = [
			[0x02, 0xF1, 42, 0],
			[0x03, 0xF2, 13, 37],
			[0x02, 0xF3, 66, 0],
			[0x0F, 0xF6, 0, 0],
			[0x0F, 0xF6, 0, 0],
			[0x02, 0xF3, 67, 0],
		];

		for (actual, desired) in input.iter()
			.filter_map(|byte| parser.push(*byte))
			.zip(desired_output.iter())
		{
			assert!(actual == *desired, "expected {:02X?}, got {:02X?}", *desired, actual);
		}
	}
}
