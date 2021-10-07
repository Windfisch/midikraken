pub struct DynArray <const SIZE: usize> {
	data: [u8; SIZE]
}

impl <const SIZE: usize> DynArray<SIZE> {
	fn find_offset(&self, index: usize) -> Option<usize> {
		let mut curr_index = 0;
		let mut offset = 0;
		while curr_index != index {
			curr_index += 1;
			offset += 2 + self.length_at_offset(offset);
			if offset >= self.data.len() {
				return None;
			}
		}
		return Some(offset);
	}

	fn length_at_offset(&self, offset: usize) -> usize {
		use core::convert::TryInto;
		let chunk = &self.data[offset..];
		u16::from_le_bytes(chunk[0..2].try_into().unwrap()) as usize
	}

	fn move_left(&mut self, from: usize, to: usize) {
		assert!(to < from);
		for i in 0..(self.data.len() - from) {
			self.data[to+i] = self.data[from+i];
		}
		for i in (to + self.data.len() - from) .. self.data.len() {
			self.data[i] = 0;
		}
	}

	fn move_right(&mut self, from: usize, to: usize) {
		assert!(to > from);
		for i in (to..self.data.len()).rev() {
			self.data[i] = self.data[i - to + from];
		}
	}

	pub fn resize(&mut self, index: usize, new_size: usize) -> Result<(),()> {
		if let Some(offset_to_resize) = self.find_offset(index) {
			let old_size = self.length_at_offset(offset_to_resize);
			let old_offset_next = offset_to_resize + 2 + old_size;
			let new_offset_next = offset_to_resize + 2 + new_size;
			if new_size == old_size {
				return Ok(());
			}
			else if new_size < old_size {
				self.move_left(old_offset_next, new_offset_next);
			}
			else { // new_size > old_size
				let trim_len = new_size - old_size;
				let clear = self.data[ (self.data.len() - trim_len) .. self.data.len() ].iter().all(|v| *v==0);
				if !clear {
					return Err(());
				}
				self.move_right(old_offset_next, new_offset_next);
			}

			// update the length field
			self.data[offset_to_resize..(offset_to_resize+2)].copy_from_slice(&(new_size as u16).to_le_bytes());
			return Ok(());
		}
		else {
			return Err(());
		}
	}

	pub fn get_mut(&mut self, index: usize) -> Option<&mut [u8]> {
		if let Some(offset) = self.find_offset(index) {
			let length = self.length_at_offset(offset);
			Some(&mut self.data[(offset+2) .. (offset+2+length)])
		}
		else {
			None
		}
	}

	pub fn get(&self, index: usize) -> Option<&[u8]> {
		if let Some(offset) = self.find_offset(index) {
			let length = self.length_at_offset(offset);
			Some(&self.data[(offset+2) .. (offset+2+length)])
		}
		else {
			None
		}
	}

	pub fn new() -> DynArray<SIZE> {
		DynArray { data: [0; SIZE] }
	}

	pub fn from_raw(bytes: &[u8]) -> DynArray<SIZE> {
		let mut result = DynArray::new();
		result.set_raw(bytes);
		return result;
	}

	pub fn set_raw(&mut self, bytes: &[u8]) {
		use core::convert::TryInto;
		let mut offset = 0;
		loop {
			let chunk = &bytes[offset..];
			let length = u16::from_le_bytes(chunk[0..2].try_into().unwrap()) as usize;
			let new_offset = offset + 2 + length;
			if new_offset + 2 >= self.data.len() {
				self.data[0..offset].copy_from_slice(&bytes[0..offset]);
				return;
			}
			offset = new_offset;
		}
	}

	pub fn raw(&self) -> &[u8; SIZE] {
		&self.data
	}
}


