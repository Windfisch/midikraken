use super::style::*;
use super::draw_title;

use crate::str_writer::*;
use core::fmt::Write;
use embedded_graphics::{
	pixelcolor::Rgb565,
	prelude::*,
	text::Text,
	draw_target::DrawTarget,
};

use crate::preset::{serialize_preset, Preset};
pub struct MainScreenState {
	redraw_pending: bool,
	last_preset: usize,
	last_dirty: bool,
	dirty_blinking: u32,
	preset_size: usize,
}

impl MainScreenState {
	const DIRTY_BLINK_TIME: u32 = 1300;
	const DIRTY_BLINK_N: u32 = 4;
	const DIRTY_BLINK_MOD: u32 = Self::DIRTY_BLINK_TIME / (2 * Self::DIRTY_BLINK_N);

	pub fn new() -> MainScreenState {
		MainScreenState {
			redraw_pending: true,
			last_preset: 0,
			last_dirty: false,
			dirty_blinking: 0,
			preset_size: 0,
		}
	}

	pub fn blink_dirty(&mut self) {
		self.dirty_blinking = Self::DIRTY_BLINK_MOD * 2 * Self::DIRTY_BLINK_N;
	}

	pub fn schedule_redraw(&mut self) {
		self.redraw_pending = true;
	}

	pub fn process(
		&mut self,
		preset_idx: usize,
		preset: &Preset,
		dirty: bool,
		flash_usage: (usize, usize),
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) {
		let mut buf = [0; 80];
		let style = normal_style!();

		if self.redraw_pending {
			draw_target.clear(Rgb565::BLACK).ok().unwrap();
			draw_title("Midikraken", draw_target);
			Text::new("Current preset:", Point::new(20, 80 + 50), style)
				.draw(draw_target)
				.ok()
				.unwrap();
			Text::new(
				slfmt!(&mut buf, "Total used: {}/{}b", flash_usage.0, flash_usage.1),
				Point::new(20, 80 + 230),
				style,
			)
			.draw(draw_target)
			.ok()
			.unwrap();
		}

		if self.last_preset != preset_idx || self.redraw_pending {
			Text::new(
				slfmt!(&mut buf, "{}  ", preset_idx),
				Point::new(180, 80 + 50),
				style,
			)
			.draw(draw_target)
			.ok()
			.unwrap();
			self.preset_size = serialize_preset(&mut None, preset);
			Text::new(
				slfmt!(&mut buf, "Preset size:{:4} bytes", self.preset_size),
				Point::new(20, 80 + 210),
				style,
			)
			.draw(draw_target)
			.ok()
			.unwrap();
		}

		let dirty_blink_rem = self.dirty_blinking % Self::DIRTY_BLINK_MOD;
		let dirty_blink_redraw =
			dirty_blink_rem % (Self::DIRTY_BLINK_MOD / 2) == (Self::DIRTY_BLINK_MOD / 2 - 1);
		let highlight_dirty = dirty_blink_rem >= Self::DIRTY_BLINK_MOD / 2;
		if self.last_dirty != dirty || self.redraw_pending || dirty_blink_redraw {
			Text::new(
				if dirty { "(unsaved)" } else { "         " },
				Point::new(40, 80 + 70),
				if highlight_dirty {
					selected_style!()
				}
				else {
					normal_style!()
				},
			)
			.draw(draw_target)
			.ok()
			.unwrap();
		}

		if self.dirty_blinking > 0 {
			self.dirty_blinking -= 1;
		}

		self.last_dirty = dirty;
		self.last_preset = preset_idx;
		self.redraw_pending = false;
	}
}

