use super::draw_title;
use super::style::*;

use crate::user_input::UserInput;
use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::*, text::Text};

pub enum MenuAction {
	Activated(usize),
	Continue,
}

pub struct MenuState {
	selected: usize,
	redraw_pending: bool,
}

impl MenuState {
	pub fn new(selected: usize) -> MenuState {
		MenuState {
			selected,
			redraw_pending: true,
		}
	}

	pub fn schedule_redraw(&mut self) {
		self.redraw_pending = true;
	}

	pub fn process(
		&mut self,
		input: UserInput,
		title: &str,
		entries: &[&str],
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) -> MenuAction {
		if input.scroll != 0 {
			let old_selected = self.selected;
			self.selected =
				(self.selected as i16 + input.scroll).rem_euclid(entries.len() as i16) as usize;
			self.redraw_item(old_selected, entries[old_selected], draw_target);
			self.redraw_item(self.selected, entries[self.selected], draw_target);
		}
		if input.button_event {
			return MenuAction::Activated(self.selected);
		}

		if self.redraw_pending {
			draw_target.clear(Rgb565::BLACK).ok().unwrap();
			draw_title(title, draw_target);
			for (i, text) in entries.iter().enumerate() {
				self.redraw_item(i, text, draw_target);
			}
		}

		self.redraw_pending = false;

		MenuAction::Continue
	}

	fn redraw_item(
		&self,
		item: usize,
		entry: &str,
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) {
		let this_style = if item == self.selected {
			selected_style!()
		}
		else {
			normal_style!()
		};
		Text::new(
			entry,
			Point::new(20, 80 + 20 + 30 + 20 * item as i32),
			this_style,
		)
		.draw(draw_target)
		.ok()
		.unwrap();
	}
}
