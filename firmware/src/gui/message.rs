use super::draw_title;
use super::style::*;

use crate::user_input::UserInput;
use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::*, text::Text};

use super::MenuAction;

pub enum MessageAction {
	None,
	ClearFlash,
}

pub struct MessageState {
	message: &'static [&'static str],
	button: &'static str,
	redraw_pending: bool,
	pub action: MessageAction,
}

impl MessageState {
	pub fn new(
		message: &'static [&'static str],
		button: &'static str,
		action: MessageAction,
	) -> MessageState {
		MessageState {
			message,
			button,
			action,
			redraw_pending: true,
		}
	}

	pub fn process(
		&mut self,
		input: UserInput,
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) -> MenuAction {
		if input.button_event {
			return MenuAction::Activated(0);
		}
		let style = normal_style!();
		if self.redraw_pending {
			draw_target.clear(Rgb565::BLACK).ok().unwrap();
			draw_title("Message", draw_target);
			for (i, text) in self.message.iter().enumerate() {
				Text::new(text, Point::new(20, 80 + 50 + 18 * (i as i32)), style)
					.draw(draw_target)
					.ok()
					.unwrap();
			}
			Text::new(self.button, Point::new(10, 80 + 230), selected_style!())
				.draw(draw_target)
				.ok()
				.unwrap();
			self.redraw_pending = false;
		}
		MenuAction::Continue
	}
}
