//! General-purpose GUI library
//!
//! This module and its submodules offer general-purpose GUI screens, such as:
//! - Menus
//! - Editable grids
//! - Message screens
//! - And the main screen
//!
//! All types here follow a simple pattern:
//! `process()` needs to be called periodically, with the display resource and the backing data ("model" in the model/view analogy) as arguments.
//! `process()` will return whether an action (e.g. selecting a menu entry, changing data etc) has been performed.
//! Also `schedule_redraw()`  will set a flag which the next `process()` will honor.

mod message;
mod mainscreen;
mod grid;
mod menu;
#[macro_use]
mod style;

pub use message::*;
pub use mainscreen::*;
pub use grid::*;
pub use menu::*;

use embedded_graphics::{
	pixelcolor::Rgb565,
	prelude::*,
	text::Text,
	draw_target::DrawTarget,
};

fn draw_title(
	title: &str,
	draw_target: &mut impl DrawTarget<Color = Rgb565>,
) {
	use embedded_graphics::geometry::*;
	use embedded_graphics::primitives::*;
	Rectangle::new(Point::new(0, 80 + 7), Size::new(240, 17))
		.into_styled(
			PrimitiveStyleBuilder::new()
				.fill_color(Rgb565::new(8, 16, 8))
				.build(),
		)
		.draw(draw_target)
		.ok()
		.unwrap();

	Text::new(title, Point::new(20, 80 + 20), title_style!())
		.draw(draw_target)
		.ok()
		.unwrap();
}

