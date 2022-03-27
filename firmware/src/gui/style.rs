macro_rules! title_style {
	() => {{
		use embedded_graphics::{mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder}, pixelcolor::Rgb565};
		MonoTextStyleBuilder::new()
			.font(&FONT_9X15)
			.text_color(Rgb565::WHITE)
			.background_color(Rgb565::new(8, 16, 8))
			.build()
	}};
}

macro_rules! normal_style {
	() => {{
		use embedded_graphics::{mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder}, pixelcolor::Rgb565};
		MonoTextStyleBuilder::new()
			.font(&FONT_9X15)
			.text_color(Rgb565::WHITE)
			.background_color(Rgb565::BLACK)
			.build()
	}};
}

macro_rules! selected_style {
	() => {{
		use embedded_graphics::{mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder}, pixelcolor::Rgb565};
		MonoTextStyleBuilder::new()
			.font(&FONT_9X15)
			.text_color(Rgb565::WHITE)
			.background_color(Rgb565::RED)
			.build()
	}};
}

macro_rules! active_style {
	() => {{
		use embedded_graphics::{mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder}, pixelcolor::Rgb565};
		MonoTextStyleBuilder::new()
			.font(&FONT_9X15)
			.text_color(Rgb565::WHITE)
			.background_color(Rgb565::BLUE)
			.build()
	}};
}

//pub(super) use title_style;
pub(super) use normal_style;
pub(super) use selected_style;
pub(super) use active_style;
