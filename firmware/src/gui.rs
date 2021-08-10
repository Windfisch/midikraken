use crate::str_writer::*;
use core::fmt::Write;
use embedded_graphics::{
	mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder},
		pixelcolor::Rgb565,
		prelude::*,
		text::Text,
};

enum GridEditingMode {
	Selecting,
	Dialing
}

pub enum GridAction {
	Continue,
	Exit,
	EnterMenu(usize, usize)
}

pub struct GridState<T, const COLS: usize, const ROWS: usize> {
	position: i16,
	state: GridEditingMode,
	redraw_pending: bool,
	_t: core::marker::PhantomData<T>
}

#[derive(PartialEq)]
enum Entry {
	Element(usize, usize),
	Back
}


impl<T, const COLS: usize, const ROWS: usize> GridState<T, COLS, ROWS> {
	pub fn new() -> GridState<T, COLS, ROWS> {
		GridState {
			position: 0,
			state: GridEditingMode::Selecting,
			redraw_pending: true,
			_t: core::marker::PhantomData{}
		}
	}

	fn entry(position: i16) -> Entry {
		if position == (ROWS*COLS) as i16 {
			Entry::Back
		}
		else {
			Entry::Element(position as usize % COLS, position as usize / COLS)
		}
	}

	pub fn schedule_redraw(&mut self) {
		self.redraw_pending = true;
	}

	fn draw_update<STRINGIFY>(
		&self,
		entry: Entry,
		selected: bool,
		data: &mut [[T; ROWS]; COLS],
		stringify: &STRINGIFY,
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	)
		where for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str
	{
		let buf = [0u8; 20];

		let style =
			if !selected {
				MonoTextStyleBuilder::new().font(&FONT_9X15).text_color(Rgb565::WHITE).background_color(Rgb565::BLACK).build()
			}
			else {
				match self.state {
					GridEditingMode::Selecting => MonoTextStyleBuilder::new().font(&FONT_9X15).text_color(Rgb565::WHITE).background_color(Rgb565::RED).build(),
					GridEditingMode::Dialing => MonoTextStyleBuilder::new().font(&FONT_9X15).text_color(Rgb565::WHITE).background_color(Rgb565::BLUE).build()
				}
			};

		match entry {
			Entry::Element(x, y) => {
				Text::new(stringify(&data[x][y], &buf), Point::new(30 + 20*(x as i32), 80+60 + 20*(y as i32)), style).draw(draw_target).ok().unwrap();
			}
			Entry::Back => {
				Text::new("Back", Point::new(10, 80+230), style).draw(draw_target).ok().unwrap();
			}
		}
		
	}

	pub fn fully_redraw<STRINGIFY>(
		&self,
		data: &mut [[T; ROWS]; COLS],
		stringify: &STRINGIFY,
		title: &str,
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	)
		where for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str
	{
		draw_target.clear(Rgb565::BLACK).ok().unwrap();

		let style = MonoTextStyleBuilder::new().font(&FONT_9X15).text_color(Rgb565::WHITE).background_color(Rgb565::BLACK).build();

		Text::new(title, Point::new(20, 80 + 20), style).draw(draw_target).ok().unwrap();

		let mut buf = [0u8; 20];
		for x in 0..(COLS as i32) {
			Text::new(slfmt!(&mut buf, "{}", x+1), Point::new(30 + 20*x, 80+40), style).draw(draw_target).ok().unwrap();
		}
		for y in 0..(ROWS as i32) {
			Text::new(slfmt!(&mut buf, "{}", y+1), Point::new(10, 80+60 + 20*y), style).draw(draw_target).ok().unwrap();
		}
		for x in 0..COLS {
			for y in 0..ROWS {
				self.draw_update(Entry::Element(x, y), (x,y)==(0,0), data, stringify, draw_target);
			}
		}
		self.draw_update(Entry::Back, false, data, stringify, draw_target);
	}

	pub fn process<STRINGIFY>(
		&mut self,
		scroll: i16,
		press: bool,
		long_press: bool,
		data: &mut [[T; ROWS]; COLS],
		increment: impl Fn(&mut T, i16),
		stringify: STRINGIFY,
		allow_dialing: bool,
		title: &str,
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	) -> GridAction
		where for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str
	{
		if self.redraw_pending {
			self.fully_redraw(data, &stringify, title, draw_target);
			self.redraw_pending = false;
		}

		let selected = Self::entry(self.position);

		match self.state {
			GridEditingMode::Selecting => {
				match selected {
					Entry::Element(x, y) => {
						if press {
							match allow_dialing {
								true => {
									self.state = GridEditingMode::Dialing;
								}
								false => {
									increment(&mut data[x][y], 1);
								}
							}
						}
						if long_press {
							return GridAction::EnterMenu(x, y);
						}
					}
					Entry::Back => {
						if press {
							return GridAction::Exit;
						}
					}
				}
				self.position = (self.position + scroll).rem_euclid((ROWS*COLS+1) as i16);
			}
			GridEditingMode::Dialing => {
				match selected {
					Entry::Element(x, y) => {
						if scroll != 0 {
							increment(&mut data[x][y], scroll);
						}
						if press {
							self.state = GridEditingMode::Selecting;
						}
						if long_press {
							return GridAction::EnterMenu(x, y);
						}
					}
					Entry::Back => {
						unreachable!();
					}
				}
			}
		}

		if scroll != 0 || press || long_press { // any user input leads to a partial redraw
			let new_selected = Self::entry(self.position);

			if selected != new_selected {
				self.draw_update(selected, false, data, &stringify, draw_target);
			}
			self.draw_update(new_selected, true, data, &stringify, draw_target);
		}
		
		GridAction::Continue
	}
}
