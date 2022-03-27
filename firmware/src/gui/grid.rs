use super::draw_title;
use super::style::*;

use crate::str_writer::*;
use crate::user_input::UserInput;
use core::fmt::Write;
use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::*, text::Text};

enum GridEditingMode {
	Selecting,
	Dialing,
}

pub enum GridAction {
	NoAction,
	ValueUpdated,
	Exit,
	EnterMenu(usize, usize),
}

pub struct GridState<T, const COLS: usize, const ROWS: usize> {
	position: i16,
	state: GridEditingMode,
	redraw_pending: bool,
	_t: core::marker::PhantomData<T>,
}

#[derive(PartialEq)]
enum Entry {
	Element(usize, usize),
	Back,
}

impl<T, const COLS: usize, const ROWS: usize> GridState<T, COLS, ROWS> {
	pub fn new() -> GridState<T, COLS, ROWS> {
		GridState {
			position: 0,
			state: GridEditingMode::Selecting,
			redraw_pending: true,
			_t: core::marker::PhantomData {},
		}
	}

	fn entry(position: i16) -> Entry {
		if position == (ROWS * COLS) as i16 {
			Entry::Back
		}
		else {
			Entry::Element(position as usize % COLS, position as usize / COLS)
		}
	}

	fn draw_update<STRINGIFY>(
		&self,
		entry: Entry,
		selected: bool,
		data: &mut [[T; ROWS]; COLS],
		stringify: &STRINGIFY,
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) where
		for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str,
	{
		let buf = [0u8; 20];

		let style = if !selected {
			normal_style!()
		}
		else {
			match self.state {
				GridEditingMode::Selecting => selected_style!(),
				GridEditingMode::Dialing => active_style!(),
			}
		};

		match entry {
			Entry::Element(x, y) => {
				Text::new(
					stringify(&data[x][y], &buf),
					Point::new(30 + 20 * (x as i32), 80 + 60 + 20 * (y as i32)),
					style,
				)
				.draw(draw_target)
				.ok()
				.unwrap();
			}
			Entry::Back => {
				Text::new("Back", Point::new(10, 80 + 230), style)
					.draw(draw_target)
					.ok()
					.unwrap();
			}
		}
	}

	pub fn fully_redraw<STRINGIFY>(
		&self,
		data: &mut [[T; ROWS]; COLS],
		stringify: &STRINGIFY,
		title: &str,
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) where
		for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str,
	{
		draw_target.clear(Rgb565::BLACK).ok().unwrap();

		let style = normal_style!();

		draw_title(title, draw_target);

		let mut buf = [0u8; 20];
		for x in 0..(COLS as i32) {
			Text::new(
				slfmt!(&mut buf, "{}", x + 1),
				Point::new(30 + 20 * x, 80 + 40),
				style,
			)
			.draw(draw_target)
			.ok()
			.unwrap();
		}
		for y in 0..(ROWS as i32) {
			Text::new(
				slfmt!(&mut buf, "{}", y + 1),
				Point::new(10, 80 + 60 + 20 * y),
				style,
			)
			.draw(draw_target)
			.ok()
			.unwrap();
		}
		for x in 0..COLS {
			for y in 0..ROWS {
				self.draw_update(
					Entry::Element(x, y),
					(x, y) == (0, 0),
					data,
					stringify,
					draw_target,
				);
			}
		}
		self.draw_update(Entry::Back, false, data, stringify, draw_target);
	}

	pub fn process<STRINGIFY>(
		&mut self,
		input: UserInput,
		data: &mut [[T; ROWS]; COLS],
		increment: impl Fn(&mut T, i16),
		stringify: STRINGIFY,
		allow_dialing: bool,
		title: &str,
		draw_target: &mut impl DrawTarget<Color = Rgb565>,
	) -> GridAction
	where
		for<'a> STRINGIFY: Fn(&T, &'a [u8]) -> &'a str,
	{
		if self.redraw_pending {
			self.fully_redraw(data, &stringify, title, draw_target);
			self.redraw_pending = false;
		}

		let mut updated = false;

		let selected = Self::entry(self.position);

		match self.state {
			GridEditingMode::Selecting => {
				match selected {
					Entry::Element(x, y) => {
						if input.button_event {
							match allow_dialing {
								true => {
									self.state = GridEditingMode::Dialing;
								}
								false => {
									increment(&mut data[x][y], 1);
									updated = true;
								}
							}
						}
						if input.long_press {
							return GridAction::EnterMenu(x, y);
						}
					}
					Entry::Back => {
						if input.button_event {
							return GridAction::Exit;
						}
					}
				}
				self.position = (self.position + input.scroll).rem_euclid((ROWS * COLS + 1) as i16);
			}
			GridEditingMode::Dialing => match selected {
				Entry::Element(x, y) => {
					if input.scroll != 0 {
						increment(&mut data[x][y], input.scroll);
						updated = true;
					}
					if input.button_event {
						self.state = GridEditingMode::Selecting;
					}
					if input.long_press {
						return GridAction::EnterMenu(x, y);
					}
				}
				Entry::Back => {
					unreachable!();
				}
			},
		}

		if input.scroll != 0 || input.button_event || input.long_press {
			// any user input leads to a partial redraw
			let new_selected = Self::entry(self.position);

			if selected != new_selected {
				self.draw_update(selected, false, data, &stringify, draw_target);
			}
			self.draw_update(new_selected, true, data, &stringify, draw_target);
		}

		if updated {
			return GridAction::ValueUpdated;
		}
		else {
			return GridAction::NoAction;
		}
	}

	pub fn schedule_redraw(&mut self) {
		self.redraw_pending = true;
	}
}
