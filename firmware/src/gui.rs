use crate::str_writer::*;
use core::fmt::Write;
use embedded_graphics::{
	mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder},
		pixelcolor::Rgb565,
		prelude::*,
		text::Text,
};

use crate::preset::{Preset, serialize_preset};

pub enum MessageAction {
	None,
	ClearFlash
}

pub struct MessageState {
	message: &'static [&'static str],
	button: &'static str,
	redraw_pending: bool,
	pub action: MessageAction
}

pub enum MenuAction {
	Activated(usize),
	Continue
}

pub struct MenuState {
	selected: usize,
	redraw_pending: bool,
}

macro_rules! title_style { () => {
	MonoTextStyleBuilder::new()
		.font(&FONT_9X15)
		.text_color(Rgb565::WHITE)
		.background_color(Rgb565::new(8,16,8))
		.build()
	}
}
	
macro_rules! normal_style { () => {
	MonoTextStyleBuilder::new()
		.font(&FONT_9X15)
		.text_color(Rgb565::WHITE)
		.background_color(Rgb565::BLACK)
		.build()
	}
}
	
macro_rules! selected_style { () => {
	MonoTextStyleBuilder::new()
		.font(&FONT_9X15)
		.text_color(Rgb565::WHITE)
		.background_color(Rgb565::RED)
		.build()
	}
}
	
macro_rules! active_style { () => {
	MonoTextStyleBuilder::new()
		.font(&FONT_9X15)
		.text_color(Rgb565::WHITE)
		.background_color(Rgb565::BLUE)
		.build()
	}
}

fn draw_title(title: &str, draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>) {
	use embedded_graphics::primitives::*;
	use embedded_graphics::geometry::*;
	Rectangle::new(Point::new(0, 80 + 7), Size::new(240, 17))
		.into_styled( PrimitiveStyleBuilder::new().fill_color(Rgb565::new(8,16,8)).build() )
		.draw(draw_target).ok().unwrap();
	
	Text::new(title, Point::new(20, 80 + 20), title_style!()).draw(draw_target).ok().unwrap();
}

impl MessageState {
	pub fn new(message: &'static [&'static str], button: &'static str, action: MessageAction) -> MessageState {
		MessageState { message, button, action, redraw_pending: true }
	}
	
	pub fn process(
		&mut self,
		_scroll: i16,
		press: bool,
		_long_press: bool,
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	) -> MenuAction
	{
		if press {
			return MenuAction::Activated(0);
		}
		let style = normal_style!();
		if self.redraw_pending {
			draw_target.clear(Rgb565::BLACK).ok().unwrap();
			draw_title("Message", draw_target);
			for (i, text) in self.message.iter().enumerate() {
				Text::new(text, Point::new(20, 80 + 50 + 18*(i as i32)), style).draw(draw_target).ok().unwrap();
			}
			Text::new(self.button, Point::new(10, 80+230), selected_style!()).draw(draw_target).ok().unwrap();
			self.redraw_pending = false;
		}
		MenuAction::Continue
	}
}


impl MenuState {
	pub fn new(selected: usize) -> MenuState {
		MenuState { selected, redraw_pending: true }
	}

	pub fn schedule_redraw(&mut self) {
		self.redraw_pending = true;
	}

	pub fn process(
		&mut self,
		scroll: i16,
		press: bool,
		_long_press: bool,
		title: &str,
		entries: &[&str],
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	) -> MenuAction
	{
		if scroll != 0 {
			let old_selected = self.selected;
			self.selected = (self.selected as i16 + scroll).rem_euclid(entries.len() as i16) as usize;
			self.redraw_item(old_selected, entries[old_selected], draw_target);
			self.redraw_item(self.selected, entries[self.selected], draw_target);
		}
		if press {
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

	fn redraw_item(&self, item: usize, entry: &str, draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>) {
		let this_style = if item == self.selected { selected_style!() } else { normal_style!() };
		Text::new(entry, Point::new(20, 80+20+30 + 20*item as i32), this_style).draw(draw_target).ok().unwrap();
	}
}


enum GridEditingMode {
	Selecting,
	Dialing
}

pub enum GridAction {
	NoAction,
	ValueUpdated,
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

pub struct MainScreenState {
	redraw_pending: bool,
	last_preset: usize,
	last_dirty: bool,
	dirty_blinking: u32,
	preset_size: usize
}

impl MainScreenState {
	const DIRTY_BLINK_TIME: u32 = 1300;
	const DIRTY_BLINK_N: u32 = 4;
	const DIRTY_BLINK_MOD: u32 = Self::DIRTY_BLINK_TIME / (2*Self::DIRTY_BLINK_N);

	pub fn new() -> MainScreenState { MainScreenState { redraw_pending: true, last_preset: 0, last_dirty: false, dirty_blinking: 0, preset_size: 0 } }

	pub fn blink_dirty(&mut self) {
		self.dirty_blinking = Self::DIRTY_BLINK_MOD * 2 * Self::DIRTY_BLINK_N;
	}

	pub fn process(
		&mut self,
		preset_idx: usize,
		preset: &Preset,
		dirty: bool,
		flash_usage: (usize, usize),
		draw_target: &mut impl embedded_graphics::draw_target::DrawTarget<Color = Rgb565>
	)
	{
		let mut buf = [0; 80];
		let style = normal_style!();
		
		if self.redraw_pending {
			draw_target.clear(Rgb565::BLACK).ok().unwrap();
			draw_title("Midikraken", draw_target);
			Text::new("Current preset:", Point::new(20, 80 + 50), style).draw(draw_target).ok().unwrap();
			Text::new(slfmt!(&mut buf, "Total used: {}/{}b", flash_usage.0, flash_usage.1), Point::new(20, 80 + 230), style).draw(draw_target).ok().unwrap();
		}
		
		if self.last_preset != preset_idx || self.redraw_pending {
			Text::new(slfmt!(&mut buf, "{}  ", preset_idx), Point::new(180, 80 + 50), style).draw(draw_target).ok().unwrap();
			self.preset_size = serialize_preset(&mut None, preset);
			Text::new(slfmt!(&mut buf, "Preset size:{:4} bytes", self.preset_size), Point::new(20, 80 + 210), style).draw(draw_target).ok().unwrap();
		}


		let dirty_blink_rem = self.dirty_blinking % Self::DIRTY_BLINK_MOD;
		let dirty_blink_redraw = dirty_blink_rem % (Self::DIRTY_BLINK_MOD / 2) == (Self::DIRTY_BLINK_MOD / 2 - 1);
		let highlight_dirty = dirty_blink_rem >= Self::DIRTY_BLINK_MOD / 2;
		if self.last_dirty != dirty || self.redraw_pending || dirty_blink_redraw {
			Text::new(
				if dirty { "(unsaved)" } else { "         " },
				Point::new(40, 80 + 70),
				if highlight_dirty { selected_style!() } else { normal_style!() }
			).draw(draw_target).ok().unwrap();
		}

		if self.dirty_blinking > 0 {
			self.dirty_blinking -= 1;
		}

		self.last_dirty = dirty;
		self.last_preset = preset_idx;
		self.redraw_pending = false;
	}
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
				normal_style!()
			}
			else {
				match self.state {
					GridEditingMode::Selecting => selected_style!(),
					GridEditingMode::Dialing => active_style!()
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

		let style = normal_style!();

		draw_title(title, draw_target);

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

		let mut updated = false;

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
									updated = true;
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
							updated = true;
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
		
		if updated {
			return GridAction::ValueUpdated;
		}
		else {
			return GridAction::NoAction;
		}
	}
}
