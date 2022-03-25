use crate::app::gui_task;
use crate::debugln::*;
use crate::flash;
use crate::gui;
use crate::preset::*;
use crate::OUTPUT_MASK;
use core::fmt::Write;
use core::sync::atomic::Ordering;
use heapless;
use rtic::mutex_prelude::*;
use simple_flash_store::FlashTrait;
use simple_flash_store::FlashStoreError;
use stm32f1xx_hal::prelude::*; // FIXME
use crate::user_input::{UserInput, UserInputHandler};

use crate::flash::MyFlashStore;

fn mode_mask_to_output_mask(mode_mask: u32) -> u32 {
	let a_part = (mode_mask & 0x000F)
		| ((mode_mask & 0x00F0) << 4)
		| ((mode_mask & 0x0F00) << 8)
		| ((mode_mask & 0xF000) << 12);

	let b_part = ((!a_part) << 4) & 0xF0F0F0;

	return a_part | b_part;
}
	
enum ActiveMenu {
	MainScreen(gui::MainScreenState),
	MainMenu(gui::MenuState),
	EventRouting(gui::GridState<EventRouteMode, 8, 8>),
	ClockRouting(gui::GridState<u8, 8, 8>),
	TrsModeSelect(gui::MenuState),
	SaveDestination(gui::MenuState),
	Message(gui::MessageState),
}



pub struct GuiHandler {
	preset_idx: usize,
	mode_mask: u32,
	flash_used_bytes: Result<usize, FlashStoreError>,
	dirty: bool,
	preset: Preset,
	preset_changed: bool,
	display: Display,
	active_menu: ActiveMenu
}

impl GuiHandler {
	pub fn new() -> GuiHandler {
		GuiHandler {
			preset_idx: 0,
			mode_mask: 0xFF0, // FIXME sensible initial value. load this from flash
			flash_used_bytes: todo!(), // FIXME
			dirty: false,
			preset: todo!(),
			preset_changed: false,
			display: todo!(),
			active_menu: ActiveMenu::MainMenu(gui::MenuState::new(0))
		}
	}

	fn handle_main_screen(&mut self, state: &mut gui::MainScreenState, input: UserInput, flash_store: &mut MyFlashStore) -> Option<ActiveMenu> {
		state.process(
			self.preset_idx,
			&self.preset,
			self.dirty,
			(self.flash_used_bytes.unwrap_or(9999), flash::FlashAdapter::SIZE),
			&mut self.display,
		);
		if input.scroll != 0 {
			if !self.dirty {
				self.preset_idx = (self.preset_idx as i16 + input.scroll).rem_euclid(10) as usize;
				self.preset = flash::read_preset_from_flash(
					self.preset_idx as u8,
					flash_store,
				)
				.unwrap_or(Preset::new());
				self.preset_changed = true;
			}
			else {
				state.blink_dirty();
			}
		}

		if input.button_event {
			return Some(ActiveMenu::MainMenu(gui::MenuState::new(0)));
		}
		else {
			return None;
		}
	}

	fn handle_save_destination(&mut self, menu_state: &mut gui::MenuState, input: UserInput, flash_store: &mut MyFlashStore) -> Option<ActiveMenu> {
		use flash::SaveError;
		let result = menu_state.process(
			input,
			"Save to...",
			&["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"],
			self.display,
		);
		match result {
			gui::MenuAction::Activated(index) => {
				if self.dirty || index != self.preset_idx {
					debugln!("Going to save settings to flash");
					let result = flash::save_preset_to_flash(index as u8, &self.preset, flash_store);
					match result {
						Ok(()) => {
							self.dirty = false;
							self.preset_idx = index;
							self.flash_used_bytes = flash_store.used_space();
							return Some(ActiveMenu::MainScreen(gui::MainScreenState::new()));
						}
						Err(SaveError::BufferTooSmall) => {
							return Some(ActiveMenu::Message(gui::MessageState::new(
								&["Preset is too large."],
								"Ok",
								gui::MessageAction::None,
							)));
						}
						Err(SaveError::NoSpaceLeft) => {
							return Some(ActiveMenu::Message(gui::MessageState::new(
								&["No space left."],
								"Ok",
								gui::MessageAction::None,
							)));
						}
						Err(SaveError::CorruptData) => {
							return Some(ActiveMenu::Message(gui::MessageState::new(
								&["Settings store is", "corrupt. Delete", "all data?"],
								"Yes",
								gui::MessageAction::ClearFlash,
							)));
						}
					}
				}
			}
			_ => {}
		}
		None
	}

	fn handle_message(&mut self, state: &mut gui::MessageState, input: UserInput, flash_store: &mut MyFlashStore) -> Option<ActiveMenu> {
		match state.process(input, self.display) {
			gui::MenuAction::Activated(_) => {
				match state.action {
					gui::MessageAction::None => {}
					gui::MessageAction::ClearFlash => {
						debugln!("Reinitializing flash...");
						if flash_store.initialize_flash().is_ok() {
							debugln!("  -> ok.");
							self.flash_used_bytes = flash_store.used_space();
						}
						else {
							debugln!("  -> FAILED!");
						}
					}
				}
				return Some(ActiveMenu::MainMenu(gui::MenuState::new(0)));
			}
			gui::MenuAction::Continue => None
		}
	}

	fn handle_event_routing(&mut self, grid_state: &mut gui::GridState<EventRouteMode, 8, 8>, input: UserInput) -> Option<ActiveMenu> {
		let result = grid_state.process(
			input,
			&mut self.preset.event_routing_table,
			|val, _inc| {
				*val = match *val {
					EventRouteMode::None => EventRouteMode::Keyboard,
					EventRouteMode::Keyboard => EventRouteMode::Controller,
					EventRouteMode::Controller => EventRouteMode::Both,
					EventRouteMode::Both => EventRouteMode::None,
				}
			},
			|val, _| match *val {
				EventRouteMode::None => " ",
				EventRouteMode::Keyboard => "K",
				EventRouteMode::Controller => "C",
				EventRouteMode::Both => "B",
			},
			false,
			"Event Routing",
			self.display,
		);
		match result {
			gui::GridAction::Exit => {
				return Some(ActiveMenu::MainMenu(gui::MenuState::new(0)));
			}
			gui::GridAction::ValueUpdated => {
				self.preset_changed = true;
				self.dirty = true;
			}
			_ => {}
		}
		None
	}

	fn handle_clock_routing(&mut self, grid_state: &mut gui::GridState<u8, 8, 8>, input: UserInput) -> Option<ActiveMenu> {
		let result = grid_state.process(
			input,
			&mut self.preset.clock_routing_table,
			|val, inc| {
				*val = (*val as i16 + inc).rem_euclid(10) as u8;
			},
			|val, _| [" ", "#", "2", "3", "4", "5", "6", "7", "8", "9"][*val as usize],
			true,
			"Clock Routing/Division",
			self.display,
		);
		match result {
			gui::GridAction::Exit => {
				return Some(ActiveMenu::MainMenu(gui::MenuState::new(1)));
			}
			gui::GridAction::ValueUpdated => {
				self.preset_changed = true;
				self.dirty = true;
			}
			_ => {}
		}
		None
	}

	fn handle_trs_mode_select(&mut self, menu_state: &mut gui::MenuState, input: UserInput) -> Option<ActiveMenu> {
		let mut entries = heapless::Vec::<heapless::String<8>, 16>::new();
		for i in 4..12 {
			// FIXME hardcoded
			let mut string = heapless::String::new();
			write!(
				&mut string,
				"{:2}: {}",
				i,
				if self.mode_mask & (1 << i) != 0 {
					"A  "
				}
				else {
					"  B"
				}
			)
			.unwrap();
			entries.push(string).unwrap();
		}
		use core::iter::FromIterator;
		let entries_str = heapless::Vec::<_, 16>::from_iter(
			entries
				.iter()
				.map(|v| v.as_str())
				.chain(core::iter::once("Back")),
		);

		let result = menu_state.process(
			input,
			"TRS Mode Select",
			&entries_str,
			self.display,
		);
		match result {
			gui::MenuAction::Activated(index) => {
				if index == entries_str.len() - 1 {
					return Some(ActiveMenu::MainMenu(gui::MenuState::new(2)));
				}
				else {
					self.mode_mask ^= 1 << (index + 4);
					OUTPUT_MASK
						.store(mode_mask_to_output_mask(self.mode_mask), Ordering::Relaxed);
					menu_state.schedule_redraw();
				}
			}
			_ => {}
		}
		None
	}

	fn handle_main_menu(&mut self, menu_state: &mut gui::MenuState, input: UserInput, flash_store: &mut MyFlashStore) -> Option<ActiveMenu> {
		let result = menu_state.process(
			input,
			"Main Menu",
			&[
				"Event routing",
				"Clock routing",
				"TRS mode A/B select",
				"Save",
				if self.dirty {
					"Revert to saved"
				}
				else {
					"(nothing to revert)"
				},
				"Clear single preset",
				"Clear all presets",
				"Back",
			],
			self.display,
		);
		match result {
			gui::MenuAction::Activated(index) => match index {
				0 => Some(ActiveMenu::EventRouting(gui::GridState::new())),
				1 => Some(ActiveMenu::ClockRouting(gui::GridState::new())),
				2 => Some(ActiveMenu::TrsModeSelect(gui::MenuState::new(0))),
				3 => Some(ActiveMenu::SaveDestination(gui::MenuState::new(self.preset_idx))),
				4 => {
					if self.dirty {
						if let Ok(pr) = flash::read_preset_from_flash(
							self.preset_idx as u8,
							flash_store,
						) {
							self.preset = pr;
							self.dirty = false;
							self.preset_changed = true;
						}
						menu_state.schedule_redraw();
					}
					None
				}
				5 => {
					self.preset = Preset::new();
					self.preset_changed = true;
					self.dirty = true;
					None
				}
				6 => Some(ActiveMenu::Message(gui::MessageState::new(
						&["Delete all data?", "Turn off to", "abort"],
						"Yes",
						gui::MessageAction::ClearFlash,
					))),
				7 => Some(ActiveMenu::MainScreen(gui::MainScreenState::new())),
				_ => unreachable!()
			},
			_ => None
		}
	}

	pub fn process(&mut self, input: UserInput, flash_store: &mut MyFlashStore, current_preset: Preset) {
		self.preset = current_preset; // Ugh. lots of copies. FIXME
		
		let menu_change = match self.active_menu {
			ActiveMenu::Message(ref mut state) => self.handle_message(state, input, flash_store),
			ActiveMenu::MainScreen(ref mut state) => self.handle_main_screen(state, input, flash_store),
			ActiveMenu::MainMenu(ref mut state) => self.handle_main_menu(state, input, flash_store),
			ActiveMenu::SaveDestination(ref mut state) => self.handle_save_destination(state, input, flash_store),
			ActiveMenu::TrsModeSelect(ref mut state) => self.handle_trs_mode_select(state, input),
			ActiveMenu::EventRouting(ref mut state) => self.handle_event_routing(state, input),
			ActiveMenu::ClockRouting(ref mut state) => self.handle_clock_routing(state, input),
		};

		if let Some(new_menu) = menu_change {
			self.active_menu = new_menu;
		}
	}
}

pub(crate) fn gui_task(c: gui_task::Context) {
	let gui_task::SharedResources {
		mut current_preset,
	} = c.shared;
	let gui_task::LocalResources {
		display,
		delay,
		user_input_handler,
		flash_store,
	} = c.local;

	display.init(delay).unwrap();
	display
		.set_orientation(st7789::Orientation::PortraitSwapped)
		.unwrap();
	display.clear(Rgb565::BLACK).unwrap();

	use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

	let mut gui_handler = GuiHandler::new();

	loop {
		delay.delay_ms(1u8);

		let input = user_input_handler.process();
		let mut preset = current_preset.lock(|p| *p);
		gui_handler.process(input, flash_store, preset);

		if gui_handler.preset_changed {
			current_preset.lock(|p| *p = gui_handler.preset);
		}

	}
}
