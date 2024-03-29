use crate::app::gui_task;
use crate::debugln::*;
use crate::display;
use crate::flash;
use crate::gui;
use crate::midi_filter_queue::MidiFilterQueueConsumer;
use crate::preset::*;
use crate::user_input::UserInput;
use crate::MIDI_INHIBIT;
use crate::OUTPUT_MASK;
use core::fmt::Write;
use core::sync::atomic::Ordering;
use heapless::{String, Vec};
use rtic::mutex_prelude::*;
use simple_flash_store::FlashStoreError;
use simple_flash_store::FlashTrait;
use stm32f1xx_hal::prelude::*; // FIXME

use crate::flash::MyFlashStore;

use embedded_hal::blocking::delay::DelayUs;

fn mode_mask_to_output_mask(mode_mask: u32) -> u32 {
	let a_part = (mode_mask & 0x000F)
		| ((mode_mask & 0x00F0) << 4)
		| ((mode_mask & 0x0F00) << 8)
		| ((mode_mask & 0xF000) << 12);

	let b_part = ((!a_part) << 4) & 0xF0F0F0;

	return a_part | b_part;
}

struct SelfTestState {
	input_good: [bool; 16],
	output_good: [bool; 16],
	menu: gui::MenuState,
	wait_counter: u16,
}

impl SelfTestState {
	pub fn new() -> SelfTestState {
		SelfTestState {
			input_good: [false; 16],
			output_good: [false; 16],
			menu: gui::MenuState::new(0),
			wait_counter: 0,
		}
	}
}

enum ActiveMenu {
	MainScreen(gui::MainScreenState),
	MainMenu(gui::MenuState),
	EventRouting(gui::GridState<EventRouteMode, 8, 8>),
	ClockRouting(gui::GridState<u8, 8, 8>),
	TrsModeSelect(gui::MenuState),
	SaveDestination(gui::MenuState),
	Message(gui::MessageState),
	Settings(gui::MenuState),
	SelfTest(SelfTestState),
}

impl ActiveMenu {
	pub fn schedule_redraw(&mut self) {
		use ActiveMenu::*;
		match self {
			MainScreen(x) => x.schedule_redraw(),
			MainMenu(x) => x.schedule_redraw(),
			EventRouting(x) => x.schedule_redraw(),
			ClockRouting(x) => x.schedule_redraw(),
			TrsModeSelect(x) => x.schedule_redraw(),
			SaveDestination(x) => x.schedule_redraw(),
			Message(_) => {}
			Settings(x) => x.schedule_redraw(),
			SelfTest(x) => x.menu.schedule_redraw(),
		}
	}
}

enum NavigateAction {
	/// Do not modify the menu stack
	Stay,
	/// Change the current entry to the new entry
	Goto(ActiveMenu),
	/// Push the new entry
	Push(ActiveMenu),
	/// Pop, returning to the previous entry
	Pop,
}

use NavigateAction::*;

struct GuiData {
	flash_used_bytes: Result<usize, FlashStoreError>,
	preset_idx: usize,
	preset: Preset,
	preset_changed: bool,
	dirty: bool,
	invert_encoder_direction: bool,
}

pub struct GuiHandler {
	display: display::Display,
	data: GuiData,
	menu_stack: Vec<ActiveMenu, 4>,
}

fn read_settings(flash_store: &mut MyFlashStore) -> Result<bool, FlashStoreError> {
	let mut buffer = [0u8; 64];
	let data = flash_store.read_file(200, &mut buffer)?;
	if data.len() >= 1 {
		Ok(data[0] != 0)
	}
	else {
		Ok(false)
	}
}

fn write_settings(
	flash_store: &mut MyFlashStore,
	invert_encoder_direction: bool,
) -> Result<(), FlashStoreError> {
	let data = [if invert_encoder_direction { 1u8 } else { 0u8 }];
	flash_store.write_file(200, &data)
}

impl GuiHandler {
	pub fn new(flash_store: &mut MyFlashStore, display: display::Display) -> GuiHandler {
		let mut menu_stack = Vec::new();
		menu_stack
			.push(ActiveMenu::MainScreen(gui::MainScreenState::new()))
			.map_err(|_| ())
			.unwrap();
		menu_stack
			.push(ActiveMenu::MainMenu(gui::MenuState::new(0)))
			.map_err(|_| ())
			.unwrap();

		let invert_encoder_direction = read_settings(flash_store).unwrap_or(false);

		GuiHandler {
			data: GuiData {
				preset_idx: 0,
				flash_used_bytes: flash_store.used_space(),
				dirty: false,
				preset: Preset::new(), // this gets overwritten on every process() anyway
				preset_changed: false,
				invert_encoder_direction,
			},
			display,
			menu_stack,
		}
	}

	pub fn init(&mut self, delay: &mut impl DelayUs<u32>) {
		use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

		self.display.init(delay).unwrap();
		self.display
			.set_orientation(st7789::Orientation::PortraitSwapped)
			.unwrap();
		self.display.clear(Rgb565::BLACK).unwrap();
	}

	fn handle_main_screen(
		data: &mut GuiData,
		state: &mut gui::MainScreenState,
		input: UserInput,
		flash_store: &mut MyFlashStore,
		display: &mut display::Display,
	) -> NavigateAction {
		state.process(
			data.preset_idx,
			&data.preset,
			data.dirty,
			(
				data.flash_used_bytes.unwrap_or(9999),
				flash::FlashAdapter::SIZE,
			),
			display,
		);
		if input.scroll != 0 {
			if !data.dirty {
				data.preset_idx = (data.preset_idx as i16 + input.scroll).rem_euclid(10) as usize;
				data.preset = flash::read_preset_from_flash(data.preset_idx as u8, flash_store)
					.unwrap_or(Preset::new());
				data.preset_changed = true;
			}
			else {
				state.blink_dirty();
			}
		}

		if input.button_event {
			return Push(ActiveMenu::MainMenu(gui::MenuState::new(0)));
		}
		else {
			return Stay;
		}
	}

	fn handle_save_destination(
		data: &mut GuiData,
		menu_state: &mut gui::MenuState,
		input: UserInput,
		flash_store: &mut MyFlashStore,
		display: &mut display::Display,
	) -> NavigateAction {
		use flash::SaveError;
		let result = menu_state.process(
			input,
			"Save to...",
			&["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"],
			display,
		);
		match result {
			gui::MenuAction::Activated(index) => {
				if data.dirty || index != data.preset_idx {
					debugln!("Going to save settings to flash");
					let result =
						flash::save_preset_to_flash(index as u8, &data.preset, flash_store);
					match result {
						Ok(()) => {
							data.dirty = false;
							data.preset_idx = index;
							data.flash_used_bytes = flash_store.used_space();
							return Pop;
						}
						Err(SaveError::BufferTooSmall) => {
							return Goto(ActiveMenu::Message(gui::MessageState::new(
								&["Preset is too large."],
								"Ok",
								gui::MessageAction::None,
							)));
						}
						Err(SaveError::NoSpaceLeft) => {
							return Goto(ActiveMenu::Message(gui::MessageState::new(
								&["No space left."],
								"Ok",
								gui::MessageAction::None,
							)));
						}
						Err(SaveError::CorruptData) => {
							return Goto(ActiveMenu::Message(gui::MessageState::new(
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
		Stay
	}

	fn handle_message(
		data: &mut GuiData,
		state: &mut gui::MessageState,
		input: UserInput,
		flash_store: &mut MyFlashStore,
		display: &mut display::Display,
	) -> NavigateAction {
		match state.process(input, display) {
			gui::MenuAction::Activated(_) => {
				match state.action {
					gui::MessageAction::None => {}
					gui::MessageAction::ClearFlash => {
						debugln!("Reinitializing flash...");
						if flash_store.initialize_flash().is_ok() {
							debugln!("  -> ok.");
							data.flash_used_bytes = flash_store.used_space();
						}
						else {
							debugln!("  -> FAILED!");
						}
					}
				}
				return Pop;
			}
			gui::MenuAction::Continue => Stay,
		}
	}

	fn handle_event_routing(
		data: &mut GuiData,
		grid_state: &mut gui::GridState<EventRouteMode, 8, 8>,
		input: UserInput,
		display: &mut display::Display,
	) -> NavigateAction {
		let result = grid_state.process(
			input,
			&mut data.preset.event_routing_table,
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
			display,
		);
		match result {
			gui::GridAction::Exit => {
				return Pop;
			}
			gui::GridAction::ValueUpdated => {
				data.preset_changed = true;
				data.dirty = true;
			}
			_ => {}
		}
		Stay
	}

	fn handle_clock_routing(
		data: &mut GuiData,
		grid_state: &mut gui::GridState<u8, 8, 8>,
		input: UserInput,
		display: &mut display::Display,
	) -> NavigateAction {
		let result = grid_state.process(
			input,
			&mut data.preset.clock_routing_table,
			|val, inc| {
				*val = (*val as i16 + inc).rem_euclid(10) as u8;
			},
			|val, _| [" ", "#", "2", "3", "4", "5", "6", "7", "8", "9"][*val as usize],
			true,
			"Clock Routing/Division",
			display,
		);
		match result {
			gui::GridAction::Exit => {
				return Pop;
			}
			gui::GridAction::ValueUpdated => {
				data.preset_changed = true;
				data.dirty = true;
			}
			_ => {}
		}
		Stay
	}

	fn handle_trs_mode_select(
		data: &mut GuiData,
		menu_state: &mut gui::MenuState,
		input: UserInput,
		display: &mut display::Display,
	) -> NavigateAction {
		let mut entries = Vec::<String<8>, 16>::new();
		for i in 4..12 {
			// FIXME hardcoded
			let mut string = String::new();
			write!(
				&mut string,
				"{:2}: {}",
				i,
				if data.preset.mode_mask & (1 << i) != 0 {
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
		let entries_str = Vec::<_, 16>::from_iter(
			entries
				.iter()
				.map(|v| v.as_str())
				.chain(core::iter::once("Back")),
		);

		let result = menu_state.process(input, "TRS Mode Select", &entries_str, display);
		match result {
			gui::MenuAction::Activated(index) => {
				if index == entries_str.len() - 1 {
					return Pop;
				}
				else {
					data.preset.mode_mask ^= 1 << (index + 4);
					data.dirty = true;
					data.preset_changed = true;
					menu_state.schedule_redraw();
				}
			}
			_ => {}
		}
		Stay
	}

	fn handle_main_menu(
		data: &mut GuiData,
		menu_state: &mut gui::MenuState,
		input: UserInput,
		flash_store: &mut MyFlashStore,
		display: &mut display::Display,
	) -> NavigateAction {
		let result = menu_state.process(
			input,
			"Main Menu",
			&[
				"Event routing",
				"Clock routing",
				"Settings",
				"Save",
				if data.dirty {
					"Revert to saved"
				}
				else {
					"(nothing to revert)"
				},
				"Clear single preset",
				"Clear all presets",
				"Back",
			],
			display,
		);
		match result {
			gui::MenuAction::Activated(index) => match index {
				0 => Push(ActiveMenu::EventRouting(gui::GridState::new())),
				1 => Push(ActiveMenu::ClockRouting(gui::GridState::new())),
				2 => Push(ActiveMenu::Settings(gui::MenuState::new(0))),
				3 => Push(ActiveMenu::SaveDestination(gui::MenuState::new(
					data.preset_idx,
				))),
				4 => {
					if data.dirty {
						if let Ok(pr) =
							flash::read_preset_from_flash(data.preset_idx as u8, flash_store)
						{
							data.preset = pr;
							data.dirty = false;
							data.preset_changed = true;
						}
						menu_state.schedule_redraw();
					}
					Stay
				}
				5 => {
					data.preset = Preset::new();
					data.preset_changed = true;
					data.dirty = true;
					Stay
				}
				6 => Push(ActiveMenu::Message(gui::MessageState::new(
					&["Delete all data?", "Turn off to", "abort"],
					"Yes",
					gui::MessageAction::ClearFlash,
				))),
				7 => Pop,
				_ => unreachable!(),
			},
			_ => Stay,
		}
	}

	fn handle_settings_menu(
		data: &mut GuiData,
		menu_state: &mut gui::MenuState,
		input: UserInput,
		flash_store: &mut MyFlashStore,
		display: &mut display::Display,
	) -> NavigateAction {
		let result = menu_state.process(
			input,
			"Settings Menu",
			&[
				"TRS mode A/B select",
				"Self test",
				if data.invert_encoder_direction {
					"Invert knob: Yes"
				}
				else {
					"Invert knob: No"
				},
				env!("VERGEN_GIT_SEMVER"),
				"Back",
			],
			display,
		);
		match result {
			gui::MenuAction::Activated(index) => match index {
				0 => Push(ActiveMenu::TrsModeSelect(gui::MenuState::new(0))),
				1 => Push(ActiveMenu::SelfTest(SelfTestState::new())),
				2 => {
					data.invert_encoder_direction = !data.invert_encoder_direction;
					menu_state.schedule_redraw();
					write_settings(flash_store, data.invert_encoder_direction).unwrap(); // FIXME make this nonpanicking
					Stay
				}
				3 => Stay,
				4 => Pop,
				_ => unreachable!(),
			},
			_ => Stay,
		}
	}

	fn handle_selftest(
		_data: &mut GuiData,
		state: &mut SelfTestState,
		input: UserInput,
		display: &mut display::Display,
		midi_channel: &mut MidiFilterQueueConsumer<'static, 16>,
		mut send_midi: impl FnMut(&[u8], usize),
	) -> NavigateAction {
		MIDI_INHIBIT.store(true, Ordering::Relaxed);
		midi_channel.set_filter(0xFFFF);

		let mut updated = false;
		while let Some(midi_message) = midi_channel.dequeue() {
			let cable_incoming = midi_message[0] >> 4;

			if midi_message[1] == 0xF0 && midi_message[2] < 16 && midi_message[3] == 0xF7 {
				let cable_outgoing = midi_message[2];
				updated |= state.input_good[cable_incoming as usize] == false
					|| state.output_good[cable_outgoing as usize] == false;
				state.input_good[cable_incoming as usize] = true;
				state.output_good[cable_outgoing as usize] = true;
			}
		}

		state.wait_counter += 1;
		if state.wait_counter >= 20 {
			state.wait_counter = 0;

			for cable in 0..16 {
				send_midi(&[0xF0, cable, 0xF7], cable as usize);
			}
			crate::app::send_task::spawn().ok();
		}

		let mut lines: [String<32>; 8] = Default::default();
		for i in 0..16 {
			let mut temp: String<12> = String::new();
			if !state.input_good[i] {
				write!(temp, "in{:02} ", i).unwrap();
			}
			else {
				write!(temp, "     ").unwrap();
			}

			if !state.output_good[i] {
				write!(temp, "out{:02} ", i).unwrap();
			}
			else {
				write!(temp, "      ").unwrap();
			}

			lines[i / 2].push_str(&temp).unwrap();
		}

		use core::iter::FromIterator;
		let lines_str = Vec::<_, 16>::from_iter(
			lines
				.iter()
				.map(|v| v.as_str())
				.chain(core::iter::once("Back")),
		);

		if updated {
			state.menu.schedule_redraw();
		}

		let result = state.menu.process(input, "Self Test", &lines_str, display);

		match result {
			gui::MenuAction::Activated(_) => {
				MIDI_INHIBIT.store(false, Ordering::Relaxed);
				midi_channel.set_filter(0x0000);
				Pop
			}
			_ => Stay,
		}
	}

	pub fn process(
		&mut self,
		mut input: UserInput,
		flash_store: &mut MyFlashStore,
		current_preset: Preset,
		midi_channel: &mut MidiFilterQueueConsumer<'static, 16>,
		send_midi: impl FnMut(&[u8], usize),
	) {
		self.data.preset = current_preset; // Ugh. lots of copies. FIXME

		if self.data.invert_encoder_direction {
			input.scroll = -input.scroll;
		}

		let menu_change = match self.menu_stack.last_mut().unwrap() {
			ActiveMenu::Message(ref mut state) => {
				Self::handle_message(&mut self.data, state, input, flash_store, &mut self.display)
			}
			ActiveMenu::MainScreen(ref mut state) => Self::handle_main_screen(
				&mut self.data,
				state,
				input,
				flash_store,
				&mut self.display,
			),
			ActiveMenu::MainMenu(ref mut state) => {
				Self::handle_main_menu(&mut self.data, state, input, flash_store, &mut self.display)
			}
			ActiveMenu::SaveDestination(ref mut state) => Self::handle_save_destination(
				&mut self.data,
				state,
				input,
				flash_store,
				&mut self.display,
			),
			ActiveMenu::TrsModeSelect(ref mut state) => {
				Self::handle_trs_mode_select(&mut self.data, state, input, &mut self.display)
			}
			ActiveMenu::EventRouting(ref mut state) => {
				Self::handle_event_routing(&mut self.data, state, input, &mut self.display)
			}
			ActiveMenu::ClockRouting(ref mut state) => {
				Self::handle_clock_routing(&mut self.data, state, input, &mut self.display)
			}
			ActiveMenu::Settings(ref mut state) => Self::handle_settings_menu(
				&mut self.data,
				state,
				input,
				flash_store,
				&mut self.display,
			),
			ActiveMenu::SelfTest(ref mut state) => Self::handle_selftest(
				&mut self.data,
				state,
				input,
				&mut self.display,
				midi_channel,
				send_midi,
			),
		};

		debug_assert!(self.menu_stack.len() >= 1);
		match menu_change {
			Stay => {}
			Push(new_menu) => {
				if self.menu_stack.push(new_menu).is_err() {
					unreachable!();
				}
			}
			Goto(new_menu) => {
				*self.menu_stack.last_mut().unwrap() = new_menu;
			}
			Pop => {
				self.menu_stack.pop();
				self.menu_stack.last_mut().unwrap().schedule_redraw();
			}
		}
	}
}

pub(crate) fn gui_task(c: gui_task::Context) {
	let gui_task::SharedResources {
		mut current_preset,
		mut midi_out_queues,
	} = c.shared;
	let gui_task::LocalResources {
		gui_handler,
		delay,
		user_input_handler,
		flash_store,
		gui_midi_in_consumer,
	} = c.local;

	gui_handler.init(delay);

	loop {
		delay.delay_ms(1u8);

		let input = user_input_handler.process();
		let preset = current_preset.lock(|p| *p);

		let send_midi = |message: &[u8], cable: usize| {
			midi_out_queues
				.lock(|q| q[cable].normal.enqueue_all(message))
				.ok();
		};

		gui_handler.process(input, flash_store, preset, gui_midi_in_consumer, send_midi);

		if gui_handler.data.preset_changed {
			current_preset.lock(|p| *p = gui_handler.data.preset);
			gui_handler.data.preset_changed = false;
		}

		// FIXME HACK: this also sets the first output mask which *should* be set when loading the preset in init...
		OUTPUT_MASK.store(
			mode_mask_to_output_mask(gui_handler.data.preset.mode_mask),
			Ordering::Relaxed,
		);
	}
}
