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
use stm32f1xx_hal::prelude::*; // FIXME

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
	display: Display,
}

use stm32f1xx_hal::gpio::{gpiob, gpioc, gpioa};
use stm32f1xx_hal::gpio::{Input,PullUp,Output,PushPull};
use crate::dma_adapter;

type RotaryKnobTimer = stm32f1xx_hal::qei::Qei<
		stm32f1xx_hal::stm32::TIM4,
		stm32f1xx_hal::timer::Tim4NoRemap,
		(gpiob::PB6<Input<PullUp>>, gpiob::PB7<Input<PullUp>>),
	>;
type RotaryKnobButton = gpioc::PC15<Input<PullUp>>;

		type Display = st7789::ST7789<
			display_interface_spi::SPIInterfaceNoCS<
				dma_adapter::WriteDmaToWriteAdapter,
				gpioa::PA2<Output<PushPull>>,
			>,
			gpioa::PA1<Output<PushPull>>,
		>;

struct UserInputHandler {
	old_pressed: bool,
	debounce: u8,
	old_count: u16,

	knob_timer: RotaryKnobTimer,
	knob_button: RotaryKnobButton,
}

struct UserInput {
	scroll: i16,
	button_event: bool,
}

impl UserInputHandler {
	pub fn new(knob_timer: RotaryKnobTimer, knob_button: RotaryKnobButton) -> UserInputHandler {
		UserInputHandler {
			old_pressed: false,
			debounce: 0,
			old_count: knob_timer.count() / 4,
			knob_button,
			knob_timer
		}
	}

	pub fn process(&mut self) -> UserInput {
		const MAX_COUNT: isize = 65536 / 4;

		let count = self.knob_timer.count() / 4;
		let scroll_raw = count as isize - self.old_count as isize;
		let scroll = if scroll_raw >= MAX_COUNT / 2 {
			scroll_raw - MAX_COUNT
		}
		else if scroll_raw <= -MAX_COUNT / 2 {
			scroll_raw + MAX_COUNT
		}
		else {
			scroll_raw
		} as i16;
		self.old_count = count;

		let pressed = self.knob_button.is_low();
		if pressed != self.old_pressed {
			self.debounce = 25;
			self.old_pressed = pressed;
		}
		let button_event = pressed && self.debounce == 1;
		if self.debounce > 0 {
			self.debounce -= 1;
		}

		return UserInput { scroll, button_event };
	}
}

impl GuiHandler {
	fn handle_main_screen(&mut self, state: &mut gui::MainScreenState, input: UserInput) -> Option<ActiveMenu> {
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
				tx.lock(|tx| {
					current_preset.lock(|p| {
						self.preset = flash::read_preset_from_flash(
							self.preset_idx as u8,
							flash_store,
							tx,
						)
						.unwrap_or(Preset::new());
						*p = self.preset;
					})
				});
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
}

pub(crate) fn gui_task(c: gui_task::Context) {
	let gui_task::SharedResources {
		mut tx,
		mut current_preset,
	} = c.shared;
	let gui_task::LocalResources {
		display,
		delay,
		knob_timer,
		knob_button,
		flash_store,
	} = c.local;

	display.init(delay).unwrap();
	display
		.set_orientation(st7789::Orientation::PortraitSwapped)
		.unwrap();
	display.clear(Rgb565::BLACK).unwrap();

	use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

	let mut active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));

	let mut dirty: bool = false;
	let mut preset_idx = 0;
	let mut mode_mask = 0xFF0; // FIXME sensible initial value. actually load this from flash...
	let mut flash_used_bytes = flash_store.used_space();

	let mut user_input_handler = UserInputHandler::new();
	loop {
		delay.delay_ms(1u8);

		let UserInput { scroll, button_event } = user_input_handler.process();

		let mut preset = current_preset.lock(|p| *p);

		match active_menu {
			ActiveMenu::Message(ref mut state) => {
				match state.process(scroll, button_event, false, display) {
					gui::MenuAction::Activated(_) => {
						match state.action {
							gui::MessageAction::None => {}
							gui::MessageAction::ClearFlash => {
								tx.lock(|tx| {
									debugln!(tx, "Reinitializing flash...");
									if flash_store.initialize_flash().is_ok() {
										debugln!(tx, "  -> ok.");
										flash_used_bytes = flash_store.used_space();
									}
									else {
										debugln!(tx, "  -> FAILED!");
									}
								});
							}
						}
						active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));
					}
					gui::MenuAction::Continue => {}
				}
			}
			ActiveMenu::MainScreen(ref mut state) => {
				active_menu = handle_main_screen(state);
			}
			ActiveMenu::MainMenu(ref mut menu_state) => {
				let result = menu_state.process(
					scroll,
					button_event,
					false,
					"Main Menu",
					&[
						"Event routing",
						"Clock routing",
						"TRS mode A/B select",
						"Save",
						if dirty {
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
						0 => {
							active_menu = ActiveMenu::EventRouting(gui::GridState::new());
						}
						1 => {
							active_menu = ActiveMenu::ClockRouting(gui::GridState::new());
						}
						2 => {
							active_menu = ActiveMenu::TrsModeSelect(gui::MenuState::new(0));
						}
						3 => {
							active_menu =
								ActiveMenu::SaveDestination(gui::MenuState::new(preset_idx));
						}
						4 => {
							if dirty {
								tx.lock(|tx| {
									current_preset.lock(|p| {
										if let Ok(pr) = flash::read_preset_from_flash(
											preset_idx as u8,
											flash_store,
											tx,
										) {
											preset = pr;
											*p = preset;
											dirty = false;
										}
									})
								});
								menu_state.schedule_redraw();
							}
						}
						5 => {
							preset = Preset::new();
							current_preset.lock(|p| *p = preset);
							dirty = true;
						}
						6 => {
							active_menu = ActiveMenu::Message(gui::MessageState::new(
								&["Delete all data?", "Turn off to", "abort"],
								"Yes",
								gui::MessageAction::ClearFlash,
							))
						}
						7 => active_menu = ActiveMenu::MainScreen(gui::MainScreenState::new()),
						_ => {
							unreachable!();
						}
					},
					_ => {}
				}
			}
			ActiveMenu::SaveDestination(ref mut menu_state) => {
				use flash::SaveError;
				let result = menu_state.process(
					scroll,
					button_event,
					false,
					"Save to...",
					&["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"],
					display,
				);
				match result {
					gui::MenuAction::Activated(index) => {
						if dirty || index != preset_idx {
							active_menu = ActiveMenu::MainScreen(gui::MainScreenState::new());
							let result = tx.lock(|tx| {
								debugln!(tx, "Going to save settings to flash");
								flash::save_preset_to_flash(index as u8, &preset, flash_store, tx)
							});
							match result {
								Ok(()) => {
									dirty = false;
									preset_idx = index;
									flash_used_bytes = flash_store.used_space();
								}
								Err(SaveError::BufferTooSmall) => {
									active_menu = ActiveMenu::Message(gui::MessageState::new(
										&["Preset is too large."],
										"Ok",
										gui::MessageAction::None,
									));
								}
								Err(SaveError::NoSpaceLeft) => {
									active_menu = ActiveMenu::Message(gui::MessageState::new(
										&["No space left."],
										"Ok",
										gui::MessageAction::None,
									));
								}
								Err(SaveError::CorruptData) => {
									active_menu = ActiveMenu::Message(gui::MessageState::new(
										&["Settings store is", "corrupt. Delete", "all data?"],
										"Yes",
										gui::MessageAction::ClearFlash,
									));
								}
							}
						}
					}
					_ => {}
				}
			}
			ActiveMenu::TrsModeSelect(ref mut menu_state) => {
				let mut entries = heapless::Vec::<heapless::String<8>, 16>::new();
				for i in 4..12 {
					// FIXME hardcoded
					let mut string = heapless::String::new();
					write!(
						&mut string,
						"{:2}: {}",
						i,
						if mode_mask & (1 << i) != 0 {
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
					scroll,
					button_event,
					false,
					"TRS Mode Select",
					&entries_str,
					display,
				);
				match result {
					gui::MenuAction::Activated(index) => {
						if index == entries_str.len() - 1 {
							active_menu = ActiveMenu::MainMenu(gui::MenuState::new(2))
						}
						else {
							mode_mask ^= 1 << (index + 4);
							OUTPUT_MASK
								.store(mode_mask_to_output_mask(mode_mask), Ordering::Relaxed);
							menu_state.schedule_redraw();
						}
					}
					_ => {}
				}
			}
			ActiveMenu::EventRouting(ref mut grid_state) => {
				let result = grid_state.process(
					scroll,
					button_event,
					false,
					&mut preset.event_routing_table,
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
						active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));
					}
					gui::GridAction::ValueUpdated => {
						current_preset.lock(|p| p.event_routing_table = preset.event_routing_table);
						dirty = true;
					}
					_ => {}
				}
			}
			ActiveMenu::ClockRouting(ref mut grid_state) => {
				let result = grid_state.process(
					scroll,
					button_event,
					false,
					&mut preset.clock_routing_table,
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
						active_menu = ActiveMenu::MainMenu(gui::MenuState::new(1));
					}
					gui::GridAction::ValueUpdated => {
						current_preset.lock(|p| p.clock_routing_table = preset.clock_routing_table);
						dirty = true;
					}
					_ => {}
				}
			}
		}
	}
}
