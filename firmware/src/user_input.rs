use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::gpio::{gpiob, gpioc, gpioa};
use stm32f1xx_hal::gpio::{Input,PullUp,Output,PushPull};
use crate::dma_adapter;

pub type RotaryKnobTimer = stm32f1xx_hal::qei::Qei<
		stm32f1xx_hal::stm32::TIM4,
		stm32f1xx_hal::timer::Tim4NoRemap,
		(gpiob::PB6<Input<PullUp>>, gpiob::PB7<Input<PullUp>>),
	>;

pub type RotaryKnobButton = gpioc::PC15<Input<PullUp>>;

		type Display = st7789::ST7789<
			display_interface_spi::SPIInterfaceNoCS<
				dma_adapter::WriteDmaToWriteAdapter,
				gpioa::PA2<Output<PushPull>>,
			>,
			gpioa::PA1<Output<PushPull>>,
		>;


pub struct UserInputHandler {
	old_pressed: bool,
	debounce: u8,
	old_count: u16,

	knob_timer: RotaryKnobTimer,
	knob_button: RotaryKnobButton,
}

pub struct UserInput {
	pub scroll: i16,
	pub button_event: bool,
	pub long_press: bool
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

		return UserInput { scroll, button_event, long_press: false };
	}
}
