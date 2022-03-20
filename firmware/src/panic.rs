use crate::machine;
use stm32f1xx_hal::gpio::{Input, Floating};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	#[cfg(feature="debugpanic")]
	{
		use stm32f1xx_hal::{serial, stm32::USART1};
		use core::fmt::Write;
		let mut tx: serial::Tx<USART1> = unsafe { MaybeUninit::uninit().assume_init() };
		writeln!(tx, "Panic!").ok();
		writeln!(tx, "{}", _info).ok();
	}

	let led: stm32f1xx_hal::gpio::gpioc::PC13<Input<Floating>> = unsafe { MaybeUninit::uninit().assume_init() };
	let mut reg = unsafe { MaybeUninit::uninit().assume_init() };
	let mut led = led.into_push_pull_output(&mut reg);

	for _ in 0..3 {
		let mut blink_thrice = |delay: u32| {
			for _ in 0..3 {
				led.set_low();
				cortex_m::asm::delay(5000000*delay);
				led.set_high();
				cortex_m::asm::delay(10000000);
			}
			cortex_m::asm::delay(10000000);
		};
		blink_thrice(1);
		blink_thrice(4);
		blink_thrice(1);
		cortex_m::asm::delay(10000000);
	}
	
	unsafe { machine::reset_to_bootloader(); }
}

