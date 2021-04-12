#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;

mod software_uart;
use software_uart::*;

use rtic::app;
use stm32f1xx_hal::{prelude::*, stm32, serial, timer};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;

use heapless::spsc::Queue;

use usb_device::bus;
use usb_device::prelude::*;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};

use parse_midi::MidiToUsbParser;

use software_uart::typenum::Unsigned;

const SYSCLK : Hertz = Hertz(72_000_000);


type NumPortPairs = software_uart::typenum::U4;


#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };

	writeln!(tx, "Panic!").ok();
	writeln!(tx, "{}", info).ok();


	use stm32f1xx_hal::gpio::{Input, Floating};
	let led: stm32f1xx_hal::gpio::gpioc::PC13<Input<Floating>> = unsafe { MaybeUninit::uninit().assume_init() };
	let mut reg = unsafe { MaybeUninit::uninit().assume_init() };
	let mut led = led.into_push_pull_output(&mut reg);
	loop {
		let mut blink_thrice = |delay: u32| {
			for _ in 0..3 {
				led.set_low().ok();
				cortex_m::asm::delay(5000000*delay);
				led.set_high().ok();
				cortex_m::asm::delay(10000000);
			}
			cortex_m::asm::delay(10000000);
		};
		blink_thrice(1);
		blink_thrice(4);
		blink_thrice(1);
		cortex_m::asm::delay(10000000);
	}
}


#[cfg(feature = "benchmark")]
static mut BENCHMARK_CYCLES: u16 = 0;
#[cfg(feature = "benchmark")]
static mut BENCHMARK_PHASE: i8 = -1;

#[cfg(feature = "benchmark")]
fn benchmark(phase: i8) -> u16 {
	if phase < 0 || phase >= 3 {
		return 0;
	}

	unsafe {
		core::ptr::write_volatile(&mut BENCHMARK_CYCLES, 0);
		core::ptr::write_volatile(&mut BENCHMARK_PHASE, phase);
		loop {
			let result = core::ptr::read_volatile(&BENCHMARK_CYCLES);
			if result != 0 {
				return result;
			}
		}
	}
}

pub struct MidiOutQueue {
	realtime: Queue<u8, heapless::consts::U16, u16, heapless::spsc::SingleCore>,
	normal: Queue<u8, heapless::consts::U16, u16, heapless::spsc::SingleCore>,
}

impl Default for MidiOutQueue {
	fn default() -> MidiOutQueue {
		MidiOutQueue {
			realtime: unsafe { Queue::u16_sc() },
			normal: unsafe { Queue::u16_sc() }
		}
	}
}

#[app(device = stm32f1xx_hal::pac)]
const APP: () = {
	struct Resources {
		tx: serial::Tx<stm32::USART1>,
		mytimer: timer::CountDownTimer<stm32::TIM2>,

		#[cfg(feature = "benchmark")]
		bench_timer: timer::CountDownTimer<stm32::TIM1>,

		queue: Queue<(u8,u8), heapless::consts::U200, u16, heapless::spsc::SingleCore>,

		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,

		midi_parsers: [MidiToUsbParser; 16],
		midi_out_queues: [MidiOutQueue; 16],

		sw_uart_rx: SoftwareUartRx<'static, NumPortPairs>,
		sw_uart_tx: SoftwareUartTx<'static, NumPortPairs>,
		sw_uart_isr: SoftwareUartIsr<'static, NumPortPairs>,
	
		usb_midi_buffer: UsbMidiBuffer,
	}

	#[init(spawn=[benchmark_task, mainloop])]
	fn init(cx : init::Context) -> init::LateResources {
		static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;
		static mut SOFTWARE_UART: Option<SoftwareUart<NumPortPairs>> = None;

		*SOFTWARE_UART = Some(SoftwareUart::new());
		let (sw_uart_tx, sw_uart_rx, sw_uart_isr) = (*SOFTWARE_UART).as_mut().unwrap().split();

		let dp = stm32::Peripherals::take().unwrap();


		// Clock configuration
		let mut flash = dp.FLASH.constrain();
		let mut rcc = dp.RCC.constrain();

		let clocks = rcc.cfgr
			.use_hse(8.mhz())
			.sysclk(SYSCLK)
			.pclk1(36.mhz())
			.pclk2(72.mhz())
			.freeze(&mut flash.acr);

		assert!(clocks.usbclk_valid());


		// GPIO and peripheral configuration
		let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
		let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
		let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
		let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

		// Configure the on-board LED (PC13, green)
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_high().ok(); // Turn off

		gpioa.pa0.into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
		gpioa.pa1.into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
		gpioa.pa2.into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
		gpioa.pa3.into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
		let _pb5 = gpiob.pb5.into_floating_input(&mut gpiob.crl);
		let _pb6 = gpiob.pb6.into_floating_input(&mut gpiob.crl);
		let _pb7 = gpiob.pb7.into_floating_input(&mut gpiob.crl);
		let _pb8 = gpiob.pb8.into_floating_input(&mut gpiob.crh);

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
		let gpio_rx = gpioa.pa10;
		let serial = serial::Serial::usart1(
			dp.USART1,
			(gpio_tx, gpio_rx),
			&mut afio.mapr,
			serial::Config::default().baudrate(38400.bps()),
			clocks,
			&mut rcc.apb2
		);
		let (mut tx, _rx) = serial.split();
		writeln!(tx, "========================================================").ok();
		writeln!(tx, "midikraken @ {}", env!("VERGEN_SHA")).ok();
		writeln!(tx, "      built on {}", env!("VERGEN_BUILD_TIMESTAMP")).ok();
		writeln!(tx, "========================================================\n").ok();

		// Configure USB
		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low().ok();
		cortex_m::asm::delay(clocks.sysclk().0 / 100);
		
		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		let usb_pins = Peripheral {
			usb: dp.USB,
			pin_dm: usb_dm,
			pin_dp: usb_dp
		};
		*USB_BUS = Some( UsbBus::new(usb_pins) );
		let usb_bus = USB_BUS.as_ref().unwrap();

		let midi = usbd_midi::midi_device::MidiClass::new(usb_bus, NumPortPairs::U8, NumPortPairs::U8).unwrap();
		let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Windfisch")
			.product("Midikraken x4")
			.serial_number("TEST")
			.device_class(usbd_midi::data::usb::constants::USB_CLASS_NONE)
			.build();
	
		
		let mut mytimer =
			timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(31250 * 3));
		mytimer.listen(timer::Event::Update);

		let queue = unsafe { Queue::u16_sc() };
		
		#[cfg(feature = "benchmark")]
		let bench_timer =
			timer::Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2)
			.start_raw(0, 0xFFFF);

		#[cfg(feature = "benchmark")]
		{
			let start_100 = bench_timer.cnt();
			cortex_m::asm::delay(100);
			let stop_100 = bench_timer.cnt();
			let start_40000 = bench_timer.cnt();
			cortex_m::asm::delay(40000);
			let stop_40000 = bench_timer.cnt();
			writeln!(tx, "delay for 100 cycles took from cpu cycle {} to {}", start_100, stop_100).ok();
			writeln!(tx, "delay for 40000 took from cpu cycle {} to {}", start_40000, stop_40000).ok();
			writeln!(tx, "sleep(1 second)").ok();
			cortex_m::asm::delay(72000000);
			writeln!(tx, "sleep(1 second)").ok();
			cortex_m::asm::delay(72000000);
			writeln!(tx, "done").ok();
			
			cx.spawn.benchmark_task().unwrap();
			writeln!(tx, "spawned").ok();
		}

		#[cfg(not(feature = "benchmark"))]
		cx.spawn.mainloop().unwrap();

		return init::LateResources { tx, mytimer, queue, usb_dev, midi,
			sw_uart_tx, sw_uart_rx, sw_uart_isr,
			midi_out_queues: Default::default(),
			midi_parsers: Default::default(),
			#[cfg(feature = "benchmark")]
			bench_timer,
			usb_midi_buffer: UsbMidiBuffer::new()
		};
	}

	#[task(resources = [queue, sw_uart_rx], priority = 8)]
	fn byte_received(c: byte_received::Context) {
		for i in 0..NumPortPairs::USIZE {
			if let Some(byte) = c.resources.sw_uart_rx.recv_byte(i) {
				c.resources.queue.enqueue((i as u8, byte)).ok(); // we can't do anything about this failing
			}
		}
	}

	#[task(resources = [tx, queue, midi_parsers, midi_out_queues, usb_dev, midi, sw_uart_tx, usb_midi_buffer], priority = 2)]
	fn mainloop(mut c: mainloop::Context) {
		loop {
			#[cfg(feature = "debugprint")]
			if is_any_queue_full(c.resources.midi_out_queues) {
				write!(c.resources.tx, "!").ok();
			}

			usb_poll(&mut c.resources.usb_dev, &mut c.resources.midi, &mut c.resources.midi_out_queues, &mut c.resources.usb_midi_buffer);

			for cable in 0..NumPortPairs::USIZE {
				if c.resources.sw_uart_tx.clear_to_send(cable) {
					if let Some(byte) = c.resources.midi_out_queues[cable].realtime.dequeue() {
						#[cfg(feature = "debugprint")] write!(c.resources.tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte).ok();
						c.resources.sw_uart_tx.send_byte(cable, byte);
					}
					else if let Some(byte) = c.resources.midi_out_queues[cable].normal.dequeue() {
						#[cfg(feature = "debugprint")] write!(c.resources.tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte).ok();
						c.resources.sw_uart_tx.send_byte(cable, byte);
					}
				}
			}

			while let Some((cable, byte)) = c.resources.queue.lock(|q| { q.dequeue() }) {
				//write!(c.resources.tx, "({},{:02X}) ", cable, byte).ok();
				if let Some(mut usb_message) = c.resources.midi_parsers[cable as usize].push(byte) {
					#[cfg(feature = "debugprint")] write!(c.resources.tx, "MIDI{} >>> USB {:02X?}... ", cable, usb_message).ok();
					usb_message[0] |= cable << 4;
					let ret = c.resources.midi.send_bytes(usb_message);

					#[cfg(feature = "debugprint")]
					match ret {
						Ok(size) => { write!(c.resources.tx, "wrote {} bytes to usb\n", size).ok(); }
						Err(error) => { write!(c.resources.tx, "error writing to usb: {:?}\n", error).ok(); }
					}
				}
			}
		}
	}

	#[cfg(feature = "benchmark")]
	#[task(resources = [tx, sw_uart_tx], priority = 1)]
	fn benchmark_task(mut c: benchmark_task::Context) {
		c.resources.tx.lock(|tx| {
			writeln!(tx, "Benchmarking...").ok();
			for i in 0..3 {
				write!(tx, "  cycle# {} ->", i).ok();
				for _ in 0..20 {
					write!(tx, " {}", benchmark(i)).ok();
				}
				writeln!(tx, "").ok();
			}
			writeln!(tx, "Done!").ok();
		});
	}

	#[task(binds = TIM2, spawn=[byte_received], resources = [mytimer, bench_timer, sw_uart_isr],  priority=100)]
	fn timer_interrupt(c: timer_interrupt::Context) {
		#[cfg(feature = "benchmark")]
		let do_benchmark = c.resources.sw_uart_isr.setup_benchmark(unsafe { core::ptr::read_volatile(&BENCHMARK_PHASE) });
		#[cfg(feature = "benchmark")]
		let start_time = c.resources.bench_timer.cnt();

		c.resources.mytimer.clear_update_interrupt_flag();

		// actual I/O is done at the very beginning to minimize jitter

		// input
		let gpiob_in = unsafe { (*stm32::GPIOB::ptr()).idr.read().bits() } as u16; // read the IO port
		let in_bits: u16 = (gpiob_in >> 5) & 0x000F;
		
		// output: *out_bits have been set in the last three thirdclocks.
		if let Some(out_bits) = c.resources.sw_uart_isr.out_bits() {
			const MASK_A: u32 = 0x87FF87FF;
			const MASK_C: u32 = 0xC000C000;
			let bits_a = (out_bits & 0x7FF) | ((out_bits & 0x800) << 4);
			let bits_c = (out_bits & 0x3000) << 2;
			let bsrr_a = (bits_a as u32 | ((!bits_a as u32) << 16)) & MASK_A;
			let bsrr_c = (bits_c as u32 | ((!bits_c as u32) << 16)) & MASK_C;

			unsafe { (*stm32::GPIOA::ptr()).bsrr.write(|w| w.bits(bsrr_a)); }; // we ensure to only access pins
			unsafe { (*stm32::GPIOC::ptr()).bsrr.write(|w| w.bits(bsrr_c)); }; // we own (using MASK_A / MASK_C)
		}

		if c.resources.sw_uart_isr.process(in_bits) != 0 {
			c.spawn.byte_received().ok();
		}

		#[cfg(feature = "benchmark")]
		{
			let stop_time = c.resources.bench_timer.cnt();
			unsafe {
				if do_benchmark {
					core::ptr::write_volatile(&mut BENCHMARK_PHASE, -1);
					core::ptr::write_volatile(&mut BENCHMARK_CYCLES, stop_time.wrapping_sub(start_time));
				}
			}
		}
	}

	extern "C" {
		fn EXTI0();
		fn EXTI1();
		fn EXTI2();
    }
};

pub struct UsbMidiBuffer {
	buffer: [u8; 128],
	head: usize,
	tail: usize,
}

impl UsbMidiBuffer {
	pub fn new() -> UsbMidiBuffer {
		UsbMidiBuffer { buffer: [0; 128], head: 0, tail: 0 }
	}
	pub fn peek<B: bus::UsbBus>(&mut self, midi: &mut usbd_midi::midi_device::MidiClass<'static, B>) -> Option<[u8; 4]> {
		use core::convert::TryInto;

		if self.head == self.tail {
			if let Ok(len) = midi.read(&mut self.buffer) {
				self.head = 0;
				self.tail = len;
				assert!(self.tail <= 128);
			}
		}
		if self.head != self.tail {
			assert!(self.tail >= self.head+4);
			return Some(self.buffer[self.head..self.head+4].try_into().unwrap());
		}
		None
	}
	pub fn acknowledge(&mut self) {
		self.head += 4;
	}
}

fn usb_poll<B: bus::UsbBus>(
	usb_dev: &mut UsbDevice<'static, B>,
	midi: &mut usbd_midi::midi_device::MidiClass<'static, B>,
	midi_out_queues: &mut [MidiOutQueue; 16],
	buffer: &mut UsbMidiBuffer,
) {
	usb_dev.poll(&mut [midi]);

	while let Some(message) = buffer.peek(midi) {
		let cable = (message[0] & 0xF0) >> 4;
		let is_realtime = message[1] & 0xF8 == 0xF8;

		if is_realtime {
			if midi_out_queues[cable as usize].realtime.enqueue(message[1]).is_ok() {
				buffer.acknowledge();
			}
			else {
				break;
			}
		}
		else {
			let queue = &mut midi_out_queues[cable as usize].normal;
			let len = parse_midi::payload_length(message[0]);
			if queue.len() + len as u16 <= queue.capacity() {
				for i in 0..len {
					queue.enqueue(message[1+i as usize]).unwrap();
				}
				buffer.acknowledge();
			}
			else {
				break;
			}
		};
	}
}
