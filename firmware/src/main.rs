#![no_main]
#![no_std]

/*
Midikraken firmware
Copyright (C) 2021 Florian Jung

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License Version 3 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

mod debugln;
mod flash;
mod machine;
mod panic;
mod preset;
mod sysex_bootloader;
#[macro_use]
mod str_writer;
mod display;
mod dma_adapter;
mod gui;
mod gui_task;
mod midi_filter_queue;
mod user_input;

#[allow(unused_imports)]
use debugln::*;

use heapless;
use heapless::spsc::Queue;
use parse_midi;

use flash::MyFlashStore;
use midi_filter_queue::{MidiFilterQueue, MidiFilterQueueConsumer, MidiFilterQueueProducer};
use simple_flash_store::FlashStore;

use sysex_bootloader::BootloaderSysexStatemachine;

#[allow(unused_imports)]
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use rtic::app;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{
	dma,
	gpio::{gpiob, gpioc, Alternate, Floating, Input, Output, PushPull},
	prelude::*,
	serial, spi, stm32, timer,
};

use dma_adapter::WriteDmaToWriteAdapter;

use usb_device::prelude::*;

use tim2_interrupt_handler::software_uart::*;
use tim2_interrupt_handler::DmaPair;

use preset::*;

const NUM_PORT_PAIRS: usize = 12;

const SYSCLK: Hertz = Hertz(72_000_000);

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

pub struct NotifyingQueue<T: Copy, const N: usize> {
	queue: Queue<T, N>,
	notification_pending: bool,
}

impl<T: Copy, const N: usize> NotifyingQueue<T, N> {
	pub fn new() -> NotifyingQueue<T, N> {
		NotifyingQueue {
			queue: Queue::new(),
			notification_pending: false,
		}
	}

	pub fn enqueue(&mut self, value: T) -> Result<(), T> {
		self.queue.enqueue(value)
	}

	pub fn enqueue_all<'a>(&mut self, message: &'a [T]) -> Result<(), &'a [T]> {
		if self.queue.len() + message.len() <= self.queue.capacity() {
			for item in message.iter() {
				self.queue.enqueue(*item).map_err(|_| ()).unwrap();
			}
			return Ok(());
		}
		else {
			return Err(message);
		}
	}

	pub fn dequeue(&mut self) -> Option<(T, bool)> {
		let item = self.queue.dequeue()?;

		if self.queue.is_empty() && self.notification_pending {
			self.notification_pending = false;
			Some((item, true))
		}
		else {
			Some((item, false))
		}
	}

	pub fn request_notification(&mut self) {
		self.notification_pending = true;
	}
}

pub struct MidiOutQueue {
	realtime: NotifyingQueue<u8, 16>,
	normal: NotifyingQueue<u8, 16>,
}

impl Default for MidiOutQueue {
	fn default() -> MidiOutQueue {
		MidiOutQueue {
			realtime: NotifyingQueue::new(),
			normal: NotifyingQueue::new(),
		}
	}
}

impl MidiOutQueue {
	pub fn dequeue(&mut self) -> Option<(u8, bool)> {
		self.realtime.dequeue().or_else(|| self.normal.dequeue())
	}

	pub fn enqueue_usb_message(&mut self, message: [u8; 4], request_notification: bool) -> bool {
		let is_realtime = message[1] & 0xF8 == 0xF8;

		if is_realtime {
			let ok = self.realtime.enqueue(message[1]).is_ok();
			if !ok && request_notification {
				self.realtime.request_notification();
			}
			ok
		}
		else {
			let ok = self.normal.enqueue_all(&message[1..]).is_ok();
			if !ok && request_notification {
				self.normal.request_notification();
			}
			ok
		}
	}
}

static mut DMA_BUFFER: DmaPair = DmaPair::zero();

static OUTPUT_MASK: AtomicU32 = AtomicU32::new(0x000000F0); // FIXME init this to F0F0F0F0

/// Used by the GUI to inhibit all USB-MIDI or MIDI routing activity.
static MIDI_INHIBIT: AtomicBool = AtomicBool::new(false);

#[app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4])]
mod app {

	static mut SOFTWARE_UART: Option<SoftwareUart<NUM_PORT_PAIRS>> = None;
	static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

	use super::*; // FIXME
	use crate::gui_task::GuiHandler;

	#[shared]
	struct Resources {
		_tx: serial::Tx<stm32::USART1>,
		queue: Queue<(u8, u8), 200>,

		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,

		sw_uart_tx: SoftwareUartTx<'static, NUM_PORT_PAIRS>, // belongs to send_task

		current_preset: Preset,

		midi_out_queues: [MidiOutQueue; 16],
	}

	#[local]
	struct LocalResources {
		clock_count: [u16; 8],
		usb_dev: UsbDevice<'static, UsbBusType>,
		bootloader_sysex_statemachine: BootloaderSysexStatemachine,
		usb_midi_buffer: UsbMidiBuffer,
		mytimer: timer::CountDownTimer<stm32::TIM2>,

		#[cfg(feature = "benchmark")]
		bench_timer: timer::CountDownTimer<stm32::TIM1>,
		midi_parsers: [parse_midi::MidiToUsbParser; 16],

		sw_uart_rx: SoftwareUartRx<'static, NUM_PORT_PAIRS>,
		sw_uart_isr: SoftwareUartIsr<'static, NUM_PORT_PAIRS>,

		spi_strobe_pin: gpioc::PC14<Output<PushPull>>,

		dma_transfer: Option<
			dma::Transfer<
				dma::W,
				(&'static mut [u8; 4], &'static [u8; 4]),
				dma::RxTxDma<
					spi::Spi<
						stm32::SPI1,
						spi::Spi1Remap,
						(
							gpiob::PB3<Alternate<PushPull>>,
							gpiob::PB4<Input<Floating>>,
							gpiob::PB5<Alternate<PushPull>>,
						),
						u8,
					>,
					dma::dma1::C2,
					dma::dma1::C3,
				>,
			>,
		>,

		gui_handler: GuiHandler,
		delay: stm32f1xx_hal::delay::Delay,

		user_input_handler: user_input::UserInputHandler,
		flash_store: MyFlashStore,

		gui_midi_in_producer: MidiFilterQueueProducer<'static, 16>,
		gui_midi_in_consumer: MidiFilterQueueConsumer<'static, 16>,
	}

	#[init(local = [
		gui_midi_in: MidiFilterQueue<16> = MidiFilterQueue::<16>::new(),
	])]
	fn init(cx: init::Context) -> (Resources, LocalResources, init::Monotonics) {
		let dp = cx.device;

		// Clock configuration
		let mut flash = dp.FLASH.constrain();
		let rcc = dp.RCC.constrain();

		let clocks = rcc
			.cfgr
			.use_hse(8.mhz())
			.sysclk(SYSCLK)
			.pclk1(36.mhz())
			.pclk2(72.mhz())
			.freeze(&mut flash.acr);

		assert!(clocks.usbclk_valid());

		let delay = stm32f1xx_hal::delay::Delay::new(cx.core.SYST, clocks);

		// GPIO and peripheral configuration
		let mut afio = dp.AFIO.constrain();
		let mut gpioa = dp.GPIOA.split();
		let mut gpiob = dp.GPIOB.split();
		let mut gpioc = dp.GPIOC.split();
		let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

		// Configure the on-board LED (PC13, green)
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_high(); // Turn off

		// SPI
		let mut spi_strobe_pin = gpioc
			.pc14
			.into_push_pull_output_with_state(&mut gpioc.crh, stm32f1xx_hal::gpio::PinState::Low); // controls shift registers
		let clk = pb3.into_alternate_push_pull(&mut gpiob.crl);
		let miso = pb4.into_floating_input(&mut gpiob.crl);
		let mosi = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);
		let mut spi = spi::Spi::spi1(
			dp.SPI1,
			(clk, miso, mosi),
			&mut afio.mapr,
			spi::Mode {
				phase: spi::Phase::CaptureOnFirstTransition,
				polarity: spi::Polarity::IdleLow,
			},
			10.mhz(),
			clocks,
		);

		// Quickly set the shift registers to their idle state. This needs to happen as quickly as possible.
		spi.transfer(&mut [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
			.unwrap();
		spi_strobe_pin.set_high();

		// SPI DMA
		let dma = dp.DMA1.split();
		let spi_dma = spi.with_rx_tx_dma(dma.2, dma.3);
		let spi_dma_transfer = spi_dma.read_write(unsafe { &mut DMA_BUFFER.received }, unsafe {
			&DMA_BUFFER.transmit
		});

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
		let gpio_rx = gpioa.pa10;
		let serial = serial::Serial::usart1(
			dp.USART1,
			(gpio_tx, gpio_rx),
			&mut afio.mapr,
			serial::Config::default().baudrate(115200.bps()),
			clocks,
		);

		#[allow(unused_mut)]
		let (mut tx, _rx) = serial.split();

		#[cfg(feature = "debugprint_version")]
		{
			writeln!(
				tx,
				"========================================================"
			)
			.ok();
			writeln!(
				tx,
				"midikraken {} @ {}",
				env!("VERGEN_GIT_SEMVER"),
				env!("VERGEN_GIT_SHA")
			)
			.ok();
			writeln!(tx, "      built on {}", env!("VERGEN_BUILD_TIMESTAMP")).ok();
			if cfg!(feature = "bootloader") {
				writeln!(tx, "      bootloader enabled").ok();
			}
			else {
				writeln!(tx, "      without bootloader").ok();
			}
			writeln!(
				tx,
				"========================================================\n"
			)
			.ok();
		}

		unsafe {
			SOFTWARE_UART = Some(SoftwareUart::new());
		}
		let (sw_uart_tx, sw_uart_rx, sw_uart_isr) =
			unsafe { SOFTWARE_UART.as_mut().unwrap().split() };

		// Configure the display

		let display_reset = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
		let display_dc = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
		let display_clk = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
		let display_mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
		let display_spi = spi::Spi::spi2(
			dp.SPI2,
			(display_clk, spi::NoMiso, display_mosi),
			spi::Mode {
				phase: spi::Phase::CaptureOnFirstTransition,
				polarity: spi::Polarity::IdleHigh,
			},
			18.mhz(),
			clocks,
		);

		let display_dma = display_spi.with_tx_dma(dma.5);
		let adapter = WriteDmaToWriteAdapter::new(display_dma);
		let di = display_interface_spi::SPIInterfaceNoCS::new(adapter, display_dc);
		let display = st7789::ST7789::new(di, display_reset, 240, 240);

		// Configure USB
		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low();
		cortex_m::asm::delay(clocks.sysclk().0 / 100);

		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		let usb_pins = Peripheral {
			usb: dp.USB,
			pin_dm: usb_dm,
			pin_dp: usb_dp,
		};
		unsafe { USB_BUS = Some(UsbBus::new(usb_pins)) };
		let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

		let midi = usbd_midi::midi_device::MidiClass::new(
			usb_bus,
			NUM_PORT_PAIRS as u8,
			NUM_PORT_PAIRS as u8,
		)
		.unwrap();
		let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x2E80))
			.manufacturer("Windfisch")
			.product("Midikraken")
			.serial_number("TEST")
			.device_class(usbd_midi::data::usb::constants::USB_CLASS_NONE)
			.build();

		let mut mytimer = timer::Timer::tim2(dp.TIM2, &clocks).start_count_down(Hertz(31250 * 3));
		mytimer.listen(timer::Event::Update);

		let knob_timer = timer::Timer::tim4(dp.TIM4, &clocks).qei(
			(
				gpiob.pb6.into_pull_up_input(&mut gpiob.crl),
				gpiob.pb7.into_pull_up_input(&mut gpiob.crl),
			),
			&mut afio.mapr,
			stm32f1xx_hal::qei::QeiOptions {
				slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
				auto_reload_value: 65535,
			},
		);
		let knob_button = gpioc.pc15.into_pull_up_input(&mut gpioc.crh);

		let queue = Queue::new();

		#[cfg(feature = "benchmark")]
		let bench_timer = timer::Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_raw(0, 0xFFFF);

		#[cfg(feature = "benchmark")]
		{
			let start_100 = bench_timer.cnt();
			cortex_m::asm::delay(100);
			let stop_100 = bench_timer.cnt();
			let start_40000 = bench_timer.cnt();
			cortex_m::asm::delay(40000);
			let stop_40000 = bench_timer.cnt();
			writeln!(
				tx,
				"delay for 100 cycles took from cpu cycle {} to {}",
				start_100, stop_100
			)
			.ok();
			writeln!(
				tx,
				"delay for 40000 took from cpu cycle {} to {}",
				start_40000, stop_40000
			)
			.ok();
			writeln!(tx, "sleep(1 second)").ok();
			cortex_m::asm::delay(72000000);
			writeln!(tx, "sleep(1 second)").ok();
			cortex_m::asm::delay(72000000);
			writeln!(tx, "done").ok();

			benchmark_task::spawn().unwrap();
			writeln!(tx, "spawned").ok();
		}

		let mut flash_store = FlashStore::new(flash::FlashAdapter::new(flash));
		let current_preset =
			flash::read_preset_from_flash(0, &mut flash_store).unwrap_or(Preset::new());

		let (gui_midi_in_producer, gui_midi_in_consumer) = cx.local.gui_midi_in.split();

		gui_task::spawn().unwrap();

		return (
			Resources {
				_tx: tx,
				queue,
				midi,
				sw_uart_tx,
				midi_out_queues: Default::default(),
				current_preset,
			},
			LocalResources {
				clock_count: [0; 8],
				bootloader_sysex_statemachine: BootloaderSysexStatemachine::new(),
				usb_dev,
				usb_midi_buffer: UsbMidiBuffer::new(),
				mytimer,
				midi_parsers: Default::default(),
				#[cfg(feature = "benchmark")]
				bench_timer,
				dma_transfer: Some(spi_dma_transfer),
				spi_strobe_pin,
				gui_handler: GuiHandler::new(&mut flash_store, display),
				delay,
				user_input_handler: user_input::UserInputHandler::new(knob_timer, knob_button),
				flash_store,
				sw_uart_rx,
				sw_uart_isr,
				gui_midi_in_consumer,
				gui_midi_in_producer,
			},
			init::Monotonics(),
		);
	}

	#[task(shared = [queue], local = [sw_uart_rx], priority = 15)]
	fn byte_received(mut c: byte_received::Context) {
		for i in 0..NUM_PORT_PAIRS {
			if let Some(byte) = c.local.sw_uart_rx.recv_byte(i) {
				c.shared
					.queue
					.lock(|queue| queue.enqueue((i as u8, byte)))
					.ok(); // we can't do anything about this failing
			}
		}
		handle_received_byte::spawn().ok();
	}

	fn route_message_internally(
		usb_message: [u8; 4],
		cable: u8,
		clock_count: &mut [u16],
		midi_out_queues: &mut [MidiOutQueue],
		preset: &Preset,
	) {
		// process for internal routing
		let is_transport = match usb_message[1] {
			0xF1 | 0xF2 | 0xF3 | 0xFA | 0xFC => true,
			_ => false,
		};
		let is_clock = usb_message[1] == 0xF8;
		let is_control_ish = match usb_message[0] & 0x0F {
			0xB | 0xC | 0xE | 0x4 | 0x5 | 0x6 | 0x7 => true,
			_ => false,
		};
		let is_keyboard_ish = match usb_message[0] & 0x0F {
			0x8 | 0x9 | 0xA | 0xD => true,
			_ => false,
		};

		debug_assert!(preset.clock_routing_table.len() == preset.event_routing_table.len());
		debug_assert!(preset.clock_routing_table[0].len() == preset.event_routing_table[0].len());
		let xlen = preset.clock_routing_table.len();
		let ylen = preset.clock_routing_table[0].len();
		debug_assert!(ylen == clock_count.len());

		if (cable as usize) < ylen {
			let cable_in = cable as usize;

			for cable_out in 0..xlen {
				let forward_event = match preset.event_routing_table[cable_out][cable_in] {
					EventRouteMode::Both => is_control_ish || is_keyboard_ish,
					EventRouteMode::Controller => is_control_ish,
					EventRouteMode::Keyboard => is_keyboard_ish,
					EventRouteMode::None => false,
				};

				let forward_clock = match preset.clock_routing_table[cable_out][cable_in] {
					0 => false,
					division => {
						is_transport || (is_clock && clock_count[cable_in] % (division as u16) == 0)
					}
				};

				if forward_event || forward_clock {
					midi_out_queues[cable_out].enqueue_usb_message(usb_message, false);
					send_task::spawn().ok();
				}
			}

			if usb_message[1] == 0xFA {
				// start
				clock_count[cable_in] = 0;
			}
			if usb_message[1] == 0xF8 {
				// clock
				clock_count[cable_in] = (clock_count[cable_in] + 1) % 27720;
				// 27720 is the least common multiple of 1,2,...,12
			}
		}
	}

	#[task(shared = [queue, midi, current_preset, midi_out_queues], local = [midi_parsers, clock_count, gui_midi_in_producer], priority = 7)]
	fn handle_received_byte(c: handle_received_byte::Context) {
		let handle_received_byte::SharedResources {
			mut queue,
			mut midi,
			mut current_preset,
			mut midi_out_queues,
		} = c.shared;
		let handle_received_byte::LocalResources {
			midi_parsers,
			clock_count,
			gui_midi_in_producer,
		} = c.local;

		while let Some((cable, byte)) = queue.lock(|q| q.dequeue()) {
			if let Some(mut usb_message) = midi_parsers[cable as usize].push(byte) {
				usb_message[0] |= cable << 4;

				gui_midi_in_producer.enqueue(usb_message).ok();

				if MIDI_INHIBIT.load(Ordering::Relaxed) {
					continue;
				}

				// send to usb
				#[cfg(feature = "debugprint_verbose")]
				debug!("MIDI{} >>> USB {:02X?}... ", cable, usb_message);
				let _ret = midi.lock(|midi| midi.send_bytes(usb_message));
				#[cfg(feature = "debugprint_verbose")]
				match _ret {
					Ok(size) => {
						debug!("wrote {} bytes to usb\n", size);
					}
					Err(error) => {
						debug!("error writing to usb: {:?}\n", error);
					}
				}

				current_preset.lock(|current_preset| {
					midi_out_queues.lock(|midi_out_queues| {
						route_message_internally(
							usb_message,
							cable,
							clock_count,
							midi_out_queues,
							current_preset,
						)
					})
				});
			}
		}
	}

	#[task(shared = [sw_uart_tx, midi_out_queues], priority = 8)]
	fn send_task(c: send_task::Context) {
		#[allow(unused_mut, unused_variables)]
		let send_task::SharedResources {
			mut sw_uart_tx,
			mut midi_out_queues,
		} = c.shared;

		let mut notify_usb_isr = false;

		sw_uart_tx.lock(|sw_uart_tx| {
			for cable in 0..NUM_PORT_PAIRS {
				if sw_uart_tx.clear_to_send(cable) {
					if let Some((byte, notify)) = midi_out_queues.lock(|q| q[cable].dequeue()) {
						#[cfg(feature = "debugprint_verbose")]
						debug!("USB >>> MIDI{} {:02X?}...\n", cable, byte);
						sw_uart_tx.send_byte(cable, byte);
						notify_usb_isr |= notify;
					}
				}
			}
		});

		if notify_usb_isr {
			rtic::pend(stm32f1xx_hal::stm32::Interrupt::USB_LP_CAN_RX0);
		}
	}

	use crate::gui_task::gui_task;
	extern "Rust" {
		#[task(shared = [current_preset, midi_out_queues], local = [gui_handler, delay, user_input_handler, flash_store, gui_midi_in_consumer], priority = 1)]
		fn gui_task(c: gui_task::Context);
	}

	#[task(binds = USB_LP_CAN_RX0, shared = [midi_out_queues, midi], local = [usb_dev, usb_midi_buffer, bootloader_sysex_statemachine], priority = 8)]
	fn usb_isr(c: usb_isr::Context) {
		let usb_isr::SharedResources {
			mut midi_out_queues,
			mut midi,
		} = c.shared;
		let usb_isr::LocalResources {
			usb_dev,
			usb_midi_buffer,
			bootloader_sysex_statemachine,
		} = c.local;
		midi.lock(|midi| {
			midi_out_queues.lock(|midi_out_queues| {
				usb_poll(
					usb_dev,
					midi,
					midi_out_queues,
					usb_midi_buffer,
					bootloader_sysex_statemachine,
				)
			})
		});

		send_task::spawn().ok();
	}

	#[cfg(feature = "benchmark")]
	#[task(shared = [sw_uart_tx], priority = 1)]
	fn benchmark_task(mut c: benchmark_task::Context) {
		debugln!("Benchmarking...").ok();
		for i in 0..3 {
			debug!("  cycle# {} ->", i).ok();
			for _ in 0..20 {
				debug!(" {}", benchmark(i)).ok();
			}
			debugln!("").ok();
		}
		debugln!("Done!").ok();
	}

	#[task(binds = TIM2, local = [mytimer, bench_timer, sw_uart_isr, dma_transfer, spi_strobe_pin],  priority=16)]
	fn timer_interrupt(c: timer_interrupt::Context) {
		#[cfg(feature = "benchmark")]
		let do_benchmark = c
			.local
			.sw_uart_isr
			.setup_benchmark(unsafe { core::ptr::read_volatile(&BENCHMARK_PHASE) });
		#[cfg(feature = "benchmark")]
		let start_time = c.local.bench_timer.cnt();

		c.local.mytimer.clear_update_interrupt_flag();

		let (recv_finished, sendbuf_consumed) = unsafe {
			tim2_interrupt_handler::optimized_interrupt_handler(
				c.local.sw_uart_isr,
				c.local.spi_strobe_pin,
				c.local.dma_transfer,
				&mut DMA_BUFFER,
				OUTPUT_MASK.load(Ordering::Relaxed),
			)
		};

		if recv_finished != 0 {
			byte_received::spawn().ok();
		}
		if sendbuf_consumed != 0 {
			send_task::spawn().ok();
		}

		#[cfg(feature = "benchmark")]
		{
			let stop_time = c.local.bench_timer.cnt();
			unsafe {
				if do_benchmark {
					core::ptr::write_volatile(&mut BENCHMARK_PHASE, -1);
					core::ptr::write_volatile(
						&mut BENCHMARK_CYCLES,
						stop_time.wrapping_sub(start_time),
					);
				}
			}
		}
	}
}

pub struct UsbMidiBuffer {
	buffer: [u8; 128],
	head: usize,
	tail: usize,
}

impl UsbMidiBuffer {
	pub fn new() -> UsbMidiBuffer {
		UsbMidiBuffer {
			buffer: [0; 128],
			head: 0,
			tail: 0,
		}
	}
	pub fn peek<B: usb_device::bus::UsbBus>(
		&mut self,
		midi: &mut usbd_midi::midi_device::MidiClass<'static, B>,
	) -> Option<[u8; 4]> {
		use core::convert::TryInto;

		if self.head == self.tail {
			if let Ok(len) = midi.read(&mut self.buffer) {
				self.head = 0;
				self.tail = len;
				assert!(self.tail <= 128);
			}
		}
		if self.head != self.tail {
			assert!(self.tail >= self.head + 4);
			return Some(self.buffer[self.head..self.head + 4].try_into().unwrap());
		}
		None
	}
	pub fn acknowledge(&mut self) {
		self.head += 4;
	}
}

fn usb_poll<B: usb_device::bus::UsbBus>(
	// FIXME inline that function in the usb_isr
	usb_dev: &mut UsbDevice<'static, B>,
	midi: &mut usbd_midi::midi_device::MidiClass<'static, B>,
	midi_out_queues: &mut [MidiOutQueue; 16],
	buffer: &mut UsbMidiBuffer,
	bootloader_sysex_statemachine: &mut BootloaderSysexStatemachine,
) {
	usb_dev.poll(&mut [midi]);

	while let Some(message) = buffer.peek(midi) {
		let cable = (message[0] & 0xF0) >> 4;

		if MIDI_INHIBIT.load(Ordering::Relaxed)
			|| midi_out_queues[cable as usize].enqueue_usb_message(message, true)
		{
			buffer.acknowledge();
		}
		else {
			break;
		}

		let is_realtime = message[1] & 0xF8 == 0xF8;
		if !is_realtime && cable == 0 {
			for i in 0..parse_midi::payload_length(message[0]) {
				bootloader_sysex_statemachine.push(message[1 + i as usize]);
			}
		}
	}
}
