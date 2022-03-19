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

use rtic::app;
use stm32f1xx_hal::{prelude::*, stm32, serial, timer, spi, dma, gpio::{Alternate, PushPull, Input, Output, Floating, PullUp, gpioa, gpiob, gpioc}};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;

use heapless::spsc::Queue;

use usb_device::bus;
use usb_device::prelude::*;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};

use parse_midi::MidiToUsbParser;

use tim2_interrupt_handler::software_uart::*;
use tim2_interrupt_handler::NumPortPairs;
use tim2_interrupt_handler::DmaPair;
use generic_array::typenum::Unsigned;

mod preset;
use preset::*;

use heapless;


#[macro_use]
mod str_writer;

mod gui;

macro_rules! debugln {
	($dst:expr, $($arg:tt)*) => {{
		if cfg!(feature = "debugprint_basic") {
			writeln!($dst, $($arg)*).ok();
		}
	}}
}

macro_rules! debug {
	($dst:expr, $($arg:tt)*) => {{
		if cfg!(feature = "debugprint_basic") {
			write!($dst, $($arg)*).ok();
		}
	}}
}

const SYSCLK : Hertz = Hertz(72_000_000);

unsafe fn reset_mcu() -> ! {
	(*stm32::SCB::ptr()).aircr.write(0x05FA0004);
	loop {}
}

unsafe fn reset_to_bootloader() -> ! {
	const BOOTKEY_ADDR: *mut u32 = 0x20003000 as *mut u32;
	core::ptr::write_volatile(BOOTKEY_ADDR, 0x157F32D4);
	reset_mcu();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	#[cfg(feature="debugpanic")]
	{
		let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
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
	
	unsafe { reset_to_bootloader(); }
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

pub struct BootloaderSysexStatemachine {
	index: usize
}

impl BootloaderSysexStatemachine {
	pub fn new() -> BootloaderSysexStatemachine {
		BootloaderSysexStatemachine { index: 0 }
	}

	pub fn push(&mut self, byte: u8) {
		let sysex = [0xF0u8, 0x00, 0x37, 0x64, 0x00, 0x00, 0x7f, 0x1e, 0x3a, 0x62];

		if byte == sysex[self.index] {
			self.index += 1;

			if self.index == sysex.len() {
				unsafe {
					reset_to_bootloader();
				}
			}
		}
		else {
			self.index = 0;
		}
	}
}

impl Default for MidiOutQueue {
	fn default() -> MidiOutQueue {
		MidiOutQueue {
			realtime: unsafe { Queue::u16_sc() },
			normal: unsafe { Queue::u16_sc() }
		}
	}
}


type TxDmaSpi2 = dma::TxDma<
		spi::Spi<
			stm32::SPI2,
			spi::Spi2NoRemap,
			(gpiob::PB13<Alternate<PushPull>>, stm32f1xx_hal::spi::NoMiso, gpiob::PB15<Alternate<PushPull>>),
			u8
		>,
		dma::dma1::C5,
	>;


pub struct WriteDmaToWriteAdapter
{
	write_dma: Option<TxDmaSpi2>
}

unsafe impl Send for WriteDmaToWriteAdapter {}


impl WriteDmaToWriteAdapter
{
	pub fn new(write_dma: TxDmaSpi2) -> WriteDmaToWriteAdapter {
		WriteDmaToWriteAdapter {
			write_dma: Some(write_dma)
		}
	}
}

impl embedded_hal::blocking::spi::Write<u8> for WriteDmaToWriteAdapter
{
	type Error = ();

	fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
		unsafe {
			// SAFETY: we will drop this reference before leaving the function
			let words_static = core::mem::transmute::<&[u8], &'static [u8]>(words);
			// SAFETY: no DMA operation is in progress
			let transfer = self.write_dma.take().unwrap().write(words_static);
			let (_, txdma) = transfer.wait();
			self.write_dma = Some(txdma);
		}
		Ok(())
	}
}

const SETTINGS_BASE: usize = (1<<17) - 2*2048;

#[derive(Debug)]
enum SaveError {
	BufferTooSmall,
	NoSpaceLeft,
	CorruptData
}

// consumes 4kb on the stack
fn save_preset_to_flash(preset_idx: u8, preset: &Preset, flash_store: &mut MyFlashStore, tx: &mut impl core::fmt::Write) -> Result<(), SaveError> {
	let mut buffer = [0; 1024];
	debugln!(tx, "saving preset {}", preset_idx);
	let len = serialize_preset(&mut None, preset);
	if len > 1024 {
		return Err(SaveError::BufferTooSmall);
	}
	debugln!(tx, "  -> serializing {} bytes", len);
	serialize_preset(&mut Some(&mut buffer[0..len]), preset);

	debugln!(tx, "  -> writing to flash");
	for i in 0..len {
		debug!(tx, "{:02X} ", buffer[i]);
	}
	debugln!(tx, "");
	let result = match flash_store.write_file(preset_idx, &buffer[0..len]) {
		Ok(()) => Ok(()),
		Err(FlashStoreError::CorruptData) => Err(SaveError::CorruptData),
		Err(FlashStoreError::NoSpaceLeft) => Err(SaveError::NoSpaceLeft),
		_ => { debugln!(tx, " -> cannot happen"); unreachable!() }
	};
	debugln!(tx, "  -> {:?}", result);
	return result;
}

// consumes 1kb on the stack
fn read_preset_from_flash(preset_idx: u8, flash_store: &mut MyFlashStore, tx: &mut impl core::fmt::Write) -> Result<Preset, SettingsError> {
	debugln!(tx, "loading preset {}", preset_idx);
	let mut buffer = [0; 1024];

	match flash_store.read_file(preset_idx, &mut buffer) {
		Ok(data) => {
			debugln!(tx, "  -> Found file of length {}", data.len());
			for i in 0..data.len() {
				debug!(tx, "{:02X} ", data[i]);
			}
			debugln!(tx, "");
			let preset = parse_preset(data)?;
			debugln!(tx, "  -> Done");
			Ok(preset)
		}
		Err(FlashStoreError::NotFound) => {
			debugln!(tx, "  -> Not found");
			Ok(Preset::new())
		}
		Err(FlashStoreError::BufferTooSmall) => Err(SettingsError), // cannot happen, we are never writing files that large
		Err(FlashStoreError::CorruptData) => Err(SettingsError),
		Err(_) => unreachable!()
	}
}

use simple_flash_store::*;

pub struct FlashAdapter {
	flash: stm32f1xx_hal::flash::Parts
}

impl FlashAdapter {
	fn new(flash: stm32f1xx_hal::flash::Parts) -> FlashAdapter {
		FlashAdapter { flash }
	}
}

type MyFlashStore = FlashStore<FlashAdapter, 2048>;

impl FlashTrait for FlashAdapter {
	const SIZE: usize = 2048;
	const PAGE_SIZE: usize = 2048; // Some chips have 1k, others have 2k. We're being pessimistic here, which is why this size differs from FlashWriter's sector size.
	const WORD_SIZE: usize = 4;
	const ERASED_VALUE: u8 = 0xFF;

	fn erase_page(&mut self, page: usize) -> Result<(), FlashAccessError> {
		// Some chips have 1k, others have 2k. We're being pessimistic here again, which is why this size differs from PAGE_SIZE.
		let mut writer = self.flash.writer(stm32f1xx_hal::flash::SectorSize::Sz1K, stm32f1xx_hal::flash::FlashSize::Sz128K);
		writer.erase((SETTINGS_BASE + page) as u32, Self::PAGE_SIZE).unwrap();
		Ok(())
	}

	fn read(&mut self, address: usize, data: &mut [u8]) -> Result<(), FlashAccessError> {
		let writer = self.flash.writer(stm32f1xx_hal::flash::SectorSize::Sz1K, stm32f1xx_hal::flash::FlashSize::Sz128K);
		data.copy_from_slice(writer.read((SETTINGS_BASE + address) as u32, data.len()).unwrap());
		Ok(())
	}

	fn write(&mut self, address: usize, data: &[u8]) -> Result<(), FlashAccessError> {
		let mut writer = self.flash.writer(stm32f1xx_hal::flash::SectorSize::Sz1K, stm32f1xx_hal::flash::FlashSize::Sz128K);
		if data.len() % 2 == 0 {
			writer.write((SETTINGS_BASE + address) as u32, data).unwrap();
		}
		else {
			writer.write((SETTINGS_BASE + address) as u32, &data[0..(data.len()-1)]).unwrap();
			writer.write((SETTINGS_BASE + address + data.len() - 1) as u32, &[data[data.len()-1], 0]).unwrap();
		}
		Ok(())
	}
}

static mut DMA_BUFFER: DmaPair = DmaPair::zero();

use core::sync::atomic::{AtomicU32, Ordering};
static OUTPUT_MASK: AtomicU32 = AtomicU32::new(0x000000F0); // FIXME init this to F0F0F0F0

fn mode_mask_to_output_mask(mode_mask: u32) -> u32 {
	let a_part = (mode_mask & 0x000F) |
	((mode_mask & 0x00F0) << 4) |
	((mode_mask & 0x0F00) << 8) |
	((mode_mask & 0xF000) << 12);

	let b_part = ((!a_part) << 4) & 0xF0F0F0;

	return a_part | b_part;
}

#[app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4])]
mod app {

	static mut SOFTWARE_UART: Option<SoftwareUart<NumPortPairs>> = None;
	static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

	use super::*; // FIXME
	#[shared]
	struct Resources {
		tx: serial::Tx<stm32::USART1>,
		queue: Queue<(u8,u8), heapless::consts::U200, u16, heapless::spsc::SingleCore>,

		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,


		sw_uart_tx: SoftwareUartTx<'static, NumPortPairs>, // belongs to send_task

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
		midi_parsers: [MidiToUsbParser; 16],

		sw_uart_rx: SoftwareUartRx<'static, NumPortPairs>,
		sw_uart_isr: SoftwareUartIsr<'static, NumPortPairs>,

		spi_strobe_pin: gpioc::PC14<Output<PushPull>>,

		dma_transfer: Option<dma::Transfer<
			dma::W,
			(&'static mut [u8; 4], &'static [u8; 4]),
			dma::RxTxDma<
				spi::Spi<
					stm32::SPI1,
					spi::Spi1Remap,
					(gpiob::PB3<Alternate<PushPull>>, gpiob::PB4<Input<Floating>>, gpiob::PB5<Alternate<PushPull>>),
					u8
				>,
				dma::dma1::C2,
				dma::dma1::C3
			>
		>>,


		display: st7789::ST7789<
			display_interface_spi::SPIInterfaceNoCS<WriteDmaToWriteAdapter, gpioa::PA2<Output<PushPull>>>, gpioa::PA1<Output<PushPull>>>,
		delay: stm32f1xx_hal::delay::Delay,

		knob_timer: stm32f1xx_hal::qei::Qei<stm32::TIM4, stm32f1xx_hal::timer::Tim4NoRemap, (gpiob::PB6<Input<PullUp>>, gpiob::PB7<Input<PullUp>>)>,
		knob_button: gpioc::PC15<Input<PullUp>>,
		flash_store: MyFlashStore,
	}

	#[init()]
	fn init(cx : init::Context) -> (Resources, LocalResources, init::Monotonics) {
		unsafe { SOFTWARE_UART = Some(SoftwareUart::new()); }
		let (sw_uart_tx, sw_uart_rx, sw_uart_isr) = unsafe { SOFTWARE_UART.as_mut().unwrap().split() };

		let dp = cx.device;
		
		// Clock configuration
		let mut flash = dp.FLASH.constrain();
		let rcc = dp.RCC.constrain();

		let clocks = rcc.cfgr
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

		// Configure the on-board LED (PC13, green)
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_high(); // Turn off

		let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

		let mut spi_strobe_pin = gpioc.pc14.into_push_pull_output_with_state(&mut gpioc.crh, stm32f1xx_hal::gpio::PinState::Low); // controls shift registers
		let clk = pb3.into_alternate_push_pull(&mut gpiob.crl);
		let miso = pb4.into_floating_input(&mut gpiob.crl);
		let mosi = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

		let mut spi = spi::Spi::spi1(dp.SPI1, (clk, miso, mosi), &mut afio.mapr, spi::Mode { phase: spi::Phase::CaptureOnFirstTransition, polarity: spi::Polarity::IdleLow }, 10.mhz(), clocks);

		// Quickly set the shift registers to their idle state. This needs to happen as quickly as possible.
		spi.transfer(&mut [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).unwrap();
		spi_strobe_pin.set_high();


		let dma = dp.DMA1.split();
		let spi_dma = spi.with_rx_tx_dma(dma.2, dma.3);
		let spi_dma_transfer = spi_dma.read_write(unsafe { &mut DMA_BUFFER.received } , unsafe { &DMA_BUFFER.transmit });

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
		let gpio_rx = gpioa.pa10;
		let serial = serial::Serial::usart1(
			dp.USART1,
			(gpio_tx, gpio_rx),
			&mut afio.mapr,
			serial::Config::default().baudrate(115200.bps()),
			clocks
		);
		let (mut tx, _rx) = serial.split();

		#[cfg(feature = "debugprint_verbose")]
		{
			writeln!(tx, "========================================================").ok();
			writeln!(tx, "midikraken @ {}", env!("VERGEN_SHA")).ok();
			writeln!(tx, "      built on {}", env!("VERGEN_BUILD_TIMESTAMP")).ok();
			if cfg!(feature = "bootloader") {
				writeln!(tx, "      bootloader enabled").ok();
			}
			else {
				writeln!(tx, "      without bootloader").ok();
			}
			writeln!(tx, "========================================================\n").ok();
		}


		// Configure the display

		let display_reset = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
		let display_dc = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
		let display_clk = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
		let display_mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
		let display_spi = spi::Spi::spi2(dp.SPI2, (display_clk, spi::NoMiso, display_mosi), spi::Mode { phase: spi::Phase::CaptureOnFirstTransition, polarity: spi::Polarity::IdleHigh }, 18.mhz(), clocks);

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
			pin_dp: usb_dp
		};
		unsafe { USB_BUS = Some( UsbBus::new(usb_pins) ) };
		let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

		let midi = usbd_midi::midi_device::MidiClass::new(usb_bus, NumPortPairs::U8, NumPortPairs::U8).unwrap();
		let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x2E80))
			.manufacturer("Windfisch")
			.product("Midikraken")
			.serial_number("TEST")
			.device_class(usbd_midi::data::usb::constants::USB_CLASS_NONE)
			.build();
	
		let mut mytimer =
			timer::Timer::tim2(dp.TIM2, &clocks)
			.start_count_down(Hertz(31250 * 3));
		mytimer.listen(timer::Event::Update);

		let knob_timer = timer::Timer::tim4(dp.TIM4, &clocks).qei(
			(gpiob.pb6.into_pull_up_input(&mut gpiob.crl), gpiob.pb7.into_pull_up_input(&mut gpiob.crl)),
			&mut afio.mapr,
			stm32f1xx_hal::qei::QeiOptions {
				slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
				auto_reload_value: 65535
			}
		);
		let knob_button = gpioc.pc15.into_pull_up_input(&mut gpioc.crh);

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
			
			benchmark_task::spawn().unwrap();
			writeln!(tx, "spawned").ok();
		}

		let mut flash_store = FlashStore::new(FlashAdapter::new(flash));
		let current_preset = read_preset_from_flash(0, &mut flash_store, &mut tx).unwrap_or(Preset::new());

		gui_task::spawn().unwrap();

		return (Resources {
			tx,
			queue,
			midi,
			sw_uart_tx,
			midi_out_queues: Default::default(),
			current_preset
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
			display,
			delay,
			knob_timer,
			knob_button,
			flash_store,
			sw_uart_rx,
			sw_uart_isr,
		},
		init::Monotonics())
	}

	#[task(shared = [queue], local = [sw_uart_rx], priority = 15)]
	fn byte_received(mut c: byte_received::Context) {
		for i in 0..NumPortPairs::USIZE {
			if let Some(byte) = c.local.sw_uart_rx.recv_byte(i) {
				c.shared.queue.lock(|queue| queue.enqueue((i as u8, byte))).ok(); // we can't do anything about this failing
			}
		}
		handle_received_byte::spawn().ok();
	}

	#[task(shared = [tx, queue, midi, current_preset, midi_out_queues], local = [midi_parsers, clock_count], priority = 7)]
	fn handle_received_byte(mut c: handle_received_byte::Context) {
		while let Some((cable, byte)) = c.shared.queue.lock(|q| { q.dequeue() }) {
			if let Some(mut usb_message) = c.local.midi_parsers[cable as usize].push(byte) {
				usb_message[0] |= cable << 4;

				// send to usb
				#[cfg(feature = "debugprint_verbose")] c.shared.tx.lock(|tx| write!(tx, "MIDI{} >>> USB {:02X?}... ", cable, usb_message).ok());
				let _ret = c.shared.midi.lock(|midi| midi.send_bytes(usb_message));
				#[cfg(feature = "debugprint_verbose")]
				match _ret {
					Ok(size) => { c.shared.tx.lock(|tx| write!(tx, "wrote {} bytes to usb\n", size).ok()); }
					Err(error) => { c.shared.tx.lock(|tx| write!(tx, "error writing to usb: {:?}\n", error).ok()); }
				}

				// process for internal routing
				let is_transport = match usb_message[1] { 0xF1 | 0xF2 | 0xF3 | 0xFA | 0xFC => true, _ => false };
				let is_clock = usb_message[1] == 0xF8;
				let is_control_ish = match usb_message[0] & 0x0F { 0xB | 0xC | 0xE | 0x4 | 0x5 | 0x6 | 0x7 => true, _ => false };
				let is_keyboard_ish = match usb_message[0] & 0x0F { 0x8 | 0x9 | 0xA | 0xD => true, _ => false };

				let ylen = c.shared.current_preset.lock(|cp| cp.event_routing_table[0].len());
				let xlen = c.shared.current_preset.lock(|cp| cp.event_routing_table.len());

				assert!(xlen == c.shared.current_preset.lock(|cp| cp.clock_routing_table.len()));
				assert!(ylen == c.shared.current_preset.lock(|cp| cp.clock_routing_table[0].len()));
				assert!(ylen == c.local.clock_count.len());
				if (cable as usize) < ylen {
					let cable_in = cable as usize;
					
					for cable_out in 0..xlen {
						let forward_event = match c.shared.current_preset.lock(|cp| cp.event_routing_table[cable_out][cable_in]) {
							EventRouteMode::Both => is_control_ish || is_keyboard_ish,
							EventRouteMode::Controller => is_control_ish,
							EventRouteMode::Keyboard => is_keyboard_ish,
							EventRouteMode::None => false
						};

						let forward_clock = match c.shared.current_preset.lock(|cp| cp.clock_routing_table[cable_out][cable_in]) {
							0 => false,
							division => is_transport || (is_clock && c.local.clock_count[cable_in] % (division as u16) == 0)
						};

						if forward_event || forward_clock {
							c.shared.midi_out_queues.lock(|qs| enqueue_message(usb_message, &mut qs[cable_out]));
							send_task::spawn().ok();
						}
					}

					if usb_message[1] == 0xFA { // start
						c.local.clock_count[cable_in] = 0;
					}
					if usb_message[1] == 0xF8 { // clock
						c.local.clock_count[cable_in] = (c.local.clock_count[cable_in] + 1) % 27720; // 27720 is the least common multiple of 1,2,...,12
					}
				}
			}
		}
	}

	#[task(shared = [tx, sw_uart_tx, midi_out_queues], priority = 8)]
	fn send_task(mut c: send_task::Context) {
		let midi_out_queues = &mut c.shared.midi_out_queues;
		let tx = &mut c.shared.tx;
		c.shared.sw_uart_tx.lock(|sw_uart_tx| {
			for cable in 0..NumPortPairs::USIZE {
				if sw_uart_tx.clear_to_send(cable) {
					if let Some(byte) = midi_out_queues.lock(|q| q[cable].realtime.dequeue()) {
						#[cfg(feature = "debugprint_verbose")] tx.lock(|tx| write!(tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte).ok());
						sw_uart_tx.send_byte(cable, byte);
					}
					else if let Some(byte) = midi_out_queues.lock(|q| q[cable].normal.dequeue()) {
						#[cfg(feature = "debugprint_verbose")] tx.lock(|tx| write!(tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte).ok());
						sw_uart_tx.send_byte(cable, byte);
					}
				}
			}
		});
	}

	#[task(shared = [tx, current_preset], local = [display, delay, knob_timer, knob_button, flash_store], priority = 1)]
	fn gui_task(mut c: gui_task::Context) {
		c.local.display.init(c.local.delay).unwrap();
		c.local.display.set_orientation(st7789::Orientation::PortraitSwapped).unwrap();
		c.local.display.clear(Rgb565::BLACK).unwrap();

		use embedded_graphics::{
				pixelcolor::Rgb565,
				prelude::*,
		};


		enum ActiveMenu {
			MainScreen(gui::MainScreenState),
			MainMenu(gui::MenuState),
			EventRouting(gui::GridState<EventRouteMode, 8, 8>),
			ClockRouting(gui::GridState<u8, 8, 8>),
			TrsModeSelect(gui::MenuState),
			SaveDestination(gui::MenuState),
			Message(gui::MessageState),
		}

		let mut active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));

		let mut old_pressed = false;
		let mut debounce = 0u8;
		let mut old_count = c.local.knob_timer.count() / 4;
		const MAX_COUNT: isize = 65536 / 4;
		let mut dirty: bool = false;
		let mut preset_idx = 0;
		let mut mode_mask = 0xFF0; // FIXME sensible initial value. actually load this from flash...
		let mut flash_used_bytes = c.local.flash_store.used_space();
		loop {
			c.local.delay.delay_ms(1u8);

			let count = c.local.knob_timer.count() / 4;
			let scroll_raw = count as isize - old_count as isize;
			let scroll =
				if scroll_raw >= MAX_COUNT / 2 { scroll_raw - MAX_COUNT }
				else if scroll_raw <= -MAX_COUNT / 2 { scroll_raw + MAX_COUNT }
				else { scroll_raw } as i16;
			old_count = count;

			let pressed = c.local.knob_button.is_low();
			if pressed != old_pressed {
				debounce = 25;
				old_pressed = pressed;
			}
			let button_event = pressed && debounce == 1;
			if debounce > 0 { debounce -= 1; }

			let mut preset = c.shared.current_preset.lock(|p| *p);

			match active_menu {
				ActiveMenu::Message(ref mut state) => {
					match state.process(scroll, button_event, false, c.local.display) {
						gui::MenuAction::Activated(_) => {
							match state.action {
								gui::MessageAction::None => {}
								gui::MessageAction::ClearFlash => {
									let flash_store = &mut c.local.flash_store;
									c.shared.tx.lock(|tx| {
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
					state.process(preset_idx, &preset, dirty, (flash_used_bytes.unwrap_or(9999), FlashAdapter::SIZE), c.local.display);
					if scroll != 0 {
						if !dirty {
							preset_idx = (preset_idx as i16 + scroll).rem_euclid(10) as usize;
							let current_preset = &mut c.shared.current_preset;
							let flash_store = &mut c.local.flash_store;
							c.shared.tx.lock(|tx| {
								current_preset.lock(|p| {
									preset = read_preset_from_flash(preset_idx as u8, flash_store, tx).unwrap_or(Preset::new());
									*p = preset;
								})
							});
						}
						else {
							state.blink_dirty();
						}
					}
					if button_event {
						active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));
					}
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
							if dirty { "Revert to saved" } else { "(nothing to revert)" },
							"Clear single preset",
							"Clear all presets",
							"Back"
						],
						c.local.display
					);
					match result {
						gui::MenuAction::Activated(index) => {
							match index {
								0 => { active_menu = ActiveMenu::EventRouting(gui::GridState::new()); }
								1 => { active_menu = ActiveMenu::ClockRouting(gui::GridState::new()); }
								2 => { active_menu = ActiveMenu::TrsModeSelect(gui::MenuState::new(0)); }
								3 => { active_menu = ActiveMenu::SaveDestination(gui::MenuState::new(preset_idx)); }
								4 => {
									if dirty {
										let current_preset = &mut c.shared.current_preset;
										let flash_store = &mut c.local.flash_store;
										c.shared.tx.lock(|tx| {
											current_preset.lock(|p| {
												if let Ok(pr) = read_preset_from_flash(preset_idx as u8, flash_store, tx) {
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
									c.shared.current_preset.lock(|p| *p = preset);
									dirty = true;
								}
								6 => { active_menu = ActiveMenu::Message(gui::MessageState::new(&["Delete all data?", "Turn off to", "abort"], "Yes", gui::MessageAction::ClearFlash)) }
								7 => { active_menu = ActiveMenu::MainScreen(gui::MainScreenState::new()) }
								_ => { unreachable!(); }
							}
						}
						_ => {}
					}
				}
				ActiveMenu::SaveDestination(ref mut menu_state) => {
					let result = menu_state.process(
						scroll,
						button_event,
						false,
						"Save to...",
						&["0","1","2","3","4","5","6","7","8","9"],
						c.local.display
					);
					match result {
						gui::MenuAction::Activated(index) => {
							if dirty || index != preset_idx {
								active_menu = ActiveMenu::MainScreen(gui::MainScreenState::new());
								let flash_store = &mut c.local.flash_store;
								let result = c.shared.tx.lock(|tx| {
									debugln!(tx, "Going to save settings to flash");
									save_preset_to_flash(index as u8, &preset, flash_store, tx)
								});
								match result {
									Ok(()) => {
										dirty = false;
										preset_idx = index;
										flash_used_bytes = flash_store.used_space();
									}
									Err(SaveError::BufferTooSmall) => {
										active_menu = ActiveMenu::Message(gui::MessageState::new(&["Preset is too large."], "Ok", gui::MessageAction::None));
									}
									Err(SaveError::NoSpaceLeft) => {
										active_menu = ActiveMenu::Message(gui::MessageState::new(&["No space left."], "Ok", gui::MessageAction::None));
									}
									Err(SaveError::CorruptData) => {
										active_menu = ActiveMenu::Message(gui::MessageState::new(&["Settings store is", "corrupt. Delete", "all data?"], "Yes", gui::MessageAction::ClearFlash));
									}
								}
							}
						}
						_ => {}
					}
				}
				ActiveMenu::TrsModeSelect(ref mut menu_state) => {
					let mut entries = heapless::Vec::<heapless::String<heapless::consts::U8>, heapless::consts::U16>::new();
					for i in 4..12 { // FIXME hardcoded
						let mut string = heapless::String::new();
						write!(&mut string, "{:2}: {}", i, if mode_mask & (1<<i) != 0 { "A  " } else { "  B" }).unwrap();
						entries.push(string).unwrap();
					}
					use core::iter::FromIterator;
					let entries_str = heapless::Vec::<_, heapless::consts::U16>::from_iter(
						entries.iter().map(|v| v.as_str())
						.chain(core::iter::once("Back"))
					);
					


					let result = menu_state.process(
						scroll,
						button_event,
						false,
						"TRS Mode Select",
						&entries_str,
						c.local.display
					);
					match result {
						gui::MenuAction::Activated(index) => {
							if index == entries_str.len() - 1 {
								active_menu = ActiveMenu::MainMenu(gui::MenuState::new(2))
							}
							else {
								mode_mask ^= 1 << (index+4);
								OUTPUT_MASK.store(mode_mask_to_output_mask(mode_mask), Ordering::Relaxed);
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
						|val, _| {
							match *val {
								EventRouteMode::None => " ",
								EventRouteMode::Keyboard => "K",
								EventRouteMode::Controller => "C",
								EventRouteMode::Both => "B",
							}
						},
						false,
						"Event Routing",
						c.local.display
					);
					match result {
						gui::GridAction::Exit => {
							active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));
						}
						gui::GridAction::ValueUpdated => {
							c.shared.current_preset.lock(|p| p.event_routing_table = preset.event_routing_table);
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
						|val, inc| { *val = (*val as i16 + inc).rem_euclid(10) as u8; },
						|val, _| { [" ","#","2","3","4","5","6","7","8","9"][*val as usize] },
						true,
						"Clock Routing/Division",
						c.local.display
					);
					match result {
						gui::GridAction::Exit => {
							active_menu = ActiveMenu::MainMenu(gui::MenuState::new(1));
						}
						gui::GridAction::ValueUpdated => {
							c.shared.current_preset.lock(|p| p.clock_routing_table = preset.clock_routing_table);
							dirty = true;
						}
						_ => {}
					}
				}
			}

		}
	}

	#[task(binds = USB_LP_CAN_RX0, shared = [midi_out_queues, midi], local = [usb_dev, usb_midi_buffer, bootloader_sysex_statemachine], priority = 8)]
	fn usb_isr(mut c: usb_isr::Context) {
		let midi_out_queues = &mut c.shared.midi_out_queues;
		let usb_dev = &mut c.local.usb_dev;
		let usb_midi_buffer = &mut c.local.usb_midi_buffer;
		let bootloader_sysex_statemachine = &mut c.local.bootloader_sysex_statemachine;
		c.shared.midi.lock(|midi|
			midi_out_queues.lock(|midi_out_queues|
				usb_poll(usb_dev, midi, midi_out_queues, usb_midi_buffer, bootloader_sysex_statemachine)
			)
		);

		send_task::spawn().ok();
	}

	#[cfg(feature = "benchmark")]
	#[task(shared = [tx, sw_uart_tx], priority = 1)]
	fn benchmark_task(mut c: benchmark_task::Context) {
		c.shared.tx.lock(|tx| {
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

	#[task(binds = TIM2, local = [mytimer, bench_timer, sw_uart_isr, dma_transfer, spi_strobe_pin],  priority=16)]
	fn timer_interrupt(c: timer_interrupt::Context) {

		#[cfg(feature = "benchmark")]
		let do_benchmark = c.local.sw_uart_isr.setup_benchmark(unsafe { core::ptr::read_volatile(&BENCHMARK_PHASE) });
		#[cfg(feature = "benchmark")]
		let start_time = c.local.bench_timer.cnt();

		c.local.mytimer.clear_update_interrupt_flag();

		let (recv_finished, sendbuf_consumed) = unsafe { tim2_interrupt_handler::optimized_interrupt_handler(
			c.local.sw_uart_isr,
			c.local.spi_strobe_pin,
			c.local.dma_transfer,
			&mut DMA_BUFFER,
			OUTPUT_MASK.load(Ordering::Relaxed)
		) };

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
					core::ptr::write_volatile(&mut BENCHMARK_CYCLES, stop_time.wrapping_sub(start_time));
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

fn enqueue_message(message: [u8; 4], queue: &mut MidiOutQueue) -> bool {
	let is_realtime = message[1] & 0xF8 == 0xF8;

	if is_realtime {
		return queue.realtime.enqueue(message[1]).is_ok();
	}
	else {
		return enqueue_all_bytes(message, &mut queue.normal);
	}
}

fn enqueue_all_bytes<T: generic_array::ArrayLength<u8>>(message: [u8; 4], queue: &mut Queue<u8, T, u16, heapless::spsc::SingleCore>) -> bool {
	let len = parse_midi::payload_length(message[0]);
	if queue.len() + len as u16 <= queue.capacity() {
		for i in 0..len {
			queue.enqueue(message[1+i as usize]).unwrap();
		}
		return true;
	}
	else {
		return false;
	}
}

fn usb_poll<B: bus::UsbBus>( // FIXME inline that function in the usb_isr
	usb_dev: &mut UsbDevice<'static, B>,
	midi: &mut usbd_midi::midi_device::MidiClass<'static, B>,
	midi_out_queues: &mut [MidiOutQueue; 16],
	buffer: &mut UsbMidiBuffer,
	bootloader_sysex_statemachine: &mut BootloaderSysexStatemachine
) {
	usb_dev.poll(&mut [midi]);

	while let Some(message) = buffer.peek(midi) {
		let cable = (message[0] & 0xF0) >> 4;
		let is_realtime = message[1] & 0xF8 == 0xF8;

		if enqueue_message(message, &mut midi_out_queues[cable as usize]) {
			buffer.acknowledge();
		}
		else {
			break;
		}
		
		if !is_realtime && cable == 0 {
			for i in 0..parse_midi::payload_length(message[0]) {
				bootloader_sysex_statemachine.push(message[1+i as usize]);
			}
		}
	}
}
