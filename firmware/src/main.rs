#![no_main]
#![no_std]

#[macro_use]
extern crate lazy_static;


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

mod software_uart;
use software_uart::*;

use rtic::app;
use stm32f1xx_hal::{prelude::*, stm32, serial, timer, spi, dma, gpio::{Alternate, PushPull, Input, Output, Floating, PullUp, gpioa, gpiob, gpioc}};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;

use heapless::spsc::Queue;

use usb_device::bus;
use usb_device::prelude::*;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};

use parse_midi::MidiToUsbParser;

use software_uart::typenum::Unsigned;

#[macro_use]
mod str_writer;
use str_writer::*;

mod gui;

const SYSCLK : Hertz = Hertz(72_000_000);


type NumPortPairs = software_uart::typenum::U12;


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
fn panic(info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };

	writeln!(tx, "Panic!").ok();
	writeln!(tx, "{}", info).ok();


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

#[derive(Copy,Clone,PartialEq)]
pub enum EventRouteMode {
	None,
	Keyboard,
	Controller,
	Both
}

impl EventRouteMode {
	pub fn from_u8(val: u8) -> Result<EventRouteMode,()> {
		match val {
			0 => Ok(EventRouteMode::None),
			1 => Ok(EventRouteMode::Keyboard),
			2 => Ok(EventRouteMode::Controller),
			3 => Ok(EventRouteMode::Both),
			_ => Err(())
		}
	}

	pub fn to_u8(&self) -> u8 {
		match self {
			EventRouteMode::None => 0,
			EventRouteMode::Keyboard => 1,
			EventRouteMode::Controller => 2,
			EventRouteMode::Both => 3
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


struct DmaPair {
	transmit: [u8; 4],
	received: [u8; 4]
}
impl DmaPair {
	pub const fn zero() -> DmaPair { DmaPair { transmit: [0xFF; 4], received: [0; 4] } }
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

#[derive(Debug)]
struct SettingsError;
impl From<core::array::TryFromSliceError> for SettingsError {
	fn from(_: core::array::TryFromSliceError) -> SettingsError { SettingsError{} }
}
impl From<stm32f1xx_hal::flash::Error> for SettingsError {
	fn from(_: stm32f1xx_hal::flash::Error) -> SettingsError { SettingsError{} }
}
impl From<()> for SettingsError {
	fn from(_: ()) -> SettingsError { SettingsError{} }
}

const SETTINGS_BASE: u32 = (1<<17) - 2*2048;
fn read_settings_from_flash(tx: &mut impl core::fmt::Write, flash: &mut stm32f1xx_hal::flash::Parts, settings_compressed: &mut DynArray<2048>) -> Result<(), SettingsError> {
	writeln!(tx, "reading stored settings").ok();
	let writer = flash.writer(stm32f1xx_hal::flash::SectorSize::Sz2K, stm32f1xx_hal::flash::FlashSize::Sz128K);

	let first_bytes = writer.read(SETTINGS_BASE, 2)?;
	if first_bytes == [0xFF, 0xFF] {
		writeln!(tx, "flash seems to be uninitialized").ok();
		return Ok(());
	}

	let settings_raw = writer.read(SETTINGS_BASE, 2048)?;
	settings_compressed.set_raw(settings_raw);

	Ok(())
}

fn write_settings_to_flash(tx: &mut impl core::fmt::Write, flash: &mut stm32f1xx_hal::flash::Parts, settings_compressed: &DynArray<2048>) -> Result<(), SettingsError> {
	writeln!(tx, "writing settings to flash").ok();
	let mut writer = flash.writer(stm32f1xx_hal::flash::SectorSize::Sz1K, stm32f1xx_hal::flash::FlashSize::Sz128K);
	writeln!(tx, " -> Erasing").ok();
	writer.change_verification(true);
	writer.erase(SETTINGS_BASE, 2048).unwrap();
	writeln!(tx, " -> Writing").ok();
	writer.change_verification(true); // FIXME
	writer.write(SETTINGS_BASE, settings_compressed.raw()).unwrap();
	writeln!(tx, " -> Done").ok();

	Ok(())
}

fn load_preset(tx: &mut impl core::fmt::Write, settings_compressed: &DynArray<2048>, preset_idx: usize) -> Result<Preset, SettingsError> {
	writeln!(tx, "Loading preset {}", preset_idx).ok();
	let data = settings_compressed.get(preset_idx).ok_or(SettingsError)?;
	writeln!(tx, "  -> Found").ok();
	let preset = parse_routing(data)?;
	writeln!(tx, "  -> Done").ok();

	Ok(preset)
}

fn parse_routing(data: &[u8]) -> Result<Preset, SettingsError> {
	const LEN_ENTRY: usize = 7;
	let mut preset = Preset::new();
	for chunk in data.chunks_exact(LEN_ENTRY) {
		let x = chunk[0] & 0x0F;
		let y = (chunk[0] & 0xF0) >> 4;
		let clock_divisor = chunk[1] & 0x0F;
		let event_routing = (chunk[1] & 0xF0) >> 4;
		let _channel_mask = &chunk[2..=3];
		let _map_channel = chunk[4];
		let _note_split = chunk[5];
		let _transpose = chunk[6];

		preset.clock_routing_table[x as usize][y as usize] = clock_divisor;
		preset.event_routing_table[x as usize][y as usize] = EventRouteMode::from_u8(event_routing)?;
	}

	Ok(preset)
}

fn serialize_routing(mut data_opt: Option<&mut [u8]>, preset: &Preset) -> usize {
	let mut bytes_written = 0;
	assert!(preset.event_routing_table.len() == preset.clock_routing_table.len());
	assert!(preset.event_routing_table[0].len() == preset.clock_routing_table[0].len());
	for x in 0..preset.event_routing_table.len() {
		for y in 0..preset.event_routing_table[0].len() {
			if preset.event_routing_table[x][y] != EventRouteMode::None || preset.clock_routing_table[x][y] != 0 {
				if let Some(ref mut data) = data_opt {
					 data[bytes_written+0] = ((x as u8) & 0x0F) | (((y as u8) & 0x0F) << 4);
					 data[bytes_written+1] = preset.clock_routing_table[x][y] | (preset.event_routing_table[x][y].to_u8() << 4);
					 data[(bytes_written+2)..=(bytes_written+6)].fill(0);
				}
				bytes_written += 7;
			}
		}
	}
	return bytes_written;
}

fn save_preset<const N: usize>(store: &mut DynArray<N>, preset_idx: usize, preset: &Preset) -> Result<(), SettingsError> {
	let len = serialize_routing(None, preset);
	store.resize(preset_idx, len)?;
	serialize_routing(store.get_mut(preset_idx), preset);
	Ok(())
}

pub struct DynArray <const SIZE: usize> {
	data: [u8; SIZE]
}

impl <const SIZE: usize> DynArray<SIZE> {
	fn find_offset(&self, index: usize) -> Option<usize> {
		let mut curr_index = 0;
		let mut offset = 0;
		while curr_index != index {
			curr_index += 1;
			offset += 2 + self.length_at_offset(offset);
			if offset >= self.data.len() {
				return None;
			}
		}
		return Some(offset);
	}

	fn length_at_offset(&self, offset: usize) -> usize {
		use core::convert::TryInto;
		let chunk = &self.data[offset..];
		u16::from_le_bytes(chunk[0..2].try_into().unwrap()) as usize
	}

	fn move_left(&mut self, from: usize, to: usize) {
		assert!(to < from);
		for i in 0..(self.data.len() - from) {
			self.data[to+i] = self.data[from+i];
		}
		for i in (to + self.data.len() - from) .. self.data.len() {
			self.data[i] = 0;
		}
	}

	fn move_right(&mut self, from: usize, to: usize) {
		assert!(to > from);
		for i in (to..self.data.len()).rev() {
			self.data[i] = self.data[i - to + from];
		}
	}

	pub fn resize(&mut self, index: usize, new_size: usize) -> Result<(),()> {
		if let Some(offset_to_resize) = self.find_offset(index) {
			let old_size = self.length_at_offset(offset_to_resize);
			let old_offset_next = offset_to_resize + 2 + old_size;
			let new_offset_next = offset_to_resize + 2 + new_size;
			if new_size == old_size {
				return Ok(());
			}
			else if new_size < old_size {
				self.move_left(old_offset_next, new_offset_next);
			}
			else { // new_size > old_size
				let trim_len = new_size - old_size;
				let clear = self.data[ (self.data.len() - trim_len) .. self.data.len() ].iter().all(|v| *v==0);
				if !clear {
					return Err(());
				}
				self.move_right(old_offset_next, new_offset_next);
			}

			// update the length field
			self.data[offset_to_resize..(offset_to_resize+2)].copy_from_slice(&(new_size as u16).to_le_bytes());
			return Ok(());
		}
		else {
			return Err(());
		}
	}

	pub fn get_mut(&mut self, index: usize) -> Option<&mut [u8]> {
		if let Some(offset) = self.find_offset(index) {
			let length = self.length_at_offset(offset);
			Some(&mut self.data[(offset+2) .. (offset+2+length)])
		}
		else {
			None
		}
	}

	pub fn get(&self, index: usize) -> Option<&[u8]> {
		if let Some(offset) = self.find_offset(index) {
			let length = self.length_at_offset(offset);
			Some(&self.data[(offset+2) .. (offset+2+length)])
		}
		else {
			None
		}
	}

	pub fn new() -> DynArray<SIZE> {
		DynArray { data: [0; SIZE] }
	}

	pub fn from_raw(bytes: &[u8]) -> DynArray<SIZE> {
		let mut result = DynArray::new();
		result.set_raw(bytes);
		return result;
	}

	pub fn set_raw(&mut self, bytes: &[u8]) {
		use core::convert::TryInto;
		let mut offset = 0;
		loop {
			let chunk = &bytes[offset..];
			let length = u16::from_le_bytes(chunk[0..2].try_into().unwrap()) as usize;
			let new_offset = offset + 2 + length;
			if new_offset + 2 >= self.data.len() {
				self.data[0..offset].copy_from_slice(&bytes[0..offset]);
				return;
			}
			offset = new_offset;
		}
	}

	pub fn raw(&self) -> &[u8; SIZE] {
		&self.data
	}
}


static mut DMA_BUFFER: DmaPair = DmaPair::zero();

type EventRoutingTable = [[EventRouteMode; 8]; 8];
type ClockRoutingTable = [[u8; 8]; 8];

#[derive(Clone, Copy)]
pub struct Preset {
	event_routing_table: EventRoutingTable,
	clock_routing_table: ClockRoutingTable
}

impl Preset {
	pub const fn new() -> Preset {
		Preset {
			event_routing_table: [[EventRouteMode::None; 8]; 8],
			clock_routing_table: [[0; 8]; 8]
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
		bootloader_sysex_statemachine: BootloaderSysexStatemachine,

		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,

		midi_parsers: [MidiToUsbParser; 16],
		midi_out_queues: [MidiOutQueue; 16],

		sw_uart_rx: SoftwareUartRx<'static, NumPortPairs>,
		sw_uart_tx: SoftwareUartTx<'static, NumPortPairs>,
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

		usb_midi_buffer: UsbMidiBuffer,

		display: st7789::ST7789<
			display_interface_spi::SPIInterfaceNoCS<WriteDmaToWriteAdapter, gpioa::PA2<Output<PushPull>>>, gpioa::PA1<Output<PushPull>>>,
		delay: stm32f1xx_hal::delay::Delay,

		knob_timer: stm32f1xx_hal::qei::Qei<stm32::TIM4, stm32f1xx_hal::timer::Tim4NoRemap, (gpiob::PB6<Input<PullUp>>, gpiob::PB7<Input<PullUp>>)>,
		knob_button: gpioc::PC15<Input<PullUp>>,

		current_preset: Preset,
		
		flash: stm32f1xx_hal::flash::Parts,

		settings_compressed: DynArray<2048>
	}

	#[init(spawn=[benchmark_task, gui_task])]
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

		let delay = stm32f1xx_hal::delay::Delay::new(cx.core.SYST, clocks);

		// GPIO and peripheral configuration
		let mut afio = dp.AFIO.constrain();
		let mut gpioa = dp.GPIOA.split();
		let mut gpiob = dp.GPIOB.split();
		let mut gpioc = dp.GPIOC.split();

		// Configure the on-board LED (PC13, green)
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_high(); // Turn off

		let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

		let mut spi_strobe_pin = gpioc.pc14.into_push_pull_output_with_state(&mut gpioc.crh, stm32f1xx_hal::gpio::PinState::Low); // controls shift registers
		let clk = pb3.into_alternate_push_pull(&mut gpiob.crl);
		let miso = pb4.into_floating_input(&mut gpiob.crl);
		let mosi = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

		let mut spi = spi::Spi::spi1(dp.SPI1, (clk, miso, mosi), &mut afio.mapr, spi::Mode { phase: spi::Phase::CaptureOnFirstTransition, polarity: spi::Polarity::IdleLow }, 10.mhz(), clocks);

		spi.transfer(&mut [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
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
		writeln!(tx, "========================================================").ok();
		writeln!(tx, "midikraken @ {}", env!("VERGEN_SHA")).ok();
		writeln!(tx, "      built on {}", env!("VERGEN_BUILD_TIMESTAMP")).ok();
		#[cfg(bootloader)]
		{
			writeln!(tx, "      bootloader enabled").ok();
		}
		writeln!(tx, "========================================================\n").ok();
		


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
		*USB_BUS = Some( UsbBus::new(usb_pins) );
		let usb_bus = USB_BUS.as_ref().unwrap();

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
			
			cx.spawn.benchmark_task().unwrap();
			writeln!(tx, "spawned").ok();
		}

		// FIXME this is very Cish. rustify!
		let mut settings_compressed = DynArray::new();
		read_settings_from_flash(&mut tx, &mut flash, &mut settings_compressed).expect("Reading settings from flash failed"); // FIXME
		let current_preset = load_preset(&mut tx, &settings_compressed, 0).expect("Loading preset failed");

		cx.spawn.gui_task().unwrap();

		return init::LateResources { tx, mytimer, queue, usb_dev, midi,
			sw_uart_tx, sw_uart_rx, sw_uart_isr,
			midi_out_queues: Default::default(),
			midi_parsers: Default::default(),
			#[cfg(feature = "benchmark")]
			bench_timer,
			usb_midi_buffer: UsbMidiBuffer::new(),
			bootloader_sysex_statemachine: BootloaderSysexStatemachine::new(),
			dma_transfer: Some(spi_dma_transfer),
			spi_strobe_pin,
			display,
			delay,
			knob_timer,
			knob_button,
			flash,
			settings_compressed,
			current_preset
		};
	}

	#[task(spawn = [handle_received_byte], resources = [queue, sw_uart_rx], priority = 99)]
	fn byte_received(c: byte_received::Context) {
		for i in 0..NumPortPairs::USIZE {
			if let Some(byte) = c.resources.sw_uart_rx.recv_byte(i) {
				c.resources.queue.enqueue((i as u8, byte)).ok(); // we can't do anything about this failing
			}
		}
		c.spawn.handle_received_byte().ok();
	}

	#[task(spawn = [send_task], resources = [tx, queue, midi_parsers, midi, current_preset, midi_out_queues], priority = 40)]
	fn handle_received_byte(mut c: handle_received_byte::Context) {
		static mut clock_count: [u16; 8] = [0; 8];

		while let Some((cable, byte)) = c.resources.queue.lock(|q| { q.dequeue() }) {
			if let Some(mut usb_message) = c.resources.midi_parsers[cable as usize].push(byte) {
				usb_message[0] |= cable << 4;

				// send to usb
				#[cfg(feature = "debugprint")] c.resources.tx.lock(|tx| write!(tx, "MIDI{} >>> USB {:02X?}... ", cable, usb_message).ok());
				let _ret = c.resources.midi.lock(|midi| midi.send_bytes(usb_message));
				#[cfg(feature = "debugprint")]
				match _ret {
					Ok(size) => { c.resources.tx.lock(|tx| write!(tx, "wrote {} bytes to usb\n", size).ok()); }
					Err(error) => { c.resources.tx.lock(|tx| write!(tx, "error writing to usb: {:?}\n", error).ok()); }
				}

				// process for internal routing
				let is_transport = match usb_message[1] { 0xF1 | 0xF2 | 0xF3 | 0xFA | 0xFC => true, _ => false };
				let is_clock = usb_message[1] == 0xF8;
				let is_control_ish = match usb_message[0] & 0x0F { 0xB | 0xC | 0xE | 0x4 | 0x5 | 0x6 | 0x7 => true, _ => false };
				let is_keyboard_ish = match usb_message[0] & 0x0F { 0x8 | 0x9 | 0xA | 0xD => true, _ => false };

				assert!(c.resources.current_preset.event_routing_table.len() == c.resources.current_preset.clock_routing_table.len());
				assert!(c.resources.current_preset.event_routing_table[0].len() == c.resources.current_preset.clock_routing_table[0].len());
				assert!(c.resources.current_preset.event_routing_table[0].len() == clock_count.len());
				if (cable as usize) < c.resources.current_preset.event_routing_table[0].len() {
					let cable_in = cable as usize;
					
					for cable_out in 0..c.resources.current_preset.event_routing_table.len() {
						let forward_event = match c.resources.current_preset.event_routing_table[cable_out][cable_in] {
							EventRouteMode::Both => is_control_ish || is_keyboard_ish,
							EventRouteMode::Controller => is_control_ish,
							EventRouteMode::Keyboard => is_keyboard_ish,
							EventRouteMode::None => false
						};

						let forward_clock = match c.resources.current_preset.clock_routing_table[cable_out][cable_in] {
							0 => false,
							division => is_transport || (is_clock && clock_count[cable_in] % (division as u16) == 0)
						};

						if forward_event || forward_clock {
							c.resources.midi_out_queues.lock(|qs| enqueue_message(usb_message, &mut qs[cable_out]));
							c.spawn.send_task().ok();
						}
					}

					if usb_message[1] == 0xFA { // start
						clock_count[cable_in] = 0;
					}
					if usb_message[1] == 0xF8 { // clock
						clock_count[cable_in] = (clock_count[cable_in] + 1) % 27720; // 27720 is the least common multiple of 1,2,...,12
					}
				}
			}
		}
	}

	#[task(resources = [tx, sw_uart_tx, midi_out_queues], priority = 50)]
	fn send_task(c: send_task::Context) {
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
	}

	#[task(resources = [tx, display, delay, knob_timer, knob_button, current_preset, settings_compressed, flash], priority = 1)]
	fn gui_task(mut c: gui_task::Context) {
		c.resources.display.init(c.resources.delay).unwrap();
		c.resources.display.set_orientation(st7789::Orientation::PortraitSwapped).unwrap();
		c.resources.display.clear(Rgb565::BLACK).unwrap();

		use embedded_graphics::{
				pixelcolor::Rgb565,
				prelude::*,
		};


		enum ActiveMenu {
			MainScreen(gui::MainScreenState),
			MainMenu(gui::MenuState),
			EventRouting(gui::GridState<EventRouteMode, 8, 8>),
			ClockRouting(gui::GridState<u8, 8, 8>),
		}

		let mut active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));

		let mut old_pressed = false;
		let mut debounce = 0u8;
		let mut old_count = c.resources.knob_timer.count() / 4;
		const MAX_COUNT: isize = 65536 / 4;
		let mut dirty: bool = false;
		let mut preset_idx = 0;
		loop {
			c.resources.delay.delay_ms(1u8);

			let count = c.resources.knob_timer.count() / 4;
			let scroll_raw = count as isize - old_count as isize;
			let scroll =
				if scroll_raw >= MAX_COUNT / 2 { scroll_raw - MAX_COUNT }
				else if scroll_raw <= -MAX_COUNT / 2 { scroll_raw + MAX_COUNT }
				else { scroll_raw } as i16;
			old_count = count;

			let pressed = c.resources.knob_button.is_low();
			if pressed != old_pressed {
				debounce = 25;
				old_pressed = pressed;
			}
			let button_event = pressed && debounce == 1;
			if debounce > 0 { debounce -= 1; }

			let mut preset = c.resources.current_preset.lock(|p| *p);

			match active_menu {
				ActiveMenu::MainScreen(ref mut state) => {
					state.process(preset_idx, dirty, c.resources.display);
					if scroll != 0 {
						if !dirty {
							preset_idx = (preset_idx as i16 + scroll).rem_euclid(10) as usize;
							let current_preset = &mut c.resources.current_preset;
							let settings_compressed = &mut c.resources.settings_compressed;
							c.resources.tx.lock(|tx| {
								current_preset.lock(|p| {
									preset = load_preset(tx, settings_compressed, preset_idx).unwrap();
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
							"Event Routing",
							"Clock Routing",
							if dirty { "Save" } else { "(nothing to save)" },
							"Back"
						],
						c.resources.display
					);
					match result {
						gui::MenuAction::Activated(index) => {
							match index {
								0 => { active_menu = ActiveMenu::EventRouting(gui::GridState::new()); }
								1 => { active_menu = ActiveMenu::ClockRouting(gui::GridState::new()); }
								2 => {
									let flash = &mut c.resources.flash;
									let settings_compressed = &mut c.resources.settings_compressed;
									c.resources.tx.lock(|tx| {
										writeln!(tx, "Going to save settings to flash").ok();
										save_preset(settings_compressed, preset_idx, &preset).unwrap(); // FIXME this must be handled!
										write_settings_to_flash(tx, flash, settings_compressed).unwrap();
									});
									dirty = false;
									menu_state.schedule_redraw();
								}
								3 => { active_menu = ActiveMenu::MainScreen(gui::MainScreenState::new()) }
								_ => { unreachable!(); }
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
						c.resources.display
					);
					match result {
						gui::GridAction::Exit => {
							active_menu = ActiveMenu::MainMenu(gui::MenuState::new(0));
						}
						gui::GridAction::ValueUpdated => {
							c.resources.current_preset.lock(|p| p.event_routing_table = preset.event_routing_table);
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
						c.resources.display
					);
					match result {
						gui::GridAction::Exit => {
							active_menu = ActiveMenu::MainMenu(gui::MenuState::new(1));
						}
						gui::GridAction::ValueUpdated => {
							c.resources.current_preset.lock(|p| p.clock_routing_table = preset.clock_routing_table);
							dirty = true;
						}
						_ => {}
					}
				}
			}

		}
	}

	#[task(binds = USB_LP_CAN_RX0, spawn = [send_task], resources = [midi_out_queues, usb_dev, midi, usb_midi_buffer, bootloader_sysex_statemachine], priority = 50)]
	fn usb_isr(mut c: usb_isr::Context) {
		usb_poll(&mut c.resources.usb_dev, &mut c.resources.midi, &mut c.resources.midi_out_queues, &mut c.resources.usb_midi_buffer, c.resources.bootloader_sysex_statemachine);

		c.spawn.send_task().ok();
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

	#[task(binds = TIM2, spawn=[byte_received, send_task], resources = [mytimer, bench_timer, sw_uart_isr, dma_transfer, spi_strobe_pin],  priority=100)]
	fn timer_interrupt(c: timer_interrupt::Context) {

		#[cfg(feature = "benchmark")]
		let do_benchmark = c.resources.sw_uart_isr.setup_benchmark(unsafe { core::ptr::read_volatile(&BENCHMARK_PHASE) });
		#[cfg(feature = "benchmark")]
		let start_time = c.resources.bench_timer.cnt();

		c.resources.mytimer.clear_update_interrupt_flag();

		// handle the SPI DMA
		let (_, spi_dma) = c.resources.dma_transfer.take().unwrap().wait();
		c.resources.spi_strobe_pin.set_low();

		let mut in_bits: u32;
		{
			// this is safe in and only in this scope, since the DMA transfers are currently halted
			let dma_buffer = unsafe { &mut DMA_BUFFER };

			in_bits = u32::from_le_bytes(dma_buffer.received);

			// FIXME make this configurable
			if true {
				in_bits = (in_bits & 0x0000000F) | ((in_bits & 0xFFFFFF00) >> 4);
			}
			if false {
				in_bits = (in_bits & 0x000000FF) | ((in_bits & 0xFFFFF000) >> 4);
			}
			if false {
				in_bits = (in_bits & 0x00000FFF) | ((in_bits & 0xFFFF0000) >> 4);
			}
		
			if let Some(out_bits) = c.resources.sw_uart_isr.out_bits() {
				// We first fill each byte with two concatenated copies of the same 4 bit block.
				// Then, for each identical bit pair, we OR exactly one of these to always-1.
				// For DIN-5-midi, one board contains a 8 bit shift register but only 4 ports,
				// so we just ignore the high nibble on these boards (not wired).
				// For TRS-midi, the lower nibble is wired to the tips and the high nibble to
				// the rings; exactly one of these is always-high and the other carries the signal.
				// This allows switching between TRS-A and TRS-B midi in software.
				let chunked_out_bits =
					((out_bits & 0x000F) as u32) << 0 |
					((out_bits & 0x00F0) as u32) << 4 |
					((out_bits & 0x0F00) as u32) << 8 |
					((out_bits & 0xF000) as u32) << 12;
				let mask = 0xF0F00FF0;
				let raw_out_bits = (chunked_out_bits | (chunked_out_bits << 4)) | mask;
				//dma_buffer.transmit = (out_bits as u32).reverse_bits().to_le_bytes();
				dma_buffer.transmit = (raw_out_bits as u32).to_be_bytes();
			}
		}

		c.resources.spi_strobe_pin.set_high();
		*c.resources.dma_transfer = Some(spi_dma.read_write(
			unsafe { &mut DMA_BUFFER.received },
			unsafe { &DMA_BUFFER.transmit }
		));

		let (recv_finished, sendbuf_consumed) = c.resources.sw_uart_isr.process((in_bits & 0xFFFF) as u16);
		if recv_finished != 0 {
			c.spawn.byte_received().ok();
		}
		if sendbuf_consumed != 0 {
			c.spawn.send_task().ok();
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
		fn EXTI3();
		fn EXTI4();
		fn EXTI5();
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
