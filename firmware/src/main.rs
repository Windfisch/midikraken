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

use embedded_hal::digital::v2::OutputPin;

mod software_uart;
use software_uart::*;

use rtic::app;
use stm32f1xx_hal::{prelude::*, stm32, serial, timer, spi, dma, gpio::{Alternate, PushPull, Input, Output, Floating, gpioa, gpiob, gpioc}};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;

use heapless::spsc::Queue;

use usb_device::bus;
use usb_device::prelude::*;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};

use parse_midi::MidiToUsbParser;

use software_uart::typenum::Unsigned;

const SYSCLK : Hertz = Hertz(72_000_000);


type NumPortPairs = software_uart::typenum::U12;


unsafe fn reset_mcu() {
	(*stm32::SCB::ptr()).aircr.write(0x05FA0004);
}

unsafe fn reset_to_bootloader() {
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

static mut DMA_BUFFER: DmaPair = DmaPair::zero();

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
				spi::SpiPayload<
					stm32::SPI1,
					spi::Spi1Remap,
					(gpiob::PB3<Alternate<PushPull>>, gpiob::PB4<Input<Floating>>, gpiob::PB5<Alternate<PushPull>>)
				>,
				dma::dma1::C2,
				dma::dma1::C3
			>
		>>,

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

		let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

		let mut spi_strobe_pin = gpioc.pc14.into_push_pull_output_with_state(&mut gpioc.crh, stm32f1xx_hal::gpio::State::Low); // controls shift registers
		let clk = pb3.into_alternate_push_pull(&mut gpiob.crl);
		let miso = pb4.into_floating_input(&mut gpiob.crl);
		let mosi = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

		let mut spi = spi::Spi::spi1(dp.SPI1, (clk, miso, mosi), &mut afio.mapr, spi::Mode { phase: spi::Phase::CaptureOnFirstTransition, polarity: spi::Polarity::IdleLow }, 10.mhz(), clocks, &mut rcc.apb2);

		spi.transfer(&mut [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
		spi_strobe_pin.set_high();


		let dma = dp.DMA1.split(&mut rcc.ahb);
		let spi_dma = spi.with_rx_tx_dma(dma.2, dma.3);
		let spi_dma_transfer = spi_dma.read_write(unsafe { &mut DMA_BUFFER.received } , unsafe { &DMA_BUFFER.transmit });

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
		#[cfg(bootloader)]
		writeln!(tx, "      bootloader enabled").ok();
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
			usb_midi_buffer: UsbMidiBuffer::new(),
			bootloader_sysex_statemachine: BootloaderSysexStatemachine::new(),
			dma_transfer: Some(spi_dma_transfer),
			spi_strobe_pin
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

	#[task(resources = [tx, queue, midi_parsers, midi_out_queues, usb_dev, midi, sw_uart_tx, usb_midi_buffer, bootloader_sysex_statemachine], priority = 2)]
	fn mainloop(mut c: mainloop::Context) {
		loop {
			usb_poll(&mut c.resources.usb_dev, &mut c.resources.midi, &mut c.resources.midi_out_queues, &mut c.resources.usb_midi_buffer, c.resources.bootloader_sysex_statemachine);

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

	#[task(binds = TIM2, spawn=[byte_received], resources = [mytimer, bench_timer, sw_uart_isr, dma_transfer, spi_strobe_pin],  priority=100)]
	fn timer_interrupt(c: timer_interrupt::Context) {

		#[cfg(feature = "benchmark")]
		let do_benchmark = c.resources.sw_uart_isr.setup_benchmark(unsafe { core::ptr::read_volatile(&BENCHMARK_PHASE) });
		#[cfg(feature = "benchmark")]
		let start_time = c.resources.bench_timer.cnt();

		c.resources.mytimer.clear_update_interrupt_flag();

		// handle the SPI DMA
		let (_, spi_dma) = c.resources.dma_transfer.take().unwrap().wait();
		c.resources.spi_strobe_pin.set_low().unwrap();

		let mut in_bits: u32;
		{
			// this is safe in and only in this scope, since the DMA transfers are currently halted
			let dma_buffer = unsafe { &mut DMA_BUFFER };

			in_bits = u32::from_be_bytes(dma_buffer.received).reverse_bits();

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
				let mask = 0xF00FF0F0;
				let raw_out_bits = (chunked_out_bits | (chunked_out_bits << 4)) | mask;
				//dma_buffer.transmit = (out_bits as u32).reverse_bits().to_le_bytes();
				dma_buffer.transmit = (raw_out_bits as u32).to_be_bytes();
			}
		}

		c.resources.spi_strobe_pin.set_high().unwrap();
		*c.resources.dma_transfer = Some(spi_dma.read_write(
			unsafe { &mut DMA_BUFFER.received },
			unsafe { &DMA_BUFFER.transmit }
		));

		if c.resources.sw_uart_isr.process((in_bits & 0xFFFF) as u16) != 0 {
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
	bootloader_sysex_statemachine: &mut BootloaderSysexStatemachine
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
			}
			else {
				break;
			}

			buffer.acknowledge();

			if cable == 0 {
				for i in 0..len {
					bootloader_sysex_statemachine.push(message[1+i as usize]);
				}
			}
		};
	}
}
