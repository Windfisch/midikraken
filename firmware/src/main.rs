#![no_main]
#![no_std]

#![feature(asm)]

use embedded_hal::digital::v2::OutputPin;

use core::ptr::{read_volatile, write_volatile};

use rtic::app;
use panic_semihosting as _;
use stm32f1xx_hal::{prelude::*, stm32, gpio::*, serial, timer, spi};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;

use heapless::spsc::Queue;

use usb_device::bus;
use usb_device::prelude::*;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};

use parse_midi::MidiToUsbParser;

type MyUsbPins = (stm32f1xx_hal::gpio::gpioa::PA11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>, stm32f1xx_hal::gpio::gpioa::PA12<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>);
type MyUsbBus = UsbBus<MyUsbPins>;

const SYSCLK : Hertz = Hertz(72_000_000);

const N_UART: usize = 12;

const RECV_BIT: u16 = 1 << 10; // we need 11 bits for marker (10) + start (9) + 8x data (8-1) + stop (0) (marker is not actually transmitted over the line)
static UART_READ_BUFFER: [u16; N_UART] = [42; N_UART];
static mut uart_send: [u16; N_UART] = [0; N_UART];
static mut uart_recv: [u16; N_UART] = [0; N_UART];
const UART_SEND_IDLE: u16 = 1;

#[cfg(feature = "benchmark")]
static mut uart_benchmark: i8 = -1;
#[cfg(feature = "benchmark")]
static mut benchmark_cycles: u16 = 0;

#[cfg(feature = "benchmark")]
fn benchmark(cycle: i8) -> u16 {
	if cycle < 0 || cycle >= 3 {
		return 0;
	}

	unsafe {
		write_volatile(&mut benchmark_cycles, 0);
		write_volatile(&mut uart_benchmark, cycle);
		loop {
			let result = read_volatile(&benchmark_cycles);
			if result != 0 {
				return result;
			}
		}
	}
}

fn uart_clear_to_send(index: usize) -> bool {
	unsafe {
		return read_volatile(&uart_send[index]) == UART_SEND_IDLE;
	}
}

fn uart_send_byte(index: usize, data: u8) {
	unsafe {
		write_volatile(&mut uart_send[index], ((data as u16) << 1) | (1 << 9));
	}
}

fn uart_recv_byte(index: usize) -> Option<u8> {
	unsafe {
		let data = read_volatile(&uart_recv[index]);
		write_volatile(&mut uart_recv[index], 0);

		if data != 0 {
			return Some(((data >> 2) & 0xFF) as u8); // shift out the marker bit and the start bit. then AND out the stop bit
		}
		else {
			return None;
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
		midi_out_queues: [MidiOutQueue; 16]
	}

	#[init(spawn=[benchmark_task, mainloop])]
	fn init(mut cx : init::Context) -> init::LateResources {
		static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

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

		let pa0 = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
		let pa1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
		let pa2 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
		let pa3 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
		let pb5 = gpiob.pb5.into_floating_input(&mut gpiob.crl);
		let pb6 = gpiob.pb6.into_floating_input(&mut gpiob.crl);
		let pb7 = gpiob.pb7.into_floating_input(&mut gpiob.crl);
		let pb8 = gpiob.pb8.into_floating_input(&mut gpiob.crh);

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

		let midi = usbd_midi::midi_device::MidiClass::new(usb_bus, 4, 4).unwrap();
		let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Windfisch")
			.product("Midikraken x16")
			.serial_number("TEST")
			.device_class(usbd_midi::data::usb::constants::USB_CLASS_NONE)
			.build();
	
		
		let mut mytimer =
			timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(31250 * 3));
		mytimer.listen(timer::Event::Update);

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
			writeln!(tx, "delay for 100 cycles took from cpu cycle {} to {}", start_100, stop_100);
			writeln!(tx, "delay for 40000 took from cpu cycle {} to {}", start_40000, stop_40000);
			writeln!(tx, "sleep(1 second)");
			cortex_m::asm::delay(72000000);
			writeln!(tx, "sleep(1 second)");
			cortex_m::asm::delay(72000000);
			writeln!(tx, "done");
			
			cx.spawn.benchmark_task();
		}
		

		let queue = unsafe { Queue::u16_sc() };

		cx.spawn.mainloop();

		let midi_out_queues: [MidiOutQueue; 16] = Default::default();

		return init::LateResources { tx, mytimer, queue, usb_dev, midi, midi_out_queues,
			midi_parsers: [
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
				MidiToUsbParser::new(),
			],
			#[cfg(feature = "benchmark")]
			bench_timer
		};
	}

	#[task(resources = [queue], priority = 8)]
	fn byte_received(mut c: byte_received::Context) {
		for i in 0..N_UART {
			if let Some(byte) = uart_recv_byte(i) {
				c.resources.queue.enqueue((i as u8, byte));
			}
		}
	}

	#[task(resources = [tx, queue, midi_parsers, midi_out_queues, usb_dev, midi], priority = 2)]
	fn mainloop(mut c: mainloop::Context) {
		loop {
			if is_any_queue_full(c.resources.midi_out_queues) {
				write!(c.resources.tx, "!");
			}

			usb_poll(&mut c.resources.usb_dev, &mut c.resources.midi, &mut c.resources.midi_out_queues);

			for cable in 0..N_UART {
				if uart_clear_to_send(cable) {
					if let Some(byte) = c.resources.midi_out_queues[cable].realtime.dequeue() {
						write!(c.resources.tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte);
						uart_send_byte(cable, byte);
					}
					else if let Some(byte) = c.resources.midi_out_queues[cable].normal.dequeue() {
						write!(c.resources.tx, "USB >>> MIDI{} {:02X?}...\n", cable, byte);
						uart_send_byte(cable, byte);
					}
				}
			}

			while let Some((cable, byte)) = c.resources.queue.lock(|q| { q.split().1.dequeue() }) {
				//write!(c.resources.tx, "({},{:02X}) ", cable, byte);
				if let Some(mut usb_message) = c.resources.midi_parsers[cable as usize].push(byte) {
					write!(c.resources.tx, "MIDI{} >>> USB {:02X?}... ", cable, usb_message);
					usb_message[0] |= cable << 4;
					match c.resources.midi.send_bytes(usb_message) {
						Ok(size) => { write!(c.resources.tx, "wrote {} bytes to usb\n", size); }
						Err(error) => { write!(c.resources.tx, "error writing to usb: {:?}\n", error); }
					}
				}
			}
		}
	}

	#[cfg(feature = "benchmark")]
	#[task(resources = [tx], priority = 1)]
	fn benchmark_task(mut c: benchmark_task::Context) {
		c.resources.tx.lock(|tx| {
			writeln!(tx, "Benchmarking...");
			for i in 0..3 {
				write!(tx, "  cycle# {} ->", i);
				for _ in 0..20 {
					write!(tx, " {}", benchmark(i));
				}
				writeln!(tx, "");
			}
			writeln!(tx, "Done!");
		});
	}

	#[task(binds = TIM2, spawn=[byte_received], resources = [mytimer, bench_timer],  priority=100)]
	fn timer_interrupt(c: timer_interrupt::Context) {
		// We have a total of 32 GPIO ports on the blue pill board, of
		// which 2 are used for USB and 1 is used for the LED.
		// B0-1, B3-15, A0-10, A15, C14-15
		// we use B0, B3, B4, ..., B15 for our 14 input uarts and
		// A0, ..., A10, A15, C14, C15 for the 14 output pins
		static mut in_bits_old: u16 = 0;
		static mut recv_active: [u16; 3] = [0; 3];
		static mut phase: usize = 0;

		static mut recv_workbuf: [u16; N_UART] = [RECV_BIT; N_UART];
		static mut send_workbuf: [u16; N_UART] = [0; N_UART];
		static mut out_bits: u16 = 0;

		#[cfg(feature = "benchmark")]
		let do_benchmark = *phase as i8 == unsafe { read_volatile(&uart_benchmark) };
		#[cfg(feature = "benchmark")]
		if do_benchmark {
			recv_active[*phase] = 0xFFFF;
			for i in 0..N_UART {
				recv_workbuf[i] = 2; // this will be the last bit received, causing additional work to happen
				unsafe { write_volatile(&mut uart_send[i], (0xFF << 1) | (1<<9)); }
				send_workbuf[i] = 0; // force an (expensive) reload to happen
			}
			unsafe { write_volatile(&mut uart_benchmark, -1); }
		}
		#[cfg(feature = "benchmark")]
		let start_time = c.resources.bench_timer.cnt();

		c.resources.mytimer.clear_update_interrupt_flag();

		let next_phase = (*phase + 1) % 3;


		// actual I/O is done at the very beginning to minimize jitter

		// input
		let gpiob_in = unsafe { (*stm32::GPIOB::ptr()).idr.read().bits() } as u16; // read the IO port

		// output: *out_bits have been set in the last three thirdclocks.
		if *phase == 0 {
			const MASK_A: u32 = 0x87FF87FF;
			const MASK_C: u32 = 0xC000C000;
			let bits_a = (*out_bits & 0x7FF) | ((*out_bits & 0x800) << 4);
			let bits_c = (*out_bits & 0x3000) << 2;
			let bsrr_a = (bits_a as u32 | ((!bits_a as u32) << 16)) & MASK_A;
			let bsrr_c = (bits_c as u32 | ((!bits_c as u32) << 16)) & MASK_C;

			unsafe { (*stm32::GPIOA::ptr()).bsrr.write(|w| w.bits(bsrr_a)); }; // we ensure to only access pins
			unsafe { (*stm32::GPIOC::ptr()).bsrr.write(|w| w.bits(bsrr_c)); }; // we own (using MASK_A / MASK_C)

			*out_bits = 0;
		}


		// handle the received bits

		let in_bits: u16 = (gpiob_in >> 5) & 0x000F;
		let falling_edge = !in_bits & *in_bits_old;
		*in_bits_old = in_bits;
		
		let active_total = recv_active[0] | recv_active[1] | recv_active[2];
		let start_of_transmission = !active_total & falling_edge;
		recv_active[next_phase] |= start_of_transmission;

		let mut finished = 0;
		for i in 0..N_UART {
			let mask = 1 << i;
		
			// TODO try moving this variable out of the loop. does this avoid unneeded reads / array bound checks?
			if recv_active[*phase] & mask != 0 { // this is the thirdclock where uart #i can read stable data?
				/*let mut recv_bit = 0;
				if in_bits & mask != 0 {
					recv_bit = RECV_BIT;
				}*/
				let recv_bit = if in_bits & mask == 0 { 0 } else { RECV_BIT };

				recv_workbuf[i] = (recv_workbuf[i] >> 1) | recv_bit;

				if recv_workbuf[i] & 1 != 0 { // we received 10 bits, i.e. the marker bit is now the LSB?
					unsafe { write_volatile(&mut uart_recv[i], recv_workbuf[i]); } // publish the received uart frame.
					recv_workbuf[i] = RECV_BIT;
					finished |= mask;
				}
			}
		}
		recv_active[*phase] &= !finished;
		if finished != 0 {
			c.spawn.byte_received();
		}

		// handle the bits to be sent; in three thirdclocks, we prepare *out_bits.
		const SEND_BATCHSIZE: usize = (N_UART+2) / 3;
		let first = *phase * SEND_BATCHSIZE;
		for i in first .. core::cmp::min(first + SEND_BATCHSIZE, N_UART) {
			if send_workbuf[i] == 0 {
				unsafe { // STM32 reads and writes u16s atomically
					send_workbuf[i] = read_volatile(&uart_send[i]);
					write_volatile(&mut uart_send[i], UART_SEND_IDLE);
				}
			}
			
			*out_bits |= (send_workbuf[i] & 1) << i;
			send_workbuf[i] >>= 1;
		}

		// at the begin of phase 0, output ports are set.

		*phase = next_phase;

		#[cfg(feature = "benchmark")]
		{
			let stop_time = c.resources.bench_timer.cnt();
			unsafe {
				if do_benchmark && read_volatile(&benchmark_cycles) == 0 {
					write_volatile(&mut benchmark_cycles, stop_time.wrapping_sub(start_time));
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

fn is_any_queue_full(midi_out_queues: &[MidiOutQueue]) -> bool {
	midi_out_queues.iter().any(|qs| qs.realtime.len()+1 > qs.realtime.capacity() || qs.normal.len()+3 > qs.normal.capacity()-4)
}

fn usb_poll<B: bus::UsbBus>(
	usb_dev: &mut UsbDevice<'static, B>,
	midi: &mut usbd_midi::midi_device::MidiClass<'static, B>,
	midi_out_queues: &mut [MidiOutQueue; 16],
) {
	if !usb_dev.poll(&mut [midi]) {
		return;
	}

	while !is_any_queue_full(midi_out_queues) {
		let mut buffer = [0u8;128]; // FIXME magic size.
		if let Ok(len) = midi.read(&mut buffer) {
			if len % 4 == 0 { // every packet should have length 4
				for message in buffer[0..len].chunks(4) {
					let cable = (message[0] & 0xF0) >> 4;
					let is_realtime = message[1] & 0xF8 == 0xF8;

					if is_realtime {
						midi_out_queues[cable as usize].realtime.split().0.enqueue(message[1]);
					}
					else {
						for i in 0..parse_midi::payload_length(message[0]) {
							midi_out_queues[cable as usize].normal.split().0.enqueue(message[1+i as usize]);
						}
					}
				}
			}
		}
		else {
			break;
		}
	}
}
