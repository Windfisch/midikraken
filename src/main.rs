#![no_main]
#![no_std]

#![feature(asm)]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

use core::ptr::{read_volatile, write_volatile};

use rtic::app;
use panic_semihosting as _;
use stm32f1xx_hal::{prelude::*, stm32, gpio::*, serial, timer, spi};
use core::fmt::Write;

use stm32f1xx_hal::time::Hertz;
const SYSCLK : Hertz = Hertz(72_000_000);

const N_UART: usize = 12;

const RECV_BIT: u16 = 1 << 10; // we need 11 bits for marker (10) + start (9) + 8x data (8-1) + stop (0) (marker is not actually transmitted over the line)
static UART_READ_BUFFER: [u16; N_UART] = [42; N_UART];
static mut uart_send: [u16; N_UART] = [0; N_UART];
static mut uart_recv: [u16; N_UART] = [0; N_UART];
const UART_SEND_IDLE: u16 = 1;

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

#[app(device = stm32f1xx_hal::pac)]
const APP: () = {
	struct Resources {
		tx: serial::Tx<stm32::USART1>,
		mytimer: timer::CountDownTimer<stm32::TIM2>
	}

	#[init]
	fn init(mut cx : init::Context) -> init::LateResources {
		//static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<MyUsbBus>> = None;

		let dp = stm32::Peripherals::take().unwrap();

		// Clock configuration
		let mut flash = dp.FLASH.constrain();
		let mut rcc = dp.RCC.constrain();

		let clocks = rcc.cfgr
			.use_hse(8.mhz())
			.sysclk(SYSCLK)
			.pclk1(36.mhz())
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
		let pb0 = gpiob.pb0.into_floating_input(&mut gpiob.crl);
		let pb1 = gpiob.pb1.into_floating_input(&mut gpiob.crl);
		let pb2 = gpiob.pb2.into_floating_input(&mut gpiob.crl);
		//let pb3 = gpiob.pb3.into_floating_input(&mut gpiob.crl);
		//let pb4 = gpiob.pb4.into_floating_input(&mut gpiob.crl);
		let pb5 = gpiob.pb5.into_floating_input(&mut gpiob.crl);
		let pb6 = gpiob.pb6.into_floating_input(&mut gpiob.crl);
		let pb7 = gpiob.pb7.into_floating_input(&mut gpiob.crl);

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
		
		let b = unsafe {
			let x = stm32::GPIOA::ptr();
			(*x).idr.read().bits()
		};

		/*
		// Configure USB
		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low().ok();
		delay(clocks.sysclk().0 / 100);
		
		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		*USB_BUS = Some( UsbBus::new(dp.USB, (usb_dm, usb_dp)) );
		let usb_bus = USB_BUS.as_ref().unwrap();

		let mut midi = usbd_midi::MidiClass::new(usb_bus);
		let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Fake company")
			.product("Serial port")
			.serial_number("TEST")
			.device_class(usbd_midi::USB_CLASS_NONE)
			.build();
		*/
		
		let mut mytimer =
			timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(38400 * 3));
		mytimer.listen(timer::Event::Update);

		// USB interrupt
		//stm32::NVIC::unmask(stm32::Interrupt::USB_LP_CAN_RX0);

		return init::LateResources { tx, mytimer };
	}

	#[task(resources = [tx], priority = 8)]
	fn byte_received(mut c: byte_received::Context) {
		writeln!(c.resources.tx, "byte received!");
		writeln!(c.resources.tx, "clear to send 0 is {}", uart_clear_to_send(0));
		uart_send_byte(0, 97);
		writeln!(c.resources.tx, "byte sent!");
		for i in 0..N_UART {
			writeln!(c.resources.tx, "#{}, has_byte = {:?}", i, uart_recv_byte(i));
		}
		writeln!(c.resources.tx, "\n------------------\n");
	}

	#[task(binds = TIM2, spawn=[byte_received], resources = [mytimer],  priority=9)]
	fn timer_interrupt(mut c: timer_interrupt::Context) {
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
			let bsrr_a = (bits_a as u32 | ((!bits_a as u32) << 16)) | MASK_A;
			let bsrr_c = (bits_c as u32 | ((!bits_c as u32) << 16)) | MASK_C;

			unsafe { (*stm32::GPIOA::ptr()).bsrr.write(|w| w.bits(bsrr_a)); }; // we ensure to only access pins
			unsafe { (*stm32::GPIOC::ptr()).bsrr.write(|w| w.bits(bsrr_c)); }; // we own (using MASK_A / MASK_C)

			*out_bits = 0;
		}


		// handle the received bits

		let in_bits: u16 = (gpiob_in & 0xfff8) >> 2 | (gpiob_in & 1);
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
	}

	extern "C" {
		fn EXTI0();
    }

};
