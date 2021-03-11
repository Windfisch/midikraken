#![no_main]
#![no_std]

#![feature(asm)]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

//use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use panic_semihosting as _;
use cortex_m::asm::delay;
use stm32f1xx_hal::{prelude::*, stm32, gpio::*, serial, timer, spi};
use core::fmt::Write;
use stm32f1xx_hal::usb::UsbBus;
use stm32::interrupt;

//use stm32f1;
//use stm32f1xx_hal::pac::NVIC_PRIO_BITS;

use stm32f1xx_hal::time::Hertz;
const SYSCLK : Hertz = Hertz(72_000_000);


static UART_READ_BUFFER: [u16; 16] = [42; 16];

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
		//let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
		let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

		// Configure the on-board LED (PC13, green)
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_high().ok(); // Turn off

		let pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);
		let pa1 = gpioa.pa1.into_floating_input(&mut gpioa.crl);
		let pa2 = gpioa.pa2.into_floating_input(&mut gpioa.crl);
		let pa3 = gpioa.pa3.into_floating_input(&mut gpioa.crl);
		let pa4 = gpioa.pa4.into_floating_input(&mut gpioa.crl);
		let pa5 = gpioa.pa5.into_floating_input(&mut gpioa.crl);
		let pa6 = gpioa.pa6.into_floating_input(&mut gpioa.crl);
		let pa7 = gpioa.pa7.into_floating_input(&mut gpioa.crl);

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
		let gpio_rx = gpioa.pa10;
		let serial = serial::Serial::usart1(
			dp.USART1,
			(gpio_tx, gpio_rx),
			&mut afio.mapr,
			serial::Config::default().baudrate(115200.bps()),
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
			.start_count_down(Hertz(1));
		mytimer.listen(timer::Event::Update);

		// USB interrupt
		//stm32::NVIC::unmask(stm32::Interrupt::USB_LP_CAN_RX0);

		return init::LateResources { tx, mytimer };
	}
	
	#[task(binds = TIM2, resources = [tx, mytimer],  priority=9)]
	fn timer_interrupt(mut c: timer_interrupt::Context) {
		static mut in_bits_old: u16 = 0;
		static mut active: [u16; 3] = [0; 3];
		static mut current: usize = 0;

		static mut buffer: [u16; 12] = [1; 12];
		static mut outbuffer: [u16; 12] = [0; 12];
		static mut send_buffer: [u16; 12] = [0; 12];

		let next = (*current + 1) % 3;
		
		c.resources.mytimer.clear_update_interrupt_flag();
		//writeln!(c.resources.tx, "Timer!");

		// receive

		let gpiob_in = unsafe { (*stm32::GPIOB::ptr()).idr.read().bits() };
		let in_bits: u16 = gpiob_in as u16 & 0xfff;
		let in_edge = in_bits ^ *in_bits_old;
		*in_bits_old = in_bits;
		
		let active_total = active[0] | active[1] | active[2];
		let start_of_transmission = !active_total & in_edge;

		active[next] |= start_of_transmission;

		let mut finished = 0;

		for i in 0..12 {
			let mask = 1 << i;
			
			if active[*current] & mask != 0 {
				/*let mut dings = 0;
				if in_bits & mask != 0 {
					dings = 1;
				}*/
				let mut dings = if in_bits & mask == 0 { 0 } else { 1 };

				buffer[i] = (buffer[i] << 1) | dings;

				if buffer[i] & (1 << 11) != 0 {
					outbuffer[i] = (buffer[i] >> 1) & 0xFF;
					buffer[i] = 1;
					finished |= mask;
				}
			}
		}

		active[*current] &= !finished;

		*current = next;

		//writeln!(c.resources.tx, "{}", o);
	}

};
