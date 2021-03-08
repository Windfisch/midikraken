#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;

//use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use panic_semihosting as _;
use cortex_m::asm::delay;
use stm32f1xx_hal::{prelude::*, stm32, gpio::*, serial, timer, spi};
use core::fmt::Write;
use stm32f1xx_hal::usb::UsbBus;

use stm32f1xx_hal::time::Hertz;
const SYSCLK : Hertz = Hertz(72_000_000);

#[app(device = stm32f1)]
const APP: () = {
	#[init]
	fn init(mut cx : init::Context) {
		//static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<MyUsbBus>> = None;

		let dp = stm32::Peripherals::take().unwrap();
		let p = &mut cx.core; //cortex_m::peripheral::Peripherals::take().unwrap();


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
		writeln!(tx, "usart initialized").ok();
		writeln!(tx, "========================================================").ok();
		writeln!(tx, "midikraken @ {}", env!("VERGEN_SHA")).ok();
		writeln!(tx, "      built on {}", env!("VERGEN_BUILD_TIMESTAMP")).ok();
		writeln!(tx, "========================================================\n").ok();
		
		// Configure USB
		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low().ok();
		delay(clocks.sysclk().0 / 100);
		
		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		/* *USB_BUS = Some( UsbBus::new(dp.USB, (usb_dm, usb_dp)) );
		let usb_bus = USB_BUS.as_ref().unwrap();

		let mut midi = usbd_midi::MidiClass::new(usb_bus);
		let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Fake company")
			.product("Serial port")
			.serial_number("TEST")
			.device_class(usbd_midi::USB_CLASS_NONE)
			.build();
		*/
		
		// Timer
		//let mytimer = timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_raw(4800, 65535);
		let mytimer = timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1);
		mytimer.start_count_down(Hertz(3*31250));

		// Periodic timer
		let mut env_timer = timer::Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(1000.hz());
		env_timer.listen(timer::Event::Update);

		// USB interrupt
		p.NVIC.enable(stm32::Interrupt::USB_LP_CAN_RX0);

		//cx.spawn.xmain(Command::Calibrate);
		//return init::LateResources { exti : dp.EXTI, tx, led, usb_dev, midi, mytimer, env_timer, test_env2, test_lfo1, test_lfo2};
	}

};
