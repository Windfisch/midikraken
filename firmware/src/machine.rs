use stm32f1xx_hal::stm32;

pub unsafe fn reset_mcu() -> ! {
	(*stm32::SCB::ptr()).aircr.write(0x05FA0004);
	loop {}
}

pub unsafe fn reset_to_bootloader() -> ! {
	const BOOTKEY_ADDR: *mut u32 = 0x20003000 as *mut u32;
	core::ptr::write_volatile(BOOTKEY_ADDR, 0x157F32D4);
	reset_mcu();
}

