#![no_std]

pub mod software_uart;
use software_uart::*;
use software_uart::typenum::Unsigned;
pub type NumPortPairs = software_uart::typenum::U12;

use stm32f1xx_hal::{prelude::*, stm32, serial, timer, spi, dma, gpio::{Alternate, PushPull, Input, Output, Floating, PullUp, gpioa, gpiob, gpioc}};

pub struct DmaPair {
	pub transmit: [u8; 4],
	pub received: [u8; 4]
}
impl DmaPair {
	pub const fn zero() -> DmaPair { DmaPair { transmit: [0xFF; 4], received: [0; 4] } }
}

pub unsafe fn optimized_interrupt_handler(
	sw_uart_isr: &mut SoftwareUartIsr<'static, NumPortPairs>,

	spi_strobe_pin: &mut gpioc::PC14<Output<PushPull>>,

	dma_transfer: &mut Option<dma::Transfer<
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

	dma_buffer_ptr: *mut DmaPair
) -> (u16, u16) {
	// handle the SPI DMA
	let (_, spi_dma) = dma_transfer.take().unwrap().wait();
	spi_strobe_pin.set_low();

	let mut in_bits: u32;
	{
		// this is safe in and only in this scope, since the DMA transfers are currently halted
		let dma_buffer = unsafe { &mut *dma_buffer_ptr };

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
	
		if let Some(out_bits) = sw_uart_isr.out_bits() {
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

	spi_strobe_pin.set_high();
	*dma_transfer = Some(spi_dma.read_write(
		unsafe { &mut (*dma_buffer_ptr).received },
		unsafe { & (*dma_buffer_ptr).transmit }
	));

	sw_uart_isr.process((in_bits & 0xFFFF) as u16)
}
