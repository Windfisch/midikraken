use stm32f1xx_hal::{
	dma,
	gpio::{gpiob, Alternate, PushPull},
	prelude::*,
	spi,
	stm32::SPI2,
};

pub type TxDmaSpi2 = dma::TxDma<
	spi::Spi<
		SPI2,
		spi::Spi2NoRemap,
		(
			gpiob::PB13<Alternate<PushPull>>,
			spi::NoMiso,
			gpiob::PB15<Alternate<PushPull>>,
		),
		u8,
	>,
	dma::dma1::C5,
>;

pub struct WriteDmaToWriteAdapter {
	write_dma: Option<TxDmaSpi2>,
}

unsafe impl Send for WriteDmaToWriteAdapter {}

impl WriteDmaToWriteAdapter {
	pub fn new(write_dma: TxDmaSpi2) -> WriteDmaToWriteAdapter {
		WriteDmaToWriteAdapter {
			write_dma: Some(write_dma),
		}
	}
}

impl embedded_hal::blocking::spi::Write<u8> for WriteDmaToWriteAdapter {
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
