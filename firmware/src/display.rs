use crate::dma_adapter;
use stm32f1xx_hal::gpio::{gpioa, Output, PushPull};

pub type Display = st7789::ST7789<
	display_interface_spi::SPIInterfaceNoCS<
		dma_adapter::WriteDmaToWriteAdapter,
		gpioa::PA2<Output<PushPull>>,
	>,
	gpioa::PA1<Output<PushPull>>,
>;
