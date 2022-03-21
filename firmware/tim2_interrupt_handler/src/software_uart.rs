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

use core::ptr::{read_volatile, write_volatile, addr_of_mut, addr_of};
use core::marker::PhantomData;

const RECV_BIT: u16 = 1 << 10; // we need 11 bits for marker (10) + start (9) + 8x data (8-1) + stop (0) (marker is not actually transmitted over the line)
const UART_SEND_IDLE: u16 = 1;

/** The software uart main struct. Holds the 1 byte send and receive buffers.
  *
  * Typically stored in a static variable and then split() into three resources.
  * The SoftwareUartRx and SoftwareUartTx resources are usually owned by a single
  * receiving / sending task, while the SoftwareUartIsr struct must be owned by a
  * timer interrupt handler which executes at 3x the desired baud rate.
  *
  * Usage example with RTIC:
  * ```
#[app(device = stm32f1xx_hal::pac)]
const APP: () = {
	struct Resources {
		mytimer: timer::CountDownTimer<stm32::TIM2>,
		sw_uart_rx: SoftwareUartRx<'static>,
		sw_uart_tx: SoftwareUartTx<'static>,
		sw_uart_backend: SoftwareUartIsr<'static>,
	}

	#[init(spawn=[benchmark_task, mainloop])]
	fn init(cx : init::Context) -> init::LateResources {
		static mut SOFTWARE_UART: Option<SoftwareUart> = None;
		*SOFTWARE_UART = Some(SoftwareUart::new());
		let (sw_uart_tx, sw_uart_rx, sw_uart_backend) = (*SOFTWARE_UART).as_mut().unwrap().split();
		let mut mytimer =
			timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(31250 * 3));
		mytimer.listen(timer::Event::Update);
		return init::LateResources { mytimer, sw_uart_tx, sw_uart_rx, sw_uart_backend };
	}

	#[task(binds = TIM2, spawn=[byte_received], resources = [mytimer, sw_uart_isr],  priority=100)]
	fn timer_interrupt(c: timer_interrupt::Context) {
		c.resources.mytimer.clear_update_interrupt_flag();

		// actual I/O is done at the very beginning to minimize jitter

		// input
		let gpiob_in = unsafe { (*stm32::GPIOB::ptr()).idr.read().bits() } as u16; // read the IO port
		let in_bits: u16 = (gpiob_in >> 5) & 0x000F;
		
		// output: *out_bits have been set in the last three thirdclocks.
		if let Some(out_bits) = c.resources.sw_uart_isr.out_bits() {
			const MASK_A: u32 = 0x87FF87FF;
			const MASK_C: u32 = 0xC000C000;
			let bits_a = (out_bits & 0x7FF) | ((out_bits & 0x800) << 4);
			let bits_c = (out_bits & 0x3000) << 2;
			let bsrr_a = (bits_a as u32 | ((!bits_a as u32) << 16)) & MASK_A;
			let bsrr_c = (bits_c as u32 | ((!bits_c as u32) << 16)) & MASK_C;

			unsafe { (*stm32::GPIOA::ptr()).bsrr.write(|w| w.bits(bsrr_a)); }; // we ensure to only access pins
			unsafe { (*stm32::GPIOC::ptr()).bsrr.write(|w| w.bits(bsrr_c)); }; // we own (using MASK_A / MASK_C)
		}

		if c.resources.sw_uart_isr.process(in_bits) != 0 {
			c.spawn.byte_received();
		}
	}
	// TODO FIXME: finish the example
  * ```
  */
pub struct SoftwareUart<const NUM_UARTS: usize> {
	send_buffers: [u16; NUM_UARTS],
	recv_buffers: [u16; NUM_UARTS],
}

/** The timer interrupt handler part of the software uart. */
pub struct SoftwareUartIsr<'a, const NUM_UARTS: usize> {
	in_bits_old: u16,
	recv_active: [u16; 3],
	phase: usize,

	recv_workbuf: [u16; NUM_UARTS],
	send_workbuf: [u16; NUM_UARTS],
	out_bits: u16,

	registers: *mut SoftwareUart<NUM_UARTS>,
	_marker: PhantomData<&'a ()>
}

/** The transmit half of the software uart's frontend. */
pub struct SoftwareUartTx<'a, const NUM_UARTS: usize> {
	registers: *mut SoftwareUart<NUM_UARTS>,
	_marker: PhantomData<&'a ()>
}

/** The receive half of the software uart's frontend. */
pub struct SoftwareUartRx<'a, const NUM_UARTS: usize> {
	registers: *mut SoftwareUart<NUM_UARTS>,
	_marker: PhantomData<&'a ()>
}

unsafe impl<'a, const NUM_UARTS: usize> Send for SoftwareUartIsr<'a, NUM_UARTS> {}
unsafe impl<'a, const NUM_UARTS: usize> Send for SoftwareUartRx<'a, NUM_UARTS> {}
unsafe impl<'a, const NUM_UARTS: usize> Send for SoftwareUartTx<'a, NUM_UARTS> {}

impl<const NUM_UARTS: usize> SoftwareUart<NUM_UARTS> {
	pub fn new() -> SoftwareUart<NUM_UARTS> {
		SoftwareUart {
			send_buffers: [UART_SEND_IDLE; NUM_UARTS],
			recv_buffers: [0; NUM_UARTS],
		}
	}

	pub fn split(&mut self) -> (SoftwareUartTx<NUM_UARTS>, SoftwareUartRx<NUM_UARTS>, SoftwareUartIsr<NUM_UARTS>) {
		return (
			SoftwareUartTx { registers: self, _marker: PhantomData },
			SoftwareUartRx { registers: self, _marker: PhantomData },
			SoftwareUartIsr {
				registers: self,
				in_bits_old: 0,
				recv_active: [0; 3],
				phase: 0,
				recv_workbuf: [RECV_BIT; NUM_UARTS],
				send_workbuf: [0; NUM_UARTS],
				out_bits: 0xFFFF,
				_marker: PhantomData
			}
		);
	}
}

impl<'a, const NUM_UARTS: usize> SoftwareUartTx<'a, NUM_UARTS> {
	/** Returns whether the specified UART is ready to accept another byte to be sent via `send_byte()`
		without overwriting the byte scheduled before. */
	pub fn clear_to_send(&self, index: usize) -> bool {
		unsafe {
			return read_volatile(addr_of!((*self.registers).send_buffers[index])) == UART_SEND_IDLE;
		}
	}

	/** Schedules `data` to be sent out via the specified UART as soon as the current transmission
		(if any) has finished. Should only be called if `clear_to_send` reports `true`.
		Note: Only one byte is buffered, so if `send_byte` is called although not `clear_to_send`,
		the previously buffered byte is lost. */
	pub fn send_byte(&mut self, index: usize, data: u8) {
		unsafe {
			write_volatile(addr_of_mut!((*self.registers).send_buffers[index]), ((data as u16) << 1) | (1 << 9));
		}
	}
}

impl<'a, const NUM_UARTS: usize> SoftwareUartRx<'a, NUM_UARTS> {
	/** Retrieves the last byte received on the specified UART, or `None`.
		Note: Only the last received byte is buffered, so if `recv_byte` is not
		called in time, data is lost. */
	pub fn recv_byte(&mut self, index: usize) -> Option<u8> {
		unsafe {
			let data = read_volatile(addr_of!((*self.registers).recv_buffers[index]));
			write_volatile(addr_of_mut!((*self.registers).recv_buffers[index]), 0);

			if data != 0 {
				return Some(((data >> 2) & 0xFF) as u8); // shift out the marker bit and the start bit. then AND out the stop bit
			}
			else {
				return None;
			}
		}
	}
}

impl<'a, const NUM_UARTS: usize> SoftwareUartIsr<'a, NUM_UARTS> {
	/** Returns the output bits in every phase 0, or None otherwise.
		Should be called before `process()` to minimize jitter. */
	pub fn out_bits(&mut self) -> Option<u16> {
		if self.phase == 0 {
			let temp = self.out_bits;
			self.out_bits = 0;
			return Some(temp);
		}
		else {
			return None;
		}
	}

	/** Sets the internal state to benchmarking mode if the current phase is `phase`.
	  * Return whether benchmarking was initiated.
	  * Note: garbage will be sent and received during benchmarking. */
	pub fn setup_benchmark(&mut self, benchmark_phase: i8) -> bool {
		if self.phase as i8 == benchmark_phase {
			self.recv_active[self.phase] = 0xFFFF;
			for i in 0..NUM_UARTS {
				self.recv_workbuf[i] = 2; // this will be the last bit received, causing additional work to happen
				unsafe { write_volatile(addr_of_mut!((*self.registers).send_buffers[i]), (0xFF << 1) | (1<<9)); }
				self.send_workbuf[i] = 0; // force an (expensive) reload to happen
			}
			return true;
		}
		else {
			return false;
		}
	}

	/** Handles the received bits and returns a bitset indicating which UARTs have finished
		receiving. Call `recv_byte` to retrieve the received bytes.
		Must be called in a timer interrupt running at 3x the baud rate. */
	pub fn process(&mut self, in_bits: u16) -> (u16, u16) {
		let next_phase = (self.phase + 1) % 3;

		// handle the received bits

		let falling_edge = !in_bits & self.in_bits_old;
		self.in_bits_old = in_bits;
		
		let active_total = self.recv_active[0] | self.recv_active[1] | self.recv_active[2];
		let start_of_transmission = !active_total & falling_edge;
		self.recv_active[next_phase] |= start_of_transmission;

		let mut recv_finished = 0;
		let recv_buffers = unsafe { addr_of_mut!((*self.registers).recv_buffers) };
		let recv_active = self.recv_active[self.phase];
		for i in 0..NUM_UARTS {
			let mask = 1 << i;
		
			if recv_active & mask != 0 { // this is the thirdclock where uart #i can read stable data?
				/*let mut recv_bit = 0;
				if in_bits & mask != 0 {
					recv_bit = RECV_BIT;
				}*/
				let recv_bit = if in_bits & mask == 0 { 0 } else { RECV_BIT };

				self.recv_workbuf[i] = (self.recv_workbuf[i] >> 1) | recv_bit;

				if self.recv_workbuf[i] & 1 != 0 { // we received 10 bits, i.e. the marker bit is now the LSB?
					unsafe { write_volatile(addr_of_mut!((*recv_buffers)[i]), self.recv_workbuf[i]); } // publish the received uart frame.
					self.recv_workbuf[i] = RECV_BIT;
					recv_finished |= mask;
				}
			}
		}
		self.recv_active[self.phase] &= !recv_finished;

		// handle the bits to be sent; in three thirdclocks, we prepare *out_bits.
		let mut sendbuf_consumed = 0;
		let send_batchsize: usize = (NUM_UARTS+2) / 3;
		let send_buffers = unsafe { addr_of_mut!((*self.registers).send_buffers) };
		let first = self.phase * send_batchsize;
		for i in first .. core::cmp::min(first + send_batchsize, NUM_UARTS) {
			let mut workbuf = self.send_workbuf[i];
			if workbuf == 0 {
				unsafe { // STM32 reads and writes u16s atomically
					workbuf = read_volatile(addr_of!((*send_buffers)[i]));
					write_volatile(addr_of_mut!((*send_buffers)[i]), UART_SEND_IDLE);
				}
				if workbuf != UART_SEND_IDLE {
					sendbuf_consumed |= 1 << i;
				}
			}
			
			self.out_bits |= (workbuf & 1) << i;
			self.send_workbuf[i] = workbuf >> 1;
		}

		self.phase = next_phase;
		return (recv_finished, sendbuf_consumed);
	}
}
