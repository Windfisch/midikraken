use heapless::spsc::{Queue, Producer, Consumer};
use core::sync::atomic::AtomicU16;
use core::sync::atomic::Ordering::Relaxed;

pub struct MidiFilterQueue<const N: usize> {
	queue: Queue<[u8; 4], N>,
	filter: AtomicU16
}

impl<const N: usize> MidiFilterQueue<N> {
	pub const fn new() -> MidiFilterQueue<N> {
		MidiFilterQueue {
			queue: Queue::new(),
			filter: AtomicU16::new(0xFFFF)
		}
	}

	pub fn split(&mut self) -> (MidiFilterQueueProducer<N>, MidiFilterQueueConsumer<N>)
	{
		let (producer, consumer) = self.queue.split();

		let producer = MidiFilterQueueProducer { producer, filter: &self.filter };
		let consumer = MidiFilterQueueConsumer { consumer, filter: &self.filter };

		(producer, consumer)
	}
}

pub struct MidiFilterQueueConsumer<'a, const N: usize> {
	consumer: Consumer<'a, [u8; 4], N>,
	filter: &'a AtomicU16
}

pub struct MidiFilterQueueProducer<'a, const N: usize> {
	producer: Producer<'a, [u8; 4], N>,
	filter: &'a AtomicU16
}

impl<const N: usize> MidiFilterQueueProducer<'_, N> {
	pub fn enqueue(&mut self, value: [u8; 4]) -> Result<(), [u8; 4]> {
		let cable = value[0] >> 4;
		if self.filter.load(Relaxed) & (1 << cable) != 0 {
			self.producer.enqueue(value)?;
		}

		Ok(())
	}
}

impl<const N: usize> MidiFilterQueueConsumer<'_, N> {
	pub fn dequeue(&mut self) -> Option<[u8; 4]> {
		self.consumer.dequeue()
	}

	pub fn set_filter(&mut self, filter_mask: u16) {
		self.filter.store(filter_mask, Relaxed);
	}
}
