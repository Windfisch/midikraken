#include <stdint.h>

#define N_UARTS 12
#define true 1
#define false 0

#define mymin(a,b) ((a)>(b) ? (b) : (a))

const uint16_t RECV_BIT = 1 << 10; // we need 11 bits for marker (10) + start (9) + 8x data (8-1) + stop (0) (marker is not actually transmitted over the line)
const uint16_t UART_SEND_IDLE = 1;

static volatile uint16_t send_buffers[N_UARTS];
static volatile uint16_t recv_buffers[N_UARTS];

static uint16_t in_bits_old = 0;

static uint16_t recv_active[3] = {0,0,0};
static int phase = 0;

static uint16_t send_workbuf[N_UARTS];
static uint16_t recv_workbuf[N_UARTS];
static uint16_t out_bits_buf = 0xFFFF;

//			send_buffers: make_array(UART_SEND_IDLE),
//			recv_buffers: make_array(0),

//				recv_workbuf: make_array(RECV_BIT),
//				send_workbuf: make_array(0),

int swu_clear_to_send(int index) {
	return send_buffers[index] == UART_SEND_IDLE;
}

void swu_send_byte(int index, uint8_t data) {
	send_buffers[index] = (data << 1) | (1<<9);
}

uint16_t swu_recv_byte(int index) {
	uint16_t data = recv_buffers[index];
	recv_buffers[index] = 0;

	if (data != 0)
		return (data >> 2) & 0xFF;
	else
		return 0xFFFF;
}

uint32_t swu_out_bits() {
	if (phase == 0) {
		uint16_t temp = out_bits_buf;
		out_bits_buf = 0;
		return temp;
	}
	else {
		return 0xFFFFFFFF;
	}
}

int swu_setup_benchmark(int benchmark_phase) {
	if (phase == benchmark_phase) {
		recv_active[phase] = 0xFFFF;
		for (int i=0; i<N_UARTS; i++) {
			recv_workbuf[i] = 2;
			send_buffers[i] = (0xFF << 1) | (1<<9);
			send_workbuf[i] = 0;
		}
		return true;
	}
	else {
		return false;
	}
}


uint16_t swu_process(uint16_t in_bits) {
	int next_phase = (phase + 1) % 3;

	// handle the received bits

	uint16_t falling_edge = ~in_bits & in_bits_old;
	in_bits_old = in_bits;
	
	uint16_t active_total = recv_active[0] | recv_active[1] | recv_active[2];
	uint16_t start_of_transmission = ~active_total & falling_edge;
	recv_active[next_phase] |= start_of_transmission;

	uint16_t recv_finished = 0;
	uint16_t recv_active_tmp = recv_active[phase];
	for (int i=0; i<N_UARTS; i++) {
		uint16_t mask = 1 << i;
	
		if (recv_active_tmp & mask) { // this is the thirdclock where uart #i can read stable data?
			/*let mut recv_bit = 0;
			if in_bits & mask {
				recv_bit = RECV_BIT;
			}*/
			uint16_t recv_bit = (in_bits & mask) ? RECV_BIT : 0;

			recv_workbuf[i] = (recv_workbuf[i] >> 1) | recv_bit;

			if (recv_workbuf[i] & 1) { // we received 10 bits, i.e. the marker bit is now the LSB?
				recv_buffers[i] = recv_workbuf[i];
				recv_workbuf[i] = RECV_BIT;
				recv_finished |= mask;
			}
		}
	}
	recv_active[phase] &= ~recv_finished;

	// handle the bits to be sent; in three thirdclocks, we prepare *out_bits.
	int send_batchsize = (N_UARTS+2) / 3;
	int first = phase * send_batchsize;
	for (int i = first; i < mymin(first + send_batchsize, N_UARTS); i++) {
		uint16_t workbuf = send_workbuf[i];
		if (workbuf == 0) {
			workbuf = send_buffers[i];
			send_buffers[i] = UART_SEND_IDLE;
		}
		
		out_bits_buf |= (workbuf & 1) << i;
		send_workbuf[i] = workbuf >> 1;
	}

	phase = next_phase;
	return recv_finished;
}
