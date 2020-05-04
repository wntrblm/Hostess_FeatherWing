#include <atmel_start.h>
#include <hpl_delay.h>
#include "wtr_usb_host.h"
#include "wtr_midi_host_driver.h"
#include "wtr_queue.h"



void print_midi_events(struct wtr_queue* midi_queue) {
	while(!wtr_queue_is_empty(midi_queue)) {
		uint8_t event[4];
		wtr_queue_pop(midi_queue, event);
		printf("%02x %02x %02x %02x\r\n", event[0], event[1], event[2], event[3]);
	}
}


static uint8_t spi_out_buf[5]; // One byte for the response start byte, 4 bytes for a midi event.
static uint8_t spi_in_buf[1];

void spi_respond(struct wtr_queue* midi_queue) {
	struct io_descriptor *io;
	spi_s_sync_get_io_descriptor(&SPI_0, &io);
	
	int32_t recv_count;
	do {
		recv_count = io_read(io, spi_in_buf, 1);
	} while (recv_count == 0);
	
	/* 42 = Get midi data */
	if(spi_in_buf[0] == 42) {
		
		if(wtr_queue_is_empty(midi_queue)) {
			memset(spi_out_buf, 0, 5);
		} else {
			wtr_queue_pop(midi_queue, spi_out_buf + 1);
		}
		spi_out_buf[0] = 43; // Response start byte
		io_write(io, spi_out_buf, 5);
	}
}


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	wtr_usb_host_init(&USB_0_inst);
	wtr_usb_midi_host_init();
	struct wtr_queue* midi_in_queue = wtr_usb_midi_get_in_queue();
	
	spi_s_sync_enable(&SPI_0);

	/* Replace with your application code */
	while (1) {
		gpio_set_pin_level(LED, 1);
		//print_midi_events(in_queue);
		//delay_ms(1000);
		//printf(".");
		gpio_set_pin_level(LED, 0);
		//print_midi_events(in_queue);
		//delay_ms(2000);
		spi_respond(midi_in_queue);
	}
}
