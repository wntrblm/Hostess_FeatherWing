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


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	wtr_usb_host_init(&USB_0_inst);
	wtr_usb_midi_host_init();
	struct wtr_queue* in_queue = wtr_usb_midi_get_in_queue();

	/* Replace with your application code */
	while (1) {
		gpio_set_pin_level(LED, 1);
		print_midi_events(in_queue);
		//delay_ms(1000);
		//printf(".");
		gpio_set_pin_level(LED, 0);
		print_midi_events(in_queue);
		//delay_ms(1000);
	}
}
