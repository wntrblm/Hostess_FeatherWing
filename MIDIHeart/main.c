#include <atmel_start.h>
#include <hpl_delay.h>

#include "wtr_hid_keyboard_host_driver.h"
#include "wtr_midi_host_driver.h"
#include "wtr_queue.h"
#include "wtr_usb_host.h"
#include "hostess_status_leds.h"


// Milliseconds
#define LED_PULSE_DURATION 30


enum hostess_spi_command {
    HTS_SPI_START_REQUEST = 0x81,
    HTS_SPI_REQUEST_READ_MIDI_EVENT = 0x83,
    HTS_SPI_REQUEST_WRITE_MIDI_EVENT = 0x84,
    HTS_SPI_REQUEST_KB_STRING = 0x85,
    HTS_SPI_REQUEST_KB_EVENT = 0x86,
    HTS_SPI_START_RESPONSE = 0xA1,
    HTS_SPI_RESPONSE_EMPTY = 0xA2,
    HTS_SPI_RESPONSE_READ_MIDI_EVENT = 0xA3,
    HTS_SPI_RESPONSE_WRITE_MIDI_EVENT = 0xA4,
    HTS_SPI_RESPONSE_KB_STRING = 0xA5,
    HTS_SPI_RESPONSE_KB_EVENT = 0xA6,
};

enum hostess_spi_state {
	HOSTESS_SPI_STATE_IDLE,
	HOSTESS_SPI_STATE_COMMAND_START,
	HOSTESS_SPI_STATE_COMMAND_DATA,
};

struct wtr_queue *midi_in_queue;
struct wtr_queue *midi_out_queue;
struct wtr_queue *keystring_queue;
struct wtr_queue *key_event_queue;
static enum hostess_spi_state spi_state = HOSTESS_SPI_STATE_IDLE;
static uint8_t spi_out_buf[16];
static uint8_t spi_in_buf[16];
static uint8_t spi_in_buf_head = 0;
static uint8_t command_data_bytes_count = 0;


void spi_complete_callback(const struct spi_s_async_descriptor *const spi_desc) {
    //no-op for now, but might need it.
}


void process_command(enum hostess_spi_command command, struct io_descriptor *io) {
    switch (command) {
        case HTS_SPI_REQUEST_READ_MIDI_EVENT:
            // Start response start byte and type
            spi_out_buf[0] = HTS_SPI_START_RESPONSE;

            if (wtr_queue_is_empty(midi_in_queue)) {
                spi_out_buf[1] = HTS_SPI_RESPONSE_EMPTY;
                io_write(io, spi_out_buf, 2);
            } else {
                spi_out_buf[1] = HTS_SPI_RESPONSE_READ_MIDI_EVENT;
                // Pop the event on the queue to the SPI out buffer,
                // but leave two bytes at the first from the
                // response start and response type.
                wtr_queue_pop(midi_in_queue, spi_out_buf + 2);
                io_write(io, spi_out_buf, 6);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        case HTS_SPI_REQUEST_WRITE_MIDI_EVENT:
            // If we just got the command, wait for 4 data bytes.
            if(spi_state == HOSTESS_SPI_STATE_COMMAND_START) {
                command_data_bytes_count = 4;
				spi_state = HOSTESS_SPI_STATE_COMMAND_DATA;
            }
            else if (spi_state == HOSTESS_SPI_STATE_COMMAND_DATA) {
                // We got a full event, write it to the MIDI out queue
                uint8_t *midi_event = spi_in_buf + 1;
                wtr_queue_push(midi_out_queue, midi_event);

                // And send along the ack response
                spi_out_buf[0] = HTS_SPI_START_RESPONSE;
                spi_out_buf[1] = HTS_SPI_RESPONSE_WRITE_MIDI_EVENT;
                io_write(io, spi_out_buf, 2);
				hostess_pulse_led(HTS_STATUS_LED_WRITE, LED_PULSE_DURATION);
            }
            break;

        case HTS_SPI_REQUEST_KB_STRING:
            // Start response start byte and type
            spi_out_buf[0] = HTS_SPI_START_RESPONSE;

            if (wtr_queue_is_empty(keystring_queue)) {
                spi_out_buf[1] = HTS_SPI_RESPONSE_EMPTY;
                io_write(io, spi_out_buf, 2);
            } else {
                spi_out_buf[1] = HTS_SPI_RESPONSE_KB_STRING;
                wtr_queue_pop(keystring_queue, spi_out_buf + 2);
                io_write(io, spi_out_buf, 3);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        case HTS_SPI_REQUEST_KB_EVENT:
            // Start response start byte and type
            spi_out_buf[0] = HTS_SPI_START_RESPONSE;

            if (wtr_queue_is_empty(key_event_queue)) {
                spi_out_buf[1] = HTS_SPI_RESPONSE_EMPTY;
                io_write(io, spi_out_buf, 2);
            } else {
                spi_out_buf[1] = HTS_SPI_RESPONSE_KB_EVENT;
                wtr_queue_pop(key_event_queue, spi_out_buf + 2);
                io_write(io, spi_out_buf, 5);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        default:
            printf("Unknown SPI request: 0x%02x\r\n", command);
            break;
    }
};


void spi_rx_callback(const struct spi_s_async_descriptor *const spi_desc) {
    int32_t recv_count;
    struct io_descriptor *io;
    spi_s_async_get_io_descriptor(&SPI_0, &io);

    recv_count = io_read(io, spi_in_buf + spi_in_buf_head, 1);

    if (recv_count == 0) return;
		
	switch(spi_state) {
		case HOSTESS_SPI_STATE_IDLE:
			if(spi_in_buf[0] == HTS_SPI_START_REQUEST) {
				spi_state = HOSTESS_SPI_STATE_COMMAND_START;
			}
			break;
			
		case HOSTESS_SPI_STATE_COMMAND_START:
			//printf("Command received: %x\r\n", spi_in_buf[0]);
			process_command(spi_in_buf[0], io);
            
			// If the command has data, the move our buf write head ahead one
			// so we can keep the command in buf[0] and add the data after.
            if(spi_state == HOSTESS_SPI_STATE_COMMAND_DATA) {
                spi_in_buf_head = 1;
            }
			// If there's no data for the command, set the state back to IDLE.
			else {
				spi_state = HOSTESS_SPI_STATE_IDLE;
			}
			break;

		case HOSTESS_SPI_STATE_COMMAND_DATA:
            spi_in_buf_head++;
            command_data_bytes_count--;

            if(command_data_bytes_count == 0) {
                process_command(spi_in_buf[0], io);
                spi_state = HOSTESS_SPI_STATE_IDLE;
				spi_in_buf_head = 0;
            }
			break;

		default:
			break; 
	}
	
	return;
}

int main(void) {
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

    // Start timers

    // Start the LED driver.
    hostess_leds_init(&TIMER_0);
    timer_start(&TIMER_0);

    // Start the USB host
    wtr_usb_host_init(&USB_0_inst);

    // Enable USB Host Drivers.
    wtr_usb_midi_host_init();
    //wtr_usb_hid_keyboard_init();

    // Grabs queues for host drivers.
    // TODO: move elsewhere?
    midi_in_queue = wtr_usb_midi_get_in_queue();
    midi_out_queue = wtr_usb_midi_get_out_queue();
    keystring_queue = wtr_usb_hid_keyboard_get_keystring_queue();
    key_event_queue = wtr_usb_hid_keyboard_get_event_queue();

    // Start listening for SPI.
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_RX, (FUNC_PTR)spi_rx_callback);
	spi_s_async_register_callback(&SPI_0, SPI_S_CB_TX, (FUNC_PTR)spi_complete_callback);
    spi_s_async_enable(&SPI_0);

	// Enable VBUS power. Should probably be moved somewhere else?
	gpio_set_pin_level(VUSB_EN_PIN, 1);

    while (1) {
		// Toggle the connected LED state.
        // TODO: Move this to a callback.
		if (wtr_usb_host_is_device_connected()) {
	        hostess_set_led(HTS_STATUS_LED_CONNECTION, true);
		} else {
	        hostess_set_led(HTS_STATUS_LED_CONNECTION, false);
		}
    }
}
