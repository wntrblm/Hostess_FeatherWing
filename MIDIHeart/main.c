#include "wtr_hid_keyboard_host_driver.h"
#include "wtr_midi_host_driver.h"
#include "wtr_queue.h"
#include "wtr_usb_host.h"
#include <atmel_start.h>
#include <hpl_delay.h>

enum HostessSpiCommands {
    HTS_SPI_START_REQUEST = 0x81,
    HTS_SPI_REQUEST_MIDI_EVENT = 0x83,
    HTS_SPI_REQUEST_KB_STRING = 0x84,
    HTS_SPI_REQUEST_KB_EVENT = 0x85,
    HTS_SPI_START_RESPONSE = 0x91,
    HTS_SPI_RESPONSE_EMPTY = 0x92,
    HTS_SPI_RESPONSE_MIDI_EVENT = 0x93,
    HTS_SPI_RESPONSE_KB_STRING = 0x94,
    HTS_SPI_RESPONSE_KB_EVENT = 0x95,
};

static uint8_t spi_out_buf[6];
static uint8_t spi_in_buf[1];

void spi_respond(struct wtr_queue *midi_queue, struct wtr_queue *keystring_queue, struct wtr_queue *key_event_queue) {
    int32_t recv_count;
    struct io_descriptor *io;
    spi_s_sync_get_io_descriptor(&SPI_0, &io);

    // Note: this blocks until 1 byte is read it seems.
    // Interrupts still happen, but maybe i should switch to async spi slave
    recv_count = io_read(io, spi_in_buf, 1);
    if (recv_count == 0)
        return;

    // First byte *must* be start request.
    if (spi_in_buf[0] != HTS_SPI_START_REQUEST)
        return;

    // read the next byte, it should be the request type.
    recv_count = io_read(io, spi_in_buf, 1);
    if (recv_count == 0)
        return;
    uint8_t request = spi_in_buf[0];

    switch (request) {
    case HTS_SPI_REQUEST_MIDI_EVENT:
        // Start response start byte and type
        spi_out_buf[0] = HTS_SPI_START_RESPONSE;

        if (wtr_queue_is_empty(midi_queue)) {
            spi_out_buf[1] = HTS_SPI_RESPONSE_EMPTY;
            io_write(io, spi_out_buf, 2);
        } else {
            spi_out_buf[1] = HTS_SPI_RESPONSE_MIDI_EVENT;
            // Pop the event on the queue to the SPI out buffer,
            // but leave two bytes at the first from the
            // response start and response type.
            wtr_queue_pop(midi_queue, spi_out_buf + 2);
            io_write(io, spi_out_buf, 6);
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
        }
        break;

    default:
        printf("Unknown SPI request: 0x%02x\r\n", request);
        break;
    }
}

int main(void) {
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

    wtr_usb_host_init(&USB_0_inst);
    // wtr_usb_midi_host_init();
    wtr_usb_hid_keyboard_init();
    struct wtr_queue *midi_in_queue = wtr_usb_midi_get_in_queue();
    struct wtr_queue *keystring_queue = wtr_usb_hid_keyboard_get_keystring_queue();
    struct wtr_queue *key_event_queue = wtr_usb_hid_keyboard_get_event_queue();

    spi_s_sync_enable(&SPI_0);

    gpio_set_pin_level(LED, 1);

    while (1) {
        spi_respond(midi_in_queue, keystring_queue, key_event_queue);
    }
}
