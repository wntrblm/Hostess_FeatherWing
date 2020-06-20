#include <atmel_start.h>
#include <hpl_delay.h>

#include "wtr_spi_dma.h"
#include "wtr_hid_keyboard_host_driver.h"
#include "wtr_midi_host_driver.h"
#include "wtr_ps4_host_driver.h"
#include "wtr_queue.h"
#include "wtr_usb_host.h"
#include "hostess_status_leds.h"
#include "hostess_command_processor.h"


/* USB wiring to control the current-limit switch and connection LED */

static hal_atomic_t vbus_fault_atomic;

void vbus_control(bool enable) {
    gpio_set_pin_level(VUSB_EN_PIN, enable);
    if(!enable) {
        hostess_flash_led(HTS_STATUS_LED_CONNECTION);
    }
}

void vbus_fault_interrupt() {
    atomic_enter_critical(&vbus_fault_atomic);
    vbus_control(false);
    atomic_leave_critical(&vbus_fault_atomic);
}

void usb_connection_callback(uint8_t port, bool state) {
    hostess_set_led(HTS_STATUS_LED_CONNECTION, state);
}


/* SPI wiring to connect SPI to the Hostess command parser. */
/*
Some notes about this code: instead of using the standard Atmel
SPI write logic, this uses wtr_spi_dma_write to configure a
DMA transaction. All bytes received from the host are ignored
until the DMA is complete.

I considered using DMA for the read side, too, but it's a lot
of work for little gain, considering most requests take 2 bytes
and they need to be parsed by this code anyway.
*/

struct wtr_spi_dma_inst dma_spi;

static int32_t spi_dma_io_write_adapter(struct io_descriptor *const io, const uint8_t *const buf, const uint16_t size) {
    return wtr_spi_dma_write(&dma_spi, (uint8_t *)buf, size);
}

void spi_rx_callback(const struct spi_s_async_descriptor *const spi_desc) {
    struct io_descriptor *io;
    spi_s_async_get_io_descriptor(&SPI_0, &io);

    // Replace the write function with the DMA-enabled write function.
    io->write = &spi_dma_io_write_adapter;

    hostess_parse_byte_stream(io);
}



int main(void) {
    // Initializes MCU, drivers and middleware
    atmel_start_init();

    // Set interrupt priorities. Setting the SPI interrupt
    // a lower priority prevents it from starving the USB
    // task.
    NVIC_SetPriority(USB_IRQn, 0);
    NVIC_SetPriority(SERCOM4_IRQn, 1);
    NVIC_SetPriority(RTC_IRQn, 3);

    // Start the LED driver.
    hostess_leds_init(&TIMER_0);
    timer_start(&TIMER_0);

    // Start the USB host
    wtr_usb_host_set_connection_callback(&usb_connection_callback);
    wtr_usb_host_init(&USB_0_inst);

    // Enable USB Host Drivers.
    wtr_usb_midi_init();
    wtr_usb_hid_keyboard_init();
    //wtr_ps4_controller_init();
    
    // Setup the vbus fault interrupt.
    ext_irq_register(VBUS_FAULT_PIN, &vbus_fault_interrupt);

    // Enable VBUS power.
    vbus_control(true);

    // Grab event queues from host drivers and wire them up to
    // the hostess command processor.
    hostess_set_queue(HOSTESS_QUEUE_MIDI_IN, wtr_usb_midi_get_in_queue());
    hostess_set_queue(HOSTESS_QUEUE_MIDI_OUT, wtr_usb_midi_get_out_queue());
    hostess_set_queue(HOSTESS_QUEUE_KB_EVENT, wtr_usb_hid_keyboard_get_event_queue());
    hostess_set_queue(HOSTESS_QUEUE_KB_STRING, wtr_usb_hid_keyboard_get_keystring_queue());

    // Configure SPI DMA
    _dma_init();
    dma_spi.dma_channel = 1;
    dma_spi.sercom = SERCOM4;

    // Start listening for SPI.
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_RX, (FUNC_PTR)spi_rx_callback);
    spi_s_async_enable(&SPI_0);

    while (1) {
    }
}
