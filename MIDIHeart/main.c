#include <atmel_start.h>
#include <hpl_delay.h>
#include <hpl_dma.h>

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

volatile hal_atomic_t spi_rx_atomic;

// void spi_rx_callback(const struct spi_s_async_descriptor *const spi_desc) {
//     struct io_descriptor *io;

//     // Disable interrupts while responding to SPI requests. An
//     // interrupt during this could cause SPI data to drop,
//     // it also prevents contention on the event queues.
//     //atomic_enter_critical(&spi_rx_atomic);
//     spi_s_async_get_io_descriptor(&SPI_0, &io);

//     hostess_parse_byte_stream(io);

//     //atomic_leave_critical(&spi_rx_atomic);
// }


void dma_callback(struct _dma_resource *dma) {
    //printf("Done.");
    // Listen to RX interrupts again.
    SERCOM4->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC;
}

int32_t spi_dma_write(struct io_descriptor *io, uint8_t *data, uint16_t length) {

    struct _dma_resource *dma;
    _dma_get_channel_resource(&dma, 1);
    dma->dma_cb.transfer_done = &dma_callback;
    dma->dma_cb.error = &dma_callback;
    _dma_set_irq_state(1, DMA_TRANSFER_COMPLETE_CB, true);
    _dma_set_irq_state(1, DMA_TRANSFER_ERROR_CB, true);
    _dma_set_source_address(1, data);
    _dma_set_destination_address(1, (void *)&REG_SERCOM4_SPI_DATA);
    _dma_set_data_amount(1, length);

    _dma_enable_transaction(1, false);
    
    // Don't respond to RX interrupts until the DMA transfer is complete.
    SERCOM4->SPI.INTENCLR.reg = SERCOM_SPI_INTENSET_RXC;

    return ERR_NONE;
}

void spi_rx_callback(const struct spi_s_async_descriptor *const spi_desc) {
    struct io_descriptor *io;
    spi_s_async_get_io_descriptor(&SPI_0, &io);

    // Replace the write function with the DMA-enabled write function.
    io->write = &spi_dma_write;

    hostess_parse_byte_stream(io);
}



int main(void) {
    // Initializes MCU, drivers and middleware
    atmel_start_init();

    // Set interrupt priorities. Setting the SPI interrupt
    // a lower priority prevents it from starving the USB
    // task.
    NVIC_SetPriority(SERCOM4_IRQn, 0);
    NVIC_SetPriority(USB_IRQn, 1);
    NVIC_SetPriority(RTC_IRQn, 3);

    // Start the LED driver.
    hostess_leds_init(&TIMER_0);
    timer_start(&TIMER_0);

    // Start the USB host
    wtr_usb_host_set_connection_callback(&usb_connection_callback);
    wtr_usb_host_init(&USB_0_inst);

    // Enable USB Host Drivers.
    wtr_usb_midi_host_init();
    //wtr_usb_hid_keyboard_init();
    //wtr_ps4_driver_init();
    
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

    // Enable DMA for the SPI buffers.
    _dma_init();

    // Start listening for SPI.
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_RX, (FUNC_PTR)spi_rx_callback);
    spi_s_async_enable(&SPI_0);

    while (1) {
    }
}
