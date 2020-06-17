#pragma once

#include <hpl_dma.h>

struct wtr_spi_dma_inst;

typedef void (*wtr_spi_dma_tx_cb)(struct wtr_spi_dma_inst*);
typedef void (*wtr_spi_dma_error_cb)(struct wtr_spi_dma_inst*);


struct wtr_spi_dma_inst {
    uint8_t dma_channel;
    Sercom *sercom;
    wtr_spi_dma_tx_cb tx_callback;
    wtr_spi_dma_error_cb error_callback;
};


void _wtr_spi_dma_tx_complete_callback(struct _dma_resource *dma) {
    if(dma->back == NULL) return;
    struct wtr_spi_dma_inst *inst = dma->back;

    if(inst->tx_callback != NULL) inst->tx_callback(inst);

    // Re-enable RX interrupts. TODO: Remove this?
    inst->sercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC;
}


void _wtr_spi_dma_error_callback(struct _dma_resource *dma) {
    if(dma->back == NULL) return;
    struct wtr_spi_dma_inst *inst = dma->back;
    if(inst->tx_callback != NULL) inst->error_callback(inst);
}


int32_t wtr_spi_dma_write(struct wtr_spi_dma_inst *inst, uint8_t *data, uint16_t length) {
    struct _dma_resource *dma;
    _dma_get_channel_resource(&dma, inst->dma_channel);
    dma->back = (void *)inst;
    dma->dma_cb.transfer_done = &_wtr_spi_dma_tx_complete_callback;
    dma->dma_cb.error = &_wtr_spi_dma_error_callback;
    _dma_set_irq_state(inst->dma_channel, DMA_TRANSFER_COMPLETE_CB, true);
    _dma_set_irq_state(inst->dma_channel, DMA_TRANSFER_ERROR_CB, true);
    _dma_set_source_address(inst->dma_channel, data);
    _dma_set_destination_address(inst->dma_channel, (void *)&inst->sercom->SPI.DATA);
    _dma_set_data_amount(inst->dma_channel, length);

    _dma_enable_transaction(inst->dma_channel, false);
    
    // Don't respond to RX interrupts until the DMA transfer is complete.
    inst->sercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENSET_RXC;

    return ERR_NONE;
}