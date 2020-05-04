/**
 * \file
 *
 * \brief I/O SPI related functionality implementation.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "hal_atomic.h"
#include "hal_spi_s_sync.h"

#include <utils_assert.h>
#include <utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Driver version
 */
#define SPI_S_SYNC_DRIVER_VERSION 0x00000001u

/** Enable TX in transfer. */
#define SPI_XFER_TX_EN (1u << 0)
/** Enable RX in transfer. */
#define SPI_XFER_RX_EN (1u << 1)

/**
 *  \brief Transfer data
 *  \param[in, out] dev Pointer to the SPI device instance.
 *  \param[in] xfer Pointer to the transfer struct instance.
 *  \param[in] flags control options.
 *  \return Error or number of characters transferred.
 *  \retval <0 Error.
 *  \retval >=0 Number of characters transferred.
 */
static int32_t _spi_s_sync_xfer(struct spi_s_sync_descriptor *spi, const struct spi_xfer *xfer, const uint8_t flags)
{
	uint32_t txcnt, rxcnt;
	union {
		uint16_t u16;
		uint8_t  u8[4];
	} tmp;
	uint32_t n_bytes;

	ASSERT(spi && xfer);

	if (xfer->size == 0) {
		return 0;
	}

	n_bytes = xfer->size;
	if (spi->dev.char_size > 1) {
		n_bytes <<= 1;
	}

	tmp.u16 = 0;
	for (txcnt = 0, rxcnt = 0; txcnt < n_bytes && rxcnt < n_bytes;) {
		if (_spi_s_sync_is_error(&spi->dev)) {
			return ERR_IO;
		}
		if ((flags & SPI_XFER_TX_EN) && _spi_s_sync_is_tx_ready(&spi->dev)) {
			tmp.u8[0] = xfer->txbuf[txcnt++];
			if (spi->dev.char_size > 1) {
				tmp.u8[1] = xfer->txbuf[txcnt++];
			}
			_spi_s_sync_write_one(&spi->dev, tmp.u16);
		}
		if ((flags & SPI_XFER_RX_EN) && _spi_s_sync_is_rx_ready(&spi->dev)) {
			tmp.u16 = _spi_s_sync_read_one(&spi->dev);

			if (xfer->rxbuf) {
				xfer->rxbuf[rxcnt++] = tmp.u8[0];
				if (spi->dev.char_size > 1) {
					xfer->rxbuf[rxcnt++] = tmp.u8[1];
				}
			}
		}
		if (spi->break_on_ss_det && _spi_s_sync_is_ss_deactivated(&spi->dev)) {
			break;
		}
	}

	if (spi->dev.char_size <= 1) {
		return (flags & SPI_XFER_RX_EN) ? rxcnt : txcnt;
	}
	return ((flags & SPI_XFER_RX_EN) ? rxcnt : txcnt) >> 1;
}

/** \brief Do SPI data write
 *
 *  Register background buffer to transmit data.
 *
 *  It never blocks and return quickly, user check status or set callback to
 *  know when data is sent.
 *
 *  \param[in] io Pointer to the I/O descriptor.
 *  \param[in] buf Pointer to the buffer to store data to write.
 *  \param[in] size Size of the data in number of characters.
 *  \return Operation status.
 *  \retval 0 Success.
 *  \retval -1 Busy, transfer in progress.
 *  \retval -3 Parameter error.
 */
static int32_t _spi_s_sync_io_write(struct io_descriptor *const io, const uint8_t *const buf, const uint16_t size)
{
	struct spi_s_sync_descriptor *spi;
	struct spi_xfer               xfer;

	ASSERT(io);

	spi = CONTAINER_OF(io, struct spi_s_sync_descriptor, io);

	xfer.txbuf = (uint8_t *)buf;
	xfer.size  = size;
	return _spi_s_sync_xfer(spi, &xfer, SPI_XFER_TX_EN);
}

/** \brief Do SPI read from ring-buffer (asynchronously)
 *
 *  Read available characters from ring-buffer and return number of characters
 *  read.
 *
 *  It never blocks and return quickly, user check status or set callback to
 *  know when data is ready.
 *
 *  \param[in] io Pointer to the I/O descriptor.
 *  \param[out] buf Pointer to the buffer to store read data,
                NULL to discard data.
 *  \param[in] size Size of the data in number of characters.
 *  \return Read result.
 *  \retval n Number of characters read.
 *  \retval <0 Error.
 */
static int32_t _spi_s_sync_io_read(struct io_descriptor *const io, uint8_t *const buf, const uint16_t size)
{
	struct spi_s_sync_descriptor *spi;
	struct spi_xfer               xfer;

	ASSERT(io);

	spi = CONTAINER_OF(io, struct spi_s_sync_descriptor, io);

	xfer.rxbuf = (uint8_t *)buf;
	xfer.size  = size;
	return _spi_s_sync_xfer(spi, &xfer, SPI_XFER_RX_EN);
}

/**
 *  \brief Initialize the SPI HAL instance function pointer for HPL APIs.
 */
void spi_s_sync_set_func_ptr(struct spi_s_sync_descriptor *spi, void *const func)
{
	ASSERT(spi);
	spi->func = (struct _spi_s_sync_hpl_interface *)func;
}

int32_t spi_s_sync_init(struct spi_s_sync_descriptor *spi, void *const hw)
{
	int32_t rc;
	ASSERT(spi && hw);
	rc = _spi_s_sync_init(&spi->dev, hw);

	if (rc < 0) {
		return rc;
	}

	spi->io.read  = _spi_s_sync_io_read;
	spi->io.write = _spi_s_sync_io_write;

	return ERR_NONE;
}

void spi_s_sync_deinit(struct spi_s_sync_descriptor *spi)
{
	ASSERT(spi);
	_spi_s_sync_deinit(&spi->dev);
}

void spi_s_sync_enable(struct spi_s_sync_descriptor *spi)
{
	ASSERT(spi);
	_spi_s_sync_enable(&spi->dev);
}

void spi_s_sync_disable(struct spi_s_sync_descriptor *spi)
{
	ASSERT(spi);
	_spi_s_sync_disable(&spi->dev);
}

int32_t spi_s_sync_set_mode(struct spi_s_sync_descriptor *spi, const enum spi_transfer_mode mode)
{
	ASSERT(spi);
	return _spi_s_sync_set_mode(&spi->dev, mode);
}

int32_t spi_s_sync_set_char_size(struct spi_s_sync_descriptor *spi, const enum spi_char_size char_size)
{
	ASSERT(spi);
	return _spi_s_sync_set_char_size(&spi->dev, char_size);
}

int32_t spi_s_sync_set_data_order(struct spi_s_sync_descriptor *spi, const enum spi_data_order dord)
{
	ASSERT(spi);
	return _spi_s_sync_set_data_order(&spi->dev, dord);
}

void spi_s_sync_break_on_ss_detect(struct spi_s_sync_descriptor *spi, const bool enable)
{
	ASSERT(spi);

	spi->break_on_ss_det = enable;
}

int32_t spi_s_sync_transfer(struct spi_s_sync_descriptor *spi, const struct spi_xfer *xfer)
{
	return _spi_s_sync_xfer(spi, xfer, SPI_XFER_RX_EN | SPI_XFER_TX_EN);
}

int32_t spi_s_sync_get_io_descriptor(struct spi_s_sync_descriptor *spi, struct io_descriptor **io)
{
	ASSERT(spi && io);
	*io = &spi->io;
	return ERR_NONE;
}

uint32_t spi_s_sync_get_version(void)
{
	return SPI_S_SYNC_DRIVER_VERSION;
}

#ifdef __cplusplus
}
#endif
