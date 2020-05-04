/**
 * \file
 *
 * \brief SPI Slave functionality declaration.
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

#ifndef _HAL_SPI_S_SYNC_H_INCLUDED
#define _HAL_SPI_S_SYNC_H_INCLUDED

#include "hpl_spi_s_sync.h"
#include "utils.h"
#include "hal_io.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup doc_driver_hal_spi_slave_sync
 *
 * @{
 */

/** \brief SPI Slave HAL driver struct for synchronous access
 */
struct spi_s_sync_descriptor {
	struct _spi_s_sync_hpl_interface *func;
	/** SPI device instance */
	struct _spi_sync_dev dev;
	/** I/O read/write */
	struct io_descriptor io;
	/** Break on SS desert detection. */
	uint8_t break_on_ss_det;
};

/** \brief Set the SPI HAL instance function pointer for HPL APIs.
 *
 *  Set SPI HAL instance function pointer for HPL APIs.
 *
 *  \param[in] spi Pointer to the HAL SPI instance.
 *  \param[in] func Pointer to the HPL api structure.
 *
 */
void spi_s_sync_set_func_ptr(struct spi_s_sync_descriptor *spi, void *const func);

/** \brief Initialize the SPI Slave HAL instance and hardware
 *
 *  Initialize SPI Slave HAL with polling mode.
 *
 *  \param[in, out] spi Pointer to the SPI Slave HAL instance.
 *  \param[in] hw Pointer to the hardware base.
 *
 *  \return Operation status.
 *  \retval 0 Success.
 *  \retval <0 Error.
 */
int32_t spi_s_sync_init(struct spi_s_sync_descriptor *spi, void *const hw);

/** \brief Deinitialize the SPI HAL instance
 *
 *  Disable and reset SPI, de-init software.
 *
 *  \param[in,out] spi Pointer to the SPI Slave HAL instance.
 */
void spi_s_sync_deinit(struct spi_s_sync_descriptor *spi);

/** \brief Enable SPI
 *
 *  \param[in,out] spi Pointer to the SPI Slave HAL instance.
 */
void spi_s_sync_enable(struct spi_s_sync_descriptor *spi);

/** \brief Disable SPI
 *
 *  \param[in,out] spi Pointer to the SPI Slave HAL instance.
 */
void spi_s_sync_disable(struct spi_s_sync_descriptor *spi);

/** \brief Set SPI mode
 *
 *  Set the SPI transfer mode (\ref spi_transfer_mode_t),
 *  which controls the clock polarity and clock phase:
 *  - Mode 0: leading edge is rising edge, data sample on leading edge.
 *  - Mode 1: leading edge is rising edge, data sample on trailing edge.
 *  - Mode 2: leading edge is falling edge, data sample on leading edge.
 *  - Mode 3: leading edge is falling edge, data sample on trailing edge.
 *  Note that SPI must be disabled to change mode.
 *
 *  \param[in,out] spi Pointer to the HAL SPI instance.
 *  \param[in] mode The mode (\ref spi_transfer_mode_t).
 *
 *  \return Operation status.
 *  \retval 0 Success.
 *  \retval <0 Error.
 */
int32_t spi_s_sync_set_mode(struct spi_s_sync_descriptor *spi, const enum spi_transfer_mode mode);

/** \brief Set SPI transfer character size in number of bits
 *
 *  The character size (\ref spi_char_size_t) influence the way the data is
 *  sent/received.
 *  For char size <= 8-bit, data is stored byte by byte.
 *  For char size between 9-bit ~ 16-bit, data is stored in 2-byte length.
 *  Note that the default and recommended char size is 8-bit since it's
 *  supported by all system.
 *  Note that the SPI must be disabled to change character size. Also it affects
 *  buffer accessing, the ring buffer should be flushed before changing it to
 *  avoid conflicts.
 *
 *  \param[in,out] spi Pointer to the HAL SPI instance.
 *  \param[in] char_size The char size (~32, recommended 8).
 *
 *  \return Operation status.
 *  \retval 0 Success.
 *  \retval <0 Error.
 */
int32_t spi_s_sync_set_char_size(struct spi_s_sync_descriptor *spi, const enum spi_char_size char_size);

/** \brief Set SPI transfer data order
 *
 *  Note that the SPI must be disabled to change data order.
 *
 *  \param[in,out] spi Pointer to the HAL SPI instance.
 *  \param[in] dord The data order: send LSB/MSB first.
 *
 *  \return Operation status.
 *  \retval 0 Success.
 *  \retval <0 Error.
 */
int32_t spi_s_sync_set_data_order(struct spi_s_sync_descriptor *spi, const enum spi_data_order dord);

/** \brief Enable/disable break on SS desert detection
 *
 *  \param[in,out] spi Pointer to the HAL SPI instance.
 *  \param[in] enable Set to \c true to break R/W loop on SS desert.
 */
void spi_s_sync_break_on_ss_detect(struct spi_s_sync_descriptor *spi, const bool enable);

/** \brief Write/read at the same time
 *
 *  \param[in,out] spi Pointer to the HAL SPI instance.
 *  \param[in] xfer Pointer to the transfer information (\ref spi_xfer).
 *
 *  \return Operation result.
 *  \retval <0 Error.
 *  \retval >=0 Number of characters transferred.
 */
int32_t spi_s_sync_transfer(struct spi_s_sync_descriptor *spi, const struct spi_xfer *xfer);

/**
 * \brief Return I/O descriptor for this SPI instance
 *
 * This function will return an I/O instance for this SPI driver instance
 *
 * \param[in] spi An SPI slave descriptor, which is used to communicate through
 *                SPI
 * \param[in] io A pointer to an I/O descriptor pointer type
 *
 * \return Error code.
 * \retval 0 No error detected
 * \retval <0 Error code
 */
int32_t spi_s_sync_get_io_descriptor(struct spi_s_sync_descriptor *spi, struct io_descriptor **io);

/** \brief Retrieve the current driver version
 *
 *  \return Current driver version.
 */
uint32_t spi_s_sync_get_version(void);

/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* _HAL_SPI_S_SYNC_H_INCLUDED */
