/**
 * \file
 *
 * \brief SAM USB host HAL
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _HAL_USB_HOST_H_INCLUDED
#define _HAL_USB_HOST_H_INCLUDED

#include "hpl_usb_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup doc_driver_hal_usb_host
 *
 * @{
 */

/**
 * @brief      USB HCD Initialization
 *
 * @param      drv     Pointer to the HCD driver instance
 * @param[in]  hw      Pointer to hardware base
 * @param[in]  prvt    The private driver data (implement specific)
 *
 * @return     Operation result status
 * @retval     ERR_DENIED Hardware has been enabled
 * @retval     ERR_NONE Operation done successfully
 */
static inline int32_t usb_h_init(struct usb_h_desc *drv, void *hw, void *prvt)
{
	return _usb_h_init(drv, hw, prvt);
}

/**
 * @brief      USB HCD de-initialization
 *
 * @param      drv   The driver
 */
static inline void usb_h_deinit(struct usb_h_desc *drv)
{
	_usb_h_deinit(drv);
}

/**
 * @brief      USB HCD enable
 *
 * @param      drv   The driver
 */
static inline void usb_h_enable(struct usb_h_desc *drv)
{
	_usb_h_enable(drv);
}

/**
 * @brief      USB HCD disable
 *
 * @param      drv   The driver
 */
static inline void usb_h_disable(struct usb_h_desc *drv)
{
	_usb_h_disable(drv);
}

/**
 * @brief      Register callbacks for USB HCD
 *
 * @param      drv   The driver
 * @param[in]  type  The callback type
 * @param[in]  cb    The callback function entry
 *
 * @return     Operation result status
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_NONE Operation done successfully
 */
static inline int32_t usb_h_register_callback(struct usb_h_desc *drv, enum usb_h_cb_type type, FUNC_PTR cb)
{
	return _usb_h_register_callback(drv, type, cb);
}

/**
 * @brief      Return current frame number
 *
 * @param      drv   The driver
 *
 * @return     Current frame number
 */
static inline uint16_t usb_h_get_frame_n(struct usb_h_desc *drv)
{
	return _usb_h_get_frame_n(drv);
}

/**
 * @brief      Return current micro frame number
 *
 * @param      drv   The driver
 *
 * @return     Current micro frame number
 */
static inline uint8_t usb_h_get_microframe_n(struct usb_h_desc *drv)
{
	return _usb_h_get_microframe_n(drv);
}

/**
 * @brief Suspend the USB bus
 *
 * @param drv The driver
 */
static inline void usb_h_suspend(struct usb_h_desc *drv)
{
	_usb_h_suspend(drv);
}

/**
 * @brief Resume the USB bus
 *
 * @param drv The driver
 */
static inline void usb_h_resume(struct usb_h_desc *drv)
{
	_usb_h_resume(drv);
}

/* Root hub related APIs */

/**
 * \brief Reset the root hub port
 *
 * \param[in,out] drv  Pointer to the USB HCD driver
 * \param[in]     port Root hub port, ignored if there is only one port
 */
static inline void usb_h_rh_reset(struct usb_h_desc *drv, uint8_t port)
{
	_usb_h_rh_reset(drv, port);
}

/**
 * \brief Suspend the root hub port
 *
 * \param[in,out] drv  Pointer to the USB HCD driver
 * \param[in]     port Root hub port, ignored if there is only one port
 */
static inline void usb_h_rh_suspend(struct usb_h_desc *drv, uint8_t port)
{
	_usb_h_rh_suspend(drv, port);
}

/**
 * \brief Resume the root hub port
 *
 * \param[in,out] drv  Pointer to the USB HCD driver
 * \param[in]     port Root hub port, ignored if there is only one port
 */
static inline void usb_h_rh_resume(struct usb_h_desc *drv, uint8_t port)
{
	_usb_h_rh_resume(drv, port);
}

/**
 * \brief Root hub or port feature status check
 *
 * Check USB Spec. for hub status and feature selectors.
 *
 * \param[in]  drv  Pointer to the USB HCD driver
 * \param[in]  port Set to 0 to get hub status, otherwise to get port status
 * \param[in]  ftr  Feature selector
 *                  (0: connection, 2: suspend, 4: reset, 9: LS, 10: HS)
 *
 * \return     \c true if the status bit is 1
 */
static inline bool usb_h_rh_check_status(struct usb_h_desc *drv, uint8_t port, uint8_t ftr)
{
	return _usb_h_rh_check_status(drv, port, ftr);
}

/* Pipe transfer functions */

/**
 * @brief      Allocate a pipe for the USB host communication
 *
 * @param      drv           The USB HCD driver
 * @param[in]  dev           The device address
 * @param[in]  ep            The endpoint address
 * @param[in]  max_pkt_size  The endpoint maximum packet size
 * @param[in]  attr          The endpoint attribute
 * @param[in]  interval      The endpoint interval
 *                           (bInterval of USB Endpoint Descriptor)
 * @param[in]  speed         The transfer speed of the endpoint
 * @param[in]  minimum_rsc   Minimum resource usage, \sa usb_h_rsc_strategy
 *
 * @return     Pointer to the allocated pipe structure instance
 * @retval     NULL allocation fail
 */
static inline struct usb_h_pipe *usb_h_pipe_allocate(struct usb_h_desc *drv, uint8_t dev, uint8_t ep,
                                                     uint16_t max_pkt_size, uint8_t attr, uint8_t interval,
                                                     uint8_t speed, bool minimum_rsc)
{
	return _usb_h_pipe_allocate(drv, dev, ep, max_pkt_size, attr, interval, speed, minimum_rsc);
}

/**
 * @brief      Free an allocated pipe
 *
 * @param      pipe  The pipe
 *
 * @return     Operation result status
 * @retval     ERR_BUSY Pipe is busy, use \ref _usb_h_pipe_abort to abort
 * @retval     ERR_NONE Operation done successfully
 */
static inline int32_t usb_h_pipe_free(struct usb_h_pipe *pipe)
{
	return _usb_h_pipe_free(pipe);
}

/**
 * @brief      Modify parameters of an allocated control pipe
 *
 * @param      pipe          The pipe
 * @param[in]  dev           The device address
 * @param[in]  ep            The endpoint address
 * @param[in]  max_pkt_size  The maximum packet size, must be equal or
 *                           less than the allocated size
 * @param[in]  speed         The working speed
 *
 * @return     Operation result status
 * @retval     ERR_NOT_INITIALIZED The pipe is not allocated
 * @retval     ERR_BUSY The pipe is busy transferring
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_UNSUPPORTED_OP The pipe is not control pipe
 * @retval     ERR_NONE The operation is done successfully
 */
static inline int32_t usb_h_pipe_set_control_param(struct usb_h_pipe *pipe, uint8_t dev, uint8_t ep,
                                                   uint16_t max_pkt_size, uint8_t speed)
{
	return _usb_h_pipe_set_control_param(pipe, dev, ep, max_pkt_size, speed);
}

/**
 * @brief      Register transfer callback on a pipe
 *
 * @param      pipe  The pipe
 * @param[in]  cb    Transfer callback function
 *
 * @return     Operation result status
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_NONE Operation done successfully
 */
static inline int32_t usb_h_pipe_register_callback(struct usb_h_pipe *pipe, usb_h_pipe_cb_xfer_t cb)
{
	return _usb_h_pipe_register_callback(pipe, cb);
}

/**
 * @brief      Issue a control transfer (request) on a pipe
 *
 * \note When there is data stage, timeout between data packets is 500ms, the
 *       timeout between last data packet and the status packet is 50ms.
 *
 * @param         pipe    The pipe
 * @param[in]     setup   Pointer to the setup packet
 * @param[in,out] data    Pointer to the data buffer
 * @param[in]     length  The data length
 * @param[in]     timeout Timeout for whole request in ms
 *
 * @return     Operation result status
 * @retval     ERR_NOT_INITIALIZED Pipe is not allocated
 * @retval     ERR_BUSY Pipe is busy transferring
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_UNSUPPORTED_OP Pipe is not control pipe
 * @retval     ERR_NONE Operation done successfully
 */
static inline int32_t usb_h_control_xfer(struct usb_h_pipe *pipe, uint8_t *setup, uint8_t *data, uint16_t length,
                                         int16_t timeout)
{
	return _usb_h_control_xfer(pipe, setup, data, length, timeout);
}

/**
 * @brief      Issue a bulk/interrupt/iso transfer on a pipe
 *
 * @param         pipe     The pipe
 * @param[in,out] data     Pointer to the data buffer
 * @param[in]     length   The data length
 * @param[in]     auto_zlp Auto append ZLP for OUT
 *
 * @return     Operation result status
 * @retval     ERR_NOT_INITIALIZED The pipe is not allocated
 * @retval     ERR_BUSY The pipe is busy transferring
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_UNSUPPORTED_OP The pipe is control pipe
 * @retval     ERR_NONE The operation is done successfully
 */
static inline int32_t usb_h_bulk_int_iso_xfer(struct usb_h_pipe *pipe, uint8_t *data, uint32_t length, bool auto_zlp)
{
	return _usb_h_bulk_int_iso_xfer(pipe, data, length, auto_zlp);
}

/**
 * @brief      Issue a periodic high bandwidth output on a pipe
 *
 * @param         pipe             The pipe
 * @param[in,out] data             Pointer to the data buffer
 * @param[in]     length           The data length
 * @param[in]     trans_pkt_size   The transaction packet sizes in a micro frame,
 *                                 0 to use endpoint max packet size
 *
 * @return     Operation result status
 * @retval     ERR_NOT_INITIALIZED The pipe is not allocated
 * @retval     ERR_BUSY The pipe is busy transferring
 * @retval     ERR_INVALID_ARG Argument error
 * @retval     ERR_UNSUPPORTED_OP The pipe is not a high bandwidth periodic pipe, or
 *                                the DMA feature is not enabled, or
 *                                high bandwidth not enabled
 * @retval     ERR_NONE The operation is done successfully
 */
static inline int32_t usb_h_high_bw_out(struct usb_h_pipe *pipe, uint8_t *data, uint32_t length,
                                        uint16_t trans_pkt_size[3])
{
	return _usb_h_high_bw_out(pipe, data, length, trans_pkt_size);
}

/**
 * @brief      Check if the pipe is busy transferring
 *
 * @param      pipe  The pipe
 *
 * @return     \c true if pipe is busy
 */
static inline bool usb_h_pipe_is_busy(struct usb_h_pipe *pipe)
{
	return _usb_h_pipe_is_busy(pipe);
}

/**
 * @brief      Abort pending transfer on a pipe
 *
 * @param      pipe  The pipe
 *
 * @return     Operation result status
 */
static inline void usb_h_pipe_abort(struct usb_h_pipe *pipe)
{
	_usb_h_pipe_abort(pipe);
}

/**
 * @brief      Return version of the driver
 */
static inline uint32_t usb_h_get_version(void)
{
	return USB_H_VERSION;
}

	/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* _HAL_USB_HOST_H_INCLUDED */
