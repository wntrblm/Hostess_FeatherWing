/**
 * \file
 *
 * \brief USB Host Stack HID Mouse Function Definition.
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
 */

#ifndef _HIDHF_MOUSE_H_
#define _HIDHF_MOUSE_H_

#include "usbhf.h"
#include "usb_protocol_hid.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONF_HIDHF_MOUSE_RPT_SIZE
#define CONF_HIDHF_MOUSE_RPT_SIZE 4
#endif

#ifndef HIDHF_IN_PIPE_IDX
#define HIDHF_IN_PIPE_IDX 0
#endif

/** Access to the relative Scroll in HID mouse report
 *  \param rpt Pointer to report data of bytes
 */
#define hid_mouse_scroll(rpt) ((int8_t)rpt[3])

struct hidhf_mouse;

/** \brief Mouse callback types */
enum hidhf_mouse_cb_type {
	HIDHF_MOUSE_BTN_CB,  /**< Button state change callback */
	HIDHF_MOUSE_MOVE_CB, /**< Mouse move callback */
	HIDHF_MOUSE_CB_N
};

/** \brief Mouse button state change callback */
typedef void (*hidhf_mouse_btn_cb_t)(struct hidhf_mouse *func, uint8_t state);
/** \brief Mouse move callback */
typedef void (*hidhf_mouse_move_cb_t)(struct hidhf_mouse *func, int8_t x, int8_t y, int8_t scroll);

/** USB HID Host Mouse or input only Function Driver support */
struct hidhf_mouse {
	/** General data for USB Host function driver. */
	struct usbhf_driver func;
	/** Interface for the function */
	int8_t iface;
	/** Function is installed */
	volatile uint8_t is_enabled : 1;
	/** It's function for a boot device */
	uint8_t is_bootdevice : 1;
	/** Reserved bits */
	uint8_t reserved_bits : 6;
	/** Report buffer size */
	uint8_t report_size;
	/** Last button status */
	uint8_t btn_state;
	/** Pipes - IN */
	struct usb_h_pipe *pipe[1];
	/** Callback invoked when button state changes */
	hidhf_mouse_btn_cb_t btn_cb;
	/** Callback invoked when mouse moves */
	hidhf_mouse_move_cb_t move_cb;
	/** Input Report buffer */
	uint8_t report[CONF_HIDHF_MOUSE_RPT_SIZE];
};

/** Cast a pointer to USB HID Host Function Driver */
#define HIDHF_MOUSE_PTR(p) ((struct hidhf_mouse *)(p))

/** \brief Initialize the HID mouse function driver and attach it to core
 *  \param core Pointer to the USB host core instance
 *  \param func Pointer to the HID mouse function driver instance
 *  \return Operation result status
 */
int32_t hidhf_mouse_init(struct usbhc_driver *core, struct hidhf_mouse *func);

/** \brief Deinitialize the HID mouse function driver and detach it from core
 *  \param core Pointer to the USB host core instance
 *  \param func Pointer to the HID mouse function driver instance
 *  \return Operation result status
 */
int32_t hidhf_mouse_deinit(struct usbhc_driver *core, struct hidhf_mouse *func);

/** \brief Register callback functions for HID mouse function driver
 *  \param     func Pointer to the HID mouse function driver instance
 *  \param[in] type Callback type
 *  \param[in] cb   Pointer to callback function
 */
void hidhf_mouse_register_callback(struct hidhf_mouse *func, enum hidhf_mouse_cb_type type, FUNC_PTR cb);

/** \brief Check if the mouse driver is enabled
 *  \param[in] func Pointer to the HID mouse function driver instance
 */
static inline bool hidhf_mouse_is_enabled(struct hidhf_mouse *func)
{
	ASSERT(func);
	return func->is_enabled;
}

/** \brief Return latest left button state
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return \c true if the button is down
 */
static inline bool hidhf_mouse_lbtn(struct hidhf_mouse *func)
{
	ASSERT(func);
	return func->report[0] & 1;
}

/** \brief Return latest right button state
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return \c true if the button is down
 */
static inline bool hidhf_mouse_rbtn(struct hidhf_mouse *func)
{
	ASSERT(func);
	return func->report[0] & 2;
}

/** \brief Return latest middle button state
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return \c true if the button is down
 */
static inline bool hidhf_mouse_mbtn(struct hidhf_mouse *func)
{
	ASSERT(func);
	return func->report[0] & 4;
}

/** \brief Return latest x movement
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return signed data for movement
 */
static inline int8_t hidhf_mouse_x(struct hidhf_mouse *func)
{
	ASSERT(func);
	return (int8_t)func->report[1];
}

/** \brief Return latest y movement
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return signed data for movement
 */
static inline int8_t hidhf_mouse_y(struct hidhf_mouse *func)
{
	ASSERT(func);
	return (int8_t)func->report[2];
}

/** \brief Return latest scroll movement
 *  \param[in] func Pointer to the HID mouse function driver instance
 *  \return signed data for movement
 */
static inline int8_t hidhf_mouse_scroll(struct hidhf_mouse *func)
{
	ASSERT(func);
	return (int8_t)func->report[3];
}

#ifdef __cplusplus
}
#endif

#endif /* USBDF_CDC_ACM_SER_H_ */
