/**
 * \file
 *
 * \brief USB Host Stack HID Mouse Function Implementation.
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

#include "hidhf_mouse.h"

/** \brief Callback invoked when interrupt IN transfer is done */
static void hidhf_mouse_in_end(struct usb_h_pipe *pipe)
{
	struct hidhf_mouse *mouse = (struct hidhf_mouse *)pipe->owner;
	uint8_t *           rpt   = pipe->x.bii.data;
	if (!mouse->is_enabled) {
		return;
	}
	if (pipe->x.bii.status == USB_H_OK && pipe->x.bii.count >= 4) {
		/* Decode mouse report */
		/* - Button state */
		if (mouse->btn_state != hid_mouse_btn_state(rpt)) {
			mouse->btn_state = hid_mouse_btn_state(rpt);
			if (mouse->btn_cb) {
				mouse->btn_cb(mouse, mouse->btn_state);
			}
		}
		/* - X, Y and Scroll */
		if ((hid_mouse_x(rpt) | hid_mouse_y(rpt) | hid_mouse_scroll(rpt)) != 0) {
			if (mouse->move_cb) {
				mouse->move_cb(mouse, hid_mouse_x(rpt), hid_mouse_y(rpt), hid_mouse_scroll(rpt));
			}
		}
	}
	/* Restart interrupt IN anyway */
	usb_h_bulk_int_iso_xfer(pipe, mouse->report, mouse->report_size, false);
}

static void hidhf_mouse_uninstall(struct hidhf_mouse *func)
{
	func->is_enabled = false;
	if (func->pipe[HIDHF_IN_PIPE_IDX]) {
		usb_h_pipe_abort(func->pipe[HIDHF_IN_PIPE_IDX]);
		usb_h_pipe_free(func->pipe[HIDHF_IN_PIPE_IDX]);
	}
}

/** \brief Try to install the mouse driver
 *  \retval ERR_NONE installed
 *  \retval ERR_NOT_FOUND not installed
 *  \retval ERR_NO_CHANGE already taken
 */
static inline int32_t hidhf_mouse_install(struct hidhf_mouse *func, struct usbh_descriptors *desc)
{
	struct usbhd_driver *  dev  = usbhf_get_dev(func);
	struct usbhc_driver *  core = usbhf_get_core(func);
	struct usb_h_desc *    hcd  = core->hcd;
	struct usb_iface_desc *piface;
	struct usb_ep_desc *   pep = NULL;
	struct usb_h_pipe *    pipe;
	uint8_t *              pd;

	if (func->is_enabled) {
		/* Driver already in use */
		return ERR_NO_CHANGE;
	}
	/* find very first interface */
	pd = usb_find_desc(desc->sod, desc->eod, USB_DT_INTERFACE);
	if (!pd) {
		/* No interface found */
		return ERR_NOT_FOUND;
	}
	/* Try to install HID interface */
	piface = (struct usb_iface_desc *)pd;
	if (piface->bInterfaceClass != HID_CLASS || piface->bInterfaceProtocol != HID_PROTOCOL_MOUSE
	    || piface->bNumEndpoints == 0) {
		return ERR_NOT_FOUND;
	}
	/* Find endpoints */
	while (1) {
		pd = usb_desc_next(pd);
		pd = usb_find_ep_desc(pd, desc->eod);
		if (NULL == pd) {
			break;
		}
		pep = (struct usb_ep_desc *)pd;
		if (pep->bEndpointAddress & USB_EP_DIR_OUT) {
			/* Skip OUT for STD mouse only */
			continue;
		}
		/* Break to allocate EP IN */
		break;
	}
	pipe = usb_h_pipe_allocate(hcd,
	                           dev->dev_addr,
	                           pep->bEndpointAddress,
	                           pep->wMaxPacketSize,
	                           pep->bmAttributes,
	                           pep->bInterval,
	                           dev->speed,
	                           true);
	if (pipe == NULL) {
		hidhf_mouse_uninstall(func);
		return ERR_NO_RESOURCE;
	}
	pipe->owner                   = (void *)func;
	func->pipe[HIDHF_IN_PIPE_IDX] = pipe;

	/* Update descriptors pointers */
	desc->sod = usb_find_iface_after((uint8_t *)piface, desc->eod, piface->bInterfaceNumber);

	/* Update status */
	func->iface         = piface->bInterfaceNumber;
	func->is_enabled    = true;
	func->is_bootdevice = (piface->bInterfaceSubClass == HID_SUB_CLASS_BOOT);
	func->report_size   = func->pipe[HIDHF_IN_PIPE_IDX]->max_pkt_size;
	if (func->report_size > CONF_HIDHF_MOUSE_RPT_SIZE) {
		func->report_size = CONF_HIDHF_MOUSE_RPT_SIZE;
	}

	func->btn_state = 0;
	usb_h_pipe_register_callback(pipe, hidhf_mouse_in_end);
	usb_h_bulk_int_iso_xfer(pipe, func->report, func->report_size, false);
	return ERR_NONE;
}

/** \brief Callback invoked on install/uninstall the function driver
 *  \param func  Pointer to the function driver instance
 *  \param ctrl  Control operation code
 *  \param param Parameter for install/uninstall
 */
static int32_t hidhf_mouse_ctrl(struct usbhf_driver *func, enum usbhf_control ctrl, void *param)
{
	switch (ctrl) {
	case USBHF_INSTALL:
		return hidhf_mouse_install(HIDHF_MOUSE_PTR(func), (struct usbh_descriptors *)param);
	case USBHF_UNINSTALL:
		hidhf_mouse_uninstall(HIDHF_MOUSE_PTR(func));
		return ERR_NONE;
	default:
		return ERR_INVALID_ARG;
	}
}

int32_t hidhf_mouse_init(struct usbhc_driver *core, struct hidhf_mouse *func)
{
	int32_t rc = 0;

	if (func->is_enabled) {
		return ERR_DENIED;
	}
	rc = usbhc_register_funcd(core, USBHF_PTR(func));
	if (rc) {
		return rc;
	}
	func->iface         = -1;
	func->is_enabled    = false;
	func->is_bootdevice = false;
	func->pipe[0]       = NULL;

	func->func.ctrl = hidhf_mouse_ctrl;

	func->btn_cb  = NULL;
	func->move_cb = NULL;

	return ERR_NONE;
}

int32_t hidhf_mouse_deinit(struct usbhc_driver *core, struct hidhf_mouse *func)
{
	int32_t rc;
	rc = usbhc_unregister_funcd(core, USBHF_PTR(func));
	if (rc) {
		return rc;
	}
	return ERR_NONE;
}

void hidhf_mouse_register_callback(struct hidhf_mouse *func, enum hidhf_mouse_cb_type type, FUNC_PTR cb)
{
	ASSERT(func);
	switch (type) {
	case HIDHF_MOUSE_BTN_CB:
		func->btn_cb = (hidhf_mouse_btn_cb_t)cb;
		break;
	case HIDHF_MOUSE_MOVE_CB:
		func->move_cb = (hidhf_mouse_move_cb_t)cb;
		break;
	default:
		return;
	}
}
