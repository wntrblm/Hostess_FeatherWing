/**
 * \file
 *
 * \brief USB Host Stack Core Layer Implementation.
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

#include "usbhc.h"
#include <usb_protocol_hub.h>

/** Driver version */
#define USBHC_VERSION 0x00000001u

/** Hub feature selector: port connection */
#define PORT_CONNECTION 0
/** Hub feature selector: port enable */
#define PORT_ENABLE 1
/** Hub feature selector: port suspend */
#define PORT_SUSPEND 2
/** Hub feature selector: port over current */
#define PORT_OVER_CURRENT 3
/** Hub feature selector: port reset */
#define PORT_RESET 4
/** Hub feature selector: port power */
#define PORT_POWER 8
/** Hub feature selector: port low speed */
#define PORT_LOW_SPEED 9
/** Hub feature selector: port high speed */
#define PORT_HIGH_SPEED 10
/** Hub feature selector: port connection change */
#define C_PORT_CONNECTION 16
/** Hub feature selector: port enable change */
#define C_PORT_ENABLE 17
/** Hub feature selector: port suspend change */
#define C_PORT_SUSPEND 18
/** Hub feature selector: port over current change */
#define C_PORT_OVER_CURRENT 19
/** Hub feature selector: port reset change */
#define C_PORT_RESET 20

/** Bits to mask port changes */
#define BM_PORT_CHANGES                                                                                                \
	((1u << C_PORT_CONNECTION) | (1u << C_PORT_SUSPEND) | (1u << C_PORT_OVER_CURRENT) | (1u << C_PORT_RESET))

/** Default endpoint 0 size */
#define EP0_SIZE_DEFAULT 64
/** Max power for a root hub */
#define POWER_MAX (CONF_USBH_POWER_MAX / 2)

/** Get byte access pointer to request in shared control buffer */
#define CTRL_REQ_BUF(buf) ((uint8_t *)buf + USBHC_CTRL_REQ_OFFSET)
/** Get structure access pointer to request in shared control buffer */
#define CTRL_REQ_PTR(buf) ((struct usb_req *)CTRL_REQ_BUF(buf))
#if CONF_USBH_VENDOR_DEV_SP
/** Get byte access pointer to VID in shared control buffer */
#define CTRL_VID_BUF(buf) ((uint8_t *)buf + USBHC_CTRL_VID_OFFSET)
/** Get value of VID saved in shared control buffer */
#define CTRL_VID_VAL(buf) (*(uint16_t *)CTRL_VID_BUF(buf))
/** Get byte access pointer to PID in shared control buffer */
#define CTRL_PID_BUF(buf) (buf + USBHC_CTRL_PID_OFFSET)
/** Get value of PID saved in shared control buffer */
#define CTRL_PID_VAL(buf) (*(uint16_t *)CTRL_PID_BUF(buf))
#endif
/** Get byte access pointer to request data in shared control buffer */
#define CTRL_DATA_BUF(buf) ((uint8_t *)buf + USBHC_CTRL_DATA_OFFSET)
/** Get byte access pointer to descriptors in shared control buffer */
#define CTRL_DESC_BUF(buf) ((uint8_t *)buf + USBHC_CTRL_DATA_OFFSET)
/** Get structure access pointer to device descriptor in shared control buffer
 */
#define CTRL_DDESC_PTR(buf) ((struct usb_dev_desc *)CTRL_DESC_BUF(buf))
/** Get structure access pointer to configuration descriptor in shared control
 *  buffer */
#define CTRL_CFGDESC_PTR(buf) ((struct usb_config_desc *)CTRL_DESC_BUF(buf))
/** Get structure access pointer (actually to word) to hub port status data in
 *  shared control buffer */
#define CTRL_PORT_STATUS_PTR(buf) ((uint32_t *)(CTRL_DATA_BUF(buf) + 0))
/** Get hub port status value (word) in shared control buffer */
#define CTRL_PORT_STATUS(buf) (*CTRL_PORT_STATUS_PTR(buf))

/**
 * \brief Hub operation return code
 */
enum hub_op_rc {
	/** Operation goes on */
	HUB_OP_GO_ON,
	/** Operation in background: fail or process in background.
	 *  When handling of hub status cause setup request, it breaks current loop.
	 */
	HUB_OP_IN_BG,
	/** Operation done.
	 *  When handling of hub status cause error,
	 *  it terminates current hub process.
	 */
	HUB_OP_END
};

/**
 * \brief USB Device Core SOF Handler
 */
struct usbhc_sof_handler {
	struct usbhc_sof_handler *next;
	usbhc_sof_cb_t            cb;
};

/**
 * \brief USB Device Core RSC Handler
 */
struct usbhc_rsc_handler {
	struct usbhc_rsc_handler *next;
	usbhc_rsc_cb_t            cb;
};

/**
 * \brief USB Device Core Enumeration Handler
 */
struct usbhc_enum_handler {
	struct usbhc_enum_handler *next;
	usbhc_enum_cb_t            cb;
};

/**
 * \brief USB Device Core Request Handler
 */
struct usbhc_req_handler {
	struct usbhc_req_handler *next;
	usbhc_req_cb_t            cb;
};

/**
 * \brief Enumeration fail on a device
 * \param     r      Pointer to root device instance
 * \param[in] status Enumeration notification status code
 * \param[in] param  The parameter to pass to callback
 */
static void usbhc_enum_fail(struct usbhc_driver *r, usbhc_enum_status_t status, void *param);

/**
 * \brief Handles hub port reset end
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Hub port number
 * \param[in] status Hub port status
 */
static inline void usbhc_port_reset_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status);

/**
 * \brief Handles hub port connection changes
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Hub port number
 * \param[in] status Hub port status
 */
static inline void usbhc_port_connect_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status);

/**
 * \brief Handles hub port suspend changes
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Hub port number
 * \param[in] status Hub port status
 */
static inline void usbhc_port_suspend_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status);

/**
 * \brief Handles hub port over current
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Hub port number
 * \param[in] status Hub port status
 */
static inline void usbhc_port_overcurr_handler(struct usbhd_driver *hub, uint8_t port);

/**
 * \brief Issue a GetDeviceDescriptor Request
 * \param     r    Pointer to root device instance
 * \param[in] len  Request length
 * \return Control request transfer status
 */
static int32_t _usbhc_get_dev_desc(struct usbhc_driver *r, uint8_t len);

/**
 * \brief Issue a GetConfigDescriptor Request
 * \param     r       Pointer to root device instance
 * \param[in] cfg_idx Configuration index to get
 * \param[in] len     Request length
 * \return Control request transfer status
 */
static int32_t _usbhc_get_config_desc(struct usbhc_driver *r, uint8_t cfg_idx, uint16_t len);

/**
 * \brief Invoke all registered SOF callbacks
 * \param[in] r Pointer to root device instance
 */
static void usbhc_sof_notify(struct usbhc_driver *r)
{
	struct usbhc_sof_handler *sof = (struct usbhc_sof_handler *)r->handlers.sof_list.head;

	while (sof != NULL) {
		sof->cb(r, USBHC_HDL_EXT_PTR(sof));
		sof = sof->next;
	}
}

/**
 * \brief Invoke all registered Enumeration notification callbacks.
 * \param[in] r      Pointer to root device instance
 * \param[in] status Enumeration notification status code
 * \param[in] param  The parameter to pass to callback
 */
static void usbhc_enum_notify(struct usbhc_driver *r, usbhc_enum_status_t status, void *param)
{
	struct usbhc_enum_handler *em = (struct usbhc_enum_handler *)r->handlers.enum_list.head;

	while (em != NULL) {
		em->cb(r, status, param);
		em = em->next;
	}
}

/**
 * \brief Invoke all registered resource waiting callbacks.
 * \param[in] r      Pointer to root device instance
 */
static void usbhc_rsc_notify(struct usbhc_driver *r)
{
	struct usbhc_rsc_handler *rsch = (struct usbhc_rsc_handler *)r->handlers.rsc_list.head;

	if (r->ctrl_status.bm.ctrl_state != USBHC_IDLE) {
		return;
	}

	// printf("RscAvailable ");

	while (rsch != NULL) {
		rsch->cb(r, USBHC_HDL_EXT_PTR(rsch));
		rsch = rsch->next;
	}
}

/**
 * \brief Invoke all registered request callbacks
 * \param[in] r Pointer to root device instance
 * \param[in] d Pointer to device instance
 * \return Number of handlers that handles the request
 */
static uint32_t usbhc_request_handler(struct usbhc_driver *r, struct usbhd_driver *d)
{
	struct usbhc_req_handler *h = (struct usbhc_req_handler *)r->handlers.req_list.head;
	int32_t                   rc;
	uint32_t                  n = 0;

	while (h != NULL) {
		if (NULL != h->cb) {
			rc = h->cb(d, r->rhfunc.pipe_0);
			if (rc == ERR_NONE) {
				n++;
			}
		}
		h = h->next;
	}
	return n;
}

/**
 * \brief Callback invoked on USB host SOF
 * \param[in] hcd Pointer to USB host controller driver instance
 */
static void usbhc_sof_cb(struct usb_h_desc *hcd)
{
	struct usbhc_driver *r = USBHC_PTR(hcd->owner);
	if (r->enum_delay) {
		r->enum_delay--;
		if (r->enum_delay == 0) {
			int32_t rc = (r->dev.status.value < USBHC_DEV_ADDRESS)
			                 ? _usbhc_get_dev_desc(r, (r->dev.ep0_size >= 18) ? 18 : r->dev.ep0_size)
			                 : _usbhc_get_config_desc(r, 0, r->dev.cfg_total);
			if (r->handlers.sof_list.head == NULL) {
				/* Disable SOF callback to reduce CPU interrupts */
				usb_h_register_callback(r->hcd, USB_H_CB_SOF, NULL);
			}
			if (rc != ERR_NONE) {
				/* USBHC_ENUM_FAIL ( usbhc, usbhd ) */
				usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)&r->dev);
			}
		}
	}
	usbhc_sof_notify(r);
}

/**
 * \brief Issue a setup request
 * \param[in]     r    Pointer to root device instance
 * \param[in]     req  Pointer to request
 * \param[in,out] data Pointer to request data
 * \return Request transfer status
 */
int32_t usbhc_request(struct usbhc_driver *r, uint8_t *req, uint8_t *data)
{
	uint16_t len = usb_get_u16(req + 6);
	return usb_h_control_xfer(r->rhfunc.pipe_0, req, data, len, (data && len) ? 500 : 50);
}

/**
 * \brief USB host hub port reset
 * \param[in] r    Pointer to root device instance
 * \param[in] hub  Pointer to hub device instance
 * \param[in] port Port number ( 1 ~ n )
 */
static void usbhc_enum_reset(struct usbhc_driver *r, struct usbhd_driver *hub, uint8_t port)
{
	/* Enumeration started */
	r->ctrl_status.bm.pending_enum = 1;
#if CONF_USBH_HUB_SP
	if (USBHD_PTR(r) != hub) {
		ASSERT(!"Hub port reset not supported yet!");
		/* hubhf_reset(hub, port); */
	} else
#endif
	{
		usb_h_rh_reset(r->hcd, port);
	}
}

#if CONF_USBH_MULTI_DEV_SP
/**
 * \brief USB host hub port suspend
 * \param[in] r    Pointer to root device instance
 * \param[in] hub  Pointer to hub device instance
 * \param[in] port Port number ( 1 ~ n )
 */
static inline void usbhc_enum_suspend(struct usbhc_driver *r, struct usbhd_driver *hub, uint8_t port)
{
	return usbhc_set_port_ftr(r, hub, PORT_SUSPEND, port);
}

/**
 * \brief USB host hub port resume
 * \param[in] r    Pointer to root device instance
 * \param[in] hub  Pointer to hub device instance
 * \param[in] port Port number ( 1 ~ n )
 */
static inline void usbhc_enum_resume(struct usbhc_driver *r, struct usbhd_driver *hub, uint8_t port)
{
	return usbhc_clr_port_ftr(r, hub, PORT_SUSPEND, port);
}
#endif

/**
 * \brief Issue a SetAddress Request
 * \param     r    Pointer to root device instance
 * \param[in] addr Address to set
 * \return Control request transfer status
 */
static inline int32_t usbhc_set_address(struct usbhc_driver *r, uint8_t addr)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
	req->bRequest       = USB_REQ_SET_ADDRESS;
	req->wValue         = addr;
	req->wIndex         = 0;
	req->wLength        = 0;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, NULL, 0, 50);
}

/**
 * \brief Issue a SetConfiguration Request
 * \param     r    Pointer to root device instance
 * \param[in] cfg  Configuration value to set
 * \return Control request transfer status
 */
static int32_t _usbhc_set_config(struct usbhc_driver *r, uint8_t cfg)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
	req->bRequest       = USB_REQ_SET_CONFIG;
	req->wValue         = cfg;
	req->wIndex         = 0;
	req->wLength        = 0;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, NULL, 0, 50);
}

static int32_t _usbhc_get_dev_desc(struct usbhc_driver *r, uint8_t len)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_DESC;
	req->wValue         = (USB_DT_DEVICE << 8);
	req->wIndex         = 0;
	req->wLength        = len;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, CTRL_DESC_BUF(r->ctrl_buf), len, 500);
}

/**
 * \brief Issue a GetConfigDescriptor Request
 * \param     r       Pointer to root device instance
 * \param[in] cfg_idx Configuration index to get
 * \param[in] len     Request length
 * \return Control request transfer status
 */
static int32_t _usbhc_get_config_desc(struct usbhc_driver *r, uint8_t cfg_idx, uint16_t len)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_DESC;
	req->wValue         = (USB_DT_CONFIG << 8) | cfg_idx;
	req->wIndex         = 0;
	req->wLength        = len > 9 ? len : 9;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, CTRL_DESC_BUF(r->ctrl_buf), len, 500);
}

/**
 * \brief Issue a GetStringDescriptor Request
 * \param     r      Pointer to root device instance
 * \param[in] idx    String descriptor index to get
 * \param[in] str_id String ID of the descriptor
 * \return Control request transfer status
 */
static int32_t _usbhc_get_str_desc(struct usbhc_driver *r, uint8_t idx, uint16_t str_id)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_DESC;
	req->wValue         = (USB_DT_STRING << 8) | idx;
	req->wIndex         = str_id;
	req->wLength        = r->ctrl_buf_size - USBHC_CTRL_DATA_OFFSET;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, CTRL_DESC_BUF(r->ctrl_buf), req->wLength, 500);
}

/**
 * \brief Issue a GetInterface Request
 * \param     r      Pointer to root device instance
 * \param[in] iface  Interface index to get
 * \return Control request transfer status
 */
static int32_t _usbhc_get_iface(struct usbhc_driver *r, uint8_t iface)
{
	struct usb_req *req = CTRL_REQ_PTR(r->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_INTERFACE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_INTERFACE;
	req->wValue         = 0;
	req->wIndex         = iface;
	req->wLength        = 1;
	return usb_h_control_xfer(r->rhfunc.pipe_0, (uint8_t *)req, CTRL_DESC_BUF(r->ctrl_buf), req->wLength, 500);
}

/**
 * \brief Issue a SetInterface Request
 * \param     r      Pointer to root device instance
 * \param[in] iface  Interface index
 * \param[in] alt    Alternate setting index
 * \return Control request transfer status
 */
static int32_t _usbhc_set_iface(struct usbhc_driver *root, uint8_t iface, uint8_t alt)
{
	struct usb_req *req = CTRL_REQ_PTR(root->ctrl_buf);
	req->bmRequestType  = USB_REQT_RECIP_INTERFACE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
	req->bRequest       = USB_REQ_SET_INTERFACE;
	req->wValue         = alt;
	req->wIndex         = iface;
	req->wLength        = 0;
	return usb_h_control_xfer(root->rhfunc.pipe_0, (uint8_t *)req, NULL, 0, 50);
}

/**
 * \brief Find upper level parent of a device
 * \param[in] d Pointer to device instance
 *
 * \code
 *
 * Device physical connection:
 *
 * RH ---- H1 ---- D11
 * |       |  ---- D12
 * |------D2
 * |------D3
 * |------H4 ---- D41
 * |----- D5
 *
 * Device logical connection:
 *
 * RH                                         . level 0
 * | -> H1 -> D2 -> D3 -> H4 -> D5 -> (RH)    . level 1
 *       |                 | -> D41 -> (H4)   . level 0
 *       | -> D11 -> D21 -> (H1)              . level 0
 *
 * Multiple device driver:
 *
 * root->dev.next       -> Free device driver list      ... -> root
 * root->dev.func       -> Root hub function driver
 * root->dev.func->next -> Free function driver list    ... -> NULL
 * hub->child     -> Connected device driver list ... -> hub
 * func->pdev     -> Device driver connected to (NULL if function is free)
 *
 * Single device driver, merge root and the only connected device
 *
 * root->dev.next -> root
 * root->dev.func -> All function drivers list    ... -> NULL
 * root->dev.hub_port:     Connected    - n
 *                         Disconnected - 0
 * root->dev.ep0_size:     Connected    - device EP0 size
 *                         Disconnected - default (64)
 * root->dev.dev_addr:     Always 1
 *
 * \endcode
 */
static struct usbhd_driver *usbhc_find_parent(struct usbhd_driver *d)
{
#if CONF_USBH_MULTI_DEV_SP
	struct usbhd_driver *p            = d;
	bool                 target_level = !p->level;
	if (p->dev_type == USBHC_ROOT) { /* Already the root, just use it */
		return p;
	}
	while (p && p->level != target_level) {
		p = p->next.pdev;
	}
	return p;
#else
	/* Root (hub) and device merged into single instance */
	return USBHD_PTR(d);
#endif
}

/**
 * \brief Find root device
 * \param[in] d Pointer to device instance
 */
static struct usbhc_driver *usbhc_find_root(struct usbhd_driver *d)
{
#if CONF_USBH_MULTI_DEV_SP
	struct usbhd_driver *p = d;
	do {
		if (p->dev_type == USBHC_ROOT) {
			return USBHC_PTR(p);
		}
		p = usbhc_find_parent(p);
	} while (p);
	return NULL;
#else
	/* Root and device merged into single instance */
	return USBHC_PTR(usbhc_find_parent(d));
#endif
}

/**
 * \brief Find hub for a device
 * \param[in] d Pointer to device instance
 */
static inline struct usbhd_driver *usbhc_find_hub(struct usbhd_driver *dev)
{
	return usbhc_find_parent(dev);
}

/**
 * \brief Find device on a hub
 * \param[in] d Pointer to device instance
 */
static struct usbhd_driver *usbhc_find_dev_on_hub(struct usbhd_driver *hub, uint8_t port)
{
#if CONF_USBH_MULTI_DEV_SP
	struct hubhf_driver *hubf = HUBHF_PTR(hub->func.pfunc);
	struct usbhd_driver *p;
	if (!hubf) { /* Hub not valid */
		return NULL;
	}
	if (hubf->child.pdev == hub) { /* Empty list */
		return NULL;
	}
	p = hubf->child.pdev;
	while (p->level != hub->level) { /* Scan children */
		if (p->hub_port == port) {   /* Found it */
			return p;
		}
		p = p->next.pdev;
	}
	/* Not found */
	return NULL;
#else
	struct usbhc_driver *r = USBHC_PTR(hub);
	if (r->dev.hub_port != port) { /* Must registered */
		return NULL;
	}
	return &r->dev;
#endif
}

#if CONF_USBH_HUB_SP
/**
 * \brief Disable physical device
 * \param     r   Pointer to root device instance
 * \param[in] hub Pointer to hub device instance
 * \param[in] d   Pointer to device instance
 */
static void usbhc_disable_dev(struct usbhc_driver *r, struct usbhd_driver *hub, struct usbhd_driver *d)
{
	if (d->dev_type == USBHC_ROOT) {
		/* Never disable HC root here */
		return;
	}
	usbhc_clr_port_ftr(r, hub, PORT_ENABLE, d->hub_port);
	/* There will be port changes interrupt then */
}
#endif

void usbhc_release_control(struct usbhc_driver *r)
{
	if (r) {
		r->ctrl_status.bm.ctrl_state   = USBHC_IDLE;
		r->ctrl_status.bm.pending_enum = 0;
#if CONF_USBH_MULTI_DEV_SP
		r->rhfunc.enum_dev = NULL;
#endif
	}
}

int32_t usbhc_take_control(struct usbhd_driver *d, struct usbhc_driver **r, const enum usbhc_ctrl_state s)
{
	struct usbhc_driver *core;
	int32_t              rc;
	if (s == USBHC_USED_BY_DEV) {
		/* Check device state */
		if (!d->status.bm.usable) {
			/* Device access denied */
			return ERR_DENIED;
		}
		if (d->status.bm.suspend) {
			/* Device is not active */
			return ERR_SUSPEND;
		}
	}
	/* Check core state */
	core = usbhc_find_root(d);
	if (!(core)) {
		/* Device not connected to the core */
		return ERR_NOT_INITIALIZED;
	}
	if (core->ctrl_status.bm.ctrl_state != USBHC_IDLE) {
		/* Core control resource is busy */
		return ERR_BUSY;
	}
	core->ctrl_status.bm.ctrl_state = s;

	rc = usb_h_pipe_set_control_param(core->rhfunc.pipe_0, d->dev_addr, 0, d->ep0_size, d->speed);
	if (rc < 0) {
		core->ctrl_status.bm.ctrl_state = USBHC_IDLE;
		return ERR_IO;
	}
#if CONF_USBH_MULTI_DEV_SP
	core->rhfunc.enum_dev = d;
#endif
	if (r) {
		*r = core;
	}
	return ERR_NONE;
}

/**
 * \brief Try to find function driver
 *
 * The descriptor pointers are updated if function driver takes effect
 *
 * \param     r    Pointer to root device instance
 * \param     d    Pointer to device instance
 * \param[in] desc Pointer to descriptors
 *
 * \return \c true if the function driver takes effect
 */
static bool usbhc_enum_installf(struct usbhc_driver *r, struct usbhd_driver *d, struct usbh_descriptors *desc)
{
	struct usbhf_driver *func      = r->rhfunc.func_list.pfunc; /* From Free list */
	int32_t              rc        = ERR_NOT_INITIALIZED;
	bool                 installed = false;
	while (func) {
		if (func->ctrl) {
			rc = func->ctrl(func, USBHF_INSTALL, desc);
		}
		if (rc == ERR_NONE) { /* Installed OK */
			/* Attached to device */
			func->pdev = d;
			/* Move from free list to device functions */
			list_delete_element(&r->rhfunc.func_list.list, (void *)func);
			list_insert_as_head(&d->func.list, (void *)func);
			/* It's installed */
			installed = true;
			break;
		}
		func = func->next.pfunc;
	}
	return installed;
}

/**
 * \brief Uninstall all functions on a device
 * \param r Pointer to root device instance
 * \param d Pointer to device instance
 */
static void usbhc_enum_uninstallf(struct usbhc_driver *r, struct usbhd_driver *d)
{
	struct usbhf_driver *func;
	func = d->func.pfunc;
	while (func) {
		/* Invoke uninstall callback */
		if (func->ctrl) {
			func->ctrl(func, USBHF_UNINSTALL, NULL);
		}
		func->pdev = &r->dev;

		/* Move function to free list */
		d->func.pfunc = func->next.pfunc;
		list_insert_as_head(&r->rhfunc.func_list.list, (void *)func);

		/* Check next function */
		func = d->func.pfunc;
	}
}

#if CONF_USBH_POWER_MAX
/**
 * \brief Update power usage
 * \param r Pointer to root device instance
 * \param d Pointer to device instance
 */
static bool usbhc_enum_more_power(struct usbhc_driver *r, struct usbhd_driver *d)
{
#if CONF_USBH_HUB_SP
	struct usbhd_driver *hub = usbhc_find_hub(d);
	if (hub != USBHD_PTR(r)) {
		/* Handled by hub, no need to update bus power */
		return true;
	}
#endif
	if (r->rhfunc.power + d->cfg_power > POWER_MAX) {
		return false;
	}
	r->rhfunc.power += d->cfg_power;
	return true;
}
#endif

/**
 * \brief Install function drivers based on configuration descriptor
 * \param r Pointer to root device instance
 * \param d Pointer to device instance
 * \return Number of functions installed
 */
static uint8_t usbhc_enum_install(struct usbhc_driver *r, struct usbhd_driver *d)
{
	uint8_t *               cfg_desc  = CTRL_DESC_BUF(r->ctrl_buf);
	uint16_t                total_len = usb_cfg_desc_total_len(cfg_desc);
	uint8_t                 n_func    = 0;
	struct usbh_descriptors desc;

	desc.eod = cfg_desc + total_len;
	desc.sod = usb_find_desc(cfg_desc, desc.eod, USB_DT_INTERFACE);

	while (NULL != desc.sod) {
		/* Install function driver and move descriptor position */
		if (!usbhc_enum_installf(r, d, &desc)) {
			/* Notify an unsupported interface
			 * USBHC_ENUM_IFACE_UNSUPPORTED ( usbhc, if_desc )
			 */
			usbhc_enum_notify(r, USBHC_ENUM_IFACE_UNSUPPORTED, &desc);
			/* Skip unsupported interface (try next) */
			desc.sod = usb_desc_next(desc.sod);
		} else {
			/* Installed */
			n_func++;
		}
		desc.sod = usb_find_desc(desc.sod, desc.eod, USB_DT_INTERFACE);
	}
	return n_func;
}

/**
 * \brief Uninstall function drivers on a device
 * \param r   Pointer to root device instance
 * \param hub Pointer to hub device instance
 * \param d   Pointer to device instance
 */
static void usbhc_enum_uninstall(struct usbhc_driver *r, struct usbhd_driver *hub, struct usbhd_driver *d)
{
#if CONF_USBH_MULTI_DEV_SP
	if (d->dev_type != USBHC_DEV) {
		struct hubhf_driver *hubf = hub->func.phub;
		/* If the device is a (root) hub
		 * Disable all things connected to this hub
		 */
		while (hubf->child.pdev != d) {
			usbhc_enum_uninstall(r, d, hubf->child.pdev);
		}
	}
#endif

		/* Disable this device */

#if CONF_USBH_MULTI_DEV_SP
	/* Do nothing if it's root device */
	if (d->dev_type == USBHC_ROOT) {
		return;
	}
#endif

	/* Uninstall all functions */
	usbhc_enum_uninstallf(r, d);

#if CONF_USBH_POWER_MAX
	/* Consume less power */
	if (USBHD_PTR(r) == hub) {
		r->rhfunc.power -= d->cfg_power;
	}
#endif

#if CONF_USBH_HUB_SP
	/* Disable physical device if parent hub keeps on */
	if (hub->status.value != USBHC_DEV_REMOVING) {
		usbhc_disable_dev(r, hub, d);
	}
#endif

#if CONF_USBH_MULTI_DEV_SP
	/* Moved from hub (bus tree) to free list */
	{
		struct usbhd_driver *prev = hub->func.phub->child.pdev;
		if (prev != d) {
			while (prev->next.pdev != d) {
				prev = prev->next.pdev;
			}
			prev->next.pdev = d->next.pdev;
		} else {
			hub->func.phub->child.pdev = d->next.pdev;
		}
	}
	list_insert_as_head(&r->dev.next.list, (void *)d);
#else
	/* No need to move instance, since it's directly used */
	(void)hub;
	(void)r;
#endif

	/* Notification: USBHC_ENUM_DISCONNECTED (usbhc, usbhd) */
	d->status.value = USBHC_DEV_DISCONNECT;
	d->hub_port     = 0;
	usbhc_enum_notify(r, USBHC_ENUM_DISCONNECTED, (void *)d);
}

/**
 * \brief Enumeration fail on a device
 * \param     r      Pointer to root device instance
 * \param[in] status Enumeration status code
 * \param[in] param  Callback parameter
 */
static void usbhc_enum_fail(struct usbhc_driver *r, usbhc_enum_status_t status, void *param)
{
	/* Free control resources */
	usbhc_release_control(r);

	if (status < USBHC_ENUM_IO_FAIL) {
		struct usbhd_driver *d = USBHD_PTR(param);
		/* Uninstall function drivers */
		usbhc_enum_uninstallf(r, d);
		/* Device is fail */
		if (status < USBHC_ENUM_UNSUPPORTED) { /* It's not accessible */
			d->status.value = USBHC_DEV_FAILED;
		}
		if (d->status.bm.state >= USBHC_DEV_ADDRESS) {
			d->status.bm.usable = 1; /* It's still usable */
		}
	} else if (status == USBHC_ENUM_IO_FAIL) {
		struct usbhd_driver *d = USBHD_PTR(param);
		d->status.value        = USBHC_DEV_FAILED;
	}
	usbhc_enum_notify(r, status, param);

	/** Resource may be available */
	usbhc_rsc_notify(r);
}

/**
 * \brief Set Address End
 * \param r   Pointer to root device instance
 * \param d   Pointer to device instance
 */
static void usbhc_set_address_end(struct usbhc_driver *r, struct usbhd_driver *d)
{
	int32_t rc;
	d->status.value = USBHC_DEV_ADDRESS;
	/* Change address */
	rc = usb_h_pipe_set_control_param(r->rhfunc.pipe_0, d->dev_addr, 0, d->ep0_size, d->speed);
	if (rc < 0) {
		usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
		return;
	}
	/* Issue get configuration descriptor after 5ms */
	r->enum_delay = 5;
	usb_h_register_callback(r->hcd, USB_H_CB_SOF, (FUNC_PTR)usbhc_sof_cb);
}

/**
 * \brief Set Configuration End
 * \param r   Pointer to root device instance
 * \param d   Pointer to device instance
 */
static void _usbhc_set_config_end(struct usbhc_driver *r, struct usbhd_driver *d)
{
	struct usbhf_driver *f;

	/* Device is configured (usable, no busy, no suspend) */
	d->status.value = USBHC_DEV_CONFIGURED | USBHC_DEV_USABLE;

	/* Free control resources */
	usbhc_release_control(r);

	/* Enable functions on device */
	f = d->func.pfunc;
	while (f) {
		if (f->ctrl) {
			f->ctrl(f, USBHF_ENABLE, NULL);
		}
		f = f->next.pfunc;
	}

	/* USBHC_ENUM_SUCCESS ( usbhc, usbhd ) */
	usbhc_enum_notify(r, USBHC_ENUM_SUCCESS, (void *)d);

	/** Resource may be available */
	usbhc_rsc_notify(r);
}

/**
 * \brief Get Device Descriptor End
 * \param r   Pointer to root device instance
 * \param d   Pointer to device instance
 */
static void _usbhc_get_dev_desc_end(struct usbhc_driver *r, struct usbhd_driver *d)
{
	uint8_t *dev_desc      = CTRL_DESC_BUF(r->ctrl_buf);
	uint8_t  bMaxPackSize0 = dev_desc[7];
#if CONF_USBH_HUB_SP
	uint8_t bDeviceClass = dev_desc[4];
#endif
	int32_t rc;
	if (r->rhfunc.pipe_0->x.ctrl.status != ERR_NONE || r->rhfunc.pipe_0->x.ctrl.count < 8) {
		/* GetDeviceDescriptor error
		 * USBHC_ENUM_FAIL ( usbhc, usbhd )
		 */
		usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
		return;
	}

	/* Process device descriptor - Update ep0 size */
	d->ep0_size = bMaxPackSize0;
	if (r->rhfunc.pipe_0->x.ctrl.count < sizeof(usb_dev_desc_t)) {
		/* Need change max packet size */
		rc = usb_h_pipe_set_control_param(
		    r->rhfunc.pipe_0, r->rhfunc.pipe_0->dev, r->rhfunc.pipe_0->ep, bMaxPackSize0, r->dev.speed);
		if (rc == ERR_NONE) {
			/* Read device descriptor again */
			rc = _usbhc_get_dev_desc(r, sizeof(usb_dev_desc_t));
		}
		if (rc < 0) {
			/* Size in device descriptor not accepted
			 * USBHC_ENUM_HAL_LIMIT ( usbhc, dev_desc )
			 */
			usbhc_enum_fail(r, USBHC_ENUM_HAL_LIMIT, (void *)dev_desc);
			return;
		}
	} else {
#if CONF_USBH_HUB_SP
		if (bDeviceClass == 0x09) { /* It's hub */
			d->dev_type = USBHC_HUB;
		}
#endif
#if CONF_USBH_VENDOR_DEV_SP
		CTRL_VID_BUF(r->ctrl_buf)[0] = dev_desc[8];
		CTRL_VID_BUF(r->ctrl_buf)[1] = dev_desc[9];
		CTRL_PID_BUF(r->ctrl_buf)[0] = dev_desc[10];
		CTRL_PID_BUF(r->ctrl_buf)[1] = dev_desc[11];
#endif
		/* Issue SetAddress */
		rc = usbhc_set_address(r, d->dev_addr);
		if (rc < 0) {
			usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
		}
	}
}

/**
 * \brief Get Device Descriptor End
 * \param r   Pointer to root device instance
 * \param d   Pointer to device instance
 */
static void _usbhc_get_config_desc_end(struct usbhc_driver *r, struct usbhd_driver *d)
{
	int32_t            rc;
	uint8_t *          data = CTRL_DESC_BUF(r->ctrl_buf);
	usb_config_desc_t *cfg  = (usb_config_desc_t *)data;

	if (r->rhfunc.pipe_0->x.ctrl.count < sizeof(usb_config_desc_t)) {
		/* USBHC_ENUM_FAIL ( usbhc, usbhd ) */
		usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
		return;
	} else if (r->rhfunc.pipe_0->x.ctrl.count == sizeof(usb_config_desc_t)) {
		/* Need read configuration descriptors again */
		d->cfg_total = cfg->wTotalLength;
		if (d->cfg_total > r->ctrl_buf_size - sizeof(usb_req_t)) {
			/* USBHC_ENUM_MEM_LIMIT ( usbhc, usbhd ) */
			usbhc_enum_fail(r, USBHC_ENUM_MEM_LIMIT, (void *)d);
			return;
		}
		rc = _usbhc_get_config_desc(r, 0, d->cfg_total);
		if (rc < 0) {
			/* USBHC_ENUM_FAIL ( usbhc, usbhd ) */
			usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
			return;
		}
		return;
	}

#if CONF_USBH_POWER_MAX
	d->cfg_power = cfg->bMaxPower;
	if (!usbhc_enum_more_power(r, d)) {
		/* USBHC_ENUM_OVERCURRENT ( usbhc, usbhd ) */
		usbhc_enum_fail(r, USBHC_ENUM_OVERCURRENT, (void *)d);
	}
#endif

	/* Install function drivers according to the descriptors */
	/* Issue SetConfigure if at least one function is installed OK */
	if (usbhc_enum_install(r, d)) {
		rc = _usbhc_set_config(r, cfg->bConfigurationValue);
		if (rc < 0) {
			/* USBHC_ENUM_FAIL ( usbhc, usbhd ) */
			usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)d);
		}
	} else {
		/* USBHC_ENUM_UNSUPPORTED ( usbhc, usbhd ) */
		usbhc_enum_fail(r, USBHC_ENUM_UNSUPPORTED, (void *)d);
	}
}

/**
 * \brief Get current device that using control pipe
 * \param r   Pointer to root device instance
 */
static struct usbhd_driver *usbhc_get_curr_ctrl_dev(struct usbhc_driver *r)
{
#if CONF_USBH_MULTI_DEV_SP
	return r->rhfunc.enum_dev;
#else
	return &r->dev;
#endif
}

/**
 * \brief To update param when USB host driver is done.
 * \param pipe Pointer to pipe instance
 */
static void usbhc_pipe_0_xfer_done(struct usb_h_pipe *pipe)
{
	struct usbhc_driver *r        = USBHC_PTR(pipe->hcd->owner);
	struct usbhd_driver *d        = usbhc_get_curr_ctrl_dev(r);
	struct usb_req *     req      = CTRL_REQ_PTR(r->ctrl_buf);
	uint16_t             req_type = req->wValue >> 8;

	if (pipe->ep != 0) {
		/* Not default request, just ignore */
		return;
	}

	if (r->ctrl_status.bm.ctrl_state == USBHC_USED_BY_CORE) {
		if (pipe->x.general.status < 0) {
			usbhc_enum_fail(r, USBHC_ENUM_IO_FAIL, (void *)d);
			return;
		}

		/* Core Enumeration in progress */
		switch (req->bRequest) {
		case USB_REQ_GET_DESC:
			if (req_type == USB_DT_DEVICE) {
				_usbhc_get_dev_desc_end(r, d);
			} else if (req_type == USB_DT_CONFIG) {
				_usbhc_get_config_desc_end(r, d);
			}
			break;
		case USB_REQ_SET_CONFIG:
			_usbhc_set_config_end(r, d);
			break;
		case USB_REQ_SET_ADDRESS:
			usbhc_set_address_end(r, d);
			break;
#if CONF_USBH_HUB_SP
		case USB_REQ_GET_STATUS:
			if (req->bmRequestType == 0xA3) { /* GetPortStatus */
				usbhc_get_port_status_end(r, usbhc_find_hub(d));
			}
			break;
		case USB_REQ_CLEAR_FTR:
			if (req->bmRequestType == 0x23) { /* ClearPortFeature */
				usbhc_clr_port_ftr_end(r, usbhc_find_hub(d));
			}
			break;
#endif
		default:
			break;
		}
		return;
	}
	usbhc_release_control(r);
	/* Let drivers handle control transfer results */
	usbhc_request_handler(r, d);

	/** Resource may be available */
	usbhc_rsc_notify(r);
}

/**
 * \brief USB host reset event handler
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Port number
 * \param[in] status Speed
 */
static void usbhc_port_reset_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status)
{
	struct usbhc_driver *r   = usbhc_find_root(USBHD_PTR(hub));
	struct usbhd_driver *dev = usbhc_find_dev_on_hub(hub, port);
#if CONF_USBH_MULTI_DEV_SP
	struct hubhf_driver *hubf = HUBHF_PTR(hub->func.pfunc);
#endif
	int32_t rc;
	/*
	 * A device is reset on the port.
	 * For new device:
	 * - User very first device in free list to enum it.
	 * - And switch to hub device if it's enumerated as a hub.
	 * For existing device:
	 * - Use the previous settings
	 */
	if (dev) {
	/* Existing device */
#if CONF_USBH_POWER_MAX
		if (hub->dev_type == USBHC_ROOT) {
			r->rhfunc.power -= dev->cfg_power;
		}
#endif
	}
#if CONF_USBH_MULTI_DEV_SP
	else if (r->dev.next.proot == r) {
		/* No free device, enumeration fail */
		usbhc_enum_fail(r, USBHC_ENUM_DEVD_LIMIT, (void *)dev);
		return;
	}
#endif
	else {
	/* New device */
#if CONF_USBH_MULTI_DEV_SP
		dev           = r->dev.next.pdev;
		dev->ep0_size = r->dev.ep0_size;
		/* Connected as device and change after get device information */
		dev->dev_type = USBHC_DEV;
#else
		dev           = &r->dev;
		dev->ep0_size = EP0_SIZE_DEFAULT;
#endif
		dev->cfg_total = sizeof(usb_config_desc_t);
		dev->hub_port  = port;
		dev->speed     = status;
#if CONF_USBH_MULTI_DEV_SP
		/* And remove from free list */
		list_remove_head(&r->dev.next.list);
		/* Link device to the hub */
		dev->next.pdev   = hubf->child.pdev;
		hubf->child.pdev = dev;

		/* Assign level */
		dev->level = !hub->level;
		/* Fast access to current processing device */
		r->rhfunc.enum_dev = dev;
#endif
	}

	/* Re-initialize pipe 0 */
	if (r->rhfunc.pipe_0 == NULL) {
		r->rhfunc.pipe_0 = usb_h_pipe_allocate(r->hcd, 0, 0, EP0_SIZE_DEFAULT, 0, 0, dev->speed, true);
		if (r->rhfunc.pipe_0 == NULL) {
			/* USBHC_ENUM_HAL_LIMIT ( usbhc, dev_desc ) */
			usbhc_enum_fail(r, USBHC_ENUM_HAL_LIMIT, NULL);
			return;
		}
		usb_h_pipe_register_callback(r->rhfunc.pipe_0, usbhc_pipe_0_xfer_done);
	}

	dev->status.value = USBHC_DEV_DEFAULT;
	dev->cfg_power    = 0;

	/* Issue GetDeviceDescriptor. */
	rc = usb_h_pipe_set_control_param(r->rhfunc.pipe_0, 0, 0, dev->ep0_size, dev->speed);
	if (rc == ERR_NONE) {
		if (dev->speed > USB_SPEED_LS) {
			/* Delay 50ms before start enum requests */
			r->enum_delay = 50;
			usb_h_register_callback(r->hcd, USB_H_CB_SOF, (FUNC_PTR)usbhc_sof_cb);
			return;
		}
		rc = _usbhc_get_dev_desc(r, (dev->ep0_size >= 18) ? 18 : dev->ep0_size);
		if (rc != ERR_NONE) {
			/* USBHC_ENUM_FAIL ( usbhc, usbhd ) */
			usbhc_enum_fail(r, USBHC_ENUM_FAIL, (void *)dev);
		}
	} else {
		/* USBHC_ENUM_HAL_LIMIT ( usbhc, dev_desc ) */
		usbhc_enum_fail(r, USBHC_ENUM_HAL_LIMIT, NULL);
	}
}

/**
 * \brief USB host suspend/resume event handler
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Port number
 * \param[in] status Port suspend status
 */
static void usbhc_port_suspend_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status)
{
	struct usbhd_driver *dev = usbhc_find_dev_on_hub(USBHD_PTR(hub), port);
	if (status) {
		if (dev != NULL) {
			dev->status.bm.suspend = true;
			usbhc_enum_notify(usbhc_find_root(hub), USBHC_ENUM_SUSPEND_CHG, (void *)dev);
		}
	} else {
		(void)hub;
		(void)port;
		if (dev != NULL) {
			dev->status.bm.suspend = false;
			usbhc_enum_notify(usbhc_find_root(hub), USBHC_ENUM_SUSPEND_CHG, (void *)dev);
		}
	}
}

/**
 * \brief USB host over current event handler
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Port number
 */
void usbhc_port_overcurr_handler(struct usbhd_driver *hub, uint8_t port)
{
	struct usbhd_driver *dev = usbhc_find_dev_on_hub(USBHD_PTR(hub), port);
	if (dev) {
		dev->status.value = USBHC_DEV_FAILED; /* Device is over current */
		usbhc_enum_notify(usbhc_find_root(hub), USBHC_ENUM_OVERCURRENT, (void *)dev);
	}
}

/**
 * \brief USB host connection/disconnection event handler
 * \param     hub    Pointer to hub device instance
 * \param[in] port   Port number
 * \param[in] status Port connection status
 */
static void usbhc_port_connect_handler(struct usbhd_driver *hub, uint8_t port, uint32_t status)
{
	if (status) {
		/* A device is connected.
		 * Start enum - reset it.
		 */
		usbhc_enum_reset(usbhc_find_root(hub), hub, port);
	} else {
		/* A device is disconnected.
		 * Uninstall it.
		 */
		usbhc_enum_uninstall(usbhc_find_root(hub), hub, usbhc_find_dev_on_hub(hub, port));
	}
}

/**
 * \brief Callback invoked on root hub changes detected
 * \param hcd Pointer to host controller driver
 */
static void usbhc_rh_change_cb(struct usb_h_desc *hcd, uint8_t port, uint8_t ftr)
{
	struct usbhc_driver *r = USBHC_PTR(hcd->owner);
	int32_t              status;
	/* Hub feature/status change has been cleared before notification */
	/* It's end of hub driver request, take control by core */
	r->ctrl_status.bm.ctrl_state = USBHC_USED_BY_CORE;
	/* Handle the feature changes */
	switch (ftr) {
	case PORT_CONNECTION:
		status = usb_h_rh_check_status(hcd, port, ftr);
		usbhc_port_connect_handler(USBHD_PTR(r), port, status);
		return;
	case PORT_RESET:
		status = usb_h_rh_check_status(hcd, port, PORT_LOW_SPEED)
		             ? USB_SPEED_LS
		             : (usb_h_rh_check_status(hcd, port, PORT_HIGH_SPEED) ? USB_SPEED_HS : USB_SPEED_FS);
		usbhc_port_reset_handler(USBHD_PTR(r), port, status);
		return;
	case PORT_SUSPEND:
		status = usb_h_rh_check_status(hcd, port, ftr);
		usbhc_port_suspend_handler(USBHD_PTR(r), port, status);
		return;
	case PORT_OVER_CURRENT:
		usbhc_port_overcurr_handler(USBHD_PTR(r), port);
	default:
		break;
	}
}

int32_t usbhc_init(struct usbhc_driver *core, struct usb_h_desc *hcd, uint8_t *buf, uint16_t buf_size)
{
	ASSERT(core && hcd && buf && (buf_size > 32));

	if (core->ctrl_buf != NULL) {
		return ERR_ALREADY_INITIALIZED;
	}

	core->dev.ep0_size = EP0_SIZE_DEFAULT;
	core->hcd          = hcd;
	hcd->owner         = (void *)core;

	/* it's core/root device */
	core->dev.dev_type = USBHC_ROOT;

	/* Empty free device list */
	list_reset(&core->dev.next.list);

	/* Empty free function list */
	list_reset(&core->rhfunc.func_list.list);

#if CONF_USBH_POWER_MAX
	/* Initialize power consumption */
	core->rhfunc.power = 0;
#endif

	/* Empty handlers list */
	list_reset(&core->handlers.sof_list);
	list_reset(&core->handlers.enum_list);
	list_reset(&core->handlers.req_list);
	list_reset(&core->handlers.rsc_list);

#if CONF_USBH_MULTI_DEV_SP
	/* Multiple device support, the root hub set address to 0 */
	core->dev.dev_addr = 0;
	/* The root hub function */
	core->dev.func.pfunc = USBHF_PTR(&core->rhfunc);
	/* No connected device to root hub */
	core->rhfunc.child.pdev = &core->dev;
#else
	/* Single device support, share the structure with device address 1 */
	core->dev.dev_addr = 1;
	list_reset(&core->dev.func.list);
#endif
	core->dev.level    = 0;
	core->dev.hub_port = 0;

	core->dev.cfg_total = 0;

	usb_h_register_callback(core->hcd, USB_H_CB_ROOTHUB_CHANGE, (FUNC_PTR)usbhc_rh_change_cb);

	core->ctrl_buf          = buf;
	core->ctrl_buf_size     = buf_size;
	core->ctrl_status.value = 0;

	/* Core is stopped */
	core->dev.status.value = USBHC_DEV_DISCONNECT;
	return ERR_NONE;
}

int32_t usbhc_deinit(struct usbhc_driver *core)
{
	ASSERT(core);

	if (core->ctrl_buf == NULL) {
		return ERR_NOT_INITIALIZED;
	}
	usbhc_stop(core);

	if (core->rhfunc.pipe_0) {
		usb_h_pipe_free(core->rhfunc.pipe_0);
		core->rhfunc.pipe_0 = NULL;
	}
	core->ctrl_buf   = NULL;
	core->hcd->owner = NULL;

	usb_h_deinit(core->hcd);
	core->hcd = NULL;

	/* Free all drivers */
	usbhc_enum_uninstall(core, USBHD_PTR(core), USBHD_PTR(core));
	return ERR_NONE;
}

#if CONF_USBH_MULTI_DEV_SP
int32_t usbhc_register_devd(struct usbhc_driver *core, struct usbhd_driver *devd)
{
	ASSERT(core && devd);
	if (core->dev.status.value != USBHC_DEV_DISCONNECT) {
		return ERR_DENIED;
	}
	/* Insert at head */
	list_insert_as_head(&core->dev.next.list, (void *)devd);
	/* No connection */
	devd->dev_type     = USBHC_DEV;
	devd->status.value = USBHC_DEV_DISCONNECT;
	devd->hub_port     = 0;
	/* No function */
	devd->func.pfunc = NULL;
	return ERR_NONE;
}

int32_t usbhc_unregister_devd(struct usbhc_driver *core, struct usbhd_driver *devd)
{
	ASSERT(core && devd);
	if (core->dev.status.value != USBHC_DEV_DISCONNECT) {
		return ERR_DENIED;
	}
	list_delete_element(&core->dev.next.list, (void *)devd);
	return ERR_NONE;
}
#endif

int32_t usbhc_register_funcd(struct usbhc_driver *core, struct usbhf_driver *funcd)
{
	ASSERT(core && funcd);
	if (core->dev.status.value != USBHC_DEV_DISCONNECT) {
		return ERR_DENIED;
	}
	/* Insert at free function list head */
	list_insert_as_head(&core->rhfunc.func_list.list, (void *)funcd);
	/* Not connected, link to ROOT */
	funcd->pdev = &core->dev;
	return ERR_NONE;
}

int32_t usbhc_unregister_funcd(struct usbhc_driver *core, struct usbhf_driver *funcd)
{
	ASSERT(core && funcd && funcd->ctrl);
	if (core->dev.status.value != USBHC_DEV_DISCONNECT) {
		return ERR_DENIED;
	}
	list_delete_element(&core->rhfunc.func_list.list, (void *)funcd);
	funcd->pdev = NULL;
	return ERR_NONE;
}

void usbhc_register_handler(struct usbhc_driver *core, enum usbhc_handler_type type, const struct usbhc_handler *h)
{
	ASSERT(core && h && h->func);
	switch (type) {
	case USBHC_HDL_SOF:
		if (is_list_element(&core->handlers.sof_list, (void *)h)) {
			return;
		}
		if (core->handlers.sof_list.head == NULL) {
			usb_h_register_callback(core->hcd, USB_H_CB_SOF, (FUNC_PTR)usbhc_sof_cb);
		}
		list_insert_as_head(&core->handlers.sof_list, (void *)h);
		break;
	case USBHC_HDL_ENUM:
		if (is_list_element(&core->handlers.enum_list, (void *)h)) {
			return;
		}
		list_insert_as_head(&core->handlers.enum_list, (void *)h);
		break;
	case USBHC_HDL_REQ:
		if (is_list_element(&core->handlers.req_list, (void *)h)) {
			return;
		}
		list_insert_as_head(&core->handlers.req_list, (void *)h);
		break;
	case USBHC_HDL_RSC:
		if (is_list_element(&core->handlers.rsc_list, (void *)h)) {
			return;
		}
		list_insert_as_head(&core->handlers.rsc_list, (void *)h);
		break;
	default:
		break;
	}
}

void usbhc_unregister_handler(struct usbhc_driver *core, enum usbhc_handler_type type, const struct usbhc_handler *h)
{
	ASSERT(core && h);
	switch (type) {
	case USBHC_HDL_SOF:
		list_delete_element(&core->handlers.sof_list, (void *)h);
		if (core->handlers.sof_list.head == NULL) {
			/* Disable SOF callback to reduce CPU interrupts */
			usb_h_register_callback(core->hcd, USB_H_CB_SOF, NULL);
		}
		break;
	case USBHC_HDL_ENUM:
		list_delete_element(&core->handlers.enum_list, (void *)h);
		break;
	case USBHC_HDL_REQ:
		list_delete_element(&core->handlers.req_list, (void *)h);
		break;
	case USBHC_HDL_RSC:
		list_delete_element(&core->handlers.rsc_list, (void *)h);
		break;
	default:
		break;
	}
}

void usbhc_start(struct usbhc_driver *core)
{
#if CONF_USBH_MULTI_DEV_SP
	struct usbhd_driver *p;
	uint8_t              i;
#endif

	ASSERT(core);
	/* Core started */
	core->dev.status.value = USBHC_DEV_ATTACHED;

#if CONF_USBH_MULTI_DEV_SP
	/* Initialize the addresses */
	p = core->dev.next.pdev;
	for (i = 1; p; i++) {
		p->dev_addr = i;
		p           = p->next.pdev;
	}
#endif

	/* Enable hardware to start */
	usb_h_enable(core->hcd);
}

void usbhc_stop(struct usbhc_driver *core)
{
	ASSERT(core);
	/* Disable hardware to stop */
	usb_h_disable(core->hcd);
	/* Core stopped */
	core->dev.status.value = USBHC_DEV_DISCONNECT;
}

#if CONF_USBH_MULTI_DEV_SP
/**
 * \brief Modify suspend status for device or devices tree
 * \param[out] d Pointer to device instance
 * \param[in]  s Status to set
 */
static void usbhc_set_suspend(struct usbhd_driver *d, bool s)
{
	if (d->dev_type != USBHC_DEV) {
		struct hubhf_driver *hubf = d->func.phub;
		struct usbhd_driver *dev  = hubf->child.pdev;
		/* If the device is a (root) hub
		 * Set suspend to all connected things
		 */
		while (dev != d) {
			usbhc_set_suspend(dev, s);
			dev = dev->next.pdev;
		}
	}
	d->status.bm.suspend = s;
}
#endif

int32_t usbhc_suspend(struct usbhc_driver *core)
{
	/* Global suspend/resume refers to the entire bus being suspended or
	 * resumed without affecting any hub's downstream facing port states.
	 */
	ASSERT(core && core->hcd);

#if CONF_USBH_MULTI_DEV_SP
	if (core->rhfunc.child.pdev == &core->dev) {
		/* Device is not connected */
		return ERR_NOT_INITIALIZED;
	}
	usb_h_suspend(core->hcd);
	usbhc_set_suspend(&core->dev, true);
#else
	if (core->dev.hub_port == 0) {
		/* Device is not connected */
		return ERR_NOT_INITIALIZED;
	}
	usb_h_suspend(core->hcd);
	core->dev.status.value |= USBHC_DEV_SUSPENDED;
#endif
	return ERR_NONE;
}

int32_t usbhc_resume(struct usbhc_driver *core)
{
	/* Global suspend/resume refers to the entire bus being suspended or
	 * resumed without affecting any hub's downstream facing port states.
	 */
	ASSERT(core);
#if CONF_USBH_MULTI_DEV_SP
	/* Resume root hub ports
	 * All connected device status not suspend
	 */
	if (core->rhfunc.child.pdev == &core->dev) {
		/* Device is not connected */
		return ERR_NOT_INITIALIZED;
	}
	usb_h_resume(core->hcd);
	usbhc_set_suspend(&core->dev, false);
#else
	if (core->dev.hub_port == 0) {
		/* Device is not connected */
		return ERR_NOT_INITIALIZED;
	}
	usb_h_resume(core->hcd);
	core->dev.status.value &= ~USBHC_DEV_SUSPENDED;
#endif
	return ERR_NONE;
}

int32_t usbhc_reset_dev(struct usbhd_driver *dev)
{
	struct usbhd_driver *hub;
	ASSERT(dev);
	if (dev->status.bm.state < USBHC_DEV_DEFAULT) {
		return ERR_DENIED;
	}
	hub = usbhc_find_hub(dev);
	if (!hub) {
		/* Seems device not connected to any hub */
		return ERR_NOT_INITIALIZED;
	}
	/* Not usable until address is set */
	dev->status.bm.usable = 0;
	usbhc_enum_reset(usbhc_find_root(hub), hub, dev->hub_port);
	return ERR_NONE;
}

int32_t usbhc_suspend_dev(struct usbhd_driver *dev)
{
	struct usbhd_driver *hub;
	ASSERT(dev);
	if (!dev->status.bm.usable) {
		return ERR_DENIED;
	}
	hub = usbhc_find_hub(dev);
	if (!hub) {
		/* Seems device not connected to any hub */
		return ERR_NOT_INITIALIZED;
	}
#if CONF_USBH_MULTI_DEV_SP
	if (dev->dev_type != USBHC_CORE) {
		usbhc_enum_suspend(usbhc_find_root(hub), hub, dev->hub_port);
		return ERR_NONE;
	}
#endif
	return usbhc_suspend(USBHC_PTR(dev));
}

int32_t usbhc_resume_dev(struct usbhd_driver *dev)
{
	struct usbhd_driver *hub;
	ASSERT(dev);
	if (!dev->status.bm.usable) {
		return ERR_DENIED;
	}
	hub = usbhc_find_hub(dev);
	if (!hub) {
		/* Seems device not connected to any hub */
		return ERR_NOT_INITIALIZED;
	}
	if (!dev->status.bm.suspend) {
		/* Seems device not suspended */
		return ERR_NONE;
	}
#if CONF_USBH_MULTI_DEV_SP
	if (dev->dev_type != USBHC_CORE) {
		usbhc_enum_resume(usbhc_find_root(hub), hub, dev->hub_port);
		return ERR_NONE;
	}
#endif
	return usbhc_resume(USBHC_PTR(dev));
}

int32_t usbhc_request_dev(struct usbhd_driver *dev, uint8_t *req, uint8_t *data)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;

	ASSERT(dev && req);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = usbhc_request(r, req, data);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_get_dev_desc(struct usbhd_driver *dev)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;
	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_get_dev_desc(r, 18);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_get_cfg_desc(struct usbhd_driver *dev, uint8_t cfg_idx)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;
	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_get_config_desc(r, cfg_idx, dev->cfg_total);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_get_str_desc(struct usbhd_driver *dev, uint8_t str_idx, uint16_t str_id)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;

	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_get_str_desc(r, str_idx, str_id);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_set_config(struct usbhd_driver *dev, uint8_t cfg)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;

	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_set_config(r, cfg);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_set_iface(struct usbhd_driver *dev, uint8_t iface, uint8_t alt)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;

	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_set_iface(r, iface, alt);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

int32_t usbhc_get_iface(struct usbhd_driver *dev, uint8_t iface)
{
	int32_t              rc;
	struct usbhc_driver *r = NULL;

	ASSERT(dev);

	rc = usbhc_take_control(dev, &r, USBHC_USED_BY_DEV);
	if (rc == ERR_NONE) {
		rc = _usbhc_get_iface(r, iface);
	}
	if (rc != ERR_NONE) {
		usbhc_release_control(r);
	}
	return rc;
}

struct usbhc_driver *usbhc_get_dev_core(struct usbhd_driver *dev)
{
	ASSERT(dev && (dev->next.proot || !CONF_USBH_MULTI_DEV_SP));
	return usbhc_find_root(dev);
}

struct usbhd_driver *usbhc_get_func_dev(struct usbhf_driver *func)
{
	ASSERT(func);
	return func->pdev;
}

uint32_t usbhc_get_version(void)
{
	return USBHC_VERSION;
}
