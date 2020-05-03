/**
 * \file
 *
 * \brief USB Host Stack Core Driver Definition.
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

#ifndef _USB_USBHC_H_
#define _USB_USBHC_H_

#include "usb_includes.h"
#include "usb_protocol.h"
#include "hal_usb_host.h"

#include "usbh_config.h"

#if !CONF_USB_H_INST_OWNER_SP
#error HAL driver and pipe owner must be supported
#endif

#if CONF_USBH_HUB_SP && !CONF_USBH_MULTI_DEV_SP
#error Multiple device support should be enabled for hub support
#endif

/**
 * \brief Enumeration status.
 */
typedef enum usbhc_enum_status {
	/* Status applied to a device */

	/** The supported USB device/function has been enabled.
	 *  enum_cb(USBHC_ENUM_SUCCESS, usbhc, usbhd)
	 */
	USBHC_ENUM_SUCCESS = 0,
	/** The USB device/function has been disconnected.
	 *  enum_cb(USBHC_ENUM_DISCONNECTED, usbhc, usbhd)
	 */
	USBHC_ENUM_DISCONNECTED,
	/** Device power is not supported.
	 *  enum_cb(USBHC_ENUM_OVERCURRENT, usbhc, usbhd)
	 */
	USBHC_ENUM_OVERCURRENT,
	/** None of the device function is supported by the drivers.
	 *  enum_cb(USBHC_ENUM_UNSUPPORTED, usbhc, usbhd)
	 */
	USBHC_ENUM_UNSUPPORTED,
	/** USB software can not support it. Not enough memory for total
	 *  configuration descriptor.
	 *  enum_cb(USBHC_ENUM_MEM_LIMIT, usbhc, usbhd)
	 */
	USBHC_ENUM_MEM_LIMIT,
	/** A problem occurred during USB enumeration.
	 *  enum_cb(USBHC_ENUM_FAIL, usbhc, usbhd)
	 */
	USBHC_ENUM_FAIL,
	/** A suspend status change on USB device
	 *  enum_cb(USBHC_ENUM_SUSPEND_CHG, usbhc, usbhd)
	 */
	USBHC_ENUM_SUSPEND_CHG,

	/** Status applied to IO, device will be uninstalled
	 *  enum_cb(USBHC_ENUM_IO_FAIL, usbhc, usbhd)
	 */
	USBHC_ENUM_IO_FAIL,

	/* Status applied to driver */

	/** USB software can not support it. Not enough device driver.
	 *  enum_cb(USBHC_ENUM_DEVD_LIMIT, usbhc, dev_desc)
	 */
	USBHC_ENUM_DEVD_LIMIT,

	/** USB driver can not support it (allocate or change pipe fail).
	 *  enum_cb(USBHC_ENUM_HAL_LIMIT, usbhc, p_desc)
	 */
	USBHC_ENUM_HAL_LIMIT,

	/* Status related to descriptor handling */

	/** USB software can not support it. No suitable function driver.
	 *  enum_cb(USBHC_ENUM_IFACE_UNSUPPORTED, usbhc, if_desc)
	 */
	USBHC_ENUM_IFACE_UNSUPPORTED
} usbhc_enum_status_t;

/**
 * \brief Device status
 *
 * Applied to value of status code.
 */
typedef enum usbhc_dev_status {
	/* State that device is not accessible */

	/** Device has not connected.*/
	USBHC_DEV_DISCONNECT = 0,
	/** Device connected but OFF. */
	USBHC_DEV_OFF = 1,
	/** Device is attached to the USB and powered, but has
	 *  not been reset.
	 */
	USBHC_DEV_ATTACHED = 2,
	/** Device is attached to the USB and powered and has
	 *  been reset, but has not been assigned a unique address.
	 */
	USBHC_DEV_DEFAULT = 3,
	/** Device is attached, but fail to work for some errors. */
	USBHC_DEV_FAILED = 4,

	/* State that device is usable outside of core */

	/** Device is attached to the USB, powered, has been
	 *  reset, and a unique device address has been assigned.
	 */
	USBHC_DEV_ADDRESS = 0x5,
	/** Device is attached to the USB, powered, has been reset,
	 *  has a unique address, is configured
	 */
	USBHC_DEV_CONFIGURED = 0x6,

	/** Device is usable outside of core */
	USBHC_DEV_USABLE = 0x10,

	/** Device is suspended */
	USBHC_DEV_SUSPENDED = 0x20,

	/** Device is removing */
	USBHC_DEV_REMOVING = 0xFF
} usbhc_dev_status_t;

/**
 * \brief Status code of device driver
 */
union usbhd_status {
	/** Status code value */
	uint8_t value;
	struct {
		/** \ref usbhc_dev_status code (<0x10) */
		uint8_t state : 4;
		/** Usable, can be used outside of core */
		uint8_t usable : 1;
		/** Suspend indicator, suspended */
		uint8_t suspend : 1;
		/** Reserved */
		uint8_t reserved : 2;
	} bm;
};

/** USB Host core device type. */
typedef enum usbhc_dev_type {
	/** Core device or root device */
	USBHC_CORE = 0,
	/** Core device or root device */
	USBHC_ROOT = 0,
	/** General USB device */
	USBHC_DEV = 1,
	/** At least one of device interface is hub */
	USBHC_HUB = 2
} usbhc_dev_type_t;

/** USB host core handler type. */
enum usbhc_handler_type {
	/* Called for each SOF each 1ms */
	USBHC_HDL_SOF,
	/* Called when device enumeration success or fail */
	USBHC_HDL_ENUM,
	/* Called when class request has been done */
	USBHC_HDL_REQ,
	/* Called when control resource is available */
	USBHC_HDL_RSC
};

/**
 * \brief USB Host function control code
 */
enum usbhf_control {
	/** Install the function driver based on the very first interface.
	 *  int32_t ctrl(usbhf, USBHF_INSTALL, struct usbh_descriptors *desc);
	 *  Return code:
	 *  - ERR_NONE      - installed;
	 *  - ERR_NO_RESOURCE - install error;
	 *  - ERR_NOT_FOUND - not installed;
	 *  - ERR_NO_CHANGE - already taken.
	 *  After installation, the desc (desc->sod) MUST be updated to SKIP the
	 *  descriptors that are taken by function driver.
	 *
	 *  ONLY very first ONE interface should be checked.
	 */
	USBHF_INSTALL,
	/** Enable the function driver.
	 *  void ctrl(usbhf, USBHF_ENABLE, NULL);
	 */
	USBHF_ENABLE,
	/** Un-install the function driver.
	 *  void ctrl(usbhf, USBHF_UNINSTALL, NULL);
	 */
	USBHF_UNINSTALL
};

/**
 * \brief Describes a list of USB descriptors.
 */
struct usbh_descriptors {
	/** Pointer to Start of Descriptors. */
	uint8_t *sod;
	/** Pointer to End of Descriptors (next to the last byte). */
	uint8_t *eod;
};

/**
 * \brief Describes a list of core handler descriptor
 */
struct usbhc_handler {
	/** Pointer to next handler. */
	struct usbhc_handler *next;
	/** Pointer to handler function. */
	FUNC_PTR func;
};

/** Cast pointer to handler */
#define USBHC_HDL_PTR(p) ((struct usbhc_handler *)(p))

/**
 * \brief Describes a list of core handler descriptor with extension
 */
struct usbhc_handler_ext {
	/** Pointer to next handler. */
	struct usbhc_handler *next;
	/** Pointer to handler function. */
	FUNC_PTR func;
	/** Pointer or parameter for extension. */
	void *ext;
};

/** Cast pointer to extended handler */
#define USBHC_HDL_EXT_PTR(p) ((struct usbhc_handler_ext *)(p))

/**
 * \brief USB Host Core Handler
 */
struct usbhc_handlers {
	/** List header of SOF notification handlers */
	struct list_descriptor sof_list;
	/** List header of ENUM status change handlers */
	struct list_descriptor enum_list;
	/** List header of request end handlers */
	struct list_descriptor req_list;
	/** List header of resource wait handlers */
	struct list_descriptor rsc_list;
};

/** USB host function driver */
struct usbhf_driver;
/** USB host hub function driver */
struct hubhf_driver;

/** USB host core (root) driver */
struct usbhc_driver;
/** USB host device driver */
struct usbhd_driver;

/** SOF callback function. */
typedef void (*usbhc_sof_cb_t)(struct usbhc_driver *, struct usbhc_handler_ext *);

/** Enumeration callback function. */
typedef void (*usbhc_enum_cb_t)(struct usbhc_driver *, int32_t status, void *param);

/** Resource wait callback function */
typedef void (*usbhc_rsc_cb_t)(struct usbhc_driver *, struct usbhc_handler_ext *);

/** Request callback function. */
typedef int32_t (*usbhc_req_cb_t)(struct usbhd_driver *, struct usb_h_pipe *);

/** Control function for USB device general function driver. */
typedef int32_t (*usbhf_control_cb_t)(struct usbhf_driver *func, enum usbhf_control ctrl, void *param);

/**
 * \brief General pointer for a device
 */
union usbhd_ptr {
	/** Pointer to device */
	struct usbhd_driver *pdev;
	/** Pointer to core driver */
	struct usbhc_driver *proot;
	/** List header */
	struct list_descriptor list;
};

/**
 * \brief General pointer for a function
 */
union usbhf_ptr {
	/** Pointer to function */
	struct usbhf_driver *pfunc;
	/** Pointer to hub function */
	struct hubhf_driver *phub;
	/** List header */
	struct list_descriptor list;
};

/**
 * \brief Structure of device driver
 */
struct usbhd_driver {
	/** Next function driver.
	 *  Points to next device that attached on same hub.
	 *  For root device, it's free device list if multiple device is supported.
	 *  For root device, it's pointer to device if only one device is supported.
	 */
	union usbhd_ptr next;
	/** Installed function drivers.
	 *  Pointers to the first function that is supported on the device.
	 *  For root device, it's free function list.
	 */
	union usbhf_ptr func;
	/** Current configuration index */
	uint8_t cfg_idx;
	/** Power consumption of current configuration (x2mA) */
	uint8_t cfg_power;
	/** Total length of configuration descriptor */
	uint16_t cfg_total;
	/** EP0 size */
	uint8_t ep0_size;
	/** Device address */
	uint8_t dev_addr;
	/** Parent hub port */
	uint8_t hub_port;
	/** Device type: ROOT/HUB/DEV
	 *  A hub is enumerated to device and change to hub when hub function
	 *  is enabled.
	 */
	uint8_t dev_type : 2;
	/** Working speed */
	uint8_t speed : 2;
	/** Level isolate, for nearby layers.
	 *  E.g., Hub level is 0, it's children have level 1.
	 */
	uint8_t level : 1;
	/** Reserved bits */
	uint8_t reserved : 3;
	/** State status code */
	volatile union usbhd_status status;
};

/** Get pointer to device driver */
#define USBHD_PTR(p) ((struct usbhd_driver *)(p))

/**
 * \brief Structure of Function driver
 */
struct usbhf_driver {
	/** Pointer to next function driver */
	union usbhf_ptr next;
	/** Pointer to control function */
	usbhf_control_cb_t ctrl;
	/** Pointer to the device */
	struct usbhd_driver *pdev;
};

/** Get pointer to function driver */
#define USBHF_PTR(p) ((struct usbhf_driver *)(p))

/**
 * \brief Structure of Hub function driver
 * hubf_driver::func::next @ 0
 * hubf_driver::child      @ +3x ptr
 */
struct hubhf_driver {
	/** General function driver instance (3 x ptr) */
	struct usbhf_driver func;
#if CONF_USBH_MULTI_DEV_SP
	/** Pointer to first device driver that attached on this hub (1 x ptr) */
	union usbhd_ptr child;
#endif
	/** Interrupt IN pipe (1 x ptr) */
	struct usb_h_pipe *pipe_in;
	/** Interface number */
	uint8_t iface;
	/** Number of ports */
	uint8_t n_ports;
	/** Reserve for align */
	uint8_t reserved[2];
};

/** Get pointer to hub driver */
#define HUBHF_PTR(p) ((struct hubhf_driver *)(p))

/**
 * \brief Structure of Root Hub function driver
 *
 * Keep next function driver, child pointer (if exist) and power
 * aligned with hub function driver.
 *
 * roothubhf_driver::func_list  @ 0
 * roothubhf_driver::child      @ +3x ptr
 */
struct roothubhf_driver {
	/** A list of function drivers (1x ptr).
	 *  For multiple device support, it's free function drivers.
	 *  For single device support, it's registered function drivers.
	 */
	union usbhf_ptr func_list;
	/** Current enumerating device (1x ptr). */
	struct usbhd_driver *enum_dev;
	/** Pointer to EP0 control pipe instance (1x ptr).
	 *  Control callback function is no need for installation.
	 */
	struct usb_h_pipe *pipe_0;
#if CONF_USBH_MULTI_DEV_SP
	/** Pointer to first device driver that attached on this root hub (1x ptr) */
	union usbhd_ptr child;
#endif
	/** Power consumption (x2mA) */
	uint16_t power;
	/** Number of ports */
	uint8_t n_ports;
	/** Reserve for align */
	uint8_t reserved;
};

/** Get pointer to hub driver */
#define ROOTHUBHF_PTR(p) ((struct rhubhf_driver *)(p))

/**
 * \brief Control buffer and pipe status
 */
union usbhc_ctrl_status {
	uint8_t value;
	struct {
		/** Control resource state, see \ref usbhc_ctrl_state */
		uint8_t ctrl_state : 2;
		/** Pending enumeration */
		uint8_t pending_enum : 1;
		/** Pending hub changes clear */
		uint8_t pending_hub : 1;
	} bm;
};

/**
 * \brief Structure of USB Host Core driver
 */
struct usbhc_driver {
	/** Device driver instance
	 *  dev.next -> First free driver
	 *  dev.dev_addr = 0
	 *  dev.ep0_size = max possible for control endpoints
	 */
	struct usbhd_driver dev;
	/** USB Host Controller Driver instance */
	struct usb_h_desc *hcd;
	/** USB Host Root Hub Function Driver instance */
	struct roothubhf_driver rhfunc;
	/** Callback handlers */
	struct usbhc_handlers handlers;
	/** Shared control buffer */
	uint8_t *ctrl_buf;
	/** Shared control buffer size */
	uint16_t ctrl_buf_size;
	/** Control resource status */
	union usbhc_ctrl_status ctrl_status;
	/** Enum request start delay */
	uint8_t enum_delay;
};

/** Get pointer to root device driver */
#define USBHC_PTR(p) ((struct usbhc_driver *)(p))

/**
 * \brief Shared control buffer structure
 *
 * Request buffer + [VID + PID (if support vendor class)] + descriptor buffer.
 */
struct usbhc_ctrl_buf {
	/** Last request sent is at first 8 bytes of shared buffer */
	struct usb_req req;
#if CONF_USBH_VENDOR_DEV_SP
	/** Vendor ID */
	uint16_t vid;
	/** Product ID */
	uint16_t pid;
#endif
	/** Access point of last data returned by USB requests */
	uint8_t data[4];
};

/**
 * \brief Control buffer and pipe status state
 */
enum usbhc_ctrl_state {
	/** Control resources are idle */
	USBHC_IDLE,
	/** Control resources are used by core */
	USBHC_USED_BY_CORE,
	/** Control resources are used by enumerated device */
	USBHC_USED_BY_DEV
};

/** Get pointer to control buffer structure instance */
#define USBHC_CTRL_BUF_PTR(p) ((struct usbhc_ctrl_buf *)(p))

/** Control request offset in shared control buffer */
#define USBHC_CTRL_REQ_OFFSET (0)

#if CONF_USBH_VENDOR_DEV_SP

/** Vendor ID offset in shared control buffer */
#define USBHC_CTRL_VID_OFFSET (sizeof(struct usb_req))
/** Product ID offset in shared control buffer */
#define USBHC_CTRL_PID_OFFSET (USBHC_CTRL_VID_OFFSET + 2)
/** Request data offset in shared control buffer */
#define USBHC_CTRL_DATA_OFFSET (USBHC_CTRL_PID_OFFSET + 2)
#else

/** Request data offset in shared control buffer */
#define USBHC_CTRL_DATA_OFFSET (sizeof(struct usb_req))
#endif

/**
 * \brief Initialize the USB host core driver
 * \param core Pointer to instance of core driver
 * \return Operation status.
 */
int32_t usbhc_init(struct usbhc_driver *core, struct usb_h_desc *hcd, uint8_t *buf, uint16_t buf_size);

/**
 * \brief Deinitialize the USB host core driver
 * \param core Pointer to instance of core driver
 * \return Operation status.
 */
int32_t usbhc_deinit(struct usbhc_driver *core);

/**
 * \brief Get access pointer to the shared control buffer
 * \param[in] core Pointer to instance of core driver
 * \return Pointer to byte access of control buffer
 */
static inline uint8_t *usbhc_get_ctrl_buf(struct usbhc_driver *core)
{
	return core->ctrl_buf;
}

#if CONF_USBH_MULTI_DEV_SP
/**
 * \brief Register device driver
 * \param     core  Pointer to instance of core driver
 * \param[in] devd Pointer to instance of device driver
 * \return Operation status code
 */
int32_t usbhc_register_devd(struct usbhc_driver *core, struct usbhd_driver *devd);

/**
 * \brief Unregister device driver
 * \param     core  Pointer to instance of core driver
 * \param[in] devd Pointer to instance of device driver
 * \return Operation status code
 */
int32_t usbhc_unregister_devd(struct usbhc_driver *core, struct usbhd_driver *devd);
#endif

/**
 * \brief Register function driver
 * \param     core  Pointer to instance of core driver
 * \param[in] funcd Pointer to instance of function driver
 * \return Operation status code
 */
int32_t usbhc_register_funcd(struct usbhc_driver *core, struct usbhf_driver *funcd);

/**
 * \brief Unregister function driver
 * \param     core  Pointer to instance of core driver
 * \param[in] funcd Pointer to instance of function driver
 * \return Operation status code
 */
int32_t usbhc_unregister_funcd(struct usbhc_driver *core, struct usbhf_driver *funcd);

/**
 * \brief Register the handler
 * \param     core Pointer to instance of core driver
 * \param[in] type USB host core handler type.
 * \param[in] h    Pointer to USB host core handler.
 */
void usbhc_register_handler(struct usbhc_driver *core, enum usbhc_handler_type type, const struct usbhc_handler *h);

/**
 * \brief Unregister the handler
 * \param     core Pointer to instance of core driver
 * \param[in] type USB host core handler type.
 * \param[in] h    Pointer to USB host core handler.
 */
void usbhc_unregister_handler(struct usbhc_driver *core, enum usbhc_handler_type type, const struct usbhc_handler *h);

/**
 * \brief Starts the host mode
 * \param core Pointer to instance of core driver
 * \return Operation status code
 */
void usbhc_start(struct usbhc_driver *core);

/**
 * \brief Stops the host mode
 * \param core Pointer to instance of core driver
 * \return Operation status code
 */
void usbhc_stop(struct usbhc_driver *core);

/**
 * \brief Suspend the bus
 * \param core Pointer to instance of core driver
 * \return Operation status code
 */
int32_t usbhc_suspend(struct usbhc_driver *core);

/**
 * \brief Resume the bus
 * \param core Pointer to instance of core driver
 * \return Operation status code
 */
int32_t usbhc_resume(struct usbhc_driver *core);

/* USB device operations */

/**
 * \brief Check if the device is usable
 * \param[in] dev Pointer to instance of device driver
 * \return \c true if device is usable
 */
static inline bool usbhc_is_dev_usable(struct usbhd_driver *dev)
{
	ASSERT(dev);
	return dev->status.bm.usable;
}

/**
 * \brief Check if the device is suspended
 * \param[in] dev Pointer to instance of device driver
 * \return \c true if device is suspended
 */
static inline bool usbhc_is_dev_suspend(struct usbhd_driver *dev)
{
	ASSERT(dev);
	return dev->status.bm.suspend;
}

/**
 * \brief Return device state
 * \param[in] dev Pointer to instance of device driver
 * \return Device state
 */
static inline bool usbhc_get_dev_state(struct usbhd_driver *dev)
{
	ASSERT(dev);
	return dev->status.bm.state;
}

/**
 * \brief Suspend the device
 * \param dev Pointer to instance of device driver
 * \return Operation status code
 */
int32_t usbhc_suspend_dev(struct usbhd_driver *dev);

/**
 * \brief Resume the device
 * \param dev Pointer to instance of device driver
 * \return Operation status code
 */
int32_t usbhc_resume_dev(struct usbhd_driver *dev);

/**
 * \brief Reset the device
 * \param dev Pointer to instance of device driver
 * \return Operation status code
 */
int32_t usbhc_reset_dev(struct usbhd_driver *dev);

/**
 * \brief Issue a control request on EP0 to a configured device
 *
 * \param         dev  Pointer to instance of device driver
 * \param[in]     req  Pointer to USB request instance, the contents inside should
 *                     keep available before end of control transfer
 * \param[in,out] data Pointer to a memory block for data input/output
 *
 * \return Operation status code
 */
int32_t usbhc_request_dev(struct usbhd_driver *dev, uint8_t *req, uint8_t *data);

/**
 * \brief Get Device Descriptor of a configured device
 *
 * Issue GetDeviceDescriptor on a configured device, request callback
 * will then happen when descriptor is returned in shared control
 * buffer.
 *
 * \param dev Pointer to instance of device driver
 *
 * \return Operation status code
 */
int32_t usbhc_get_dev_desc(struct usbhd_driver *dev);

/**
 * \brief Get Configuration Descriptor of a configured device
 *
 * Issue GetConfigurationDescriptor on a configured device, request callback
 * will then happen when descriptors are returned in shared control buffer.
 *
 * \param     dev     Pointer to instance of device driver
 * \param[in] cfg_idx Configuration index
 *
 * \return Operation status code
 */
int32_t usbhc_get_cfg_desc(struct usbhd_driver *dev, uint8_t cfg_idx);

/**
 * \brief Get String Descriptor on a configured device
 *
 * Issue GetStringDescriptor on a configured device, request callback
 * will then happen when descriptor is returned in shared control buffer.
 *
 * \param     dev   Pointer to instance of device driver
 * \param[in] str_idx String index
 * \param[in] str_id  String ID code
 *
 * \return Operation status code
 */
int32_t usbhc_get_str_desc(struct usbhd_driver *dev, uint8_t str_idx, uint16_t str_id);

/**
 * \brief Set Configure on a usable device
 *
 * Issue SetConfigure on a usable device, request callback
 * will then happen when request status is done.
 *
 * \param     dev Pointer to instance of device driver
 * \param[in] cfg Configure value
 *
 * \return Operation status code
 */
int32_t usbhc_set_config(struct usbhd_driver *dev, uint8_t cfg);

/**
 * \brief Get Alternate Interface on a configured device
 *
 * Issue GetInterface on a configured device, request callback
 * will then happen when descriptor is returned in shared control buffer.
 *
 * \param     dev   Pointer to instance of device driver
 * \param[in] iface Interface number
 *
 * \return Operation status code
 */
int32_t usbhc_get_iface(struct usbhd_driver *dev, uint8_t iface);

/**
 * \brief Set Alternate Interface on a configured device
 *
 * Issue SetInterface on a configured device, request callback
 * will then happen when descriptor is returned in shared control buffer.
 *
 * \param     dev   Pointer to instance of device driver
 * \param[in] iface Interface number
 * \param[in] alt   Alternate setting
 *
 * \return Operation status code
 */
int32_t usbhc_set_iface(struct usbhd_driver *dev, uint8_t iface, uint8_t alt);

/**
 * \brief Get Core that the device attached to
 * \param[in] dev Pointer to instance of device driver
 * \return Pointer to instance of core driver
 */
struct usbhc_driver *usbhc_get_dev_core(struct usbhd_driver *dev);

/**
 * \brief Get Device that using the function
 * \param[in] func Pointer to instance of function driver
 * \return Pointer to instance of device driver
 */
struct usbhd_driver *usbhc_get_func_dev(struct usbhf_driver *func);

/**
 * \internal
 * \brief Take the control resources on root device for requests
 *
 * The resources are released after request done.
 *
 * \param[in]  d Pointer to device instance
 * \param[out] r Pointer to fill a pointer to root device instance
 * \param[in]  s The target user of the control resource
 */
int32_t usbhc_take_control(struct usbhd_driver *d, struct usbhc_driver **r, const enum usbhc_ctrl_state s);

/**
 * \internal
 * \brief Issue a setup request
 * EP0 usage not checked here, \ref usbhc_take_control should be invoked
 * to take EP0 and control buffer resource.
 * \param[in]     r    Pointer to root device instance
 * \param[in]     req  Pointer to request
 * \param[in,out] data Pointer to request data
 * \return Request transfer status
 */
int32_t usbhc_request(struct usbhc_driver *r, uint8_t *req, uint8_t *data);

/**
 * \internal
 * \brief Release the control resources on root device
 * \param[in] d   Pointer to device instance
 * \param     r   Pointer to root device instance
 */
void usbhc_release_control(struct usbhc_driver *r);

/**
 * \brief Return version
 */
uint32_t usbhc_get_version(void);

#endif /* USBHC_H_ */
