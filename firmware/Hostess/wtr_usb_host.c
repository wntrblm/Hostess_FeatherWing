#include "wtr_usb_host.h"
#include "hal_delay.h"
#include "hpl_usb_host.h"
#include "usb_protocol.h"
#include "wtr_usb_defs.h"
#include "wtr_usb_requests.h"
#include "wtr_debug.h"
#include <stdio.h>

/* Macros */

#define ENUM_CHECKED(x)                                                                                                \
    do {                                                                                                               \
        int f_status = (x);                                                                                            \
        if (f_status != ERR_NONE) {                                                                                    \
            wtr_debug_printf("Enumeration error: %s returned %d at %s:%d\r\n", #x, f_status, __FILE__, __LINE__);      \
            _enum_data.failure_reason = f_status;                                                                      \
            return _handle_enumeration_event(ENUM_E_FAILED);                                                           \
        }                                                                                                              \
                                                                                                                       \
    } while (0)

/* Constants and enumerations */

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
/** Default endpoint 0 size */
#define EP0_SIZE_DEFAULT 64
/** Length of the reply buffer for control endpoint requests */
#define WTR_USB_REPLY_BUFFER_LEN 255
/** Max number of drivers */
#define WTR_USB_MAX_HOST_DRIVERS 10
/** Number of scheduled funcs allowed */
#define WTR_USB_MAX_SCHEDULED_FUNCS 16

enum enumeration_state {
    // There's no device attached.
    ENUM_S_UNATTACHED,
    // A device has just been attached and not yet reset or addressed.
    ENUM_S_ATTACHED,
    // The device was just reset.
    ENUM_S_RESET,
    // The device's descriptor has been read.
    ENUM_S_DEFAULT,
    // The device has been given an address.
    ENUM_S_ADDRESSED,
    // The device's configuration descriptor has been read and its configuration
    // has been set.
    ENUM_S_CONFIGURED,
    // The device has been enumerated and the driver has been attached.
    ENUM_S_READY,
    // Something went wrong.
    ENUM_S_FAILED,
};

enum enumeration_event {
    ENUM_E_CONNECTED,
    ENUM_E_DISCONNECTED,
    ENUM_E_RESET,
    ENUM_E_RESET_COMPLETE,
    ENUM_E_DEV_DESC_READ,
    ENUM_E_ADDRESS_SET,
    ENUM_E_ADDRESS_SET_COMPLETE,
    ENUM_E_CONF_DESC_READ,
    ENUM_E_CONFIG_SET,
    ENUM_E_FAILED,
};

/* Structs */

struct enumeration_data {
    enum enumeration_event event;
    enum enumeration_state state;
    uint8_t port;
    int32_t speed;
    uint8_t address;
    uint8_t conf_desc[WTR_USB_REPLY_BUFFER_LEN];
    int32_t failure_reason;
};

struct scheduled_func_desc {
    uint32_t delay;
    wtr_usb_scheduled_func func;
};

/* Global variables used across this module. */

static wtr_usb_connection_callback _connection_callback;
static struct usb_h_desc *_drv;
static struct usb_h_pipe *_pipe_0;
static struct enumeration_data _enum_data;
static uint8_t req_buf[64];
static uint8_t rep_buf[WTR_USB_REPLY_BUFFER_LEN];
static struct wtr_usb_host_driver _host_drivers[WTR_USB_MAX_HOST_DRIVERS];
static size_t _host_driver_count = 0;
static struct scheduled_func_desc _scheduled_funcs[WTR_USB_MAX_SCHEDULED_FUNCS];
static uint8_t _devices_connected = 0;

/* Private function forward declarations. */

static void _handle_enumeration_event(enum enumeration_event);
static void _continue_enumeration();
static int32_t _create_pipe_0();
static int32_t _handle_device_descriptor();
static int32_t _handle_config_descriptor();
static void _cleanup_enumeration();
static void _handle_start_of_frame(struct usb_h_desc *);
static void _handle_pipe_0_xfer_done(struct usb_h_pipe *);
static void _handle_root_hub_change_event(struct usb_h_desc *, uint8_t, uint8_t);

/* Public functions */

void wtr_usb_host_init(struct usb_h_desc *drv) {
    usb_h_register_callback(drv, USB_H_CB_ROOTHUB_CHANGE, (FUNC_PTR)_handle_root_hub_change_event);
    usb_h_register_callback(drv, USB_H_CB_SOF, (FUNC_PTR)_handle_start_of_frame);
    usb_h_enable(drv);
    _drv = drv;
}

void wtr_usb_host_set_connection_callback(wtr_usb_connection_callback func) { _connection_callback = func; }

void wtr_usb_host_register_driver(struct wtr_usb_host_driver driver) {
    ASSERT(_host_driver_count != WTR_USB_MAX_HOST_DRIVERS);
    _host_drivers[_host_driver_count] = driver;
    _host_driver_count++;
}

void wtr_usb_host_schedule_func(wtr_usb_scheduled_func func, uint32_t delay) {
    bool assigned = false;
    for (uint32_t i = 0; i < WTR_USB_MAX_SCHEDULED_FUNCS; i++) {
        if (_scheduled_funcs[i].func == NULL) {
            _scheduled_funcs[i].func = func;
            _scheduled_funcs[i].delay = delay;
            assigned = true;
            break;
        }
    }

    ASSERT(assigned);
}

bool wtr_usb_host_is_device_connected() { return _devices_connected > 0 ? true : false; }

/* Private method implementations */

/* Handles the USB enumeration process.

    This is the main heart of the enumeration process. It processes events from
    either root hub events or pipe 0 data transfers and steps through the
    enumeration process.
*/
static void _handle_enumeration_event(enum enumeration_event event) {
    _enum_data.event = event;

    switch (event) {
    // Device newly connected, reset its port.
    case ENUM_E_CONNECTED:
        wtr_debug_printf("Port %u connection.\r\n", _enum_data.port);
        usb_h_rh_reset(_drv, _enum_data.port);
        _enum_data.state = ENUM_S_ATTACHED;
        break;

    // Device reset, open a pipe to it and get its configuration descriptor.
    case ENUM_E_RESET:
        wtr_debug_printf("Port %u reset.\r\n", _enum_data.port);
        ENUM_CHECKED(_create_pipe_0());

        // The device needs some time to reset, so schedule a continuation.
        // This will pick up at ENUM_E_RESET_COMPLETE.
        wtr_usb_host_schedule_func(&_continue_enumeration, 150);
        break;

    case ENUM_E_RESET_COMPLETE:
        ENUM_CHECKED(wtr_usb_send_get_dev_desc_request(_pipe_0, req_buf, rep_buf, WTR_USB_REPLY_BUFFER_LEN));
        _enum_data.state = ENUM_S_RESET;
        break;

    // Device descriptor read. This one is a little complex so it is implemented
    // in
    // a helper function.
    case ENUM_E_DEV_DESC_READ:
        wtr_debug_printf("Got device descriptor reply\r\n");
        ENUM_CHECKED(_handle_device_descriptor());
        _enum_data.state = ENUM_S_DEFAULT;
        break;

    // The device descriptor has been read and the device has a real address.
    // Reconfigure the pipe to use the new address and then read the
    // configuration descriptor header.
    case ENUM_E_ADDRESS_SET:
        wtr_debug_printf("Address set\r\n");
        ENUM_CHECKED(
            usb_h_pipe_set_control_param(_pipe_0, _enum_data.address, 0, _pipe_0->max_pkt_size, _pipe_0->speed));
        // Wait a little bit before asking for the config descriptor, since some
        // devices need
        // a little bit more time. This will pick up at ENUM_E_ADDRESS_SET_COMPLETE.
        wtr_usb_host_schedule_func(&_continue_enumeration, 5);
        break;

    case ENUM_E_ADDRESS_SET_COMPLETE:
        ENUM_CHECKED(wtr_usb_send_get_conf_desc_request(_pipe_0, 0, req_buf, rep_buf, sizeof(usb_config_desc_t)));
        _enum_data.state = ENUM_S_ADDRESSED;
        break;

    // The configuration descriptor header or the full configuration descriptor
    // has been read.
    // Process that and store it and then set the configuration. This one is a
    // little complex
    // so it is implemented in a separate helper function.
    case ENUM_E_CONF_DESC_READ:
        wtr_debug_printf("Got config descriptor reply\r\n");
        ENUM_CHECKED(_handle_config_descriptor());
        _enum_data.state = ENUM_S_CONFIGURED;
        break;

    case ENUM_E_CONFIG_SET:
        wtr_debug_printf("Enumeration successful.\r\n");
        _enum_data.state = ENUM_S_READY;

        // Remove the callback for pipe 0. If the driver uses it,
        // we don't want our callback to be invoked anymore.
        usb_h_pipe_register_callback(_pipe_0, NULL);

        // Find a driver for the device.
        bool driver_found = false;
        for (size_t i = 0; i < WTR_USB_MAX_HOST_DRIVERS; i++) {
            struct wtr_usb_host_driver *driver = &_host_drivers[i];
            if (driver->enumeration_callback == NULL)
                break;

            int32_t result =
                driver->enumeration_callback(_pipe_0, USB_STRUCT_PTR(usb_config_desc, &_enum_data.conf_desc));

            if (result == WTR_USB_HD_STATUS_SUCCESS) {
                driver_found = true;
                break;
            }

            // TODO: check for WTR_USB_HD_STATUS_FAILED, make sure ctrl_pipe gets cleaned up.
        }

        // Did we find a driver for the device? if so, the driver is responsible for
        // pipe 0, and should free it if its
        // not needed. Otherwise it'll get handled by _cleanup_enumeration.
        // TODO: De-address the device or otherwise disable it???
        if (driver_found)
            _pipe_0 = NULL;
        _cleanup_enumeration();

        _devices_connected++;
        if (_connection_callback != NULL)
            _connection_callback(_enum_data.port, true);
        break;

    // Device disconnected. Teardown any enumeration resources and notify the
    // device driver.
    case ENUM_E_DISCONNECTED:
        wtr_debug_printf("Port %u disconnection.\r\n", _enum_data.port);
        _enum_data.state = ENUM_S_UNATTACHED;
        for (size_t i = 0; i < WTR_USB_MAX_HOST_DRIVERS; i++) {
            struct wtr_usb_host_driver *driver = &_host_drivers[i];
            if (driver->disconnection_callback == NULL)
                break;
            driver->disconnection_callback(_enum_data.port);
        }
        _cleanup_enumeration();

        _devices_connected--;
        if (_connection_callback != NULL)
            _connection_callback(_enum_data.port, false);
        break;

    case ENUM_E_FAILED:
        wtr_debug_printf("Enumeration failed in state %i, reason: %li.\r\n", _enum_data.state, _enum_data.failure_reason);
        _enum_data.state = ENUM_S_FAILED;
        // TODO: Notify user code.
        _cleanup_enumeration();
        break;
    }
}

/* This is used when a delay is needed in the enumeration process.
Instead of calling "delay_ms", it uses the start-of-frame callback to trigger a
function
call after a certain amount of time. This function simply goes to the next
enumeration
event.
*/
static void _continue_enumeration() { _handle_enumeration_event(_enum_data.event + 1); }

static int32_t _create_pipe_0() {
    _pipe_0 = usb_h_pipe_allocate(_drv, 0, 0, EP0_SIZE_DEFAULT, 0, 0, _enum_data.speed, true);

    if (_pipe_0 == NULL) {
        return ERR_NO_RESOURCE;
    }

    usb_h_pipe_register_callback(_pipe_0, _handle_pipe_0_xfer_done);

    return ERR_NONE;
}

/* Handles the device descriptor response
    Make sure we have the whole descriptor and then set the address.
*/
static int32_t _handle_device_descriptor() {
    int32_t request_status = 0;

    if (_pipe_0->x.ctrl.count < sizeof(usb_dev_desc_t)) {
        // Update max packet size and re-request the descriptor.
        // Max packet size is byte 7 in the device descriptor.
        uint8_t bMaxPackSize0 = rep_buf[7];

        if (bMaxPackSize0 > WTR_USB_REPLY_BUFFER_LEN) {
            return ERR_OVERFLOW;
        }

        request_status =
            usb_h_pipe_set_control_param(_pipe_0, _pipe_0->dev, _pipe_0->ep, bMaxPackSize0, _pipe_0->speed);

        if (request_status != ERR_NONE) {
            wtr_debug_printf("Failed to adjust endpoint packet size, status: %lu\r\n", request_status);
            return request_status;
        }

        wtr_debug_printf("Requesting full device descriptor.\r\n");

        return wtr_usb_send_get_dev_desc_request(_pipe_0, req_buf, rep_buf, sizeof(usb_dev_desc_t));
    }

    struct usb_dev_desc *dev_desc = USB_STRUCT_PTR(usb_dev_desc, rep_buf);

    wtr_debug_printf("USB Descriptor:\r\n");
    wtr_debug_printf("\tLength: %u\r\n", dev_desc->bLength);
    wtr_debug_printf("\tType: %u\r\n", dev_desc->bDescriptorType);
    wtr_debug_printf("\tClass: %u\r\n", dev_desc->bDeviceClass);
    wtr_debug_printf("\tSublass: %u\r\n", dev_desc->bDeviceSubClass);
    wtr_debug_printf("\tProtocol: %u\r\n", dev_desc->bDeviceProtocol);
    wtr_debug_printf("\tEP0 max packet size: %u\r\n", dev_desc->bMaxPacketSize0);
    wtr_debug_printf("\tVendor: %x\r\n", dev_desc->idVendor);
    wtr_debug_printf("\tProduct: %x\r\n", dev_desc->idProduct);
    wtr_debug_printf("\tNum configurations: %u\r\n", dev_desc->bNumConfigurations);

    wtr_debug_printf("Assigning address 1\r\n");

    // TODO: Address assignment
    _enum_data.address = 1;

    return wtr_usb_send_set_address_request(_pipe_0, _enum_data.address, req_buf);
}

static int32_t _handle_config_descriptor() {
    struct usb_config_desc *conf_desc = USB_STRUCT_PTR(usb_config_desc, rep_buf);

    // We didn't get the whole descriptor header, fail.
    if (_pipe_0->x.ctrl.count < sizeof(struct usb_config_desc)) {
        wtr_debug_printf("Enumeration failed, config descriptor is not the right size.\r\n");
        return ERR_WRONG_LENGTH;
    }

    // We got the descriptor header, but need to request the full descriptor.
    if (_pipe_0->x.ctrl.count == sizeof(struct usb_config_desc)) {
        uint16_t descriptor_length = conf_desc->wTotalLength;

        if (descriptor_length > WTR_USB_REPLY_BUFFER_LEN) {
            return ERR_OVERFLOW;
        }

        wtr_debug_printf("Requesting full config descriptor.\r\n");

        return wtr_usb_send_get_conf_desc_request(_pipe_0, 0, req_buf, rep_buf, descriptor_length);
    }

    wtr_debug_printf("Got full descriptor. Length: %u\r\n", conf_desc->wTotalLength);

    memcpy(_enum_data.conf_desc, (uint8_t *)conf_desc, conf_desc->wTotalLength);

    wtr_debug_printf("Setting configuration\r\n");
    return wtr_usb_send_set_config_request(_pipe_0, 1, req_buf);
}

static void _cleanup_enumeration() {
    if (_pipe_0 != NULL) {
        usb_h_pipe_free(_pipe_0);
        _pipe_0 = NULL;
    }

    _enum_data = (const struct enumeration_data){0};
}

/* HAL -> wtr adapters and such */

/* Start of frame callback.
Mostly used for timing and queueing later requests and such.
*/
static void _handle_start_of_frame(struct usb_h_desc *drv) {
    // Go through each scheduled func and count down their delay.
    // if it hits zero, trigger the func and clear it.
    for (uint32_t i = 0; i < WTR_USB_MAX_SCHEDULED_FUNCS; i++) {
        if (_scheduled_funcs[i].func == NULL)
            continue;
        _scheduled_funcs[i].delay--;
        if (_scheduled_funcs[i].delay != 0)
            continue;

        _scheduled_funcs[i].func();
        _scheduled_funcs[i].func = NULL;
    }

    // Call any driver start-of-frame callbacks.
    for (uint32_t i = 0; i < _host_driver_count; i++) {
        if (_host_drivers[i].sof_callback == NULL)
            continue;
        _host_drivers[i].sof_callback();
    }
}

/* Handles root hub events from the usb host HAL.
    Triggers the appropriate enumeration event. - The goal here is to adapt from
   their
    interface to ours as quickly as possible.
*/
static void _handle_root_hub_change_event(struct usb_h_desc *drv, uint8_t port, uint8_t ftr) {
    // TODO: Make sure the port matches the enumeration data port.
    int32_t status;
    switch (ftr) {
    case PORT_CONNECTION:
        _enum_data.port = port;
        status = usb_h_rh_check_status(drv, port, ftr);
        if (status == 1) {
            _handle_enumeration_event(ENUM_E_CONNECTED);
        } else {
            _handle_enumeration_event(ENUM_E_DISCONNECTED);
        }
        return;
    case PORT_RESET:
        status = usb_h_rh_check_status(drv, port, PORT_LOW_SPEED) ? USB_SPEED_LS : USB_SPEED_FS;
        _enum_data.speed = status;
        _handle_enumeration_event(ENUM_E_RESET);
        return;
    // Not implemented yet.
    case PORT_SUSPEND:
        return;
    // Not implemented by the root hub, instead, the vbus handler deals with this.
    // However, this event but may be implemented in downstream hubs.
    case PORT_OVER_CURRENT:
        wtr_debug_printf("Port overcurrent. Port: %u\r\n", port);
        return;
    default:
        wtr_debug_printf("Unknown port change event: %u\r\n", ftr);
        return;
    }
}

/* Handles pipe 0 transfers from the usb host HAL.
    Triggers the appropriate enumeration event - The goal here is to adapt from
   their
    interface to ours as quickly as possible.
*/
static void _handle_pipe_0_xfer_done(struct usb_h_pipe *pipe) {
    struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
    uint16_t req_type = req->wValue >> 8;

    wtr_debug_printf("Pipe 0 transfer done. Request: %u, Status: %i\r\n", req->bRequest, pipe->x.general.status);

    if (pipe->x.general.status < 0) {
        // TODO: Idk, do something about this error?
        return;
    }

    /* Enumeration process */
    switch (req->bRequest) {
    case USB_REQ_GET_DESC:
        if (req_type == USB_DT_DEVICE) {
            _handle_enumeration_event(ENUM_E_DEV_DESC_READ);
        } else if (req_type == USB_DT_CONFIG) {
            _handle_enumeration_event(ENUM_E_CONF_DESC_READ);
        }
        break;
    case USB_REQ_SET_ADDRESS:
        _handle_enumeration_event(ENUM_E_ADDRESS_SET);
        break;

    case USB_REQ_SET_CONFIG:
        _handle_enumeration_event(ENUM_E_CONFIG_SET);
        break;

    default:
        wtr_debug_printf("Unknown request: %u\r\n", req->bRequest);
        return;
        break;
    }
}