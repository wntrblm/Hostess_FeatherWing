#include "wtr_ps4_host_driver.h"
#include "usb_protocol.h"
#include "usb_protocol_hid.h"
#include "wtr_usb_host.h"
#include "wtr_utils.h"
#include "wtr_debug.h"
#include <stdio.h>

/* Constants */

/* Global variables */

static struct usb_h_pipe *_out_pipe;
static uint8_t _out_pipe_buf[64];
static struct usb_h_pipe *_in_pipe;
static uint8_t _in_pipe_buf[64];       // TODO make this a constant
static uint8_t _in_pipe_buf_prev[64];  // TODO make this a constant
// static hid_kbd_input_report_t _report;
// static hid_kbd_input_report_t _prev_report;
// static struct hid_keyboard_event event_queue_buf[sizeof(struct hid_keyboard_event) * EVENT_QUEUE_SIZE];
// static struct wtr_queue event_queue;

/* Private function forward declarations */

static int32_t _handle_enumeration(struct usb_h_pipe *, struct usb_config_desc *);
static int32_t _handle_disconnection(uint8_t port);
static void _poll();
static void _handle_pipe_in(struct usb_h_pipe *pipe);
static void _handle_pipe_out(struct usb_h_pipe *pipe);
static void _handle_ctrl_in(struct usb_h_pipe *pipe);
static void _handle_report();

/* Public functions */

void wtr_ps4_controller_init() {
    // event_queue.data = (uint8_t *)event_queue_buf;
    // event_queue.item_size = sizeof(struct hid_keyboard_event);
    // event_queue.capacity = EVENT_QUEUE_SIZE;
    // wtr_queue_init(&event_queue);

    struct wtr_usb_host_driver driver = {};
    driver.enumeration_callback = &_handle_enumeration;
    driver.disconnection_callback = &_handle_disconnection;

    wtr_usb_host_register_driver(driver);
};

/* Private functions */

static int32_t _handle_enumeration(struct usb_h_pipe *pipe_0, struct usb_config_desc *config_descriptor_header) {
    // Get points to the full configuration descriptor
    uint8_t *pd = (uint8_t *)config_descriptor_header;
    uint8_t *eod = (pd) + (config_descriptor_header->wTotalLength);

    // TODO: VID/PID filtering.
    // Sony VID: 0x054C
    // PS4 controller PIDs: 0x05C4 0x09CC

    // Search for the IN and OUT endpoints.
    struct usb_iface_desc *iface_desc;
    struct usb_ep_desc *in_ep = NULL;
    struct usb_ep_desc *out_ep = NULL;

    // Have we found the interface descriptor for the PS4 controller??
    bool found_iface = false;

    while (1) {
        pd = usb_desc_next(pd);

        if (pd == eod) {
            wtr_debug_printf("End of descriptors.\r\n");
            break;
        }

        if (usb_desc_type(pd) == USB_DT_INTERFACE) {
            iface_desc = USB_STRUCT_PTR(usb_iface_desc, pd);
            if (iface_desc->bInterfaceClass == HID_CLASS) {
                found_iface = true;
            } else {
                found_iface = false;
            }
        }

        if (found_iface && usb_desc_type(pd) == USB_DT_ENDPOINT) {
            struct usb_ep_desc *ep = USB_STRUCT_PTR(usb_ep_desc, pd);
            bool is_in = ep->bEndpointAddress & USB_EP_DIR_IN;
            wtr_debug_printf("Found endpoint descriptor. Address: %x, Attributes %x, packet "
                   "size: %u, interval: %u\r\n",
                   ep->bEndpointAddress, ep->bmAttributes, ep->wMaxPacketSize, ep->bInterval);
            if (is_in) {
                in_ep = ep;
            } else {
                out_ep = ep;
            }
        }
    }

    if (in_ep == NULL || out_ep == NULL) {
        wtr_debug_printf("Could not find the IN endpoint.\r\n");
        return WTR_USB_HD_STATUS_UNSUPPORTED;
    }

    _in_pipe = usb_h_pipe_allocate(pipe_0->hcd, pipe_0->dev, in_ep->bEndpointAddress, in_ep->wMaxPacketSize,
                                   in_ep->bmAttributes, in_ep->bInterval, pipe_0->speed, true);

    if (_in_pipe == NULL) {
        wtr_debug_printf("Failed to allocate IN pipe!\r\n");
        goto failed;
    }

    usb_h_pipe_register_callback(_in_pipe, _handle_pipe_in);

    _out_pipe = usb_h_pipe_allocate(pipe_0->hcd, pipe_0->dev, out_ep->bEndpointAddress, out_ep->wMaxPacketSize,
                                    out_ep->bmAttributes, out_ep->bInterval, pipe_0->speed, true);
    if (_out_pipe == NULL) {
        wtr_debug_printf("Failed to allocate OUT pipe!\r\n");
        goto failed;
    }

    usb_h_pipe_register_callback(_out_pipe, _handle_pipe_out);

    wtr_debug_printf("Pipes allocated!\r\n");

    // Enable HID reports by setting Idle to 0.
    // TODO: Should probably have a callback to check if this succeeds. Probably easier
    // once I have the control request queue.
    usb_fill_SetIdle_req((struct usb_req *)_out_pipe_buf, 0, 0, iface_desc->bInterfaceNumber);
    usb_h_pipe_register_callback(pipe_0, _handle_ctrl_in);
    usb_h_control_xfer(pipe_0, _out_pipe_buf, NULL, 0, 500);

    return WTR_USB_HD_STATUS_SUCCESS;

failed:
    if (_in_pipe != NULL) {
        usb_h_pipe_free(_in_pipe);
        _in_pipe = NULL;
    }
    if (_out_pipe != NULL) {
        usb_h_pipe_free(_out_pipe);
        _out_pipe = NULL;
    }
    return WTR_USB_HD_STATUS_FAILED;
}

static int32_t _handle_disconnection(uint8_t port) {
    if (_in_pipe != NULL) {
        usb_h_pipe_abort(_in_pipe);
        usb_h_pipe_free(_in_pipe);
        _in_pipe = NULL;
    }
    if (_out_pipe != NULL) {
        usb_h_pipe_abort(_out_pipe);
        usb_h_pipe_free(_out_pipe);
        _out_pipe = NULL;
    }

    return ERR_NONE;
}

static void _set_controller_params(uint8_t r, uint8_t g, uint8_t b, uint8_t rumble) {
    memset(_out_pipe_buf, 0, 64);

    _out_pipe_buf[0] = 0x05;
    _out_pipe_buf[1] = 0xFF;
    _out_pipe_buf[4] = rumble;
    _out_pipe_buf[6] = r;
    _out_pipe_buf[7] = g;
    _out_pipe_buf[8] = b;

    usb_h_bulk_int_iso_xfer(_out_pipe, _out_pipe_buf, 32, false);
}

// Called whenever a new, valid report comes in. Handles detecting
// key presses and releases.
// static void _handle_report() {
//     ..

//     // Copy the new report over the previous one.
//     memcpy(_prev_report.byte, _report.byte, sizeof(hid_kbd_input_report_t));
// }

// Called to send a request to poll the device for a new report. This is
// scheduled whenever a successful report comes in.
static void _poll() {
    int32_t result = usb_h_bulk_int_iso_xfer(_in_pipe, _in_pipe_buf, _in_pipe->max_pkt_size, false);

    if (result != ERR_NONE) {
        wtr_debug_printf("Failed to start IN transfer!");
    }
}

static void _handle_pipe_in(struct usb_h_pipe *pipe) {
    // bii is the bulk/iso/interupt transfer status.
    struct usb_h_bulk_int_iso_xfer *bii = &pipe->x.bii;

    // Pipe closed due to disconnect.
    if (bii->status == USB_H_ABORT)
        return;

    if (bii->status != USB_H_OK) {
        wtr_debug_printf("Error in PS4 IN. State: %u, Status: %i, Count: %lu, Size: "
               "%lu\r\n",
               bii->state, bii->status, bii->count, bii->size);
        return;
    }

    if (bii->count == 64) {
        // See if the new report is different from the last, if so, process the
        // new events.
        if (memcmp(_in_pipe_buf, _in_pipe_buf_prev, 64) != 0) {
            if (_in_pipe_buf[5] & 0x80) {
                _set_controller_params(0x00, 0xFF, 0x00, 0);
            }
            if (_in_pipe_buf[5] & 0x40) {
                _set_controller_params(0xFF, 0xA5, 0x00, 0);
            }
            if (_in_pipe_buf[5] & 0x20) {
                _set_controller_params(0x00, 0x00, 0xFF, 0);
            }
            if (_in_pipe_buf[5] & 0x10) {
                _set_controller_params(0xFF, 0x00, 0xA5, 0);
            }
            memcpy(_in_pipe_buf_prev, _in_pipe_buf, 64);
        }
    } else if (bii->count > 0) {
        printf("Unexpected HID report, size is %lu\r\n", bii->count);
    }

    // Make another request to the endpoint to continue polling.
    wtr_usb_host_schedule_func(&_poll, pipe->interval);
}

static void _handle_pipe_out(struct usb_h_pipe *pipe) {
}

static void _handle_ctrl_in(struct usb_h_pipe *pipe) {

    _set_controller_params(0xFF, 0x5A, 0x7F, 0xFF);

    // Send the first read request to the IN endpoint. The callback will handle
    // scheduling polling.
    _poll();
}