#include "wtr_midi_host_driver.h"
#include "wtr_midi_debug.h"
#include "wtr_usb_host.h"
#include <stdio.h>

/* Constants and macros */

#define USB_AUDIO_INTERFACE_CLASS 0x01
#define USB_AUDIO_MIDI_STREAMING_SUBCLASS 0x03

/* Global variables */

static uint8_t _in_queue_data[WTR_USB_MIDI_HOST_BUF_SIZE];
static struct wtr_queue _in_queue;
static uint8_t _out_queue_data[WTR_USB_MIDI_HOST_BUF_SIZE];
static struct wtr_queue _out_queue;
static uint8_t _in_pipe_buf[64];
static uint8_t _out_pipe_buf[64];
static struct usb_h_pipe *_in_pipe;
static struct usb_h_pipe *_out_pipe;

/* Private function forward declarations */

static int32_t _handle_enumeration(struct usb_h_pipe *, struct usb_config_desc *);
static int32_t _handle_disconnection(uint8_t port);
static void _poll_in_pipe();
static void _write_out_pipe();
static void _handle_sof();
static void _handle_pipe_in(struct usb_h_pipe *);
static void _handle_pipe_out(struct usb_h_pipe *);

/* Public functions */

void wtr_usb_midi_host_init() {
    _in_queue.data = _in_queue_data;
    _in_queue.item_size = USB_MIDI_EVENT_PACKET_SIZE;
    _in_queue.capacity = WTR_USB_MIDI_HOST_BUF_SIZE / USB_MIDI_EVENT_PACKET_SIZE;
    _in_queue.overflow_behavior = WTR_QUEUE_OVF_DROP_OLDEST;
    wtr_queue_init(&_in_queue);

    _out_queue.data = _out_queue_data;
    _out_queue.item_size = USB_MIDI_EVENT_PACKET_SIZE;
    _out_queue.capacity = WTR_USB_MIDI_HOST_BUF_SIZE / USB_MIDI_EVENT_PACKET_SIZE;
    _out_queue.overflow_behavior = WTR_QUEUE_OVF_DROP_OLDEST;
    wtr_queue_init(&_out_queue);

    struct wtr_usb_host_driver driver;
    driver.enumeration_callback = &_handle_enumeration;
    driver.disconnection_callback = &_handle_disconnection;
    driver.sof_callback = &_handle_sof;

    wtr_usb_host_register_driver(driver);
};

bool wtr_usb_midi_is_device_attached() {
    // TODO
    return true;
}

struct wtr_queue *wtr_usb_midi_get_in_queue() {
    return &_in_queue;
}

struct wtr_queue *wtr_usb_midi_get_out_queue() {
    return &_out_queue;
}

/* Private functions */

static int32_t _handle_enumeration(struct usb_h_pipe *pipe_0, struct usb_config_desc *config_descriptor_header) {
    // Get points to the full configuration descriptor
    uint8_t *pd = (uint8_t *)config_descriptor_header;
    uint8_t *eod = (pd) + (config_descriptor_header->wTotalLength);

    // Search for the IN and OUT endpoints.
    struct usb_ep_desc *in_ep = NULL;
    struct usb_ep_desc *out_ep = NULL;

    // Have we found the interface descriptor for USB MIDI?
    bool found_iface = false;

    while (1) {
        pd = usb_desc_next(pd);

        if (pd == eod) {
            printf("End of descriptors.\r\n");
            break;
        }

        if (usb_desc_type(pd) == USB_DT_INTERFACE) {
            struct usb_iface_desc *iface_desc = USB_STRUCT_PTR(usb_iface_desc, pd);
            if (iface_desc->bInterfaceClass == USB_AUDIO_INTERFACE_CLASS &&
                iface_desc->bInterfaceSubClass == USB_AUDIO_MIDI_STREAMING_SUBCLASS) {
                found_iface = true;
            } else {
                found_iface = false;
            }
        }

        if (found_iface && usb_desc_type(pd) == USB_DT_ENDPOINT) {
            struct usb_ep_desc *ep_desc = USB_STRUCT_PTR(usb_ep_desc, pd);
            bool is_in = ep_desc->bEndpointAddress & USB_EP_DIR_IN;
            printf("Found endpoint descriptor. Address: %x, Attributes %x, packet "
                   "size: %u, interval: %u, input?: %u\r\n",
                   ep_desc->bEndpointAddress, ep_desc->bmAttributes, ep_desc->wMaxPacketSize, ep_desc->bInterval,
                   is_in);

            if (is_in) {
                in_ep = ep_desc;
            } else {
                out_ep = ep_desc;
            }
        }

        if (in_ep != NULL && out_ep != NULL)
            break;
    }

    if (in_ep == NULL || out_ep == NULL) {
        printf("Could not find and IN and OUT endpoints.\r\n");
        return WTR_USB_HD_STATUS_UNSUPPORTED;
    }

    _out_pipe = usb_h_pipe_allocate(pipe_0->hcd,
                                    pipe_0->dev,               // 0x01
                                    out_ep->bEndpointAddress,  // 0x02 for test device.
                                    out_ep->wMaxPacketSize,    // 0x40 for test device.
                                    out_ep->bmAttributes,      // 0x02 (Bulk transfer)
                                    out_ep->bInterval,         // 0x00
                                    pipe_0->speed,             // 0x01, USB_SPEED_FS
                                    true);

    if (_out_pipe == NULL) {
        printf("Failed to allocate OUT pipe!\r\n");
        goto failed;
    }

    usb_h_pipe_register_callback(_out_pipe, _handle_pipe_out);

    _in_pipe = usb_h_pipe_allocate(pipe_0->hcd,
                                   pipe_0->dev,              // 0x01
                                   in_ep->bEndpointAddress,  // 0x81 for test device
                                   in_ep->wMaxPacketSize,    // 0x40 for test device
                                   in_ep->bmAttributes,      // 0x02 (Bulk transfer)
                                   in_ep->bInterval,         // 0x00
                                   pipe_0->speed,            // 0x01, USB_SPEED_FS
                                   true);

    // Set a nack limit on the in pipe to prevent it from starving other drivers.
    _in_pipe->nack_limit = 100000;

    if (_in_pipe == NULL) {
        printf("Failed to allocate IN pipe!\r\n");
        goto failed;
    }

    usb_h_pipe_register_callback(_in_pipe, _handle_pipe_in);

    // This driver doesn't need pipe 0, so free it.
    usb_h_pipe_free(pipe_0);

    printf("Pipes allocated!\r\n");

    // Schedule the first poll of the IN pipe. It will handle scheduling
    // itself again.
    wtr_usb_host_schedule_func(&_poll_in_pipe, _in_pipe->interval);

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
    printf("MIDI disconnection called.\r\n");

    // TODO: Check the port and make sure its this device.
    // requires the usb host driver to track address -> port
    // relationships.

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

    wtr_queue_empty(&_in_queue);
    wtr_queue_empty(&_out_queue);

    return ERR_NONE;
}

static void _poll_in_pipe() {
    if (_in_pipe == NULL)
        return;

    int32_t result = usb_h_bulk_int_iso_xfer(_in_pipe, _in_pipe_buf, _in_pipe->max_pkt_size, false);

    if (result != ERR_NONE) {
        printf("Unable to start MIDI poll!, status: %li\r\n", result);
    }
}

static void _write_out_pipe() {
    if (_out_pipe == NULL || usb_h_pipe_is_busy(_out_pipe))
        return;

    // Send a request with the latest message from the out buf
    if (wtr_queue_is_empty(&_out_queue))
        return;

    wtr_queue_pop(&_out_queue, _out_pipe_buf);

    int32_t result = usb_h_bulk_int_iso_xfer(_out_pipe, _out_pipe_buf, _out_pipe->max_pkt_size, false);

    if (result != ERR_NONE) {
        printf("Sending MIDI out failed, reason: %li\r\n", result);
        return;
    }
}

static void _handle_sof() { _write_out_pipe(); }

static void _handle_pipe_in(struct usb_h_pipe *pipe) {
    // bii is the bulk/iso/interupt transfer status.
    struct usb_h_bulk_int_iso_xfer *bii = &pipe->x.bii;

    // Pipe closed due to disconnect.
    if (bii->status == USB_H_ABORT)
        return;

    // Request timed out. This is normal, just schedule another request.
    if (bii->status == USB_H_TIMEOUT)
        goto reschedule;

    if (bii->status != USB_H_OK) {
        printf("Error in MIDI IN. State: %u, Status: %i, Count: %lu, Size: %lu\r\n", bii->state, bii->status,
               bii->count, bii->size);
        return;
    }

    if (bii->count > 0) {
        /* Go through each valid event in the data and push it into the queue */
        uint8_t *event_ptr = bii->data;

        if (bii->count % USB_MIDI_EVENT_PACKET_SIZE != 0) {
            printf("Data is not a valid midi packet!\r\n");
            goto reschedule;
        }

        while (event_ptr < bii->data + bii->count) {
            // Invalid MIDI event packet, we've hit the end of the data.
            if (event_ptr[0] == 0)
                break;

            wtr_queue_push(&_in_queue, event_ptr);

            event_ptr += 4;
        }
    }

// Schedule the next poll.
reschedule:
    if (pipe->interval == 0) {
        _poll_in_pipe();
    } else {
        wtr_usb_host_schedule_func(&_poll_in_pipe, pipe->interval);
    }
}

static void _handle_pipe_out(struct usb_h_pipe *pipe) {}