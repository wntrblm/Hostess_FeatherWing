#include "wtr_hid_keyboard_host_driver.h"
#include "hid_keymap.h"
#include "usb_protocol.h"
#include "usb_protocol_hid.h"
#include "wtr_queue.h"
#include "wtr_usb_host.h"
#include <stdio.h>

/* Constants */

#define EVENT_QUEUE_SIZE 64
#define KEYSTRING_QUEUE_SIZE 64
#define KEY_ERR_OVF 0x01

#define HID_LEFTCTRL 0xe0    // Keyboard Left Control
#define HID_LEFTSHIFT 0xe1   // Keyboard Left Shift
#define HID_LEFTALT 0xe2     // Keyboard Left Alt
#define HID_LEFTMETA 0xe3    // Keyboard Left GUI
#define HID_RIGHTCTRL 0xe4   // Keyboard Right Control
#define HID_RIGHTSHIFT 0xe5  // Keyboard Right Shift
#define HID_RIGHTALT 0xe6    // Keyboard Right Alt
#define HID_RIGHTMETA 0xe7   // Keyboard Right GUI

/* Global variables */

static struct usb_h_pipe *_ctrl_pipe;
static uint8_t _ctrl_req_buf[sizeof(struct usb_req) + 1];
static struct usb_h_pipe *_in_pipe;
static uint8_t _in_pipe_buf[sizeof(hid_kbd_input_report_t)];
static uint8_t _locking_keys;
static hid_kbd_input_report_t _report;
static hid_kbd_input_report_t _prev_report;
static struct hid_keyboard_event event_queue_buf[sizeof(struct hid_keyboard_event) * EVENT_QUEUE_SIZE];
static struct wtr_queue event_queue;
static uint8_t keystring_queue_buf[KEYSTRING_QUEUE_SIZE];
static struct wtr_queue keystring_queue;

/* Private function forward declarations */

static int32_t _handle_enumeration(struct usb_h_pipe *, struct usb_config_desc *);
static int32_t _handle_disconnection(uint8_t port);
static void _poll();
static void _handle_pipe_in(struct usb_h_pipe *pipe);
static void _handle_ctrl_pipe(struct usb_h_pipe *pipe);
static void _handle_event_for_locking_keys(struct hid_keyboard_event *event);
static void _handle_event_for_keystring(struct hid_keyboard_event *event);
static void _handle_report();
static inline void _handle_report_control_key_helper(uint8_t prev_bit, uint8_t curr_bit, uint8_t keycode,
                                                     uint8_t modifiers);

/* Public functions */

void wtr_usb_hid_keyboard_init() {
    event_queue.data = (uint8_t *)event_queue_buf;
    event_queue.item_size = sizeof(struct hid_keyboard_event);
    event_queue.capacity = EVENT_QUEUE_SIZE;
    wtr_queue_init(&event_queue);

    keystring_queue.data = (uint8_t *)keystring_queue_buf;
    keystring_queue.item_size = sizeof(uint8_t);
    keystring_queue.capacity = KEYSTRING_QUEUE_SIZE;
    wtr_queue_init(&keystring_queue);

    struct wtr_usb_host_driver driver;
    driver.enumeration_callback = &_handle_enumeration;
    driver.disconnection_callback = &_handle_disconnection;

    wtr_usb_host_register_driver(driver);
};

struct wtr_queue *wtr_usb_hid_keyboard_get_event_queue() {
    return &event_queue;
}

struct wtr_queue *wtr_usb_hid_keyboard_get_keystring_queue() {
    return &keystring_queue;
}

/* Private functions */

static int32_t _handle_enumeration(struct usb_h_pipe *pipe_0, struct usb_config_desc *config_descriptor_header) {
    // Get points to the full configuration descriptor
    uint8_t *pd = (uint8_t *)config_descriptor_header;
    uint8_t *eod = (pd) + (config_descriptor_header->wTotalLength);

    // Search for the IN and OUT endpoints.
    struct usb_ep_desc *in_ep = NULL;

    // Have we found the interface descriptor for USB keyboard / Boot device?
    bool found_iface = false;

    while (1) {
        pd = usb_desc_next(pd);

        if (pd == eod) {
            printf("End of descriptors.\r\n");
            break;
        }

        printf("Found descriptor at %p, length: %u, type: %x\r\n", pd, pd[0], pd[1]);

        if (usb_desc_type(pd) == USB_DT_INTERFACE) {
            struct usb_iface_desc *iface_desc = USB_STRUCT_PTR(usb_iface_desc, pd);
            if (iface_desc->bInterfaceClass == HID_CLASS && iface_desc->bInterfaceSubClass == HID_SUB_CLASS_BOOT &&
                iface_desc->bInterfaceProtocol == HID_PROTOCOL_KEYBOARD) {
                found_iface = true;
            } else {
                found_iface = false;
            }
        }

        if (found_iface && usb_desc_type(pd) == USB_DT_ENDPOINT) {
            in_ep = USB_STRUCT_PTR(usb_ep_desc, pd);
            printf("Found endpoint descriptor. Address: %x, Attributes %x, packet "
                   "size: %u, interval: %u\r\n",
                   in_ep->bEndpointAddress, in_ep->bmAttributes, in_ep->wMaxPacketSize, in_ep->bInterval);
        }
    }

    if (in_ep == NULL) {
        printf("Could not find the IN endpoint.\r\n");
        return WTR_USB_HD_STATUS_UNSUPPORTED;
    }

    _in_pipe = usb_h_pipe_allocate(pipe_0->hcd, pipe_0->dev, in_ep->bEndpointAddress, in_ep->wMaxPacketSize,
                                   in_ep->bmAttributes, in_ep->bInterval, pipe_0->speed, true);

    if (_in_pipe == NULL) {
        printf("Failed to allocate IN pipe!\r\n");
        goto failed;
    }

    usb_h_pipe_register_callback(_in_pipe, _handle_pipe_in);

    // We need pipe0 to send SetReport requests to set the LED status.
    _ctrl_pipe = pipe_0;

    printf("Pipes allocated!\r\n");

    // Send the first read request to the IN endpoint. The callback will handle
    // scheduling polling.
    _poll();

    return WTR_USB_HD_STATUS_SUCCESS;

failed:
    if (_in_pipe != NULL) {
        usb_h_pipe_free(_in_pipe);
        _in_pipe = NULL;
    }
    if (_ctrl_pipe != NULL) {
        usb_h_pipe_free(_ctrl_pipe);
        _ctrl_pipe = NULL;
    }
    return WTR_USB_HD_STATUS_FAILED;
}

static int32_t _handle_disconnection(uint8_t port) {
    if (_in_pipe != NULL) {
        usb_h_pipe_abort(_in_pipe);
        usb_h_pipe_free(_in_pipe);
        _in_pipe = NULL;
    }

    if (_ctrl_pipe != NULL) {
        usb_h_pipe_abort(_ctrl_pipe);
        usb_h_pipe_free(_ctrl_pipe);
        _ctrl_pipe = NULL;
    }

    return ERR_NONE;
}

// Handles a new keyboard event and updates the keystring buffer
// if needed.
static void _handle_event_for_keystring(struct hid_keyboard_event *event) {
    if (event->type != HID_KB_EVENT_KEY_PRESS)
        return;
    if (event->keycode < 1 || event->keycode > 127)
        return;

    uint8_t ascii_val = hid_to_ascii_map[event->keycode];

    // Handle shifting
    // TODO: Handle caps and num lock.
    bool shifted = event->modifiers & (HID_KBD_L_SHIFT | HID_KBD_R_SHIFT);

    if (shifted) {
        // Letters.
        if (ascii_val >= 'a' && ascii_val <= 'z') {
            ascii_val -= 32;
        }

        // Numbers & printable symbols
        switch (ascii_val) {
        case '`':
            ascii_val = '~';
            break;
        case '1':
            ascii_val = '!';
            break;
        case '2':
            ascii_val = '@';
            break;
        case '3':
            ascii_val = '#';
            break;
        case '4':
            ascii_val = '$';
            break;
        case '5':
            ascii_val = '%';
            break;
        case '6':
            ascii_val = '^';
            break;
        case '7':
            ascii_val = '&';
            break;
        case '8':
            ascii_val = '*';
            break;
        case '9':
            ascii_val = '(';
            break;
        case '0':
            ascii_val = ')';
            break;
        case '-':
            ascii_val = '_';
            break;
        case '+':
            ascii_val = '=';
            break;
        case '[':
            ascii_val = '{';
            break;
        case ']':
            ascii_val = '}';
            break;
        case '\\':
            ascii_val = '|';
            break;
        case ';':
            ascii_val = ':';
            break;
        case '\'':
            ascii_val = '"';
            break;
        case ',':
            ascii_val = '<';
            break;
        case '.':
            ascii_val = '>';
            break;
        case '/':
            ascii_val = '?';
            break;
        default:
            break;
        }
    }

    // TODO: Handle control characters.

    if (!wtr_queue_is_full(&keystring_queue))
        wtr_queue_push(&keystring_queue, &ascii_val);
}

// Handles updating the locking keys and LEDs whenever a there's a key event for
// caps lock, scroll lock, or num lock.
static void _handle_event_for_locking_keys(struct hid_keyboard_event *event) {
    bool changed = true;

    switch (event->keycode) {
    case HID_CAPS_LOCK:
        _locking_keys ^= HID_LED_CAPS_LOCK;
        break;

    case HID_SCROLL_LOCK:
        _locking_keys ^= HID_LED_SCROLL_LOCK;
        break;

    case HID_KBD_NUM_LOCK:
        _locking_keys ^= HID_LED_NUM_LOCK;
        break;

    default:
        changed = false;
        break;
    }

    if (changed) {
        usb_fill_SetReport_req((struct usb_req *)_ctrl_req_buf, REPORT_TYPE_OUTPUT, 0, 0, 1);
        _ctrl_req_buf[sizeof(struct usb_req)] = _locking_keys;
        usb_h_control_xfer(_ctrl_pipe, _ctrl_req_buf, _ctrl_req_buf + 8, 1, 500);
    }
}

// Called whenever a new, valid report comes in. Handles detecting
// key presses and releases.
static void _handle_report() {
    // TODO: There's a clever way to check for pressed and released keys at the
    // same time, but I don't feel like figuring it out right now.
    // Check for newly pressed keys.
    for (size_t i = 0; i < 6; i++) {
        bool found = false;
        uint8_t curr_val = _report.field.key[i];

        if (curr_val == 0 || curr_val == KEY_ERR_OVF)
            continue;

        for (size_t j = 0; j < 6; j++) {
            uint8_t prev_val = _prev_report.field.key[j];

            if (curr_val == prev_val)
                found = true;
        }

        if (!found) {
            struct hid_keyboard_event event;
            event.type = HID_KB_EVENT_KEY_PRESS;
            event.keycode = curr_val;
            event.modifiers = _report.field.modifier.byte;
            if (!wtr_queue_is_full(&event_queue))
                wtr_queue_push(&event_queue, (uint8_t *)(&event));
            _handle_event_for_keystring(&event);
            _handle_event_for_locking_keys(&event);
        }
    }

    // Check for newly released keys.
    for (size_t i = 0; i < 6; i++) {
        bool found = false;
        uint8_t prev_val = _prev_report.field.key[i];

        if (prev_val == 0 || prev_val == KEY_ERR_OVF)
            continue;

        for (size_t j = 0; j < 6; j++) {
            uint8_t curr_val = _report.field.key[j];

            if (curr_val == prev_val)
                found = true;
        }

        if (!found) {
            struct hid_keyboard_event event;
            event.type = HID_KB_EVENT_KEY_RELEASE;
            event.keycode = prev_val;
            event.modifiers = _report.field.modifier.byte;
            if (!wtr_queue_is_full(&event_queue))
                wtr_queue_push(&event_queue, (uint8_t *)(&event));
            _handle_event_for_keystring(&event);
            // don't send events for locking key release.
        }
    }

    // Map control keys to their corresponding usage codes to make things
    // easier for the event processor.
    _handle_report_control_key_helper(_report.field.modifier.bm.lctrl, _prev_report.field.modifier.bm.lctrl,
                                      HID_LEFTCTRL, _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.rctrl, _prev_report.field.modifier.bm.rctrl,
                                      HID_RIGHTCTRL, _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.lshift, _prev_report.field.modifier.bm.lshift,
                                      HID_LEFTSHIFT, _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.rshift, _prev_report.field.modifier.bm.rshift,
                                      HID_RIGHTSHIFT, _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.lalt, _prev_report.field.modifier.bm.lalt, HID_LEFTALT,
                                      _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.ralt, _prev_report.field.modifier.bm.ralt, HID_RIGHTALT,
                                      _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.lgui, _prev_report.field.modifier.bm.lgui, HID_LEFTMETA,
                                      _report.field.modifier.byte);
    _handle_report_control_key_helper(_report.field.modifier.bm.rgui, _prev_report.field.modifier.bm.rgui,
                                      HID_RIGHTMETA, _report.field.modifier.byte);

    // Copy the new report over the previous one.
    memcpy(_prev_report.byte, _report.byte, sizeof(hid_kbd_input_report_t));
}

static inline void _handle_report_control_key_helper(uint8_t prev_bit, uint8_t curr_bit, uint8_t keycode,
                                                     uint8_t modifiers) {
    struct hid_keyboard_event event;
    event.keycode = keycode;
    event.modifiers = modifiers;
    if (curr_bit && !prev_bit) {
        event.type = HID_KB_EVENT_KEY_PRESS;
        if (!wtr_queue_is_full(&event_queue))
            wtr_queue_push(&event_queue, (uint8_t *)(&event));
    } else if (!curr_bit && prev_bit) {
        event.type = HID_KB_EVENT_KEY_RELEASE;
        if (!wtr_queue_is_full(&event_queue))
            wtr_queue_push(&event_queue, (uint8_t *)(&event));
    }
}

// Called to send a request to poll the device for a new report. This is
// scheduled whenever a successful report comes in.
static void _poll() {
    int32_t result = usb_h_bulk_int_iso_xfer(_in_pipe, _in_pipe_buf, _in_pipe->max_pkt_size, false);

    if (result != ERR_NONE) {
        printf("Failed to start bulk transfer!");
    }
}

static void _handle_pipe_in(struct usb_h_pipe *pipe) {
    // bii is the bulk/iso/interupt transfer status.
    struct usb_h_bulk_int_iso_xfer *bii = &pipe->x.bii;

    // Pipe closed due to disconnect.
    if (bii->status == USB_H_ABORT)
        return;

    if (bii->status != USB_H_OK) {
        printf("Error in HID Keyboard IN. State: %u, Status: %i, Count: %lu, Size: "
               "%lu\r\n",
               bii->state, bii->status, bii->count, bii->size);
        return;
    }

    if (bii->count == sizeof(hid_kbd_input_report_t)) {
        memcpy(_report.byte, bii->data, bii->count);

        // See if the new report is different from the last, if so, process the
        // new events.
        if (memcmp(_report.byte, _prev_report.byte, sizeof(hid_kbd_input_report_t)) != 0) {
            _handle_report();
        }
    } else if (bii->count > 0) {
        printf("Unexpected HID report, size is %lu\r\n", bii->count);
    }

    // Make another request to the endpoint to continue polling.
    wtr_usb_host_schedule_func(&_poll, pipe->interval);
}

static void _handle_ctrl_pipe(struct usb_h_pipe *pipe) {}