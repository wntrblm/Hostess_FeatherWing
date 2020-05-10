#include <stdio.h>
#include "usb_protocol.h"
#include "usb_protocol_hid.h"
#include "wtr_queue.h"
#include "wtr_usb_host.h"
#include "wtr_hid_keyboard_host_driver.h"
#include "hid_keymap.h"


/* Constants */
#define EVENT_QUEUE_SIZE 64
#define KEYSTRING_QUEUE_SIZE 64
#define KEY_ERR_OVF 0x01

/* Global variables */

struct usb_h_pipe *_in_pipe;
uint8_t _in_pipe_buf[sizeof(hid_kbd_input_report_t)];
hid_kbd_input_report_t _report;
hid_kbd_input_report_t _prev_report;
struct hid_keyboard_event event_queue_buf[sizeof(struct hid_keyboard_event) * EVENT_QUEUE_SIZE];
struct wtr_queue event_queue;
uint8_t keystring_queue_buf[KEYSTRING_QUEUE_SIZE];
struct wtr_queue keystring_queue;


/* Private function forward declarations */

static int32_t _handle_enumeration(struct usb_h_pipe*, struct usb_config_desc*);
static int32_t _handle_disconnection(uint8_t port);
static void _handle_pipe_in(struct usb_h_pipe* pipe);

/* Public functions */

void wtr_usb_hid_keyboard_init() {
    event_queue.data = (uint8_t*)event_queue_buf;
    event_queue.item_size = sizeof(struct hid_keyboard_event);
    event_queue.capacity = EVENT_QUEUE_SIZE;
    wtr_queue_init(&event_queue);

    keystring_queue.data = (uint8_t*)keystring_queue_buf;
    keystring_queue.item_size = sizeof(uint8_t);
    keystring_queue.capacity = KEYSTRING_QUEUE_SIZE;
    wtr_queue_init(&keystring_queue);

	struct wtr_usb_host_driver driver;
	driver.enumeration_callback = _handle_enumeration;
	driver.disconnection_callback = _handle_disconnection;

	wtr_usb_host_register_driver(driver);
};

struct wtr_queue* wtr_usb_hid_keyboard_get_event_queue() {
    return &event_queue;
}

struct wtr_queue* wtr_usb_hid_keyboard_get_keystring_queue() {
    return &keystring_queue;
}


/* Private functions */

static int32_t _handle_enumeration(struct usb_h_pipe *pipe_0, struct usb_config_desc *config_descriptor_header) {
    // Get points to the full configuration descriptor
	uint8_t *pd = (uint8_t *) config_descriptor_header;
	uint8_t *eod = (pd) + (config_descriptor_header->wTotalLength);
	
	// Search for the IN and OUT endpoints.
	struct usb_ep_desc *in_ep = NULL;

    // Have we found the interface descriptor for USB keyboard / Boot device?
    bool found_iface = false;
	
	while(1) {
		pd = usb_desc_next(pd);
		
		if (pd == eod) {
			printf("End of descriptors.\r\n");
			break;
		}

		printf("Found descriptor at %p, length: %u, type: %x\r\n", pd, pd[0], pd[1]);

        if(usb_desc_type(pd) == USB_DT_INTERFACE) {
            struct usb_iface_desc *iface_desc = USB_STRUCT_PTR(usb_iface_desc, pd);
            if( iface_desc->bInterfaceClass == HID_CLASS
                && iface_desc->bInterfaceSubClass == HID_SUB_CLASS_BOOT
                && iface_desc->bInterfaceProtocol == HID_PROTOCOL_KEYBOARD) {
                found_iface = true;
            } else {
                found_iface = false;
            }
        }
		
		if(found_iface && usb_desc_type(pd) == USB_DT_ENDPOINT) {
			in_ep = USB_STRUCT_PTR(usb_ep_desc, pd);
			printf(
				"Found endpoint descriptor. Address: %x, Attributes %x, packet size: %u, interval: %u\r\n",
				in_ep->bEndpointAddress,
				in_ep->bmAttributes,
				in_ep->wMaxPacketSize,
				in_ep->bInterval
			);
		}
	}
	
	if(in_ep == NULL) {
		printf("Could not find the IN endpoint.\r\n");
		return;
	}
	
	_in_pipe = usb_h_pipe_allocate(
		pipe_0->hcd,
		pipe_0->dev,
		in_ep->bEndpointAddress,
		in_ep->wMaxPacketSize,
		in_ep->bmAttributes,
		in_ep->bInterval,
		pipe_0->speed,
		true
    );
		
	if (_in_pipe == NULL) {
		printf("Failed to allocate IN pipe!\r\n");
		return;
	} 
	
	usb_h_pipe_register_callback(_in_pipe, _handle_pipe_in);
	
	// This driver doesn't need pipe 0, so free it.
	usb_h_pipe_free(pipe_0);
	
	printf("Pipes allocated!\r\n");
	
	// Send the first read request to the IN endpoint. The callback will handle polling.
	int32_t result = usb_h_bulk_int_iso_xfer(_in_pipe, _in_pipe_buf, _in_pipe->max_pkt_size, false);
	
	if(result != ERR_NONE) {
		printf("Failed to start bulk transfer!");
	}
}


static int32_t _handle_disconnection(uint8_t port) {
    printf("Please implement handle disconnection for usb hid keyboard.\r\n");
}


// TODO: Remove
static void _print_bytes(uint8_t* bytes, size_t len) {
    for(size_t i = 0; i < len; i++) {
        printf("%2x ", bytes[i]);
    }
    printf("\r\n");
}


// Handles a new keyboard event and updates the keystring buffer
// if needed.
static void _handle_event_for_keystring(struct hid_keyboard_event *event) {
    switch(event->type) {
        case HID_KB_EVENT_KEY_PRESS:
            if(event->keycode > 1 && event->keycode < 127) {
                uint8_t ascii_val = hid_to_ascii_map[event->keycode];
                if(!wtr_queue_is_full(&keystring_queue)) wtr_queue_push(&keystring_queue, &ascii_val);
            }
            break;

        default:
            break;
    }
}


// Called whenever a new, valid report comes in. Handles detecting
// key presses and releases.
static void _handle_report() {
    // TODO: include control keys as well.
    // TODO: There's a clever way to check for pressed and released keys at the
    // same time, but I don't feel like figuring it out right now.
    // Check for newly pressed keys.
    for(size_t i = 0; i < 6; i++) {
        bool found = false;
        uint8_t curr_val = _report.field.key[i];

        if(curr_val == 0 || curr_val == KEY_ERR_OVF) continue;

        for(size_t j = 0; j < 6; j++) {
            uint8_t prev_val = _prev_report.field.key[j];

            if(curr_val == prev_val) found = true;
        }

        if(!found) {
            struct hid_keyboard_event event;
            event.type = HID_KB_EVENT_KEY_PRESS;
            event.keycode = curr_val;
            event.modifiers = _report.field.modifier.byte;
            if(!wtr_queue_is_full(&event_queue)) wtr_queue_push(&event_queue, (uint8_t *)(&event));
            _handle_event_for_keystring(&event);
        }
    }

    // Check for newly released keys.
    for(size_t i = 0; i < 6; i++) {
        bool found = false;
        uint8_t prev_val = _prev_report.field.key[i];

        if(prev_val == 0 || prev_val == KEY_ERR_OVF) continue;

        for(size_t j = 0; j < 6; j++) {
            uint8_t curr_val = _report.field.key[j];

            if(curr_val == prev_val) found = true;
        }

        if(!found) {
            struct hid_keyboard_event event;
            event.type = HID_KB_EVENT_KEY_RELEASE;
            event.keycode = prev_val;
            event.modifiers = _report.field.modifier.byte;
            if(!wtr_queue_is_full(&event_queue)) wtr_queue_push(&event_queue, (uint8_t *)(&event));
            _handle_event_for_keystring(&event);
        }
    }

    // Copy the new report over the previous one.
    memcpy(_prev_report.byte, _report.byte, sizeof(hid_kbd_input_report_t));
}


static void _handle_pipe_in(struct usb_h_pipe* pipe) {
	// bii is the bulk/iso/interupt transfer status.
	struct usb_h_bulk_int_iso_xfer* bii = &pipe->x.bii;
	
	if (bii->status != USB_H_OK) {
		printf(
			"Error in HID Keyboard IN. State: %u, Status: %i, Count: %lu, Size: %lu\r\n",
			bii->state,
			bii->status,
			bii->count,
			bii->size
		);
		return;
	}
	
	if (bii->count == sizeof(hid_kbd_input_report_t)) {
        memcpy(_report.byte, bii->data, bii->count);

        // See if the new report is different from the last, if so, process the
        // new events.
        if(memcmp(_report.byte, _prev_report.byte, sizeof(hid_kbd_input_report_t)) != 0) {
            _handle_report();
        }
	}
    else if (bii->count > 0) {
        printf("Unexpected HID report, size is %lu\r\n", bii->count);
    }
	
	// Make another request to the endpoint to continue polling.
    // TODO: Use polling interval!
	int32_t result = usb_h_bulk_int_iso_xfer(pipe, _in_pipe_buf, pipe->max_pkt_size, false);
	
	if(result != ERR_NONE) {
		printf("Failed to start bulk transfer!");
	}
}
