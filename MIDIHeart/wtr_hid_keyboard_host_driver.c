#include <stdio.h>
#include "wtr_usb_host.h"
#include "wtr_hid_keyboard_host_driver.h"



/* Constants */

#define USB_HID_IFACE_CLASS 0x03
#define USB_HID_BOOT_IFACE_SUBCLASS 0x01
#define USB_HID_KEYBOARD_REPORT_SIZE 8

/* Global variables */

struct usb_h_pipe *_in_pipe;
uint8_t _in_pipe_buf[USB_HID_KEYBOARD_REPORT_SIZE];
uint8_t _report[USB_HID_KEYBOARD_REPORT_SIZE];
uint8_t _prev_report[USB_HID_KEYBOARD_REPORT_SIZE];


/* Private function forward declarations */

static int32_t _handle_enumeration(struct usb_h_pipe*, struct usb_config_desc*);
static int32_t _handle_disconnection(uint8_t port);
static void _handle_pipe_in(struct usb_h_pipe* pipe);

/* Public functions */

void wtr_usb_hid_keyboard_init() {
	struct wtr_usb_host_driver driver;
	driver.enumeration_callback = _handle_enumeration;
	driver.disconnection_callback = _handle_disconnection;

	wtr_usb_host_register_driver(driver);
};


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
            if( iface_desc->bInterfaceClass == USB_HID_IFACE_CLASS && iface_desc->bInterfaceSubClass == USB_HID_BOOT_IFACE_SUBCLASS ) {
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


static void _print_bytes(uint8_t* bytes, size_t len) {
    for(size_t i = 0; i < len; i++) {
        printf("%2x ", bytes[i]);
    }
    printf("\r\n");
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
	
	if (bii->count == USB_HID_KEYBOARD_REPORT_SIZE) {
        memcpy(_report, bii->data, bii->count);

        if(memcmp(_report, _prev_report, USB_HID_KEYBOARD_REPORT_SIZE) != 0) {
            _print_bytes(_report, USB_HID_KEYBOARD_REPORT_SIZE);
        memcpy(_prev_report, _report, USB_HID_KEYBOARD_REPORT_SIZE);
        }
	}
    else if (bii->count > 0) {
        printf("Unexpected HID report, size is %lu\r\n", bii->count);
    }
	
	// Make another request to the endpoint to continue polling.
	int32_t result = usb_h_bulk_int_iso_xfer(pipe, _in_pipe_buf, pipe->max_pkt_size, false);
	
	if(result != ERR_NONE) {
		printf("Failed to start bulk transfer!");
	}
}
