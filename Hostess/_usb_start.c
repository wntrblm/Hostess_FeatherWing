/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "atmel_start.h"
#include "usb_start.h"
#include "hpl_usb_host.h"
#include "hal_usb_host.h"

bool midi_notes[127];

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


static uint8_t req_buf[CONF_USBH_CTRL_BUF_SIZE];
static uint8_t rep_buf[CONF_USBH_CTRL_BUF_SIZE];
static uint8_t current_conf_desc[CONF_USBH_CTRL_BUF_SIZE];
static uint8_t midi_data_buf[64];


/** Get structure access pointer macros */
#define USB_STRUCT_PTR(str, buf) ((struct str *)(buf))


static int32_t send_request_get_dev_desc(struct usb_h_pipe* pipe_0, uint8_t len) {
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_DESC;
	req->wValue         = (USB_DT_DEVICE << 8);
	req->wIndex         = 0;
	req->wLength        = 18;
	return usb_h_control_xfer(pipe_0, (uint8_t *)req, (uint8_t *)rep_buf, len, 500);
}

static int32_t send_request_get_conf_desc(struct usb_h_pipe* pipe_0, uint8_t cfg_idx, uint16_t len)
{
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
	req->bRequest       = USB_REQ_GET_DESC;
	req->wValue         = (USB_DT_CONFIG << 8) | cfg_idx;
	req->wIndex         = 0;
	req->wLength        = len;
	return usb_h_control_xfer(pipe_0, (uint8_t *)req, (uint8_t *)rep_buf, len, 500);
}

static int32_t send_request_set_address(struct usb_h_pipe* pipe_0, uint8_t address) {
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
	req->bRequest       = USB_REQ_SET_ADDRESS;
	req->wValue         = address;
	req->wIndex         = 0;
	req->wLength        = 0;
	return usb_h_control_xfer(pipe_0, (uint8_t *)req, NULL, 0, 500);
}

static int32_t send_request_set_config(struct usb_h_pipe* pipe_0, uint8_t cfg)
{
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	req->bmRequestType  = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
	req->bRequest       = USB_REQ_SET_CONFIG;
	req->wValue         = cfg;
	req->wIndex         = 0;
	req->wLength        = 0;
	return usb_h_control_xfer(pipe_0, (uint8_t *)req, NULL, 0, 50);
}


static void wtr_usb_rh_connection_handler(struct usb_h_desc *hcd, uint32_t status, uint8_t port) {
	if (status) {
		/* A device was connected. Reset and enumerate. */
		usb_h_rh_reset(hcd, port);
		printf("Port %u reset.\r\n", port);
	} else {
		/* A device was disconnected. Tear down and resources. */
		// TODO
		printf("Device disconnection event not implemented.\r\n");
	}
}

static void wtr_usb_get_dev_desc_reply_handler(struct usb_h_pipe *pipe) {
	uint32_t request_status = 0;
	// TODO: Handle error.
	
	if (pipe->x.ctrl.count < sizeof(usb_dev_desc_t)) {
		// Update max packet size and re-request the descriptor.
		// Max packet size is byte 7 in the device descriptor.
		uint8_t bMaxPackSize0 = rep_buf[7];
		request_status = usb_h_pipe_set_control_param(
			pipe, pipe->dev, pipe->ep, bMaxPackSize0, pipe->speed);
			
		if(request_status != ERR_NONE) {
			printf("Failed to adjust endpoint packet size, status: %lu\r\n", request_status);
			return;
		}
		
		printf("Requesting full device descriptor.\r\n");
		
		send_request_get_dev_desc(pipe, sizeof(usb_dev_desc_t));
		
		return;
	}
	
	struct usb_dev_desc* dev_desc = USB_STRUCT_PTR(usb_dev_desc, rep_buf);
	
	printf("USB Descriptor:\r\n");
	printf("\tLength: %u\r\n", dev_desc->bLength);
	printf("\tType: %u\r\n", dev_desc->bDescriptorType);
	printf("\tClass: %u\r\n", dev_desc->bDeviceClass);
	printf("\tSublass: %u\r\n", dev_desc->bDeviceSubClass);
	printf("\tProtocol: %u\r\n", dev_desc->bDeviceProtocol);
	printf("\tEP0 max packet size: %u\r\n", dev_desc->bMaxPacketSize0);
	printf("\tVendor: %x\r\n", dev_desc->idVendor);
	printf("\tProduct: %x\r\n", dev_desc->idProduct);
	printf("\tNum configurations: %u\r\n", dev_desc->bNumConfigurations);
	
	printf("Assigning address 1\r\n");
	request_status = send_request_set_address(pipe, 1);
	
	printf("Request status: %lu\r\n", request_status);
}

static void print_buf_hex(uint8_t* buf, uint32_t size) {
	for(uint32_t i = 0; i < size; i++) {
		printf("%02x ", buf[i]);
	}
	printf("\r\n");
}


static void midi_in_handler(struct usb_h_pipe *pipe) {
	// bii is the bulk/iso/interupt transfer status.
	struct usb_h_bulk_int_iso_xfer* bii = &pipe->x.bii;
	
	//printf("PCFG 0: %x\r\n", ((UsbHost *)pipe->hcd->hw)->HostPipe[0].PCFG.reg);
	
	if (bii->status != USB_H_OK) {
		printf(
			"Error in MIDI IN. State: %u, Status: %i, Count: %lu, Size: %lu\r\n",
			bii->state,
			bii->status,
			bii->count,
			bii->size
		);
		return;
	}
	
	if (bii->count > 0) {
		//print_buf_hex(bii->data, bii->count);
		uint8_t* event_idx = bii->data;
		while(event_idx < bii->data + bii->count) {
			// Invalid MIDI event packet, we've hit the end of the data.
			if(event_idx[0] == 0) break;
			
			uint8_t event_type = event_idx[0] & 0xF;
			if(event_type == 0x9) {
				// Note on.
				midi_notes[event_idx[2]] = true;
			}
			else if (event_type == 0x8) {
				// Note off.
				midi_notes[event_idx[2]] = false;
			}
			
			event_idx += 4;
		}
	}
	
	//delay_ms(100);
	
	// Make another request to the endpoint to continue polling.
	int32_t result = usb_h_bulk_int_iso_xfer(pipe, midi_data_buf, pipe->max_pkt_size, false);
	
	if(result != ERR_NONE) {
		printf("Failed to start bulk transfer!");
	}
}


static void midi_out_handler(struct usb_h_pipe *pipe) {
	struct usb_h_bulk_int_iso_xfer* bii = &pipe->x.bii;
	printf(
		"Got USB MIDI out data. State: %u, Status: %i, Count: %lu, Size: %lu\r\n",
		bii->state,
		bii->status,
		bii->count,
		bii->size
	);
}


static void wtr_usb_midi_install_driver(struct usb_h_pipe *pipe_0) {
	// Get the current configuration descriptor.
	uint8_t *pd = current_conf_desc;
	struct usb_config_desc* conf_desc = USB_STRUCT_PTR(usb_config_desc, current_conf_desc);
	uint8_t *eod = (current_conf_desc) + (conf_desc->wTotalLength);
	
	// Search for the IN and OUT endpoints.
	struct usb_ep_desc *in_ep = NULL;
	struct usb_ep_desc *out_ep = NULL;
	
	while(1) {
		pd = usb_desc_next(pd);
		
		if (pd == eod) {
			printf("End of descriptors.\r\n");
			break;
		}

		printf("Found descriptor at %p, length: %u, type: %x\r\n", pd, pd[0], pd[1]);
		
		if(usb_desc_type(pd) == USB_DT_ENDPOINT) {
			struct usb_ep_desc *ep_desc = USB_STRUCT_PTR(usb_ep_desc, pd);
			bool is_in = ep_desc->bEndpointAddress & USB_EP_DIR_IN;
			printf(
				"Found endpoint descriptor. Address: %x, Attributes %x, packet size: %u, interval: %u, input?: %u\r\n",
				ep_desc->bEndpointAddress,
				ep_desc->bmAttributes,
				ep_desc->wMaxPacketSize,
				ep_desc->bInterval,
				is_in
			);
			
			if(is_in) {
				in_ep = ep_desc;
			} else {
				out_ep = ep_desc;
			}
		}
	}
	
	if(in_ep == NULL) {
		printf("Could not find and IN and OUT endpoints.\r\n");
		return;
	}
	
	struct usb_h_pipe *in_pipe = usb_h_pipe_allocate(
		pipe_0->hcd,
		pipe_0->dev, // 0x01
		in_ep->bEndpointAddress, // 0x81 for test device
		in_ep->wMaxPacketSize, // 0x40 for test device
		in_ep->bmAttributes, // 0x02 (Bulk transfer)
		in_ep->bInterval, // 0x00
		pipe_0->speed, // 0x01, USB_SPEED_FS
		true);
		
	if (in_pipe == NULL) {
		printf("Failed to allocate IN pipe!\r\n");
		return;
	} 
	
	usb_h_pipe_register_callback(in_pipe, midi_in_handler);
	
	struct usb_h_pipe *out_pipe = usb_h_pipe_allocate(
		pipe_0->hcd,
		pipe_0->dev,  // 0x01
		out_ep->bEndpointAddress, // 0x02 for test device.
		out_ep->wMaxPacketSize,  // 0x40 for test device.
		out_ep->bmAttributes, // 0x02 (Bulk transfer)
		out_ep->bInterval, // 0x00
		pipe_0->speed, // 0x01, USB_SPEED_FS
		true);
	
	if (out_pipe == NULL) {
		printf("Failed to allocate OUT pipe!\r\n");
		return;
	}
	
	usb_h_pipe_register_callback(out_pipe, midi_out_handler);
	
	printf("Pipes allocated!\r\n");
	
	// Send the first read request to the IN endpoint. The callback will handle polling.
	int32_t result = usb_h_bulk_int_iso_xfer(in_pipe, midi_data_buf, in_pipe->max_pkt_size, false);
	
	if(result != ERR_NONE) {
		printf("Failed to start bulk transfer!");
	}
}


static void wtr_usb_get_conf_desc_reply_handler(struct usb_h_pipe *pipe) {
	uint32_t request_status = 0;
	struct usb_config_desc* conf_desc = USB_STRUCT_PTR(usb_config_desc, rep_buf);
	
	// We didn't get the whole descriptor header, fail.
	if (pipe->x.ctrl.count < sizeof(struct usb_config_desc)) {
		printf("Enumeration failed, config descriptor is not the right size.\r\n");
		return;
	}
	
	// We got the descriptor header, but need to request the full descriptor.
	if(pipe->x.ctrl.count == sizeof(struct usb_config_desc)) {
		uint16_t descriptor_length = conf_desc->wTotalLength;
		
		// TODO: Make sure the descriptor length isn't longer than our reply buffer.
		
		printf("Requesting full config descriptor.\r\n");
		
		send_request_get_conf_desc(pipe, 0, descriptor_length);
		
		return;
	}

	printf("Got full descriptor. Length: %u\r\n", conf_desc->wTotalLength);	

	memcpy(current_conf_desc, (uint8_t *) conf_desc, conf_desc->wTotalLength);
	
	printf("Setting configuration");
	request_status = send_request_set_config(pipe, 1);
	
	if(request_status != ERR_NONE) {
		printf("Failed to send configuration set request\r\n");
	}
}


static void wtr_usb_set_address_reply_handler(struct usb_h_pipe *pipe, struct usb_req *req) {
	uint16_t address = req->wValue;
	uint32_t request_status = usb_h_pipe_set_control_param(pipe, address, 0, pipe->max_pkt_size, pipe->speed);
	
	if(request_status != ERR_NONE) {
		printf("Failed to adjust endpoint address to %u, status: %lu\r\n", address, request_status);
		return;
	}
	
	printf("Pipe 0 reconfigured to use address %u\r\n", address);
	
	delay_ms(5);
	
	send_request_get_conf_desc(pipe, 0, sizeof(usb_config_desc_t));
}

static void wtr_usb_rh_pipe_0_xfer_done(struct usb_h_pipe *pipe) {
	struct usb_h_desc *hcd = pipe->hcd;
	
	if (pipe->ep != 0) {
		printf("Hmm, pipe endpoint wasn't 0, it was %u. What's up with that?\r\n", pipe->ep);
	}
	
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	uint16_t req_type = req->wValue >> 8;
	
	printf("Pipe 0 transfer done. Request: %u, Status: %i\r\n", req->bRequest, pipe->x.general.status);
	
	if(pipe->x.general.status < 0){
		return;
	}
	
	/* Enumeration process */
	switch(req->bRequest) {
		case USB_REQ_GET_DESC:
			if (req_type == USB_DT_DEVICE) {
				printf("Got device descriptor reply\r\n");
				wtr_usb_get_dev_desc_reply_handler(pipe);
			} else if(req_type == USB_DT_CONFIG) {
				printf("Got config descriptor reply\r\n");
				wtr_usb_get_conf_desc_reply_handler(pipe);
			}
			break;
		case USB_REQ_SET_ADDRESS:
			printf("Got set address reply\r\n");
			wtr_usb_set_address_reply_handler(pipe, req);
			break;
			
		case USB_REQ_SET_CONFIG:
			printf("Got set config reply\r\n");
			delay_ms(1000);
			wtr_usb_midi_install_driver(pipe);
			break;

		default:
			printf("Unknown request: %u\r\n", req->bRequest);
			return;
			break;
	}
}


static void wtr_usb_rh_reset_handler(struct usb_h_desc *hcd, uint32_t status, uint8_t port) {
	uint32_t speed = status; // Status indicates the USB speed.
	
	// Set up the pipe for endpoint zero.
	struct usb_h_pipe* pipe_0 = usb_h_pipe_allocate(hcd, 0, 0, EP0_SIZE_DEFAULT, 0, 0, speed, true);
	usb_h_pipe_register_callback(pipe_0, wtr_usb_rh_pipe_0_xfer_done);
	
	printf("Setup pipe_0\r\n");
	
	// Issue a request for the descriptor.
	// TODO: Maybe delay before sending requests to a low speed device.
	uint32_t request_status = send_request_get_dev_desc(pipe_0, 18);
	
	printf("Sent get desc request, status: %lu\r\n", request_status);
}

static void wtr_usb_rh_change_cb(struct usb_h_desc *hcd, uint8_t port, uint8_t ftr)
{
	int32_t status;
	switch (ftr) {
		case PORT_CONNECTION:
			status = usb_h_rh_check_status(hcd, port, ftr);
			printf("Port connection. Port: %u, Status: %li\r\n", port, status);
			wtr_usb_rh_connection_handler(hcd, status, port);
			return;
		case PORT_RESET:
			status = usb_h_rh_check_status(hcd, port, PORT_LOW_SPEED)
			? USB_SPEED_LS
			: (usb_h_rh_check_status(hcd, port, PORT_HIGH_SPEED) ? USB_SPEED_HS : USB_SPEED_FS);
			printf("Port reset. Port: %u, Status: %li\r\n", port, status);
			wtr_usb_rh_reset_handler(hcd, status, port);
			return;
		case PORT_SUSPEND:
			status = usb_h_rh_check_status(hcd, port, ftr);
			printf("Port suspend. Port: %u, Status: %li\r\n", port, status);
			return;
		// Not implemented by the root hub, but may be implemented in downstream hubs
		case PORT_OVER_CURRENT:
			printf("Port overcurrent. Port: %u\r\n", port);
			return;
		default:
			printf("Unknown port change event: %u\r\n", ftr);
			return;
	}
}

void usb_init(void)
{
	usb_h_register_callback(&USB_0_inst, USB_H_CB_ROOTHUB_CHANGE, (FUNC_PTR)wtr_usb_rh_change_cb);
	usb_h_enable(&USB_0_inst);
	printf("USB init complete\r\n");
}
