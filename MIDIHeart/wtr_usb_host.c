#include <stdio.h>
#include "usb_protocol.h"
#include "hal_delay.h"
#include "wtr_usb_host.h"
#include "hpl_usb_host.h"
#include "wtr_usb_defs.h"
#include "wtr_usb_requests.h"

/* Macros */

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


enum EnumerationState {
    ENUM_S_UNATTACHED,
    ENUM_S_ATTACHED,
    ENUM_S_POWERED,
    ENUM_S_DEFAULT,
    ENUM_S_ADDRESSED,
    ENUM_S_CONFIGURED,
    ENUM_S_READY,
};

enum EnumerationEvent {
	ENUM_E_CONNECTED,
    ENUM_E_RESET,
    ENUM_E_DEV_DESC_READ,
    ENUM_E_ADDRESS_SET,
    ENUM_E_CONF_DESC_READ,
    ENUM_E_CONFIG_SET,
	ENUM_E_FAILED,
};

#define WTR_USB_REPLY_BUFFER_LEN 255

/* Global variables used across this module. */

enum EnumerationState _enum_state = ENUM_S_UNATTACHED;
struct usb_h_desc *_drv;
struct usb_h_pipe *_pipe_0;
struct EnumerationData {
    uint8_t port;
    int32_t speed;
    uint8_t address;
	uint8_t conf_desc[WTR_USB_REPLY_BUFFER_LEN];
} _enum_data;
uint8_t req_buf[64];
uint8_t rep_buf[WTR_USB_REPLY_BUFFER_LEN];

//TODO: make this an array so multiple callbacks can be registered.
wtr_usb_enumeration_callback _enumeration_callback;

/* Private function forward declarations. */


static void _handle_enumeration_event(enum EnumerationEvent);
static int32_t _create_pipe_0();
static void _handle_device_descriptor();
static void _handle_config_descriptor();
static void _handle_pipe_0_xfer_done(struct usb_h_pipe *);
static void _handle_root_hub_change_event(struct usb_h_desc *, uint8_t, uint8_t);


/* Public functions */

void wtr_usb_host_init(struct usb_h_desc *drv) {
    usb_h_register_callback(drv, USB_H_CB_ROOTHUB_CHANGE, (FUNC_PTR)_handle_root_hub_change_event);
	usb_h_enable(drv);
    _drv = drv;
}


void wtr_usb_host_register_enumeration_callback(wtr_usb_enumeration_callback cb) {
	_enumeration_callback = cb;
}


/* Private method implementations */

/* Handles the USB enumeration process.

    This is the main heart of the enumeration process. It processes events from
    either root hub events or pipe 0 data transfers and steps through the
    enumeration process.
*/
static void _handle_enumeration_event(enum EnumerationEvent event){
    // TODO: Error handling... throughout. Maybe a macro can help.
    switch(event) {
        // Device newly connected, reset its port.
        case ENUM_E_CONNECTED:
			printf("Port %u connection.\r\n", _enum_data.port);
            usb_h_rh_reset(_drv, _enum_data.port);
            break;

        // Device reset, open a pipe to it and get its configuration descriptor.
        case ENUM_E_RESET:
            printf("Port %u reset.\r\n", _enum_data.port);
            _create_pipe_0();
            wtr_usb_send_get_dev_desc_request(_pipe_0, req_buf, rep_buf, WTR_USB_REPLY_BUFFER_LEN);
            break;
        
        // Device descriptor read. This one is a little complex so it is implemented in
        // a helper function.
        case ENUM_E_DEV_DESC_READ:
			printf("Got device descriptor reply\r\n");
            _handle_device_descriptor();
            break;

        // The device descriptor has been read and the device has a real address.
        // Reconfigure the pipe to use the new address and then read the
        // configuration descriptor header.
        case ENUM_E_ADDRESS_SET:
			printf("Address set\r\n");
	        usb_h_pipe_set_control_param(_pipe_0, _enum_data.address, 0, _pipe_0->max_pkt_size, _pipe_0->speed);
            // Wait before asking for the conf descriptor. TODO: Maybe move this to a queue.
	        delay_ms(5);
	        wtr_usb_send_get_conf_desc_request(_pipe_0, 0, req_buf, rep_buf, sizeof(usb_config_desc_t));
            break;

        // The configuration descriptor header or the full configuration descriptor has been read.
        // Process that and store it and then set the configuration. This one is a little complex
        // so it is implemented in a separate helper function.
        case ENUM_E_CONF_DESC_READ:
			printf("Got config descriptor reply\r\n");
            _handle_config_descriptor();
			break;
        
        case ENUM_E_CONFIG_SET:
			printf("Enumeration successful.\r\n");
			if(_enumeration_callback != NULL) {
				// TODO: Check return value, cleanup resources if needed.
				_enumeration_callback(_pipe_0, USB_STRUCT_PTR(usb_config_desc, &_enum_data.conf_desc));
			}
            // Call enumeration callback
            break;

        case ENUM_E_FAILED:
            // TODO: Something more here, and cleanup resources.
            printf("Enumeration failed.\r\n");
            break;
    }
}


static int32_t _create_pipe_0() {
    _pipe_0 = usb_h_pipe_allocate(_drv, 0, 0, EP0_SIZE_DEFAULT, 0, 0, _enum_data.speed, true);

    if(_pipe_0 == NULL) {
        return ERR_NO_RESOURCE;
    }

    usb_h_pipe_register_callback(_pipe_0, _handle_pipe_0_xfer_done);

    return ERR_NONE;
}

/* Handles the device descriptor response
    Make sure we have the whole descriptor and then set the address.
*/
static void _handle_device_descriptor() {
	uint32_t request_status = 0;
	// TODO: Handle error.
	
	if (_pipe_0->x.ctrl.count < sizeof(usb_dev_desc_t)) {
		// Update max packet size and re-request the descriptor.
		// Max packet size is byte 7 in the device descriptor.
		uint8_t bMaxPackSize0 = rep_buf[7];
		request_status = usb_h_pipe_set_control_param(
			_pipe_0, _pipe_0->dev, _pipe_0->ep, bMaxPackSize0, _pipe_0->speed);
			
		if(request_status != ERR_NONE) {
			printf("Failed to adjust endpoint packet size, status: %lu\r\n", request_status);
			return;
		}
		
		printf("Requesting full device descriptor.\r\n");

        // TODO: Error check
		wtr_usb_send_get_dev_desc_request(_pipe_0, req_buf, rep_buf, sizeof(usb_dev_desc_t));
		
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
    // TODO: Address assignment, error handling
     _enum_data.address = 1;
	wtr_usb_send_set_address_request(_pipe_0, _enum_data.address, req_buf);
}


static void _handle_config_descriptor() {
	uint32_t request_status = 0;
	struct usb_config_desc* conf_desc = USB_STRUCT_PTR(usb_config_desc, rep_buf);
	
	// We didn't get the whole descriptor header, fail.
	if (_pipe_0->x.ctrl.count < sizeof(struct usb_config_desc)) {
		printf("Enumeration failed, config descriptor is not the right size.\r\n");
		return;
	}
	
	// We got the descriptor header, but need to request the full descriptor.
	if(_pipe_0->x.ctrl.count == sizeof(struct usb_config_desc)) {
		uint16_t descriptor_length = conf_desc->wTotalLength;
		
		// TODO: Make sure the descriptor length isn't longer than our reply buffer.
		
		printf("Requesting full config descriptor.\r\n");
		
		wtr_usb_send_get_conf_desc_request(_pipe_0, 0, req_buf, rep_buf, descriptor_length);
		
		return;
	}

	printf("Got full descriptor. Length: %u\r\n", conf_desc->wTotalLength);	

	memcpy(_enum_data.conf_desc, (uint8_t *) conf_desc, conf_desc->wTotalLength);
	
	printf("Setting configuration\r\n");
	request_status = wtr_usb_send_set_config_request(_pipe_0, 1, req_buf);
	
	if(request_status != ERR_NONE) {
		printf("Failed to send configuration set request\r\n");
	}
}



/* HAL -> wtr adapters and such */

/* Handles root hub events from the usb host HAL.
    Triggers the appropriate enumeration event. - The goal here is to adapt from their
    interface to ours as quickly as possible.
*/
static void _handle_root_hub_change_event(struct usb_h_desc *drv, uint8_t port, uint8_t ftr)
{
    // TODO: Make sure the port matches the enumeration data port.
	int32_t status;
	switch (ftr) {
		case PORT_CONNECTION:
            status = usb_h_rh_check_status(drv, port, ftr);
            if (status == 1) {
                _enum_data.port = port;
                _handle_enumeration_event(ENUM_E_CONNECTED);
            } else {
                // TODO: Handle disconnection event.
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
		// Not implemented by the root hub, but may be implemented in downstream hubs
		case PORT_OVER_CURRENT:
			printf("Port overcurrent. Port: %u\r\n", port);
			return;
		default:
			printf("Unknown port change event: %u\r\n", ftr);
			return;
	}
}

/* Handles pipe 0 transfers from the usb host HAL.
    Triggers the appropriate enumeration event - The goal here is to adapt from their
    interface to ours as quickly as possible.
*/
static void _handle_pipe_0_xfer_done(struct usb_h_pipe *pipe) {
	struct usb_req *req = USB_STRUCT_PTR(usb_req, req_buf);
	uint16_t req_type = req->wValue >> 8;
	
	printf("Pipe 0 transfer done. Request: %u, Status: %i\r\n", req->bRequest, pipe->x.general.status);
	
	if(pipe->x.general.status < 0){
        // TODO: Idk, do something about this error?
		return;
	}
	
	/* Enumeration process */
	switch(req->bRequest) {
		case USB_REQ_GET_DESC:
			if (req_type == USB_DT_DEVICE) {
                _handle_enumeration_event(ENUM_E_DEV_DESC_READ);
			} else if(req_type == USB_DT_CONFIG) {
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
			printf("Unknown request: %u\r\n", req->bRequest);
			return;
			break;
	}
}