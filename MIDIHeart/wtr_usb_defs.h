#pragma once


// Get a pointer to a USB struct type from a pointer to bytes.
#define USB_STRUCT_PTR(str, buf) ((struct str *)(buf))

// Enumeration callback function
typedef int32_t (*wtr_usb_enumeration_callback)(struct usb_h_pipe *pipe_0, struct usb_config_desc *config_descriptor_header);
// Disconnection callback function
typedef int32_t (*wtr_usb_disconnection_callback)(uint8_t port);


// Host driver structure
struct wtr_usb_host_driver {
    wtr_usb_enumeration_callback enumeration_callback;
    wtr_usb_disconnection_callback disconnection_callback;
};