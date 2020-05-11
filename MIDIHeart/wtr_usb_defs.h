#pragma once

#include "hal_usb_host.h"
#include "usb_protocol.h"

// Get a pointer to a USB struct type from a pointer to bytes.
#define USB_STRUCT_PTR(str, buf) ((struct str *)(buf))

// Enumeration callback function
typedef int32_t (*wtr_usb_enumeration_callback)(struct usb_h_pipe *pipe_0,
                                                struct usb_config_desc *config_descriptor_header);
// Disconnection callback function
typedef int32_t (*wtr_usb_disconnection_callback)(uint8_t port);
// Scheduled function
typedef void (*wtr_usb_scheduled_func)(void);

// Host driver structure
struct wtr_usb_host_driver {
    wtr_usb_enumeration_callback enumeration_callback;
    wtr_usb_disconnection_callback disconnection_callback;
};

// Host driver enumeration result

enum wtr_usb_host_driver_enum_result {
    WTR_USB_HD_STATUS_SUCCESS,
    WTR_USB_HD_STATUS_UNSUPPORTED,
    WTR_USB_HD_STATUS_FAILED,
};