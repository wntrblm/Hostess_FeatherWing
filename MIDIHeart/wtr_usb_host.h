#pragma once

#include "usb_protocol.h"
#include "hal_usb_host.h"
#include "wtr_usb_defs.h"


/* Initialize host functionality */
void wtr_usb_host_init(struct usb_h_desc *drv);

void wtr_usb_host_register_enumeration_callback(wtr_usb_enumeration_callback cb);


// Later
// void wtr_usb_host_register_enumeration_error_callback();