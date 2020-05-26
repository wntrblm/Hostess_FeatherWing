#pragma once

#include <stdbool.h>

#include "wtr_usb_defs.h"

/* Initialize host functionality */
void wtr_usb_host_init(struct usb_h_desc *drv);

void wtr_usb_host_register_driver(struct wtr_usb_host_driver driver);

void wtr_usb_host_schedule_func(wtr_usb_scheduled_func func, uint32_t delay);

bool wtr_usb_host_is_device_connected();
