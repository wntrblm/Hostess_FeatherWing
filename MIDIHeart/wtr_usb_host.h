#pragma once

#include <stdbool.h>

#include "wtr_usb_defs.h"

/* Initialize host functionality */
void wtr_usb_host_init(struct usb_h_desc *drv);

void wtr_usb_host_vbus_fault_handler();

typedef void (*wtr_usb_vbus_control_func)(bool);
void wtr_usb_host_set_vbus_control_func(wtr_usb_vbus_control_func);

void wtr_usb_host_register_driver(struct wtr_usb_host_driver driver);

void wtr_usb_host_schedule_func(wtr_usb_scheduled_func func, uint32_t delay);

typedef void (*wtr_usb_connection_callback)(uint8_t port, bool state);
void wtr_usb_host_set_connection_callback(wtr_usb_connection_callback);

bool wtr_usb_host_is_device_connected();
