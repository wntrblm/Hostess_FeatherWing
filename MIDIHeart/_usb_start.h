/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#ifndef USB_DEVICE_MAIN_H
#define USB_DEVICE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "usbhc.h"

extern bool midi_notes[127];

extern struct usbhc_driver USB_HOST_CORE_0_inst;

void USB_HOST_CORE_0_example(void);

#include "hidhf_mouse.h"

extern struct hidhf_mouse USB_HOST_HID_MOUSE_0_inst;

void USB_HOST_HID_MOUSE_0_example(void);

/**
 * \berif Initialize USB
 */
void usb_init(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_DEVICE_MAIN_H
