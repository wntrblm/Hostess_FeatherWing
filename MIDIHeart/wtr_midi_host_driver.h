#pragma once

#include "wtr_queue.h"
#include "wtr_usb_defs.h"

#ifndef WTR_USB_MIDI_HOST_BUF_SIZE
#define WTR_USB_MIDI_HOST_BUF_SIZE 256
#endif

void wtr_usb_midi_host_init();

bool wtr_usb_midi_is_device_attached();

struct wtr_queue *wtr_usb_midi_get_in_queue();
