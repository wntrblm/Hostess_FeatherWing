#pragma once

#include "wtr_queue.h"
#include "wtr_usb_defs.h"

enum hid_keyboard_event_type {
    HID_KB_EVENT_UNKNOWN,
    HID_KB_EVENT_KEY_PRESS,
    HID_KB_EVENT_KEY_RELEASE,
} __attribute__((__packed__));

struct hid_keyboard_event {
    enum hid_keyboard_event_type type;
    uint8_t keycode;
    uint8_t modifiers;
};

void wtr_usb_hid_keyboard_init();

struct wtr_queue *wtr_usb_hid_keyboard_get_event_queue();
struct wtr_queue *wtr_usb_hid_keyboard_get_keystring_queue();