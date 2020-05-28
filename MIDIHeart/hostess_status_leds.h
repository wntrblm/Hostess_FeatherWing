#pragma once

#include <atmel_start.h>


enum hostess_status_led {
    HTS_STATUS_LED_CONNECTION,
    HTS_STATUS_LED_READ,
    HTS_STATUS_LED_WRITE,
    HTS_STATUS_LED_COUNT,
};


void hostess_leds_init(struct timer_descriptor *timer);
void hostess_pulse_led(enum hostess_status_led led, uint32_t duration);
void hostess_set_led(enum hostess_status_led led, bool state);