#include "hostess_status_leds.h"

static uint32_t _led_counter[3] = {};


inline static uint8_t _lookup_pin(enum hostess_status_led led) {
    switch(led)
    {
        #ifdef CONNECTED_LED_PIN
        case HTS_STATUS_LED_CONNECTION:
            return CONNECTED_LED_PIN;
        #endif

        #ifdef READ_LED_PIN
        case HTS_STATUS_LED_READ:
            return READ_LED_PIN;
        #endif

        #ifdef WRITE_LED_PIN
        case HTS_STATUS_LED_WRITE:
            return WRITE_LED_PIN;
        #endif

        default:
            return 0;
    }
}

static void _timer_task_callback(const struct timer_task *const timer_task) {
    // Step through the LEDs and see if they need to be turned off.
    for(size_t i = 0; i < HTS_STATUS_LED_COUNT; i++) {
        uint8_t pin = _lookup_pin(i);
        if(pin == 0 || _led_counter[i] == 0) continue;
        _led_counter[i]--;
        if(_led_counter[i] == 0) {
            gpio_set_pin_level(pin, false);
        }
    }
}


static struct timer_task _task = {
    .cb = &_timer_task_callback,
    .mode = TIMER_TASK_REPEAT,
};


void hostess_leds_init(struct timer_descriptor *timer) {
    _task.interval = 1;
    timer_add_task(timer, &_task);
}


void hostess_pulse_led(enum hostess_status_led led, uint32_t duration) {
    uint8_t pin = _lookup_pin(led);
    if (pin == 0) return;
    gpio_set_pin_level(pin, true);
    _led_counter[led] = duration;
}


void hostess_set_led(enum hostess_status_led led, bool state) {
    uint8_t pin = _lookup_pin(led);
    if (pin == 0) return;
    gpio_set_pin_level(pin, state);
    _led_counter[led] = 0;
}


void hostess_step_leds() {
}