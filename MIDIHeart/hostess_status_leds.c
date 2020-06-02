#include "hostess_status_leds.h"


/* Macros and constants */


// Milliseconds
#define STARTUP_ANIMATION_DURATION 700

#define LED_FLASH_PERIOD 100


enum led_state {
    LED_STATE_DEFAULT,
    LED_STATE_FLASHING,
};

/* Global variables */

static uint32_t _led_counter[3];
static enum led_state _led_state[3];
static struct timer_task _task;
static uint32_t _animation_timer = 0;


/* Private function forward declarations */


static uint8_t _lookup_pin(enum hostess_status_led led);
static void _timer_task_callback(const struct timer_task *const timer_task);


/* Public functions */


void hostess_leds_init(struct timer_descriptor *timer) {
    // Turn off all the LEDs.
    for(size_t i = 0; i < HTS_STATUS_LED_COUNT; i++) {
        uint8_t pin = _lookup_pin(i);
        if(pin == 0) continue;
        gpio_set_pin_level(pin, false);
    }

    _animation_timer = 0;

    _task.interval = 1;
	_task.cb = &_timer_task_callback;
	_task.mode = TIMER_TASK_REPEAT;

    timer_add_task(timer, &_task);
}


void hostess_pulse_led(enum hostess_status_led led, uint32_t duration) {
    if(_led_state[led] == LED_STATE_FLASHING) return;
    uint8_t pin = _lookup_pin(led);
    if (pin == 0) return;
    gpio_set_pin_level(pin, true);
    _led_counter[led] = duration;
}


void hostess_set_led(enum hostess_status_led led, bool state) {
	// Don't allow setting the LED during the startup animation or while it's flashing
	if(_animation_timer < STARTUP_ANIMATION_DURATION) return;
    if(_led_state[led] == LED_STATE_FLASHING) return;

    uint8_t pin = _lookup_pin(led);
	// Don't try to set an LED that doesn't exist.
    if (pin == 0) return;

    gpio_set_pin_level(pin, state);
    _led_counter[led] = 0;
}

void hostess_flash_led(enum hostess_status_led led) {
    uint8_t pin = _lookup_pin(led);
    if (pin == 0) return;
    _led_state[led] = LED_STATE_FLASHING;
    _led_counter[led] = LED_FLASH_PERIOD;
    gpio_set_pin_level(pin, true);
}

void hostess_stop_flashing_led(enum hostess_status_led led) {
    uint8_t pin = _lookup_pin(led);
    if (pin == 0) return;
    if(_led_state[led] != LED_STATE_FLASHING) return;
    _led_state[led] = LED_STATE_DEFAULT;
    _led_counter[led] = 0;
    gpio_set_pin_level(pin, false);
}


/* Private functions */

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
    // Do the startup animation. :)
    if(_animation_timer < STARTUP_ANIMATION_DURATION) {
        _animation_timer++;
        for(size_t i = 0; i < HTS_STATUS_LED_COUNT; i++) {
            uint8_t pin = _lookup_pin(i);
            if(pin == 0) continue;
            if(_animation_timer > (uint32_t)((float)(STARTUP_ANIMATION_DURATION) * i / HTS_STATUS_LED_COUNT)) {
                gpio_set_pin_level(pin, true);
            }
        }
    }
	else if(_animation_timer == STARTUP_ANIMATION_DURATION) {
		// Turn the LEDs we turned on off.
        for(size_t i = 0; i < HTS_STATUS_LED_COUNT; i++) {
            uint8_t pin = _lookup_pin(i);
            if(pin == 0) continue;
			gpio_set_pin_level(pin, false);
        }
		_animation_timer = STARTUP_ANIMATION_DURATION + 1;
	}

    // Step through the LEDs and see if they need to be turned off.
    for(size_t i = 0; i < HTS_STATUS_LED_COUNT; i++) {
        uint8_t pin = _lookup_pin(i);
        if(pin == 0 || _led_counter[i] == 0) continue;
        _led_counter[i]--;
        if(_led_counter[i] == 0) {
            if(_led_state[i] == LED_STATE_DEFAULT) {
                gpio_set_pin_level(pin, false);
            } else {
                gpio_set_pin_level(pin, !gpio_get_pin_level(pin));
                _led_counter[i] = LED_FLASH_PERIOD;
            }
        }
    }
}