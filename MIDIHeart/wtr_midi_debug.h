#include <stdbool.h>
#include <stdint.h>

void midi_debug_on_msg(uint8_t *message);
void midi_debug_note_on(uint8_t note);
void midi_debug_note_off(uint8_t note);
void midi_debug_check_stuck_notes();
