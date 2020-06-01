#include "wtr_midi_debug.h"
#include <stdio.h>

static bool notes[127];

void midi_debug_on_msg(uint8_t *message) {
    switch (message[0]) {
    case 0x09:
        midi_debug_note_on(message[2]);
        break;
    case 0x08:
        midi_debug_note_off(message[2]);
        break;
    default:
        break;
    }
}

void midi_debug_note_on(uint8_t note) { notes[note] = true; }

void midi_debug_note_off(uint8_t note) { notes[note] = false; }

void midi_debug_check_stuck_notes() {
    bool stuck = false;
    for (size_t i = 0; i < 127; i++) {
        if (notes[i] == true) {
            printf("Note %i on.\r\n", i);
            stuck = true;
        }
    }
    if (!stuck) {
        // printf("No stuck notes.\r\n");
    }
}