#pragma once

#include <hal_io.h>
#include "wtr_queue.h"

// Milliseconds
#define LED_PULSE_DURATION 30


enum hostess_command_result_state {
	HOSTESS_CMD_RESULT_DONE,
	HOSTESS_CMD_RESULT_NEEDS_DATA,
};

struct hostess_command_result {
    enum hostess_command_result_state state;
    uint8_t remaining_bytes;
};

enum hostess_command {
    HOSTESS_CMD_START_REQUEST = 0x81,
    HOSTESS_CMD_REQUEST_READ_MIDI_EVENT = 0x83,
    HOSTESS_CMD_REQUEST_WRITE_MIDI_EVENT = 0x84,
    HOSTESS_CMD_REQUEST_KB_STRING = 0x85,
    HOSTESS_CMD_REQUEST_KB_EVENT = 0x86,
    HOSTESS_CMD_START_RESPONSE = 0xA1,
    HOSTESS_CMD_RESPONSE_EMPTY = 0xA2,
    HOSTESS_CMD_RESPONSE_READ_MIDI_EVENT = 0xA3,
    HOSTESS_CMD_RESPONSE_WRITE_MIDI_EVENT = 0xA4,
    HOSTESS_CMD_RESPONSE_KB_STRING = 0xA5,
    HOSTESS_CMD_RESPONSE_KB_EVENT = 0xA6,
};

enum hostess_queue {
    HOSTESS_QUEUE_MIDI_IN,
    HOSTESS_QUEUE_MIDI_OUT,
    HOSTESS_QUEUE_KB_EVENT,
    HOSTESS_QUEUE_KB_STRING,
};

void hostess_set_queue(enum hostess_queue queue_name, struct wtr_queue *queue);

/* Process a single command.

This function's behavior solely depends on its input parameters and the state of the
respective queues. Generally, this is invoked by the stateful stream parser invoked
through hostess_parse_stream_byte();

The result tells whether the command is completely done (HOSTESS_CMD_RESULT_DONE) or
requires more data bytes (HOSTESS_CMD_RESULT_NEEDS_DATA) and how many bytes are needed.
*/
struct hostess_command_result hostess_process_command(enum hostess_command command, uint8_t *in_buf, uint8_t in_buf_len, struct io_descriptor *io);


/* Process a byte stream that contains hostess commands.

This should be called for every byte that comes in from the command communication pipe
(for example, for every SPI byte received). It will read one byte at a time from the
io descriptor and then handle figuring out which command to execute once a full command
is received and hand it off to hostess_process_command to execute the command.
hostess_process_command will write the response to the io descriptor.

This method is *stateful*- it has internal state that's used to track the parser
status.

*/
void hostess_parse_byte_stream(struct io_descriptor *io);