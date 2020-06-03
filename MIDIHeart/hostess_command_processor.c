#include "hostess_command_processor.h"
#include "hostess_status_leds.h"


enum stream_parser_state {
    PARSER_STATE_IDLE,
    PARSER_STATE_COMMAND_START,
    PARSER_STATE_COMMAND_BYTES,
};


static uint8_t _response_out_buf[16];
static struct wtr_queue *_midi_in_queue;
static struct wtr_queue *_midi_out_queue;
static struct wtr_queue *_keystring_queue;
static struct wtr_queue *_key_event_queue;

static struct {
    enum stream_parser_state state;
    uint8_t command_byte;
    uint8_t in_buf[16];
    uint8_t in_buf_offset;
    uint8_t command_data_bytes;
} _stream_parser_data;


void hostess_set_queue(enum hostess_queue queue_name, struct wtr_queue *queue) {
    switch(queue_name) {
        case HOSTESS_QUEUE_MIDI_IN:
            _midi_in_queue = queue;
            break;

        case HOSTESS_QUEUE_MIDI_OUT:
            _midi_out_queue = queue;
            break;
        
        case HOSTESS_QUEUE_KB_EVENT:
            _key_event_queue = queue;
            break;

        case HOSTESS_QUEUE_KB_STRING:
            _keystring_queue = queue;
            break;

        default:
            ASSERT(false);
            break;
    }
}


struct hostess_command_result hostess_process_command(enum hostess_command command, uint8_t *in_buf, uint8_t in_buf_len, struct io_descriptor *io) {
    switch (command) {
        case HOSTESS_CMD_REQUEST_READ_MIDI_EVENT:
            // Start response start byte and type
            _response_out_buf[0] = HOSTESS_CMD_START_RESPONSE;

            if (wtr_queue_is_empty(_midi_in_queue)) {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_EMPTY;
                io_write(io, _response_out_buf, 2);
            } else {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_READ_MIDI_EVENT;
                // Pop the event on the queue to the SPI out buffer,
                // but leave two bytes at the first from the
                // response start and response type.
                wtr_queue_pop(_midi_in_queue, _response_out_buf + 2);
                io_write(io, _response_out_buf, 6);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        case HOSTESS_CMD_REQUEST_WRITE_MIDI_EVENT:
            // If we just got the command, wait for 4 data bytes.
            if(in_buf_len != 4) {
                return (struct hostess_command_result){
                    .state = HOSTESS_CMD_RESULT_NEEDS_DATA,
                    .remaining_bytes = 4,
                };
            }
            // We got a full event, write it to the MIDI out queue
            else {
                wtr_queue_push(_midi_out_queue, in_buf);

                // And send along the ack response
                _response_out_buf[0] = HOSTESS_CMD_START_RESPONSE;
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_WRITE_MIDI_EVENT;
                io_write(io, _response_out_buf, 2);
                hostess_pulse_led(HTS_STATUS_LED_WRITE, LED_PULSE_DURATION);
            }
            break;

        case HOSTESS_CMD_REQUEST_KB_STRING:
            // Start response start byte and type
            _response_out_buf[0] = HOSTESS_CMD_START_RESPONSE;

            if (wtr_queue_is_empty(_keystring_queue)) {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_EMPTY;
                io_write(io, _response_out_buf, 2);
            } else {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_KB_STRING;
                wtr_queue_pop(_keystring_queue, _response_out_buf + 2);
                io_write(io, _response_out_buf, 3);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        case HOSTESS_CMD_REQUEST_KB_EVENT:
            // Start response start byte and type
            _response_out_buf[0] = HOSTESS_CMD_START_RESPONSE;

            if (wtr_queue_is_empty(_key_event_queue)) {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_EMPTY;
                io_write(io, _response_out_buf, 2);
            } else {
                _response_out_buf[1] = HOSTESS_CMD_RESPONSE_KB_EVENT;
                wtr_queue_pop(_key_event_queue, _response_out_buf + 2);
                io_write(io, _response_out_buf, 5);
                hostess_pulse_led(HTS_STATUS_LED_READ, LED_PULSE_DURATION);
            }
            break;

        default:
            printf("Unknown SPI request: 0x%02x\r\n", command);
            break;
    }

    return (struct hostess_command_result){
        .state = HOSTESS_CMD_RESULT_DONE
    };
};


void hostess_parse_byte_stream(struct io_descriptor *io) {
    int32_t recv_count = io_read(io, _stream_parser_data.in_buf + _stream_parser_data.in_buf_offset, 1);

    if (recv_count == 0) return;
        
    switch(_stream_parser_data.state) {
        case PARSER_STATE_IDLE:
            if(_stream_parser_data.in_buf[0] == HOSTESS_CMD_START_REQUEST) {
                _stream_parser_data.state = PARSER_STATE_COMMAND_START;
            }
            break;
            
        case PARSER_STATE_COMMAND_START:
            _stream_parser_data.command_byte = _stream_parser_data.in_buf[0];
            struct hostess_command_result result = hostess_process_command(_stream_parser_data.command_byte, NULL, 0, io);
            
            // If the command has data, the move our buf write head ahead one
            // so we can keep the command in buf[0] and add the data after.
            if(result.state == HOSTESS_CMD_RESULT_NEEDS_DATA) {
                _stream_parser_data.in_buf_offset = 0;
                _stream_parser_data.command_data_bytes = result.remaining_bytes;
                _stream_parser_data.state = PARSER_STATE_COMMAND_BYTES;
            }
            // If there's no data for the command, set the state back to IDLE.
            else {
                _stream_parser_data.state = PARSER_STATE_IDLE;
            }
            break;

        case PARSER_STATE_COMMAND_BYTES:
            _stream_parser_data.in_buf_offset++;

            if(_stream_parser_data.in_buf_offset == _stream_parser_data.command_data_bytes) {
                hostess_process_command(_stream_parser_data.command_byte, _stream_parser_data.in_buf, _stream_parser_data.command_data_bytes, io);
                _stream_parser_data.state = PARSER_STATE_IDLE;
                _stream_parser_data.in_buf_offset = 0;
            }
            break;

        default:
            break; 
    }
};