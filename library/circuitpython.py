import board
import busio
import digitalio
import usb_midi
import time
import random
from spi_device import SPIDevice

midi_out = usb_midi.ports[1]

led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

uart = busio.UART(board.TX, board.RX, baudrate=57600, timeout=0.01)

_spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
spi = SPIDevice(_spi, digitalio.DigitalInOut(board.D9), phase=1, polarity=0, baudrate=8000000)


def process_serial():
    line = uart.readline()

    if line is not None:
        led.value = True
        line = line.decode('ascii')
        print(line, end='')
        led.value = False


def _wait_for_byte(io, byte, timeout=10000000):
    start = time.monotonic_ns()
    buf = bytearray(1)

    while True:
        if time.monotonic_ns() - start > timeout:
            return False
        io.readinto(buf)
        if buf[0] == byte:
            return True


class HostessSpiCommands:
    START_REQUEST = 0x81
    REQUEST_MIDI_READ_EVENT = 0x83
    REQUEST_MIDI_WRITE_EVENT = 0x84
    REQUEST_KB_STRING = 0x85
    REQUEST_KB_EVENT = 0x86
    START_RESPONSE = 0xA1
    RESPONSE_EMPTY = 0xA2
    RESPONSE_MIDI_READ_EVENT = 0xA3
    RESPONSE_MIDI_WRITE_EVENT = 0xA4
    RESPONSE_KB_STRING = 0xA5
    RESPONSE_KB_EVENT = 0xA6


class HostessMidiPortIn:
    def __init__(self, spi):
        self.spi = spi

    def read(self):
        with self.spi as io:
            # Read MIDI byte command
            io.write(bytearray([
                HostessSpiCommands.START_REQUEST,
                HostessSpiCommands.REQUEST_MIDI_READ_EVENT
            ]))

            # Wait for response byte
            if not _wait_for_byte(io, HostessSpiCommands.START_RESPONSE):
                print("timeout")
                return None

            spi_result = bytearray(8)
            io.readinto(spi_result)

            if spi_result[0] == HostessSpiCommands.RESPONSE_EMPTY:
                return None

            if spi_result[0] != HostessSpiCommands.RESPONSE_MIDI_READ_EVENT:
                print("Got unexpected response")
                return None

            if spi_result[1] != 0:
                return spi_result[1:]

            print(spi_result)

        return None


    def write(self, data):
        with self.spi as io:
            # Write MIDI byte command
            io.write(bytearray([
                HostessSpiCommands.START_REQUEST,
                HostessSpiCommands.REQUEST_MIDI_WRITE_EVENT
            ]) + data)

            # Wait for response byte
            if not _wait_for_byte(io, HostessSpiCommands.START_RESPONSE):
                print("Didn't get start response")
                return False

            # Wait for response byte
            if not _wait_for_byte(io, HostessSpiCommands.RESPONSE_MIDI_WRITE_EVENT):
                print("Didn't get ack")
                return False

        return True


class HostessKeyboard:
    def __init__(self, spi):
        self.spi = spi

    def read(self):
        with self.spi as io:
            # Read MIDI byte command
            io.write(bytearray([
                HostessSpiCommands.START_REQUEST,
                HostessSpiCommands.REQUEST_KB_STRING,
            ]))

            # Wait for response byte
            if not _wait_for_byte(io, HostessSpiCommands.START_RESPONSE):
                return None

            spi_result = bytearray(2)
            io.readinto(spi_result)

            if spi_result[0] == HostessSpiCommands.RESPONSE_EMPTY:
                return None

            if spi_result[0] != HostessSpiCommands.RESPONSE_KB_STRING:
                print("Got unexpected response")
                return None

            return spi_result[1]

        return None


class HostessKeyboardEvents:
    def __init__(self, spi):
        self.spi = spi

    def read(self):
        with self.spi as io:
            # Read MIDI byte command
            io.write(bytearray([
                HostessSpiCommands.START_REQUEST,
                HostessSpiCommands.REQUEST_KB_EVENT,
            ]))

            # Wait for response byte
            if not _wait_for_byte(io, HostessSpiCommands.START_RESPONSE):
                return None

            spi_result = bytearray(4)
            io.readinto(spi_result)

            if spi_result[0] == HostessSpiCommands.RESPONSE_EMPTY:
                return None

            if spi_result[0] != HostessSpiCommands.RESPONSE_KB_EVENT:
                print("Got unexpected response")
                return None

            return spi_result[1:]

        return None


hostess_midi_in = HostessMidiPortIn(spi)
hostess_kb = HostessKeyboard(spi)
hostess_kb_events = HostessKeyboardEvents(spi)
upstream_midi_out = usb_midi.ports[1]

while True:
    #process_serial()

    ### MIDI IN test

    midi_event = hostess_midi_in.read()

    if midi_event is not None:
        # Send the MIDI event to the computer
        # over this Feather's USB MIDI connection
        upstream_midi_out.write(midi_event[1:])
        print(midi_event)

    ### Keyboard event test

    # event = hostess_kb_events.read()
    # if event is not None:
    #    print(event)

    ### Keyboard string test

    #char = hostess_kb.read()

    #if char is not None:
        #print(chr(char), end='')