# Winterbloom Hostess FeatherWing

![Picture of the FeatherWing](hostess.jpg)

**:warning: Hostess is *extremely* experimental at this stage. It is not ready for prime time. I welcome folks who want to help out and poke around, but please do not expect anything here to work or make sense. Please reach out to me at me@thea.codes if you're interesting in helping out.**

Hostess is a [FeatherWing](https://learn.adafruit.com/adafruit-feather/featherwings) that provides a USB host co-processor. The FeatherWing handles all USB Host related tasks, including enumeration **and running the device drivers**, and communicates with the primary processor over [SPI](https://learn.adafruit.com/circuitpython-basics-i2c-and-spi/spi-devices). This means the primary processor can communicate with USB devices with minimal software and CPU time.

For example, a [CircuitPython](https://circuitpython.org)-based board could communicate with a USB MIDI device with minimal code:

```python
_spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
spi = SPIDevice(_spi, digitalio.DigitalInOut(board.D9), phase=1, polarity=0, baudrate=8000000)
hostess_midi_in = HostessMidiPortIn(spi)

while True:
    midi_event = hostess_midi_in.read()

    if midi_event is not None:
        print(midi_event)
```

Presently, Hostess has basic support for the following USB devices:

* MIDI
* Keyboard

And in-progress support for the following devices:

* PS4 controller

And I hope to add support for these devices eventually:

* USB Serial (CDC)
* Generic HID Gamepad
* Mouse

## Hardware

Hostess's [hardware](hardware) is based around the [Atmel SAMD21](https://www.microchip.com/wwwproducts/en/ATsamd21g18), a low-cost 32-bit ARM microcontroller with USB host capabilities. It incorporates a USB current-limiting switch to prevent devices from drawing too much current and provides the proper amount of capacitance for powering USB devices. This makes it a bit more robust compared to using an OTG adapter with existing development boards.

Hostess's hardware is the most complete part of the project. PCBs can be manufactured at OSHPark or other fabs and all of the components on the BOM are readily available at Mouser and DigiKey.

To upload the firmware and debug the device, you'll need a [SOICBite](https://github.com/SimonMerrett/SOICbite) and a J-Link or similar.

## Firmware

Hostess's [firmware](firmware) is written in C using [Atmel START](https://start.atmel.com/). Presently building it requires [Atmel Studio 7](https://www.microchip.com/mplab/avr-support/atmel-studio-7), though I'd like to switch to a Makefile.

## Library

Communicating with Hostess is the most incomplete part of the project. Presently, there is just an [example file](library/circuitpython.py) on how to communicate with it from CircuitPython. I'd like to make this a complete library as well as create an Arduino library.

## License and contributing

The code here is available under the [MIT License](firmware/LICENSE), the hardware designs are available under [CC BY-SA 4.0](hardware/LICENSE). I welcome contributors, please read the [Code of Conduct](CODE_OF_CONDUCT.md) first. :)
