The SPI Slave Asynchronous Driver
=================================

The serial peripheral interface (SPI) is a synchronous serial communication
interface.

SPI devices communicate in full duplex mode using a master-slave
architecture with a single master. The slave device uses the control signal
and clocks from master for reading and writing. Slave device is selected through
slave select (SS) line.

When data is written through the I/O writing function, the data buffer pointer
and its size is logged internally by the driver. The data is sent character by
character in background interrupts. When all data in the buffer is sent,
callback is invoked to notify that it's done.

When the driver is enabled, the characters shifted in will be filled to a
receiving ring-buffer, then the available data can be read out through the I/O
reading function. On each characters' reception a callback is invoked.

In some cases, the SS deactivation is considered, it's notified through a
completion callback with status code zero. When status code is lower than zero,
it indicates error.

Features
--------

* Initialization/de-initialization
* Enabling/disabling
* Control of the following settings:

  * SPI mode
  * Character size
  * Data order
* Data transfer: transmission, reception and full-duplex
* Callbacks management on:

  * Transmission done
  * Received character
  * Completion by SS detection or error

Applications
------------

* SPI to I2C bridge that bridges SPI commands to I2C interface.
* SPI WIFI module

Dependencies
------------

SPI slave capable hardware

Concurrency
-----------

N/A

Limitations
-----------

When received data is not read promptly, the ring-buffer is used out. In this
case the oldest characters will be overwritten by the newest ones.

Known issues and workarounds
----------------------------

When writing data through SPI slave, the time that the data appears on data line
depends on the SPI hardware, and previous writing state, since there can be
data in output fifo filled by previous broken transmitting. The number of such
dummy/broken characters is limited by hardware. Whether these dummy/broken
characters can be flushed is also limited by hardware.

