The SPI Slave Synchronous Driver
================================

The serial peripheral interface (SPI) is a synchronous serial communication
interface.

SPI devices communicate in full duplex mode using a master-slave
architecture with a single master. The slave device uses the control signal
and clocks from master for reading and writing. Slave device is selected through
slave select (SS) line.

When data is read or written through the I/O writing function, the driver keeps
polling until amount of characters achieved. Also it's possible to perform
full-duplex read and write through transfer function, which process read and
write at the same time.

When SS detection is considered, a "break on SS detection" option can be enabled
to make it possible to terminate the read/write/transfer on SS desertion.

Features
--------

* Initialization/de-initialization
* Enabling/disabling
* Control of the following settings:

  * SPI mode
  * Character size
  * Data order
* Data transfer: transmission, reception and full-duplex

Applications
------------

* SPI to I2C bridge that bridges SPI commands to I2C interface.

Dependencies
------------

SPI slave capable hardware

Concurrency
-----------

N/A

Limitations
-----------

N/A

Known issues and workarounds
----------------------------

When writing data through SPI slave, the time that the data appears on data line
depends on the SPI hardware, and previous writing state, since there can be
data in output fifo filled by previous broken transmitting. The number of such
dummy/broken characters is limited by hardware. Whether these dummy/broken
characters can be flushed is also limited by hardware.

