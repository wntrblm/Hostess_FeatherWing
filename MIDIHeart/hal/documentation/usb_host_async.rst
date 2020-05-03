The USB Host Asynchronous Driver
==================================

Universal Serial Bus (USB) is an industry standard that defines the cables,
connectors and communication protocols used in a bus for connection,
communication, and power supply between computers and electronic devices.

The USB host driver provides necessary APIs to support USB states and
USB data flow. So that the USB Device enumeration, class and vendor support can
be implemented base on it. The driver is asynchronous, which means that all USB
data processing is done in callbacks.

To recognized a device, a USB host must handle a subset of the USB events. The
USB host should turn on VBus supply, monitor root hub events, build up control
communication to request descriptors of attached device. An application or upper
stack that uses the USB host driver should have its descriptors buffer prepared,
issue the control requests, analyze the returned descriptors and handle them
correctly.

Usually, a USB host application that can enumerates USB device may use the
following sequence:

* Initialize
* Register callback and handle root hub events, where:

  * On connection, issue USB Reset
  * On USB Reset end, initialize pipe 0, register request end callback
    and issue *GetDeviceDescriptor* request
  * On disconnection, free all allocated pipes

* In pipe 0 request done handling callback:

  * On *GetDeviceDescriptor* OK, modify pipe 0 endpoint size according to the
    descriptor returned, issue *SetAddress* request
  * On *SetAddress* request OK, issue *GetConfigurationDescriptor* request
  * On *GetConfigurationDescriptor* request OK, create pipes based on returned
    configuration descriptor; issue *SetConfigure* request

* Enable
* After the *SetConfigure* request is done correctly, the created pipes should
  be OK to communicate with the USB device

Features
--------

* Initialization/de-initialization
* Enabling/disabling
* USB root hub control

  * Attach/detach detection
  * Reset
  * Suspend/resume
  * Connection speed detection
* USB host frame number and micro frame number status
* Callback management for:

  * Start of Frame (SOF)
  * Root hub events
* Pipes management:

  * Pipe allocation/free
  * Control pipe request
  * Bulk, interrupt and ISO pipe transfer
  * Pipe callback management for transfer done

Applications
------------

USB Host stack, to manage root hub, issue control requests and process
data.

Dependencies
------------

* USB host capable hardware
* 48MHz clock for low-speed and full-speed and 480MHz clock for high-speed

Concurrency
-----------

N/A
