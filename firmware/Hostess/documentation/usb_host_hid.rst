==================
USB Host HID Mouse
==================

USB Host HID Mouse is a part of the USB Host Stack library. It provides support
for USB HID Mouse Function driver implementation. It enumerates and handles
inputs from USB HID Mouse function/interface. For more detailed definition and
description about this class, user can refer to
<Device Class Definition for Human Interface Devices (HID), Vision 1.11>.

The driver is registered to the USB Host Core Driver, so that when USB Mouse
Device or USB Composite Device with Mouse Function is connected, it can be
enabled to drive the mouse function.

Features
--------

* Initialization/de-initialization.
* Notifications about button state change for HID Mouse.
* Notifications about mouse move or scroll for HID Mouse.

Applications
------------

* Detect and operate on a HID Mouse.

Dependencies
------------

* USB Host Driver
* USB Host Stack Core
* USB Protocol HID

Limitations
-----------

N/A



