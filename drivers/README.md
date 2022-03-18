## Device drivers

A "driver" is a piece of software that lets a computer (i.e. the raspberry pi) talk to a connected device. Lucky for us, some of the drivers are pre-written by Adafruit. The drivers for the Gomspace P31u and AX5043 Radio chip we have to write ourselves.

The `DeviceContainer` is a container class for all the `Device` objects.
Each concrete implementation of a `Device` class represents the functionality of that hardware device. At the most basic level, every device must be able to talk to the RPi (which is done in the `connect` method) and get data/status from the sensor/device (which is done in `collect_telemetry`). For implementing a new device, at minimum you only need to implement the abstract `_collect_telem` and `_connect_to_hardware` methods. Then you interface with the device with the `collect_telemetry` and `connect` methods which wrap the above with some error handling capabilities.

## Hardware Connections

The devices connected to the Raspberry Pi are connected via the data busses as in the diagram below:

![Hardware Functional Connections](device_connections.png)


Note: This is not a comprehensive guide on how to wire the devices. For actually wiring devices together, see https://cornell.app.box.com/file/821721695559. For I/O allocation see also: https://cornell.app.box.com/file/762626657056
