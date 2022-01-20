import logging
from typing import List
from drivers import adc, gom, imu, rtc
from communications.satellite_radio import Radio
from drivers.device import Device


class DeviceContainer:
    """Container class allowing the easy initialization to all Devices on the spacecraft"""

    def __init__(self) -> None:
        self.gom = gom.Gomspace()
        self.radio = Radio()
        self.gyro = imu.Gyro()
        self.magacc = imu.MagnetAccelerometer()
        self.adc = adc.ADC()
        self.rtc = rtc.RTC()

        self.devices: List[Device] = [
            self.gom,
            self.radio,
            self.gyro,
            self.magacc,
            self.adc,
            self.rtc,
        ]

    def connect(self):
        result: List[bool] = []
        for device in self.devices:
            device.connect()
            result.append(device.connected)

        if all(result):
            logging.info("All devices succefully connected!")

        return result

    def device_status(self):
        return {device.name: device.connected for device in self.devices}
