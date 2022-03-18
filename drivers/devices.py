import logging
from typing import Dict
from drivers import adc, gom, imu, rtc
from communications.satellite_radio import Radio
from drivers.device import Device, DeviceEnum


class DeviceContainer:
    """Container class allowing the easy initialization to all Devices on the spacecraft"""

    __dict__: Dict[str, Device]

    def __init__(self) -> None:
        self.gom = gom.Gomspace()
        self.radio = Radio()
        self.gyro = imu.Gyro()
        self.magacc = imu.MagnetAccelerometer()
        self.adc = adc.ADC(self.gyro)
        self.rtc = rtc.RTC()

    def connect(self) -> Dict[DeviceEnum, bool]:
        """Tries connecting to all hardware devices and return a dict of the device name, and a bool of whether the
         RPi is able to talk with the device"""
        result: Dict[DeviceEnum, bool] = {}
        for device in self.__dict__.values():
            device.connect()
            result.update({device.name: device.connected})

        if all(result.values()):
            logging.info("All devices succefully connected!")
        else:
            logging.error(
                f"Devices not connected: {[name.value for name, connected in result.items() if connected is False]}"
            )
        return result
