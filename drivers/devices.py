import logging
from typing import Dict
from drivers import adc, gom, imu, rtc
from communications.satellite_radio import Radio
from drivers.device import Device


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

    def connect(self):
        result: Dict[str, bool] = {}
        for name, device in self.__dict__.items():
            device.connect()
            result.update({name: device.connected})

        if all(result):
            logging.info("All devices succefully connected!")
        else:
            logging.error(
                f"Devices not connected: {[name for name, connected in result if connected is False]}"
            )
        return result
