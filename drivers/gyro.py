import time
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
from typing import Tuple, List, cast
from utils.constants import GYRO_RANGE
import utils.parameters as params

gyro_biases = (params.GYRO_BIAS_X, params.GYRO_BIAS_Y, params.GYRO_BIAS_Z)
gyro_biases_temperature_dependence = (params.GYRO_BIAS_DXDT,
                                      params.GYRO_BIAS_DYDT,
                                      params.GYRO_BIAS_DZDT)


class GyroSensor:  # TODO rename class and file to something more representative

    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.fxos = adafruit_fxos8700.FXOS8700(i2c)
        self.fxas = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=GYRO_RANGE)

    def get_acceleration(self) -> Tuple[float, float, float]:
        return self.fxos.accelerometer  # m/s^2

    def get_gyro(self) -> Tuple[float, float, float]:
        return self.fxas.gyroscope  # rad/s

    def get_gyro_corrected(self) -> Tuple[float, float, float]:
        """Returns gyro data with corrections for measured bias"""
        gyro_data = self.get_gyro()
        delta_gyro_temp = self.get_temp() - params.GYRO_BIAS_TEMPERATURE
        corrected_gyro_data = [0.0, 0.0, 0.0]
        for i in range(3):
            corrected_gyro_data[i] = gyro_data[i] - gyro_biases[i]
            corrected_gyro_data[i] = corrected_gyro_data[i] + gyro_biases_temperature_dependence[i] * float(
                delta_gyro_temp)
        return_tuple: Tuple[float, float, float] = cast('Tuple[float, float, float]', tuple(corrected_gyro_data))
        return return_tuple

    def get_mag(self) -> Tuple[float, float, float]:
        return self.fxos.magnetometer  # microTeslas

    def get_temp(self) -> int:
        return self.fxas._read_u8(0x12)
