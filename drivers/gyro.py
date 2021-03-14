import time
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
from typing import Tuple
from utils.constants import GYRO_RANGE


class GyroSensor:  # TODO rename class and file to something more representative
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.fxos = adafruit_fxos8700.FXOS8700(i2c)
        self.fxas = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=GYRO_RANGE)

    def get_acceleration(self) -> Tuple[float, float, float]:
        return self.fxos.accelerometer  # m/s^2

    def get_gyro(self) -> Tuple[float, float, float]:
        return self.fxas.gyroscope  # rad/s

    def get_mag(self) -> Tuple[float, float, float]:
        return self.fxos.magnetometer  # microTeslas

    def get_temp(self) -> float:
        return self.fxas._read_u8(0x12)


# Test to estimate the constant bias for the gyroscopic measurements
def estimate_constant_bias():
    tuple_sum = [0.0, 0.0, 0.0]

    for i in range(1000):
        gyro_val = sensor.gyro
        tuple_sum[0] += gyro_val[0]
        tuple_sum[1] += gyro_val[1]
        tuple_sum[2] += gyro_val[2]
        time.sleep(1)

    print(f"Tuple sum: {tuple_sum}")
    tuple_avg = [(val / 1000.0) for val in tuple_sum]
    print(f"Average gyro values: {tuple_avg}")


# TODO
def estimate_white_noise():
    pass
