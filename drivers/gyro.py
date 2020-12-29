import time
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c


class GyroSensor:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.fxos = adafruit_fxos8700.FXOS8700(self.i2c)
        self.fxas = adafruit_fxas21002c.FXAS21002C(self.i2c)

    def get_acceleration(self) -> tuple:
        return self.fxos.accelerometer  # m/s^2

    def get_gyro(self) -> tuple:
        return self.fxas.gyroscope  # rad/s

    def get_mag(self) -> tuple:
        return self.fxos.magnetometer  # microTeslas


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

# User these lines for UART
# uart = busio.UART(board.TX, board.RX)
# sensor = adafruit_bno055.BNO055_UART(uart)


# if __name__ == "__main__":
#    read_all_sensor_values()
