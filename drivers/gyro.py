import time
import board
import busio
import adafruit_bno055


class GyroSensor:
    def __init__(self):
        self.i2c = i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

    def get_temperature(self):
        return self.sensor.temperature

    def get_acceleration(self):
        return self.sensor.acceleration

    def get_gyro(self) -> tuple:
        return self.sensor.gyro


# Use these lines for I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)


# Read all sensor values and verify that they follow the expected format and return reasonable values without causing errors
def read_all_sensor_values():
    while True:
        print("Temperature: {} degrees C".format(sensor.temperature))
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print("Euler angle: {}".format(sensor.euler))
        print("Quaternion: {}".format(sensor.quaternion))
        print(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
        print("Gravity (m/s^2): {}".format(sensor.gravity))
        print()

        time.sleep(1)


# Simply query gyro while checking logic analyzer to see if it makes multiple I2C reads
def check_with_logic_analyzer():
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))


# Test to find the maximum frequnecy for querying the gyroscope before any optimizations to underlying driver
def estimate_gyro_max_frequency():
    import timeit

    # Hunter's Gyro UKF Currently Relies on Gyro running at 250Hz
    def thousand_measurements():
        for i in range(1000):
            print("Gyroscope (rad/sec): {}".format(sensor.gyro))
            time.sleep(0.001)

    resulting_time = timeit.timeit(thousand_measurements)
    print(f"It took {resulting_time} to execute 1000 gyro measurements")
    frequency = 1000.0 / resulting_time
    print(f"Gyro took measurements at frequency: {frequency}")


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


if __name__ == "__main__":
    read_all_sensor_values()
