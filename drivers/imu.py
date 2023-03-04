from adafruit_blinka.agnostic import board_id

if board_id and board_id != "GENERIC_LINUX_PC":
    import board
    import busio
import adafruit_fxos8700
import adafruit_fxas21002c
from fsw.utils.constants import GYRO_RANGE
import fsw.utils.parameters as params
from fsw.drivers.device import Device, DeviceEnum

gyro_biases = (params.GYRO_BIAS_X, params.GYRO_BIAS_Y, params.GYRO_BIAS_Z)
gyro_biases_temperature_dependence = (
    params.GYRO_BIAS_DXDT,
    params.GYRO_BIAS_DYDT,
    params.GYRO_BIAS_DZDT,
)


class Gyro(Device):
    driver: adafruit_fxas21002c.FXAS21002C

    def __init__(self) -> None:
        super().__init__(DeviceEnum.gyro)

    def _connect_to_hardware(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.driver = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=GYRO_RANGE)

    def _collect_telem(self):
        return self._collect_gyro(), self._collect_temp()

    def _collect_gyro(self, corrected=False):
        gyro_data: list[float] = self.driver.gyroscope

        if corrected:
            # correcting for any bias caused by temperature differences
            # these parameters will need to be determined via TVAC testing
            delta_gyro_temp = self._collect_temp() - params.GYRO_BIAS_TEMPERATURE
            corrected_gyro_data = [0.0, 0.0, 0.0]
            for i in range(3):
                corrected_gyro_data[i] = gyro_data[i] - gyro_biases[i]
                corrected_gyro_data[i] = corrected_gyro_data[
                    i
                ] + gyro_biases_temperature_dependence[i] * float(delta_gyro_temp)

            return corrected_gyro_data

        return gyro_data

    def _collect_temp(self) -> int:
        return self.driver._read_u8(0x12)


class MagnetAccelerometer(Device):
    driver: adafruit_fxos8700.FXOS8700

    def __init__(self) -> None:
        super().__init__(DeviceEnum.magacc)

    def _connect_to_hardware(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.driver = adafruit_fxos8700.FXOS8700(i2c)

    def _collect_accelerometer(self):
        return self.driver.accelerometer

    def _collect_magnetometer(self):
        return self.driver.magnetometer

    def _collect_telem(self):
        return self._collect_magnetometer(), self._collect_accelerometer()
