from os import popen
from time import time, sleep
from typing import Dict, Union, NamedTuple, Tuple
import logging

import numpy as np
import psutil
from adafruit_blinka.agnostic import board_id
from uptime import uptime

from drivers.power.power_structs import eps_hk_t
from telemetry.sensor import SynchronousSensor
from utils import constants
from utils.constants import DB_FILE
from utils.constants import (
    MAX_GYRO_RATE,
    BATTERY_VOLTAGE,
    SUN_CURRENT,
    SYSTEM_CURRENT,
    BATT_MODE,
    PPT_MODE,
)

# from utils.db import GyroModel
from utils.db import TelemetryModel, create_sensor_tables_from_path
from utils.exceptions import (
    PiSensorError,
    PressureError,
    GomSensorError,
    GyroError,
    ThermocoupleError,
)


class ImuResult(NamedTuple):
    # TODO: find better implementation of data storage than dicts
    # gyro values (degrees/second)
    gyrox: float
    gyroy: float
    gyroz: float

    # magnetometer values (microTesla?)
    magx: float
    magy: float
    magz: float

    # accelerometer values (m/s^2)
    accx: float
    accy: float
    accz: float


def moving_average(x, w):
    """x: 1-D data, w: number of samples to average over."""
    return np.convolve(x, np.ones(w), "valid") / w


class GomSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.hk = eps_hk_t()  # HouseKeeping data: eps_hk_t struct

    def poll(self):
        super().poll()
        self.hk = self._parent.devices.gom.collect_telem()


class GyroSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.rot: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s
        self.mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # microTesla
        self.acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2
        self.tmp: int = int()  # deg C

    def poll(self):
        super().poll()
        self.rot, self.tmp = self._parent.devices.gyro.collect_telem()
        self.mag, self.acc = self._parent.devices.magacc.collect_telem()

        # self.result = ImuResult(*self.rot, *self.mag, *self.acc)

    def poll_smoothed(self, freq=10, duration=1, samples=10):
        # poll and smooth gyro data

        n_data = freq * duration
        data = []
        for _ in range(n_data):
            data.append(self._parent.devices.gyro._collect_gyro())
            sleep(1.0 / freq)

        data = np.asarray(data).T

        smoothed = np.empty(3)

        # smooth data using convolution
        smoothed[0] = moving_average(data[0], samples)
        smoothed[1] = moving_average(data[1], samples)
        smoothed[2] = moving_average(data[2], samples)

        return smoothed

    def get_rot(self):
        return self.rot

    def get_acc(self):
        return self.acc

    def get_mag(self):
        return self.mag

    def write(self):
        # TODO
        pass
        # gyro_tuple = (self.rot, self.acc, self.mag, self.tmp, self.poll_time)
        # gyro_model = GyroModel.from_tuple(gyro_tuple)
        #
        # try:
        #     session = self._parent.create_session()
        #     session.add(gyro_model)
        #     session.commit()
        # finally:
        #     session.close()


class PressureSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.pressure = float()  # pressure, psi

    def poll(self):
        super().poll()
        if self._parent.devices.adc is not None:
            self.pressure = self._parent.devices.adc.read_pressure()


class ThermocoupleSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.tmp = float()  # Fuel tank temperature, deg C

    def poll(self):
        super().poll()
        if self._parent.devices.adc is not None:
            self.tmp = self._parent.devices.adc.read_temperature()


class PiSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.cpu: int = int()  # can be packed as uint8
        self.ram: int = int()  # can be packed as uint8
        self.disk: int = int()  # can be packed as uint8
        self.boot_time: float = float()
        self.up_time: int = int()
        self.tmp: float = float()  # can be packed as a short
        self.all: Tuple[int, int, int, float, int, float, float] = tuple()

    def poll(self):
        super().poll()
        self.cpu = int(psutil.cpu_percent())
        self.ram = int(psutil.virtual_memory().percent)
        self.disk = int(psutil.disk_usage("/").percent)
        self.boot_time = psutil.boot_time()
        self.up_time = int(uptime())
        if board_id and board_id != "GENERIC_LINUX_PC":
            self.tmp = float(popen("vcgencmd measure_temp").readline().strip()[5:-2])
        self.all = (
            self.cpu,
            self.ram,
            self.disk,
            self.boot_time,
            self.up_time,
            self.tmp,
            self.poll_time,
        )


class RtcSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.rtc_time = int()
        self.rtc_temp = int()

    def poll(self):
        super().poll()
        self.rtc_time, self.rtc_temp = self._parent.devices.rtc.collect_telem()


class OpNavSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.quat = tuple() * 4
        self.pos = tuple() * 3
        self.acq_time = float()

    def poll(self):
        raise NotImplementedError  # TODO: Opnav interface


class Telemetry(SynchronousSensor):
    def __init__(self, parent):
        # The purpose of the parent object is to ensure that only one object is defined for each sensor/component
        # So almost all calls to sensors will be made through self._parent.<insert sensor stuff here>
        super().__init__(parent)

        self.gom = GomSensor(parent)
        self.gyr = GyroSensor(parent)
        self.prs = PressureSensor(parent)
        self.thm = ThermocoupleSensor(parent)
        self.rpi = PiSensor(parent)
        self.rtc = RtcSensor(parent)
        self.opn = OpNavSensor(parent)

        self.sensors = [self.gom, self.gyr, self.prs, self.thm, self.rpi, self.rtc]

        create_session = create_sensor_tables_from_path(
            DB_FILE
        )  # instantiate DB session
        self.session = create_session()  # instantiate DB session

    def poll(self):
        # polls every sensor for the latest telemetry that can be accessed
        # by the rest of the software.
        super().poll()
        for sensor in self.sensors:
            sensor.poll()

    def sensor_check(self):
        """Makes sure that the values returned by the sensors are reasonable"""

        # if data is over an hour old, poll new data
        if (time() - self.poll_time) > 3600:
            self.poll()

        if any([abs(i) > MAX_GYRO_RATE for i in self.gyr.rot]):
            logging.error("Gyro not functioning properly")
            raise GyroError(f"Unreasonable gyro values: {self.gyr.rot}")

        if self.prs.pressure < 0 or self.prs.pressure > 2000:
            logging.error("Pressure sensor not functioning properly")
            raise PressureError(f"Unreasonable pressure: {self.prs.pressure}")

        if self.thm.tmp < -200 or self.thm.tmp > 200:
            logging.error("Thermocouple not functioning properly")
            raise ThermocoupleError(
                f"Unreasonable fuel tank temperature: {self.thm.tmp}"
            )

        if self.gom.hk.vbatt < 5500 or self.gom.hk.vbatt > 8500:
            logging.error("Gom HK not functioning properly")
            raise GomSensorError(f"Unreasonable battery voltage: {self.gom.hk.vbatt}")

        if any(i < 0 for i in self.rpi.all):
            logging.error("RPi sensors not functioning properly")
            raise PiSensorError

    def standard_packet(self):
        if (
            time() - self.poll_time
        ) > 60 * 60:  # if the latest data is over an hour old, poll new data
            self.poll()

        return (
            self.rtc.rtc_time,
            self.opn.pos[0],
            self.opn.pos[1],
            self.opn.pos[2],
            self.opn.quat[0],
            self.opn.quat[1],
            self.opn.quat[2],
            self.opn.quat[3],
            self.gom.hk.temp[0],
            self.gom.hk.temp[1],
            self.gom.hk.temp[2],
            self.gom.hk.temp[3],
            self.gyr.tmp,
            self.thm.tmp,
            self.gom.hk.curin[0],
            self.gom.hk.curin[1],
            self.gom.hk.curin[2],
            self.gom.hk.vboost[0],
            self.gom.hk.vboost[1],
            self.gom.hk.vboost[2],
            self.gom.hk.cursys,
            self.gom.hk.vbatt,
            self.prs.pressure,
        )

    def standard_packet_dict(self):
        return {
            "rtc_time": self.rtc.rtc_time,
            "position_x": 1,  # FIXME Opnav results interface
            "position_y": 2,
            "position_z": 3,
            "attitude_1": 4,
            "attitude_2": 5,
            "attitude_3": 6,
            "attitude_4": 7,
            "hk_temp_1": self.gom.hk.temp[0],  # ushort
            "hk_temp_2": self.gom.hk.temp[1],  # ushort
            "hk_temp_3": self.gom.hk.temp[2],  # ushort
            "hk_temp_4": self.gom.hk.temp[3],  # ushort
            "gyro_temp": self.gyr.tmp,
            "thermo_temp": self.thm.tmp,
            "curin_1": self.gom.hk.curin[0],  # ushort
            "curin_2": self.gom.hk.curin[1],  # ushort
            "curin_3": self.gom.hk.curin[2],  # ushort
            "vboost_1": self.gom.hk.vboost[0],  # ushort
            "vboost_2": self.gom.hk.vboost[1],  # ushort
            "vboost_3": self.gom.hk.vboost[2],  # ushort
            "cursys": self.gom.hk.cursys,  # ushort
            "vbatt": self.gom.hk.vbatt,  # ushort
            "prs_pressure": self.prs.pressure,
        }

    def minimal_packet(self):
        return {
            BATTERY_VOLTAGE: self.gom.hk.vbatt,
            SUN_CURRENT: self.gom.hk.cursun,
            SYSTEM_CURRENT: self.gom.hk.cursys,
            BATT_MODE: self.gom.hk.battmode,
            PPT_MODE: self.gom.hk.pptmode,
        }

    def detailed_packet_dict(self) -> Dict[str, Union[int, float]]:
        """Returns a dictionary of every possible data point we'd want to downlink. The implementation of this is
        barbaric at best, but works. Something like a NamedTuple would work fantastic here, but would require
        overhaul of quite a bit of FSW"""

        gx, gy, gz = self.gyr.get_rot()  # rot
        ax, ay, az = self.gyr.get_acc()  # acc
        bx, by, bz = self.gyr.get_mag()  # mag
        return {
            constants.TIME: self.poll_time,
            constants.VBOOST_1: self.gom.hk.vboost[0],
            constants.VBOOST_2: self.gom.hk.vboost[1],
            constants.VBOOST_3: self.gom.hk.vboost[2],
            constants.BATTERY_VOLTAGE: self.gom.hk.vbatt,
            constants.CURRENT_IN_1: self.gom.hk.curin[0],
            constants.CURRENT_IN_2: self.gom.hk.curin[1],
            constants.CURRENT_IN_3: self.gom.hk.curin[2],
            constants.CURSUN: self.gom.hk.cursun,
            constants.SYSTEM_CURRENT: self.gom.hk.cursys,
            constants.RESERVED1: self.gom.hk.reserved1,
            constants.CUROUT1: self.gom.hk.curout[0],
            constants.CUROUT2: self.gom.hk.curout[1],
            constants.CUROUT3: self.gom.hk.curout[2],
            constants.CUROUT4: self.gom.hk.curout[3],
            constants.CUROUT5: self.gom.hk.curout[4],
            constants.CUROUT6: self.gom.hk.curout[5],
            constants.OUTPUTS: int(
                str(self.gom.hk.output[:]).replace(",", "").replace(" ", "")[1:-1], 2
            ),
            constants.LATCHUPS1: self.gom.hk.latchup[0],
            constants.LATCHUPS2: self.gom.hk.latchup[1],
            constants.LATCHUPS3: self.gom.hk.latchup[2],
            constants.LATCHUPS4: self.gom.hk.latchup[3],
            constants.LATCHUPS5: self.gom.hk.latchup[4],
            constants.LATCHUPS6: self.gom.hk.latchup[5],
            constants.WDT_TIME_LEFT_I2C: self.gom.hk.wdt_i2c_time_left,
            constants.WDT_TIME_LEFT_GND: self.gom.hk.wdt_gnd_time_left,
            constants.WDT_COUNTS_I2C: self.gom.hk.counter_wdt_i2c,
            constants.WDT_COUNTS_GND: self.gom.hk.counter_wdt_gnd,
            constants.GOM_BOOTS: self.gom.hk.counter_boot,
            constants.GOM_BOOTCAUSE: self.gom.hk.bootcause,
            constants.GOM_BATTMODE: self.gom.hk.battmode,
            constants.HK_TEMP_1: self.gom.hk.temp[0],
            constants.HK_TEMP_2: self.gom.hk.temp[1],
            constants.HK_TEMP_3: self.gom.hk.temp[2],
            constants.HK_TEMP_4: self.gom.hk.temp[3],
            constants.GOM_PPT_MODE: self.gom.hk.pptmode,
            constants.RESERVED2: self.gom.hk.reserved2,
            constants.RTC_TIME: self.rtc.rtc_time,
            constants.RPI_CPU: self.rpi.cpu,
            constants.RPI_RAM: self.rpi.ram,
            constants.RPI_DSK: self.rpi.disk,
            constants.RPI_TEMP: self.rpi.tmp,
            constants.RPI_BOOT: self.rpi.boot_time,
            constants.RPI_UPTIME: self.rpi.up_time,
            constants.GYROX: gx,
            constants.GYROY: gy,
            constants.GYROZ: gz,
            constants.ACCX: ax,
            constants.ACCY: ay,
            constants.ACCZ: az,
            constants.MAGX: bx,
            constants.MAGY: by,
            constants.MAGZ: bz,
            constants.GYRO_TEMP: self.gyr.tmp,
            constants.THERMOCOUPLE_TEMP: self.thm.tmp,
            constants.PROP_TANK_PRESSURE: self.prs.pressure,
            constants.POS_X: 0,
            constants.POS_Y: 1,
            constants.POS_Z: 2,
            constants.ATT_1: 3,
            constants.ATT_2: 4,
            constants.ATT_3: 5,
            constants.ATT_4: 6,
        }

    def write_telem(self):
        try:
            time_polled = time()
            gx, gy, gz = self.gyr.get_rot()  # rot
            ax, ay, az = self.gyr.get_acc()  # acc
            bx, by, bz = self.gyr.get_mag()  # mag

            telemetry_data = TelemetryModel(
                time_polled=time_polled,
                GOM_vboost1=self.gom.hk.vboost[0],
                GOM_vboost2=self.gom.hk.vboost[1],
                GOM_vboost3=self.gom.hk.vboost[2],
                GOM_vbatt=self.gom.hk.vbatt,
                GOM_curin1=self.gom.hk.curin[0],
                GOM_curin2=self.gom.hk.curin[1],
                GOM_curin3=self.gom.hk.curin[2],
                GOM_cursun=self.gom.hk.cursun,
                GOM_cursys=self.gom.hk.cursys,
                GOM_reserved1=self.gom.hk.reserved1,
                GOM_curout1=self.gom.hk.curout[0],
                GOM_curout2=self.gom.hk.curout[1],
                GOM_curout3=self.gom.hk.curout[2],
                GOM_curout4=self.gom.hk.curout[3],
                GOM_curout5=self.gom.hk.curout[4],
                GOM_curout6=self.gom.hk.curout[5],
                GOM_outputs=int(
                    str(self.gom.hk.output[:]).replace(",", "").replace(" ", "")[1:-1],
                    2,
                ),
                GOM_latchup1=self.gom.hk.latchup[0],
                GOM_latchup2=self.gom.hk.latchup[1],
                GOM_latchup3=self.gom.hk.latchup[2],
                GOM_latchup4=self.gom.hk.latchup[3],
                GOM_latchup5=self.gom.hk.latchup[4],
                GOM_latchup6=self.gom.hk.latchup[5],
                GOM_wdt_i2c_time_left=self.gom.hk.wdt_i2c_time_left,
                GOM_wdt_gnd_time_left=self.gom.hk.wdt_gnd_time_left,
                GOM_counter_wdt_i2c=self.gom.hk.counter_wdt_i2c,
                GOM_counter_wdt_gnd=self.gom.hk.counter_wdt_gnd,
                GOM_counter_boot=self.gom.hk.counter_boot,
                GOM_bootcause=self.gom.hk.bootcause,
                GOM_battmode=self.gom.hk.battmode,
                GOM_temp1=self.gom.hk.temp[0],
                GOM_temp2=self.gom.hk.temp[1],
                GOM_temp3=self.gom.hk.temp[2],
                GOM_temp4=self.gom.hk.temp[3],
                GOM_pptmode=self.gom.hk.pptmode,
                GOM_reserved2=self.gom.hk.reserved2,
                RTC_measurement_taken=self.rtc.rtc_time,
                RPI_cpu=self.rpi.cpu,
                RPI_ram=self.rpi.ram,
                RPI_dsk=self.rpi.disk,
                RPI_tmp=self.rpi.tmp,
                RPI_boot=self.rpi.boot_time,
                RPI_uptime=self.rpi.up_time,
                GYRO_gyr_x=gx,
                GYRO_gyr_y=gy,
                GYRO_gyr_z=gz,
                GYRO_acc_x=ax,
                GYRO_acc_y=ay,
                GYRO_acc_z=az,
                GYRO_mag_x=bx,
                GYRO_mag_y=by,
                GYRO_mag_z=bz,
                GYRO_temperature=self.gyr.tmp,
                THERMOCOUPLE_temperature=self.thm.tmp,
                PRESSURE_pressure=self.prs.pressure,
            )
            self.session.add(telemetry_data)
            self.session.commit()
        finally:
            self.session.close()

    def query_telem(self):
        # TODO
        # querys telemetry from database
        raise NotImplementedError
