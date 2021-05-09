from time import time, sleep
from os import popen
import psutil
from uptime import uptime
from telemetry.sensor import SynchronousSensor
import numpy as np
from drivers.power.power_structs import eps_hk_t, hkparam_t
from utils.exceptions import PiSensorError, PressureError, GomSensorError, GyroError, ThermocoupleError
# from utils.db import GyroModel
from utils.db import TelemetryModel, create_sensor_tables_from_path
from utils.constants import MAX_GYRO_RATE, GomOutputs, DB_FILE
import utils.parameters as params
from typing import Tuple


def moving_average(x, w):
    """x: 1-D data, w: number of samples to average over."""
    return np.convolve(x, np.ones(w), 'valid') / w


class GomSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.hk = eps_hk_t()  # HouseKeeping data: eps_hk_t struct
        self.hkparam = hkparam_t()  # hkparam_t struct
        self.percent = float()
        self.is_electrolyzing = bool()

    def poll(self):
        super().poll()
        if self.parent.gom is not None:
            self.hk = self.parent.gom.get_health_data(level="eps")
            self.hkparam = self.parent.gom.get_health_data()
            battery_voltage = self.hk.vbatt  # mV
            self.percent = (battery_voltage - params.GOM_VOLTAGE_MIN) / \
                           (params.GOM_VOLTAGE_MAX - params.GOM_VOLTAGE_MIN)
            self.is_electrolyzing = bool(self.hk.output[GomOutputs.electrolyzer.value])


class GyroSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.rot: Tuple[float, float, float] = (float(), float(), float())  # rad/s
        self.mag: Tuple[float, float, float] = (float(), float(), float())  # microTesla
        self.acc: Tuple[float, float, float] = (float(), float(), float())  # m/s^2
        self.tmp: float = float()  # deg C

    def poll(self):
        super().poll()
        if self.parent.gyro is not None:
            self.rot = self.parent.gyro.get_gyro_corrected()
            self.mag = self.parent.gyro.get_mag()
            self.acc = self.parent.gyro.get_acceleration()
            self.tmp = self.parent.gyro.get_temp()

    def poll_smoothed(self, freq=10, duration=1, samples=10):
        # poll and smooth gyro data

        n_data = freq * duration
        data = [None] * n_data
        for i in range(n_data):
            data[i] = self.parent.gyro.get_gyro_corrected()
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


class PressureSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.pressure = float()  # pressure, psi

    def poll(self):
        super().poll()
        if self.parent.adc is not None:
            self.pressure = self.parent.adc.read_pressure()


class ThermocoupleSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.tmp = float()  # Fuel tank temperature, deg C

    def poll(self):
        super().poll()
        if self.parent.adc is not None:
            self.tmp = self.parent.adc.read_temperature()


class PiSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.cpu: int = int()  # can be packed as short
        self.ram: int = int()  # can be packed as short
        self.disk: int = int()  # can be packed as short
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
        self.tmp = float(popen("vcgencmd measure_temp").readline().strip()[5:-2])
        self.all = (self.cpu,
                    self.ram,
                    self.disk,
                    self.boot_time,
                    self.up_time,
                    self.tmp,
                    self.poll_time)


class RtcSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.rtc_time = int()

    def poll(self):
        super().poll()
        if self.parent.rtc is not None:
            self.rtc_time = self.parent.rtc.get_time()


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
        # So almost all calls to sensors will be made through self.parent.<insert sensor stuff here>
        super().__init__(parent)

        self.gom = GomSensor(parent)
        self.gyr = GyroSensor(parent)
        self.prs = PressureSensor(parent)
        self.thm = ThermocoupleSensor(parent)
        self.rpi = PiSensor(parent)
        self.rtc = RtcSensor(parent)
        self.opn = OpNavSensor(parent)

        self.sensors = [self.gom, self.gyr, self.prs, self.thm, self.rpi, self.rtc]

        create_session = create_sensor_tables_from_path(DB_FILE)  # instantiate DB session
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

        if any(i > MAX_GYRO_RATE for i in tuple(map(abs, self.gyr.rot))):
            self.parent.logger.error("Gyro not functioning properly")
            raise GyroError(f"Unreasonable gyro values: {self.gyr.rot}")

        if self.prs.pressure < 0 or self.prs.pressure > 2000:
            self.parent.logger.error("Pressure sensor not functioning properly")
            raise PressureError(f"Unreasonable pressure: {self.prs.pressure}")

        if self.thm.tmp < -200 or self.thm.tmp > 200:
            self.parent.logger.error("Thermocouple not functioning properly")
            raise ThermocoupleError(f"Unreasonable fuel tank temperature: {self.thm.tmp}")

        if self.gom.percent < 0 or self.gom.percent > 1.5:
            self.parent.logger.error("Gom HK not functioning properly")
            raise GomSensorError(f"Unreasonable battery percentage: {self.gom.percent}")

        if any(i < 0 for i in self.rpi.all):
            self.parent.logger.error("RPi sensors not functioning properly")
            raise PiSensorError

    def standard_packet(self):
        if (time() - self.poll_time) > 60 * 60:  # if the latest data is over an hour old, poll new data
            self.poll()

        return (self.rtc.rtc_time,
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
                self.prs.pressure)

    def standard_packet_dict(self):
        return {'rtc_time': self.rtc.rtc_time,
                'position_x': 1,  # FIXME Opnav results interface
                'position_y': 2,
                'position_z': 3,
                'attitude_1': 4,
                'attitude_2': 5,
                'attitude_3': 6,
                'attitude_4': 7,
                'hk_temp_1': self.gom.hk.temp[0],  # ushort
                'hk_temp_2': self.gom.hk.temp[1],  # ushort
                'hk_temp_3': self.gom.hk.temp[2],  # ushort
                'hk_temp_4': self.gom.hk.temp[3],  # ushort
                'gyro_temp': self.gyr.tmp,
                'thermo_temp': self.thm.tmp,
                'curin_1': self.gom.hk.curin[0],  # ushort
                'curin_2': self.gom.hk.curin[1],  # ushort
                'curin_3': self.gom.hk.curin[2],  # ushort
                'vboost_1': self.gom.hk.vboost[0],  # ushort
                'vboost_2': self.gom.hk.vboost[1],  # ushort
                'vboost_3': self.gom.hk.vboost[2],  # ushort
                'cursys': self.gom.hk.cursys,  # ushort
                'vbatt': self.gom.hk.vbatt,  # ushort
                'prs_pressure': self.prs.pressure}

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

                # TODO figure out if this is just one or array
                # GOM_outputs = Column(Integer),
                GOM_outputs=self.gom.hk.outputs[0],


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
                GOM_battmode=self.gom.hk.battmodeColumn,
                GOM_temp1=self.gom.hk.temp[0],
                GOM_temp2=self.gom.hk.temp[1],
                GOM_temp3=self.gom.hk.temp[2],
                GOM_temp4=self.gom.hk.temp[3],
                GOM_pptmode=self.gom.hk.pptmode,
                GOM_reserved2=self.gom.hk.reserved2,
                RTC_measurement_taken=self.rtc.rtc_time,
                RPI_cpu=self.cpu,  # TODO: fix this
                RPI_ram=self.ram,
                RPI_dsk=self.disk,
                RPI_tmp=self.tmp,
                RPI_boot=self.boot_time,
                RPI_uptime=self.up_time,
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
                THERMOCOUPLE_pressure=self.thm.tmp,
                PRESSURE_pressure=self.pressure
            )
            self.session.add(telemetry_data)
            self.session.commit()
        finally:
            self.session.close()

    def query_telem(self, sensor):
        if sensor.equals("all"):
            self.query_all()
        elif sensor.equals("gom"):
            self.query_gom()
        elif sensor.equals("rtc"):
            self.query_rtc()
        elif sensor.equals("rpi"):
            self.query_rpi()
        elif sensor.equals("gyro"):
            self.query_gyro()
        elif sensor.equals("thm"):
            self.query_thm()
        elif sensor.equals("pressure"):
            self.query_pressure()
        else:
            raise ValueError

    def query_all(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries:
            print(entry)
            # replace print with downlink

    def query_gom(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[1:37]:
            print(entry)
            # replace print with downlink

    def query_rtc(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[37:38]:
            print(entry)
            # replace print with downlink

    def query_rpi(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[38:44]:
            print(entry)
            # replace print with downlink

    def query_gyro(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[44:55]:
            print(entry)
            # replace print with downlink

    def query_thm(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[55:56]:
            print(entry)
            # replace print with downlink

    def query_pressure(self):
        entries = self.session.query(TelemetryModel).all()
        for entry in entries[56:57]:
            print(entry)
            # replace print with downlink

