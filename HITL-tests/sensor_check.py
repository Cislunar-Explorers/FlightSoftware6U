from drivers.gom import Gomspace
from drivers.rtc import RTC
from drivers.imu import GyroSensor
from drivers.adc import ADC
from communications.satellite_radio import Radio
from core.camera import PiCam
from core.camera import CameraMux
from drivers.nemo.nemo import Nemo
import logging
from time import time


def check_gom():
    try:
        gom = Gomspace()
        hk = gom.get_health_data(level="eps")
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error(
            "GOM init failed. Is it charged above 7.0V? Check SDA/SCL (with an oscilloscope if need be)"
        )
    else:
        logging.info(f"GOM initialization successful. Battery voltage: {hk.vbatt}")


def check_sensor_board():
    try:
        gyro = GyroSensor()
        gyro_data = gyro.get_gyro()
    except Exception as e:
        logging.error(e, exc_info=True)
        gyro = None
        logging.error(
            "GYRO init failed. Check power, gnd, and SDA/SCL connections to sensor board"
        )
    else:
        logging.info(f"Gyro initialization successful. Gyro rates: {gyro_data}")

    if gyro:
        try:
            adc = ADC(gyro)
            temp = adc.read_temperature()
            pressure = adc.read_pressure()
        except Exception as e:
            logging.error(e, exc_info=True)
            logging.error(
                "ADC init failed. Check power, gnd, and SDA/SCL connections to sensor board"
            )
        else:
            logging.info(
                f"ADC initialization successful. Pressure:{pressure}, Thermocouple:{temp}"
            )
    else:
        logging.warning("Not initializing ADC because Gyro failed")

    try:
        rtc = RTC()
        rtc_time = rtc.get_time()
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error(
            "RTC init failed. Check power, gnd, and SDA/SCL connections to sensor board"
        )
    else:
        logging.info(f"RTC initialization successful. RTC time: {rtc_time}")


def check_radio():
    try:
        Radio()
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error(
            "RADIO init Failed. Make sure radio/comms board connectors are correct"
        )
    else:
        logging.info("RADIO initialization successful")


def check_nemo():
    try:
        nemo = Nemo(port_id=3, reset_gpio_ch=16)
        nemo.self_test()
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error(
            "NEMO init failed. Check 'i2cdetect -y 3' for device address 0x13"
        )
    else:
        logging.info("NEMO initialized successfully")


def check_mux():
    try:
        mux = CameraMux()
        mux.selectCamera(1)
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error(
            "MUX init failed. Make sure it's pressed down fully onto GPIO connections, and that SDA/SCL isn't grounded"
        )
    else:
        logging.info("MUX initialized successfully")


def check_cams():
    try:
        mux = CameraMux()
        mux.selectCamera(1)
    except Exception as e:
        logging.error(e, exc_info=True)
        logging.error("MUX init failed. Not attempting to init cameras")
    else:
        logging.info("MUX initialized successfully")
        camera = PiCam()
        for i in [1, 2, 3]:
            try:
                mux.selectCamera(i)
                f, t = camera.rawObservation(f"initialization-{i}-{int(time())}")
            except Exception as e:
                logging.error(e, exc_info=True)
                logging.error(f"CAM{i} init failed")
            else:
                logging.info(f"CAM{i} initialized successfully: {f}: {t}")


def sensor_check(input_str):
    sensor_checks = {
        1: check_gom,
        2: check_radio,
        3: check_sensor_board,
        4: check_mux,
        5: check_cams,
        6: check_nemo,
    }

    for char in input_str:
        char_int = int(char)
        sensor_checks[char_int]()


if __name__ == "__main__":
    input_str = ""
    logging.info("Sensor checks:")
    while input_str != "-1":
        logging.info(
            "\nEnter the number(s) corresponding to the device(s) you want to check.\n"
            " Gomspace: 1\n Radio: 2\n Sensor Board: 3\n Camera Mux: 4\n Cameras: 5\n Nemo: 6\n"
            "To test the Gom, type '1' and enter. To test the gom, radio, and mux, type '124' and enter"
        )
        input_str = input()
        try:
            sensor_check(input_str)
        except (KeyError, ValueError):
            logging.error("Invalid input. Make sure you only use digits 1 thru 6")
