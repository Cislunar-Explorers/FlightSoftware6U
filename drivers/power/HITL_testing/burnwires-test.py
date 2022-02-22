from drivers.gom import Gomspace
import logging
from drivers.gyro import GyroSensor
from time import sleep, time
import threading


# from utils.constants import BURNWIRE_DURATION


def gyro_thread(gyro_freq: int):
    gyro_data = []
    logging.info("Reading Gyro data (rad/s)")
    for _ in range(2000):
        gyro_reading = gyro.get_gyro()
        gyro_time = time()
        gyro_list = list(gyro_reading)
        gyro_list.append(gyro_time)
        gyro_data.append(gyro_list)
        sleep(1.0 / gyro_freq)

    # writes gyro data to gyro_data.txt. Caution, this file will be overwritten with every successive test
    logging.info("Writing gyro data to file")
    with open("gyro_data.txt", "w") as filehandle:
        filehandle.writelines("%s\n" % line for line in gyro_data)


if __name__ == "__main__":

    gyro = GyroSensor()
    gom_controller = Gomspace()

    # start new thread to log gyro data
    gyro_threader = threading.Thread(target=gyro_thread)
    gyro_threader.start()
    gom_controller.burnwire.pulse(2)  # TODO, use gom's async instead of threading
