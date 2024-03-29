from drivers.imu import Gyro
import numpy as np
import logging
from typing import List


gyro = Gyro()
gyro.connect()

# take 500 gyro measurements and store data

data: List[List[float]] = []
# print("getting data")
temperature = gyro._collect_temp()
for i in range(500):
    data.append(gyro._collect_gyro())

# print("Done getting data")

np_data = np.array(data)

biases = np.mean(np_data, axis=0)
standard_devs = np.std(np_data, axis=0)

logging.info(f"Biases: {biases}")
logging.info(f"Stds: {standard_devs}")
logging.info(f"Temperature: {temperature}")
