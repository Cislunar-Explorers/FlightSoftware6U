from drivers.gyro import GyroSensor
import numpy as np
from utils.log import get_log
from typing import List, Tuple

logger = get_log()
gyro = GyroSensor()

# take 500 gyro measurements and store data

data: List[Tuple[float, float, float]] = []
# print("getting data")
temperature = gyro.get_temp()
for i in range(500):
    data.append(gyro.get_gyro())

# print("Done getting data")

np_data = np.array(data)

biases = np.mean(np_data, axis=0)
standard_devs = np.std(np_data, axis=0)

logger.info(f"Biases: {biases}")
logger.info(f"Stds: {standard_devs}")
logger.info(f"Temperature: {temperature}")
