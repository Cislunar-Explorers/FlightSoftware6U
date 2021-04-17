from drivers.gyro import GyroSensor
import numpy as np
from utils.log import get_log

logger = get_log()
gyro = GyroSensor()

# take 500 gyro measurements and store data

data = [None] * 500
# print("getting data")
temperature = gyro.get_temp()
for i in range(len(data)):
    data[i] = gyro.get_gyro()

# print("Done getting data")

data = np.array(data)

data = np.array(data)

biases = np.mean(data, axis=0)
standard_devs = np.std(data, axis=0)

logger.info(f"Biases: {biases}")
logger.info(f"Stds: {standard_devs}")
logger.info(f"Temperature: {temperature}")
