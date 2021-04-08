from drivers.gyro import GyroSensor
import numpy as np

gyro = GyroSensor()

# take 1000 gyro measurements and store data

data = [] * 1000

for i in range(len(data)):
    data[i] = gyro.get_gyro()

data = np.array(data)
biases = np.mean(data, axis=0)
standard_devs = np.std(data, axis=0)
