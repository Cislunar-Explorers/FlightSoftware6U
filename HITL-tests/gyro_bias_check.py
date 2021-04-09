from drivers.gyro import GyroSensor
import numpy as np

gyro = GyroSensor()

# take 500 gyro measurements and store data

data = [] * 500

temperature = gyro.get_temp()
for i in range(len(data)):
    data[i] = gyro.get_gyro()

data = np.array(data)
biases = np.mean(data, axis=0)
standard_devs = np.std(data, axis=0)

print(f"Biases: {biases}")
print(f"Stds: {standard_devs}")
print(f"Temperature: {temperature}")
