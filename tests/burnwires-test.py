from drivers.gom import Gomspace
from datetime import datetime
from utils.log import get_log
from drivers.gyro import GyroSensor

# from utils.constants import

logger = get_log()
gyro = GyroSensor()
gom_controller = Gomspace()

gom_controller.burnwire2(1, delay=5)
start_time = datetime.now()

difference = 0
gyro_data = []
logger.info("Gyro data (rad/s)")
while difference < 20:
    difference = (datetime.now() - start_time).total_seconds()
    # read gom output data
    hkdata = gom_controller.get_health_data("eps")

    # read gyro data
    for i in range(10):
        gyro_reading = gyro.get_gyro()
        gyro_time = datetime.now().time().isoformat()
        gyro_list = list(gyro_reading)
        gyro_list.append(gyro_time)
        gyro_data.append(gyro_list)
        logger.info(f"{gyro_reading}")

# writes gyro data to gyro_data.txt. Caution, this file will be overwritten with every successive test
with open('gyro_data.txt', 'w') as filehandle:
    filehandle.writelines("%s\n" % line for line in gyro_data)
