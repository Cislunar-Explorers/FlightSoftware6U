import communications.satellite_radio as sr
from communications.serialization import DataHandler
import board
import busio
import time
from adafruit_bus_device.spi_device import SPIDevice
from communications.ax5043_manager.ax5043_driver import Ax5043
from communications.ax5043_manager.ax5043_manager import Manager

# Radio setup
driver = Ax5043(SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)))
mgr = Manager(driver)
dh = DataHandler()

# Just used to register command in the DataHandler so that it can later unpack
sr.registerDownlink(dh, 1, 2, number1=0, number2=1)

print('Entering Receiving Mode')

cycleNumber = 1

# Enter listening mode
while True:

    print('Cycle: ' + str(cycleNumber))

    message = sr.receiveSignal(mgr)

    if message is not None:
        print(dh.unpack_data(message))
        mode_id, command_id, dataDict = dh.unpack_data(message)
        print('Message Received')
        break

    else:
        cycleNumber += 1
        time.sleep(1)

time.sleep(5)

# Multiply numbers received by 2 and then send them back
downlinkData = dh.pack_data(1, 2, number1=dataDict['number1'] * 2, number2=dataDict['number2'] * 2)

print('Transmitting')
sr.transmitData(mgr, downlinkData)
print('Terminated')
