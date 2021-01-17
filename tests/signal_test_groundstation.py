import communications.groundstation as gs
from communications.commands import CommandHandler
import board
import busio
import time
from adafruit_bus_device.spi_device import SPIDevice
from communications.ax5043_manager.ax5043_driver import Ax5043
from communications.ax5043_manager.ax5043_manager import Manager

#Radio setup
driver = Ax5043(SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)))
mgr = Manager(driver)
ch = CommandHandler()

#Pack command into a bytearray
gs.registerCommand(ch,1,2,number1=7, number2=4)
command = ch.pack_command(1,2,number1=7, number2=4)

#Send the command to the satellite
gs.transmitCommand(driver, command)

print('Transmitted')
print('Entering Receiving Mode')

cycleNumber = 1

#Enter listening mode
while True:

    print('Cycle: ' + str(cycleNumber))

    message = gs.receiveSignal(mgr)

    if message is not None:
        print(ch.unpack_command(message))
        print('Terminated')
        break

    else:
        cycleNumber += 1
        time.sleep(1)