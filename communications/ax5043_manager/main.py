import logging
import time
import board
import busio
from adafruit_bus_device.spi_device import SPIDevice
from ax5043_driver import Ax5043
from ax5043_manager import Manager

logging.basicConfig(level=logging.DEBUG)
driver = Ax5043(SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)))
mgr = Manager(driver)

mgr.rx_enabled = True
mgr.tx_enabled = True

mgr.inbox.put(bytearray([0xCA, 0xFA, 0xBA, 0xBE]))

cycles = 0
while True:
    logging.debug('Start of control cycle')

    # Dispatch components
    mgr.dispatch()

    # Health monitoring
    if mgr.is_faulted():
        logging.error('Radio manager faulted')
        mgr.reset_requested = True

    cycles += 1
    if cycles >= 10: break
    time.sleep(1)

mgr.tx_enabled = False
mgr.rx_enabled = False
mgr.dispatch()
