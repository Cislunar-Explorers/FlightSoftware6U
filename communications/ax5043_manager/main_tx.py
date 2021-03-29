import logging
import time
import board
import busio
from adafruit_bus_device.spi_device import SPIDevice
from ax5043_driver import Ax5043
from ax5043_manager import Manager
from communications.commands import CommandHandler

logging.basicConfig(level=logging.DEBUG)
driver = Ax5043(SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)))
mgr = Manager(driver)

mgr.tx_enabled = True

ch = CommandHandler()
command = ch.pack_command(1,8,9,some_number=67,long_string='abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz')
print(command)
mgr.inbox.put(command)

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
    # After 10s, break for clean shutdown
    # (TODO: use interrupt handler to ensure clean shutdown when killed,
    # or monitor transmitting state and exit when complete)
    if cycles >= 10: break
    time.sleep(1)

mgr.tx_enabled = False
mgr.rx_enabled = False
mgr.dispatch()
