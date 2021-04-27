#Packing Imports
import utils.struct as us
from utils.constants import FMEnum, ZERO_WORD, ONE_WORD
from utils.exceptions import UnknownFlightModeException, SerializationException
from sys import maxsize, float_info

#Transimission Imports
import logging
import time
import board
import busio
from adafruit_bus_device.spi_device import SPIDevice
from communications.ax5043_manager.ax5043_driver import Ax5043
from communications.ax5043_manager.ax5043_manager import Manager
from bitstring import BitArray

from datetime import datetime

class Groundstation():

    def __init__(self):

        self.driver = Ax5043(SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)))
        self.mgr = Manager(self.driver)
        self.last_transmit_time = datetime.today()
        self.last_telemetry_time = datetime.today()

    # Monitor radio health, request reset if faulted
    def monitorHealth(self):
        if self.mgr.is_faulted():
            logging.error('Radio manager faulted')
            self.mgr.reset_requested = True
            return None

    #Gets the signal from the radio board and returns it in a bytearray
    def receiveSignal(self):

        self.mgr.rx_enabled = True
        self.mgr.dispatch()
        self.monitorHealth()
        
        if self.mgr.outbox.empty():
            return None
        else:
            downlink = self.mgr.outbox.get()
            print('Inflated Bytes: ' + str(downlink))
            return self.bit_deflation(downlink, ZERO_WORD, ONE_WORD)

    #Downlink given bytearray to ground station
    def transmit(self, signal:bytearray):

        self.mgr.tx_enabled = True
        self.mgr.inbox.put(signal)

        cycles = 0

        while True:
           
            self.mgr.dispatch()
            self.monitorHealth()

            cycles += 1
            # After 5s, break for clean shutdown
            # (TODO: use interrupt handler to ensure clean shutdown when killed,
            # or monitor transmitting state and exit when complete)
            if cycles >= 10: break
            time.sleep(1)

        self.mgr.tx_enabled = False
        self.mgr.rx_enabled = False
        self.mgr.dispatch()
        
        self.last_transmit_time = datetime.today()
    
    def bit_deflation(self, downlink: bytearray, zero_word: bytearray, one_word: bytearray):
    
        deflatedBitArray = BitArray('',bin='')
        
        #Recover 
        for i in range(len(downlink)//2):
            byte = downlink[i*2:(i*2)+2]
            
            if byte == zero_word:
                deflatedBitArray += '0b0'
            else:
                deflatedBitArray += '0b1'
        
        return deflatedBitArray.bytes