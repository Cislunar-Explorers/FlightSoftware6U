#Packing Imports
from communications.serialization import DataHandler
import utils.struct as us
from utils.constants import FMEnum
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

#Gets the signal from the radio board and returns it in a usable format
def receiveSignal(mgr: Manager):

    mgr.rx_enabled = True

    mgr.dispatch()

    # Health monitoring
    if mgr.is_faulted():
        logging.error('Radio manager faulted')
        mgr.reset_requested = True
        return None
    
    #Return contents (bytearray) in outbox, None if empty
    if mgr.outbox.empty():
        return None
    else:
        return mgr.outbox.get()

def transmitData(mgr:Manager, signal:bytearray):
   
   #Radio board setup
   logging.basicConfig(level=logging.DEBUG)

   #Allow radio to transmit
   mgr.tx_enabled = True

   #Put signal in the radio inbox
   mgr.inbox.put(signal)

   logging.debug('Transmitting')

   #Send signal
   mgr.dispatch()

   # Health monitoring
   if mgr.is_faulted():
      logging.error('Radio manager faulted')
      mgr.reset_requested = True

   #Back to idle
   mgr.tx_enabled = False
   mgr.rx_enabled = False
   mgr.dispatch()

#Registers the downlink data in the DataHandler
def registerDownlink(dh: DataHandler, mode_id, data_id, **kwargs):

    totalBytes = 0

    try:
        for arg in kwargs:
            if isinstance(kwargs[arg], bool): #boolean
               dh.register_new_codec(arg,us.pack_bool, us.unpack_bool)
               totalBytes += 1
            elif isinstance(kwargs[arg], int) and abs(kwargs[arg]) <= maxsize: #int
               dh.register_new_codec(arg,us.pack_unsigned_int, us.unpack_unsigned_int)
               totalBytes += 4
            elif isinstance(kwargs[arg], int): #long
               dh.register_new_codec(arg,us.pack_unsigned_long, us.unpack_unsigned_long)
               totalBytes += 8
            elif isinstance(kwargs[arg], float) and abs(kwargs[arg]) <= float_info.max: #float
               dh.register_new_codec(arg,us.pack_float, us.unpack_float)
               totalBytes += 4
            elif isinstance(kwargs[arg], float): #double
               dh.register_new_codec(arg,us.pack_double, us.unpack_double)
               totalBytes += 8
        
        data_args= list(kwargs.keys())
        data_tuple = (data_args, totalBytes)
        data_dict = {data_id: data_tuple}
        dh.register_mode_datatypes(mode_id, data_dict)

    except:
        raise SerializationException()