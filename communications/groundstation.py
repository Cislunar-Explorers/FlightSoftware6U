#Packing Imports
from communications.commands import CommandHandler
import utils.struct as us
from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException, SerializationException
from sys import maxsize, float_info

#Transimission Imports
import logging
import time
#import board
import busio
from adafruit_bus_device.spi_device import SPIDevice
from communications.ax5043_manager.ax5043_driver import Ax5043
from communications.ax5043_manager.ax5043_manager import Manager

#Packs data into a bytearray
def registerCommand(ch: CommandHandler, mode_id, command_id, **kwargs):

    totalBytes = 0

    try:
        for arg in kwargs:
            if isinstance(kwargs[arg], bool): #boolean
               ch.register_new_codec(arg,us.pack_bool, us.unpack_bool)
               totalBytes += 1
            elif isinstance(kwargs[arg], int) and abs(kwargs[arg]) <= maxsize: #int
               ch.register_new_codec(arg,us.pack_unsigned_int, us.unpack_unsigned_int)
               totalBytes += 4
            elif isinstance(kwargs[arg], int): #long
               ch.register_new_codec(arg,us.pack_unsigned_long, us.unpack_unsigned_long)
               totalBytes += 8
            elif isinstance(kwargs[arg], float) and abs(kwargs[arg]) <= float_info.max: #float
               ch.register_new_codec(arg,us.pack_float, us.unpack_float)
               totalBytes += 4
            elif isinstance(kwargs[arg], float): #double
               ch.register_new_codec(arg,us.pack_double, us.unpack_double)
               totalBytes += 8
        
        command_args= list(kwargs.keys())
        command_data_tuple = (command_args, totalBytes)
        command_dict = {command_id: command_data_tuple}
        ch.register_mode_commands(mode_id, command_dict)

    except:
        raise SerializationException()

#Transmits given signal over the radio
def transmitCommand(mgr: Manager, signal:bytearray):
   
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

ch = CommandHandler()
registerCommand(ch,1,2,number1=6,number2=4)