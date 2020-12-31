#Packing Imports
from commands import CommandHandler
import utils.struct as us
from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException, SerializationException
from sys import maxsize, float_info

#Transimission Imports
import unittest
import logging
from ax5043_manager.ax5043_manager import Manager
from ax5043_manager.ax5043_driver import Reg, Bits

#Packs data into a bytearray
def packData (mode_id, command_id, **kwargs):

    ch = CommandHandler()
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

        command_buffer = ch.pack_command(mode_id, command_id, **kwargs)
        return command_buffer
    except:
        raise SerializationException()

#Transmits given input over the radio
def transmitData(radio, mode_id, command_id, **kwargs):
    
    command_buffer = packData(mode_id, command_id, **kwargs)
    tester = unittest.TestCase()
    
    radio.read_defaults[Reg.XTALSTATUS] = Bits.XTAL_RUN
    radio.read_defaults[Reg.POWSTAT] |= Bits.SVMODEM
    mgr = Manager(radio)
    
    # Transition to autoranging
    mgr.dispatch()
    
    # Transition to idle
    mgr.dispatch()
   
    # Stay in idle when tx_enabled=False
    mgr.inbox.put(command_buffer)
    mgr.dispatch()
    tester.assertTrue(isinstance(mgr.state, Manager.Idle))
   
    # Transition to transmitting
    mgr.tx_enabled = True
    mgr.dispatch()
    tester.assertTrue(isinstance(mgr.state, Manager.Transmitting))
    mgr.dispatch()
    tester.assertFalse(mgr.is_faulted())