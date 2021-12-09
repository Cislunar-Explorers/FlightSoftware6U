from communications.groundstation import Groundstation
from communications.command_handler import CommandHandler
import logging


ch = CommandHandler(None)
groundstation = Groundstation()
commandCounter = 1
transmitInterval = 3

fm_num = 0
while fm_num > -1:
    command_num = int(input("What command ID would you like to send?\n"))
    if command_num > -1:
        kwarg_str = input("Please input any arguments as a valid python dictionary:\n")
        kwarg_dict = eval(kwarg_str)
        packed = ch.pack_command(command_num, **kwarg_dict)
        groundstation.transmit(packed)
        logging.info(f"Sent over radio: {str(packed.hex())}")
