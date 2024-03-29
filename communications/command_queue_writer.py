"""command_queue_writer.py: a workaround to not having a radio. Instead of sending
commands to the EDU/HITL through the radio board, use this to write commands to a txt file, which is then read by
main.py's read_command_queue_from_file method """

from communications.command_handler import CommandHandler
import logging


ch = CommandHandler(None)

fm_num = int(0)
filename = "command_queue.txt"
command_counter = 1

while fm_num > -1:
    command_num = int(input("What command ID would you like to send?\n"))
    if command_num > -1:
        kwarg_str = input("Please input any arguments as a valid python dictionary:\n")
        kwarg_dict = eval(kwarg_str)
        packed = ch.pack_command(command_num, **kwarg_dict)
        file = open(filename, "a")
        file.write(str(packed.hex()) + "\n")
        file.close()
        logging.info(f"Wrote hex bytes {str(packed.hex())}")
