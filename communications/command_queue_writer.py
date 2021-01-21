from communications.commands import CommandHandler
from utils.log import get_log

logger = get_log()

ch = CommandHandler()

fm_num = int(0)
filename = "command_queue.txt"
while fm_num > -1:
    fm_num = int(input("What flight mode would you like to command in?\n"))
    if fm_num > -1:
        command_num = int(input("What command ID would you like to send?\n"))
        if command_num > -1:
            kwarg_str = input("Please input any arguments as a valid python dictionary")
            kwarg_dict = eval(kwarg_str)
            packed = ch.pack_command(fm_num, command_num, **kwarg_dict)
            file = open(filename, "a")
            file.write(str(packed.hex()) + "\n")
            file.close()
            logger.info(f"Wrote hex bytes {str(packed.hex())}")