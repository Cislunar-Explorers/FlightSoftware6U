from communications.commands import CommandHandler
from utils.log import get_log

logger = get_log()

# Commands we want to execute for demo: (bootup) split, change FM and "run" opnav, start/stop electrolysis
ch = CommandHandler()
commands = [None] * 5

# split
commands[0] = ch.pack_command(0, 1)

# opnav
commands[1] = ch.pack_command(2, 1)

# start electrolysis
commands[2] = ch.pack_command(2, 3, **{"state": True})

# stop electrolysis
commands[3] = ch.pack_command(2, 3, **{"state": False})

user_in = int(0)
filename = "command_queue.txt"
while user_in > -1:
    user_in = int(input("What command would you like to send?\n"
                        "1: split\n"
                        "2: opnav\n"
                        "3: electrolysis start\n"
                        "4: electrolysis stop\n"
                        "5: set opnav interval\n"))
    if user_in > -1:
        if user_in == 5:
            interval = int(input("New OpNav interval value (in minutes):"))
            commands[4] = ch.pack_command(2, 12, **{"interval": interval})

        file = open(filename, "a")
        file.write(str(commands[user_in - 1].hex()) + "\n")
        file.close()
        logger.info(f"Wrote hex bytes {str(commands[user_in - 1].hex())}")
