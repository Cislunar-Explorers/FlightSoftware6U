from communications.commands import CommandHandler

# Commands we want to execute for demo: (bootup) split, change FM and "run" opnav, start/stop electrolysis
ch = CommandHandler()
commands = list()

# split
commands.append(ch.pack_command(0, 1))

# opnav
commands.append(ch.pack_command(2, 1))

# start electrolysis
commands.append(ch.pack_command(2, 3, **{"state": True}))

# stop electrolysis
commands.append(ch.pack_command(2, 3, **{"state": False}))

user_in = int(0)
filename = "communications/command_queue.txt"
while user_in > -1:
    user_in = int(input("What command would you like to send?\n"
                        "1: split\n"
                        "2: opnav\n"
                        "3: electrolysis start\n"
                        "4: electrolysis stop\n"))
    if user_in > -1:
        file = open(filename, "w")
        file.write(str(commands[user_in - 1], 'utf-8'))
        file.close()
