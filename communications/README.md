# 1.1 General Command Specification


Commands will be broken up by the mode they are intended for, the property or function of that mode, and the data or arguments to be passed to it. The current specification is as follows with the mode and property stored as single bytes and the data as an unsigned short specifying the length followed by the data itself.

Note: this gives the data passed to commands a maximum length of 65535.

```boo
+-----------+------------+--------------------------------+
| mode      : byte       |                        1 bytes |
+-----------+------------+--------------------------------+
| property  : byte       |                        1 bytes |
+-----------+------------+--------------------------------+
| data      : bytes      |                  2 + len(data) |
+-----------+------------+--------------------------------+
                         |            4 + len(data) bytes |
                         +--------------------------------+
```


# 2.1 Design Goal

Desire is to make this as similar as possible to RPC protocol. This means that every mode will be capable of individually handling the commands.

This has the complication of requiring that each flight mode be responsible for serialization/deserialization of the commands that it handles.

Are there any workarounds to this that still give us the benefit of handling the commands separately?


# 3. Forward Work

# 3.1 Commands and Flight Mode Transition

Flight Mode enums are specified in the constants.py file, which dictate which mode should be invoked to handle the command. It remains to be decided if receiving a command specifying a particular flight mode should immediately cause a jump into that flight mode, so that it can be handled.

This seems to be the easiest way to implement it, so in the name of simplicity it's how we will begin. We must consider in the future, when should we not allow a command to cause this immediate jump? For now, the base flight mode will implement the functionality of handling a command and triggering the flight mode change and it will simply allow the commands to always interrupt the current flight mode and set a new one on the receipt of a command. In the future, individual flight modes with priority, should override this handle in order to ensure that it can queue the command, finish up its operation, clean up, and then cleanly pass control over to the execution of the new command.


# 3.2 Breakdown of Possiblee Commands

We need to break down the list of possible commands, in order to begin handling them.