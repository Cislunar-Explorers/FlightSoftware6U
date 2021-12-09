# General Command Specification

Command bytes consist of the following set of data which define which command gets executed and whether it's safe to execute.
The MAC is generated from the rest of the contents of the message.
The Counter is simply a counter of all commands/telemetry sent.
More info on the MAC/Counter implementation can be found at https://cornell.app.box.com/file/759191053032
The id is the ID number of the command to be executed. These IDs are defined in the CommandEnum in utils.constants
len_data is the number of bytes in data, which is stored as one byte.
data is the data as to be packed/unpacked by the command's codecs.

Note: this gives the data passed to commands a maximum length of 256. The buffer size on the AX5043 radio chip is also 256 bytes [citation needed].

```boo
+-----------+------------+--------------------------------+
| MAC       : bytes      |                        4 bytes |
+-----------+------------+--------------------------------+
| Counter   : bytes      |                        3 bytes |
+-----------+------------+--------------------------------+
| id        : byte       |                         1 byte |
+-----------+------------+--------------------------------+
| len_data  : byte       |                         1 byte |
+-----------+------------+--------------------------------+
| data      : bytes      |                len(data) bytes |
+-----------+------------+--------------------------------+
                         |            9 + len(data) bytes |
                         +--------------------------------+
```


# Design Goal

Prior work had each flight mode individually handle the (de)serialization of each command, but this led to much frustration because it required altering six different files just to implement one command.
Now the desire is to make the command system as easy to implement new commands while making everything much more self-documenting.

# Forward Work
- [ ] Document all required commands
- [ ] Better command error handling/error codes
- [ ] Document and Implement restrictions on commands (maybe there are some commands we don't want to execute in a give flight mode)
## Breakdown of All Commands
All commmands are defined in `command_definitions.py`, and are further documented on Box.
