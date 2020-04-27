from ..constants import *
from communications.struct import *


def calc_command_size(data: bytes):
    return MIN_COMMAND_SIZE + len(data)


def pack_command(mode: int, command_id: int, data: bytes):
    buf = bytearray(calc_command_size(data))
    buf[MODE_OFFSET] = mode
    buf[ID_OFFSET] = command_id
    pack_short(buf, DATA_LEN_OFFSET, len(data))
    buf[DATA_OFFSET:] = data

