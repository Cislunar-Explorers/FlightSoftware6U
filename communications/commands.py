from struct import error as StructError

from flight_modes.flight_mode_factory import FLIGHT_MODE_DICT
from utils.constants import (
    MIN_COMMAND_SIZE,
    MODE_OFFSET,
    ID_OFFSET,
    DATA_LEN_OFFSET,
    DATA_OFFSET,
)
from utils.exceptions import (
    CommandException,
    CommandPackingException,
    CommandUnpackingException,
)
from utils.struct import (
    pack_unsigned_short,
    unpack_unsigned_short,
)


def calc_command_size(data: bytes):
    return MIN_COMMAND_SIZE + len(data)


def pack_command_bytes(mode: int, command_id: int, data: bytes):
    buf = bytearray(calc_command_size(data))
    buf[MODE_OFFSET] = mode
    buf[ID_OFFSET] = command_id
    pack_unsigned_short(buf, DATA_LEN_OFFSET, len(data))
    buf[DATA_OFFSET:] = data
    return bytes(buf)


def unpack_command_bytes(data: bytes):
    mode = data[MODE_OFFSET]
    command_id = data[ID_OFFSET]
    data_len = unpack_unsigned_short(data, DATA_LEN_OFFSET)[1]
    if data_len + DATA_OFFSET != len(data):
        raise CommandUnpackingException(
            f"Incorrect data buffer size: {len(data)}, expected "
            f"{data_len + DATA_OFFSET} for command with Mode: {mode}"
            f"CommandID: {command_id}"
        )
    return mode, command_id, data[DATA_OFFSET:]


class CommandHandler:
    def __init__(self):
        self.command_dict = dict()
        self.packers = dict()
        self.unpackers = dict()
        # Packers and unpackers will always have identical set of keys
        # Enforced in practice
        self.register_codecs()

    # Register packers and unpackers for all possible named arguments to function calls
    # Register the ModeID, ApplicationID, combos that result in certain function parameters
    def register_codecs(self):
        for mode_id, mode_class in FLIGHT_MODE_DICT.items():
            self.register_mode_commands(mode_id, mode_class.command_codecs)
            for argname, pack_tuple in mode_class.command_arg_unpackers.items():
                packer, unpacker = pack_tuple
                self.register_new_codec(argname, packer, unpacker)

    def get_command_size(self, mode: int, application_id: int):
        return self.command_dict[mode][application_id][1] + DATA_OFFSET

    def register_new_codec(self, arg: str, packer, unpacker):
        if arg in self.packers:
            raise CommandException(
                "Trying to register a codec for an existing argument"
            )

        self.packers[arg] = packer
        self.unpackers[arg] = unpacker

    def pack_command(self, mode: int, command_id: int, **kwargs) -> bytes:
        func_args, buffer_size = self.command_dict[mode][command_id]
        data_buffer = bytearray(buffer_size)
        offset = 0
        try:
            for arg in func_args:
                off = self.packers[arg](data_buffer, offset, kwargs[arg])
                offset += off
            return pack_command_bytes(mode, command_id, data_buffer)
        except StructError as exc:
            raise CommandPackingException(str(exc))
        except KeyError as exc:
            if arg is not None:
                raise CommandPackingException(
                    f"KeyError occured for arg: {arg}, using Mode: {mode}; CommandID: {command_id} "
                    f"KeyError was: {str(exc)}"
                )
            else:
                raise CommandPackingException(
                    f"KeyError occurred, no such command: {command_id} for mode: {mode} "
                    f"KeyError was: {str(exc)}"
                )

    def unpack_command(self, data: bytes):
        mode, command_id, arg_data = unpack_command_bytes(data)
        func_args, buffer_size = self.command_dict[mode][command_id]
        if (buffer_size + DATA_OFFSET) != len(data):
            raise CommandUnpackingException(
                f"Received command with data len: {len(data)}, but expected length: {buffer_size}; "
                f"for command with Mode: {mode}, CommandID: {command_id}"
            )
        offset = 0
        kwargs = dict()
        for arg in func_args:
            off, value = self.unpackers[arg](arg_data, offset)
            kwargs[arg] = value
            offset += off
        return mode, command_id, kwargs

    def register_mode_commands(self, mode: int, application_ids_to_arg_tuples: dict):
        if mode in self.command_dict:
            raise CommandException(
                f"Trying to register mode {mode} that already exists"
            )
        # Should be a dict of application ids -> tuple (ordered list of arguments, length of arguments)
        # argument names must be registered separately
        self.command_dict[mode] = application_ids_to_arg_tuples
