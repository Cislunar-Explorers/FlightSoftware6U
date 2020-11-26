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
    SerializationException,
    DeserializationException,
    DownlinkDataPackingException,
    DownlinkDataUnpackingException,
)
from utils.struct import (
    pack_unsigned_short,
    unpack_unsigned_short,
)

def calc_data_size(data: bytes):
    return MIN_COMMAND_SIZE + len(data)


def pack_bytes(mode: int, datatype_id: int, data: bytes):
    buf = bytearray(calc_data_size(data))
    buf[MODE_OFFSET] = mode
    buf[ID_OFFSET] = datatype_id
    pack_unsigned_short(buf, DATA_LEN_OFFSET, len(data))
    buf[DATA_OFFSET:] = bytes(data)
    return bytes(buf)


def unpack_bytes(data: bytes):
    mode = data[MODE_OFFSET]
    datatype_id = data[ID_OFFSET]
    data_len = unpack_unsigned_short(data, DATA_LEN_OFFSET)[1]
    if data_len + DATA_OFFSET != len(data):
        raise DeserializationException(
            f"Incorrect data buffer size: {len(data)}, expected "
            f"{data_len + DATA_OFFSET} for data with Mode: {mode}"
            f"DataTypeID: {datatype_id}"
        )
    return mode, datatype_id, data[DATA_OFFSET:]


class DataHandler:
    def __init__(self):
        self.data_dict = dict()
        self.packers = dict()
        self.unpackers = dict()
        # Packers and unpackers will always have identical set of keys
        # Enforced in practice
        self.register_codecs()

    # Register packers and unpackers for all possible named arguments to function calls
    # Register the ModeID, ApplicationID, combos that result in certain function parameters
    def register_codecs(self):
        for mode_id, mode_class in FLIGHT_MODE_DICT.items():
            self.register_mode_datatypes(mode_id, mode_class.sensordata_codecs)
            for argname, pack_tuple in mode_class.sensordata_arg_unpackers.items():
                packer, unpacker = pack_tuple
                self.register_new_codec(argname, packer, unpacker)

    def get_data_size(self, mode: int, application_id: int):
        return self.data_dict[mode][application_id][1] + DATA_OFFSET

    def register_new_codec(self, arg: str, packer, unpacker):
        if arg in self.packers:
            raise SerializationException(
                "Trying to register a codec for an existing argument"
            )

        self.packers[arg] = packer
        self.unpackers[arg] = unpacker

    def pack_data(self, mode: int, datatype_id: int, **kwargs) -> bytes:
        try:
            func_args, buffer_size = self.data_dict[mode][datatype_id]
            data_buffer = bytearray(buffer_size)
            offset = 0
            for arg in func_args:
                off = self.packers[arg](data_buffer, offset, kwargs[arg])
                offset += off
            return pack_bytes(mode, datatype_id, data_buffer)
        except StructError as exc:
            raise SerializationException(str(exc))
        except KeyError as exc:
            if arg is not None:
                raise SerializationException(
                    f"KeyError occured for arg: {arg}, using Mode: {mode}; DataTypeID: {datatype_id} "
                    f"KeyError was: {str(exc)}"
                )
            else:
                raise SerializationException(
                    f"KeyError occurred, no such Data Type: {datatype_id} for mode: {mode} "
                    f"KeyError was: {str(exc)}"
                )

    def unpack_data(self, data: bytes):
        mode, datatype_id, arg_data = unpack_bytes(data)
        func_args, buffer_size = self.data_dict[mode][datatype_id]
        if (buffer_size + DATA_OFFSET) != len(data):
            raise DeserializationException(
                f"Received data with data len: {len(data)}, but expected length: {buffer_size}; "
                f"for data with Mode: {mode}, DataTypeID: {datatype_id}"
            )
        offset = 0
        kwargs = dict()
        for arg in func_args:
            off, value = self.unpackers[arg](arg_data, offset)
            kwargs[arg] = value
            offset += off
        return mode, datatype_id, kwargs

    def register_mode_datatypes(self, mode: int, application_ids_to_arg_tuples: dict):
        if mode in self.data_dict:
            raise SerializationException(
                f"Trying to register mode {mode} that already exists"
            )
        # Should be a dict of application ids -> tuple (ordered list of arguments, length of arguments)
        # argument names must be registered separately
        self.data_dict[mode] = application_ids_to_arg_tuples

dh = DataHandler()
dh.pack_data(0,1,gyro = 726)