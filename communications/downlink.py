from struct import error as StructError
from sys import maxsize, float_info
import utils.struct as us
from bitstring import BitArray

from flight_modes.flight_mode_factory import FLIGHT_MODE_DICT
from utils.constants import (
    MIN_COMMAND_SIZE,
    MODE_OFFSET,
    ID_OFFSET,
    DATA_LEN_OFFSET,
    DATA_OFFSET,
    MAC_OFFSET,
    MAC_LENGTH,
    MAC,
    COUNTER_OFFSET,
    COUNTER_SIZE
)
from utils.exceptions import (
    SerializationException,
    DownlinkPackingException,
    DownlinkUnpackingException,
)
from utils.struct import (
    pack_unsigned_short,
    unpack_unsigned_short,
    packer_dict
)


def calc_downlink_size(data: bytes):
    return MIN_COMMAND_SIZE + len(data)


def pack_downlink_bytes(counter: int, mode: int, downlink_id: int, data: bytes):
    buf = bytearray(calc_downlink_size(data))
    buf[COUNTER_OFFSET - MAC_LENGTH: COUNTER_OFFSET - MAC_LENGTH + COUNTER_SIZE] = counter.to_bytes(COUNTER_SIZE,'big')
    buf[MODE_OFFSET - MAC_LENGTH] = mode
    buf[ID_OFFSET- MAC_LENGTH] = downlink_id
    pack_unsigned_short(buf, DATA_LEN_OFFSET- MAC_LENGTH, len(data))
    buf[DATA_OFFSET- MAC_LENGTH:] = data
    return MAC + bytes(buf)


def unpack_downlink_bytes(data: bytes):
    mac = data[:MAC_LENGTH]
    counter = int.from_bytes(data[COUNTER_OFFSET:COUNTER_OFFSET + COUNTER_SIZE],'big')
    mode = data[MODE_OFFSET]
    downlink_id = data[ID_OFFSET]
    data_len = unpack_unsigned_short(data, DATA_LEN_OFFSET)[1]
    if data_len + DATA_OFFSET != len(data):
        raise DownlinkUnpackingException(
            f"Incorrect data buffer size: {len(data)}, expected "
            f"{data_len + DATA_OFFSET} for downlink with Mode: {mode}"
            f"DownlinkID: {downlink_id}"
        )
    return mac, counter, mode, downlink_id, data[DATA_OFFSET:]

 
class DownlinkHandler:
    def __init__(self):
        self.downlink_dict = dict()
        self.packers = dict()
        self.unpackers = dict()
        # Packers and unpackers will always have identical set of keys
        # Enforced in practice
        self.register_codecs()
        self.register_downlinks()
    
    def register_codecs(self):
        for mode_id, mode_name in FLIGHT_MODE_DICT.items():
            for argname, arg_type in mode_name.downlink_arg_types.items():
                pack_tuple = packer_dict[arg_type]
                packer, unpacker = pack_tuple
                self.register_new_codec(argname, packer, unpacker)


    def get_downlink_size(self, mode: int, application_id: int):
        return self.downlink_dict[mode][application_id][1] + DATA_OFFSET

    def register_new_codec(self, arg: str, packer, unpacker):
        #if arg in self.packers:
            #raise SerializationException(
            #    "Trying to register a codec for an existing argument"
            #)

        self.packers[arg] = packer
        self.unpackers[arg] = unpacker

    def pack_downlink(self, counter:int, mode: int, downlink_id: int, **kwargs) -> bytes:
        func_args, buffer_size = self.downlink_dict[mode][downlink_id]
        data_buffer = bytearray(buffer_size)
        offset = 0
        try:
            for arg in func_args:
                off = self.packers[arg](data_buffer, offset, kwargs[arg])
                offset += off
            return pack_downlink_bytes(counter, mode, downlink_id, data_buffer)
        except StructError as exc:
            raise DownlinkPackingException(str(exc))
        except KeyError as exc:
            if arg is not None:
                raise DownlinkPackingException(
                    f"KeyError occured for arg: {arg}, using Mode: {mode}; DownlinkID: {downlink_id} "
                    f"KeyError was: {str(exc)}"
                )
            else:
                raise DownlinkPackingException(
                    f"KeyError occurred, no such downlink: {downlink_id} for mode: {mode} "
                    f"KeyError was: {str(exc)}"
                )

    def unpack_downlink(self, data: bytes):
        
        try:
            mac, counter, mode, downlink_id, arg_data = unpack_downlink_bytes(data)
            func_args, buffer_size = self.downlink_dict[mode][downlink_id]
        except:
            raise DownlinkUnpackingException(
                f'Unknown downlink received. Mode: {mode}, Downlink ID: {downlink_id}'
            )
        if (buffer_size + DATA_OFFSET) != len(data):
            raise DownlinkUnpackingException(
                f"Received downlink with data len: {len(data)}, but expected length: {buffer_size}; "
                f"for command with Mode: {mode}, DownlinkID: {downlink_id}"
            )
        offset = 0
        kwargs = dict()
        for arg in func_args:
            off, value = self.unpackers[arg](arg_data, offset)
            kwargs[arg] = value
            offset += off
        return mac, counter, mode, downlink_id, kwargs
    
    def register_downlinks(self):
        for mode_id, mode_name in FLIGHT_MODE_DICT.items():
            
            downlink_dict = {}

            for downlink_id, downlink_data_tuple in mode_name.downlink_codecs.items():
                
                downlink_dict[downlink_id] = downlink_data_tuple
                self.downlink_dict[mode_id] = downlink_dict 

    #Used only for testing
    def register_new_downlink(self, mode_id:int, downlink_id:int, **kwargs):
        
        totalBytes = 0

        try:
            for arg in kwargs:
                if isinstance(kwargs[arg], bool): #boolean
                    self.register_new_codec(arg,us.pack_bool, us.unpack_bool)
                    totalBytes += 1
                elif isinstance(kwargs[arg], int) and abs(kwargs[arg]) <= maxsize: #int
                    self.register_new_codec(arg,us.pack_unsigned_int, us.unpack_unsigned_int)
                    totalBytes += 4
                elif isinstance(kwargs[arg], int): #long
                    self.register_new_codec(arg,us.pack_unsigned_long, us.unpack_unsigned_long)
                    totalBytes += 8
                elif isinstance(kwargs[arg], float) and abs(kwargs[arg]) <= float_info.max: #float
                    self.register_new_codec(arg,us.pack_float, us.unpack_float)
                    totalBytes += 4
                elif isinstance(kwargs[arg], float): #double
                    self.register_new_codec(arg,us.pack_double, us.unpack_double)
                    totalBytes += 8
            
            downlink_args = list(kwargs.keys())
            downlink_data_tuple = (downlink_args, totalBytes)
            self.downlink_dict[mode_id][downlink_id] = downlink_data_tuple

        except:
            raise SerializationException()

def bit_inflation(downlink: bytearray, zero_word:bytes, one_word:bytes):
   
   #Convert bytes to bits
    downlinkBitString = BitArray(bytes=downlink).bin

    inflatedByteArray = bytearray('',encoding='utf-8')
    
    #Add two bytes for every bit corresponding to the appropriate word
    for bit in downlinkBitString:
        if bit == '0':
            inflatedByteArray += zero_word
        else:
            inflatedByteArray += one_word

    return inflatedByteArray

def bit_deflation(downlink: bytearray, zero_word: bytearray, one_word: bytearray):
    
    deflatedBitArray = BitArray('',bin='')
    
    #Recover 
    for i in range(len(downlink)//2):
        byte = downlink[i*2:(i*2)+2]
        
        if byte == zero_word:
            deflatedBitArray += '0b0'
        else:
            deflatedBitArray += '0b1'
    
    return deflatedBitArray.bytes