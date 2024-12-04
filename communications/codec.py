from fsw_utils.struct import packer_dict
from typing import Dict, Any


ALLOWABLE_DATATYPES = tuple(packer_dict.keys())


class Codec:
    """Codec: a class for storing all the data relevant to packing and unpacking a variable.
    `name` is the name of the codec which you are defining the codecs for,
    `dtype` must be one of `ALLOWABLE_DATATYPES`"""

    def __init__(self, name: str, dtype: str) -> None:
        self.name = name
        self.packer = packer_dict[dtype][0]
        self.unpacker = packer_dict[dtype][1]
        self.num_bytes: int = packer_dict[dtype][2]

    def pack(self, value: Any) -> bytes:
        """Takes in a value and converts it into bytes to be sent over radio"""
        buf = bytearray(self.num_bytes)
        self.packer(buf, 0, value)
        return buf

    def unpack(self, to_unpack: bytes) -> Dict[str, Any]:
        """Takes bytes data received over radio and converts it into a variable"""
        return {self.name: self.unpacker(to_unpack, 0)[1]}
