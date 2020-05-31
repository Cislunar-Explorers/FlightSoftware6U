import struct

# All packing is done in Big Endian as specified by > argument to struct module


def pack_bool(buf, off, val):
    struct.pack_into(">?", buf, off, val)
    return 1


def unpack_bool(buf, off):
    val = struct.unpack_from(">?", buf, off)[0]
    return 1, val


def pack_unsigned_short(buf, off, val):
    struct.pack_into(">H", buf, off, val)
    return 2


def unpack_unsigned_short(buf, off):
    val = struct.unpack_from(">H", buf, off)[0]
    return 2, val


def pack_unsigned_int(buf, off, val):
    struct.pack_into(">I", buf, off, val)
    return 4


def unpack_unsigned_int(buf, off):
    val = struct.unpack_from(">I", buf, off)[0]
    return 4, val


def pack_unsigned_long(buf, off, val):
    struct.pack_into(">Q", buf, off, val)
    return 8


def unpack_unsigned_long(buf, off):
    val = struct.unpack_from(">Q", buf, off)[0]
    return 8, val


def pack_float(buf, off, val):
    struct.pack_into(">f", buf, off, val)
    return 4


def unpack_float(buf, off):
    val = struct.unpack_from(">f", buf, off)[0]
    return 4, val


def pack_double(buf, off, val):
    struct.pack_into(">d", buf, off, val)
    return 8


def unpack_double(buf, off):
    val = struct.unpack_from(">d", buf, off)[0]
    return 8, val
