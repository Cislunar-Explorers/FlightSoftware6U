import struct

# All packing is done in Big Endian as specified by > argument to struct module


def pack_bool(buf, off, val):
    struct.pack_into(">?", buf, off, val)
    return 1


def unpack_bool(buf, off):
    val = struct.unpack_from(">?", buf, off)[0]
    return 1, val


def pack_unsigned_int8(buf, off, val):
    struct.pack_into(">B", buf, off, val)
    return 1


def unpack_unsigned_int8(buf, off):
    val = struct.unpack_from(">B", buf, off)[0]
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


MAXSTRINGLEN = 2 ** 16 - 5  # TODO: No idea if this value is correct


def pack_str(buf, off, s):
    assert len(s) <= MAXSTRINGLEN, "trying to pack too long of a string"
    sbytes = bytes(s, "utf-8")
    struct.pack_into(">H%ds" % (len(s),), buf, off, len(s), sbytes)
    return 2 + len(s)


def unpack_str(buf, off):
    slen = struct.unpack_from(">H", buf, off)[0]
    sbytes = buf[off + 2: off + 2 + slen]
    return 2 + slen, str(sbytes, "utf-8")

packer_dict = {
'bool': (pack_bool,unpack_bool),
'uint8': (pack_unsigned_int8,unpack_unsigned_int8),
'short': (pack_unsigned_short,unpack_unsigned_short),
'int': (pack_unsigned_int,unpack_unsigned_int),
'long': (pack_unsigned_long,unpack_unsigned_long),
'float': (pack_float,unpack_float),
'double': (pack_double,unpack_double),
'string': (pack_str,unpack_str)
}
