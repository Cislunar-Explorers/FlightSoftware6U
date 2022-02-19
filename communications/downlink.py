from bitstring import BitArray


def bit_inflation(downlink: bytearray, zero_word: bytes, one_word: bytes):
    # Convert bytes to bits
    downlinkBitString = BitArray(bytes=downlink).bin

    inflatedByteArray = bytearray('', encoding='utf-8')

    # Add two bytes for every bit corresponding to the appropriate word
    for bit in downlinkBitString:
        if bit == '0':
            inflatedByteArray += zero_word
        else:
            inflatedByteArray += one_word

    return inflatedByteArray
