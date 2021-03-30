from bitstring import BitArray
from communications.commands import CommandHandler

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