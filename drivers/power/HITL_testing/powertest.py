from power_structs import *


if __name__ == "__main__":
    logger.debug("----TESTING SEND/RECV----\n")

    logger.debug("sending struct eps_config2_t -->")
    struct = eps_config2_t()
    logger.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 10
    logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 7
    logger.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 5
    logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 8
    logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (10, 2)
    logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (1, 2, 3, 4)
    logger.debug(
        "res2:     "
        + "("
        + str(struct.reserved2[0])
        + ", "
        + str(struct.reserved2[1])
        + ", "
        + str(struct.reserved2[2])
        + ", "
        + str(struct.reserved2[3])
        + ")"
    )
    byte = c_structToByteArray(struct)
    logger.debug("struct in bytes:    ")
    acc = []
    for n in byte:
        acc += [n]
    logger.debug(acc)
    array = c_byteArrayToBytes(byte)
    logger.debug("byte array:")
    acc = []
    for n in array:
        acc += [n]
    logger.debug(acc)

    logger.debug("\n--> receiving struct")
    structend = c_bytesToStruct(array, "eps_config2_t")
    logger.debug("struct:   " + str(structend))
    logger.debug("maxvolt:  " + str(structend.batt_maxvoltage))
    logger.debug("safevolt: " + str(structend.batt_safevoltage))
    logger.debug("critvolt: " + str(structend.batt_criticalvoltage))
    logger.debug("normvolt: " + str(structend.batt_normalvoltage))
    logger.debug(
        "res1:     "
        + "("
        + str(structend.reserved1[0])
        + ", "
        + str(structend.reserved1[1])
        + ")"
    )
    logger.debug(
        "res2:     "
        + "("
        + str(structend.reserved2[0])
        + ", "
        + str(structend.reserved2[1])
        + ", "
        + str(structend.reserved2[2])
        + ", "
        + str(structend.reserved2[3])
        + ")"
    )
    logger.debug("----end----\n")

    logger.debug("sending struct TestingStruct -->")
    teststruct = TestingStruct()
    logger.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    logger.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    logger.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    logger.debug("field3:   " + str(teststruct.field3))
    send = c_structToBytes(teststruct)
    logger.debug(
        "bytearray:"
        + "["
        + str(send[0])
        + ", "
        + str(send[1])
        + ", "
        + str(send[2])
        + ", "
        + str(send[3])
        + "]"
    )

    logger.debug("\n--> receiving struct")
    recv = c_bytesToStruct(send, "TestingStruct")
    logger.debug("struct: " + str(recv))
    logger.debug("field1: " + str(recv.field1))
    logger.debug("field2: " + str(recv.field2))
    logger.debug("field3: " + str(recv.field3))
    logger.debug("----end----\n")

    logger.debug("----TESTING HELPERS----\n")

    logger.debug(">testing c_structToByteArray with TestingStruct")
    teststruct = TestingStruct()
    logger.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    logger.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    logger.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    logger.debug("field3:   " + str(teststruct.field3))
    array = c_structToByteArray(teststruct)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logger.debug("bytearray: " + str(acc))
    logger.debug("----end----\n")

    logger.debug(">testing c_byteArrayToBytes")
    logger.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    logger.debug("bytes:     " + str(acc2))
    logger.debug("----end----\n")

    logger.debug(">testing c_bytesToByteArray")
    logger.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logger.debug("bytearray: " + str(acc))
    logger.debug("----end----\n")

    logger.debug(">testing c_byteArrayToStruct")
    logger.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "")
    logger.debug("struct:   " + str(struct))
    struct.field1 = 25
    logger.debug("field1:   " + str(teststruct.field1))
    struct.field2 = 255
    logger.debug("field2:   " + str(teststruct.field2))
    struct.field3 = 257
    logger.debug("field3:   " + str(teststruct.field3))
    logger.debug("----end----\n")

    logger.debug(">testing c_structToByteArray with esp_config2_t")
    struct = eps_config2_t()
    logger.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 5
    logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 9
    logger.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 6
    logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 1
    logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (1247, 267)
    logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (255, 2, 3, 4)
    logger.debug(
        "res2:     "
        + "("
        + str(struct.reserved2[0])
        + ", "
        + str(struct.reserved2[1])
        + ", "
        + str(struct.reserved2[2])
        + ", "
        + str(struct.reserved2[3])
        + ")"
    )
    array = c_structToByteArray(struct)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logger.debug("bytearray: " + str(acc))
    logger.debug("----end----\n")

    logger.debug(">testing c_byteArrayToBytes")
    logger.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    logger.debug("bytes:     " + str(acc2))
    logger.debug("----end----\n")

    logger.debug(">testing c_bytesToByteArray")
    logger.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logger.debug("bytearray: " + str(acc))
    logger.debug("----end----\n")

    logger.debug(">testing c_byteArrayToStruct")
    logger.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "eps_config2_t")
    logger.debug("struct:   " + str(struct))
    logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    logger.debug("safevolt: " + str(struct.batt_safevoltage))
    logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    logger.debug(
        "res2:     "
        + "("
        + str(struct.reserved2[0])
        + ", "
        + str(struct.reserved2[1])
        + ", "
        + str(struct.reserved2[2])
        + ", "
        + str(struct.reserved2[3])
        + ")"
    )
    logger.info("----end----\n")

    # g = [1, 2, 3, 4, 5, 6, 7]
    # logger.debug(reverseList(g))
