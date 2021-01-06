from power_structs import *


if __name__ == "__main__":
    gom_logger.debug("----TESTING SEND/RECV----\n")

    gom_logger.debug("sending struct eps_config2_t -->")
    struct = eps_config2_t()
    gom_logger.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 10
    gom_logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 7
    gom_logger.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 5
    gom_logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 8
    gom_logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (10, 2)
    gom_logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (1, 2, 3, 4)
    gom_logger.debug(
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
    gom_logger.debug("struct in bytes:    ")
    acc = []
    for n in byte:
        acc += [n]
    gom_logger.debug(acc)
    array = c_byteArrayToBytes(byte)
    gom_logger.debug("byte array:")
    acc = []
    for n in array:
        acc += [n]
    gom_logger.debug(acc)

    gom_logger.debug("\n--> receiving struct")
    structend = c_bytesToStruct(array, "eps_config2_t")
    gom_logger.debug("struct:   " + str(structend))
    gom_logger.debug("maxvolt:  " + str(structend.batt_maxvoltage))
    gom_logger.debug("safevolt: " + str(structend.batt_safevoltage))
    gom_logger.debug("critvolt: " + str(structend.batt_criticalvoltage))
    gom_logger.debug("normvolt: " + str(structend.batt_normalvoltage))
    gom_logger.debug(
        "res1:     "
        + "("
        + str(structend.reserved1[0])
        + ", "
        + str(structend.reserved1[1])
        + ")"
    )
    gom_logger.debug(
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
    gom_logger.debug("----end----\n")

    gom_logger.debug("sending struct TestingStruct -->")
    teststruct = TestingStruct()
    gom_logger.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    gom_logger.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    gom_logger.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    gom_logger.debug("field3:   " + str(teststruct.field3))
    send = c_structToBytes(teststruct)
    gom_logger.debug(
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

    gom_logger.debug("\n--> receiving struct")
    recv = c_bytesToStruct(send, "TestingStruct")
    gom_logger.debug("struct: " + str(recv))
    gom_logger.debug("field1: " + str(recv.field1))
    gom_logger.debug("field2: " + str(recv.field2))
    gom_logger.debug("field3: " + str(recv.field3))
    gom_logger.debug("----end----\n")

    gom_logger.debug("----TESTING HELPERS----\n")

    gom_logger.debug(">testing c_structToByteArray with TestingStruct")
    teststruct = TestingStruct()
    gom_logger.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    gom_logger.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    gom_logger.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    gom_logger.debug("field3:   " + str(teststruct.field3))
    array = c_structToByteArray(teststruct)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    gom_logger.debug("bytearray: " + str(acc))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_byteArrayToBytes")
    gom_logger.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    gom_logger.debug("bytes:     " + str(acc2))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_bytesToByteArray")
    gom_logger.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    gom_logger.debug("bytearray: " + str(acc))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_byteArrayToStruct")
    gom_logger.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "")
    gom_logger.debug("struct:   " + str(struct))
    struct.field1 = 25
    gom_logger.debug("field1:   " + str(teststruct.field1))
    struct.field2 = 255
    gom_logger.debug("field2:   " + str(teststruct.field2))
    struct.field3 = 257
    gom_logger.debug("field3:   " + str(teststruct.field3))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_structToByteArray with esp_config2_t")
    struct = eps_config2_t()
    gom_logger.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 5
    gom_logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 9
    gom_logger.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 6
    gom_logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 1
    gom_logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (1247, 267)
    gom_logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (255, 2, 3, 4)
    gom_logger.debug(
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
    gom_logger.debug("bytearray: " + str(acc))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_byteArrayToBytes")
    gom_logger.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    gom_logger.debug("bytes:     " + str(acc2))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_bytesToByteArray")
    gom_logger.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    gom_logger.debug("bytearray: " + str(acc))
    gom_logger.debug("----end----\n")

    gom_logger.debug(">testing c_byteArrayToStruct")
    gom_logger.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "eps_config2_t")
    gom_logger.debug("struct:   " + str(struct))
    gom_logger.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    gom_logger.debug("safevolt: " + str(struct.batt_safevoltage))
    gom_logger.debug("critvolt: " + str(struct.batt_criticalvoltage))
    gom_logger.debug("normvolt: " + str(struct.batt_normalvoltage))
    gom_logger.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    gom_logger.debug(
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
    gom_logger.info("----end----\n")

    # g = [1, 2, 3, 4, 5, 6, 7]
    # logger.debug(reverseList(g))
