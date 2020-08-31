from power_structs import *


if __name__ == "__main__":
    logging.debug("----TESTING SEND/RECV----\n")

    logging.debug("sending struct eps_config2_t -->")
    struct = eps_config2_t()
    logging.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 10
    logging.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 7
    logging.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 5
    logging.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 8
    logging.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (10, 2)
    logging.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (1, 2, 3, 4)
    logging.debug(
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
    logging.debug("struct in bytes:    ")
    acc = []
    for n in byte:
        acc += [n]
    logging.debug(acc)
    array = c_byteArrayToBytes(byte)
    logging.debug("byte array:")
    acc = []
    for n in array:
        acc += [n]
    logging.debug(acc)

    logging.debug("\n--> receiving struct")
    structend = c_bytesToStruct(array, "eps_config2_t")
    logging.debug("struct:   " + str(structend))
    logging.debug("maxvolt:  " + str(structend.batt_maxvoltage))
    logging.debug("safevolt: " + str(structend.batt_safevoltage))
    logging.debug("critvolt: " + str(structend.batt_criticalvoltage))
    logging.debug("normvolt: " + str(structend.batt_normalvoltage))
    logging.debug(
        "res1:     "
        + "("
        + str(structend.reserved1[0])
        + ", "
        + str(structend.reserved1[1])
        + ")"
    )
    logging.debug(
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
    logging.debug("----end----\n")

    logging.debug("sending struct TestingStruct -->")
    teststruct = TestingStruct()
    logging.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    logging.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    logging.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    logging.debug("field3:   " + str(teststruct.field3))
    send = c_structToBytes(teststruct)
    logging.debug(
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

    logging.debug("\n--> receiving struct")
    recv = c_bytesToStruct(send, "TestingStruct")
    logging.debug("struct: " + str(recv))
    logging.debug("field1: " + str(recv.field1))
    logging.debug("field2: " + str(recv.field2))
    logging.debug("field3: " + str(recv.field3))
    logging.debug("----end----\n")

    logging.debug("----TESTING HELPERS----\n")

    logging.debug(">testing c_structToByteArray with TestingStruct")
    teststruct = TestingStruct()
    logging.debug("struct:   " + str(teststruct))
    teststruct.field1 = 25
    logging.debug("field1:   " + str(teststruct.field1))
    teststruct.field2 = 255
    logging.debug("field2:   " + str(teststruct.field2))
    teststruct.field3 = 257
    logging.debug("field3:   " + str(teststruct.field3))
    array = c_structToByteArray(teststruct)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logging.debug("bytearray: " + str(acc))
    logging.debug("----end----\n")

    logging.debug(">testing c_byteArrayToBytes")
    logging.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    logging.debug("bytes:     " + str(acc2))
    logging.debug("----end----\n")

    logging.debug(">testing c_bytesToByteArray")
    logging.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logging.debug("bytearray: " + str(acc))
    logging.debug("----end----\n")

    logging.debug(">testing c_byteArrayToStruct")
    logging.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "")
    logging.debug("struct:   " + str(struct))
    struct.field1 = 25
    logging.debug("field1:   " + str(teststruct.field1))
    struct.field2 = 255
    logging.debug("field2:   " + str(teststruct.field2))
    struct.field3 = 257
    logging.debug("field3:   " + str(teststruct.field3))
    logging.debug("----end----\n")

    logging.debug(">testing c_structToByteArray with esp_config2_t")
    struct = eps_config2_t()
    logging.debug("struct:   " + str(struct))
    struct.batt_maxvoltage = 5
    logging.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 9
    logging.debug("safevolt: " + str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 6
    logging.debug("critvolt: " + str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 1
    logging.debug("normvolt: " + str(struct.batt_normalvoltage))
    struct.reserved1 = (1247, 267)
    logging.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    struct.reserved2 = (255, 2, 3, 4)
    logging.debug(
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
    logging.debug("bytearray: " + str(acc))
    logging.debug("----end----\n")

    logging.debug(">testing c_byteArrayToBytes")
    logging.debug("bytearray: " + str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes:
        acc2 += [n]
    logging.debug("bytes:     " + str(acc2))
    logging.debug("----end----\n")

    logging.debug(">testing c_bytesToByteArray")
    logging.debug("bytes:     " + str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)):
        acc += [array[n]]
    logging.debug("bytearray: " + str(acc))
    logging.debug("----end----\n")

    logging.debug(">testing c_byteArrayToStruct")
    logging.debug("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "eps_config2_t")
    logging.debug("struct:   " + str(struct))
    logging.debug("maxvolt:  " + str(struct.batt_maxvoltage))
    logging.debug("safevolt: " + str(struct.batt_safevoltage))
    logging.debug("critvolt: " + str(struct.batt_criticalvoltage))
    logging.debug("normvolt: " + str(struct.batt_normalvoltage))
    logging.debug(
        "res1:     "
        + "("
        + str(struct.reserved1[0])
        + ", "
        + str(struct.reserved1[1])
        + ")"
    )
    logging.debug(
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
    logging.info("----end----\n")

    # g = [1, 2, 3, 4, 5, 6, 7]
    # logging.debug(reverseList(g))
