from power_structs import *

if __name__ == '__main__':
    print("----TESTING SEND/RECV----\n")

    print("sending struct eps_config2_t -->")
    struct = eps_config2_t();			print("struct:   "+str(struct))
    struct.batt_maxvoltage = 10;		print("maxvolt:  "+str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 7;		print("safevolt: "+str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 5; 	print("critvolt: "+str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 8;		print("normvolt: "+str(struct.batt_normalvoltage))
    struct.reserved1 = (10, 2); 		print("res1:     "+"("+str(struct.reserved1[0])+", "+str(struct.reserved1[1])+")")
    struct.reserved2 = (1, 2, 3, 4); 	print("res2:     "+"("+str(struct.reserved2[0])+", "+str(struct.reserved2[1])+", "+str(struct.reserved2[2])+", "+str(struct.reserved2[3])+")")
    byte = c_structToByteArray(struct); print("struct in bytes:    ")
    acc = []
    for n in byte: acc += [n]
    print(acc)
    array = c_byteArrayToBytes(byte);	print("byte array:")
    acc = []
    for n in array: acc += [n]
    print(acc)

    print("\n--> receiving struct")
    structend = c_bytesToStruct(array, "eps_config2_t")
    print("struct:   "+str(structend))
    print("maxvolt:  "+str(structend.batt_maxvoltage))
    print("safevolt: "+str(structend.batt_safevoltage))
    print("critvolt: "+str(structend.batt_criticalvoltage))
    print("normvolt: "+str(structend.batt_normalvoltage))
    print("res1:     "+"("+str(structend.reserved1[0])+", "+str(structend.reserved1[1])+")")
    print("res2:     "+"("+str(structend.reserved2[0])+", "+str(structend.reserved2[1])+", "+str(structend.reserved2[2])+", "+str(structend.reserved2[3])+")")
    print("----end----\n")

    print("sending struct TestingStruct -->")
    teststruct = TestingStruct();			print("struct:   "+str(teststruct))
    teststruct.field1 = 25;					print("field1:   "+str(teststruct.field1))
    teststruct.field2 = 255;				print("field2:   "+str(teststruct.field2))
    teststruct.field3 = 257; 				print("field3:   "+str(teststruct.field3))
    send = c_structToBytes(teststruct); 	print("bytearray:"+"["+str(send[0])+", "+str(send[1])+", "+str(send[2])+", "+str(send[3])+"]")

    print("\n--> receiving struct")
    recv = c_bytesToStruct(send, "TestingStruct"); print("struct: "+str(recv))
    print("field1: "+str(recv.field1))
    print("field2: "+str(recv.field2))
    print("field3: "+str(recv.field3))
    print("----end----\n")


    print("----TESTING HELPERS----\n")

    print(">testing c_structToByteArray with TestingStruct")
    teststruct = TestingStruct();	print("struct:   "+str(teststruct))
    teststruct.field1 = 25;			print("field1:   "+str(teststruct.field1))
    teststruct.field2 = 255;		print("field2:   "+str(teststruct.field2))
    teststruct.field3 = 257;		print("field3:   "+str(teststruct.field3))
    array = c_structToByteArray(teststruct)
    acc = []
    for n in range(len(array)): acc += [array[n]]
    print("bytearray: "+str(acc))
    print("----end----\n")

    print(">testing c_byteArrayToBytes")
    print("bytearray: "+str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes: acc2 += [n]
    print("bytes:     "+str(acc2))
    print("----end----\n")

    print(">testing c_bytesToByteArray")
    print("bytes:     "+str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)): acc += [array[n]]
    print("bytearray: "+str(acc))
    print("----end----\n")

    print(">testing c_byteArrayToStruct")
    print("bytearray: "+str(acc))
    struct = c_byteArrayToStruct(array, "")
    print("struct:   "+str(struct))
    struct.field1 = 25;			print("field1:   "+str(teststruct.field1))
    struct.field2 = 255;		print("field2:   "+str(teststruct.field2))
    struct.field3 = 257;		print("field3:   "+str(teststruct.field3))
    print("----end----\n")

    print(">testing c_structToByteArray with esp_config2_t")
    struct = eps_config2_t();			print("struct:   "+str(struct))
    struct.batt_maxvoltage = 5;			print("maxvolt:  "+str(struct.batt_maxvoltage))
    struct.batt_safevoltage = 9;		print("safevolt: "+str(struct.batt_safevoltage))
    struct.batt_criticalvoltage = 6; 	print("critvolt: "+str(struct.batt_criticalvoltage))
    struct.batt_normalvoltage = 1;		print("normvolt: "+str(struct.batt_normalvoltage))
    struct.reserved1 = (1247, 267); 	print("res1:     "+"("+str(struct.reserved1[0])+", "+str(struct.reserved1[1])+")")
    struct.reserved2 = (255, 2, 3, 4); 	print("res2:     "+"("+str(struct.reserved2[0])+", "+str(struct.reserved2[1])+", "+str(struct.reserved2[2])+", "+str(struct.reserved2[3])+")")
    array = c_structToByteArray(struct)
    acc = []
    for n in range(len(array)): acc += [array[n]]
    print("bytearray: "+str(acc))
    print("----end----\n")

    print(">testing c_byteArrayToBytes")
    print("bytearray: "+str(acc))
    bytes = c_byteArrayToBytes(acc)
    acc2 = []
    for n in bytes: acc2 += [n]
    print("bytes:     "+str(acc2))
    print("----end----\n")

    print(">testing c_bytesToByteArray")
    print("bytes:     "+str(acc2))
    array = c_bytesToByteArray(bytes)
    acc = []
    for n in range(len(array)): acc += [array[n]]
    print("bytearray: "+str(acc))
    print("----end----\n")

    print(">testing c_byteArrayToStruct")
    print("bytearray: " + str(acc))
    struct = c_byteArrayToStruct(array, "eps_config2_t")
    print("struct:   " + str(struct))
    print("maxvolt:  " + str(struct.batt_maxvoltage))
    print("safevolt: " + str(struct.batt_safevoltage))
    print("critvolt: " + str(struct.batt_criticalvoltage))
    print("normvolt: " + str(struct.batt_normalvoltage))
    print("res1:     " + "("+str(struct.reserved1[0])+", "+str(struct.reserved1[1])+")")
    print("res2:     " + "("+str(struct.reserved2[0])+", "+str(struct.reserved2[1])+", "+str(struct.reserved2[2])+", "+str(struct.reserved2[3])+")")
    print("----end----\n")

    #g = [1, 2, 3, 4, 5, 6, 7]
    #print(reverseList(g))
