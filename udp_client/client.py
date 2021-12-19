import json
import socket
import logging


class Client:
    def __init__(self, addr: str, port: int):
        """
        create a UDP socket at client side

        @param: addr: the IP address of the server
        @param: port: the port that the server is running in
        """
        self.serverAddressPort = (addr, port)
        self.clientSock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def send_data(self, data_dict):
        """
        send python dict as a json dict to server.

        @param: data_dict: the data, in python dictionary format
        """
        json_dict = json.dumps(data_dict).encode("utf-8")
        self.clientSock.sendto(json_dict, self.serverAddressPort)
        # print for testing purposes
        logging.info("Sent data over UDP")
        logging.debug(data_dict)
