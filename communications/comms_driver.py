from queue import Queue
from threading import Thread
from socketserver import UDPServer, BaseRequestHandler
import socket


HOSTNAME = "127.0.0.1"
PORT = 5000


class Comms:
    def __init__(self, *, queue: Queue):
        self.queue = queue

    # Implement this function to add received commands to the queue forever
    def read_telemetry_forever(self):
        raise NotImplementedError

    # Receive commands and add them to the queue indefinitely
    def listen(self):
        self.listening_thread = Thread(target=self.read_telemetry_forever)
        self.listening_thread.start()

    # Override this method to assure safety for specific implementation
    def stop(self):
        print("Halting comms thread...")
        if self.listening_thread.is_alive() is True:
            self.listening_thread.terminate()

    def send_packet(self, packet: bytes):
        raise NotImplementedError


# TODO look into Threading/Forking Servers as documented here:
# https://docs.python.org/2/library/socketserver.html
class UDPHandler(BaseRequestHandler):
    # TODO add validation and parsing of the commands themselves
    # For now, it simply adds the data to the queue
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        print(f"{self.client_address[0]} wrote: {data}")
        self.queue.put(data)
        # Change what the response should be here
        socket.sendto(data.upper(), self.client_address)


# TODO fix initializer of IPComms
class IPComms(Comms):
    def __init__(
        self,
        *,
        queue: Queue,
        server_host: str = HOSTNAME,
        server_port: int = PORT,
        client_host: str = HOSTNAME,
        client_port: int = PORT,
        await_response=False,
    ):
        super().__init__(queue=queue)
        UDPHandler.queue = self.queue
        self.server_address = (server_host, server_port)
        self.server = UDPServer(self.server_address, UDPHandler)
        self.client_address = (client_host, client_port)
        self.await_response = await_response

    def read_telemetry_forever(self):
        self.server.serve_forever()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()

    # XXX
    def send_packet(self, packet: bytes):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(packet, self.client_address)
            print(f"Sent packet: {packet}")
            if self.await_response:
                received_data = str(sock.recv(1024), "utf-8")
                print(f"Received: {received_data}")
        finally:
            sock.close()


class AX5043Comms(Comms):
    def __init__(self, *, queue: Queue):
        super().__init__(queue=queue)
        # Initialize AX5043 Chip here

    def read_telemetry_forever(self):
        raise NotImplementedError

    def send_packet(self, data: bytes):
        raise NotImplementedError


# TODO pass handler function to init of CommunicationsSystem
# to handle command parsing
class CommunicationsSystem:
    def __init__(
        self, *, queue: Queue, use_ax5043=True, host=None, port=None,
    ):
        if use_ax5043 is True:
            self.comms = AX5043Comms(queue=queue)
        else:
            host = host if host is not None else HOSTNAME
            port = port if port is not None else PORT
            self.comms = IPComms(queue=queue, server_host=host, server_port=port)

    def listen(self):
        self.comms.listen()

    def stop(self):
        self.comms.stop()

    def send_packet(self, data):
        self.comms.send_packet(data)

    def __enter__(self):
        self.listen()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# CommunicationsSystem can be used for both ground station and satellite
# should have the functionality of listening on one thread and continuously
# reading and still be able to send a packet
if __name__ == "__main__":
    q = Queue()
    comms = CommunicationsSystem(queue=q, use_ax5043=False)
    comms.listen()
    while True:
        if q.empty() is not True:
            print(f"Read {q.get()} from queue")
        else:
            comms.send_packet(b"sent packet because q was empty")
