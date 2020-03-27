from queue import Queue
from socketserver import UDPServer, BaseRequestHandler

class Comms(Thread):
    def __init__(self, queue: Queue):
        self.received_commands_queue = queue

    def listen():
        pass

    def run():
        self.listen()

    def send_packet(self, packet: bytes):
        pass

class UDPHandler(BaseRequestHandler):
    def handle(self):
        self.data = self.request.recv(1024).strip()
        print(f"{self.client_address[0]} wrote: {self.data}")
        # Send back the same data all in uppercase
        self.request.sendall(self.data.upper())

class IPComms(Comms):
    def __init__(self, queue: Queue, host: str, port: int):
        super().__init__(self, queue)
        self.server = UDPServer((host, port), UDPHandler)

    def listen():
        with self.server as server:
            server.serve_forever()

    def send_packet(self, packet: bytes):
        pass
