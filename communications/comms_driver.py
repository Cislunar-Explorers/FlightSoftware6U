from queue import Queue
from threading import Thread
from socketserver import UDPServer, BaseRequestHandler
from SocketServer import ThreadingMixIn 
import socket
import AXPythonWrapper


HOSTNAME = "127.0.0.1"
PORT = 5000


class Comms:
    def __init__(self, *, queue: Queue):
        self.queue = queue

    # Receive commands and add them to the queue indefinitely
    def start(self):
       raise NotImplementedError


    # Override this method to assure safety for specific implementation
    def stop(self):
      raise NotImplementedError
        

    def send_packet(self, packet: bytes):
        raise NotImplementedError

    



# basic UDPHandler without threading
class UDPHandler(BaseRequestHandler):
    # TODO add validation and parsing of the commands themselves
    # For now, it simply adds the data to the queue
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        print(f"{self.client_address[0]} wrote: {data}")
        self.queue.put(data)
        # Change what the response should be here
        # to send ints instead of strings
       socket.sendto(str(number).encode('utf-8'), client_address)

class TCPHandler(BaseRequestHandler):
   def handle(self):
        # TCPHandler alternative to the UDPHandler
        self.data = self.rfile.readline().strip()
        print "{} wrote:".format(self.client_address[0])
        print(f"{self.client_address[0]} wrote: {data}")
         self.queue.put(data)
    
        self.wfile.write(self.data.upper())


#Allows us to create multithreaded TCP server
class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass

#Allows us to create multithreaded UDP server
class ThreadedUDPRequestHandler(SocketServer.BaseRequestHandler):

    #override the handle function using multithreading
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        current_thread = threading.current_thread()
        print("{}: client: {}, wrote: {}".format(current_thread.name, self.client_address, data))
        #TODO: parse command, and log deconstructed command (same for telemetry)
        socket.sendto("message received: " + data.upper(), self.client_address)

#Allows us to create multithreaded TCP server
class ThreadedUDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    pass




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
        send_thread: threading.Thread,
    ):
        super().__init__(queue=queue)
        UDPHandler.queue = self.queue
        self.server_address = (server_host, server_port)
        self.server = UDPServer(self.server_address, UDPHandler)
        self.client_address = (client_host, client_port)
        self.await_response = await_response


    # create thread to listen forever
    def start(self):
        HOST, PORT = self.host, self.port
        # with socketserver.UDPServer((HOST, PORT), MyUDPHandler) as server:
        # server.serve_forever() ensures loop forever
       
        server = ThreadedUDPServer((HOST, PORT), ThreadedUDPRequestHandler)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True

        try:
            server_thread.start()
            print("Server started at {} port {}".format(HOST, PORT))
            while True: time.sleep(100)
        except (KeyboardInterrupt, SystemExit):
            self.stop()
        print("Waiting for command")

    #function to stop listening thread
    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        exit()
    
    #function to connect to UDP socket
    def connect(packet: bytes):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(packet, self.client_address)
            msgFromServer = UDPClientSocket.recvfrom(bufferSize)
            msg = "Message from Server {}".format(msgFromServer[0])
            print(msg)
            if self.await_response:
                received_data = str(sock.recv(1024), "utf-8")
                print(f"Received: {received_data}")
        finally:
            sock.close()

    # send packet using IP Comms (override Comms send_packet )
    def send_packet(self, packet: bytes):
        send_thread = threading.Thread(target=connect(packet))
        send_thread.start()

        # try:
        #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #     sock.sendto(packet, self.client_address)
        #     print(f"Sent packet: {packet}")
        #     if self.await_response:
        #         received_data = str(sock.recv(1024), "utf-8")
        #         print(f"Received: {received_data}")
        # finally:
        #     sock.close()


class AX5043Comms(Comms):
    def __init__(self, *, queue: Queue):
        super().__init__(queue=queue)
        # Initialize AX5043 Chip here (already have code written, but just need to look at parameter registers)
        send_thread = threading.Thread(target=send(self, queue))
        send_thread.start()

    #TODO: include register address once specific register that we need
    #      is determined
    #possibly add other wrapper functions for axlib and call when necessary
    def read_telemetry_forever(self):
        while(true):
            AXPythonWrapper.ax_read_reg(address)
            
 
    #start listening thread
    def start(self):
        receive_thread = threading.Thread(target=self.read_telemetry_forever(self))
        receive_thread.start()

    # add command to queue
    def send_packet(self, data: bytes):
        queue.add(data)

    #TODO: include register address once specific register that we need
    #      is determined
    #possibly add other wrapper functions for axlib and call when necessary
    # convert data into char
    def send(self, queue):
        while(not queue.empty()):
            data = queue.get()
            AXPythonWrapper.ax_write_reg(address, data)

            



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

    #start listen thread
    def start(self):
        self.comms.start()

    #stop listen thread
    def stop(self):
        self.comms.stop()

    #send command
    def send_packet(self, data):
        self.comms.send_packet(data)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# CommunicationsSystem can be used for both ground station and satellite
# should have the functionality of listening on one thread and continuously
# reading and still be able to send a packet
if __name__ == "__main__":
    q = Queue()
    comms = CommunicationsSystem(queue=q, use_ax5043=False)
    comms.start()
   
    while True:
        if q.empty() is not True:
            print(f"Read {q.get()} from queue")
        else:
            comms.send_packet(b"sent packet because q was empty")
