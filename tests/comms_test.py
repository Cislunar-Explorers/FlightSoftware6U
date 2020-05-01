from time import sleep

from queue import Queue
from communications.comms_driver import CommunicationsSystem


class TestIPCommunicationsSystem:
    def test_initialize_ip_comms(self):
        q = Queue()
        comms_system = CommunicationsSystem(queue=q, use_ax5043=False)
        comms_system.listen()
        comms_system.stop()

    def test_ip_comms_adds_to_queue(self):
        q = Queue()
        comms_system = CommunicationsSystem(queue=q, use_ax5043=False)
        try:
            comms_system.listen()
            data_packet = b"data_packet"
            comms_system.send_packet(data_packet)
            sleep(1)
            assert not q.empty(), "Data packet was not added to queue"
            assert (
                data_packet == q.get()
            ), "Data packet received does not match what was sent"
        finally:
            comms_system.stop()

    def test_enter_exit(self):
        q = Queue()
        comms_system = CommunicationsSystem(queue=q, use_ax5043=False)
        with comms_system:
            data_packet = b"data_packet"
            comms_system.send_packet(data_packet)
            sleep(1)
            assert q.empty() is False, "Data packet was not added to queue"
            assert (
                data_packet == q.get()
            ), "Data packet received does not match what was sent"
