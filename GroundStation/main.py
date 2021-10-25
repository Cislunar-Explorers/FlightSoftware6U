from udp_client_tester.servertester import ServerTester

if __name__ == "__main__":
    server = ServerTester("192.168.0.101", 3333)
    server.listen_for_data()