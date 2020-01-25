from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor

# Here's a UDP version of the simplest possible protocol
class EchoUDP(DatagramProtocol):
    def datagramReceived(self, datagram, address):
        print("get on up!")
        self.transport.write(datagram, address)

def main():
    reactor.listenUDP(36864, EchoUDP())
    reactor.run()

if __name__ == '__main__':
    main()