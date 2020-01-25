from __future__ import print_function
import socket
import struct
from collections import namedtuple
from platform import python_version_tuple
import optirx as rx
import sys
from Quaternion import Quat

try:
    from simplejson import dumps, encoder

    encoder.FLOAT_REPR = lambda o: ("%.4f" % o)
except ImportError:
    from json import dumps, encoder

    encoder.FLOAT_REPR = lambda o: ("%.4f" % o)


def recv_data():
    # def recv_date(vehicle_name)
    # the second optional command line argument
    # is the version string of the NatNet server;
    # may be necessary to receive data without
    # the initial SenderData packet
    if sys.argv[2:]:
        version = tuple(map(int, sys.argv[2]))
    else:
        version = (2, 7, 0, 0)  # the latest SDK version

    # "Create a data socket."
    PORT = 1511
    MULTICAST_ADDRESS = '239.255.42.99'
    SOCKET_BUFSIZE = 1024

    datasock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    datasock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    datasock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    datasock.bind((MULTICAST_ADDRESS, PORT))

    # join a multicast group
    mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_ADDRESS), socket.INADDR_ANY)
    datasock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    datasock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    datasock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_BUFSIZE)
    datasock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    datasock.settimeout(3)
    print(datasock.gettimeout())

    # dsock = rx.mkdatasock(ip_address='', multicast_address='239.255.42.99', port=1511)
    while 1:
        print("looping")
        data = datasock.recv(rx.MAX_PACKETSIZE)
        packet = rx.unpack(data, version=version)
        if type(packet) is rx.SenderData:
            version = packet.natnet_version
            # print("NatNet version received:", version)
        if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
            packet_dict = packet._asdict()
            all_bodies = packet_dict['rigid_bodies']
            """
            @todo: ought to be able to specify which rigid body we're looking for, i.e. all_bodies['name_of_helicopter']
            Take a second look at Optitrack data (get rigid body by name), then return data for just the helicopter
            """
            for body in all_bodies:
                contortion = body._asdict()['orientation']
                euler = Quat([elem for elem in contortion]).equatorial
                print(euler)
                # print(dumps(packet._asdict(), indent=4))
                # count += 1
    return contortion


recv_data()
