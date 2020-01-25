from __future__ import print_function
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

#def recv_date(vehicle_name)

    if sys.argv[2:]:
        version = tuple(map(int, sys.argv[2]))
    else:
        version = (2, 7, 0, 0)  # the latest SDK version

    dsock = rx.mkdatasock()
    count = 0
    #while count < max_count:
    data = dsock.recv(rx.MAX_PACKETSIZE)
    packet = rx.unpack(data, version=version)
    if type(packet) is rx.SenderData:
        version = packet.natnet_version
        #print("NatNet version received:", version)
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
        #print(dumps(packet._asdict(), indent=4))
        #count += 1
    return contortion, euler

