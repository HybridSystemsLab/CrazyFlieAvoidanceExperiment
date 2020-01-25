"""OptiRX demo: connect to Optitrack on the same machine, print received data.

Usage:
    python optrix_demo.py [number_of_packets_to_print] [natnet_version]

where natnet_version is 2500, 2600, 2700 etc
for Motive 1.5, 1.6 betas, and 1.7.x respectively.
"""


from __future__ import print_function
import socket
import optirx as rx
import sys
from Quaternion import Quat

# pretty-printer for parsed

try:
    from simplejson import dumps, encoder
    encoder.FLOAT_REPR = lambda o: ("%.4f" % o)
except ImportError:
    from json import dumps, encoder
    encoder.FLOAT_REPR = lambda o: ("%.4f" % o)

RIGID_BODIES=3
HOST = '204.104.224.2'
PORT = 8888
''
def update_feedback(rigid_body_ids=None, mode='Euler'):

    def process_state(body, mode):
        location = body['position']
        if mode is 'quaternion':
            orientation = [elem for elem in body['orientation']]
        else:
            orientation = Quat([elem for elem in body['orientation']]).equatorial  # Euler Angles
        return list(location), (orientation).tolist()


    # the second optional command line argument
    # is the version string of the NatNet server;
    # may be necessary to receive data without
    # the initial SenderData packet
    if sys.argv[2:]:
        version = tuple(map(int, sys.argv[2]))
    else:
        version = (2, 7, 0, 0)  # the latest SDK version

    # @todo: really shoudln't be making a socket each time
    dsock = rx.mkdatasock()
    try:
        data = dsock.recv(rx.MAX_PACKETSIZE)
        packet = rx.unpack(data, version=version)
    except:
        pass

    if type(packet) is rx.SenderData:
        version = packet.natnet_version
        print("NatNet version received:", version)
    if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
            all_bodies = packet[RIGID_BODIES]  # elem 3 contains info on all rigid bodies

            output = {}
            for body in all_bodies:
                body = body._asdict()
                if rigid_body_ids is not None:  # check if we care about this rigid body
                    if body['id'] in rigid_body_ids:
                        location, orientation = process_state(body, mode)
                    else:
                        continue
                else:
                    location, orientation = process_state(body, mode)
                output[str(body['id'])] = {'location': location, 'orientation': orientation}
    # time.sleep(.5)
    return location, orientation





