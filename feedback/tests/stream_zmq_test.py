import zmq
from feedback import optitrack_feedback
import msgpack
import numpy as np
import math
from collections import namedtuple
from feedback.transformations import euler_from_quaternion
import time

context = zmq.Context()
optitrack_conn = context.socket(zmq.REP)
optitrack_conn.bind("tcp://204.102.224.3:5000")

while 1:
    packet = optitrack_conn.recv()
    filtered = msgpack.unpackb(packet)
    #print "Got", filtered[3], filtered[4], filtered[5], filtered[6]
    optitrack_conn.send(b'Ack', zmq.NOBLOCK)

    EulerAngles = namedtuple('EulerAngles', 'pitch roll yaw')

    res = euler_from_quaternion([filtered[3], filtered[4], filtered[5], filtered[6]], axes='syxz')
    res = [elem*(180/math.pi) for elem in res]
    print filtered[0], filtered[1], filtered[2], res
    #time.sleep(.5)

    #print res, filtered[-1], filtered[-2]

    """
    Rbn = np.matrix([[q[1]**2 + q[2]**2 - q[3]**2 - q[4]**2,  2*(q[2] * q[3] - q[1] * q[4]), 2*(q[1] * q[3] + q[2] * q[4])],
       [2*(q[2] * q[3] + q[1] * q[4]), q[1]**2 - q[2]**2 + q[3]**2 - q[4]**2,  2*(q[3] * q[4] - q[1] * q[2])],
       [2*(q[2] * q[4] - q[1] * q[3]), 2*(q[1] * q[2] + q[3] * q[4]), q[1]**2 - q[2]**2 - q[3]**2 + q[4]**2]])

    R = np.transpose(Rbn)
    phi   =  math.atan2(R[1, 2], R[2, 2])*(180/math.pi)
    theta = -math.asin(R[0, 2])*(180/math.pi)
    psi   =  -math.atan2(R[0, 1], R[0, 0])*(180/math.pi)
    print(phi, theta, psi)
    """
