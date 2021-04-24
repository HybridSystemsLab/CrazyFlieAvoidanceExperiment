import logging
import time
import zmq
import simplejson as json
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI to the Crazyflie to connect to
uri = 'radio://0/100/2M/E7E7E7E7E7'

 
def motor_set(cf, m1=0, m2=0, m3=0, m4=0):
    cf.param.set_value("motorPowerSet.m1", m1)
    cf.param.set_value("motorPowerSet.m2", m2)
    cf.param.set_value("motorPowerSet.m3", m3)
    cf.param.set_value("motorPowerSet.m4", m4)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    context = zmq.Context()
    # recieve work
    rcvsocket = context.socket(zmq.PULL)
    rcvsocket.bind("tcp://127.0.0.1:1212")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value("motorPowerSet.enable", 1)
        motor_set(scf.cf)
        time.sleep(0.2)
        print('Ready to send')
        sys.stdout.flush()

        while(True):
            mVals = rcvsocket.recv_json()
            print(mVals)
            sys.stdout.flush()
#            motor_set(scf.cf, mVals['m1'],mVals['m2'],mVals['m3'],mVals['m4'])
            
        motor_set(scf.cf)
