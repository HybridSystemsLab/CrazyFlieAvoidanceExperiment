import logging
import time
import zmq
import simplejson as json
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.callbacks import Caller

# URI to the Crazyflie to connect to
uri = 'radio://0/100/250K/E7E7E7E7E7'
 
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
    
    crazyflie = Crazyflie()
    crazyflie.connected.add_callback(connected)
    crazyflie.open_link("radio://0/100/250K")

    crazyflie.param.set_value("motorPowerSet.enable", 1)
    motor_set(crazyflie)
    time.sleep(0.2)
    print('Ready to send')
    sys.stdout.flush()

    data_received_cb = Caller()
 
    while(True):
        mVals = rcvsocket.recv_json()
#            print(mVals)
#            sys.stdout.flush()
#        motor_set(crazyflie, mVals['m1'],mVals['m2'],mVals['m3'],mVals['m4'])
        motor_set(crazyflie)
            
    motor_set(crazyflie)

# Callback called when the connection is established to the Crazyflie
def connected(link_uri):
    crazyflie.log.add_config(logconf)

    if logconf.valid:
        logconf.data_received_cb.add_callback(data_received_callback)
        logconf.start()
    else:
        print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")

def data_received_callback(timestamp, data, logconf):
    print("[%d][%s]: %s" % (timestamp, logconf.name, data))
