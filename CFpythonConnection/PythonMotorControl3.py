import logging
import time
import zmq
import simplejson as json
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
uri = 'radio://0/100/250K/E7E7E7E7E7'

sendTime = 0
 
def motor_set(cf, m1=0, m2=0, m3=0, m4=0):
    cf.param.set_value("motorPowerSet.m1", m1)
    cf.param.set_value("motorPowerSet.m2", m2)
    cf.param.set_value("motorPowerSet.m3", m3)
    cf.param.set_value("motorPowerSet.m4", m4)

#def data_received_callback(timestamp, data, logconf):
#        print("[%d][%s]: %s" % (timestamp, logconf.name, data))

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
        logconf = LogConfig("Mvals", period_in_ms=10)
        scf.cf.log.add_config(logconf)
#        logconf.data_received_cb.add_callback(data_received_callback)
#        logconf.start()

        while(True):
            mVals = rcvsocket.recv_json()
#            print(mVals)
#            sys.stdout.flush()
#            motor_set(scf.cf, mVals['m1'],mVals['m2'],mVals['m3'],mVals['m4'])
#            sendTime = time.time()
            print("%f %f" % (time.time() - mVals['time'], mVals['time']))
            sys.stdout.flush()
            motor_set(scf.cf)
            
        motor_set(scf.cf)
        logconf.stop()
