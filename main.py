import math
import zmq
from motive_client import OptitrackProcessor as optitrack
import time
from feedback.frames import FrameHistory
import pid
import simplejson
import trajectoryplanner as plan
import threading
import thread
import logging
from multiprocessing.pool import ThreadPool
import sys

def input_thread(L):
    raw_input()
    L.append(None)


def sendToClient(roll, pitch, thrust, yaw):
    cmd["ctrl"]["roll"] = roll
    cmd["ctrl"]["pitch"] = pitch
    cmd["ctrl"]["thrust"] = thrust
    cmd["ctrl"]["yaw"] = yaw
    client_conn.send_json(cmd)


def resetPIDs():
    r_pid.reset_dt()
    p_pid.reset_dt()
    y_pid.reset_dt()
    v_pid.reset_dt()
    vv_pid.reset_dt()

    vv_pid.Integrator = 0.0
    r_pid.Integrator = 0.0
    p_pid.Integrator = 0.0
    y_pid.Integrator = 0.0


def validVelocity(dist, dt, threshold):
    if(abs(dist)>threshold):
        velocity = dist/dt
    else:
        velocity = 0

    return velocity



"""
connection to optitrack NOTE: ip address might change
"""
processor = optitrack(ip_address='128.114.56.19')

"""
ZMQ setup to communicate with client
"""
context = zmq.Context()
client_conn = context.socket(zmq.PUSH)
client_conn.connect("tcp://127.0.0.1:1212")

# PID constants
# rollP, rollI, rollD = 13, .6, 10
# pitchP, pitchI, pitchD = 13, .6, 10
rollP, rollI, rollD = 13, 0, 10
pitchP, pitchI, pitchD = 13, 0, 10

#rollP, rollI, rollD = 12, .2, 1
#pitchP, pitchI, pitchD = 12, .2, 1
yawP, yawI, yawD = .6, 0, 0



# Roll, Pitch and Yaw PID controllers
r_pid = pid.PID_RP(name="roll", P=rollP, I=rollI, D=rollD, Integrator_max=11, Integrator_min=-11, set_point=0,
                   set_point_max=1.5, set_point_min=-1.5)
p_pid = pid.PID_RP(name="pitch", P=pitchP, I=pitchI, D=pitchD, Integrator_max=11, Integrator_min=-11, set_point=0,
                   set_point_max=1.3, set_point_min=-1.3)
y_pid = pid.PID_RP(name="yaw", P=yawP, I=yawI, D=yawD, Integrator_max=5, Integrator_min=-5, set_point=0)


vertP, vertI, vertD = 0.6, 1.2, 0
#vertP, vertI, vertD = 3, 1, 0
velP, velI, velD = .75, 1.0e-4, 0


# Vertical position and velocity PID loops
v_pid = pid.PID_RP(name="position", P=vertP, I=vertI, D=vertD, Integrator_max=1, Integrator_min=-1, set_point=.4)

# veloctiy pid?
vv_pid = pid.PID_RP(name="velocity", P=velP, I=velI, D=velD, set_point=0)


"""
Initialize command obj
"""

cmd = {
    "version": 1,
    "client_name": "N/A",
    "ctrl": {
        "roll": 0.1,
        "pitch": 0.1,
        "yaw": 0.0,
        "thrust": 0.0
    }
}

x, y, z, angle, yaw, roll, pitch = 0, 0, 0, 0, 0, 0, 0
yaw_sp = 0
calibrated = 0
calibration_state = []


"""
Trajectory planner thread set up
"""

state = [0,0,0,0,0,0,0,0,0]

logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] (%(threadName)-10s) %(messages)s',)

pool = ThreadPool(processes=1)
threadFin = True



if __name__ == "__main__":

    # allows enter key to quit program
    L = []
    thread.start_new_thread(input_thread, (L,))

    # projectile lists and variables
    prevCoords = [0.0, 0.0, 0.0]
    curCoords = [0.0, 0.0, 0.0]
    projdistance = [0.0, 0.0, 0.0]
    projvelocity = [0.0, 0.0, 0.0]
    prevTime = time.time()


    # initialize all the variables
    frame_history = FrameHistory(filtering=False)
    dt, prev_z = 0, 0
    prev_t = time.time()

    sendToClient(0, 0, 0, 0)
    resetPIDs()

    r_pid.set_point_to(0)
    p_pid.set_point_to(0)

    detectedCount = 0
    output_data = False

    while True:

        if L:
            break
        try:

            # safety in the case of loosing track of quad
            if detectedCount > 50:
                print"Quadcopter lost for more than 50 frames: exiting"
                L.append('q')

            # Get rigid body data from Optitrack (8, 32 b4)
            frame_data = processor.recv_data(rigid_body_ids=[1], mode='quaternion')
            
            if frame_history.update(frame_data) is None:
                detectedCount += 1

                continue
            else:
                detectedCount = 0



            # assign current state values of the Crazyflie and projectile
            state = frame_history.filtered_frame.state

            #current possition of Quad and projectile
            x, y, z = state[0], state[1], state[2]
            if(len(frame_data)==1):
				curCoords=curCoords;
            else:
				curCoords[0], curCoords[1], curCoords[2] = -frame_data[1][0], frame_data[1][2], frame_data[1][1]
            
            if(calibrated==0):
                print "Calibrating"
                if len(calibration_state) == 0:
                    calibration_state = state[0:2]
                    sendToClient(6,0, 50, 0)
                    time.sleep(0.4)
                else:
                    x_cal = state[0] - calibration_state[0]
                    y_cal = state[1] - calibration_state[1]
                    if(x_cal != 0):
                        angle = math.degrees(math.atan(y_cal/x_cal))
                    else:
                        angle = 180 - math.sign(y_cal)
                    if(abs(angle)<5):
                        calibrated = 1;
                        print "\n\n\n\n\n---Calibrated---\n\n\n\n\n"
                    else:
                        calibration_state = []
                        calibrated = 0
                        yaw_out = yaw = y_pid.update(((angle - yaw_sp + 180) % 360) - 180)
                        sendToClient(0,0, 50, yaw_out)
                        time.sleep(0.4)
                    
                
            else:
                print "Running"
                # if the specified rigid bodies are in view
                data_out = open('Trajectory_actual.txt', 'a+')
    
    
                if (time.time() - prevTime > .002):
                # get the time step
                    curTime = time.time()
                    projdt = curTime - prevTime
    
    
    
                    projdistance[0] = curCoords[0] - prevCoords[0]
                    projdistance[1] = curCoords[1] - prevCoords[1]
                    projdistance[2] = curCoords[2] - prevCoords[2]
    
                    # only valid velocity if the object has moved more than .75 mm in any direction
                    projvelocity[0] = validVelocity(projdistance[0], projdt, .005)
                    projvelocity[1] = validVelocity(projdistance[1], projdt, .005)
                    projvelocity[2] = validVelocity(projdistance[2], projdt, .005)
    
                    # first packet that is send has a huge velocity
                    # -- we shouldn't be seeing any high velocities
                    if (abs(projvelocity[0]) > 30):
                        projvelocity[0] = 0
                    if (abs(projvelocity[1]) > 30):
                        projvelocity[1] = 0
                    if (abs(projvelocity[2]) > 30):
                        projvelocity[2] = 0
    
                    # store the time step and store the last coordinates
                    prevTime = curTime
                    prevCoords[0], prevCoords[1], prevCoords[2] = curCoords[0], curCoords[1], curCoords[2]
    
                # create the state vector
                state = [float(x), float(y), float(z), float(curCoords[0]), float(curCoords[1]), float(curCoords[2]),
                        float(projvelocity[0]), float(projvelocity[1]), float(projvelocity[2])]
    
    
                
                # if the thread has completed a round, start it again with the most recent data
                if (threadFin == True):
                    TrajectoryPlanner = pool.apply_async(plan.trajectory, tuple(state))  # tuple of args for foo
                    threadFin = False
    
                # if the thread is ready, get the planned trajectory and set PIDs
                if (TrajectoryPlanner.ready()):
                    threadFin = True
    
                    destination = TrajectoryPlanner.get()  # get the return value from your function.
    
                    xdes = destination[0]
                    ydes = destination[1]
                    #zdes = float(destination[2])
                    collision = destination[3]
                    # print("coordinates --> (" + str(xdes) + ", " + str(ydes) + ")")
                    print("coordinates (" + str(x) + ", " + str(y) + ") --> (" + str(xdes) + ", " + str(ydes) + ")")
                    sys.stdout.flush()
                    r_pid.set_point_to(xdes)
                    p_pid.set_point_to(ydes)
    
                    if(collision == True):
    
                        # output_data=True #start writing to file the trajectories
    
                        print("Collision detected --> (" + str(xdes) + ", " + str(ydes) + ")")
                        sys.stdout.flush()
    
    
                if(output_data == True):
                    # write to file about the actual vechile trajectory and the actual projectile trajectory
                    x_p, y_p, z_p = curCoords[0], curCoords[1], curCoords[2]
    
                    data_out.write(str(x))
                    data_out.write(',')
                    data_out.write(str(y))
                    data_out.write(',')
                    data_out.write(str(z))
                    data_out.write(',')
                    data_out.write(str(x_p))
                    data_out.write(',')
                    data_out.write(str(y_p))
                    data_out.write(',')
                    data_out.write(str(z_p))
                    data_out.write('\n')
    
                data_out.close()
    
    
                ####################
                # Roll, Pitch, Yaw #
                ####################
                roll_sp = roll = r_pid.update(x)
                pitch_sp = pitch = p_pid.update(y)
                #print "Yaw PID in:"
                #print ((angle - yaw_sp + 180) % 360) - 180
                yaw_out = yaw = y_pid.update(0)
                #yaw_out = yaw = y_pid.update(((angle - yaw_sp + 180) % 360) - 180)
    
                #############################
                # Update vertical PID loops #
                #############################
                velocity = v_pid.update(z)
                velocity = max(min(velocity, 10), -10)  # Limit vertical velocity between -1 and 1 m/sec
                vv_pid.set_point = velocity
                dt = (time.time() - prev_t)
                if (dt == 0):
                    #print("DT is zero!!!")
                    dt = 0.008
                curr_velocity = (z - prev_z) / dt
                #curr_acc = (curr_velocity - prev_vz) / dt
                thrust_sp = vv_pid.update(curr_velocity)
    
                ##########################
                # Update previous values #
                ##########################
                prev_z = z
                #prev_vz = curr_velocity
                prev_t = time.time()
    
                """ Thrust was being generated as a decimal value instead of as percent in other examples """
                thrust_sp = max(min(thrust_sp, 1), 0.40)
    
                ############################################################
                # rotate global pitch and roll to vehicle coordinate frame #
                ############################################################
                #pitch_corr = pitch_sp * math.cos(math.radians(-angle)) - roll_sp * math.sin(math.radians(-angle))
                #roll_corr = pitch_sp * math.sin(math.radians(-angle)) + roll_sp * math.cos(math.radians(-angle))
    
                if (yaw_out < -200):
                    yaw_out = -200
                if (yaw_out > 200):
                    yaw_out = 200
    
                #########################
                # send values to client #
                #########################
    
                #print(x,y,z)
                #print "roll: " + str(roll_corr) + " pitch: " + str(pitch_corr) + " thrust: " + str(thrust_sp * 100) + " yaw: " + str(yaw_out)
    
                #print "X Location:" + str(x) + ", Roll: " + str(roll) #+ ", Roll input: " + str(roll_corr)
                #print "Y Location:" + str(y) + ", Pitch: " + str(pitch) #+ ", Pitch input: " + str(pitch_corr)
    
                if (not math.isnan(thrust_sp)):
                    sendToClient(roll,pitch, thrust_sp * 100, yaw_out)


        except simplejson.scanner.JSONDecodeError as e:
            print e

    # If we kill the program manually we send the copter a low thrust value so it descends gracefully
    for i in range(0, 20):
        sendToClient(0, 0, 60 - i, 0)
        time.sleep(0.15)

    sendToClient(0, 0, 0, 0)

    print "Vehicle Killed"