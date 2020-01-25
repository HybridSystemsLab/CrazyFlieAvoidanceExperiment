import zmq
import time


class PID_RP:
    """

    Discrete PID controller

    """

    def __init__(self, name="N/A", P=1.0, I=0.0, D=10.0, Derivator=0, Integrator=0, Integrator_max=20000,
                 Integrator_min=-20000, set_point=0.0, power=1.0, zmq_connection=None ,set_point_max = 1000,set_point_min = -1000):
        self._zmq = zmq_connection
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.name = name
        self.set_point_max = set_point_max
        self.set_point_min = set_point_min
        self.set_point=set_point

        #e,e1,e2 are e[k], e[k-1], e[k-2] from the difference equation.
        self.e = 0
        self.e1 = 0
        self.e2 = 0

        #u,u1,u2 are u[k], u[k-1], u[k-2] from the difference equation.
        self.u = 0
        self.u1 = 0
        self.u2 = 0

        #Ts is not currently known, so it is set to .01s here. N is the derivative-filter, and is chosen to be 20.
        self.Ts = 0.0086
        self.N = 5

        self.count = 0
        self.timeSum = 0
        self.start = 0


        self.prev_t = 0

        self._z_data = {
            "name": name,
            "data": {
                "OUT": 0.0
            }
        }
    def reset_dt(self):
        self.prev_t = time.time()

    def update(self, currentState):

	    #test code for calculating average time step

        if(self.start == 0):
            self.start = time.time()
        else:
            #self.timeSum += time.time() - self.start
            #self.Ts = time.time() - self.start
            #print(time.time() - self.start)


            self.start = time.time()
            #self.count += 1
            #print(self.timeSum/self.count)
            #print(self.count)



        #Since new output is being calculated, indeces for e and u are shifted back.
        self.e2 = self.e1
        self.e1 = self.e
        self.e = self.set_point - currentState
        self.u2 = self.u1
        self.u1 = self.u

        #All b's and a's below are defined using the discretized PID scheme.
        b0 = self.Kp*(1 + (self.N*self.Ts)) + self.Ki*self.Ts*(1 + (self.N*self.Ts)) + self.Kd*self.N
        b1 = -1*(self.Kp*(2 + (self.N*self.Ts)) + self.Ki*self.Ts + 2*self.Kd*self.N)
        b2 = self.Kp + self.Kd*self.N
        a0 = 1 + self.N*self.Ts
        a1 = -1*(2 + self.N*self.Ts)
        a2 = 1

        # Final control input returned by PID's.
        self.u = -(a1/a0)*(self.u1) - (a2/a0)*(self.u2) + (b0/a0)*self.e + (b1/a0)*self.e1 + (b2/a0)*self.e2

        return self.u

    def set_point_to(self,set_point):
        """
        Initilize the setpoint of PID
        """
        if set_point > self.set_point_max:
            self.set_point = self.set_point_max
            return
        elif set_point < self.set_point_min:
            self.set_point = self.set_point_min
            return
        self.set_point = set_point


    def print_update(self):
        print str(self.name) + " pid average update time: " + str(self.timeSum/self.count)






class PID_T:
    """

    Discrete PID controller

    """

    def __init__(self, name="N/A", P=1.0, I=0.0, D=10.0, Derivator=0, Integrator=0, Integrator_max=20000,
                 Integrator_min=-20000, set_point=0.0, power=1.0, zmq_connection=None ,set_point_max = 1000,set_point_min = -1000):
        self._zmq = zmq_connection
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.name = name
        self.set_point_max = set_point_max
        self.set_point_min = set_point_min
        self.set_point=set_point

        #e,e1,e2 are e[k], e[k-1], e[k-2] from the difference equation.
        self.e = 0
        self.e1 = 0
        self.e2 = 0

        #u,u1,u2 are u[k], u[k-1], u[k-2] from the difference equation.
        self.u = 0
        self.u1 = 0
        self.u2 = 0

        #Ts is not currently known, so it is set to .01s here. N is the derivative-filter, and is chosen to be 20.
        self.Ts = 0.0086
        self.N = 5

        self.count = 0
        self.timeSum = 0
        self.start = 0


        self.prev_t = 0

        self._z_data = {
            "name": name,
            "data": {
                "OUT": 0.0
            }
        }
    def reset_dt(self):
        self.prev_t = time.time()

    def update(self, currentState):



        # Final control input returned by PID's.
        self.u = self.u1 + (self.Kp + self.Kd/self.Ts)*self.e + (-self.Kp - 2*(self.Kd/self.Ts))*self.e1 + (self.Kd/self.Ts)*self.e2

        # Since new output is being calculated, indeces for e and u are shifted back.
        self.e2 = self.e1
        self.e1 = self.e
        self.e = self.set_point - currentState
        self.u2 = self.u1
        self.u1 = self.u

        #print str(self.u) + " = " + str(self.u1) + " + " + str((self.Kp + self.Kd/self.Ts)*self.e) + " + " + str((-self.Kp - 2*(self.Kd/self.Ts)*self.e1)) + " + " + str((self.Kd/self.Ts)*self.e2)



        return self.u

    def set_point_to(self,set_point):
        """
        Initilize the setpoint of PID
        """
        if set_point > self.set_point_max:
            self.set_point = self.set_point_max
            return
        elif set_point < self.set_point_min:
            self.set_point = self.set_point_min
            return
        self.set_point = set_point


    def print_update(self):
        print str(self.name) + " pid average update time: " + str(self.timeSum/self.count)









