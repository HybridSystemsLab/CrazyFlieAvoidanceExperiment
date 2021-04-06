import numpy as np

class HyCtrl:
    def __init__(self):
        self.p = np.array([0, 0, 0])
        self.v = np.array([0, 0, 0])
        self.R = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.omega = np.array([0, 0, 0])
        self.z = [0, 0, 0]
        self.h = 1
        self.qhat_1 = [1, 0, 0, 0]

    def updateController(self, t, dt, lastState, r):
        DVal = self.D(r,t)
        if(DVal >= 1):
            self.g(dt, lastState, r, DVal)
        if(self.C() == 1):
            thrust, torque = self.f(dt, lastState, r)
        elif(DVal == 0):
            raise Exception("Not in flow or jump set")
        return thrust, torque
    
    def f(self, dt, lastState):
        return 0, [0,0,0]
    
    def g(self, DVal):
        if(DVal == 1):
            self.qhat_1 = argmaxP(self.R, self.qhat_1)
        elif(DVal == 2):
            self.h = -self.h
        elif(DVal == 3):
            self.h = -self.h
            self.qhat_1 = argmaxP(self.R, self.qhat_1)
    
    def C(self):
        argmaxQ()
        return 1
    
    def D(self,r,t):
        return self.D_1(r,t) or self.D_2(r)
        
    def D_1(self,r,t):
        self.getReference(r,t)
        mu = self.makeMu(r,t)
        R_0 = self.makeR_0(r,t)
        a = self.argmaxP(self.R*R_0.T, self.qhat_1)
        return a[0]*self.h <= -self.Qdelta
        
    def D_2(self,r,t):
        mu = self.makeMu(r,t)
        R_0 = self.makeR_0(r,t)
        Q = self.makeQ(self.R*R_0.T)
        return np.linalg.norm(self.qhat_1.T*Q[0:3, 0]) >= self.alpha
    
    @staticmethod
    def argmaxP(R, qhat):
        Q = makeQ(R)
        if(qhat.T*Q[0:3, 0] < qhat.T*Q[0:3, 1]):
            return Q[0:3,1]
        else:
            return Q[0:3,0]
    
    def getReference(self,r,t):
        print "getReference not implemented\n"
        return self.p, self.v, self.R, self.omega
        
    def makeMu(self,r,t):
        return 0
    
    def makeR_0(self,r,t):
        _,_,R_r,_ = self.getReference(r,t)
        mu = self.makeMu(r,t)
        muOverMuNorm = mu/np.linalg.norm(mu)
        gamma = -self.S(np.matmul(R_r,np.array([0,0,1]).T))*muOverMuNorm;
        squareTerm = (self.S(gamma)^2)/(1-np.matmul(np.array([0,0,1]),R_r.T)*muOverMuNorm)
        R_0 = (np.identity(3) + self.S(gamma) + squareTerm)*R_r
        return 
    
    @staticmethod
    def makeQ(R):
        tr = np.trace(R)
        qw = 1
        qx = 0
        qy = 0
        qz = 0
        if(tr > 0):
            S = (tr + 1)**0.5 * 2
            qw = S/4
            qx = (R[2,1] - R[1,2])/S
            qy = (R[0,2] - R[2,0])/S
            qz = (R[1,0] - R[0,1])/S
        elif((R[0,0]>R[1,1]) and (R[0,0] > R[2,2])):
            S = ((1+R[0,0]-R[1,1]-R[2,2])**0.5)*2
            qw = (R[2,1] - R[1,2])/S
            qx = S/4
            qy = (R[0,1] + R[1,0])/S
            qz = (R[0,2] + R[2,0])/S
        elif(R[1,1] > R[2,2]):
            S = ((1+R[1,1]-R[0,0]-R[2,2])**0.5)*2
            qw = (R[0,2] - R[2,0])/S
            qx = (R[0,1] + R[1,0])/S
            qy = S/4
            qz = (R[1,2] + R[2,1])/S
        else:
            S = ((1+R[2,2]-R[0,0]-R[1,1])**0.5)*2
            qw = (R[1,0] - R[0,1])/S
            qx = (R[0,2] + R[2,0])/S
            qy = (R[1,2] + R[2,1])/S
            qz = S/4
        return np.array([qw,qx,qy,qz])
    
    @staticmethod
    def S(x):
        print "Input for S"
        print x
        a = np.array([0, -x[2], x[1]])
        b = np.array([x[2], 0, -x[0]])
        c = np.array([-x[1], x[0], 0])
        d = np.array([a, b, c])
        return a
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        