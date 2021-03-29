import numpy

class HyCtrl:
    def __init__(self):
        self.p = numpy.array([0, 0, 0])
        self.v = numpy.array([0, 0, 0])
        self.R = numpy.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.omega = numpy.array([0, 0, 0])

'''    def updateController(x_a, x_c):
        print("Controller update")
        if(g(x) == 1):
            
        if(f(x) == 1):
            C(x)
        TM = thrust
        TM = 0
        return TM
    
    def f(x):
        return 0
    
    def g(x):
        return 0
    
    def C(x):
        return 0
    
    def D(x):
        return 0'''
