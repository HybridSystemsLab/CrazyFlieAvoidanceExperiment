# standard libraries
import math
import numpy as np
import scipy.integrate
import time

# custom libraries
from vectors import *
from cost import *

def epsilonTarget(x, y, z, target, epsilon):

    if(abs(target[0])-abs(x)<epsilon and abs(target[1])-abs(y)<epsilon and abs(target[2])-abs(z)<epsilon):
        return True
    else:
        return False

target = np.asfarray([0.0, 0.0, 0.4])

def trajectory(x, y, z, px, py, pz, pxdot, pydot, pzdot):

    start = time.time()

    # N: Event horizon of choice.(20)
    N = 20
    
    # t_s = Sampling time--step through which time is discretized.
    t_s = 0.05

    # M: control horizon(6)
    M = 6

    # K: Discretization of angle space used in linear trajectory generation 
    # used in the trajectory optimization process. Increasing K results in a marginal, linear
    # increase in run time.
    K = 5

    # epsilon: distance away from target that we tolerate (0.5)
    epsilon = 1.0

    # vs: Sampling velocity that is analogous to sampling time
    # Product of vs and ts yield the space discretization that structures each linear trajectory.
    v_s = 2

    # quad and projectile prediction lists
    #xiPredict = np.asfarray([[0 for i in range(3)] for j in range(N)])
    pPredict = np.asfarray([[0 for i in range(6)] for j in range(N)])

    # diff eq estimator
    pDE = scipy.integrate.ode(pDot).set_integrator('vode', method='bdf')

    # xi_0: Current vehicle coordinates.
    xi_0 = [x, y, z]
    # p_0: Current projectile coordinates.
    p_0 = [px, py, pz, pxdot, pydot, pzdot]

    x_0 = x
    y_0 = y
    z_0 = 0.4

    # safety net
    R_n = 1.0e-04
    R_d = 1.0e-03
    R_v = 6.0e-02
    R_p = 6.0e-02
    R_col = 0.2#6* (R_n + R_d + R_v + R_p)
    collision = False

    # xiPredict: N x 3 matrix containing predicted vehicle coordinates.
    # pPredict: N x 6 matrix into which N projectile state vectors are stored
    #   each projectile state vector is associated with a different time step in {ts,2ts,...,Nts}.
    # pPredict contains approximate solutions to the initial value problem for the derivative function pDot, 
    #    pDot models projectile dynamics in three dimensions.

    xiPredict = np.asfarray(xi_0)
    pPredict[0][:] = np.asfarray(p_0)


    pDE.set_initial_value(p_0, 0)

    # In this for-loop, vehicle and obstacle predictions are calculated
    for j in range(0, N):
        pPredict[j][:] = pDE.integrate(pDE.t + t_s)
        #xiPredict[j][:] = np.subtract(xiPredict[j - 1][:], t_s * (target - xiPredict[j - 1][:]))

    # MATLAB
    #projfile = open('projectiledata.txt', 'a')
    #for i in range(0,M):
    #    projfile.write(" ".join(map(repr, pPredict[i,0:3])))
    #    projfile.write("\n")
    #projfile.close()

    # collision detection
    
    # collision detection
    for i in range(0, N - 1):
        dist_ij = line2line(xiPredict, xiPredict, pPredict[i][0:3], pPredict[i+1][0:3])
        if (dist_ij <= R_col):
            collision = True
            print("collision detected at " + str(i) + " time stamp")
            break
    
    

    #for i in range(0, N - 1):
    #    if (collision == True):
    #        break
    #    for j in range(0, N - 1):
    #        dist_ij = line2line(xiPredict[i][:], xiPredict[i + 1][:], pPredict[j][0:3], pPredict[j + 1][0:3])
    #        if (dist_ij <= R_col):
    #            collision = True
    #            #print("collision detected: " + str(time.time() - start))
    #            break
    

    # At this point in the code, a collision has been detected, so an optimal evasive trajectory is planned.
    # theta: Discretized array of angle space comprised of K evenly spaced angles.
    theta = np.linspace(0, 2 * math.pi, K)
    # U: K x N x 3 input matrix comprised of K linear trajectories--each of which discretized into N three dimensional coordinates.
    U = np.zeros((K, N, 3))
    for k in range(0, K):
        U[k][0][0:3] = xi_0

        for j in range(1, N):
            U[k][j][0] = x_0 + j * t_s * v_s * math.cos(theta[k])
            #if (U[k][j][0] > 1.6):
            #    U[k][j][0] = 1.6
            #elif (U[k][j][0] < -1.6):
            #    U[k][j][0] = -1.6
            U[k][j][1] = y_0 + j * t_s * v_s * math.sin(theta[k])
            #if (U[k][j][1] > 1.6):
            #    U[k][j][1] = 1.6
            #elif (U[k][j][1] < -0.4):
            #    U[k][j][1] = -0.4
            U[k][j][2] = z_0


    """
    for i in range(0,N):
        U[K][i][0] = float(x_0)
        U[K][i][1] = float(y_0)
        U[K][i][2] = float(z_0)
    """


    # if we are close to the target and there is no predicted collision, then remain at target
    if(not collision and epsilonTarget(x_0, y_0, z_0, target.tolist(), epsilon)):
        u_opt = target.tolist()
    
    # otherwise, run optimaization and trajectory planning 
    else:
        Cost_min = 1.0e10
        u_opt = np.asfarray([[0 for j in range(3)] for t in range(N)])
        safeList = []

        # In this for-loop, input trajectories are analyzed for safety with the same collision detection process as above. Unsafe trajectories are discarded.
        for k in range(0, K):
            unSafe = False
            u_k = U[k][:][:]
            
            # to N-1 because we are doing lines between successive points
            for j in range(0, N-1):

                #print(u_k[j][0:3])
                dist_ij = line2line(u_k[j][0:3], u_k[j + 1][0:3], pPredict[j][0:3], pPredict[j + 1][0:3])
                if (dist_ij) <= R_col:
                    unSafe = True
                    break

            if(unSafe == False):
                safeList.append(u_k)

        # Here, safe trajectories are optimized regarding minimal distance to the target.
        if (len(safeList) == 0):
            # 
            #    problem if there is nothing in the safeList
            #
            u_opt = target.tolist()
        else:
            #endpoints = [0.0, 0.6, 0.4, 0.0]

            for u in safeList:

                # calculate total cost of trajectory and minimize
                Cost_k = costSum(u, target, N)

                #print("Cost: " + str(Cost_k) + " for " + str(u[:][M]))
                if (Cost_k < Cost_min):
                    Cost_min = Cost_k
                    u_opt = u[:][M]

        '''        # MATLAB
                coststack = np.append(u[:][M], float(Cost_k))
                endpoints = np.vstack((endpoints, coststack))

            # MATLAB
            trajfile = open("trajectories.txt", 'a')
            for i in range(0,len(endpoints)):
                #print(endpoints[i].tolist())
                trajfile.write(" ".join(map(repr, endpoints[i].tolist())))
                trajfile.write("\n")
            trajfile.close()
        '''

    ret = list(u_opt)
    ret.append(collision)
    
    # MATLAB
    '''optfile = open("uoptimal.txt", 'a')
    optfile.write(" ".join(map(repr, ret)))
    optfile.write("\n")
    optfile.close()
    '''
    #for i in range(0,6):
    #    ret.append(pPredict[M-1,i].tolist())        

    #print(time.time() - start)

    return ret

