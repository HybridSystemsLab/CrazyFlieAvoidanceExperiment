
import numpy as np
import sys

def costObjSum(l, p, objLow, objHigh, N):
    totalCost = 0
    for i in range(0, N):
        totalCost = totalCost + cost(l[i][:], p.tolist()) - min(cost(l[i][:], objLow[i][:]), cost(l[i][:], objLow[i][:]))

    return totalCos 

# calculates the sum of costs across N points on the trajectory
def costSum(l, p, N):
    totalCost = 0

    for i in range(0, N):
        totalCost = totalCost + cost(l[i][:], p.tolist())

    return totalCost


# cost function - distance from point to target point
def cost(xi, p):

    cost = np.linalg.norm(np.subtract(xi, p))
    return cost