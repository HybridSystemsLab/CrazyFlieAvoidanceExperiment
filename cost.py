
import numpy as np


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