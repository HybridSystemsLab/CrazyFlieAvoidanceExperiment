import numpy as np
import math

# NOTE: this library is for 3D vectors

# calculates dot product
def dot(v,w):
    x,y,z = v
    X,Y,Z = w
    return x*X + y*Y + z*Z

# calculates length of vector
def length(v):
    x,y,z = v
    return math.sqrt(x*x + y*y + z*z)
  
# creates vector from 2 points
def vector(b,e):
    x,y,z = b
    X,Y,Z = e
    return (X-x, Y-y, Z-z)
  
# calculates unit vector
def unit(v):
    x,y,z = v
    mag = length(v)
    return (x/mag, y/mag, z/mag)
  
# calculates distance between two points
def distance(p0,p1):
    return length(vector(p0,p1))
  
# scales a vector by a factor sc
def scale(v,sc):
    x,y,z = v
    return (x * sc, y * sc, z * sc)

# calculates the sum of two vectors
def add(v,w):
    x,y,z = v
    X,Y,Z = w
    return (x+X, y+Y, z+Z)

# calculates minimum distance from a point to a line segment
def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return dist

# computes the distance between two line segments A and B, 
# A connects point a0 to point a1, and B connects point b0 to point b1
def line2line(a0,a1,b0,b1):

    # calculate vectors and magnitudes
    A = a1 - a0
    B = b1 - b0

    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    # both lines are just points
    if(magA == 0 and magB == 0):
        return np.linalg.norm(A-B)

    # A is just a point
    if(magA == 0):
        return pnt2line(a1,b0,b1)

    # B is just a point
    if(magB == 0):
        return pnt2line(b0,a0,a1)

    # unit vectors
    _A = A / magA
    _B = B / magB

    # Calculate denomitator
    cross = np.cross(_A, _B);
    denom = np.linalg.norm(cross)**2


    # if lines are parallel (denom=0) test if lines overlap.
    # if they don't overlap then there is a closest point solution.
    # if they do overlap, there are infinite closest positions, but there is a closest distance
    if(denom == 0):

        d0 = np.dot(_A,(b0-a0))
        d1 = np.dot(_A,(b1-a0))

        # is segment B before A?
        if(d0 <= 0 >= d1):
            if(np.absolute(d0) < np.absolute(d1)):
                return np.linalg.norm(a0-b0)
            return np.linalg.norm(a0-b1)


        # is segment B after A?
        elif(d0 >= magA <= d1):
            if(np.absolute(d0) < np.absolute(d1)):
                return np.linalg.norm(a1-b0)
            return np.linalg.norm(a1-b1)


        # segments overlap, return distance between parallel segments
        return np.linalg.norm(((d0*_A)+a0)-b0)

    # lines criss-cross: Calculate the projected closest points
    t = (b0 - a0);
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA/denom;
    t1 = detB/denom;

    pA = a0 + (_A * t0) # projected closest point on segment A
    pB = b0 + (_B * t1) # projected closest point on segment B


    # clamp projections
    if(t0 < 0):
        pA = a0
    elif(t0 > magA):
        pA = a1

    if(t1 < 0):
        pB = b0
    elif(t1 > magB):
        pB = b1

    # clamp projection A
    if((t0 < 0) or (t0 > magA)):
        dot = np.dot(_B,(pA-b0))
        if(dot < 0):
            dot = 0
        elif(dot > magB):
            dot = magB
        pB = b0 + (_B * dot)

    # clamp projection B
    if((t1 < 0) or (t1 > magB)):
        dot = np.dot(_A,(pB-a0))
        if(dot < 0):
            dot = 0
        elif(dot > magA):
            dot = magA
            pA = a0 + (_A * dot)


    return np.linalg.norm(pA-pB)


# pDot: Equations of motion that govern projectile dynamics.
def pDot(t, projectile):
    # C_D: Drag coefficient associated with shape of projectile.
    C_D = 1.28

    # rho: Air density (kg/m^3).
    rho = 1.2041

    # SA: Surface area of projectile (m^2).
    SA = 1.8e-3

    # m: Mass of projectile (kg),
    m = 0.2

    # g: Gravitational constant.
    g = 9.8

    # v_x,v_y,v_z: Projectile velocities in specified direction.
    v_x = projectile[3]
    v_y = projectile[4]
    v_z = projectile[5]

    # R_x,R_y,R_z: Resistance exerted on projectile in specified direction.
    R_x = (C_D * rho * SA * (v_x) ** 2) / 2
    R_y = (C_D * rho * SA * (v_y) ** 2) / 2
    R_z = (C_D * rho * SA * (v_z) ** 2) / 2

    # a_x,a_y,a_z: Projectile accelerations in specificed direction.
    # air resistance depends on the sign of the velocity
    if(v_x < 0):
        a_x = R_x / m
    else:
        a_x = -R_x / m

    if(v_y < 0):
        a_y = R_y / m
    else:
        a_y = -R_y / m

    if(v_z < 0):
        a_z = -g + R_z / m
    else:
        a_z = -g - R_z / m


    # projectileDot: Derivative of projectile state vector.
    projectileDot = [v_x, v_y, v_z, a_x, a_y, a_z]
    return projectileDot


