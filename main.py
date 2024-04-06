import numpy as np
import math as m
import matplotlib.pyplot as plt
from testing import getTrajectory
from testing import generateVT
from testing import getDistance
from simulation import simulate

numTargets = 5
x0 = 0 #m
y0 = 0 #m
maxVel = 1 #m/s
maxYaw = (m.pi)/4 #Rad/s
dT = 100 #Second
totalT = 1
    
iHeadAng, VT_XLoc, UpperYTraj, LowerYTraj = getTrajectory(maxVel, maxYaw, x0, y0, dT)

VT_YLocs = generateVT(UpperYTraj, LowerYTraj, VT_XLoc, numTargets)
targetDists = getDistance(VT_XLoc, VT_YLocs, x0, y0) #Euclidian Distance
simulate(maxVel, maxYaw, dT)

print("\nVirtual Target Locations:")
for x in VT_YLocs:
    print(x)

print("\nVirtual Target Distances:")
for x in targetDists:
    print(x)