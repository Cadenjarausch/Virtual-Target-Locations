import math as m
from virtualTargets import getTrajectory
from virtualTargets import generateVT
from virtualTargets import getDistance

numTargets = 5
x0 = 0 #m
y0 = 0 #m
maxVel = 1 #m/s
maxYaw = (m.pi)*0.125 #Rad/s
dT = 1 #Second
    
iHeadAng, VT_XLoc, UpperYTraj, LowerYTraj = getTrajectory(maxVel, maxYaw, x0, y0, dT)

VT_YLocs = generateVT(UpperYTraj, LowerYTraj, VT_XLoc, numTargets)
targetDists = getDistance(VT_XLoc, VT_YLocs, x0, y0) #Euclidian Distance

print("\nVirtual Target Locations:")
for x in VT_YLocs:
    print(x)

print("\nVirtual Target Distances:")
for x in targetDists:
    print(x)