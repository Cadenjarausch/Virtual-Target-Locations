import math as m

#This approach utilizes the bicycle model, a way of predicting vehicular motion in small time intervals by simplifying motion to linear and anglular velocities

#iMaxVel - Maximum Velocity of the Vehicle [m/s]
#maxYawAng - Maximum Yaw Angle of the Vehicle [rad/s]
#XPos - X Position of the Vehicle (Measured from Center) [m]
#YPos - Y Position of the Vehicle (Measured from Center) [m]
#dT - Timestep Defined by the User [s]

def getTrajectory(MaxVel, maxYawAng, XPos, YPos, dT):
    #Initialize the yaw angle [rad/s]
    initAng = 0
    
    #Calculate New Heading Angle
    headAng = initAng + (maxYawAng * dT)
    
    #Calculate Linear Displacement of Interceptor
    dXi = MaxVel * m.cos(headAng) * dT
    dYi = MaxVel * m.sin(headAng) * dT
    
    #Update Postion
    XPos += dXi
    upperYPos = YPos + dYi
    lowerYPos = YPos - dYi
    print("\nUpper and Lower Trajectory Bounds:")
    print("Y Lower: ", lowerYPos)
    print("Y Upper: ", upperYPos)
    
    return(headAng, XPos, upperYPos, lowerYPos)

def generateVT(YPosUpper, YPosLower, XPos, numTargets):
    totalHeight = YPosUpper - YPosLower
    dY = totalHeight / numTargets
    
    targetYLocs = [0] * numTargets
    targetYLocs[0] = YPosLower + (0.5 * dY)
    
    for i in range (numTargets - 1):
        targetYLocs[i + 1] = targetYLocs[i] + dY
    
    return(targetYLocs)

def getDistance(targetXLoc, targetYLocs, XPos, YPos):
    targetDists = [0] * len(targetYLocs)
    
    for x in range (len(targetDists)):
        targetDists[x] = m.sqrt((targetXLoc-XPos)**2 + (targetYLocs[x] - YPos)**2)
        
    return(targetDists)

