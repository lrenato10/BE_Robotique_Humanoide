# Create a 7DOF manipulator robot and moves it along a constant (random) velocity during 10secs.
import numpy as np
from numpy.linalg import pinv,norm
from pinocchio import neutral
#from robot_arm import Robot
from legged_robot import Robot
from inverse_kinematics import InverseKinematics, CallbackLogger
import time

# Create a 7DOF robot.
robot = Robot()

# Hide the floor.
robot.viewer.viewer.gui.setVisibility('world/floor','OFF')

IK = InverseKinematics(robot)

# Move the robot during 10secs at velocity v.
dt = 1e-4

IK.waistRefPose.translation = np.array([0., 0., 2.0])

#iterations
it=100

#steps number (walk)
steps=1

# Step Height
h=0.2
# Step distance
d=1
# Floor height
h_floor=0.3

#speed
vwx=-(d/2)/(it*dt)
vwy=(d/2)/(it*dt)
vflx=-d/(it*dt)
vfrx=-d/(it*dt)
vflz=-d/(it*dt)
vfrz=-d/(it*dt)

#positions
pflx=0
pfly=0.5
pflz=0.3
pfrx=0
pfry=-0.5
pfrz=0.3
pwx=0
pwy=0
pwz=2.0

#attention with time domain
def spline(x, x0, h, d, h_floor):
    #z = -4*h/d**2*(x-x0)**2+4*h/d*(x-x0) + h_floor
    #if (x-x0) >= d:
    #    return h_floor
    z=h_floor
    return z



#Initial condition
for i in range(it):
    # Feet reference
    IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
    IK.rightFootRefPose.rotation = np.identity(3)
    # Waist reference
    pwy += vwy*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)

for k in range(steps):
    #movement of left Foot
    foot_ref = pflx
    print(foot_ref)
    for i in range(it):
        # Feet reference
        pflx += vflx*dt
        pflz = spline(pflx, foot_ref, h, d, h_floor)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        pwx += vwx*dt
        pwy -= 2*vwy*dt
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)

    #movement of right Foot
    for j in range(it):
        # Feet reference
        pfrx += vfrx*dt
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        pwx += vwx*dt
        pwy += 2*vwy*dt
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)

# Last step
for i in range(it):
        # Feet reference
        pflx += vflx*dt
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        pwx += vwx*dt
        pwy -= 2*vwy*dt
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)

#Align feet
for i in range(it):
    # Feet reference
    pfrx += vfrx*dt
    IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
    IK.rightFootRefPose.rotation = np.identity(3)
    # Waist reference
    pwx += vwx*dt
    pwy += 2*vwy*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)

#Return waist
for i in range(it):
    # Feet reference
    IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
    IK.rightFootRefPose.rotation = np.identity(3)
    # Waist reference
    pwy -= vwy*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)
# identity in rotation matrix give the same rotation of fix repair

