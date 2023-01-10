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
h=0.1
#h=0.6
# Step distance
d=0.3
#d=0.75
# Waist distance
dw=2

# Floor height
h_floor=0.3

#speed
vwx=-(d/2)/(it*dt)
vwy=(0.5/2)/(it*dt)
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

# identity in rotation matrix give the same rotation of fix repair


#attention with time domain
def spline(x, x0, h, d, h_floor):
    z = -4*h/d**2*(-x+x0)**2+4*h/d*(-x+x0) + h_floor
    print(f'x={x}, x0={x0}, h={h}, d={d}, h_floor={h_floor}, z={z}')
    if z <= h_floor:
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
    pwy -= dw*vwy*dt
    pwx += 0.4*vwx*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)

#Movement of left Foot in d
foot_ref_l = pflx
for i in range(it):
    # Feet reference
    pflx += vflx*dt
    pflz = spline(pflx, foot_ref_l, h, d, h_floor)
    IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
    IK.rightFootRefPose.rotation = np.identity(3)
    # Waist reference
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)
for i in range(2*it):
    # Feet reference
    pflz = spline(pflx, foot_ref_l, h, d, h_floor)
    IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
    IK.rightFootRefPose.rotation = np.identity(3)
    # Waist reference
    pwx += vwx*dt
    pwy += dw*vwy*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)


#Move right and left feet in 2d
for k in range(steps):
    foot_ref_r = pfrx
    #movement of right Foot
    for j in range(it*2):
        # Feet reference
        pfrx += vfrx*dt
        pfrz = spline(pfrx, foot_ref_r, h, d*2, h_floor)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)
    for j in range(it*2):
        # Feet reference
        pfrz = spline(pfrx, foot_ref_r, h, d*2, h_floor)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        pwx += vwx*dt
        pwy -= dw*vwy*dt
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)
    
    foot_ref_l = pflx
    #movement of left Foot
    for i in range(it*2):
        # Feet reference
        pflx += vflx*dt
        pflz = spline(pflx, foot_ref_l, h, d*2, h_floor)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)
    for i in range(it*2):
        # Feet reference
        pflz = spline(pflx, foot_ref_l, h, d*2, h_floor)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        pwx += vwx*dt
        pwy += dw*vwy*dt
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)


#Align feet (Movement of right Foot in d)
foot_ref_r = pfrx
for i in range(it):
        # Feet reference
        pfrx += vfrx*dt
        pfrz = spline(pfrx, foot_ref_r, h, d, h_floor)
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
        IK.q = IK.solve(IK.q)
        robot.display(IK.q)
        time.sleep(dt)
for i in range(2*it):
        # Feet reference
        IK.leftFootRefPose.translation = np.array([pflx, pfly, pflz])
        IK.leftFootRefPose.rotation = np.identity(3)
        IK.rightFootRefPose.translation = np.array([pfrx, pfry, pfrz])
        IK.rightFootRefPose.rotation = np.identity(3)
        # Waist reference
        #pwx += vwx*dt
        pwy -= dw*vwy*dt
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
    pwy += dw*vwy*dt
    IK.waistRefPose.translation = np.array([pwx, pwy, pwz])
    
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)
