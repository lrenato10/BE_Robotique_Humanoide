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
#for j in range (robot.model.nv):
    #p = np.array (robot.model.nv * [0])
    #p = p + 2
    #print(type(p))
    
    #q =  q + p

'''   
q = neutral (robot.model)    

IK.leftFootRefPose.translation = np.array([0.2, 0.5, 0.3])
q = IK.solve(q)
robot.display(q)
time.sleep(dt)
'''

#for j in range (robot.model.nv):
#    v = np.array (robot.model.nv * [0])
#    v [j] = 1
#q = neutral (robot.model)
#print(q)
#q[6] = np.pi/4
#q[13] = np.pi/4
#print(q).0

IK.waistRefPose.translation = np.array([0., 0., 2.0])

#iterations
it=100

#speed
vw=-0.8/(it*dt)
vfl=-1/(it*dt)
vfr=-1/(it*dt)

#positions
pfl=0
pfr=0
pw=0

#movement of left Foot
for i in range(it):
    pfl += vfl*dt
    IK.leftFootRefPose.translation = np.array([pfl, 0.5, 0.3])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.rightFootRefPose.translation = np.array([pfr, -0.5, 0.3])
    IK.rightFootRefPose.rotation = np.identity(3)
    pw += vw*dt
    IK.waistRefPose.translation = np.array([pw, 0., 2.0])
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)

#movement of right Foot
for j in range(it):
    pfr += vfr*dt
    IK.rightFootRefPose.translation = np.array([pfr, -0.5, 0.3])
    IK.leftFootRefPose.rotation = np.identity(3)
    IK.leftFootRefPose.translation = np.array([pfl, 0.5, 0.3])
    IK.rightFootRefPose.rotation = np.identity(3)
    pw += vw*dt
    IK.waistRefPose.translation = np.array([pw, 0., 2.0])
    IK.q = IK.solve(IK.q)
    robot.display(IK.q)
    time.sleep(dt)

# identity in rotation matrix give the same rotation of fix repair

