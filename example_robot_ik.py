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
dt = 1
#for j in range (robot.model.nv):
    #p = np.array (robot.model.nv * [0])
    #p = p + 2
    #print(type(p))
    
    #q =  q + p
    
q = neutral (robot.model)    
q = IK.solve(q)
robot.display(q)
time.sleep(dt)
