# Create a 7DOF manipulator robot and moves it along a constant (random) velocity during 10secs.
import numpy as np
from numpy.linalg import pinv,norm
from pinocchio import neutral
#from robot_arm import Robot
from legged_robot import Robot
import time

# Create a 7DOF robot.
robot = Robot()

# Hide the floor.
robot.viewer.viewer.gui.setVisibility('world/floor','OFF')

# Move the robot during 10secs at velocity v.
dt = 1e-3
for j in range (robot.model.nv):
    v = np.array (robot.model.nv * [0])
    v [j] = 1
    q = neutral (robot.model)
    for i in range(1000):
        q += v*dt
        robot.display(q)
        time.sleep(dt)
    for i in range(1000):
        q -= v*dt
        robot.display(q)
        time.sleep(dt)
