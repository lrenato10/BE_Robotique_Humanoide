import numpy as np
import numpy.linalg
from pinocchio import forwardKinematics, log, neutral
import eigenpy
#eigenpy.switchToNumpyMatrix()
from scipy.optimize import fmin_bfgs, fmin_slsqp

def nothing(x):
    return x

class CallbackLogger:
     def __init__(self, ik):
          self.nfeval = 1
          self.ik = ik
     def __call__(self,x):
          print('===CBK=== {0:4d}   {1}'.format(self.nfeval,
                                                self.ik.latestCost))
          self.nfeval += 1

class InverseKinematics (object):
    leftFootJoint = 'left_leg_6_joint'
    rightFootJoint = 'right_leg_6_joint'
    waistJoint = 'waist_joint'

    def __init__ (self, robot):
        self.robot = robot
        self.q = neutral (self.robot.model)
        self.q[6] = -np.pi/8
        self.q[12] = -np.pi/8
        #print(self.q)
        #p = np.array (robot.model.nv * [0])
        #p = p + 2
        forwardKinematics (self.robot.model, self.robot.data, self.q)
        # Initialize references of feet and center of mass with initial values
        #self.leftFootRefPose = robot.data.oMi [robot.leftFootJointId].copy ()
        self.leftFootRefPose = self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']].copy ()
        #print(self.leftFootRefPose.translation)
        #print(type(self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']]))
        #print(dir(self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']]))
        #self.rightFootRefPose = robot.data.oMi [robot.rightFootJointId].copy ()
        self.rightFootRefPose = self.robot.data.oMi [self.robot.jointId['right_ankle_joint_y']].copy ()
        #print(self.rightFootRefPose.translation)
        #self.waistRefPose = robot.data.oMi [robot.waistJointId].copy ()
        self.waistRefPose = self.robot.data.oMi [self.robot.jointId['bowl_joint_z']].copy ()
        #print(self.waistRefPose.translation)

    def current_position(self, q):
        forwardKinematics (self.robot.model, self.robot.data, q)
        # Current position
        self.leftFootPose = self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']].copy ()
        #print(type(self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']]))
        #print(dir(self.robot.data.oMi [self.robot.jointId['left_ankle_joint_y']]))
        self.rightFootPose = self.robot.data.oMi [self.robot.jointId['right_ankle_joint_y']].copy ()
        self.waistPose = self.robot.data.oMi [self.robot.jointId['bowl_joint_z']].copy ()

    def cost (self, q):
        # Write your code here
        # Difference between ref and current position of feet and waist
        self.current_position(q)
        #print(self.leftFootPose.translation)
        # Only translation
        #return (1/2*np.linalg.norm(self.leftFootRefPose.translation-self.leftFootPose.translation)**2 + 1/2*np.linalg.norm(self.rightFootRefPose.translation-self.rightFootPose.translation)**2 + 1/2*np.linalg.norm(self.waistRefPose.translation-self.waistPose.translation)**2) 
        # Translation + Rotation
        # points to compare rotation 
        px=np.array([0.1, 0, 0]) # x axis
        py=np.array([0, 0.1, 0]) # y axis
        return (1/2*np.linalg.norm(self.leftFootRefPose.translation-self.leftFootPose.translation)**2 + 1/2*np.linalg.norm(self.rightFootRefPose.translation-self.rightFootPose.translation)**2 + 1/2*np.linalg.norm(self.waistRefPose.translation-self.waistPose.translation)**2 + 1/2*np.linalg.norm(self.rightFootRefPose.act(px)-self.rightFootPose.act(px))**2 + 1/2*np.linalg.norm(self.leftFootRefPose.act(px)-self.leftFootPose.act(px))**2 + 1/2*np.linalg.norm(self.rightFootRefPose.act(py)-self.rightFootPose.act(py))**2 + 1/2*np.linalg.norm(self.leftFootRefPose.act(py)-self.leftFootPose.act(py))**2) 


    def solve (self, q):
        # Minimize cost function
        # Optimize cost without any constraints in BFGS, with traces.
        #xopt_bfgs = fmin_bfgs(self.cost, q, callback=print)
        xopt_bfgs = fmin_bfgs(self.cost, q, callback=nothing)
        #print('\n *** Xopt in BFGS = ',xopt_bfgs,'\n\n\n\n')
        return xopt_bfgs