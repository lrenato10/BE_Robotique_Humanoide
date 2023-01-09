from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import forwardKinematics, Inertia, JointModelRX, Model, SE3
import gepetto.corbaserver
from display import Display
import eigenpy
eigenpy.switchToNumpyArray()

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint 
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3). 
    The configuration is nq=7. The velocity is the same. 
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).
    
    See tp1.py for an example of use.
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = Model ()
        self.createArm3DOF()
        self.data = self.model.createData()
        self.q0 = zero(self.model.nq)

    def createArm3DOF(self,rootId=0, prefix='', jointPlacement=SE3.Identity()):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        jointName          = prefix + "shoulder_joint"
        joint              = JointModelRX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'shoulder', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'shoulder',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'upperarm', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'upperarm',jointId,SE3(eye(3),np.array([0., 0., .5]))))

        jointName          = prefix + "elbow_joint"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 1.0] ))
        joint              = JointModelRX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'elbow', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'elbow',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'lowerarm', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'lowerarm',jointId,SE3(eye(3),np.array([0., 0., .5]))))

        jointName          = prefix + "wrist_joint"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 1.0] ))
        joint              = JointModelRX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'wrist', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'wrist',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'hand', .1,.1,.25,color)
        self.visuals.append( Visual('world/' + prefix + 'hand',jointId,SE3(eye(3),np.array([0., 0., .25]))))


    def display(self,q):
        forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()


