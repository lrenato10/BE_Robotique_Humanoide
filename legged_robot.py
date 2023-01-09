from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, JointModelRX, \
    JointModelRY, JointModelRZ, JointModelPX, JointModelPY, JointModelPZ, forwardKinematics, neutral
import gepetto.corbaserver
from display import Display
import eigenpy
#eigenpy.switchToNumpyArray()


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
    Define a class Robot representing a biped robot
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D
      objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot,
      each element of the list being an object Visual (see above).
    '''

    jointId = {
        'universe': 0
    }


    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = Model ()
        self.createLeggedRobot ()
        self.data = self.model.createData()
        #self.q0 = neutral (self.model)
        self.q0 = zero(self.model.nq)
        

    def createLeggedRobot (self,rootId=0, prefix='', jointPlacement=SE3.Identity()):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        #jointId = rootId

        # Bowl/Waist
        jointName          = prefix + "bowl_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 2.30] ))
        joint              = JointModelPX()
        self.jointId[jointName] = self.model.addJoint(self.jointId['universe'],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "bowl_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelPY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "bowl_joint_z"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelPZ()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())        
        #self.viewer.viewer.gui.addSphere('world/' + prefix + 'bowl_j', 0.3,colorred)
        #self.visuals.append( Visual('world/' + prefix + 'bowl_j',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'bowl', 0.3, 1, 0.3,color)
        self.visuals.append( Visual('world/' + prefix + 'bowl',self.jointId[jointName],SE3(eye(3),np.array([0., 0., 0]))))

        #Left joints
        prefix="left_"
        
        jointName          = prefix + "hip_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, 0.5, 0] ))
        joint              = JointModelRX()
        self.jointId[jointName] = self.model.addJoint(self.jointId['bowl_joint_z'],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "hip_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "hip_joint_z"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRZ()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())        
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'hip', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'hip',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'upperleg', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'upperleg',self.jointId[jointName],SE3(eye(3),np.array([0., 0., -0.5]))))

        jointName_pre=jointName
        jointName          = prefix + "knee_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -1.0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'knee', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'knee',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'lowerleg', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'lowerleg',self.jointId[jointName],SE3(eye(3),np.array([0., 0., -0.5]))))

        
        jointName          = prefix + "ankle_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -1.0] ))
        joint              = JointModelRX()
        self.jointId[jointName] = self.model.addJoint(self.jointId["left_knee_joint_y"],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "ankle_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'ankle', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'ankle',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'foot', 1, 0.3,0.15,color)
        self.visuals.append( Visual('world/' + prefix + 'foot',self.jointId[jointName],SE3(eye(3),np.array([-0.3, 0., -0.25]))))

        #Right joints
        
        prefix="right_"
        
        jointName          = prefix + "hip_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, -0.5, 0] ))
        joint              = JointModelRX()
        self.jointId[jointName] = self.model.addJoint(self.jointId["bowl_joint_z"],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "hip_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "hip_joint_z"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRZ()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())        
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'hip', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'hip',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'upperleg', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'upperleg',self.jointId[jointName],SE3(eye(3),np.array([0., 0., -0.5]))))

        jointName_pre=jointName
        jointName          = prefix + "knee_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -1.0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'knee', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'knee',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'lowerleg', .1,.1,.5,color)
        self.visuals.append( Visual('world/' + prefix + 'lowerleg',self.jointId[jointName],SE3(eye(3),np.array([0., 0., -0.5]))))

        
        jointName          = prefix + "ankle_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -1.0] ))
        joint              = JointModelRX()
        self.jointId[jointName] = self.model.addJoint(self.jointId["right_knee_joint_y"],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        jointName_pre=jointName
        jointName          = prefix + "ankle_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRY()
        self.jointId[jointName] = self.model.addJoint(self.jointId[jointName_pre],joint,jointPlacement,jointName)
        self.model.appendBodyToJoint(self.jointId[jointName],Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'ankle', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'ankle',self.jointId[jointName],SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'foot', 1, 0.3,0.15,color)
        self.visuals.append( Visual('world/' + prefix + 'foot',self.jointId[jointName],SE3(eye(3),np.array([-0.3, 0., -0.25]))))

    def display(self,q):
        forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()
