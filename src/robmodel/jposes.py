'''
The joint poses between joint frames and successor frames in a robot model
'''

from enum import Enum
import math

from kgprim.core import Pose
import kgprim.motions as motions
from kgprim.motions import PoseSpec
import kgprim.ct as ct
import kgprim.ct.models
import kgprim.ct.frommotions

import kgprim.values as numeric_argument

from robmodel.connectivity import JointKind as JointKind

class JointTransformPolarity(Enum):
    '''
    A binary enumeration, to distinguish the two transforms associated with a
    joint pose.
    '''

    joint_X_successor=0
    successor_X_joint=1


class JointPoses:

    '''
    A container of the relative poses associated with the joints of a robot.

    A joint pose is a relation between the joint-successor frame (i.e. a link
    frame) and the joint frame itself.

    This class constructs and stores relative poses (`kgprim.core.Pose`), but
    can also provide the associated coordinate transforms models
    (`kgprim.ct.models.CoordinateTransform`).

    The structure of a joint
    transform depends only on the joint kind, its value depends only on the
    current joint status value. Therefore, any joint transform depends on a
    symbolic variable representing the joint status.

    The argument of a joint pose (and therefore of any derived transform) is in
    fact an instance of a plain `numeric_argument.Expression` whose argument is
    a `numeric_argument.Variable`. The variables are conventionally called
    `q<i>`, with `<i>` ranging from 0 to the number of degrees of freedom minus
    one.
    '''

    def __init__(self, robot, frames, axes):
        self.poseSpecByJoint = {}
        self.poseSpecByPose  = {}
        self.jointToSymVar = {}
        self.symVarToJoint = {}

        for joint in robot.joints.values() :
            i = robot.jointNum(joint)
            succ = robot.successor(joint)
            succFrame = frames.linkFrames [ succ ]
            jointFrame= frames.jointFrames[ joint ]
            pose      = Pose(target=succFrame, reference=jointFrame)
            axis      = axes[joint.name]
            # So, we have the pose of the link (successor) frame relative to the joint frame

            symname = "q{0}".format(i-1)
            qvar = numeric_argument.Variable(name=symname)
            qexpr= numeric_argument.Expression(argument=qvar)
            motionSteps = None
            motionF    = None
            if joint.kind == 'fixed' :
                # aribtrary motion step of 0 value
                motionSteps = [motions.translation(motions.Axis.Z, 0.0)]
            else :
                if joint.kind == JointKind.prismatic :
                    motionF = lambda axis,amount : motions.translation(axis, amount)
                elif joint.kind == JointKind.revolute :
                    motionF = lambda axis,amount : motions.rotation(axis, amount)
                else:
                    raise RuntimeError("Unknown joint type '{0}'".format(joint.kind))
                if axis == (1,0,0) :
                    motionSteps = [motionF( motions.Axis.X, qexpr )]
                elif axis == (-1,0,0) :
                    motionSteps = [motionF( motions.Axis.X, -qexpr )]
                elif axis == (0,1,0) :
                    motionSteps = [motionF( motions.Axis.Y, qexpr )]
                elif axis == (0,-1,0) :
                    motionSteps = [motionF( motions.Axis.Y, -qexpr )]
                elif axis == (0,0,1) :
                    motionSteps = [motionF( motions.Axis.Z, qexpr )]
                elif axis == (0,0,-1) :
                    motionSteps = [motionF( motions.Axis.Z, -qexpr )]
                else:
                    jointMotion = motionF( motions.Axis.Z, qexpr )
                    # The joint axis is an arbitrary versor...
                    # Thus, we first rotate the frame so that our Z axis aligns
                    # with the joint axis, and only then we add the motion of
                    # the joint itself.
                    x,y,z = axis[:]
                    if y != 0.0 :
                        rz = - math.tan(x/y)
                        rx = - (math.pi/2 - math.tan( z / math.sqrt(x*x + y*y) ) )

                        step1 = motions.rotation(motions.Axis.Z, rz)
                        step2 = motions.rotation(motions.Axis.X, rx)
                        motionSteps = [step1, step2, jointMotion, -step2, -step1 ]
                    else :
                        ry = math.tan(x/z) # if z was also 0, another branch above would be executed
                        step1 = motions.rotation(motions.Axis.Y, ry)
                        motionSteps = [ step1, jointMotion, -step1 ]

            poseSpec = PoseSpec(pose=pose, motion=motions.MotionSequence(steps=motionSteps, mode=motions.MotionSequence.Mode.currentFrame))
            self.poseSpecByJoint[joint] = poseSpec
            self.poseSpecByPose [pose ] = poseSpec
            self.jointToSymVar[joint] = qvar
            self.symVarToJoint[qvar] = joint

        self.robot = robot
        self.robotFrames = frames
        self.jointPosesModel = motions.PosesSpec(name=robot.name+'-joints', poses=self.poseSpecByJoint.values())

    __toTransformPolarity = {
        JointTransformPolarity.joint_X_successor : ct.models.TransformPolarity.movedFrameOnTheRight,
        JointTransformPolarity.successor_X_joint : ct.models.TransformPolarity.movedFrameOnTheLeft
    }


    def jointTransform(self, joint, polarity=JointTransformPolarity.joint_X_successor):
        poseSpec = self.poseSpecByJoint[joint]
        mode     = JointPoses.__toTransformPolarity[ polarity ]

        return ct.frommotions.toCoordinateTransform(poseSpec=poseSpec, polarity=mode)




