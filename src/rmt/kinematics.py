import logging
import numpy

import kgprim.motions as motions
import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr
import motiondsl.motiondsl as motdsl

logger = logging.getLogger(__name__)

class RobotKinematics:
    '''The composition of the constant poses and the joint poses of a robot.

    This class is a simple aggregation of the geometry model and the joint-poses
    model. By merging the two, this class have access to the full robot
    kinematics.

    Thanks to `kgprim.motions.ConnectedFramesInspector`, an arbitrary relative pose
    between two frames on the robot can be obtained.
    '''
    def __init__(self, geometry, jointPoses):
        self.robotGeometry = geometry
        self.jointPoses = jointPoses
        self.baseFrame = geometry.framesModel.linkFrames[ geometry.connectivityModel.base ]

        allPoses = geometry.posesModel.mergeModel( jointPoses.jointPosesModel )

        self.framesConnectivity = motions.ConnectedFramesInspector(allPoses)



def base_H_ee(kinematics, framename):
    if framename not in kinematics.robotGeometry.framesModel.framesByName:
        logger.error("Could not find frame '{0}' in model '{1}'".format(framename, kinematics.robotGeometry.robotName))
        return None

    ee = kinematics.robotGeometry.framesModel.framesByName[ framename ]
    if not kinematics.framesConnectivity.hasRelativePose(ee, kinematics.baseFrame):
        logger.error("Frame '{0}' and the base frame do not seem to be connected".format(framename))
        return None

    poseSpec = kinematics.framesConnectivity.getPoseSpec(ee, kinematics.baseFrame)
    cotr = frommotions.toCoordinateTransform(poseSpec)
    H = mxrepr.hCoordinatesSymbolic(cotr)
    q = numpy.zeros( len(H.variables) )
    H = H.setVariablesValue( valueslist=q )
    return H


def serializeToMotionDSLModel(robotKinematics, ostream):
    header ='''
Model {modelname}

Convention = currentFrame

'''.format(modelname=robotKinematics.robotGeometry.robotName)

    ostream.write(header)
    for jp in robotKinematics.jointPoses.poseSpecByJoint.values():
        text = motdsl.poseSpecToMotionDSLSnippet( jp )
        ostream.write(text)
        ostream.write('\n')
    for cp in robotKinematics.robotGeometry.byPose.values() :
        text = motdsl.poseSpecToMotionDSLSnippet( cp )
        ostream.write(text)
        ostream.write('\n')

