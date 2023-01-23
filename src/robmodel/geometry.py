'''
The geometric part of a robot model, as relative poses between robot frames.
'''

import logging

from kgprim.core  import Pose
from kgprim import motions
from robmodel import ordering
from robmodel import frames

logger = logging.getLogger(__name__)

class Geometry:

    '''
    A model of the geometry of a multi-rigid-body mechanism.

    The geometry model consists of a set of constant relative poses between
    reference frames attached to the mechanism. The `reference` of any pose must
    always be a link frame; the `target` is a joint frame or an arbitrary user
    frame.

    This class relies on `kgprim.motions.PosesSpec`, and purely performs various
    consistency checks between the given models.

    A succesfully constructed instance always includes the connectivity model
    with ordering, the frames model, and the poses which are the actual
    geometric data.
    '''

    def __init__(self, connectModel, framesModel, posesModel, jointAxes=None):
        '''
        Construct the geometry model of a mechanism by composing the arguments.

        Parameters:

        - `connectModel` the connectivity model of the mechanism, with ordering
        - `framesModel` the set of the Cartesian frames attached to the mechanism
        - `posesModel` the constant, relative poses between the frames
        - `jointAxes` the versors of the joint axes, in joint frame coordinates

        The third argument is the actual geometric data; this constructor makes
        sure that the three arguments are consistent and therefore can be
        composed as a whole.

        In fact, the poses model stored in this instance is not the given
        `posesModel`. The input poses are always replaced by poses pointing to
        objects of `framesModel`. This is because the frames in the given
        `posesModel` might be simple placeholders. The name of the frame is used
        to establish the correspondence between a placeholder frame and a
        robot-attached frame.

        The `jointAxes` argument defaults to `None`. In that case it is assumed
        that the axis of any joint is the Z axis of its frame. Otherwise, the
        argument should be a dictionary indexed by joint name, with values being
        3-value tuples. Any tuple must represent the 3D joint axis versor, using
        coordinates in the joint frame.
        '''

        if not isinstance(connectModel, ordering.Robot) :
            raise RuntimeError("An ordered robot model is required")
        rname = connectModel.name
        if framesModel.robot.name != rname :
            raise RuntimeError("Robot name mismatch in the connectivity and frames model ('{0}' vs '{1})".format(rname, framesModel.robot.name))

        self.byPose = {}
        for poseSpec in posesModel.poses :
            ignore = False
            pose = poseSpec.pose

            warnmsg = '''Frame '{0}' not found on the given frames-model '{1}', ignoring'''
            if pose.target.name not in framesModel.framesByName :
                logger.warning(warnmsg.format(pose.target.name, rname))
                ignore = True
            if pose.reference.name not in framesModel.framesByName :
                logger.warning(warnmsg.format(pose.reference.name, rname))
                ignore = True

            if not ignore:
                # Retrieve by name the frames actually "attached" to the robot
                # links (because the given poses (in `posesModel`) might be
                # referring to placeholder frames)
                tgtF = framesModel.framesByName[ pose.target.name ]
                refF = framesModel.framesByName[ pose.reference.name ]

                if framesModel.frameRole(refF) is not frames.FrameRole.linkRef:
                    logger.warning( ("Ignoring pose ({}) whose reference frame "
                                     "is not a link frame")
                                    .format(pose))
                elif tgtF.body != refF.body :
                    logger.warning( ("Ignoring pose ({}) whose frames are not "
                                     "attached to the same link").format(pose) )
                else:
                    # Rebuild the Pose instance with the frames from the frames
                    # model, which are attached to links
                    realpose = Pose(target=tgtF, reference=refF)
                    realposespec = motions.PoseSpec(pose=realpose, motion=poseSpec.motion)
                    self.byPose[ realpose ] = realposespec
                    #print( realpose )
                    #print( poseSpec.motion.steps, "\n\n")

        # We iterate over joints and fetch the predecessor, as the joint_wrt_predecessor
        # is a constant pose also for loop joints.
        self.byJoint = {}
        for joint in connectModel.joints.values() :
            predFrame = framesModel.linkFrames [ connectModel.predecessor(joint) ]
            jointFrame= framesModel.jointFrames[ joint ]
            pose = Pose(target=jointFrame, reference=predFrame)
            if pose not in self.byPose :
                logger.warning("The geometry model does not seem to have information about the pose of '{0}' wrt '{1}'".format(
                    jointFrame.name, predFrame.name))
            else :
                self.byJoint[joint] = self.byPose[pose]

        self.ordering = connectModel
        self.frames   = framesModel
        self.poses    = motions.PosesSpec(posesModel.name, list(self.byPose.values()))
        # The last line creates a new PosesSpec model, to make sure we store the
        # model whose poses reference robot attached frames. As said before, the
        # PosesSpec model given to the constructor might have poses that refer
        # to the un-attached frames.

        if jointAxes == None :
            jointAxes = {}
            for joint in connectModel.joints.values() :
                jointAxes[joint.name] = (0.0,0.0,1.0) # default is Z axis
        else :
            if jointAxes.keys() != connectModel.joints.keys():
                logger.warning("The names in the joint-axes dictionary do not " +
                               "match the names in the connectivity model")
        self.axes = jointAxes

    @property
    def robotName(self):
        return self.ordering.name
    @property
    def connectivityModel(self):
        return self.ordering
    @property
    def framesModel(self):
        return self.frames
    @property
    def posesModel(self):
        return self.poses
    @property
    def jointAxes(self):
        return self.axes

