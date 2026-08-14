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
    reference frames attached to the mechanism.

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

        The reference and target of any given pose must be frames attached to
        the same link of the mechanism, so that the pose itself is a constant.
        The reference of pose P need not be the default link frame F, as long
        as the pose of P's target relative to F can be inferred by the other
        relations (for example, the input data may be F1 wrt F, F2 wrt F1,
        F3 wrt F2; but F, F1, F2, F3 must all be attached to the same link).

        In fact, the frames in the given `posesModel` might be simple placeholders.
        The frame name is used to fetch the intended robot-attached frame from
        the given `framesModel`. The given poses are not stored in this instance,
        which instead stores poses referring to attached frames.

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

            warnmsg = '''Frame '%s' not found on the given frames-model '%s', ignoring'''
            if pose.target.name not in framesModel.framesByName :
                logger.warning(warnmsg, pose.target.name, rname)
                ignore = True
            if pose.reference.name not in framesModel.framesByName :
                logger.warning(warnmsg, pose.reference.name, rname)
                ignore = True

            if not ignore:
                # Retrieve by name the frames actually "attached" to the robot
                # links (because the given poses (in `posesModel`) might be
                # referring to placeholder frames)
                tgtF = framesModel.framesByName[ pose.target.name ]
                refF = framesModel.framesByName[ pose.reference.name ]

                if tgtF.body != refF.body :
                    logger.warning( "Ignoring pose '%s' whose frames are not "
                                    "attached to the same link", pose )
                else:
                    # Rebuild the Pose instance with the frames from the frames
                    # model, which are attached to links
                    realpose = Pose(target=tgtF, reference=refF)
                    realposespec = motions.PoseSpec(pose=realpose, motion=poseSpec.motion)
                    self.byPose[ realpose ] = realposespec
                    #print( realpose )
                    #print( poseSpec.motion.steps, "\n\n")

        # We iterate over joints and fetch the predecessor, as the joint_wrt_predecessor
        # is a constant pose (also for loop joints).
        inspector = motions.ConnectedFramesInspector(motions.PosesSpec("temporary", list(self.byPose.values())))
        self.byJoint = {}
        for joint in connectModel.joints.values() :
            predFrame = framesModel.linkFrames [ connectModel.predecessor(joint) ]
            jointFrame= framesModel.jointFrames[ joint ]
            pose = Pose(target=jointFrame, reference=predFrame)
            if pose not in self.byPose :
                logger.info("No explicit data for the pose of joint frame '%s' wrt link frame '%s'",
                    jointFrame.name, predFrame.name)
                inferred = inspector.getPoseSpec(targetFrame=jointFrame, referenceFrame=predFrame)
                if inferred is not None:
                    self.byJoint[joint] = inferred
                    self.byPose[inferred.pose] = inferred
                    # Note that this will make a loop in the frames graph, because we are storing an
                    # explicit pose (i.e. an edge) between two frames that are connected already
                    # (otherwise the inspector would have not found the pose)
                else:
                   logger.error("Could not determine the pose of joint frame '%s' wrt link frame '%s'",
                        jointFrame.name, predFrame.name)
            else :
                self.byJoint[joint] = self.byPose[pose]

        self.ordering = connectModel
        self.frames   = framesModel
        self.poses    = motions.PosesSpec(posesModel.name, list(self.byPose.values()))
        # The last line creates a new PosesSpec model, to make sure we store the
        # model whose poses reference robot attached frames. As said before, the
        # PosesSpec model given to the constructor might have poses that refer
        # to the un-attached frames.

        self.framesPathFinder =motions.ConnectedFramesInspector(self.poses)

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

    def getPoseSpec(self, target, reference=None):
        '''
        Retrive the pose specification of the given frame pair

        Parameters:
        - `target`: the robot-attached frame (`kgprim.core.Attachment`) whose
          pose is required.
        - `reference`: the robot-attached frame which is the reference
          for the desired pose. If None, the reference defaults to the
          frame of the link to which `target` is attached to.
        Both parameters must be attached to the same link, which must be
        a link of the robot this geometry model refers to.

        Returns: the `kgprim.motions.PoseSpec` describing the pose of `target`
        relative to `reference`, according to the geometrical data in this
        instance. None, if the information cannot be retrieved.

        For example, if `target` is the elbow frame attached to the forearm
        link of a humanoid robot, and `reference` is None, this function
        returns the pose of the elbow frame relative to the default
        forearm frame.

        Note that the return value is always a constant pose, a piece of the
        geometrical information of the robot model.
        '''
        ref = reference or self.frames.byLink.get(target.body)
        if ref is None:
            logger.error("could not determine the reference for the relative pose of '%s'", target.name)
        return self.framesPathFinder.getPoseSpec(target, ref)

## @deprecated("Use the geometry model member function")
def getPoseSpec(geometryModel, frame):
    '''
    Retrive the pose specification of the given frame, from the given geometry
    model.

    Parameters:
      - `geometryModel`: and instance of `Geometry`
      - `frame`: a `kgprim.core.Attachment` instance, whose `entity` field
        must be a `kgprim.core.Frame`. The `body` field should be a link of the
        same robot as the one that the geometry model refers to.

    Returns: the `kgprim.motions.PoseSpec` describing the pose of the given
    frame, relative to the default frame of the link it is attached to.
    None, if the information cannot be retrieved.

    So for example, if the given `frame` is the elbow frame attached to the
    forearm link of a humanoid robot, this function returns the pose of the
    elbow frame relative to the forearm frame.
    '''
    pose = geometryModel.framesModel.poseRelativeToSupportingLinkFrame(frame)
    if pose is not None :
        pose = geometryModel.framesPathFinder.getPoseSpec(pose.target, pose.reference)
    return pose
