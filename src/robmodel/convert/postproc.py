import logging, copy
import numpy as np

import robmodel.connectivity
from robmodel.connectivity import Link
from robmodel.connectivity import JointKind
from robmodel.connectivity import KPair
import robmodel.frames
import robmodel.geometry
import robmodel.inertia
import robmodel.convert.utils as utils

import kgprim.core as primitives # to create instances of Frame
import kgprim.motions as motions
from kgprim.motions import PoseSpec

# coordinate transforms, for the inertia conversions
import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr


logger = logging.getLogger(__name__)

def weld(link1, link2):
    return Link(name=f'{link1.name}+{link2.name}')

def collapseFixedJoints(connectivity, ordering=None, frames=None, geometry=None, inertia=None):
    fixedJoints = [jo for jo in connectivity.joints.values() if jo.kind == JointKind.fixed]

    if len(fixedJoints)==0:
        return connectivity, ordering, frames, geometry, inertia

    pairs = copy.copy(connectivity.pairs)
    partToComposite = {}

    def _resolve(link):
        nonlocal partToComposite
        while link.name in partToComposite:
            link = partToComposite[link.name]
        return link

    for fixj in fixedJoints:
        pair = connectivity.jointToLinkPair(fixj)
        del pairs[fixj]
        link1 = _resolve(pair[0])
        link2 = _resolve(pair[1])
        composite = weld(link1, link2)
        logger.debug("merging link '%s' and '%s'", link1.name, link2.name)
        partToComposite[link1.name] = composite
        partToComposite[link2.name] = composite

    # store a new map now that we have the definitive results
    toNewLink = { link:_resolve(link) for link in connectivity.links.values() }

    # map every new link to the ones that it replaced
    compositeToParts = {}
    for link,newLink in toNewLink.items() :
        if link != newLink:
            compositeToParts.setdefault(newLink, []).append(link)

    newpairs = [ KPair( joint = joint,
                        link1 = toNewLink[pair[0]],
                        link2 = toNewLink[pair[1]] ) for joint,pair in pairs.items() ]
    newconnectivity = robmodel.connectivity.Robot(name=connectivity.name, pairs=newpairs)

    newordering = ordering
    sortedFixedJoints = []  # larger IDs first
    if ordering is not None:
        ids = { toNewLink[ordering.base].name:0 }
        delta = 0
        # The integer id of any joint has to be reduced by the count of
        # fixed joints _with smaller id_ that were dropped.
        # The ID of a link is the same as the ID of the supporting joint.
        # In the numbering scheme, IDs are required only for links and for loop joints.
        for joint in ordering.joints.values(): # iteration is sorted by original ID
            if joint.kind != JointKind.fixed:
                jid =  ordering.jointNum(joint) - delta
                if joint in ordering.loopJoints:
                    ids[joint.name] = jid
                else:
                     link =  toNewLink[ordering.successor(joint)]
                     ids[link.name] = jid
            else:
                delta = delta+1 # keep track of how many fixed joints were dropped so far
                sortedFixedJoints.insert(0, joint)  # insert at the head
                if joint in ordering.loopJoints:
                    logger.error("unsupported operation: collapsing a fixed _and_ loop joint")
                    raise RuntimeError("collapsing fixed joints that are also loop joints is not supported!")

        # Create the new model
        newordering = robmodel.ordering.Robot(newconnectivity, {'robot':ordering.name, 'nums':ids})

        # Sort the lists of dropped links according to the original ID
        # This will be needed later
        for droppedLinks in compositeToParts.values():
            droppedLinks.sort(key=ordering.linkNum)
        # for k,seq in compositeToParts.items():
        #     print(k)
        #     for l in seq:
        #         print("\t",l)

    newframes = frames
    if frames is not None:
        if ordering is None:
            raise RuntimeError("cannot pass a frames model without the ordering model")

        # Rebuild the user frames: same frames, attached to the new bodies
        userFrames = {primitives.Attachment( attachment.entity, toNewLink[attachment.body] )
                                for attachment in frames.userFrames.values() }

        # Preserve the frames of the joints/links that were deleted,
        # as additional user frames attached to the new links
        for joint in sortedFixedJoints:
            oldFrameAttachment = frames.byJoint[joint]
            userFrames.add( primitives.Attachment(
                              entity= oldFrameAttachment.entity,
                              body  = toNewLink[ oldFrameAttachment.body ] ) )
        for droppedLinks in compositeToParts.values():
            for dropped in droppedLinks:
                oldFrameAttachment = frames.byLink[ dropped ]
                userFrames.add( primitives.Attachment(
                                  entity=oldFrameAttachment.entity,
                                  body=toNewLink[dropped] ) )

        newframes = robmodel.frames.RobotDefaultFrames(newordering, userFrames)

    newgeometry = geometry
    if geometry is not None:
        poses = []
        for oldPose in geometry.posesModel.poses:
            ref = oldPose.pose.reference
            tgt = oldPose.pose.target
            ref = newframes.byName[ref.name]
            tgt = newframes.byName[tgt.name]
            poses.append( PoseSpec(
                            pose   = primitives.Pose(target=tgt, reference=ref),
                            motion = oldPose.motion) )
        # Need to add the (identity) pose of the frame of the links that were
        # dropped, relative to the frame of the fixed joints that were dropped.
        # Those are not in the pose model already, because technically they are
        # joint-state dependent poses (so not in the geometry model of a robot).
        # But we know those dropped joints were fixed, so that such relative pose
        # is constant (and equal to the identity)
        for joint in sortedFixedJoints:
            fixedJointFrame = newframes.byName[ frames.byJoint[joint].name ]
            successorFrame  = newframes.byName[ frames.byLink[ ordering.successor(joint) ].name ]
            poses.append( PoseSpec(
                pose = primitives.Pose(target=successorFrame, reference=fixedJointFrame),
                motion = motions.MotionSequence(steps=[]) ) )

        # We still miss the connection between the roots of the subtrees
        # that were dropped, and the new links into which they were mapped.
        # The frames coincide, thus the pose is the identity, nevertheless
        # we must explicitly add it to make sure the graph of frames is
        # fully connected
        for newLink, droppedLinks in compositeToParts.items():
            droppedRoot = droppedLinks[0]
            newLinkFrame = newframes.byLink[newLink]
            droppedLinkFrame = newframes.byName[ frames.byLink[droppedRoot].name ]
            poses.append( PoseSpec(
                pose = primitives.Pose(target=droppedLinkFrame, reference=newLinkFrame),
                motion = motions.MotionSequence(steps=[]) ) )

        jointAxes = {joint:geometry.jointAxes[joint] for joint in newordering.joints}
        posesContainer = motions.PosesSpec( name=geometry.posesModel.name, poses=poses )
        newgeometry = robmodel.geometry.Geometry(newordering, newframes, posesContainer, jointAxes)

    newinertia = inertia
    if inertia is not None:
        if geometry is None:
            raise RuntimeError("cannot lump the inertia without the geometric data of the robot")
        inertialData = inertia.inertia # shallow copy of the dict with the current properties
        for newLink, droppedList in compositeToParts.items():
            linkFrame = newframes.byLink[newLink]
            newmass = 0
            newcom = np.array([0,0,0])
            newmoments = robmodel.inertia.IMoments(frame=linkFrame, ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
            for dropped in droppedList:
                del inertialData[dropped.name]
                props = inertia.byLink(dropped)
                if props and props.mass != 0.0:
                    com = np.array([props.com.x, props.com.y, props.com.z])

                    # Transform the CoM vector in coordinates of the same frame used for the moments
                    if props.com.frame != props.moments.frame:
                        # we must query the old geometry model
                        pose = geometry.getPoseSpec(target=props.com.frame, reference=props.moments.frame)
                        if pose is None:
                            logger.error("failed to obtain the pose of the coordinate frame of the CoM relative to the coordinate frame of the moments of inertia")
                            continue
                        cotr = frommotions.toCoordinateTransform(pose, right_frame=props.com.frame)
                        H = mxrepr.hCoordinatesNumeric(cotr)
                        com = H[0:3,0:3] @ com + H[0:3,3]

                    # Transform the moments of inertia in coordinates of the new link frame
                    momentsFrameInNewModel = newframes.byName[props.moments.frame.name]
                    if momentsFrameInNewModel is None :
                        logger.error("failed to find the inertial moments frame '%s' of link '%s' in the frames model",
                            momentsFrameInNewModel.name, dropped.name)
                        continue
                    pose = newgeometry.getPoseSpec(target=momentsFrameInNewModel, reference=linkFrame)
                    if pose is None:
                        logger.error("failed to find the pose of the inertial moments frame '%s' relative to the link frame '%s'",
                            momentsFrameInNewModel.name, linkFrame.name)
                        continue
                    cotr = frommotions.toCoordinateTransform(pose, right_frame=linkFrame) # current_T_link
                    H = mxrepr.hCoordinatesNumeric(cotr)
                    moments = utils.rotoTranslateInertiaMoments(props.moments, props.mass, com, H[0:3,3], H[0:3,0:3])

                    # Transform the CoM vector in link frame coordinates
                    com = H[0:3,0:3].T @ (com - H[0:3,3])

                    newcom  = (newcom*newmass + com*props.mass) / (newmass + props.mass)
                    newmass = newmass + props.mass
                    newmoments.ixx = newmoments.ixx + moments.ixx
                    newmoments.ixy = newmoments.ixy + moments.ixy
                    newmoments.ixz = newmoments.ixz + moments.ixz
                    newmoments.iyy = newmoments.iyy + moments.iyy
                    newmoments.iyz = newmoments.iyz + moments.iyz
                    newmoments.izz = newmoments.izz + moments.izz

            inertialData[newLink.name] = robmodel.inertia.BodyInertia(mass=newmass,
                com= robmodel.inertia.CoM(frame=linkFrame, x=newcom[0], y=newcom[1], z=newcom[2]),
                moments = newmoments )

        newinertia = robmodel.inertia.RobotLinksInertia(newordering, newframes, inertialData)

    return newconnectivity, newordering, newframes, newgeometry, newinertia


