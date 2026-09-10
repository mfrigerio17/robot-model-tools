import logging, re, math
import numpy as np
from mako.template import Template

import robmodel.convert.utils as utils

import kgprim.core
import kgprim.ct as ct
import kgprim.motions as mot
import kgprim.ct.repr.mxrepr as mxrepr

from robmodel.connectivity import JointKind
from robmodel.treeutils import TreeUtils
import robmodel.frames
import robmodel.geometry

logger = logging.getLogger(__name__)

tpl = Template('''
Robot ${robot.name}
{
% for link in robot.links.values():
% if link == robot.base :
    RobotBase ${id(link.name)} {
% else :
    link ${id(link.name)} {
        id = ${robot.linkNum(link)}
% endif
% if inertia is not None :
<% m,x,y,z,ix,iy,iz,ixy,ixz,iyz = linkInertia(link) %>
        inertia_properties {
            mass = ${tostr(m)}
            CoM  = (${tostr(x)}, ${tostr(y)}, ${tostr(z)})
            Ix=${tostr(ix)}  Iy=${tostr(iy)}  Iz=${tostr(iz)}  Ixy=${tostr(ixy)}  Ixz=${tostr(ixz)}  Iyz=${tostr(iyz)}
        }
% endif
        children {
        % for child in tree.children(link):
            ${id(child.name)} via ${id(robot.linkPairToJoint(link, child).name)}
        % endfor
        }
        <% userFrames = linkUserFrames(link) %>
        % if len(userFrames) > 0 :
        frames {
        % for f in userFrames :
            <% x,y,z,rx,ry,rz = frameParams(f) %>
            ${f.name} {
                translation = (${tostr(x)}, ${tostr(y)}, ${tostr(z)})
                rotation    = (${tostr(rx, True)}, ${tostr(ry, True)}, ${tostr(rz, True)})
            }
        % endfor
        }
        % endif
    }

% endfor


% for joint in robot.joints.values() :
% if jIsSupported(joint) :
<% x,y,z,rx,ry,rz = jointFrameParams(joint) %>
    ${jSection(joint)} ${id(joint.name)} {
        ref_frame {
            translation = (${tostr(x)}, ${tostr(y)}, ${tostr(z)})
            rotation    = (${tostr(rx, True)}, ${tostr(ry, True)}, ${tostr(rz, True)})
        }
    }
% else :
// WARNING: unsupported joint type for '${joint.name}'
% endif
% endfor

}
'''
)


__joint_section = {
    JointKind.prismatic : "p_joint",
    JointKind.revolute  : "r_joint"
}

def __isSupported(joint):
    ret = joint.kind in __joint_section
    if not ret:
        logger.warning("Unsupported joint kind '{}' for joint '{}'".format(joint.kind, joint.name))
    return ret

def jointSectionName(joint):
    return __joint_section[ joint.kind ]

def jointFrameParams(geometryModel, joint):
    poseSpec = geometryModel.byJoint[ joint ]
    return poseParams(poseSpec)

def userFrameParams(geometryModel, frame):
    poseSpec = geometryModel.getPoseSpec(frame)
    if poseSpec is not None :
        return poseParams(poseSpec)

    logger.warning("Could not find pose information for frame '%s'", frame.name)
    return 0,0,0,0,0,0

def poseParams( poseSpec ):
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    H  = mxrepr.hCoordinatesNumeric(xt)
    irx,iry,irz = utils.getIntrinsicXYZFromR( H )

    return H[0,3], H[1,3], H[2,3], irx,iry,irz

def linkUserFrames(framesModel, link):
    ret = []
    lf = framesModel.linkFrames[link]
    for uf in framesModel.graph[lf] :
        kind = framesModel.kind(lf, uf)
        if kind == robmodel.frames.FrameRelationKind.generic :
            ret.append(uf)
    return ret

def inertiaProperties(geometryModel, inertiaModel, link) :

    props = inertiaModel.byLink(link)
    if props is None :
        logger.warning("Missing inertia properties for link '%s'", link.name)
        return (1, 0.1, 0.1, 0.1, 1, 1, 1, 0, 0, 0)

    # we are optimistic, no conversions needed by default
    com     = (props.com.x, props.com.y, props.com.z, 1)
    moments = props.moments

    frame_com  = props.com.frame
    frame_moms = props.moments.frame
    frame_link = geometryModel.framesModel.byLink[link]
    link_TR_comfr = None
    link_H_comfr  = None

    if frame_com != frame_link :
        link_TR_comfr = geometryModel.getPoseSpec(frame_com)
        if link_TR_comfr is None:
            logger.error("Cannot find the pose of the CoM-frame '{}' relative to the frame of link '{}', for the input model '{}'"
                              .format(frame_com, link, geometryModel.robotName))
            raise RuntimeError("KinDSL export: failed to convert a CoM")
        link_CT_comfr = ct.frommotions.toCoordinateTransform  ( link_TR_comfr, right_frame=frame_com )
        link_H_comfr  = mxrepr.hCoordinatesNumeric.matrix_repr( link_CT_comfr )
        com = link_H_comfr @ com

    if  frame_moms != frame_link :
        pose_of_moments_frame = geometryModel.getPoseSpec(frame_moms)
        if pose_of_moments_frame is None:
            logger.error("Cannot find the pose of frame '{}' relative to the frame of link '{}', for the input model '{}'"
                .format(frame_moms, link, geometryModel.robotName))
            raise RuntimeError("KinDSL export: failed to convert inertia moments")
        moms_CT_link = ct.frommotions.toCoordinateTransform(pose_of_moments_frame, right_frame=frame_link)
        moms_H_link  = mxrepr.hCoordinatesNumeric.matrix_repr(moms_CT_link)

        # the CoM relative to the moments frame
        com_moms = moms_H_link @ com

        # the origin of the link frame in the moments frame
        origin_moms = moms_H_link[0:3,3]

        moments = utils.rotoTranslateInertiaMoments(props.moments, props.mass, com_moms, origin_moms, moms_H_link[0:3,0:3] )

    return (props.mass, com[0], com[1], com[2],
        moments.ixx,
        moments.iyy,
        moments.izz,
        moments.ixy,
        moments.ixz,
        moments.iyz)



def convert(geometry, inertia):
    '''
    "Normalize" the given models so as to conform with the KinDSL format
    constraints: the joint axis is always the Z axis of the joint frame.
    The relative orientation of the robot frames might therefore change,
    thus the inertial properties need to be also adapted.
    '''
    connectivity = geometry.connectivityModel
    frames       = geometry.framesModel

    # Overall strategy and remarks:
    # - consider the joints whose axis is NOT the Z axis of the joint frame
    # - figure out the rotations to align the Z axis of the frame with the joint axis
    # - moving a joint frame effectively moves the successor frame too (the two coincide at 0 joint state)
    #   - must take it into account when processing in turn the joints supported by that link
    # - when moving a joint frame (thus the successor frame too), we preserve the
    #   information of the pose of the original frame, adding an additional custom
    #   frame ("user" frame) in the model, with its pose
    # - such new frame allows us to "reseat" relative poses and inertial properties,
    #   by changing symbolically their frame of reference, without performing any
    #   numerical conversion (at least in this function)

    # In the current implementation I compute the rotations to align the Z of the
    # original joint frame. This makes it easy to derive the steps to locate the
    # new frame, by simple composition of:
    #  - the steps to move to where the link frame of the original model was
    #  - the steps to locate the joint frame, from the original model, unchanged
    #  - the new rotations computed for the alignment of Z
    # While this is correct and beautiful (because it minimises numerical
    # coputations in favor of symbolic composition), it may be suboptimal
    # in terms of number of steps. The alternative, is to consider the new
    # link frame, find the rotations to align its Z axis, convert numerically
    # the original translation steps; then, compose:
    #  - the modified translation steps
    #  - the computed rotations to align Z
    # In this way it's possible to get the same model but with less non-zero parameters.

    # Observe that the specs of the pose of any joint frame may change compared to
    # the input model, for _two_ reasons:
    #  - the joint frame itself has to "physically" move to align the Z axis, and/or
    #  - it is the link(predecessor) frame that moved (due to another joint), thus
    #    the relative pose with the joint frame changes anyway!

    # Bookkeeping support vars
    rot_from_linkframe_to_original = {}
    allNewPoses = []
    linkFramesOfOriginalModel = {}
    newUserFrames = list(frames.userFrames.values())

    for joint in connectivity.joints.values():
        # We need to find the intrinsic (successive) rotations rx ry for the
        # joint frame, such that the Z axis of the resulting frame is aligned
        # with the joint axis; such is the convention of the KinDSL format.
        # We take the generic rotation matrix corresponding to instrinsic rx ry
        # rotations, and we equate the third column (Z axis) with the joint axis:
        #
        #     sin(ry)         = axis_x
        #   - sin(rx) cos(ry) = axis_y
        #     cos(rx) cos(ry) = axis_z
        axis = np.array( geometry.jointAxes[joint.name] )
        axis = np.round(axis, 5)
        if np.array_equal(axis, np.array([0,0,1])):
            rotsToAlignZ = mot.MotionSequence([])
        else:
            ry = math.asin( axis[0] )
            if axis[2] != 0.0 :
                rx = math.atan2( -axis[1], axis[2])
            else :
                cy = math.cos(ry)
                if round(cy,5) != 0.0 :
                    arg = - axis[1] / cy
                    if math.fabs(arg) > 1 :
                        arg = math.copysign(1, arg)
                    rx = math.asin( arg )
                else:
                    rx = 0.0
            steps = []
            if rx != 0.0: steps.append( mot.MotionStep(mot.MotionStep.Kind.Rotation, mot.Axis.X, rx) )
            if ry != 0.0: steps.append( mot.MotionStep(mot.MotionStep.Kind.Rotation, mot.Axis.Y, ry) )
            rotsToAlignZ = mot.MotionSequence(steps, mot.MotionSequence.Mode.currentFrame)
            rotsToOriginalFrame = mot.reverse(rotsToAlignZ)

            # Keep track that the frame of the successor link changes too
            successor = connectivity.successor(joint)
            rot_from_linkframe_to_original[successor] = rotsToOriginalFrame

            # Keep track of the changes: create another link-attached frame to represent
            # the original link frame, and store the relative pose
            originalframe = kgprim.core.Attachment(body=successor, entity=kgprim.core.Frame("original_" + successor.name))
            linkFramesOfOriginalModel[successor] = originalframe
            newUserFrames.append(originalframe)
            pose = mot.Pose(target=originalframe, reference=frames.byLink[successor])
            allNewPoses.append( mot.PoseSpec(pose=pose, motion=rotsToOriginalFrame) )

        # Adjust the specs of the joint frame pose relative to the predecessor frame
        #
        predec = connectivity.predecessor(joint)
        rotsToOriginalFrame = rot_from_linkframe_to_original.get(predec, mot.MotionSequence([]))

        # the original pose specs, of the joint frame relative to the link frame of the original model
        jointFramePoseSpec = geometry.byJoint[ joint ]
        # The full list of steps to move to the new desired frame, is the composition of the
        # steps to move back to the original frame, plus the original specs of where the frame
        # is, plus our rotation to align Z with the joint axis.
        motions = [rotsToOriginalFrame]
        motions.extend( jointFramePoseSpec.motion.sequences )
        motions.append( rotsToAlignZ )

        allNewPoses.append( mot.PoseSpec(jointFramePoseSpec.pose, mot.MotionPath(motions)) )

    # Adjust the specs of the poses of all the custom frames.
    # There is no need to change the concrete motion steps, we can just update the
    # reference frame: for the original poses relative to the original link-frame,
    # we just swap the reference with the new frame we created
    for name,frame in frames.userFrames.items():
        poseSpec = geometry.getPoseSpec(frame)

        # we only care for robot links whose frame actually changed
        if frame.body in linkFramesOfOriginalModel:
            trueOriginalLinkFrame       = frames.byLink[frame.body]
            newFrameWhereTheOriginalWas = linkFramesOfOriginalModel[frame.body]

            if poseSpec.pose.reference == trueOriginalLinkFrame:
                newpose = mot.Pose(target=poseSpec.pose.target, reference=newFrameWhereTheOriginalWas)
                poseSpec = mot.PoseSpec(pose=newpose, motion=poseSpec.motion)
        allNewPoses.append(poseSpec)

    newFramesModel = robmodel.frames.RobotDefaultFrames(connectivity, newUserFrames)
    newPosesModel  = mot.PosesSpec(name=geometry.poses.name, poses=allNewPoses)
    newgeometry    = robmodel.geometry.Geometry(connectivity, newFramesModel, newPosesModel)

    # For the inertial properties we do the same as above: no numerical conversions,
    # we only change the frame with respect to which the properties are expressed
    # (if necessary). This way we leverage the existing code that does the numerical
    # conversion of inertia.
    if inertia is None: return newgeometry, inertia
    inertiaData = {}
    for lname,link in connectivity.links.items():
        linkInertia = inertia.byLink(link)
        if link in linkFramesOfOriginalModel:
            trueOriginalLinkFrame       = frames.byLink[link]
            newFrameWhereTheOriginalWas = linkFramesOfOriginalModel[link]
            if linkInertia.com.frame == trueOriginalLinkFrame:
                linkInertia.com.frame = newFrameWhereTheOriginalWas
            if linkInertia.moments.frame == trueOriginalLinkFrame:
                linkInertia.moments.frame = newFrameWhereTheOriginalWas
        inertiaData[lname] = linkInertia

    newinertia = robmodel.inertia.RobotLinksInertia(connectivity, newFramesModel, inertiaData)

    return newgeometry, newinertia


def modelText(geometryModel, inertiaModel=None):
    geometry, inertia = convert(geometryModel, inertiaModel)
    connect= geometry.connectivityModel
    frames = geometry.framesModel
    formatter = utils.FloatsFormatter(pi_string="PI", round_digits=5)
    tree = TreeUtils(connect)

    return tpl.render(
        robot=connect,
        tree=tree,
        inertia=inertia,
        jSection=jointSectionName,
        jIsSupported=__isSupported,
        jointFrameParams=lambda j : jointFrameParams(geometry, j),
        linkUserFrames=lambda l : linkUserFrames(frames, l),
        frameParams= lambda f : userFrameParams(geometry, f),
        linkInertia= lambda link : inertiaProperties(geometry, inertia, link),
        tostr=lambda num, isAngle=False: formatter.float2str(num, isAngle),
        id= lambda s: re.sub('\\W', '_', s)
    )

