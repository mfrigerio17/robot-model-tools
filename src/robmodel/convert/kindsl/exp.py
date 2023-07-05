import logging
import numbers
from mako.template import Template

import robmodel.convert.utils as utils

import kgprim.ct as ct
import kgprim.ct.metadata as ctmetadata
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
    RobotBase ${link.name} {
% else :
    link ${link.name} {
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
            ${child.name} via ${robot.linkPairToJoint(link, child).name}
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
    ${jSection(joint)} ${joint.name} {
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
    poseSpec = robmodel.geometry.getPoseSpec(geometryModel, frame)
    if poseSpec is not None :
        return poseParams(poseSpec)

    logger.warning("Could not find pose information for frame '{f}'".format(f=frame.name))
    return 0,0,0,0,0,0

def poseParams( poseSpec ):
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    mdata = ctmetadata.TransformMetadata(xt)
    H = None
    if mdata.parametric:
        H  = mxrepr.hCoordinatesSymbolic(xt)
    else:
        H  = mxrepr.hCoordinatesNumeric(xt)
    # the next will fail in case of symbols, because it uses functions from math
    # We need a way to retrieve the "right" math functions for a given transform,
    #  like C++ traits.
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
        link_TR_comfr = robmodel.geometry.getPoseSpec(geometryModel, frame_com)
        if link_TR_comfr is None:
            logger.error("Cannot find the pose of the CoM-frame '{}' relative to the frame of link '{}', for the input model '{}'"
                              .format(frame_com, link, geometryModel.robotName))
            raise RuntimeError("KinDSL export: failed to convert a CoM")
        link_CT_comfr = ct.frommotions.toCoordinateTransform  ( link_TR_comfr, right_frame=frame_com )
        link_H_comfr  = mxrepr.hCoordinatesNumeric.matrix_repr( link_CT_comfr )
        com = link_H_comfr @ com

    if  frame_moms != frame_link :
        pose_of_moments_frame = robmodel.geometry.getPoseSpec(geometryModel, frame_moms)
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

def modelText(geometryModel, inertiaModel=None):
    connect= geometryModel.connectivityModel
    frames = geometryModel.framesModel
    formatter = Formatter(utils.FloatsFormatter(pi_string="PI"))
    tree = TreeUtils(connect)

    return tpl.render(
        robot=connect,
        tree=tree,
        inertia=inertiaModel,
        jSection=jointSectionName,
        jIsSupported=__isSupported,
        jointFrameParams=lambda j : jointFrameParams(geometryModel, j),
        linkUserFrames=lambda l : linkUserFrames(frames, l),
        frameParams= lambda f : userFrameParams(geometryModel, f),
        linkInertia= lambda link : inertiaProperties(geometryModel, inertiaModel, link),
        tostr=lambda num, isAngle=False: formatter.toString(num, isAngle)
    )

class Formatter:
    def __init__(self, floatsFormatter):
        self.fformatter = floatsFormatter

    def toString(self, value, isAngle=False):
        if isinstance(value, numbers.Number):
            return self.fformatter.float2str(value, isAngle)
        else:
            return str(value)

