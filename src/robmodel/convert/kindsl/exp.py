import logging
from mako.template import Template

import robmodel.convert.utils as utils

import kgprim.ct as ct
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

def geometry(geometryModel):
    connect= geometryModel.connectivityModel
    frames = geometryModel.framesModel
    formatter = utils.FloatsFormatter(pi_string="PI")
    tree = TreeUtils(connect)
    return tpl.render(
        robot=connect,
        tree=tree,
        jSection=jointSectionName,
        jIsSupported=__isSupported,
        jointFrameParams=lambda j : jointFrameParams(geometryModel, j),
        linkUserFrames=lambda l : linkUserFrames(frames, l),
        frameParams= lambda f : userFrameParams(geometryModel, f),
        tostr=lambda num, isAngle=False: formatter.float2str(num, isAngle)
    )

