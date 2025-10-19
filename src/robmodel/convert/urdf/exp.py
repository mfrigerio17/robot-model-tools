import logging
import numpy as np
import dataclasses
from mako.template import Template

import robmodel.convert.utils as utils
import robmodel.geometry
import robmodel.jlimits

import kgprim.ct as ct
import kgprim.ct.repr.mxrepr as mxrepr

logger = logging.getLogger(__name__)

# The template for URDF generation.
# - generate an element for each link and joint of the input model
# - if the geometry model is not None, also generate the joint "origin" element
#   with its numerical attributes
# - generate dummy link/joint pairs to represent possible additional user frames
#   available in the input model
# - generate dummy link/joint pairs to represent possible extra frames given to
#   the generate function (see variable `extraPoses`)

tpl = Template('''
<robot name="${robot.name}">
%if inertia is not None :
    %for link in robot.links.values():
<% m,cx,cy,cz,r,p,y,ixx,iyy,izz,ixy,ixz,iyz = linkInertia(link) %>
    <link name="${link.name}">
        <inertial>
            <origin xyz="${tostr(cx)} ${tostr(cy)} ${tostr(cz)}" rpy="${tostr(r)} ${tostr(p)} ${tostr(y)}"/>
            <mass value="${tostr(m)}"/>
            <inertia ixx="${tostr(ixx)}"  ixy="${tostr(ixy)}"  ixz="${tostr(ixz)}" iyy="${tostr(iyy)}" iyz="${tostr(iyz)}" izz="${tostr(izz)}" />
        </inertial>
    </link>
    %endfor
%else :
    %for link in robot.links.values():
    <link name="${link.name}">
    </link>
    %endfor
%endif

%if geometry is not None :
<% dummyLinks = set() %>
%for aframe in geometry.framesModel.userAttachedFrames:
    <link name="${aframe.entity.name}">
    </link>
<% x,y,z,rx,ry,rz = jointParams(geometryModel=geometry, attachedFrame=aframe);  dummyLinks.add(aframe.entity.name) %>
    <joint name="dummy_${aframe.entity.name}" type="fixed">
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
        <parent link="${aframe.body.name}"/>
        <child  link="${aframe.entity.name}"/>
    </joint>

%endfor
%if extraPoses is not None :
    <!-- Dummy links/joints to model the given extra frames -->

%for pose in extraPoses:
<%  ref = pose.pose.reference.name; tgt=pose.pose.target.name %>
%   if ref not in robot.links.values() and ref not in dummyLinks :
    <link name="${ref}"></link>
<%      dummyLinks.add(ref) %>
%   endif
%   if tgt not in robot.links.values() and tgt not in dummyLinks :
    <link name="${tgt}"></link>
<%      dummyLinks.add(tgt) %>
    %endif
<% x,y,z,rx,ry,rz = jointParams(poseSpec=pose) %>
    <joint name="dummy_between_${ref}_and_${tgt}" type="fixed">
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
        <parent link="${ref}"/>
        <child  link="${tgt}"/>
    </joint>
%endfor

    <!-- end of section for extra dummy links/joints -->
%endif
%endif

%for joint in robot.joints.values():
    <joint name="${joint.name}" type="${jointKind(joint)}">
<% jlim = jlimits.byJoint[joint] %>
%if geometry is not None :
<% x,y,z,rx,ry,rz = jointParams(geometryModel=geometry, joint=joint) %>
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
<% x,y,z = geometry.jointAxes[joint.name] %>
        <axis xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}"/>
%endif
        <parent link="${robot.predecessor(joint).name}"/>
        <child  link="${robot.successor  (joint).name}"/>
        <limit effort="${jlim.force}" velocity="${jlim.velocity}" lower="${jlim.lower_pos}" upper="${jlim.upper_pos}" />
    </joint>

% endfor
</robot>
'''
)

def jointKind(joint):
    if isinstance(joint.kind, str) :
        return joint.kind
    return joint.kind.name

def _poseSpecToURDFJointParamters(poseSpec):
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    H  = mxrepr.hCoordinatesNumeric.matrix_repr(xt)
    irx,iry,irz = utils.getIntrinsicXYZFromR( H )
    erx,ery,erz = utils.intrinsic2extrinsic_XYZ(irx, iry, irz)

    return H[0,3], H[1,3], H[2,3], erx, ery, erz


def jointOrigin(**kwargs):
    '''
    The six numerical parameters for a <joint> element in the URDF.
    Accepts three combinations of arguments: geometry model and joint,
    geometry model and attached frame, or an explicit pose-specification.
    '''
    poseSpec = None
    if "geometryModel" in kwargs:
        geometry = kwargs["geometryModel"]
        if "joint" in kwargs:
            poseSpec = geometry.byJoint[ kwargs["joint"] ]
        elif "attachedFrame" in kwargs:
            attachedFrame = kwargs["attachedFrame"]
            poseSpec = robmodel.geometry.getPoseSpec(geometry, attachedFrame)
            if poseSpec is None:
                logger.warning("Could not retrieve the pose of frame '{}'".format(attachedFrame.entity.name))
                return 0,0,0,0,0,0
        else:
            raise TypeError("need 'joint' or 'attachedFrame' when the geometry model is given")
    elif "poseSpec" in kwargs:
        poseSpec = kwargs["poseSpec"]
    else:
        raise TypeError("invalid arguments")

    return _poseSpecToURDFJointParamters(poseSpec)


def linkInertia(geometryModel, inertiaModel, link):
    props_in = inertiaModel.byLink(link)
    if props_in is None :
        return 0, 0,0,0, 0,0,0, 0,0,0,0,0,0

    com = np.array((props_in.com.x, props_in.com.y, props_in.com.z, 1))
    rpy = (0.0,0.0,0.0)

    # Check if the CoM coordinates are not link-frame coordinates.
    # If not, we perform the roto-translation.
    # The URDF wants the CoM in link-frame coordinates.
    frame_com = props_in.com.frame
    if frame_com.body  != link:
        link_TR_comfr = robmodel.geometry.getPoseSpec(geometryModel, frame_com)
        if link_TR_comfr is None:
            logger.error("Could not retrieve the pose of the source CoM frame '{}' relative to the frame of link '{}'"
                .format(frame_com, link))
            raise RuntimeError("URDF export: failed to convert a CoM")
        link_CT_comfr = ct.frommotions.toCoordinateTransform( link_TR_comfr )
        link_H_comfr  = mxrepr.hCoordinatesNumeric.matrix_repr( link_CT_comfr )
        com = np.matmult(link_H_comfr, com)

    # Similarly for the inertia moments.
    # If a custom frame is used for the source data, use its orientation to
    # determine the 'rpy' attribute in the URDF. There is no need to rotate the
    # inertia moments.
    # In general, though, we need to translate them because the URDF wants them
    # in a frame with origin at the CoM.
    com__tr_moments = com[0:3] # default translation vector for the inertia moments
    frame_moments = props_in.moments.frame
    if frame_moments.body != link:
        link_TR_momentsfr = robmodel.geometry.getPoseSpec(geometryModel, frame_moments)
        if link_TR_momentsfr is None:
            logger.error("Could not retrieve the pose of the source inertia moments frame '{}' relative to the frame of link '{}'"
                .format(frame_com, link))
            raise RuntimeError("URDF export: failed to convert inertia moments")
        link_CT_momentsfr = ct.frommotions.toCoordinateTransform(link_TR_momentsfr)
        link_H_momentsfr  = mxrepr.hCoordinatesNumeric.matrix_repr(link_CT_momentsfr)
        irx,iry,irz = utils.getIntrinsicXYZFromR(link_H_momentsfr)
        rpy         = utils.intrinsic2extrinsic_XYZ(irx, iry, irz)

        momentsfr_CT_link = ct.frommotions.toCoordinateTransform(link_TR_momentsfr, rigth_frame=geometryModel.framesModel.linkFrame[link])
        momentsfr_H_link  = mxrepr.hCoordinatesNumeric.matrix_repr(momentsfr_CT_link)
        com__tr_moments = momentsfr_H_link @ com

    moments = utils.translateInertiaMoments(props_in.moments, props_in.mass, com__tr_moments)

    return( props_in.mass, com[0], com[1], com[2], rpy[0], rpy[1], rpy[2], moments.ixx, moments.iyy, moments.izz, -moments.ixy, -moments.ixz, -moments.iyz)


def ordering(orderingModel):
    logger.warn("Generated joint limits are arbitrary")
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=orderingModel,
        geometry=None,
        jointParams=None,
        jointKind = jointKind,
        tostr=lambda num: formatter.float2str(num)
    )

def modelText(geometryModel, inertiaModel=None, userExtraPoses=None, jointLimits=None):
    if jointLimits is None:
        logger.warn("generated joint limits are arbitrary")
        limits = {joint : dataclasses.asdict(robmodel.jlimits.JointLimit())
                     for joint in geometryModel.connectivityModel.joints.keys()}
        jointLimits = robmodel.jlimits.JointLimits(geometryModel.connectivityModel, limits)

    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=geometryModel.connectivityModel,
        geometry=geometryModel,
        inertia=inertiaModel,
        jlimits=jointLimits,
        extraPoses=userExtraPoses,
        jointParams=jointOrigin,
        jointKind = jointKind,
        linkInertia = lambda link: linkInertia(geometryModel, inertiaModel, link),
        tostr       = lambda num: formatter.float2str(num),
    )
