import logging
import numpy as np
import numpy.linalg
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
<% m,cx,cy,cz,r,p,y,ixx,iyy,izz,ixy,ixz,iyz,rpy_nonzero = linkInertia(link) %>
    <link name="${link.name}">
        <inertial>
%       if rpy_nonzero:
            <origin xyz="${tostr(cx)} ${tostr(cy)} ${tostr(cz)}" rpy="${tostr(r)} ${tostr(p)} ${tostr(y)}"/>
%       else:
            <origin xyz="${tostr(cx)} ${tostr(cy)} ${tostr(cz)}"/>
%endif
            <mass value="${tostr(m)}"/>
            <inertia ixx="${tostr(ixx)}" iyy="${tostr(iyy)}" izz="${tostr(izz)}" ixy="${tostr(ixy)}" ixz="${tostr(ixz)}" iyz="${tostr(iyz)}" />
        </inertial>
    </link>
    %endfor
%else :
    %for link in robot.links.values():
    <link name="${link.name}">
    </link>
    %endfor
%endif

%if includeDummies :
    <!-- Dummy links/joints to model extra frames -->
%if geometry is not None :
<% dummyLinks = set() %>
%for _,aframe in geometry.framesModel.userFrames.items():
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
%endif
%endif
    <!-- end of section for extra dummy links/joints -->
%endif

%for joint in robot.joints.values():
    <joint name="${joint.name}" type="${jointKind(joint)}">
%if geometry is not None :
<% x,y,z,rx,ry,rz = jointParams(geometryModel=geometry, joint=joint) %>
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
<% x,y,z = geometry.jointAxes[joint.name] %>
        <axis xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}"/>
%endif
        <parent link="${robot.predecessor(joint).name}"/>
        <child  link="${robot.successor  (joint).name}"/>
%if jointKind(joint) != "fixed" :
<% jlim = jointLimits(joint) %>
        <limit effort="${jlim.force}" velocity="${jlim.velocity}" lower="${jlim.lower_pos}" upper="${jlim.upper_pos}" />
%endif
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
        return 0, 0,0,0, 0,0,0, 0,0,0,0,0,0, False

    com = np.array((props_in.com.x, props_in.com.y, props_in.com.z, 1))
    rpy = (0.0,0.0,0.0)
    rpy_nonzero = False
    linkFrame = geometryModel.framesModel.byLink[link]

    # Check if the CoM coordinates are not link-frame coordinates.
    # If not, we perform the roto-translation.
    # The URDF wants the CoM position in link-frame coordinates.
    frame_com = props_in.com.frame
    if frame_com.body != link:
        logger.error( ("The frame of the CoM must be one "
                       "attached to the corresponding link (the link is "
                       "'%s', the frame is '%s' attached to '%s')"),
                       link.name, frame_com.name, frame_com.body.name)
        raise RuntimeError("URDF export: failed to convert inertia moments")
    if frame_com != linkFrame:
        link_TR_comfr = robmodel.geometry.getPoseSpec(geometryModel, frame_com)
        if link_TR_comfr is None:
            logger.error(("Could not retrieve the pose of the input CoM "
                          "frame '%s' relative to the frame of link '%s'"),
                            frame_com.name, link.name)
            raise RuntimeError("URDF export: failed to convert a CoM")
        link_CT_comfr = ct.frommotions.toCoordinateTransform( link_TR_comfr )
        link_H_comfr  = mxrepr.hCoordinatesNumeric.matrix_repr( link_CT_comfr )
        com = np.matmult(link_H_comfr, com) # CoM position in link-frame coordinates

    # For the inertia moments...
    # If a custom frame is used for the source data, use its orientation to
    # determine the 'rpy' attribute in the URDF. There is no need to rotate the
    # inertia moments.
    # In general, though, we need to translate them because the URDF wants
    # the inertia moments about the CoM.
    moments = props_in.moments
    frame_moments = props_in.moments.frame
    if frame_moments.body != link:
        logger.error( ("The frame of the inertial moments must be one "
                       "attached to the corresponding link (the link is "
                       "'%s', the frame is '%s' attached to '%s')"),
                       link.name, frame_moments.name, frame_moments.body.name)
        raise RuntimeError("URDF export: failed to convert inertia moments")

    if frame_moments == linkFrame:
        # inertia moments are in link-framme coordinates
        moments = utils.translateInertiaMoments(props_in.moments, props_in.mass, com)
    else:
        link_TR_momentsfr = robmodel.geometry.getPoseSpec(geometryModel, frame_moments)
        if link_TR_momentsfr is None:
            logger.error(("Could not retrieve the pose of frame '%s' "
                          "relative to the frame of link '%s'"),
                          frame_moments.name, link.name)
            raise RuntimeError("URDF export: failed to convert inertia moments")

        link_CT_momentsfr = ct.frommotions.toCoordinateTransform(link_TR_momentsfr, right_frame=frame_moments)
        link_H_momentsfr  = mxrepr.hCoordinatesNumeric(link_CT_momentsfr)
        irx,iry,irz       = utils.getIntrinsicXYZFromR(link_H_momentsfr)
        trvec             = link_H_momentsfr[0:4,3]

        # I have no way to check symbolically whether the frame of the
        # inertia moments is already the CoM frame. I do it numerically
        already_in_com_frame = (abs(irx+iry+iry)<1e-5) and (np.linalg.norm(trvec-com)<1e-5)
        if not already_in_com_frame:
            rpy = utils.intrinsic2extrinsic_XYZ(irx, iry, irz)
            rpy_nonzero = np.linalg.norm(rpy)>1e-5
            momentsfr_CT_link = ct.frommotions.toCoordinateTransform(link_TR_momentsfr, rigth_frame=linkFrame)
            momentsfr_H_link  = mxrepr.hCoordinatesNumeric.matrix_repr(momentsfr_CT_link)

            # The position of the CoM relative to the origin of the
            # moments-frame, in moments-frame coordinates
            com__tr_moments = momentsfr_H_link @ com
            moments = utils.translateInertiaMoments(props_in.moments, props_in.mass, com__tr_moments)

    return( props_in.mass, com[0], com[1], com[2], rpy[0], rpy[1], rpy[2],
        moments.ixx, moments.iyy, moments.izz, -moments.ixy, -moments.ixz, -moments.iyz,
        rpy_nonzero)


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

def modelText(geometryModel, inertiaModel=None, userExtraPoses=None, jointLimits=None, includeDummies=True):
    if jointLimits is None:
        logger.warning("generated joint limits are arbitrary")
        limits = {joint : dataclasses.asdict(robmodel.jlimits.JointLimit())
                     for joint in geometryModel.connectivityModel.joints.keys()}
        jointLimits = robmodel.jlimits.JointLimits(geometryModel.connectivityModel, limits)

    def jLimits(joint):
        nonlocal jointLimits
        data = jointLimits.byJoint.get(joint)
        if data is None:
            logger.warning("no limits data for joint '%s'", joint.name)
            data = robmodel.jlimits.JointLimit()
        return data

    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=geometryModel.connectivityModel,
        geometry=geometryModel,
        inertia=inertiaModel,
        jointLimits=jLimits,
        extraPoses=userExtraPoses,
        jointParams=jointOrigin,
        jointKind = jointKind,
        includeDummies = includeDummies,
        linkInertia = lambda link: linkInertia(geometryModel, inertiaModel, link),
        tostr       = lambda num: formatter.float2str(num),
    )
