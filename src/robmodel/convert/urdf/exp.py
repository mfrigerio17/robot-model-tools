import logging
from mako.template import Template

import robmodel.convert.utils as utils
import robmodel.geometry

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
%for link in robot.links.values():
    <link name="${link.name}">
    </link>
%endfor

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
%if geometry is not None :
<% x,y,z,rx,ry,rz = jointParams(geometryModel=geometry, joint=joint) %>
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
<% x,y,z = geometry.jointAxes[joint.name] %>
        <axis xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}"/>
%endif
        <parent link="${robot.predecessor(joint).name}"/>
        <child  link="${robot.successor  (joint).name}"/>
        <limit effort="30" velocity="10.0" lower="-3.14" upper="3.14" />
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

def geometry(geometryModel, userExtraPoses=None):
    logger.warn("Generated joint limits are arbitrary")
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=geometryModel.connectivityModel,
        geometry=geometryModel,
        extraPoses=userExtraPoses,
        jointParams=jointOrigin,
        jointKind = jointKind,
        tostr=lambda num: formatter.float2str(num)
    )
