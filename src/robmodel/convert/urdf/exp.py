import logging
from mako.template import Template

import robmodel.convert.utils as utils
import robmodel.geometry

import kgprim.ct as ct
import kgprim.ct.repr.mxrepr as mxrepr

logger = logging.getLogger(__name__)

tpl = Template('''
<robot name="${robot.name}">
%for link in robot.links.values():
    <link name="${link.name}">
    </link>
%endfor

%if geometry is not None :
%for aframe in geometry.framesModel.userAttachedFrames:
    <link name="${aframe.entity.name}">
    </link>
    <% x,y,z,rx,ry,rz = dummyJointParams(geometry, aframe) %>
    <joint name="dummy_${aframe.entity.name}" type="fixed">
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
        <parent link="${aframe.body.name}"/>
        <child  link="${aframe.entity.name}"/>
    </joint>

%endfor
%endif

%for joint in robot.joints.values():
    <joint name="${joint.name}" type="${jointKind(joint)}">
%if geometry is not None :
    <% x,y,z,rx,ry,rz = jointParams(geometry, joint) %>
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
%endif
        <parent link="${robot.predecessor(joint).name}"/>
        <child  link="${robot.successor  (joint).name}"/>
        <% x,y,z = geometry.jointAxes[joint.name] %>
        <axis xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}"/>
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

def fixedDummyJointOrigin(geometryModel, attachedFrame):
    poseSpec = robmodel.geometry.getPoseSpec(geometryModel, attachedFrame)
    if poseSpec is None:
        logger.warning("Could not retrieve the pose of frame '{}'".format(attachedFrame.entity.name))
        return 0,0,0,0,0,0
    return _poseSpecToURDFJointParamters(poseSpec)

def jointOrigin(geometryModel, joint):
    poseSpec = geometryModel.byJoint[ joint ]
    return _poseSpecToURDFJointParamters(poseSpec)


def ordering(orderingModel):
    logger.warn("Generated joint limits are arbitrary")
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=orderingModel,
        geometry=None,
        jointParams=None,
        tostr=lambda num: formatter.float2str(num)
    )

def geometry(geometryModel):
    logger.warn("Generated joint limits are arbitrary")
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=geometryModel.connectivityModel,
        geometry=geometryModel,
        jointParams=jointOrigin,
        dummyJointParams=fixedDummyJointOrigin,
        jointKind = jointKind,
        tostr=lambda num: formatter.float2str(num)
    )
