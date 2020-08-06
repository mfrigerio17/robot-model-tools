from mako.template import Template

import robmodel.convert.utils as utils

import kgprim.ct as ct
import kgprim.ct.repr.mxrepr as mxrepr

tpl = Template('''
<robot name="${robot.name}">
%for link in robot.links.values():
    <link name="${link.name}">
    </link>
%endfor

%for joint in robot.joints.values():
    <joint name="${joint.name}" type="${joint.kind.name}">
%if geometry is not None :
    <% x,y,z,rx,ry,rz = jointParams(geometry, joint) %>
        <origin xyz="${tostr(x)} ${tostr(y)} ${tostr(z)}" rpy="${tostr(rx)} ${tostr(ry)} ${tostr(rz)}"/>
%endif
        <parent link="${robot.predecessor(joint).name}"/>
        <child  link="${robot.successor  (joint).name}"/>
        <axis xyz="0 0 1"/>
    </joint>

% endfor
</robot>
'''
)

def jointOrigin(geometryModel, joint):
    poseSpec = geometryModel.byJoint[ joint ]
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    H  = mxrepr.hCoordinatesNumeric.matrix_repr(xt)
    irx,iry,irz = utils.getIntrinsicXYZFromR( H )
    erx,ery,erz = utils.intrinsic2extrinsic_XYZ(irx, iry, irz)

    return H[0,3], H[1,3], H[2,3], erx, ery, erz


def ordering(orderingModel):
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=orderingModel,
        geometry=None,
        jointParams=None,
        tostr=lambda num: formatter.float2str(num)
    )

def geometry(geometryModel):
    formatter = utils.FloatsFormatter()
    return tpl.render(
        robot=geometryModel.connectivityModel,
        geometry=geometryModel,
        jointParams=jointOrigin,
        tostr=lambda num: formatter.float2str(num)
    )