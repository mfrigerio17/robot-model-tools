import logging
from mako.template import Template

import kgprim.ct as ct
import kgprim.ct.repr.mxrepr as mxrepr

import robmodel.convert.utils as utils

tpl = Template('''
local sim = require('sim')

local function create_model()

local hl, hj, hm, hroot = nil

hroot = sim.createPrimitiveShape(
    sim.primitiveshape_capsule,
    {0.05, 0.05, 0.1},
    0)

hl = hroot

-- set it as model, model base, with all properties enabled
sim.setModelProperty(hroot, 0)

sim.setObjectAlias(hroot, "base")
sim.setObjectInt32Param(hroot, sim.shapeintparam_static, 1)
sim.setObjectInt32Param(hroot, sim.shapeintparam_respondable, 1)

hm = sim.importShape(0,
    "${robot.name}/meshes/base.stl",
    0, 0, 1)
sim.setObjectParent(hm, hroot, false)
sim.setObjectAlias(hm, "base_mesh")


% for joint in robot.joints.values() :

hj = sim.createJoint(
    sim.joint_revolute_subtype,
    sim.jointmode_dynamic,
    0, nil)
sim.setObjectInt32Param(hj, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)
sim.setObjectAlias(hj, "${joint.name}")

-- "connect" the joint with the predecessor link
sim.setObjectParent(hj, hl)
sim.setObjectMatrix(hj, ${jointPoseTable(joint)}, sim.handle_parent)

-- despite the name, this really sets the maximum force, when in position control mode
sim.setJointTargetForce(hj, 30)

-- joints are not normally visible
sim.setObjectInt32Param(hj, sim.objintparam_visibility_layer, 512)

<% link = robot.successor(joint) %>

hl = sim.createPrimitiveShape(
    sim.primitiveshape_capsule,
    {0.05, 0.05, 0.1},
    0)
sim.setObjectAlias(hl, "${link.name}")
sim.setObjectParent(hl, hj);
sim.setObjectMatrix(hl, {1,0,0,0,0,1,0,0,0,0,1,0}, sim.handle_parent)
sim.setObjectInt32Param(hl, sim.shapeintparam_static, 0)      -- i.e. makes it dynamic...
sim.setObjectInt32Param(hl, sim.shapeintparam_respondable, 1)
sim.setObjectFloatParam(hl, sim.shapefloatparam_mass, 0.5)

hm = sim.importShape(0,
    "${robot.name}/meshes/${link.name}.stl",
    0, 0, 1)
sim.setObjectAlias(hm, "${link.name}_mesh")
sim.setObjectParent(hm, hl);
sim.setObjectMatrix(hm, {1,0,0,0,0,1,0,0,0,0,1,0}, sim.handle_parent)
sim.setObjectSpecialProperty(hm,
    sim.objectspecialproperty_collidable |
    sim.objectspecialproperty_measurable |
    sim.objectspecialproperty_detectable)

% endfor

return hroot

end


return {
    create_model = create_model
}
''')


def jointFramePoseCoefficients(geometryModel, joint):
    poseSpec = geometryModel.byJoint[ joint ]
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    H  = mxrepr.hCoordinatesNumeric(xt)

    values = []
    for c in [0,1,2]:
        values.extend( [H[c,0], H[c,1], H[c,2], H[c,3]] )
    return values

def listAsLuaTableCode(values, formatter) :
    text = ",".join([formatter.float2str(v) for v in values])
    return "{" + text + "}"

def code_creating_model(geometryModel):
    formatter = utils.FloatsFormatter()

    return tpl.render(
        robot = geometryModel.connectivityModel,
        jointPoseTable = lambda j : listAsLuaTableCode(
            jointFramePoseCoefficients(geometryModel, j), formatter)
    )

